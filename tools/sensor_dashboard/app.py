#!/usr/bin/env python3
from __future__ import annotations

import csv
import glob
import math
import re
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import serial
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg


MPU_RE = re.compile(
    r"(?:MPU(?:6050)?\s+)?A\[mg\]=(?P<ax>-?\d+),(?P<ay>-?\d+),(?P<az>-?\d+)\s+"
    r"G\[mdps\]=(?P<gx>-?\d+),(?P<gy>-?\d+),(?P<gz>-?\d+)\s+"
    r"T\[cC\]=(?P<temp>-?\d+)"
)

MLX_RE = re.compile(
    r"MLX(?:90614)?\s+Ta\[cC\]=(?P<ambient>-?\d+)\s+To\[cC\]=(?P<object>-?\d+)"
)

NUM = r"-?\d+(?:\.\d+)?"

TSC_RE = re.compile(
    rf"TSC(?:1641)?\s+I\[A\]=(?P<current>{NUM})\s+U\[V\]=(?P<voltage>{NUM})\s+"
    rf"P\[W\]=(?P<power>{NUM})\s+Vsh\[mV\]=(?P<shunt>{NUM})"
)

INA_RE = re.compile(
    rf"INA(?:228)?\s+I\[A\]=(?P<current>{NUM})\s+U\[V\]=(?P<voltage>{NUM})\s+"
    rf"P\[W\]=(?P<power>{NUM})\s+T\[C\]=(?P<temp>{NUM})\s+Vsh\[mV\]=(?P<shunt>{NUM})"
)

PSU_RE = re.compile(rf"PSU set\[V\]=(?P<setpoint>{NUM})")

I2C_SCAN_RE = re.compile(r"I2C scan:\s*(?P<count>\d+)\s*device\(s\):(?P<addresses>.*)")
MPU_INIT_RE = re.compile(r"MPU6050 init:\s*(?P<status>.+)")
MLX_INIT_RE = re.compile(r"MLX90614 init:\s*(?P<status>.+)")
TSC_INIT_RE = re.compile(r"TSC1641 init:\s*(?P<status>.+)")
INA_INIT_RE = re.compile(r"INA228 init:\s*(?P<status>.+)")


@dataclass
class SensorState:
    monotonic_s: float = 0.0
    wall_time_s: float = 0.0

    ax_g: Optional[float] = None
    ay_g: Optional[float] = None
    az_g: Optional[float] = None

    gx_dps: Optional[float] = None
    gy_dps: Optional[float] = None
    gz_dps: Optional[float] = None

    mpu_temp_c: Optional[float] = None
    mlx_ambient_c: Optional[float] = None
    mlx_object_c: Optional[float] = None

    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    accel_magnitude_g: Optional[float] = None
    gyro_magnitude_dps: Optional[float] = None

    tsc_current_a: Optional[float] = None
    tsc_voltage_v: Optional[float] = None
    tsc_power_w: Optional[float] = None
    tsc_shunt_mv: Optional[float] = None

    ina_current_a: Optional[float] = None
    ina_voltage_v: Optional[float] = None
    ina_power_w: Optional[float] = None
    ina_temp_c: Optional[float] = None
    ina_shunt_mv: Optional[float] = None

    psu_setpoint_v: Optional[float] = None


class SerialWorker(QtCore.QThread):
    line_received = QtCore.Signal(str)
    connected = QtCore.Signal(str)
    disconnected = QtCore.Signal(str)
    error = QtCore.Signal(str)

    def __init__(self, port: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self._stop_requested = False
        self._serial: Optional[serial.Serial] = None

    def stop(self) -> None:
        self._stop_requested = True
        if self._serial and self._serial.is_open:
            try:
                self._serial.cancel_read()
            except Exception:
                pass

    def send_line(self, text: str) -> bool:
        """Send one command to the board. Returns False if not connected."""
        if not (self._serial and self._serial.is_open):
            return False
        try:
            self._serial.write((text + "\n").encode("ascii"))
            return True
        except Exception as exc:
            self.error.emit(str(exc))
            return False

    def run(self) -> None:
        try:
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.2,
                write_timeout=0.5,
            )
            self.connected.emit(self.port)

            while not self._stop_requested:
                raw = self._serial.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.line_received.emit(line)

        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if self._serial:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self.disconnected.emit(self.port)


class MetricCard(QtWidgets.QFrame):
    def __init__(self, title: str, unit: str = "", parent=None):
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)

        self.title = QtWidgets.QLabel(title)
        self.value = QtWidgets.QLabel("—")
        self.unit = QtWidgets.QLabel(unit)

        self.value.setStyleSheet("font-size: 29px; font-weight: 650;")
        self.title.setStyleSheet("font-size: 13px;")
        self.unit.setStyleSheet("font-size: 12px;")

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.unit)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class TextStatusCard(QtWidgets.QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)

        self.title = QtWidgets.QLabel(title)
        self.value = QtWidgets.QLabel("—")
        self.value.setWordWrap(True)
        self.value.setStyleSheet("font-size: 20px; font-weight: 600;")

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.title)
        layout.addWidget(self.value)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class Dashboard(QtWidgets.QMainWindow):
    MAX_POINTS = 900

    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 Sensor Dashboard")
        self.resize(1260, 880)

        self.worker: Optional[SerialWorker] = None
        self.start_monotonic = time.monotonic()
        self.state = SensorState()

        self.csv_file = None
        self.csv_writer = None

        self.t = deque(maxlen=self.MAX_POINTS)
        self.roll = deque(maxlen=self.MAX_POINTS)
        self.pitch = deque(maxlen=self.MAX_POINTS)
        self.accel_total = deque(maxlen=self.MAX_POINTS)
        self.gyro_total = deque(maxlen=self.MAX_POINTS)
        self.mpu_temp = deque(maxlen=self.MAX_POINTS)
        self.mlx_ambient = deque(maxlen=self.MAX_POINTS)
        self.mlx_object = deque(maxlen=self.MAX_POINTS)
        self.tsc_current = deque(maxlen=self.MAX_POINTS)
        self.ina_current = deque(maxlen=self.MAX_POINTS)

        self._build_ui()
        self.refresh_ports()

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(50)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        controls = QtWidgets.QHBoxLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_button = QtWidgets.QPushButton("Refresh ports")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.record_button = QtWidgets.QPushButton("Record CSV")
        self.record_button.setCheckable(True)

        controls.addWidget(QtWidgets.QLabel("STM32 port:"))
        controls.addWidget(self.port_combo, 1)
        controls.addWidget(self.refresh_button)
        controls.addWidget(self.connect_button)
        controls.addWidget(self.record_button)
        root.addLayout(controls)

        status_layout = QtWidgets.QHBoxLayout()
        self.connection_status = QtWidgets.QLabel("STM32: disconnected")
        self.i2c_status = QtWidgets.QLabel("I²C: —")
        self.mpu_status = QtWidgets.QLabel("MPU6050: —")
        self.mlx_status = QtWidgets.QLabel("MLX90614: —")
        self.tsc_status = QtWidgets.QLabel("TSC1641: —")
        self.ina_status = QtWidgets.QLabel("INA228: —")

        for label in (
            self.connection_status,
            self.i2c_status,
            self.mpu_status,
            self.mlx_status,
            self.tsc_status,
            self.ina_status,
        ):
            label.setStyleSheet("font-weight: 600;")
            status_layout.addWidget(label)

        status_layout.addStretch()
        root.addLayout(status_layout)

        # Control of the DAC output on PA5, the voltage that tells the
        # power supply what to produce. Slider and number box show the same
        # value; the board only ever changes it when told to.
        control = QtWidgets.QHBoxLayout()
        control.addWidget(QtWidgets.QLabel("PSU setpoint (DAC on PA5):"))

        self.setpoint_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.setpoint_slider.setRange(0, 3300)      # millivolts
        self.setpoint_slider.setSingleStep(10)
        self.setpoint_slider.setPageStep(100)

        self.setpoint_spin = QtWidgets.QDoubleSpinBox()
        self.setpoint_spin.setRange(0.0, 3.3)
        self.setpoint_spin.setDecimals(3)
        self.setpoint_spin.setSingleStep(0.05)
        self.setpoint_spin.setSuffix(" V")

        self.setpoint_apply = QtWidgets.QPushButton("Apply")
        self.setpoint_zero = QtWidgets.QPushButton("Zero")

        control.addWidget(self.setpoint_slider, 1)
        control.addWidget(self.setpoint_spin)
        control.addWidget(self.setpoint_apply)
        control.addWidget(self.setpoint_zero)
        root.addLayout(control)

        cards = QtWidgets.QGridLayout()

        self.cards = {
            "tsc_current": MetricCard("Current", "TSC1641, A"),
            "ina_current": MetricCard("Current", "INA228, A"),
            "tsc_shunt": MetricCard("Shunt drop", "TSC1641, mV"),
            "ina_shunt": MetricCard("Shunt drop", "INA228, mV"),
            "tsc_voltage": MetricCard("Load voltage", "TSC1641, V"),
            "ina_voltage": MetricCard("Bus voltage", "INA228, V"),
            "psu_setpoint": MetricCard("PSU setpoint", "DAC output, V"),
            "ina_power": MetricCard("Power", "INA228, W"),
            "object_temp": MetricCard("Object temperature", "MLX90614, °C"),
            "sensor_temp": MetricCard("IR sensor temperature", "MLX90614, °C"),
            "mpu_temp": MetricCard("IMU temperature", "MPU6050, °C"),
            "thermal": TextStatusCard("Object vs sensor"),
            "roll": MetricCard("Roll (around X)", "degrees"),
            "pitch": MetricCard("Pitch (around Y)", "degrees"),
            "accel": MetricCard("Total acceleration", "g"),
            "gyro": MetricCard("Rotation rate", "°/s"),
            "motion": TextStatusCard("Motion state"),
        }

        # Sorted by physical quantity rather than by chip, so the two
        # current sensors sit next to each other and can be compared at a
        # glance. Rows are filled automatically, which makes it impossible
        # for two cards to end up in the same cell.
        sections = [
            ("Electrical", [
                "tsc_current", "ina_current", "tsc_shunt", "ina_shunt",
                "tsc_voltage", "ina_voltage", "psu_setpoint", "ina_power",
            ]),
            ("Temperature", [
                "object_temp", "sensor_temp", "mpu_temp", "thermal",
            ]),
            ("Motion", [
                "roll", "pitch", "accel", "gyro", "motion",
            ]),
        ]

        columns = 4
        row = 0

        for title, keys in sections:
            header = QtWidgets.QLabel(title)
            header.setStyleSheet(
                "font-size: 12px; font-weight: 700; letter-spacing: 1px;"
                "color: palette(mid); margin-top: 6px;"
            )
            cards.addWidget(header, row, 0, 1, columns)
            row += 1

            for index, key in enumerate(keys):
                cards.addWidget(self.cards[key], row + index // columns,
                                index % columns)

            row += (len(keys) + columns - 1) // columns

        for column in range(columns):
            cards.setColumnStretch(column, 1)

        root.addLayout(cards)

        note = QtWidgets.QLabel(
            "Tilt is derived from gravity. During strong motion the "
            "accelerometer-based angles are temporarily less accurate."
        )
        note.setWordWrap(True)
        root.addWidget(note)

        self.tabs = QtWidgets.QTabWidget()
        root.addWidget(self.tabs, 1)

        self.orientation_plot = pg.PlotWidget(title="Board tilt")
        self.orientation_plot.setLabel("left", "Angle", units="°")
        self.orientation_plot.setLabel("bottom", "Time", units="s")
        self.orientation_plot.showGrid(x=True, y=True, alpha=0.25)
        self.orientation_plot.addLegend()
        self.roll_curve = self.orientation_plot.plot(
            name="Roll (X)", pen=pg.mkPen("#4dabf7", width=2))
        self.pitch_curve = self.orientation_plot.plot(
            name="Pitch (Y)", pen=pg.mkPen("#ff922b", width=2))
        self.tabs.addTab(self.orientation_plot, "Tilt")

        self.motion_plot = pg.PlotWidget(title="Motion")
        self.motion_plot.setLabel("left", "Value")
        self.motion_plot.setLabel("bottom", "Time", units="s")
        self.motion_plot.showGrid(x=True, y=True, alpha=0.25)
        self.motion_plot.addLegend()
        self.accel_curve = self.motion_plot.plot(
            name="Acceleration, g", pen=pg.mkPen("#51cf66", width=2))
        self.gyro_curve = self.motion_plot.plot(
            name="Rotation, °/s", pen=pg.mkPen("#cc5de8", width=2))
        self.tabs.addTab(self.motion_plot, "Motion")

        self.temp_plot = pg.PlotWidget(title="Temperature")
        self.temp_plot.setLabel("left", "Temperature", units="°C")
        self.temp_plot.setLabel("bottom", "Time", units="s")
        self.temp_plot.showGrid(x=True, y=True, alpha=0.25)
        self.temp_plot.addLegend()
        self.mpu_temp_curve = self.temp_plot.plot(
            name="MPU6050", pen=pg.mkPen("#ff6b6b", width=2))
        self.mlx_ambient_curve = self.temp_plot.plot(
            name="MLX90614 ambient", pen=pg.mkPen("#22b8cf", width=2))
        self.mlx_object_curve = self.temp_plot.plot(
            name="MLX90614 object", pen=pg.mkPen("#fcc419", width=2))
        self.tabs.addTab(self.temp_plot, "Temperature")

        self.current_plot = pg.PlotWidget(title="Current")
        self.current_plot.setLabel("left", "Current", units="A")
        self.current_plot.setLabel("bottom", "Time", units="s")
        self.current_plot.showGrid(x=True, y=True, alpha=0.25)
        self.current_plot.addLegend()
        self.tsc_current_curve = self.current_plot.plot(
            name="TSC1641", pen=pg.mkPen("#2ecc71", width=2))
        self.ina_current_curve = self.current_plot.plot(
            name="INA228", pen=pg.mkPen("#e67e22", width=2))
        self.tabs.addTab(self.current_plot, "Current")

        self.diagnostics = QtWidgets.QPlainTextEdit()
        self.diagnostics.setReadOnly(True)
        self.diagnostics.document().setMaximumBlockCount(2000)
        self.tabs.addTab(self.diagnostics, "Diagnostics")

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.record_button.toggled.connect(self.toggle_recording)

        self.setpoint_slider.valueChanged.connect(
            lambda mv: self.setpoint_spin.setValue(mv / 1000.0))
        self.setpoint_spin.valueChanged.connect(
            lambda v: self.setpoint_slider.setValue(round(v * 1000)))
        self.setpoint_apply.clicked.connect(self.apply_setpoint)
        self.setpoint_zero.clicked.connect(self.zero_setpoint)

    def refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        ports = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current in ports:
            self.port_combo.setCurrentText(current)

    def toggle_connection(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait(1500)
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.warning(
                self,
                "No port found",
                "No /dev/ttyACM* or /dev/ttyUSB* device is available.",
            )
            return

        self.worker = SerialWorker(port)
        self.worker.line_received.connect(self.handle_line)
        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)
        self.worker.error.connect(self.on_error)
        self.worker.start()

    def on_connected(self, port: str) -> None:
        self.connection_status.setText(f"STM32: {port}")
        self.connect_button.setText("Disconnect")
        self.diagnostics.appendPlainText(f"[DASHBOARD] Connected to {port}")

    def on_disconnected(self, port: str) -> None:
        self.connection_status.setText("STM32: disconnected")
        self.connect_button.setText("Connect")
        self.diagnostics.appendPlainText(f"[DASHBOARD] Disconnected from {port}")

    def on_error(self, message: str) -> None:
        self.diagnostics.appendPlainText(f"[ERROR] {message}")
        QtWidgets.QMessageBox.critical(self, "Port error", message)

    @staticmethod
    def _is_ok(text: str) -> bool:
        lowered = text.lower()
        return (
            lowered.startswith("ok")
            or "0x68" in lowered
            or "0x5a" in lowered
        )

    def send_command(self, text: str) -> None:
        if not (self.worker and self.worker.isRunning()
                and self.worker.send_line(text)):
            self.diagnostics.appendPlainText(
                f"[DASHBOARD] Not connected, command dropped: {text}")
            return
        self.diagnostics.appendPlainText(f"[DASHBOARD] Sent: {text}")

    def apply_setpoint(self) -> None:
        self.send_command(f"set {self.setpoint_spin.value():.3f}")

    def zero_setpoint(self) -> None:
        self.setpoint_spin.setValue(0.0)
        self.send_command("set 0")

    def handle_line(self, line: str) -> None:
        self.diagnostics.appendPlainText(line)

        scan = I2C_SCAN_RE.search(line)
        if scan:
            addresses = scan.group("addresses").strip()
            self.i2c_status.setText(f"I²C: {addresses or 'no devices'}")
            return

        mpu_init = MPU_INIT_RE.search(line)
        if mpu_init:
            status = mpu_init.group("status").strip()
            self.mpu_status.setText(f"MPU6050: {status}")
            return

        mlx_init = MLX_INIT_RE.search(line)
        if mlx_init:
            status = mlx_init.group("status").strip()
            self.mlx_status.setText(f"MLX90614: {status}")
            return

        tsc_init = TSC_INIT_RE.search(line)
        if tsc_init:
            status = tsc_init.group("status").strip()
            self.tsc_status.setText(f"TSC1641: {status}")
            return

        ina_init = INA_INIT_RE.search(line)
        if ina_init:
            status = ina_init.group("status").strip()
            self.ina_status.setText(f"INA228: {status}")
            return

        mpu = MPU_RE.search(line)
        if mpu:
            self.state.monotonic_s = time.monotonic()
            self.state.wall_time_s = time.time()

            self.state.ax_g = int(mpu.group("ax")) / 1000.0
            self.state.ay_g = int(mpu.group("ay")) / 1000.0
            self.state.az_g = int(mpu.group("az")) / 1000.0

            self.state.gx_dps = int(mpu.group("gx")) / 1000.0
            self.state.gy_dps = int(mpu.group("gy")) / 1000.0
            self.state.gz_dps = int(mpu.group("gz")) / 1000.0

            self.state.mpu_temp_c = int(mpu.group("temp")) / 100.0
            self._calculate_motion_values()
            self._append_state()
            self._update_cards()
            self._write_csv("MPU6050")
            return

        mlx = MLX_RE.search(line)
        if mlx:
            self.state.monotonic_s = time.monotonic()
            self.state.wall_time_s = time.time()

            self.state.mlx_ambient_c = int(mlx.group("ambient")) / 100.0
            self.state.mlx_object_c = int(mlx.group("object")) / 100.0

            self._append_state()
            self._update_cards()
            self._write_csv("MLX90614")
            return

        tsc = TSC_RE.search(line)
        if tsc:
            self.state.monotonic_s = time.monotonic()
            self.state.wall_time_s = time.time()

            self.state.tsc_current_a = float(tsc.group("current"))
            self.state.tsc_voltage_v = float(tsc.group("voltage"))
            self.state.tsc_power_w = float(tsc.group("power"))
            self.state.tsc_shunt_mv = float(tsc.group("shunt"))

            self._append_state()
            self._update_cards()
            self._write_csv("TSC1641")
            return

        ina = INA_RE.search(line)
        if ina:
            self.state.monotonic_s = time.monotonic()
            self.state.wall_time_s = time.time()

            self.state.ina_current_a = float(ina.group("current"))
            self.state.ina_voltage_v = float(ina.group("voltage"))
            self.state.ina_power_w = float(ina.group("power"))
            self.state.ina_temp_c = float(ina.group("temp"))
            self.state.ina_shunt_mv = float(ina.group("shunt"))

            self._append_state()
            self._update_cards()
            self._write_csv("INA228")
            return

        psu = PSU_RE.search(line)
        if psu:
            self.state.psu_setpoint_v = float(psu.group("setpoint"))
            self._update_cards()

    def _calculate_motion_values(self) -> None:
        s = self.state

        if None not in (s.ax_g, s.ay_g, s.az_g):
            ax = float(s.ax_g)
            ay = float(s.ay_g)
            az = float(s.az_g)

            s.accel_magnitude_g = math.sqrt(ax * ax + ay * ay + az * az)

            # Static tilt estimate from gravity vector.
            s.roll_deg = math.degrees(math.atan2(ay, az))
            s.pitch_deg = math.degrees(
                math.atan2(-ax, math.sqrt(ay * ay + az * az))
            )

        if None not in (s.gx_dps, s.gy_dps, s.gz_dps):
            gx = float(s.gx_dps)
            gy = float(s.gy_dps)
            gz = float(s.gz_dps)
            s.gyro_magnitude_dps = math.sqrt(gx * gx + gy * gy + gz * gz)

    def _append_state(self) -> None:
        elapsed = self.state.monotonic_s - self.start_monotonic
        self.t.append(elapsed)
        self.roll.append(self.state.roll_deg)
        self.pitch.append(self.state.pitch_deg)
        self.accel_total.append(self.state.accel_magnitude_g)
        self.gyro_total.append(self.state.gyro_magnitude_dps)
        self.mpu_temp.append(self.state.mpu_temp_c)
        self.mlx_ambient.append(self.state.mlx_ambient_c)
        self.mlx_object.append(self.state.mlx_object_c)
        self.tsc_current.append(self.state.tsc_current_a)
        self.ina_current.append(self.state.ina_current_a)

    @staticmethod
    def _fmt(value: Optional[float], digits: int = 2) -> str:
        return "—" if value is None else f"{value:.{digits}f}"

    def _motion_text(self) -> str:
        s = self.state

        if s.gyro_magnitude_dps is None or s.accel_magnitude_g is None:
            return "No data"

        rotation = s.gyro_magnitude_dps
        accel_error = abs(s.accel_magnitude_g - 1.0)

        if rotation < 5.0 and accel_error < 0.10:
            return "Nearly still"
        if rotation < 30.0 and accel_error < 0.35:
            return "Moving"
        return "Strong motion"

    def _thermal_text(self) -> str:
        s = self.state

        if s.mlx_object_c is None or s.mlx_ambient_c is None:
            return "No data"

        delta = s.mlx_object_c - s.mlx_ambient_c

        if abs(delta) < 0.5:
            return f"About the same ({delta:+.2f} °C)"
        if delta > 0.0:
            return f"{delta:.2f} °C warmer than sensor"
        return f"{abs(delta):.2f} °C cooler than sensor"

    def _update_cards(self) -> None:
        s = self.state

        self.cards["object_temp"].set_value(self._fmt(s.mlx_object_c))
        self.cards["sensor_temp"].set_value(self._fmt(s.mlx_ambient_c))
        self.cards["mpu_temp"].set_value(self._fmt(s.mpu_temp_c))
        self.cards["roll"].set_value(self._fmt(s.roll_deg, 1))
        self.cards["pitch"].set_value(self._fmt(s.pitch_deg, 1))
        self.cards["accel"].set_value(self._fmt(s.accel_magnitude_g, 3))
        self.cards["gyro"].set_value(self._fmt(s.gyro_magnitude_dps, 2))
        self.cards["motion"].set_value(self._motion_text())
        self.cards["thermal"].set_value(self._thermal_text())
        self.cards["tsc_current"].set_value(self._fmt(s.tsc_current_a, 4))
        self.cards["tsc_voltage"].set_value(self._fmt(s.tsc_voltage_v, 3))
        self.cards["ina_current"].set_value(self._fmt(s.ina_current_a, 4))
        self.cards["ina_voltage"].set_value(self._fmt(s.ina_voltage_v, 3))
        self.cards["ina_power"].set_value(self._fmt(s.ina_power_w, 3))
        self.cards["psu_setpoint"].set_value(self._fmt(s.psu_setpoint_v, 3))
        self.cards["tsc_shunt"].set_value(self._fmt(s.tsc_shunt_mv, 3))
        self.cards["ina_shunt"].set_value(self._fmt(s.ina_shunt_mv, 3))

    @staticmethod
    def _clean(values):
        return [float("nan") if value is None else value for value in values]

    def update_plots(self) -> None:
        if not self.t:
            return

        x = list(self.t)
        self.roll_curve.setData(x, self._clean(self.roll))
        self.pitch_curve.setData(x, self._clean(self.pitch))
        self.accel_curve.setData(x, self._clean(self.accel_total))
        self.gyro_curve.setData(x, self._clean(self.gyro_total))
        self.mpu_temp_curve.setData(x, self._clean(self.mpu_temp))
        self.mlx_ambient_curve.setData(x, self._clean(self.mlx_ambient))
        self.mlx_object_curve.setData(x, self._clean(self.mlx_object))
        self.tsc_current_curve.setData(x, self._clean(self.tsc_current))
        self.ina_current_curve.setData(x, self._clean(self.ina_current))

    def toggle_recording(self, enabled: bool) -> None:
        if enabled:
            logs = Path.cwd() / "logs"
            logs.mkdir(exist_ok=True)
            filename = logs / time.strftime("sensors_%Y%m%d_%H%M%S.csv")

            self.csv_file = filename.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "timestamp",
                "source",
                "roll_deg",
                "pitch_deg",
                "acceleration_g",
                "rotation_speed_dps",
                "mpu_temperature_c",
                "mlx_sensor_temperature_c",
                "mlx_object_temperature_c",
                "object_minus_sensor_c",
                "motion_state",
                "tsc_current_a",
                "tsc_voltage_v",
                "tsc_power_w",
                "tsc_shunt_mv",
                "ina_current_a",
                "ina_voltage_v",
                "ina_power_w",
                "ina_temperature_c",
                "ina_shunt_mv",
                "psu_setpoint_v",
            ])

            self.record_button.setText("Stop recording")
            self.diagnostics.appendPlainText(
                f"[DASHBOARD] Recording to {filename}"
            )
        else:
            if self.csv_file:
                self.csv_file.close()

            self.csv_file = None
            self.csv_writer = None
            self.record_button.setText("Record CSV")

    def _write_csv(self, source: str) -> None:
        if not self.csv_writer:
            return

        s = self.state

        delta = None
        if s.mlx_object_c is not None and s.mlx_ambient_c is not None:
            delta = s.mlx_object_c - s.mlx_ambient_c

        self.csv_writer.writerow([
            time.strftime(
                "%Y-%m-%d %H:%M:%S",
                time.localtime(s.wall_time_s),
            ),
            source,
            s.roll_deg,
            s.pitch_deg,
            s.accel_magnitude_g,
            s.gyro_magnitude_dps,
            s.mpu_temp_c,
            s.mlx_ambient_c,
            s.mlx_object_c,
            delta,
            self._motion_text(),
            s.tsc_current_a,
            s.tsc_voltage_v,
            s.tsc_power_w,
            s.tsc_shunt_mv,
            s.ina_current_a,
            s.ina_voltage_v,
            s.ina_power_w,
            s.ina_temp_c,
            s.ina_shunt_mv,
            s.psu_setpoint_v,
        ])

        self.csv_file.flush()

    def closeEvent(self, event) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait(1500)

        if self.csv_file:
            self.csv_file.close()

        super().closeEvent(event)


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("STM32 Sensor Dashboard")
    window = Dashboard()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
