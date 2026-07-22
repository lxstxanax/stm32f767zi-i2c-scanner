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


# Mid greys stay readable on a light and on a dark desktop theme alike,
# which the dimmed palette roles do not.
TITLE_COLOR = "#8b949e"
UNIT_COLOR = "#6e7781"
ACCENT_COLOR = "#4dabf7"

CARD_STYLE = """
QFrame {
    border: 1px solid palette(mid);
    border-radius: 6px;
}
"""


class MetricCard(QtWidgets.QFrame):
    """Compact readout: small caption, big number, unit underneath."""

    def __init__(self, title: str, unit: str = "", parent=None):
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setStyleSheet(CARD_STYLE)

        self.title = QtWidgets.QLabel(title)
        self.value = QtWidgets.QLabel("—")
        self.unit = QtWidgets.QLabel(unit)

        self.title.setStyleSheet(f"border: none; font-size: 10px; color: {TITLE_COLOR};")
        self.value.setStyleSheet("border: none; font-size: 18px; font-weight: 650;")
        self.unit.setStyleSheet(f"border: none; font-size: 9px; color: {UNIT_COLOR};")
        self.setMaximumHeight(58)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(0)
        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.unit)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class TextStatusCard(QtWidgets.QFrame):
    """Same size as a MetricCard, but holds a short phrase."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setStyleSheet(CARD_STYLE)

        self.title = QtWidgets.QLabel(title)
        self.value = QtWidgets.QLabel("—")
        self.value.setWordWrap(True)

        self.title.setStyleSheet(f"border: none; font-size: 10px; color: {TITLE_COLOR};")
        self.value.setStyleSheet("border: none; font-size: 13px; font-weight: 600;")
        self.setMaximumHeight(58)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(0)
        layout.addWidget(self.title)
        layout.addWidget(self.value, 1)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class Dashboard(QtWidgets.QMainWindow):
    MAX_POINTS = 900

    # tab name, plot title, y label, y units, [(deque name, legend, colour)]
    PLOTS = [
        ("Current", "Current", "Current", "A", [
            ("tsc_current", "TSC1641", "#2ecc71"),
            ("ina_current", "INA228", "#e67e22"),
        ]),
        ("Tilt", "Board tilt", "Angle", "°", [
            ("roll", "Roll (X)", "#4dabf7"),
            ("pitch", "Pitch (Y)", "#ff922b"),
        ]),
        ("Motion", "Motion", "Value", "", [
            ("accel_total", "Acceleration, g", "#51cf66"),
            ("gyro_total", "Rotation, °/s", "#cc5de8"),
        ]),
        ("Temperature", "Temperature", "Temperature", "°C", [
            ("mpu_temp", "MPU6050", "#ff6b6b"),
            ("mlx_ambient", "MLX90614 ambient", "#22b8cf"),
            ("mlx_object", "MLX90614 object", "#fcc419"),
        ]),
    ]

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
        cards.setSpacing(5)
        cards.setContentsMargins(0, 0, 0, 0)

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
                "font-size: 11px; font-weight: 700; letter-spacing: 1px;"
                f"color: {ACCENT_COLOR}; margin-top: 4px;"
            )
            cards.addWidget(header, row, 0, 1, columns)
            row += 1

            for index, key in enumerate(keys):
                cards.addWidget(self.cards[key], row + index // columns,
                                index % columns)

            row += (len(keys) + columns - 1) // columns

        for column in range(columns):
            cards.setColumnStretch(column, 1)

        # The block of cards keeps its natural height so that everything
        # left over goes to the plots below.
        cards_panel = QtWidgets.QWidget()
        cards_panel.setLayout(cards)
        cards_panel.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                  QtWidgets.QSizePolicy.Maximum)
        root.addWidget(cards_panel)

        note = QtWidgets.QLabel(
            "Tilt is derived from gravity. During strong motion the "
            "accelerometer-based angles are temporarily less accurate."
        )
        note.setWordWrap(True)
        note.setStyleSheet(f"font-size: 11px; color: {UNIT_COLOR};")
        root.addWidget(note)

        self.tabs = QtWidgets.QTabWidget()
        root.addWidget(self.tabs, 1)

        # Every curve is described once. Each entry names the deque that
        # feeds it, so the same description builds both the single-plot
        # tabs and the overview page, and drawing them stays one loop.
        self.curve_sets = []

        for tab_name, title, y_label, y_units, series in self.PLOTS:
            plot, curves = self._make_plot(title, y_label, y_units, series)
            self.tabs.addTab(plot, tab_name)
            self.curve_sets.append(curves)

        self.tabs.addTab(self._make_overview(), "All graphs")

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

    def _make_plot(self, title, y_label, y_units, series):
        """Build one plot. Returns it together with {deque name: curve}."""
        plot = pg.PlotWidget(title=title)
        plot.setLabel("left", y_label, units=y_units or None)
        plot.setLabel("bottom", "Time", units="s")
        plot.showGrid(x=True, y=True, alpha=0.2)
        plot.addLegend(offset=(-10, 10))
        plot.setMenuEnabled(False)

        curves = {}
        for source, legend, colour in series:
            curves[source] = plot.plot(name=legend,
                                       pen=pg.mkPen(colour, width=2))
        return plot, curves

    def _make_overview(self) -> QtWidgets.QWidget:
        """One scrollable page holding a copy of every plot, stacked."""
        page = QtWidgets.QWidget()
        column = QtWidgets.QVBoxLayout(page)
        column.setContentsMargins(0, 0, 0, 0)
        column.setSpacing(6)

        combined = {}
        for _, title, y_label, y_units, series in self.PLOTS:
            plot, curves = self._make_plot(title, y_label, y_units, series)
            plot.setMinimumHeight(200)
            column.addWidget(plot)
            combined.update(curves)

        self.curve_sets.append(combined)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(page)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        return scroll

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
        cache = {}

        # Only the visible page is redrawn; the rest catches up when it is
        # opened, which keeps the overview page from costing anything while
        # a single plot is on screen.
        visible = self.tabs.currentIndex()
        if visible >= len(self.curve_sets):
            return

        for source, curve in self.curve_sets[visible].items():
            if source not in cache:
                cache[source] = self._clean(getattr(self, source))
            curve.setData(x, cache[source])

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

    # Follow the desktop theme instead of pyqtgraph's own default, and
    # smooth the lines - thin traces look ragged without it.
    palette = app.palette()
    pg.setConfigOption("background", palette.window().color())
    pg.setConfigOption("foreground", palette.windowText().color())
    pg.setConfigOptions(antialias=True)

    window = Dashboard()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
