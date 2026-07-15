#ifndef TSC1641_H
#define TSC1641_H

/*
 * tsc1641.h — TSC1641 power monitor (STEVAL-DIGAFEV1 eval board)
 *
 * Wiring (this project): I2C1, PB6 = SCL, PB9 = SDA.
 * Board: 4.7k pull-ups on SDA/SCL via JP_SDA/JP_SCL jumpers,
 *        address jumpers JP_A0/JP_A1 both on GND -> 0x40,
 *        onboard shunt R6 = 5 mOhm.
 *
 * USAGE:
 *   #include "tsc1641.h"
 *   TSC1641_Init(&hi2c1);        // once, after MX_I2C1_Init()
 *   TSC1641_Update();            // periodically (e.g. main loop)
 *   // then read the tsc_* globals from anywhere (main, UART, Live Watch)
 */

#include <stdint.h>
#include "main.h"   /* HAL types (I2C_HandleTypeDef) */

/* ---- Device ---------------------------------------------------------- */
#define TSC1641_ADDR          0x40  /* 7-bit, JP_A0/JP_A1 on GND */

#define TSC1641_REG_CONF      0x00
#define TSC1641_REG_VSHUNT    0x01  /* LSB 2.5 µV, signed */
#define TSC1641_REG_VLOAD     0x02  /* LSB 2 mV */
#define TSC1641_REG_POWER     0x03  /* LSB 25 mW (datasheet map; see note) */
#define TSC1641_REG_CURRENT   0x04  /* LSB = 2.5 µV / Rshunt, signed */
#define TSC1641_REG_FLAGS     0x07
#define TSC1641_REG_RSHUNT    0x08  /* LSB 10 µOhm */
#define TSC1641_REG_DIE_ID    0xFF  /* reads 0x1000 */

#define TSC1641_RSHUNT_VAL     500U /* 5 mOhm / 10 µOhm */
#define TSC1641_CURRENT_LSB_UA 500  /* µA per bit with 5 mOhm shunt */

/* NOTE: register map tables (DS14338 Table 12, UM3202 Table 4) say
 * 0x03 = power, 0x04 = current, but the datasheet section headers and an
 * ST community report suggest they may be swapped on real silicon.
 * If tsc_current_ua and tsc_power_mw look exchanged, swap the two
 * register defines above. */

/* ---- Live-watch globals (updated by TSC1641_Update) ------------------ */
extern volatile int32_t tsc_current_ua;   /* load current, µA (signed) */
extern volatile int32_t tsc_vload_mv;     /* load voltage, mV (signed: a floating
                                             VLOAD input can read slightly < 0) */
extern volatile uint32_t tsc_power_mw;    /* DC power, mW */
extern volatile int16_t tsc_vshunt_raw;   /* raw shunt voltage reg, LSB 2.5 µV */
extern volatile uint8_t tsc_online;       /* 1 = last I2C exchange OK */

/* ---- API -------------------------------------------------------------- */

/**
  * @brief  Configure the TSC1641: continuous shunt+load conversion (1024 µs)
  *         and program the shunt value so the chip computes current itself.
  * @param  hi2c: I2C bus the sensor is on (stored for later updates)
  * @retval 1 = OK, 0 = chip did not answer on I2C
  */
uint8_t TSC1641_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief  Read current, load voltage and power into the tsc_* globals.
  *         Call periodically from the main loop.
  */
void TSC1641_Update(void);

#endif /* TSC1641_H */
