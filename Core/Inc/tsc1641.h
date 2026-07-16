#ifndef TSC1641_H
#define TSC1641_H

/*
 * TSC1641 power monitor (on the STEVAL-DIGAFEV1 eval board).
 *
 * Wiring here: I2C1, PB6 = SCL, PB9 = SDA. The board has 4.7k pull-ups
 * (JP_SDA/JP_SCL), address jumpers JP_A0/JP_A1 on GND -> 0x40, and a
 * 5 mOhm shunt (R6).
 *
 * Usage:
 *   TSC1641_Init(&hi2c1);   // once, after MX_I2C1_Init()
 *   TSC1641_Update();       // periodically; then read the tsc_* globals
 */

#include <stdint.h>
#include "main.h"

#define TSC1641_ADDR         0x40

#define TSC1641_REG_CONF     0x00
#define TSC1641_REG_VSHUNT   0x01  /* LSB 2.5 uV, signed */
#define TSC1641_REG_VLOAD    0x02  /* LSB 2 mV */
#define TSC1641_REG_POWER    0x03  /* LSB 25 mW */
#define TSC1641_REG_CURRENT  0x04  /* LSB = 2.5 uV / Rshunt, signed */
#define TSC1641_REG_RSHUNT   0x08  /* LSB 10 uOhm */

/* Note: the datasheet register table (DS14338) says 0x03 = power and
 * 0x04 = current, but there are reports of the two being swapped on real
 * silicon. If current and power look exchanged, swap the defines above. */

#define TSC1641_RSHUNT_10UOHM  500   /* 5 mOhm shunt in 10-uOhm units */

/* Measurements, refreshed by TSC1641_Update(). volatile so Live Watch
 * and the main loop always see fresh values. */
extern volatile float   tsc_current_a;   /* load current, A (signed) */
extern volatile float   tsc_vload_v;     /* load voltage, V */
extern volatile float   tsc_power_w;     /* power, W */
extern volatile int16_t tsc_vshunt_raw;  /* raw shunt register, LSB 2.5 uV */
extern volatile uint8_t tsc_online;      /* 1 = last I2C exchange OK */

/* Start continuous conversion and program the shunt value.
 * Returns 1 on success, 0 if the chip doesn't answer. */
uint8_t TSC1641_Init(I2C_HandleTypeDef *hi2c);

/* Read all measurements into the tsc_* globals. */
void TSC1641_Update(void);

#endif /* TSC1641_H */
