#ifndef XNX_I2C_H
#define XNX_I2C_H

/*
 * Small I2C helpers: bus scan + 8/16-bit register access.
 *
 * Device addresses are plain 7-bit everywhere (same numbers the scan
 * prints); the <<1 shift that HAL wants happens inside these functions.
 * 16-bit registers go high byte first on the wire, which is what the
 * TSC1641, INA226 and most other power monitors use.
 */

#include <stdint.h>
#include "main.h"

/* 0x00-0x07 and 0x78-0x7F are reserved, so 0x08..0x77 = 112 addresses max */
typedef struct {
    uint8_t addr[112];
    uint8_t count;
} xnx_i2c_scan_t;

/* Probe every address on the bus, fill result, return how many answered. */
uint8_t xnx_i2c_scan(I2C_HandleTypeDef *hi2c, xnx_i2c_scan_t *result);

HAL_StatusTypeDef xnx_i2c_write_reg8 (I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t value);
HAL_StatusTypeDef xnx_i2c_read_reg8  (I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef xnx_i2c_write_reg16(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint16_t value);
HAL_StatusTypeDef xnx_i2c_read_reg16 (I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint16_t *value);

#endif /* XNX_I2C_H */
