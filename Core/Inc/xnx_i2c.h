#ifndef XNX_I2C_H
#define XNX_I2C_H

/*
 * xnx_i2c.h — personal I2C utility library
 *
 * SETUP (in your main.c or wherever you use this):
 *
 *   #define XNX_I2C_BUS  (&hi2c1)   // point to any I2CX handle
 *   #include "xnx_i2c.h"
 *
 * The HAL header must be included before this file (main.h already does it).
 * Works on any STM32 — just change XNX_I2C_BUS.
 */

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Result type for I2C scan
 * -------------------------------------------------------------------------
 * Addresses 0x00–0x07 and 0x78–0x7F are reserved by the I2C spec.
 * Usable range is 0x08–0x77 → 112 addresses maximum.
 * -------------------------------------------------------------------------*/
typedef struct {
    uint8_t addr[112];
    uint8_t count;
} xnx_i2c_scan_t;

/* -------------------------------------------------------------------------
 * xnx_i2c_scan_ex — scan with explicit bus handle
 *
 *   hi2c   : pointer to any I2C_HandleTypeDef (hi2c1, hi2c2, ...)
 *   result : pointer to xnx_i2c_scan_t to fill
 *   returns: number of devices found
 * -------------------------------------------------------------------------*/
static inline uint8_t xnx_i2c_scan_ex(I2C_HandleTypeDef *hi2c,
                                       xnx_i2c_scan_t    *result)
{
    result->count = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        /* HAL expects 8-bit address (7-bit << 1) */
        HAL_StatusTypeDef s = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 10);
        if (s == HAL_OK) {
            result->addr[result->count++] = addr;
        }
    }

    return result->count;
}

/* -------------------------------------------------------------------------
 * Register access
 *
 * addr is the plain 7-bit device address (as reported by xnx_i2c_scan),
 * shifting for HAL is done internally. All functions return HAL_OK on
 * success, or HAL_ERROR/HAL_BUSY/HAL_TIMEOUT on failure.
 *
 * 16-bit variants send/receive the HIGH byte first (big-endian on the
 * wire) — the convention used by TSC1641, INA226 and most power monitors.
 * -------------------------------------------------------------------------*/

#ifndef XNX_I2C_TIMEOUT
#define XNX_I2C_TIMEOUT 100  /* ms per transaction, override before include */
#endif

static inline HAL_StatusTypeDef xnx_i2c_write_reg16_ex(I2C_HandleTypeDef *hi2c,
                                                       uint8_t addr,
                                                       uint8_t reg,
                                                       uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)value };
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2, XNX_I2C_TIMEOUT);
}

static inline HAL_StatusTypeDef xnx_i2c_read_reg16_ex(I2C_HandleTypeDef *hi2c,
                                                      uint8_t addr,
                                                      uint8_t reg,
                                                      uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1), reg,
                                           I2C_MEMADD_SIZE_8BIT, buf, 2,
                                           XNX_I2C_TIMEOUT);
    if (s == HAL_OK) {
        *value = (uint16_t)((uint16_t)buf[0] << 8 | buf[1]);
    }
    return s;
}

static inline HAL_StatusTypeDef xnx_i2c_write_reg8_ex(I2C_HandleTypeDef *hi2c,
                                                      uint8_t addr,
                                                      uint8_t reg,
                                                      uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1, XNX_I2C_TIMEOUT);
}

static inline HAL_StatusTypeDef xnx_i2c_read_reg8_ex(I2C_HandleTypeDef *hi2c,
                                                     uint8_t addr,
                                                     uint8_t reg,
                                                     uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1, XNX_I2C_TIMEOUT);
}

/* -------------------------------------------------------------------------
 * xnx_i2c_scan — scan using the configured default bus (XNX_I2C_BUS)
 *
 * Implemented as a macro so XNX_I2C_BUS expands at the call site,
 * not at include time — avoids forward-declaration issues with CubeMX
 * globals that are declared after the includes block.
 *
 * Requires: #define XNX_I2C_BUS (&hi2cX) before including this header
 * -------------------------------------------------------------------------*/
#ifdef XNX_I2C_BUS
#define xnx_i2c_scan(result) xnx_i2c_scan_ex(XNX_I2C_BUS, (result))

/* Register access on the default bus.
 * Usage:
 *   xnx_i2c_write_reg16(0x40, 0x00, 0x1234);        // write 0x1234 to reg 0
 *   uint16_t v;
 *   if (xnx_i2c_read_reg16(0x40, 0x02, &v) == HAL_OK) { ... }
 */
#define xnx_i2c_write_reg16(addr, reg, value) xnx_i2c_write_reg16_ex(XNX_I2C_BUS, (addr), (reg), (value))
#define xnx_i2c_read_reg16(addr, reg, pvalue) xnx_i2c_read_reg16_ex(XNX_I2C_BUS, (addr), (reg), (pvalue))
#define xnx_i2c_write_reg8(addr, reg, value)  xnx_i2c_write_reg8_ex(XNX_I2C_BUS, (addr), (reg), (value))
#define xnx_i2c_read_reg8(addr, reg, pvalue)  xnx_i2c_read_reg8_ex(XNX_I2C_BUS, (addr), (reg), (pvalue))
#endif /* XNX_I2C_BUS */

#endif /* XNX_I2C_H */
