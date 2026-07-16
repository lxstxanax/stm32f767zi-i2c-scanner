#include "xnx_i2c.h"

#define TIMEOUT_MS 100

uint8_t xnx_i2c_scan(I2C_HandleTypeDef *hi2c, xnx_i2c_scan_t *result)
{
    result->count = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, addr << 1, 2, 10) == HAL_OK) {
            result->addr[result->count++] = addr;
        }
    }
    return result->count;
}

HAL_StatusTypeDef xnx_i2c_write_reg8(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, TIMEOUT_MS);
}

HAL_StatusTypeDef xnx_i2c_read_reg8(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, TIMEOUT_MS);
}

HAL_StatusTypeDef xnx_i2c_write_reg16(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { value >> 8, value & 0xFF };
    return HAL_I2C_Mem_Write(hi2c, addr << 1, reg, I2C_MEMADD_SIZE_8BIT,
                             buf, 2, TIMEOUT_MS);
}

HAL_StatusTypeDef xnx_i2c_read_reg16(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(hi2c, addr << 1, reg,
                                           I2C_MEMADD_SIZE_8BIT, buf, 2, TIMEOUT_MS);
    if (s == HAL_OK) {
        *value = (buf[0] << 8) | buf[1];
    }
    return s;
}
