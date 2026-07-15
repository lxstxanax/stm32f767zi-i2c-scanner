/*
 * tsc1641.c — TSC1641 power monitor driver (see tsc1641.h for usage)
 */

#include "tsc1641.h"
#include "xnx_i2c.h"

volatile int32_t tsc_current_ua = 0;
volatile uint32_t tsc_vload_mv = 0;
volatile uint32_t tsc_power_mw = 0;
volatile int16_t tsc_vshunt_raw = 0;
volatile uint8_t tsc_online = 0;

static I2C_HandleTypeDef *tsc_bus = NULL;

uint8_t TSC1641_Init(I2C_HandleTypeDef *hi2c)
{
  tsc_bus = hi2c;

  /* 0x0037 = continuous shunt+load conversion, 1024 µs (power-up default) */
  if (xnx_i2c_write_reg16_ex(tsc_bus, TSC1641_ADDR, TSC1641_REG_CONF,
                             0x0037) != HAL_OK)
  {
    return 0;
  }
  if (xnx_i2c_write_reg16_ex(tsc_bus, TSC1641_ADDR, TSC1641_REG_RSHUNT,
                             TSC1641_RSHUNT_VAL) != HAL_OK)
  {
    return 0;
  }
  return 1;
}

void TSC1641_Update(void)
{
  uint16_t raw;
  uint8_t ok = 1;

  if (tsc_bus == NULL)
  {
    tsc_online = 0;
    return;
  }

  if (xnx_i2c_read_reg16_ex(tsc_bus, TSC1641_ADDR, TSC1641_REG_CURRENT,
                            &raw) == HAL_OK)
  {
    tsc_current_ua = (int32_t)(int16_t)raw * TSC1641_CURRENT_LSB_UA;
  }
  else
  {
    ok = 0;
  }

  if (xnx_i2c_read_reg16_ex(tsc_bus, TSC1641_ADDR, TSC1641_REG_VLOAD,
                            &raw) == HAL_OK)
  {
    tsc_vload_mv = (uint32_t)raw * 2U;
  }
  else
  {
    ok = 0;
  }

  if (xnx_i2c_read_reg16_ex(tsc_bus, TSC1641_ADDR, TSC1641_REG_POWER,
                            &raw) == HAL_OK)
  {
    tsc_power_mw = (uint32_t)raw * 25U;
  }
  else
  {
    ok = 0;
  }

  if (xnx_i2c_read_reg16_ex(tsc_bus, TSC1641_ADDR, TSC1641_REG_VSHUNT,
                            &raw) == HAL_OK)
  {
    tsc_vshunt_raw = (int16_t)raw;
  }
  else
  {
    ok = 0;
  }

  tsc_online = ok;
}
