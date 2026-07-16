#include "tsc1641.h"
#include "xnx_i2c.h"

volatile float   tsc_current_a = 0;
volatile float   tsc_vload_v = 0;
volatile float   tsc_power_w = 0;
volatile int16_t tsc_vshunt_raw = 0;
volatile uint8_t tsc_online = 0;

static I2C_HandleTypeDef *tsc_bus;

uint8_t TSC1641_Init(I2C_HandleTypeDef *hi2c)
{
  tsc_bus = hi2c;

  /* 0x0037 = continuous shunt+load conversion, 1024 us conversion time */
  if (xnx_i2c_write_reg16(tsc_bus, TSC1641_ADDR, TSC1641_REG_CONF, 0x0037) != HAL_OK)
    return 0;
  if (xnx_i2c_write_reg16(tsc_bus, TSC1641_ADDR, TSC1641_REG_RSHUNT, TSC1641_RSHUNT_10UOHM) != HAL_OK)
    return 0;
  return 1;
}

void TSC1641_Update(void)
{
  uint16_t raw;
  uint8_t ok = 1;

  if (tsc_bus == NULL) {
    tsc_online = 0;
    return;
  }

  /* current LSB = 2.5 uV / 5 mOhm = 0.5 mA */
  if (xnx_i2c_read_reg16(tsc_bus, TSC1641_ADDR, TSC1641_REG_CURRENT, &raw) == HAL_OK)
    tsc_current_a = (int16_t)raw * 0.0005f;
  else
    ok = 0;

  if (xnx_i2c_read_reg16(tsc_bus, TSC1641_ADDR, TSC1641_REG_VLOAD, &raw) == HAL_OK)
    tsc_vload_v = (int16_t)raw * 0.002f;
  else
    ok = 0;

  if (xnx_i2c_read_reg16(tsc_bus, TSC1641_ADDR, TSC1641_REG_POWER, &raw) == HAL_OK)
    tsc_power_w = (int16_t)raw * 0.025f;
  else
    ok = 0;

  if (xnx_i2c_read_reg16(tsc_bus, TSC1641_ADDR, TSC1641_REG_VSHUNT, &raw) == HAL_OK)
    tsc_vshunt_raw = (int16_t)raw;
  else
    ok = 0;

  tsc_online = ok;
}
