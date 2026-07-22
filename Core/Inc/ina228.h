#ifndef INA228_H
#define INA228_H

#include <stdint.h>
#include "xnx_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * INA228 — 85 V, 20-bit power monitor (TI INA228EVM board).
 *
 * The EVM leaves the R1 shunt pads empty: a shunt must be soldered on
 * (or wired across J1) before anything can be measured. J1 pinout is
 * pin 1 = GND, pin 2 = VBUS, pins 3/4 = IN-, pins 5/6 = IN+.
 *
 * With A0/A1 both grounded the device answers on 0x40, which is where
 * the TSC1641 already sits. Tie A0 to VS to move it to 0x41.
 */

#define INA228_ADDRESS_A1GND_A0GND       0x40U
#define INA228_ADDRESS_A1GND_A0VS        0x41U
#define INA228_ADDRESS_A1VS_A0GND        0x44U
#define INA228_ADDRESS_A1VS_A0VS         0x45U

#define INA228_REG_CONFIG                0x00U
#define INA228_REG_ADC_CONFIG            0x01U
#define INA228_REG_SHUNT_CAL             0x02U
#define INA228_REG_VSHUNT                0x04U
#define INA228_REG_VBUS                  0x05U
#define INA228_REG_DIETEMP               0x06U
#define INA228_REG_CURRENT               0x07U
#define INA228_REG_POWER                 0x08U
#define INA228_REG_MANUFACTURER_ID       0x3EU
#define INA228_REG_DEVICE_ID             0x3FU

#define INA228_MANUFACTURER_ID_VALUE     0x5449U   /* "TI" */
#define INA228_DEVICE_ID_VALUE           0x228U    /* upper 12 bits */

typedef enum
{
    INA228_STATUS_OK = 0,
    INA228_STATUS_INVALID_ARGUMENT,
    INA228_STATUS_I2C_ERROR,
    INA228_STATUS_DEVICE_NOT_FOUND,
    INA228_STATUS_BAD_DEVICE_ID
} ina228_status_t;

typedef enum
{
    /* Shunt input range: wide is +-163.84 mV, precise is +-40.96 mV
       with four times finer resolution. */
    INA228_RANGE_WIDE = 0,
    INA228_RANGE_PRECISE = 1
} ina228_range_t;

typedef struct
{
    float shunt_ohms;         /* what is actually fitted, 5 mOhm = 0.005f */
    float max_current_a;      /* sets one current LSB = max / 2^19 */
    ina228_range_t range;
} ina228_config_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint16_t device_id;
    uint16_t shunt_cal;        /* what was programmed, for the health check */
    float current_lsb_a;
    ina228_range_t range;
    uint8_t initialized;
} ina228_t;

typedef struct
{
    int32_t current_raw;
    int32_t shunt_raw;
    uint32_t bus_raw;
    uint32_t power_raw;

    float current_a;          /* signed: shows direction of flow */
    float bus_voltage_v;
    float power_w;
    float shunt_voltage_mv;
    float die_temperature_c;
} ina228_data_t;

void ina228_get_default_config(ina228_config_t *config);

ina228_status_t ina228_init(
    ina228_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address,
    const ina228_config_t *config);

ina228_status_t ina228_read(
    ina228_t *device,
    ina228_data_t *data);

/* Confirm the chip still holds its calibration - see tsc1641_check(). */
ina228_status_t ina228_check(ina228_t *device);

const char *ina228_status_string(ina228_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* INA228_H */
