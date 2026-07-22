#ifndef TSC1641_H
#define TSC1641_H

#include <stdint.h>
#include "xnx_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TSC1641 — power monitor on the STEVAL-DIGAFEV1 eval board.
 *
 * Wiring here: I2C1, PB6 = SCL, PB9 = SDA. The board carries 4.7k
 * pull-ups (JP_SDA/JP_SCL), address jumpers JP_A0/JP_A1 on GND -> 0x40,
 * and a 5 mOhm shunt (R6).
 */

/* JP_A1/JP_A0 jumper positions, datasheet DS14338 Table 6. The pins must
   never be left floating: pull the jumpers off and the address is
   anyone's guess. */
#define TSC1641_ADDRESS_A1GND_A0GND      0x40U
#define TSC1641_ADDRESS_A1GND_A0VS       0x41U
#define TSC1641_ADDRESS_A1VS_A0GND       0x42U
#define TSC1641_ADDRESS_A1VS_A0VS        0x43U

#define TSC1641_REG_CONF                 0x00U
#define TSC1641_REG_VSHUNT               0x01U
#define TSC1641_REG_VLOAD                0x02U
#define TSC1641_REG_POWER                0x03U
#define TSC1641_REG_CURRENT              0x04U
#define TSC1641_REG_FLAGS                0x07U
#define TSC1641_REG_RSHUNT               0x08U
#define TSC1641_REG_DIE_ID               0xFFU

#define TSC1641_DIE_ID_VALUE             0x1000U

/*
 * Note: the register table in DS14338 lists 0x03 as power and 0x04 as
 * current, but there are reports of the two being swapped on real
 * silicon. If current and power come out exchanged, swap the defines.
 */

typedef enum
{
    TSC1641_STATUS_OK = 0,
    TSC1641_STATUS_INVALID_ARGUMENT,
    TSC1641_STATUS_I2C_ERROR,
    TSC1641_STATUS_DEVICE_NOT_FOUND,
    TSC1641_STATUS_BAD_DEVICE_ID
} tsc1641_status_t;

typedef struct
{
    float shunt_ohms;    /* R6 on the eval board is 5 mOhm */
} tsc1641_config_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint16_t die_id;
    float current_lsb_a;
    uint8_t initialized;
} tsc1641_t;

typedef struct
{
    int16_t current_raw;
    int16_t shunt_raw;
    int16_t load_raw;
    int16_t power_raw;

    float current_a;         /* signed: shows direction of flow */
    float load_voltage_v;
    float power_w;
    float shunt_voltage_mv;
} tsc1641_data_t;

void tsc1641_get_default_config(tsc1641_config_t *config);

tsc1641_status_t tsc1641_init(
    tsc1641_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address,
    const tsc1641_config_t *config);

tsc1641_status_t tsc1641_read(
    tsc1641_t *device,
    tsc1641_data_t *data);

const char *tsc1641_status_string(tsc1641_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* TSC1641_H */
