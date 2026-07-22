#include "ina228.h"

#include <stddef.h>

#define INA228_WORD_SIZE_24BIT        3U

/* Fixed conversion factors, datasheet SLYS021A section 7.6. */
#define INA228_VBUS_V_PER_LSB         0.0001953125f    /* 195.3125 uV */
#define INA228_VSHUNT_V_PER_LSB_WIDE  0.0000003125f    /* 312.5 nV */
#define INA228_VSHUNT_V_PER_LSB_FINE  0.000000078125f  /* 78.125 nV */
#define INA228_DIETEMP_C_PER_LSB      0.0078125f       /* 7.8125 m°C */
#define INA228_POWER_LSB_FACTOR       3.2f
#define INA228_SHUNT_CAL_CONSTANT     13107200000.0f   /* 13107.2e6 */
#define INA228_CURRENT_LSB_DIVISOR    524288.0f        /* 2^19 */
#define INA228_SHUNT_CAL_MAX          32767.0f         /* register is 15 bits */

#define INA228_CONFIG_RESET           0x8000U
#define INA228_CONFIG_ADCRANGE        0x0010U

/* Continuous conversion of bus, shunt and temperature; 1052 us per
   channel, averaged over 16 samples to keep the reading steady. */
#define INA228_ADC_CONFIG_CONTINUOUS  0xFB6AU

static ina228_status_t ina228_read_word(
    const ina228_t *device,
    uint8_t reg,
    uint16_t *value)
{
    if ((device == NULL) ||
        (device->hi2c == NULL) ||
        (value == NULL))
    {
        return INA228_STATUS_INVALID_ARGUMENT;
    }

    if (xnx_i2c_read_reg16(
            device->hi2c,
            device->address,
            reg,
            value) != HAL_OK)
    {
        return INA228_STATUS_I2C_ERROR;
    }

    return INA228_STATUS_OK;
}

static ina228_status_t ina228_write_word(
    const ina228_t *device,
    uint8_t reg,
    uint16_t value)
{
    if ((device == NULL) || (device->hi2c == NULL))
    {
        return INA228_STATUS_INVALID_ARGUMENT;
    }

    if (xnx_i2c_write_reg16(
            device->hi2c,
            device->address,
            reg,
            value) != HAL_OK)
    {
        return INA228_STATUS_I2C_ERROR;
    }

    return INA228_STATUS_OK;
}

static ina228_status_t ina228_read_long_word(
    const ina228_t *device,
    uint8_t reg,
    uint32_t *value)
{
    uint8_t rx[INA228_WORD_SIZE_24BIT];

    if ((device == NULL) ||
        (device->hi2c == NULL) ||
        (value == NULL))
    {
        return INA228_STATUS_INVALID_ARGUMENT;
    }

    if (xnx_i2c_read(
            device->hi2c,
            device->address,
            reg,
            rx,
            sizeof(rx)) != HAL_OK)
    {
        return INA228_STATUS_I2C_ERROR;
    }

    *value = ((uint32_t)rx[0] << 16U) |
             ((uint32_t)rx[1] << 8U) |
             (uint32_t)rx[2];

    return INA228_STATUS_OK;
}

/*
 * The measurement registers are read as three bytes, but only the top 20
 * bits carry the result - the lowest four are reserved and always zero.
 * Shifting them away leaves a 20-bit signed number, so the sign bit sits
 * at position 19 and a set sign bit means the value must be brought below
 * zero by subtracting 2^20. A negative current simply means it flows the
 * other way through the shunt.
 */
static int32_t ina228_raw_to_signed_20bit(uint32_t raw)
{
    int32_t value = (int32_t)(raw >> 4U);

    if ((value & 0x00080000L) != 0L)
    {
        value -= 0x00100000L;
    }

    return value;
}

void ina228_get_default_config(ina228_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->shunt_ohms = 0.005f;      /* 5 mOhm soldered on R1 */
    config->max_current_a = 10.0f;    /* 19.07 uA per current count */
    config->range = INA228_RANGE_WIDE;
}

ina228_status_t ina228_init(
    ina228_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address,
    const ina228_config_t *config)
{
    uint16_t manufacturer_id = 0U;
    uint16_t device_id = 0U;
    uint16_t config_word = 0U;
    float shunt_cal = 0.0f;

    if ((device == NULL) ||
        (hi2c == NULL) ||
        (config == NULL) ||
        (address > 0x7FU))
    {
        return INA228_STATUS_INVALID_ARGUMENT;
    }

    if ((config->shunt_ohms <= 0.0f) || (config->max_current_a <= 0.0f))
    {
        return INA228_STATUS_INVALID_ARGUMENT;
    }

    device->hi2c = hi2c;
    device->address = address;
    device->device_id = 0U;
    device->range = config->range;
    device->initialized = 0U;

    if (xnx_i2c_is_device_ready(hi2c, address, 3U, 100U) != HAL_OK)
    {
        return INA228_STATUS_DEVICE_NOT_FOUND;
    }

    ina228_status_t status =
        ina228_read_word(device, INA228_REG_MANUFACTURER_ID, &manufacturer_id);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    status = ina228_read_word(device, INA228_REG_DEVICE_ID, &device_id);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    device->device_id = device_id;

    if ((manufacturer_id != INA228_MANUFACTURER_ID_VALUE) ||
        ((device_id >> 4U) != INA228_DEVICE_ID_VALUE))
    {
        return INA228_STATUS_BAD_DEVICE_ID;
    }

    status = ina228_write_word(device, INA228_REG_CONFIG, INA228_CONFIG_RESET);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    HAL_Delay(2U);

    if (device->range == INA228_RANGE_PRECISE)
    {
        config_word |= INA228_CONFIG_ADCRANGE;
    }

    status = ina228_write_word(device, INA228_REG_CONFIG, config_word);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    status = ina228_write_word(
        device,
        INA228_REG_ADC_CONFIG,
        INA228_ADC_CONFIG_CONTINUOUS);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    /*
     * Calibration, datasheet equations 2 and 3.
     *
     * The chip reports current as a plain integer count, and CURRENT_LSB
     * is how many amps one count is worth. The current register is 20 bits
     * wide and signed, so it can hold +-2^19 counts; dividing the largest
     * current we ever expect by 2^19 spreads those counts over exactly
     * that range. Asking for more current than needed therefore costs
     * resolution, asking for less clips the reading.
     *
     * SHUNT_CAL then tells the chip how the measured shunt voltage
     * translates into those counts, which is why the shunt value must
     * match what is physically soldered on the board.
     */
    device->current_lsb_a = config->max_current_a / INA228_CURRENT_LSB_DIVISOR;

    shunt_cal = INA228_SHUNT_CAL_CONSTANT *
                device->current_lsb_a *
                config->shunt_ohms;

    if (device->range == INA228_RANGE_PRECISE)
    {
        shunt_cal *= 4.0f;
    }

    if (shunt_cal > INA228_SHUNT_CAL_MAX)
    {
        shunt_cal = INA228_SHUNT_CAL_MAX;
    }

    status = ina228_write_word(
        device,
        INA228_REG_SHUNT_CAL,
        (uint16_t)(shunt_cal + 0.5f));

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    device->initialized = 1U;

    return INA228_STATUS_OK;
}

ina228_status_t ina228_read(
    ina228_t *device,
    ina228_data_t *data)
{
    uint32_t raw = 0U;
    uint16_t raw_word = 0U;
    float shunt_v_per_lsb = 0.0f;

    if ((device == NULL) ||
        (device->initialized == 0U) ||
        (data == NULL))
    {
        return INA228_STATUS_INVALID_ARGUMENT;
    }

    ina228_status_t status =
        ina228_read_long_word(device, INA228_REG_CURRENT, &raw);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    data->current_raw = ina228_raw_to_signed_20bit(raw);
    data->current_a = (float)data->current_raw * device->current_lsb_a;

    status = ina228_read_long_word(device, INA228_REG_VBUS, &raw);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    data->bus_raw = raw >> 4U;
    data->bus_voltage_v = (float)data->bus_raw * INA228_VBUS_V_PER_LSB;

    status = ina228_read_long_word(device, INA228_REG_POWER, &raw);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    data->power_raw = raw;
    data->power_w = INA228_POWER_LSB_FACTOR *
                    device->current_lsb_a *
                    (float)raw;

    status = ina228_read_long_word(device, INA228_REG_VSHUNT, &raw);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    shunt_v_per_lsb = (device->range == INA228_RANGE_PRECISE)
                          ? INA228_VSHUNT_V_PER_LSB_FINE
                          : INA228_VSHUNT_V_PER_LSB_WIDE;

    data->shunt_raw = ina228_raw_to_signed_20bit(raw);
    data->shunt_voltage_mv = (float)data->shunt_raw * shunt_v_per_lsb * 1000.0f;

    status = ina228_read_word(device, INA228_REG_DIETEMP, &raw_word);

    if (status != INA228_STATUS_OK)
    {
        return status;
    }

    data->die_temperature_c = (float)(int16_t)raw_word * INA228_DIETEMP_C_PER_LSB;

    return INA228_STATUS_OK;
}

const char *ina228_status_string(ina228_status_t status)
{
    switch (status)
    {
        case INA228_STATUS_OK:
            return "OK";

        case INA228_STATUS_INVALID_ARGUMENT:
            return "invalid argument";

        case INA228_STATUS_I2C_ERROR:
            return "I2C error";

        case INA228_STATUS_DEVICE_NOT_FOUND:
            return "device not found";

        case INA228_STATUS_BAD_DEVICE_ID:
            return "bad device ID";

        default:
            return "unknown";
    }
}
