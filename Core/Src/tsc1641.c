#include "tsc1641.h"

#include <stddef.h>

/* Continuous shunt and load conversion, 1024 us (power-up default). */
#define TSC1641_CONF_CONTINUOUS       0x0037U

#define TSC1641_RSHUNT_OHMS_PER_LSB   0.00001f   /* 10 uOhm */
#define TSC1641_VSHUNT_V_PER_LSB      0.0000025f /* 2.5 uV */
#define TSC1641_VLOAD_V_PER_LSB       0.002f     /* 2 mV */
#define TSC1641_POWER_W_PER_LSB       0.025f     /* 25 mW */

static tsc1641_status_t tsc1641_read_word(
    const tsc1641_t *device,
    uint8_t reg,
    uint16_t *value)
{
    if ((device == NULL) ||
        (device->hi2c == NULL) ||
        (value == NULL))
    {
        return TSC1641_STATUS_INVALID_ARGUMENT;
    }

    if (xnx_i2c_read_reg16(
            device->hi2c,
            device->address,
            reg,
            value) != HAL_OK)
    {
        return TSC1641_STATUS_I2C_ERROR;
    }

    return TSC1641_STATUS_OK;
}

static tsc1641_status_t tsc1641_write_word(
    const tsc1641_t *device,
    uint8_t reg,
    uint16_t value)
{
    if ((device == NULL) || (device->hi2c == NULL))
    {
        return TSC1641_STATUS_INVALID_ARGUMENT;
    }

    if (xnx_i2c_write_reg16(
            device->hi2c,
            device->address,
            reg,
            value) != HAL_OK)
    {
        return TSC1641_STATUS_I2C_ERROR;
    }

    return TSC1641_STATUS_OK;
}

void tsc1641_get_default_config(tsc1641_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->shunt_ohms = 0.005f;
}

tsc1641_status_t tsc1641_init(
    tsc1641_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address,
    const tsc1641_config_t *config)
{
    uint16_t rshunt_register = 0U;
    uint16_t die_id = 0U;

    if ((device == NULL) ||
        (hi2c == NULL) ||
        (config == NULL) ||
        (address > 0x7FU))
    {
        return TSC1641_STATUS_INVALID_ARGUMENT;
    }

    if (config->shunt_ohms <= 0.0f)
    {
        return TSC1641_STATUS_INVALID_ARGUMENT;
    }

    device->hi2c = hi2c;
    device->address = address;
    device->die_id = 0U;
    device->initialized = 0U;

    if (xnx_i2c_is_device_ready(hi2c, address, 3U, 100U) != HAL_OK)
    {
        return TSC1641_STATUS_DEVICE_NOT_FOUND;
    }

    /*
     * Check who answered BEFORE writing anything: 0x40 is also the
     * factory address of the INA228, and configuring the wrong chip
     * would scribble over its registers.
     */
    tsc1641_status_t status =
        tsc1641_read_word(device, TSC1641_REG_DIE_ID, &die_id);

    if (status != TSC1641_STATUS_OK)
    {
        return TSC1641_STATUS_BAD_DEVICE_ID;
    }

    device->die_id = die_id;

    if (die_id != TSC1641_DIE_ID_VALUE)
    {
        return TSC1641_STATUS_BAD_DEVICE_ID;
    }

    status = tsc1641_write_word(
        device,
        TSC1641_REG_CONF,
        TSC1641_CONF_CONTINUOUS);

    if (status != TSC1641_STATUS_OK)
    {
        return status;
    }

    /*
     * Telling the chip the shunt value lets it work out the current on
     * its own, instead of the microcontroller doing the division. The
     * register counts in steps of 10 uOhm, so a 5 mOhm shunt is 500.
     */
    rshunt_register =
        (uint16_t)((config->shunt_ohms / TSC1641_RSHUNT_OHMS_PER_LSB) + 0.5f);

    status = tsc1641_write_word(device, TSC1641_REG_RSHUNT, rshunt_register);

    if (status != TSC1641_STATUS_OK)
    {
        return status;
    }

    device->shunt_register = rshunt_register;

    /*
     * What one count of the current register is worth, by Ohm's law: the
     * smallest shunt voltage the chip can resolve is 2.5 uV, so across a
     * 5 mOhm shunt that is 2.5 uV / 5 mOhm = 500 uA per count. A smaller
     * shunt measures larger currents but resolves them more coarsely.
     */
    device->current_lsb_a = TSC1641_VSHUNT_V_PER_LSB / config->shunt_ohms;

    device->initialized = 1U;

    return TSC1641_STATUS_OK;
}

tsc1641_status_t tsc1641_read(
    tsc1641_t *device,
    tsc1641_data_t *data)
{
    uint16_t raw = 0U;

    if ((device == NULL) ||
        (device->initialized == 0U) ||
        (data == NULL))
    {
        return TSC1641_STATUS_INVALID_ARGUMENT;
    }

    tsc1641_status_t status =
        tsc1641_read_word(device, TSC1641_REG_CURRENT, &raw);

    if (status != TSC1641_STATUS_OK)
    {
        return status;
    }

    data->current_raw = (int16_t)raw;
    data->current_a = (float)data->current_raw * device->current_lsb_a;

    status = tsc1641_read_word(device, TSC1641_REG_VLOAD, &raw);

    if (status != TSC1641_STATUS_OK)
    {
        return status;
    }

    data->load_raw = (int16_t)raw;
    data->load_voltage_v = (float)data->load_raw * TSC1641_VLOAD_V_PER_LSB;

    status = tsc1641_read_word(device, TSC1641_REG_POWER, &raw);

    if (status != TSC1641_STATUS_OK)
    {
        return status;
    }

    data->power_raw = (int16_t)raw;
    data->power_w = (float)data->power_raw * TSC1641_POWER_W_PER_LSB;

    status = tsc1641_read_word(device, TSC1641_REG_VSHUNT, &raw);

    if (status != TSC1641_STATUS_OK)
    {
        return status;
    }

    data->shunt_raw = (int16_t)raw;
    data->shunt_voltage_mv =
        (float)data->shunt_raw * TSC1641_VSHUNT_V_PER_LSB * 1000.0f;

    return TSC1641_STATUS_OK;
}

tsc1641_status_t tsc1641_check(tsc1641_t *device)
{
    uint16_t rshunt = 0U;

    if ((device == NULL) || (device->initialized == 0U))
    {
        return TSC1641_STATUS_INVALID_ARGUMENT;
    }

    tsc1641_status_t status =
        tsc1641_read_word(device, TSC1641_REG_RSHUNT, &rshunt);

    if (status != TSC1641_STATUS_OK)
    {
        return status;
    }

    /* An empty shunt register means the chip restarted behind our back. */
    if (rshunt != device->shunt_register)
    {
        return TSC1641_STATUS_DEVICE_NOT_FOUND;
    }

    return TSC1641_STATUS_OK;
}

const char *tsc1641_status_string(tsc1641_status_t status)
{
    switch (status)
    {
        case TSC1641_STATUS_OK:
            return "OK";

        case TSC1641_STATUS_INVALID_ARGUMENT:
            return "invalid argument";

        case TSC1641_STATUS_I2C_ERROR:
            return "I2C error";

        case TSC1641_STATUS_DEVICE_NOT_FOUND:
            return "device not found";

        case TSC1641_STATUS_BAD_DEVICE_ID:
            return "bad device ID (another chip on this address?)";

        default:
            return "unknown";
    }
}
