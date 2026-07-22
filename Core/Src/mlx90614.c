#include "mlx90614.h"

#include <stddef.h>

#define MLX90614_READ_WORD_SIZE       3U
#define MLX90614_ERROR_FLAG           0x8000U
#define MLX90614_CENTI_KELVIN_OFFSET  27315L

static uint8_t mlx90614_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0U;

    for (size_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];

        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

static mlx90614_status_t mlx90614_read_word(
    mlx90614_t *device,
    uint8_t command,
    uint16_t *value)
{
    uint8_t rx[MLX90614_READ_WORD_SIZE];

    if ((device == NULL) ||
        (device->hi2c == NULL) ||
        (value == NULL))
    {
        return MLX90614_STATUS_INVALID_ARGUMENT;
    }

    if (xnx_i2c_read(
            device->hi2c,
            device->address,
            command,
            rx,
            sizeof(rx)) != HAL_OK)
    {
        return MLX90614_STATUS_I2C_ERROR;
    }

    const uint8_t pec_input[5] = {
        (uint8_t)(device->address << 1U),
        command,
        (uint8_t)((device->address << 1U) | 0x01U),
        rx[0],
        rx[1]
    };

    if (mlx90614_crc8(pec_input, sizeof(pec_input)) != rx[2])
    {
        return MLX90614_STATUS_PEC_ERROR;
    }

    *value = (uint16_t)(((uint16_t)rx[1] << 8U) | rx[0]);

    return MLX90614_STATUS_OK;
}

static mlx90614_status_t mlx90614_raw_to_centi_c(
    uint16_t raw,
    int32_t *temperature_centi_c)
{
    if (temperature_centi_c == NULL)
    {
        return MLX90614_STATUS_INVALID_ARGUMENT;
    }

    if ((raw & MLX90614_ERROR_FLAG) != 0U)
    {
        return MLX90614_STATUS_SENSOR_ERROR;
    }

    /*
     * MLX90614: 1 LSB = 0.02 K.
     * In hundredths of °C:
     * T[cC] = raw * 2 - 27315.
     */
    *temperature_centi_c =
        ((int32_t)raw * 2L) - MLX90614_CENTI_KELVIN_OFFSET;

    return MLX90614_STATUS_OK;
}

mlx90614_status_t mlx90614_init(
    mlx90614_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address)
{
    uint16_t id_word = 0U;

    if ((device == NULL) || (hi2c == NULL) || (address > 0x7FU))
    {
        return MLX90614_STATUS_INVALID_ARGUMENT;
    }

    device->hi2c = hi2c;
    device->address = address;
    device->id_word = 0U;
    device->initialized = 0U;

    if (xnx_i2c_is_device_ready(hi2c, address, 3U, 100U) != HAL_OK)
    {
        return MLX90614_STATUS_NOT_FOUND;
    }

    mlx90614_status_t status =
        mlx90614_read_word(device, MLX90614_EEPROM_ID1, &id_word);

    if (status != MLX90614_STATUS_OK)
    {
        return status;
    }

    if ((id_word == 0x0000U) || (id_word == 0xFFFFU))
    {
        return MLX90614_STATUS_BAD_ID;
    }

    device->id_word = id_word;
    device->initialized = 1U;

    return MLX90614_STATUS_OK;
}

mlx90614_status_t mlx90614_read_ambient(
    mlx90614_t *device,
    int32_t *temperature_centi_c)
{
    uint16_t raw = 0U;

    if ((device == NULL) ||
        (device->initialized == 0U) ||
        (temperature_centi_c == NULL))
    {
        return MLX90614_STATUS_INVALID_ARGUMENT;
    }

    mlx90614_status_t status =
        mlx90614_read_word(device, MLX90614_RAM_TA, &raw);

    if (status != MLX90614_STATUS_OK)
    {
        return status;
    }

    return mlx90614_raw_to_centi_c(raw, temperature_centi_c);
}

mlx90614_status_t mlx90614_read_object(
    mlx90614_t *device,
    int32_t *temperature_centi_c)
{
    uint16_t raw = 0U;

    if ((device == NULL) ||
        (device->initialized == 0U) ||
        (temperature_centi_c == NULL))
    {
        return MLX90614_STATUS_INVALID_ARGUMENT;
    }

    mlx90614_status_t status =
        mlx90614_read_word(device, MLX90614_RAM_TOBJ1, &raw);

    if (status != MLX90614_STATUS_OK)
    {
        return status;
    }

    return mlx90614_raw_to_centi_c(raw, temperature_centi_c);
}

mlx90614_status_t mlx90614_read(
    mlx90614_t *device,
    mlx90614_data_t *data)
{
    if ((device == NULL) ||
        (device->initialized == 0U) ||
        (data == NULL))
    {
        return MLX90614_STATUS_INVALID_ARGUMENT;
    }

    mlx90614_status_t status =
        mlx90614_read_word(
            device,
            MLX90614_RAM_TA,
            &data->ambient_raw);

    if (status != MLX90614_STATUS_OK)
    {
        return status;
    }

    status = mlx90614_raw_to_centi_c(
        data->ambient_raw,
        &data->ambient_centi_c);

    if (status != MLX90614_STATUS_OK)
    {
        return status;
    }

    status = mlx90614_read_word(
        device,
        MLX90614_RAM_TOBJ1,
        &data->object_raw);

    if (status != MLX90614_STATUS_OK)
    {
        return status;
    }

    return mlx90614_raw_to_centi_c(
        data->object_raw,
        &data->object_centi_c);
}

const char *mlx90614_status_string(mlx90614_status_t status)
{
    switch (status)
    {
        case MLX90614_STATUS_OK:
            return "OK";

        case MLX90614_STATUS_INVALID_ARGUMENT:
            return "invalid argument";

        case MLX90614_STATUS_NOT_FOUND:
            return "device not found";

        case MLX90614_STATUS_I2C_ERROR:
            return "I2C error";

        case MLX90614_STATUS_PEC_ERROR:
            return "PEC error";

        case MLX90614_STATUS_SENSOR_ERROR:
            return "sensor error flag";

        case MLX90614_STATUS_BAD_ID:
            return "bad ID";

        default:
            return "unknown";
    }
}
