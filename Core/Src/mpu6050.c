#include "mpu6050.h"

#define MPU6050_REG_SMPLRT_DIV           0x19U
#define MPU6050_REG_CONFIG               0x1AU
#define MPU6050_REG_GYRO_CONFIG          0x1BU
#define MPU6050_REG_ACCEL_CONFIG         0x1CU
#define MPU6050_REG_ACCEL_XOUT_H         0x3BU
#define MPU6050_REG_PWR_MGMT_1           0x6BU
#define MPU6050_REG_PWR_MGMT_2           0x6CU
#define MPU6050_REG_WHO_AM_I             0x75U

#define MPU6050_PWR1_DEVICE_RESET        0x80U
#define MPU6050_PWR1_CLKSEL_PLL_XGYRO    0x01U
#define MPU6050_RANGE_FIELD_MASK         0x18U
#define MPU6050_DLPF_FIELD_MASK          0x07U
#define MPU6050_BURST_DATA_LENGTH        14U

static mpu6050_status_t mpu6050_from_hal_status(HAL_StatusTypeDef status)
{
    return (status == HAL_OK) ? MPU6050_STATUS_OK : MPU6050_STATUS_I2C_ERROR;
}

static int16_t mpu6050_be_to_i16(const uint8_t *bytes)
{
    const uint16_t value =
        (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
    return (int16_t)value;
}

static float mpu6050_accel_scale(mpu6050_accel_range_t range)
{
    switch (range)
    {
        case MPU6050_ACCEL_RANGE_2G:  return 16384.0f;
        case MPU6050_ACCEL_RANGE_4G:  return 8192.0f;
        case MPU6050_ACCEL_RANGE_8G:  return 4096.0f;
        case MPU6050_ACCEL_RANGE_16G: return 2048.0f;
        default:                      return 0.0f;
    }
}

static float mpu6050_gyro_scale(mpu6050_gyro_range_t range)
{
    switch (range)
    {
        case MPU6050_GYRO_RANGE_250_DPS:  return 131.0f;
        case MPU6050_GYRO_RANGE_500_DPS:  return 65.5f;
        case MPU6050_GYRO_RANGE_1000_DPS: return 32.8f;
        case MPU6050_GYRO_RANGE_2000_DPS: return 16.4f;
        default:                          return 0.0f;
    }
}

static mpu6050_status_t mpu6050_update_bits(
    const mpu6050_t *device,
    uint8_t reg,
    uint8_t mask,
    uint8_t value)
{
    if ((device == NULL) || (device->hi2c == NULL))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    uint8_t current = 0U;
    if (xnx_i2c_read_reg8(
            device->hi2c,
            device->address,
            reg,
            &current) != HAL_OK)
    {
        return MPU6050_STATUS_I2C_ERROR;
    }

    current = (uint8_t)((current & (uint8_t)(~mask)) | (value & mask));

    return mpu6050_from_hal_status(
        xnx_i2c_write_reg8(device->hi2c, device->address, reg, current));
}

void mpu6050_get_default_config(mpu6050_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->accel_range = MPU6050_ACCEL_RANGE_2G;
    config->gyro_range = MPU6050_GYRO_RANGE_250_DPS;
    config->dlpf = MPU6050_DLPF_44_HZ;
    config->sample_rate_divider = 9U; /* 1 kHz / (1 + 9) = 100 Hz */
}

mpu6050_status_t mpu6050_read_who_am_i(
    const mpu6050_t *device,
    uint8_t *who_am_i)
{
    if ((device == NULL) || (device->hi2c == NULL) || (who_am_i == NULL))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    return mpu6050_from_hal_status(
        xnx_i2c_read_reg8(
            device->hi2c,
            device->address,
            MPU6050_REG_WHO_AM_I,
            who_am_i));
}

mpu6050_status_t mpu6050_set_accel_range(
    mpu6050_t *device,
    mpu6050_accel_range_t range)
{
    const float scale = mpu6050_accel_scale(range);
    if ((device == NULL) || (scale <= 0.0f))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    const mpu6050_status_t status = mpu6050_update_bits(
        device,
        MPU6050_REG_ACCEL_CONFIG,
        MPU6050_RANGE_FIELD_MASK,
        (uint8_t)((uint8_t)range << 3U));

    if (status == MPU6050_STATUS_OK)
    {
        device->config.accel_range = range;
        device->accel_lsb_per_g = scale;
    }

    return status;
}

mpu6050_status_t mpu6050_set_gyro_range(
    mpu6050_t *device,
    mpu6050_gyro_range_t range)
{
    const float scale = mpu6050_gyro_scale(range);
    if ((device == NULL) || (scale <= 0.0f))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    const mpu6050_status_t status = mpu6050_update_bits(
        device,
        MPU6050_REG_GYRO_CONFIG,
        MPU6050_RANGE_FIELD_MASK,
        (uint8_t)((uint8_t)range << 3U));

    if (status == MPU6050_STATUS_OK)
    {
        device->config.gyro_range = range;
        device->gyro_lsb_per_dps = scale;
    }

    return status;
}

mpu6050_status_t mpu6050_set_dlpf(
    mpu6050_t *device,
    mpu6050_dlpf_t dlpf)
{
    if ((device == NULL) || ((uint8_t)dlpf > (uint8_t)MPU6050_DLPF_5_HZ))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    const mpu6050_status_t status = mpu6050_update_bits(
        device,
        MPU6050_REG_CONFIG,
        MPU6050_DLPF_FIELD_MASK,
        (uint8_t)dlpf);

    if (status == MPU6050_STATUS_OK)
    {
        device->config.dlpf = dlpf;
    }

    return status;
}

mpu6050_status_t mpu6050_set_sample_rate_divider(
    mpu6050_t *device,
    uint8_t divider)
{
    if ((device == NULL) || (device->hi2c == NULL))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    const mpu6050_status_t status = mpu6050_from_hal_status(
        xnx_i2c_write_reg8(
            device->hi2c,
            device->address,
            MPU6050_REG_SMPLRT_DIV,
            divider));

    if (status == MPU6050_STATUS_OK)
    {
        device->config.sample_rate_divider = divider;
    }

    return status;
}

mpu6050_status_t mpu6050_init(
    mpu6050_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address,
    const mpu6050_config_t *config)
{
    if ((device == NULL) || (hi2c == NULL) || (config == NULL))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    if ((address != MPU6050_ADDRESS_AD0_LOW) &&
        (address != MPU6050_ADDRESS_AD0_HIGH))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    device->hi2c = hi2c;
    device->address = address;
    device->accel_lsb_per_g = 0.0f;
    device->gyro_lsb_per_dps = 0.0f;
    device->config = *config;

    if (xnx_i2c_is_device_ready(hi2c, address, 3U, 20U) != HAL_OK)
    {
        return MPU6050_STATUS_DEVICE_NOT_FOUND;
    }

    uint8_t who_am_i = 0U;
    mpu6050_status_t status = mpu6050_read_who_am_i(device, &who_am_i);
    if (status != MPU6050_STATUS_OK)
    {
        return status;
    }
    if (who_am_i != MPU6050_WHO_AM_I_VALUE)
    {
        return MPU6050_STATUS_BAD_DEVICE_ID;
    }

    if (xnx_i2c_write_reg8(
            hi2c,
            address,
            MPU6050_REG_PWR_MGMT_1,
            MPU6050_PWR1_DEVICE_RESET) != HAL_OK)
    {
        return MPU6050_STATUS_I2C_ERROR;
    }
    HAL_Delay(100U);

    if (xnx_i2c_write_reg8(
            hi2c,
            address,
            MPU6050_REG_PWR_MGMT_1,
            MPU6050_PWR1_CLKSEL_PLL_XGYRO) != HAL_OK)
    {
        return MPU6050_STATUS_I2C_ERROR;
    }

    if (xnx_i2c_write_reg8(
            hi2c,
            address,
            MPU6050_REG_PWR_MGMT_2,
            0x00U) != HAL_OK)
    {
        return MPU6050_STATUS_I2C_ERROR;
    }
    HAL_Delay(10U);

    status = mpu6050_set_dlpf(device, config->dlpf);
    if (status != MPU6050_STATUS_OK)
    {
        return status;
    }

    status = mpu6050_set_sample_rate_divider(
        device,
        config->sample_rate_divider);
    if (status != MPU6050_STATUS_OK)
    {
        return status;
    }

    status = mpu6050_set_gyro_range(device, config->gyro_range);
    if (status != MPU6050_STATUS_OK)
    {
        return status;
    }

    status = mpu6050_set_accel_range(device, config->accel_range);
    if (status != MPU6050_STATUS_OK)
    {
        return status;
    }

    return MPU6050_STATUS_OK;
}

mpu6050_status_t mpu6050_read_raw(
    const mpu6050_t *device,
    mpu6050_raw_data_t *data)
{
    if ((device == NULL) || (device->hi2c == NULL) || (data == NULL))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    uint8_t buffer[MPU6050_BURST_DATA_LENGTH];
    if (xnx_i2c_read(
            device->hi2c,
            device->address,
            MPU6050_REG_ACCEL_XOUT_H,
            buffer,
            MPU6050_BURST_DATA_LENGTH) != HAL_OK)
    {
        return MPU6050_STATUS_I2C_ERROR;
    }

    data->accel_x = mpu6050_be_to_i16(&buffer[0]);
    data->accel_y = mpu6050_be_to_i16(&buffer[2]);
    data->accel_z = mpu6050_be_to_i16(&buffer[4]);
    data->temperature = mpu6050_be_to_i16(&buffer[6]);
    data->gyro_x = mpu6050_be_to_i16(&buffer[8]);
    data->gyro_y = mpu6050_be_to_i16(&buffer[10]);
    data->gyro_z = mpu6050_be_to_i16(&buffer[12]);

    return MPU6050_STATUS_OK;
}

mpu6050_status_t mpu6050_read(
    const mpu6050_t *device,
    mpu6050_data_t *data)
{
    if ((device == NULL) || (data == NULL) ||
        (device->accel_lsb_per_g <= 0.0f) ||
        (device->gyro_lsb_per_dps <= 0.0f))
    {
        return MPU6050_STATUS_INVALID_ARGUMENT;
    }

    mpu6050_raw_data_t raw;
    const mpu6050_status_t status = mpu6050_read_raw(device, &raw);
    if (status != MPU6050_STATUS_OK)
    {
        return status;
    }

    data->accel_x_g = (float)raw.accel_x / device->accel_lsb_per_g;
    data->accel_y_g = (float)raw.accel_y / device->accel_lsb_per_g;
    data->accel_z_g = (float)raw.accel_z / device->accel_lsb_per_g;

    data->temperature_c = ((float)raw.temperature / 340.0f) + 36.53f;

    data->gyro_x_dps = (float)raw.gyro_x / device->gyro_lsb_per_dps;
    data->gyro_y_dps = (float)raw.gyro_y / device->gyro_lsb_per_dps;
    data->gyro_z_dps = (float)raw.gyro_z / device->gyro_lsb_per_dps;

    return MPU6050_STATUS_OK;
}

const char *mpu6050_status_string(mpu6050_status_t status)
{
    switch (status)
    {
        case MPU6050_STATUS_OK:               return "OK";
        case MPU6050_STATUS_INVALID_ARGUMENT: return "invalid argument";
        case MPU6050_STATUS_I2C_ERROR:         return "I2C error";
        case MPU6050_STATUS_DEVICE_NOT_FOUND:  return "device not found";
        case MPU6050_STATUS_BAD_DEVICE_ID:     return "bad WHO_AM_I";
        default:                              return "unknown error";
    }
}
