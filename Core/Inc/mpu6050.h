#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "xnx_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_ADDRESS_AD0_LOW          0x68U
#define MPU6050_ADDRESS_AD0_HIGH         0x69U
#define MPU6050_WHO_AM_I_VALUE           0x68U

typedef enum
{
    MPU6050_STATUS_OK = 0,
    MPU6050_STATUS_INVALID_ARGUMENT,
    MPU6050_STATUS_I2C_ERROR,
    MPU6050_STATUS_DEVICE_NOT_FOUND,
    MPU6050_STATUS_BAD_DEVICE_ID
} mpu6050_status_t;

typedef enum
{
    MPU6050_ACCEL_RANGE_2G = 0,
    MPU6050_ACCEL_RANGE_4G = 1,
    MPU6050_ACCEL_RANGE_8G = 2,
    MPU6050_ACCEL_RANGE_16G = 3
} mpu6050_accel_range_t;

typedef enum
{
    MPU6050_GYRO_RANGE_250_DPS = 0,
    MPU6050_GYRO_RANGE_500_DPS = 1,
    MPU6050_GYRO_RANGE_1000_DPS = 2,
    MPU6050_GYRO_RANGE_2000_DPS = 3
} mpu6050_gyro_range_t;

typedef enum
{
    MPU6050_DLPF_260_HZ = 0,
    MPU6050_DLPF_184_HZ = 1,
    MPU6050_DLPF_94_HZ = 2,
    MPU6050_DLPF_44_HZ = 3,
    MPU6050_DLPF_21_HZ = 4,
    MPU6050_DLPF_10_HZ = 5,
    MPU6050_DLPF_5_HZ = 6
} mpu6050_dlpf_t;

typedef struct
{
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t gyro_range;
    mpu6050_dlpf_t dlpf;
    uint8_t sample_rate_divider;
} mpu6050_config_t;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_data_t;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} mpu6050_data_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
    mpu6050_config_t config;
} mpu6050_t;

void mpu6050_get_default_config(mpu6050_config_t *config);

mpu6050_status_t mpu6050_init(
    mpu6050_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address,
    const mpu6050_config_t *config);

mpu6050_status_t mpu6050_read_who_am_i(
    const mpu6050_t *device,
    uint8_t *who_am_i);

mpu6050_status_t mpu6050_set_accel_range(
    mpu6050_t *device,
    mpu6050_accel_range_t range);

mpu6050_status_t mpu6050_set_gyro_range(
    mpu6050_t *device,
    mpu6050_gyro_range_t range);

mpu6050_status_t mpu6050_set_dlpf(
    mpu6050_t *device,
    mpu6050_dlpf_t dlpf);

mpu6050_status_t mpu6050_set_sample_rate_divider(
    mpu6050_t *device,
    uint8_t divider);

mpu6050_status_t mpu6050_read_raw(
    const mpu6050_t *device,
    mpu6050_raw_data_t *data);

mpu6050_status_t mpu6050_read(
    const mpu6050_t *device,
    mpu6050_data_t *data);

const char *mpu6050_status_string(mpu6050_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
