#ifndef MLX90614_H
#define MLX90614_H

#include <stdint.h>
#include "xnx_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MLX90614_DEFAULT_ADDRESS      0x5AU

#define MLX90614_RAM_TA               0x06U
#define MLX90614_RAM_TOBJ1            0x07U
#define MLX90614_EEPROM_ID1           0x3CU

typedef enum
{
    MLX90614_STATUS_OK = 0,
    MLX90614_STATUS_INVALID_ARGUMENT,
    MLX90614_STATUS_NOT_FOUND,
    MLX90614_STATUS_I2C_ERROR,
    MLX90614_STATUS_PEC_ERROR,
    MLX90614_STATUS_SENSOR_ERROR,
    MLX90614_STATUS_BAD_ID
} mlx90614_status_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint16_t id_word;
    uint8_t initialized;
} mlx90614_t;

typedef struct
{
    uint16_t ambient_raw;
    uint16_t object_raw;

    /* Temperature in hundredths of a degree Celsius.
       Example: 2537 means 25.37 °C. */
    int32_t ambient_centi_c;
    int32_t object_centi_c;
} mlx90614_data_t;

mlx90614_status_t mlx90614_init(
    mlx90614_t *device,
    I2C_HandleTypeDef *hi2c,
    uint8_t address);

mlx90614_status_t mlx90614_read(
    mlx90614_t *device,
    mlx90614_data_t *data);

mlx90614_status_t mlx90614_read_ambient(
    mlx90614_t *device,
    int32_t *temperature_centi_c);

mlx90614_status_t mlx90614_read_object(
    mlx90614_t *device,
    int32_t *temperature_centi_c);

const char *mlx90614_status_string(mlx90614_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* MLX90614_H */
