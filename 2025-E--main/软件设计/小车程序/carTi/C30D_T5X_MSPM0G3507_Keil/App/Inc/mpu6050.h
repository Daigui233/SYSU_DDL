#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_I2C_ADDRESS  (0x68U)
#define MPU6050_WHO_AM_I_VALUE (0x68U)

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_RawData;

/* Call after PA28/PA31 and MPU_I2C have been initialized. */
bool MPU6050_Init(void);
bool MPU6050_ReadWhoAmI(uint8_t *who_am_i);
bool MPU6050_ReadRaw(MPU6050_RawData *data);

/* Conversion factors for the default +/-2 g and +/-250 degree/s ranges. */
float MPU6050_AccelG(int16_t raw);
float MPU6050_GyroDps(int16_t raw);
float MPU6050_TemperatureC(int16_t raw);

#endif
