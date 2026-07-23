#include "attitude.h"

#include "mpu6050.h"
#include "ti_msp_dl_config.h"

#include <math.h>
#include <stdint.h>

#define RAD_TO_DEG             57.2957795f
#define COMPLEMENTARY_GYRO       0.98f
#define GYRO_CALIBRATION_SAMPLES 100U

static Attitude g_attitude;
static float g_gyro_bias_x;
static float g_gyro_bias_y;
static float g_gyro_bias_z;

static void accel_angles(const MPU6050_RawData *raw,
                         float *roll_deg, float *pitch_deg)
{
    float ax = MPU6050_AccelG(raw->accel_x);
    float ay = MPU6050_AccelG(raw->accel_y);
    float az = MPU6050_AccelG(raw->accel_z);

    *roll_deg = atan2f(ay, az) * RAD_TO_DEG;
    *pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

bool attitude_init(void)
{
    MPU6050_RawData raw;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    uint16_t count;

    g_attitude.roll_deg = 0.0f;
    g_attitude.pitch_deg = 0.0f;
    g_attitude.yaw_deg = 0.0f;
    g_attitude.valid = false;
    if (!MPU6050_Init()) return false;

    /*
     * Keep the car still during this short startup calibration. One sample
     * per millisecond estimates the three gyro zero offsets.
     */
    for (count = 0U; count < GYRO_CALIBRATION_SAMPLES; count++) {
        if (!MPU6050_ReadRaw(&raw)) return false;
        sum_x += raw.gyro_x;
        sum_y += raw.gyro_y;
        sum_z += raw.gyro_z;
        delay_cycles(CPUCLK_FREQ / 1000U);
    }
    g_gyro_bias_x = MPU6050_GyroDps(
        (int16_t)(sum_x / (int32_t)GYRO_CALIBRATION_SAMPLES));
    g_gyro_bias_y = MPU6050_GyroDps(
        (int16_t)(sum_y / (int32_t)GYRO_CALIBRATION_SAMPLES));
    g_gyro_bias_z = MPU6050_GyroDps(
        (int16_t)(sum_z / (int32_t)GYRO_CALIBRATION_SAMPLES));
    accel_angles(&raw, &g_attitude.roll_deg, &g_attitude.pitch_deg);
    g_attitude.valid = true;
    return true;
}

void attitude_update(float dt_seconds)
{
    MPU6050_RawData raw;
    float accel_roll;
    float accel_pitch;
    float gyro_roll;
    float gyro_pitch;
    float gyro_yaw;

    if (!g_attitude.valid || !MPU6050_ReadRaw(&raw)) {
        g_attitude.valid = false;
        return;
    }
    accel_angles(&raw, &accel_roll, &accel_pitch);
    gyro_roll = MPU6050_GyroDps(raw.gyro_x) - g_gyro_bias_x;
    gyro_pitch = MPU6050_GyroDps(raw.gyro_y) - g_gyro_bias_y;
    gyro_yaw = MPU6050_GyroDps(raw.gyro_z) - g_gyro_bias_z;

    g_attitude.roll_deg =
        COMPLEMENTARY_GYRO * (g_attitude.roll_deg + gyro_roll * dt_seconds) +
        (1.0f - COMPLEMENTARY_GYRO) * accel_roll;
    g_attitude.pitch_deg =
        COMPLEMENTARY_GYRO * (g_attitude.pitch_deg + gyro_pitch * dt_seconds) +
        (1.0f - COMPLEMENTARY_GYRO) * accel_pitch;
    g_attitude.yaw_deg += gyro_yaw * dt_seconds;
    if (g_attitude.yaw_deg > 180.0f) g_attitude.yaw_deg -= 360.0f;
    if (g_attitude.yaw_deg < -180.0f) g_attitude.yaw_deg += 360.0f;
}

const Attitude *attitude_get(void)
{
    return &g_attitude;
}
