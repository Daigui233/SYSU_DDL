#include "mpu6050.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define MPU6050_REG_SMPLRT_DIV   (0x19U)
#define MPU6050_REG_CONFIG       (0x1AU)
#define MPU6050_REG_GYRO_CONFIG  (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG (0x1CU)
#define MPU6050_REG_ACCEL_XOUT_H (0x3BU)
#define MPU6050_REG_PWR_MGMT_1   (0x6BU)
#define MPU6050_REG_PWR_MGMT_2   (0x6CU)
#define MPU6050_REG_WHO_AM_I     (0x75U)

#define MPU6050_I2C_TIMEOUT_LOOPS (800000U)
#define MPU6050_RESET_DELAY_CYCLES (CPUCLK_FREQ / 10U)

static bool i2c_wait_idle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0U) {
        uint32_t status = DL_I2C_getControllerStatus(MPU_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) return false;
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) return true;
    }
    return false;
}

static bool i2c_wait_transfer_done(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0U) {
        uint32_t status = DL_I2C_getControllerStatus(MPU_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) return false;
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U)
            return i2c_wait_idle();
    }
    return false;
}

static void i2c_abort_transfer(void)
{
    DL_I2C_resetControllerTransfer(MPU_I2C_INST);
    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);
}

static bool i2c_write_register(uint8_t reg, uint8_t value)
{
    uint8_t packet[2];

    if (!i2c_wait_idle()) {
        i2c_abort_transfer();
        return false;
    }

    packet[0] = reg;
    packet[1] = value;
    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
    if (DL_I2C_fillControllerTXFIFO(MPU_I2C_INST, packet, 2U) != 2U)
        return false;

    DL_I2C_startControllerTransfer(MPU_I2C_INST, MPU6050_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2U);
    if (!i2c_wait_transfer_done()) {
        i2c_abort_transfer();
        return false;
    }
    return true;
}

static bool i2c_read_registers(uint8_t reg, uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint32_t timeout;

    if ((data == NULL) || (length == 0U)) return false;
    if (!i2c_wait_idle()) {
        i2c_abort_transfer();
        return false;
    }

    DL_I2C_flushControllerTXFIFO(MPU_I2C_INST);
    if (DL_I2C_fillControllerTXFIFO(MPU_I2C_INST, &reg, 1U) != 1U)
        return false;
    DL_I2C_startControllerTransfer(MPU_I2C_INST, MPU6050_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    if (!i2c_wait_transfer_done()) {
        i2c_abort_transfer();
        return false;
    }

    DL_I2C_flushControllerRXFIFO(MPU_I2C_INST);
    DL_I2C_startControllerTransfer(MPU_I2C_INST, MPU6050_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);

    for (index = 0U; index < length; index++) {
        timeout = MPU6050_I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(MPU_I2C_INST)) {
            if ((DL_I2C_getControllerStatus(MPU_I2C_INST) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                i2c_abort_transfer();
                return false;
            }
            if (timeout-- == 0U) {
                i2c_abort_transfer();
                return false;
            }
        }
        data[index] = DL_I2C_receiveControllerData(MPU_I2C_INST);
    }

    if (!i2c_wait_transfer_done()) {
        i2c_abort_transfer();
        return false;
    }
    return true;
}

static int16_t be16(const uint8_t *bytes)
{
    return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

bool MPU6050_ReadWhoAmI(uint8_t *who_am_i)
{
    return i2c_read_registers(MPU6050_REG_WHO_AM_I, who_am_i, 1U);
}

bool MPU6050_Init(void)
{
    uint8_t who_am_i = 0U;

    if (!MPU6050_ReadWhoAmI(&who_am_i) ||
        (who_am_i != MPU6050_WHO_AM_I_VALUE)) return false;

    if (!i2c_write_register(MPU6050_REG_PWR_MGMT_1, 0x80U)) return false;
    delay_cycles(MPU6050_RESET_DELAY_CYCLES);
    if (!i2c_write_register(MPU6050_REG_PWR_MGMT_1, 0x01U)) return false;
    if (!i2c_write_register(MPU6050_REG_PWR_MGMT_2, 0x00U)) return false;
    if (!i2c_write_register(MPU6050_REG_SMPLRT_DIV, 0x04U)) return false;
    if (!i2c_write_register(MPU6050_REG_CONFIG, 0x03U)) return false;
    if (!i2c_write_register(MPU6050_REG_GYRO_CONFIG, 0x00U)) return false;
    if (!i2c_write_register(MPU6050_REG_ACCEL_CONFIG, 0x00U)) return false;
    return true;
}

bool MPU6050_ReadRaw(MPU6050_RawData *data)
{
    uint8_t raw[14];

    if ((data == NULL) ||
        !i2c_read_registers(MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw)))
        return false;

    data->accel_x = be16(&raw[0]);
    data->accel_y = be16(&raw[2]);
    data->accel_z = be16(&raw[4]);
    data->temperature = be16(&raw[6]);
    data->gyro_x = be16(&raw[8]);
    data->gyro_y = be16(&raw[10]);
    data->gyro_z = be16(&raw[12]);
    return true;
}

float MPU6050_AccelG(int16_t raw)
{
    return (float)raw / 16384.0f;
}

float MPU6050_GyroDps(int16_t raw)
{
    return (float)raw / 131.0f;
}

float MPU6050_TemperatureC(int16_t raw)
{
    return ((float)raw / 340.0f) + 36.53f;
}
