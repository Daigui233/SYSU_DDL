#include "ti_msp_dl_config.h"
#include "mpu6050.h"

/* Oscilloscope test: 80 MHz / 40 / 2500 = 800 Hz, 500 / 2500 = 20%. */
#define PWM_TEST_PERIOD_COUNTS 2500U
#define PWM_TEST_DUTY_COUNTS    500U

/* Watch these variables in Keil to verify the sensor and inspect live data. */
volatile bool g_mpu6050_ready;
volatile bool g_mpu6050_last_read_ok;
volatile MPU6050_RawData g_mpu6050_raw;

static void emergency_stop(void)
{
    DL_TimerA_stopCounter(carPWM_INST);
    SYSCFG_DL_motorOutputsLow();
    while (1) __WFI();
}

void NMI_Handler(void)       { emergency_stop(); }
void HardFault_Handler(void) { emergency_stop(); }
void SVC_Handler(void)       { emergency_stop(); }
void PendSV_Handler(void)    { emergency_stop(); }
void SysTick_Handler(void)   { emergency_stop(); }

static void pwm_test_init(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(carPWM_INST);
    DL_I2C_reset(MPU_I2C_INST);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(carPWM_INST);
    DL_I2C_enablePower(MPU_I2C_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    /* Clamp every connected driver input low before configuring clocks. */
    SYSCFG_DL_motorOutputsLow();

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXOUT_IOMUX);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU_I2C_IOMUX_SDA,
        GPIO_MPU_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU_I2C_IOMUX_SCL,
        GPIO_MPU_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MPU_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MPU_I2C_IOMUX_SCL);
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_SYSCTL_CLK_init();
    SYSCFG_DL_carPWM_init();
    SYSCFG_DL_MPU_I2C_init();

    DL_TimerA_setLoadValue(carPWM_INST, PWM_TEST_PERIOD_COUNTS - 1U);
    DL_TimerA_setCaptureCompareValue(
        carPWM_INST, PWM_TEST_DUTY_COUNTS, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(
        carPWM_INST, PWM_TEST_DUTY_COUNTS, DL_TIMER_CC_1_INDEX);

    /* Scope PA0 and PA1. Opposite AT8236 inputs PA22 and PB13 remain low. */
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_carPWM_C0_IOMUX, GPIO_carPWM_C0_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_carPWM_C1_IOMUX, GPIO_carPWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_carPWM_C0_PORT, GPIO_carPWM_C0_PIN);
    DL_GPIO_enableOutput(GPIO_carPWM_C1_PORT, GPIO_carPWM_C1_PIN);
    DL_TimerA_startCounter(carPWM_INST);
}

int main(void)
{
    MPU6050_RawData sample;

    pwm_test_init();
    g_mpu6050_ready = MPU6050_Init();
    g_mpu6050_last_read_ok = false;

    while (1) {
        if (g_mpu6050_ready) {
            g_mpu6050_last_read_ok = MPU6050_ReadRaw(&sample);
            if (g_mpu6050_last_read_ok) g_mpu6050_raw = sample;
        }
        delay_cycles(CPUCLK_FREQ / 100U);
    }
}
