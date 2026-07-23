#include "board_port.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

#include <string.h>

#define CONTROL_PERIOD_MS          (1000U / APP_CONTROL_HZ)
#define MOTOR_MAIN_PERIOD          2500U
#define MOTOR_LEFT_REVERSE_PERIOD 12500U
#define MOTOR_CHANNEL_COUNT            2U

typedef struct {
    int16_t applied_pwm;
    uint8_t reverse_coast_ticks;
} MotorSafetyState;

static volatile uint32_t g_millis;
static volatile uint8_t g_control_ticks;
static volatile int32_t g_encoder_left;
static volatile int32_t g_encoder_right;
static bool g_key_latched[3];
static uint8_t g_key_debounce[3];
static MotorSafetyState g_motor_safety[MOTOR_CHANNEL_COUNT];
static volatile bool g_motor_outputs_enabled;

static uint32_t duty_counts(int16_t pwm, uint32_t period)
{
    uint32_t magnitude = (pwm < 0) ? (uint32_t)(-pwm) : (uint32_t)pwm;
    if (magnitude > 1000U) magnitude = 1000U;
    return magnitude * period / 1000U;
}

static void motor_pin_force_low(uint32_t iomux, GPIO_Regs *port, uint32_t pin)
{
    /* Clear the GPIO latch before taking the pin away from the timer. */
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_initDigitalOutput(iomux);
    DL_GPIO_enableOutput(port, pin);
}

static void motor_pin_select_pwm(uint32_t iomux, uint32_t function,
                                 GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_initPeripheralOutputFunction(iomux, function);
    DL_GPIO_enableOutput(port, pin);
}

static void motor_apply_raw(uint8_t channel, int16_t pwm)
{
    uint32_t main_duty = duty_counts(pwm, MOTOR_MAIN_PERIOD);
    uint32_t aux_duty;

    switch (channel) {
    case 0U: /* Left: AIN1=PA1, AIN2=PB13. */
        aux_duty = duty_counts(pwm, MOTOR_LEFT_REVERSE_PERIOD);
        if (pwm > 0) {
            motor_pin_force_low(GPIO_carPWM_C1_IOMUX,
                                GPIO_carPWM_C1_PORT, GPIO_carPWM_C1_PIN);
            DL_TimerA_setCaptureCompareValue(carPWM_INST, 0U, DL_TIMER_CC_1_INDEX);
            DL_TimerG_setCaptureCompareValue(car2N_INST, aux_duty, DL_TIMER_CC_0_INDEX);
            motor_pin_select_pwm(GPIO_car2N_C0_IOMUX, GPIO_car2N_C0_IOMUX_FUNC,
                                 GPIO_car2N_C0_PORT, GPIO_car2N_C0_PIN);
        } else if (pwm < 0) {
            motor_pin_force_low(GPIO_car2N_C0_IOMUX,
                                GPIO_car2N_C0_PORT, GPIO_car2N_C0_PIN);
            DL_TimerG_setCaptureCompareValue(car2N_INST, 0U, DL_TIMER_CC_0_INDEX);
            DL_TimerA_setCaptureCompareValue(carPWM_INST, main_duty, DL_TIMER_CC_1_INDEX);
            motor_pin_select_pwm(GPIO_carPWM_C1_IOMUX, GPIO_carPWM_C1_IOMUX_FUNC,
                                 GPIO_carPWM_C1_PORT, GPIO_carPWM_C1_PIN);
        } else {
            motor_pin_force_low(GPIO_carPWM_C1_IOMUX,
                                GPIO_carPWM_C1_PORT, GPIO_carPWM_C1_PIN);
            motor_pin_force_low(GPIO_car2N_C0_IOMUX,
                                GPIO_car2N_C0_PORT, GPIO_car2N_C0_PIN);
            DL_TimerA_setCaptureCompareValue(carPWM_INST, 0U, DL_TIMER_CC_1_INDEX);
            DL_TimerG_setCaptureCompareValue(car2N_INST, 0U, DL_TIMER_CC_0_INDEX);
        }
        break;
    case 1U: /* Right: BIN1=PA0, BIN2=PA22. */
        if (pwm > 0) {
            motor_pin_force_low(GPIO_car1N_C1_IOMUX,
                                GPIO_car1N_C1_PORT, GPIO_car1N_C1_PIN);
            DL_TimerG_setCaptureCompareValue(car1N_INST, 0U, DL_TIMER_CC_1_INDEX);
            DL_TimerA_setCaptureCompareValue(carPWM_INST, main_duty, DL_TIMER_CC_0_INDEX);
            motor_pin_select_pwm(GPIO_carPWM_C0_IOMUX, GPIO_carPWM_C0_IOMUX_FUNC,
                                 GPIO_carPWM_C0_PORT, GPIO_carPWM_C0_PIN);
        } else if (pwm < 0) {
            motor_pin_force_low(GPIO_carPWM_C0_IOMUX,
                                GPIO_carPWM_C0_PORT, GPIO_carPWM_C0_PIN);
            DL_TimerA_setCaptureCompareValue(carPWM_INST, 0U, DL_TIMER_CC_0_INDEX);
            DL_TimerG_setCaptureCompareValue(car1N_INST, main_duty, DL_TIMER_CC_1_INDEX);
            motor_pin_select_pwm(GPIO_car1N_C1_IOMUX, GPIO_car1N_C1_IOMUX_FUNC,
                                 GPIO_car1N_C1_PORT, GPIO_car1N_C1_PIN);
        } else {
            motor_pin_force_low(GPIO_carPWM_C0_IOMUX,
                                GPIO_carPWM_C0_PORT, GPIO_carPWM_C0_PIN);
            motor_pin_force_low(GPIO_car1N_C1_IOMUX,
                                GPIO_car1N_C1_PORT, GPIO_car1N_C1_PIN);
            DL_TimerA_setCaptureCompareValue(carPWM_INST, 0U, DL_TIMER_CC_0_INDEX);
            DL_TimerG_setCaptureCompareValue(car1N_INST, 0U, DL_TIMER_CC_1_INDEX);
        }
        break;
    default:
        break;
    }
}

static void motor_gpio_force_low(void)
{
    SYSCFG_DL_motorOutputsLow();
}

void board_init(void)
{
    g_millis = 0U;
    g_control_ticks = 0U;
    g_encoder_left = 0;
    g_encoder_right = 0;
    memset(g_key_latched, 0, sizeof(g_key_latched));
    memset(g_key_debounce, 0, sizeof(g_key_debounce));
    memset(g_motor_safety, 0, sizeof(g_motor_safety));
    g_motor_outputs_enabled = false;

    /* The imported SYSConfig timer was 15 ms. Use exactly 10 ms for 100 Hz. */
    DL_TimerG_setLoadValue(TIMER_0_INST, 999U);

    /* Motor timers are generated stopped. Zero every input before starting. */
    motor_apply_raw(0U, 0);
    motor_apply_raw(1U, 0);
    DL_TimerA_startCounter(carPWM_INST);
    DL_TimerG_startCounter(car1N_INST);
    DL_TimerG_startCounter(car2N_INST);
    g_motor_outputs_enabled = true;

    DL_ADC12_startConversion(ADC_INST);
    DL_ADC12_startConversion(ccdADC_INST);

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ESP_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(ESP_INST_INT_IRQN);
}

void board_poll(void)
{
    /* Reserved for non-interrupt peripherals (OLED, IMU or PS2 extensions). */
}

uint32_t board_millis(void)
{
    return g_millis;
}

bool board_take_control_tick(void)
{
    bool available;
    __disable_irq();
    available = (g_control_ticks != 0U);
    if (available) g_control_ticks--;
    __enable_irq();
    return available;
}

void board_motor_set(uint8_t channel, int16_t pwm)
{
    MotorSafetyState *state;
    int32_t delta;

    if ((channel >= MOTOR_CHANNEL_COUNT) || !g_motor_outputs_enabled) return;
    if (pwm > APP_PWM_LIMIT_PERMILLE) pwm = APP_PWM_LIMIT_PERMILLE;
    if (pwm < -APP_PWM_LIMIT_PERMILLE) pwm = -APP_PWM_LIMIT_PERMILLE;
    state = &g_motor_safety[channel];

    if (pwm == 0) {
        state->applied_pwm = 0;
        state->reverse_coast_ticks = 0U;
        motor_apply_raw(channel, 0);
        return;
    }

    if (state->reverse_coast_ticks != 0U) {
        state->reverse_coast_ticks--;
        state->applied_pwm = 0;
        motor_apply_raw(channel, 0);
        return;
    }

    if ((state->applied_pwm != 0) &&
        ((state->applied_pwm < 0) != (pwm < 0))) {
        state->applied_pwm = 0;
        state->reverse_coast_ticks = APP_MOTOR_REVERSE_COAST_TICKS;
        motor_apply_raw(channel, 0);
        return;
    }

    delta = (int32_t)pwm - state->applied_pwm;
    if (delta > APP_MOTOR_PWM_SLEW_PER_TICK)
        delta = APP_MOTOR_PWM_SLEW_PER_TICK;
    else if (delta < -APP_MOTOR_PWM_SLEW_PER_TICK)
        delta = -APP_MOTOR_PWM_SLEW_PER_TICK;
    state->applied_pwm = (int16_t)(state->applied_pwm + delta);
    motor_apply_raw(channel, state->applied_pwm);
}

void board_all_motors_stop(void)
{
    uint8_t channel;
    for (channel = 0U; channel < MOTOR_CHANNEL_COUNT; channel++) {
        g_motor_safety[channel].applied_pwm = 0;
        g_motor_safety[channel].reverse_coast_ticks = 0U;
        motor_apply_raw(channel, 0);
    }
}

void board_motor_fault_shutdown(void)
{
    __disable_irq();
    if (g_motor_outputs_enabled) {
        motor_apply_raw(0U, 0);
        motor_apply_raw(1U, 0);
        motor_gpio_force_low();
        DL_TimerA_stopCounter(carPWM_INST);
        DL_TimerG_stopCounter(car1N_INST);
        DL_TimerG_stopCounter(car2N_INST);
        g_motor_outputs_enabled = false;
    }
}

void board_encoder_snapshot(int32_t *encoder_a, int32_t *encoder_b)
{
    __disable_irq();
    *encoder_a = g_encoder_left;
    *encoder_b = g_encoder_right;
    g_encoder_left = 0;
    g_encoder_right = 0;
    __enable_irq();
}

void board_line_adc_read(uint16_t values[6])
{
    values[0] = DL_ADC12_getMemResult(ADC_INST, ADC_ADCMEM_0);
    values[1] = DL_ADC12_getMemResult(ADC_INST, ADC_ADCMEM_1);
    values[2] = DL_ADC12_getMemResult(ADC_INST, ADC_ADCMEM_2);
    values[3] = DL_ADC12_getMemResult(ADC_INST, ADC_ADCMEM_3);
    values[4] = DL_ADC12_getMemResult(ADC_INST, ADC_ADCMEM_4);
    values[5] = DL_ADC12_getMemResult(ADC_INST, ADC_ADCMEM_5);
    DL_ADC12_startConversion(ADC_INST);
}

uint16_t board_aux_adc_read(void)
{
    uint16_t value = DL_ADC12_getMemResult(ccdADC_INST, ccdADC_ADCMEM_0);
    DL_ADC12_startConversion(ccdADC_INST);
    return value;
}

bool board_key_pressed(uint8_t key_index)
{
    const uint32_t pins[3] = {KEY_KEY1_PIN, KEY_KEY2_PIN, KEY_KEY3_PIN};
    bool down;
    if (key_index >= 3U) return false;
    down = (DL_GPIO_readPins(KEY_PORT, pins[key_index]) == 0U);
    if (down) {
        if (g_key_debounce[key_index] < 3U) g_key_debounce[key_index]++;
        if ((g_key_debounce[key_index] >= 2U) && !g_key_latched[key_index]) {
            g_key_latched[key_index] = true;
            return true;
        }
    } else {
        g_key_debounce[key_index] = 0U;
        g_key_latched[key_index] = false;
    }
    return false;
}

void board_led_set(bool on)
{
    if (on) DL_GPIO_setPins(myShow_PORT, myShow_LED_PIN);
    else DL_GPIO_clearPins(myShow_PORT, myShow_LED_PIN);
}

void board_buzzer_set(bool on)
{
    if (on) DL_GPIO_setPins(myShow_PORT, myShow_Buzzer_PIN);
    else DL_GPIO_clearPins(myShow_PORT, myShow_Buzzer_PIN);
}

bool board_uart_send_byte(uint8_t value)
{
    uint32_t timeout = 80000U;
    while (DL_UART_isBusy(ESP_INST)) {
        if (--timeout == 0U) return false;
    }
    DL_UART_Main_transmitData(ESP_INST, value);
    return true;
}

void board_uart_send(const uint8_t *data, size_t length)
{
    size_t index;
    for (index = 0U; index < length; index++) {
        if (!board_uart_send_byte(data[index])) break;
    }
}

void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMERG_IIDX_ZERO) {
        g_millis += CONTROL_PERIOD_MS;
        if (g_control_ticks < 10U) g_control_ticks++;
    }
}

void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(ESP_INST)) {
    case DL_UART_IIDX_RX:
        protocol_rx_byte_isr((uint8_t)DL_UART_Main_receiveData(ESP_INST));
        break;
    default:
        break;
    }
}

static void encoder_left_isr(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        encoder2_PORT, encoder2_leftA_PIN | encoder2_leftB_PIN);
    if ((status & encoder2_leftA_PIN) != 0U) {
        if (DL_GPIO_readPins(encoder2_PORT, encoder2_leftB_PIN) == 0U)
            g_encoder_left++;
        else
            g_encoder_left--;
    }
    if ((status & encoder2_leftB_PIN) != 0U) {
        if (DL_GPIO_readPins(encoder2_PORT, encoder2_leftA_PIN) == 0U)
            g_encoder_left--;
        else
            g_encoder_left++;
    }
    DL_GPIO_clearInterruptStatus(encoder2_PORT, status);
}

static void encoder_right_isr(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        encoder_PORT, encoder_RightA_PIN | encoder_RightB_PIN);
    if ((status & encoder_RightA_PIN) != 0U) {
        if (DL_GPIO_readPins(encoder_PORT, encoder_RightB_PIN) == 0U)
            g_encoder_right++;
        else
            g_encoder_right--;
    }
    if ((status & encoder_RightB_PIN) != 0U) {
        if (DL_GPIO_readPins(encoder_PORT, encoder_RightA_PIN) == 0U)
            g_encoder_right--;
        else
            g_encoder_right++;
    }
    DL_GPIO_clearInterruptStatus(encoder_PORT, status);
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
    case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
        encoder_left_isr();
        break;
    case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        encoder_right_isr();
        break;
    default:
        break;
    }
}
