#include "car_control.h"

#include "app_config.h"
#include "board_port.h"

#include <string.h>

#define PI_F                 3.14159265358979323846f
#define REFERENCE_PWM_FULL   16799.0f
#define TARGET_LIMIT_MPS     0.5f

CarControl g_car;
static uint8_t g_stall_ticks[2];
static uint8_t g_reverse_ticks[2];

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int incremental_pi(uint8_t index)
{
    MotorParameter *motor = &g_car.motor[index];
    float error = motor->target_mps - motor->encoder_mps;
    float pwm_limit = REFERENCE_PWM_FULL *
                      (float)APP_PWM_LIMIT_PERMILLE / 1000.0f;

    motor->pwm += motor->kp * (error - motor->previous_error) +
                  motor->ki * error;
    motor->previous_error = error;
    motor->pwm = clampf(motor->pwm, -pwm_limit, pwm_limit);
    return (int)motor->pwm;
}

static void latch_motor_fault(uint32_t fault)
{
    uint8_t index;
    g_car.fault_flags |= fault;
    g_car.enabled = false;
    g_car.move_x_mps = 0.0f;
    g_car.move_z_radps = 0.0f;
    for (index = 0U; index < 2U; index++) {
        g_car.motor[index].pwm = 0.0f;
        g_car.motor[index].previous_error = 0.0f;
        g_car.motor[index].target_mps = 0.0f;
    }
    board_motor_fault_shutdown();
}

static bool motor_feedback_is_safe(uint8_t index, int pwm)
{
    float target = g_car.motor[index].target_mps;
    float encoder = g_car.motor[index].encoder_mps;
    uint32_t stall_fault = (index == 0U) ? CAR_FAULT_LEFT_STALL :
                                           CAR_FAULT_RIGHT_STALL;
    uint32_t direction_fault = (index == 0U) ? CAR_FAULT_LEFT_DIRECTION :
                                               CAR_FAULT_RIGHT_DIRECTION;

    if ((absf(target) >= APP_MOTOR_STALL_TARGET_MPS) &&
        ((pwm >= APP_MOTOR_STALL_PWM_COUNTS) ||
         (pwm <= -APP_MOTOR_STALL_PWM_COUNTS)) &&
        (absf(encoder) < APP_MOTOR_STALL_ENCODER_MPS)) {
        if (g_stall_ticks[index] < 255U) g_stall_ticks[index]++;
    } else {
        g_stall_ticks[index] = 0U;
    }

    if ((absf(target) >= APP_MOTOR_STALL_TARGET_MPS) &&
        (absf(encoder) >= APP_MOTOR_STALL_ENCODER_MPS) &&
        ((target < 0.0f) != (encoder < 0.0f))) {
        if (g_reverse_ticks[index] < 255U) g_reverse_ticks[index]++;
    } else {
        g_reverse_ticks[index] = 0U;
    }

    if (g_stall_ticks[index] >= APP_MOTOR_STALL_TICKS) {
        latch_motor_fault(stall_fault);
        return false;
    }
    if (g_reverse_ticks[index] >= APP_MOTOR_REVERSE_FAULT_TICKS) {
        latch_motor_fault(direction_fault);
        return false;
    }
    return true;
}

void car_control_init(void)
{
    uint8_t index;
    memset(&g_car, 0, sizeof(g_car));
    g_car.wheel_spacing_m = APP_WHEEL_SPACING_M;
    g_car.wheel_perimeter_m = APP_WHEEL_DIAMETER_M * PI_F;
    g_car.encoder_counts_per_rev = APP_ENCODER_COUNTS_PER_REV;
    for (index = 0U; index < 2U; index++) {
        g_car.motor[index].kp = APP_DEFAULT_VELOCITY_KP;
        g_car.motor[index].ki = APP_DEFAULT_VELOCITY_KI;
    }
    car_control_reset_pi();
}

void car_control_set_motion(float vx_mps, float wz_radps)
{
    float half_spacing;
    if (g_car.fault_flags != 0U) return;

    vx_mps = clampf(vx_mps, -TARGET_LIMIT_MPS, TARGET_LIMIT_MPS);
    half_spacing = 0.5f * g_car.wheel_spacing_m;
    g_car.move_x_mps = vx_mps;
    g_car.move_z_radps = wz_radps;
    g_car.motor[0].target_mps = clampf(
        vx_mps - wz_radps * half_spacing, -TARGET_LIMIT_MPS, TARGET_LIMIT_MPS);
    g_car.motor[1].target_mps = clampf(
        vx_mps + wz_radps * half_spacing, -TARGET_LIMIT_MPS, TARGET_LIMIT_MPS);
    g_car.enabled = true;
}

void car_control_set_diff_targets(float left_mps, float right_mps)
{
    if (g_car.fault_flags != 0U) return;
    g_car.motor[0].target_mps = clampf(
        left_mps, -TARGET_LIMIT_MPS, TARGET_LIMIT_MPS);
    g_car.motor[1].target_mps = clampf(
        right_mps, -TARGET_LIMIT_MPS, TARGET_LIMIT_MPS);
    g_car.enabled = true;
}

void car_control_update_encoders(int32_t encoder_a, int32_t encoder_b)
{
    float scale = APP_CONTROL_HZ * g_car.wheel_perimeter_m /
                  g_car.encoder_counts_per_rev;
    g_car.motor[0].encoder_mps = (float)encoder_a * scale;
    g_car.motor[1].encoder_mps = (float)encoder_b * scale;
}

void car_control_step(void)
{
    int pwm_left;
    int pwm_right;
    int permille_left;
    int permille_right;

    if (g_car.fault_flags != 0U) {
        board_motor_fault_shutdown();
        return;
    }
    if (!g_car.enabled) {
        board_all_motors_stop();
        return;
    }

    pwm_left = incremental_pi(0U);
    pwm_right = incremental_pi(1U);
    if (!motor_feedback_is_safe(0U, pwm_left)) return;
    if (!motor_feedback_is_safe(1U, pwm_right)) return;

    permille_left = (int)((float)pwm_left * 1000.0f / REFERENCE_PWM_FULL);
    permille_right = (int)((float)pwm_right * 1000.0f / REFERENCE_PWM_FULL);
    board_motor_set(0U, (int16_t)permille_left);
    board_motor_set(1U, (int16_t)permille_right);
}

void car_control_reset_pi(void)
{
    uint8_t index;
    for (index = 0U; index < 2U; index++) {
        g_car.motor[index].pwm = 0.0f;
        g_car.motor[index].previous_error = 0.0f;
        g_car.motor[index].target_mps = 0.0f;
    }
    memset(g_stall_ticks, 0, sizeof(g_stall_ticks));
    memset(g_reverse_ticks, 0, sizeof(g_reverse_ticks));
}

void car_control_stop(void)
{
    g_car.enabled = false;
    g_car.move_x_mps = 0.0f;
    g_car.move_z_radps = 0.0f;
    car_control_reset_pi();
    board_all_motors_stop();
}

uint32_t car_control_fault_flags(void)
{
    return g_car.fault_flags;
}
