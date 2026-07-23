#include "line_follow.h"

#include "app_config.h"
#include "board_port.h"

#include <string.h>

static const uint16_t k_sensor_min[APP_LINE_SENSOR_COUNT] = {
    340U, 1000U, 600U, 300U, 300U, 500U
};
static const uint16_t k_sensor_max[APP_LINE_SENSOR_COUNT] = {
    1750U, 2050U, 1700U, 1500U, 1600U, 1900U
};
static const uint16_t k_sensor_weight[APP_LINE_SENSOR_COUNT] = {
    1U, 100U, 200U, 300U, 400U, 500U
};

LineFollow g_line;

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float pid_update(LinePid *pid, float measurement, float target)
{
    float error = target - measurement;
    float derivative = error - pid->previous_error;
    float output;

    pid->integral = clampf(pid->integral + error, -2000.0f, 2000.0f);
    pid->previous_error = error;
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return clampf(output, -pid->output_limit, pid->output_limit);
}

static void read_position(void)
{
    uint8_t i;
    uint32_t weighted = 0U;
    uint32_t sum = 0U;

    board_line_adc_read(g_line.raw);
    for (i = 0U; i < APP_LINE_SENSOR_COUNT; i++) {
        int32_t normalized;
        uint16_t raw = g_line.raw[i];
        if (raw < k_sensor_min[i]) raw = k_sensor_min[i];
        if (raw > k_sensor_max[i]) raw = k_sensor_max[i];
        normalized = ((int32_t)raw - k_sensor_min[i]) * 1000L /
                     ((int32_t)k_sensor_max[i] - k_sensor_min[i]);
        if (normalized < 5L) normalized = 1L;
        g_line.normalized[i] = (int16_t)normalized;
        weighted += (uint32_t)k_sensor_weight[i] * (uint32_t)normalized;
        sum += (uint32_t)normalized;
    }

    if (sum != 0U) {
        g_line.position = (float)weighted / (float)sum;
        g_line.filtered_position = 0.6f * g_line.filtered_position +
                                   0.4f * g_line.position;
    }
}

void line_follow_init(void)
{
    memset(&g_line, 0, sizeof(g_line));
    g_line.filtered_position = APP_LINE_TARGET_POSITION;
    g_line.position = APP_LINE_TARGET_POSITION;
    g_line.trace_pid.kp = 0.030f;
    g_line.trace_pid.ki = 0.0f;
    g_line.trace_pid.kd = 1.6f;
    g_line.trace_pid.output_limit = 7.0f;
    g_line.turn_pid.kp = 0.473f;
    g_line.turn_pid.ki = 0.009f;
    g_line.turn_pid.kd = 0.0f;
    g_line.turn_pid.output_limit = 4.0f;
    g_line.state = LINE_STOP;
}

void line_follow_start(uint8_t laps)
{
    g_line.target_laps = (laps == 0U) ? 1U : laps;
    g_line.completed_laps = 0U;
    g_line.turn_count = 0U;
    g_line.state_ticks = 0U;
    g_line.last_corner_tick = 0U;
    g_line.trace_pid.integral = 0.0f;
    g_line.trace_pid.previous_error = 0.0f;
    g_line.turn_pid.integral = 0.0f;
    g_line.turn_pid.previous_error = 0.0f;
    g_line.state = LINE_TRACE;
}

void line_follow_stop(void)
{
    g_line.state = LINE_STOP;
}

bool line_follow_update(uint32_t tick, float *left_mps, float *right_mps)
{
    LinePid *pid;
    float base_speed;
    float correction;

    if ((left_mps == NULL) || (right_mps == NULL) ||
        (g_line.state == LINE_STOP)) {
        return false;
    }

    read_position();
    g_line.state_ticks++;

    if ((g_line.state == LINE_TRACE) &&
        (g_line.filtered_position <= 210.0f) &&
        ((tick - g_line.last_corner_tick) > 200U)) {
        g_line.state = LINE_TURN;
        g_line.state_ticks = 0U;
        g_line.last_corner_tick = tick;
    }

    if ((g_line.state == LINE_TURN) && (g_line.state_ticks >= 30U) &&
        (g_line.filtered_position >= 230.0f)) {
        g_line.turn_count++;
        g_line.state = LINE_TRACE;
        g_line.state_ticks = 0U;
        if ((g_line.turn_count % 4U) == 0U) {
            g_line.completed_laps++;
            if (g_line.completed_laps >= g_line.target_laps) {
                line_follow_stop();
                *left_mps = 0.0f;
                *right_mps = 0.0f;
                return false;
            }
        }
    }

    pid = (g_line.state == LINE_TURN) ? &g_line.turn_pid : &g_line.trace_pid;
    base_speed = (g_line.state == LINE_TURN) ? APP_LINE_TURN_SPEED_MPS :
                                               APP_LINE_BASE_SPEED_MPS;
    correction = pid_update(pid, g_line.filtered_position,
                            APP_LINE_TARGET_POSITION) *
                 APP_LINE_CORRECTION_MPS_PER_UNIT;
    *left_mps = base_speed - correction;
    *right_mps = base_speed + correction;
    return true;
}

void line_follow_set_pid(float kp, float ki, float kd)
{
    g_line.trace_pid.kp = kp;
    g_line.trace_pid.ki = ki;
    g_line.trace_pid.kd = kd;
    g_line.trace_pid.integral = 0.0f;
    g_line.trace_pid.previous_error = 0.0f;
}
