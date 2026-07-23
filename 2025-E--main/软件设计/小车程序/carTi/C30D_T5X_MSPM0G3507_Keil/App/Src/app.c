#include "app.h"

#include "app_config.h"
#include "attitude.h"
#include "board_port.h"
#include "car_control.h"
#include "line_follow.h"
#include "oled.h"
#include "protocol.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_HEARTBEAT_PERIOD_MS 500U

static uint32_t g_control_tick;
static uint32_t g_heartbeat_tick;
static uint32_t g_oled_flush_millis;
static AppMode g_active_mode;
static uint8_t g_selected_laps;
static bool g_heartbeat_on;

void app_request_stop(void)
{
    g_protocol_command.requested_mode = APP_MODE_STOP;
    g_protocol_command.command_ready = true;
}

void app_request_line(uint8_t laps)
{
    if (laps == 0U) laps = 1U;
    if (laps > 9U) laps = 9U;
    g_selected_laps = laps;
    g_protocol_command.line_laps = laps;
    g_protocol_command.requested_mode = APP_MODE_LINE;
    g_protocol_command.command_ready = true;
}

void app_request_motion(float vx_mps, float wz_radps, uint32_t command_tick)
{
    g_protocol_command.vx_mps = vx_mps;
    g_protocol_command.wz_radps = wz_radps;
    g_protocol_command.last_command_tick = command_tick;
    g_protocol_command.requested_mode = APP_MODE_REMOTE;
    g_protocol_command.command_ready = true;
}

void app_init(void)
{
    board_init();
    car_control_init();
    line_follow_init();
    protocol_init();
    (void)attitude_init();
    (void)oled_init();
    g_control_tick = 0U;
    g_active_mode = APP_MODE_STOP;
    g_selected_laps = 1U;
    g_heartbeat_tick = board_millis();
    g_oled_flush_millis = g_heartbeat_tick;
    g_heartbeat_on = true;
    board_led_set(g_heartbeat_on);
    board_buzzer_set(false);
    oled_show_status(attitude_get()->roll_deg, attitude_get()->pitch_deg,
        attitude_get()->yaw_deg, 0.0f, 0.0f, attitude_get()->valid);
}

static void update_keys(void)
{
    if (board_key_pressed(0U)) {
        g_selected_laps++;
        if (g_selected_laps > 9U) g_selected_laps = 1U;
        g_protocol_command.line_laps = g_selected_laps;
    }
    if (board_key_pressed(1U)) {
        if (g_protocol_command.requested_mode == APP_MODE_LINE)
            app_request_stop();
        else
            app_request_line(g_selected_laps);
    }
    if (board_key_pressed(2U)) {
        if (g_active_mode == APP_MODE_STOP)
            app_request_line(g_selected_laps);
        else
            app_request_stop();
    }
}

static void apply_requested_mode(void)
{
    if (!g_protocol_command.command_ready) return;
    g_protocol_command.command_ready = false;

    if (g_protocol_command.requested_mode != g_active_mode) {
        car_control_stop();
        line_follow_stop();
        board_buzzer_set(false);
        g_active_mode = g_protocol_command.requested_mode;
        if (g_active_mode == APP_MODE_LINE)
            line_follow_start(g_protocol_command.line_laps);
    }
}

static void control_step(void)
{
    int32_t encoder_left;
    int32_t encoder_right;
    float left_target;
    float right_target;

    g_control_tick++;
    update_keys();
    apply_requested_mode();
    board_encoder_snapshot(&encoder_left, &encoder_right);
    car_control_update_encoders(encoder_left, encoder_right);

    switch (g_active_mode) {
    case APP_MODE_REMOTE:
        if ((g_control_tick - g_protocol_command.last_command_tick) >
            APP_COMMAND_TIMEOUT_TICKS) {
            app_request_stop();
            car_control_stop();
        } else {
            car_control_set_motion(g_protocol_command.vx_mps,
                                   g_protocol_command.wz_radps);
        }
        break;
    case APP_MODE_LINE:
        if (line_follow_update(g_control_tick, &left_target, &right_target)) {
            car_control_set_diff_targets(left_target, right_target);
        } else {
            car_control_stop();
            g_active_mode = APP_MODE_STOP;
            g_protocol_command.requested_mode = APP_MODE_STOP;
            board_buzzer_set(true);
        }
        break;
    case APP_MODE_STOP:
    default:
        car_control_stop();
        break;
    }

    car_control_step();
    attitude_update(1.0f / (float)APP_CONTROL_HZ);
    if ((g_control_tick % (APP_CONTROL_HZ / APP_DISPLAY_HZ)) == 0U) {
        const Attitude *attitude = attitude_get();
        oled_show_status(attitude->roll_deg, attitude->pitch_deg,
            attitude->yaw_deg, g_car.motor[0].encoder_mps,
            g_car.motor[1].encoder_mps, attitude->valid);
    }
    if ((g_control_tick % (APP_CONTROL_HZ / APP_TELEMETRY_HZ)) == 0U)
        protocol_send_telemetry(g_control_tick, board_aux_adc_read());
}

void app_process(void)
{
    uint32_t now;

    board_poll();
    protocol_process(g_control_tick);

    /* PB22/D22 initialization heartbeat; no delay and no control-loop block. */
    now = board_millis();
    if ((uint32_t)(now - g_heartbeat_tick) >= APP_HEARTBEAT_PERIOD_MS) {
        g_heartbeat_tick = now;
        g_heartbeat_on = !g_heartbeat_on;
        board_led_set(g_heartbeat_on);
    }

    while (board_take_control_tick()) control_step();

    /* Flush one 128-byte OLED page at most once per 10 ms control period. */
    now = board_millis();
    if (now != g_oled_flush_millis) {
        g_oled_flush_millis = now;
        oled_process();
    }
}
