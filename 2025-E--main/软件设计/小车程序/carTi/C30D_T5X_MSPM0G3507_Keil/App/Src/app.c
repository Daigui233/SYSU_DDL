#include "app.h"

#include "app_config.h"
#include "board_port.h"
#include "car_control.h"
#include "line_follow.h"
#include "protocol.h"

#include <stdbool.h>
#include <stdint.h>

static uint32_t g_control_tick;
static AppMode g_active_mode;
static uint8_t g_selected_laps;

void app_init(void)
{
    board_init();
    car_control_init();
    line_follow_init();
    protocol_init();
    g_control_tick = 0U;
    g_active_mode = APP_MODE_STOP;
    g_selected_laps = 1U;
    board_led_set(true);
    board_buzzer_set(false);
}

static void update_keys(void)
{
    if (board_key_pressed(0U)) {
        g_selected_laps++;
        if (g_selected_laps > 9U) g_selected_laps = 1U;
        g_protocol_command.line_laps = g_selected_laps;
    }
    if (board_key_pressed(1U)) {
        g_protocol_command.requested_mode =
            (g_protocol_command.requested_mode == APP_MODE_LINE) ?
            APP_MODE_REMOTE : APP_MODE_LINE;
        g_protocol_command.command_ready = true;
    }
    if (board_key_pressed(2U)) {
        if (g_active_mode == APP_MODE_STOP) {
            g_protocol_command.requested_mode = APP_MODE_LINE;
            g_protocol_command.line_laps = g_selected_laps;
        } else {
            g_protocol_command.requested_mode = APP_MODE_STOP;
        }
        g_protocol_command.command_ready = true;
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
            g_protocol_command.requested_mode = APP_MODE_STOP;
            g_protocol_command.command_ready = true;
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
    board_led_set(g_active_mode != APP_MODE_STOP);
    if ((g_control_tick % (APP_CONTROL_HZ / APP_TELEMETRY_HZ)) == 0U)
        protocol_send_telemetry(g_control_tick, board_aux_adc_read());
}

void app_process(void)
{
    board_poll();
    protocol_process(g_control_tick);
    while (board_take_control_tick()) control_step();
}
