#include "protocol.h"

#include "app_config.h"
#include "board_port.h"
#include "car_control.h"

#include <string.h>

#define RX_RING_SIZE       128U
#define ASCII_BUFFER_SIZE   64U
#define BINARY_COMMAND_SIZE 11U

ProtocolCommand g_protocol_command;

static volatile uint8_t g_rx_ring[RX_RING_SIZE];
static volatile uint8_t g_rx_write;
static uint8_t g_rx_read;
static uint8_t g_binary[BINARY_COMMAND_SIZE];
static uint8_t g_binary_count;
static char g_ascii[ASCII_BUFFER_SIZE];
static uint8_t g_ascii_count;
static bool g_ascii_active;

static int16_t be16_read(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void be16_write(uint8_t *data, int16_t value)
{
    data[0] = (uint8_t)((uint16_t)value >> 8U);
    data[1] = (uint8_t)value;
}

static int16_t saturate_i16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}

uint8_t protocol_xor(const uint8_t *data, size_t length)
{
    uint8_t value = 0U;
    size_t index;
    for (index = 0U; index < length; index++) value ^= data[index];
    return value;
}

void protocol_init(void)
{
    memset((void *)&g_protocol_command, 0, sizeof(g_protocol_command));
    g_protocol_command.requested_mode = APP_MODE_STOP;
    g_protocol_command.line_laps = 1U;
    g_rx_write = 0U;
    g_rx_read = 0U;
    g_binary_count = 0U;
    g_ascii_count = 0U;
    g_ascii_active = false;
}

void protocol_rx_byte_isr(uint8_t value)
{
    uint8_t next = (uint8_t)((g_rx_write + 1U) % RX_RING_SIZE);
    if (next != g_rx_read) {
        g_rx_ring[g_rx_write] = value;
        g_rx_write = next;
    }
}

static void accept_motion(float vx, float wz, uint32_t tick)
{
    g_protocol_command.vx_mps = vx;
    g_protocol_command.wz_radps = wz;
    g_protocol_command.requested_mode = APP_MODE_REMOTE;
    g_protocol_command.last_command_tick = tick;
    g_protocol_command.command_ready = true;
}

static void parse_binary(uint32_t tick)
{
    if ((g_binary[10] != 0x7DU) ||
        (g_binary[9] != protocol_xor(g_binary, 9U))) return;

    /* The legacy frame contains three motion fields; this differential
     * chassis only consumes forward speed and yaw rate. */
    if (g_binary[1] <= 3U) {
        accept_motion((float)be16_read(&g_binary[3]) / 1000.0f,
                      (float)be16_read(&g_binary[7]) / 1000.0f, tick);
    }
}

static void skip_separators(char **cursor)
{
    while ((**cursor == ' ') || (**cursor == ',') ||
           (**cursor == ':') || (**cursor == '=')) {
        (*cursor)++;
    }
}

static bool parse_small_float(char **cursor, float *value)
{
    char *p = *cursor;
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint32_t divisor = 1U;
    bool negative = false;
    bool have_digit = false;

    skip_separators(&p);
    if ((*p == '-') || (*p == '+')) {
        negative = (*p == '-');
        p++;
    }
    while ((*p >= '0') && (*p <= '9')) {
        have_digit = true;
        if (whole < 10000U) whole = whole * 10U + (uint32_t)(*p - '0');
        p++;
    }
    if (*p == '.') {
        p++;
        while ((*p >= '0') && (*p <= '9')) {
            have_digit = true;
            if (divisor < 100000U) {
                fraction = fraction * 10U + (uint32_t)(*p - '0');
                divisor *= 10U;
            }
            p++;
        }
    }
    if (!have_digit) return false;

    *value = (float)whole + (float)fraction / (float)divisor;
    if (negative) *value = -*value;
    *cursor = p;
    return true;
}

static void parse_ascii(uint32_t tick)
{
    char *cursor = g_ascii;
    float first;
    float second;

    g_ascii[g_ascii_count] = '\0';
    if ((strncmp(cursor, "STOP", 4U) == 0) && (cursor[4] == '\0')) {
        g_protocol_command.requested_mode = APP_MODE_STOP;
        g_protocol_command.command_ready = true;
    } else if (strncmp(cursor, "LINE", 4U) == 0) {
        cursor += 4;
        if (parse_small_float(&cursor, &first)) {
            if (first < 1.0f) first = 1.0f;
            if (first > 9.0f) first = 9.0f;
            g_protocol_command.line_laps = (uint8_t)first;
        }
        g_protocol_command.requested_mode = APP_MODE_LINE;
        g_protocol_command.command_ready = true;
    } else if ((cursor[0] == '[') || (strncmp(cursor, "MOVE", 4U) == 0)) {
        if (cursor[0] == '[') cursor++;
        else cursor += 4;
        if (!parse_small_float(&cursor, &first)) return;
        if (!parse_small_float(&cursor, &second)) return;
        /* [left,right] keeps the SYS convention: values are cm/s. */
        if (g_ascii[0] == '[') {
            accept_motion((first + second) * 0.005f,
                          (second - first) * 0.01f / APP_WHEEL_SPACING_M, tick);
        } else {
            /* MOVE takes forward speed (m/s) and yaw rate (rad/s). */
            accept_motion(first, second, tick);
        }
    }
}

static void consume_byte(uint8_t value, uint32_t tick)
{
    if (g_ascii_active) {
        if (value == '%') {
            parse_ascii(tick);
            g_ascii_active = false;
            g_ascii_count = 0U;
        } else if (g_ascii_count < (ASCII_BUFFER_SIZE - 1U)) {
            g_ascii[g_ascii_count++] = (char)value;
        } else {
            g_ascii_active = false;
            g_ascii_count = 0U;
        }
        return;
    }

    if (value == '@') {
        g_ascii_active = true;
        g_ascii_count = 0U;
        g_binary_count = 0U;
        return;
    }

    if (g_binary_count == 0U) {
        if (value == 0x7BU) g_binary[g_binary_count++] = value;
    } else {
        g_binary[g_binary_count++] = value;
        if (g_binary_count == BINARY_COMMAND_SIZE) {
            parse_binary(tick);
            g_binary_count = 0U;
        }
    }
}

void protocol_process(uint32_t tick)
{
    while (g_rx_read != g_rx_write) {
        uint8_t value = g_rx_ring[g_rx_read];
        g_rx_read = (uint8_t)((g_rx_read + 1U) % RX_RING_SIZE);
        consume_byte(value, tick);
    }
}

void protocol_send_telemetry(uint32_t tick, uint16_t battery_adc)
{
    uint8_t frame[24] = {0U};
    float left = g_car.motor[0].encoder_mps;
    float right = g_car.motor[1].encoder_mps;
    int16_t vx = saturate_i16((left + right) * 500.0f);
    int16_t wz = saturate_i16((right - left) * 1000.0f / g_car.wheel_spacing_m);
    uint16_t battery_mv = (uint16_t)((uint32_t)battery_adc * 3300U / 4095U);

    (void)tick;
    frame[0] = 0x7BU;
    frame[1] = (g_protocol_command.requested_mode == APP_MODE_STOP) ? 1U : 0U;
    be16_write(&frame[2], vx);
    be16_write(&frame[4], 0);
    be16_write(&frame[6], wz);
    /* 8..19 are reserved for an optional IMU, matching the reference frame. */
    be16_write(&frame[20], (int16_t)battery_mv);
    frame[22] = protocol_xor(frame, 22U);
    frame[23] = 0x7DU;
    board_uart_send(frame, sizeof(frame));

}
