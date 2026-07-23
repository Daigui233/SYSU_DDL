#include "oled.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define OLED_WIDTH                128U
#define OLED_PAGE_COUNT             8U
#define OLED_I2C_TIMEOUT_LOOPS  800000U
#define OLED_CONTROL_COMMAND      0x00U
#define OLED_CONTROL_DATA         0x40U
#define OLED_MAX_DATA_PER_WRITE      7U

static uint8_t g_framebuffer[OLED_WIDTH * OLED_PAGE_COUNT];
static uint8_t g_flush_page;
static bool g_ready;

static void i2c_abort(void)
{
    DL_I2C_resetControllerTransfer(OLED_I2C_INST);
    DL_I2C_flushControllerTXFIFO(OLED_I2C_INST);
    DL_I2C_flushControllerRXFIFO(OLED_I2C_INST);
}

static bool i2c_wait_idle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0U) {
        uint32_t status = DL_I2C_getControllerStatus(OLED_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) return false;
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) return true;
    }
    return false;
}

static bool i2c_wait_done(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT_LOOPS;

    while (timeout-- != 0U) {
        uint32_t status = DL_I2C_getControllerStatus(OLED_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) return false;
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U)
            return i2c_wait_idle();
    }
    return false;
}

/*
 * This follows the known-good driver exactly: the MSPM0 I2C TX FIFO holds
 * eight bytes, so each transaction contains one control byte and no more
 * than seven SSD1306 command/data bytes.
 */
static bool oled_write(uint8_t control, const uint8_t *data, uint8_t length)
{
    uint8_t packet[OLED_MAX_DATA_PER_WRITE + 1U];
    uint8_t index;
    uint8_t packet_length;

    if ((data == NULL) || (length == 0U) ||
        (length > OLED_MAX_DATA_PER_WRITE)) return false;
    if (!i2c_wait_idle()) {
        i2c_abort();
        return false;
    }

    packet[0] = control;
    for (index = 0U; index < length; index++) packet[index + 1U] = data[index];
    packet_length = (uint8_t)(length + 1U);
    DL_I2C_flushControllerTXFIFO(OLED_I2C_INST);
    if (DL_I2C_fillControllerTXFIFO(
            OLED_I2C_INST, packet, packet_length) != packet_length) {
        i2c_abort();
        return false;
    }
    DL_I2C_startControllerTransfer(OLED_I2C_INST, OLED_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, packet_length);
    if (!i2c_wait_done()) {
        i2c_abort();
        return false;
    }
    return true;
}

static bool oled_command(uint8_t command)
{
    return oled_write(OLED_CONTROL_COMMAND, &command, 1U);
}

static bool oled_set_position(uint8_t page, uint8_t column)
{
    if ((page >= OLED_PAGE_COUNT) || (column >= OLED_WIDTH)) return false;
    return oled_command((uint8_t)(0xB0U + page)) &&
           oled_command((uint8_t)(column & 0x0FU)) &&
           oled_command((uint8_t)(0x10U | (column >> 4)));
}

static bool oled_clear_panel(void)
{
    static const uint8_t blank[OLED_MAX_DATA_PER_WRITE] = {0};
    uint8_t page;
    uint8_t column;
    uint8_t count;

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        if (!oled_set_position(page, 0U)) return false;
        for (column = 0U; column < OLED_WIDTH;
             column = (uint8_t)(column + count)) {
            count = (uint8_t)(OLED_WIDTH - column);
            if (count > OLED_MAX_DATA_PER_WRITE)
                count = OLED_MAX_DATA_PER_WRITE;
            if (!oled_write(OLED_CONTROL_DATA, blank, count)) return false;
        }
    }
    return true;
}

static const uint8_t *glyph(char c)
{
    static const uint8_t blank[5] = {0, 0, 0, 0, 0};
    static const uint8_t plus[5]  = {0x08, 0x08, 0x3E, 0x08, 0x08};
    static const uint8_t minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t digit[10][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E}
    };
    static const uint8_t upper[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
    };
    static const uint8_t lower_m[5] = {0x7C,0x04,0x18,0x04,0x78};
    static const uint8_t lower_s[5] = {0x48,0x54,0x54,0x54,0x20};

    if ((c >= '0') && (c <= '9')) return digit[(uint8_t)(c - '0')];
    if ((c >= 'A') && (c <= 'Z')) return upper[(uint8_t)(c - 'A')];
    if (c == '+') return plus;
    if (c == '-') return minus;
    if (c == '.') return dot;
    if (c == '/') return slash;
    if (c == ':') return colon;
    if (c == 'm') return lower_m;
    if (c == 's') return lower_s;
    return blank;
}

static void draw_text(uint8_t page, uint8_t column, const char *text)
{
    uint16_t index;

    if ((page >= OLED_PAGE_COUNT) || (text == NULL)) return;
    index = (uint16_t)page * OLED_WIDTH + column;
    while ((*text != '\0') && (index + 5U < sizeof(g_framebuffer))) {
        const uint8_t *shape = glyph(*text++);
        uint8_t i;
        for (i = 0U; i < 5U; i++) g_framebuffer[index++] = shape[i];
        g_framebuffer[index++] = 0U;
        if ((index / OLED_WIDTH) != page) break;
    }
}

static void format_angle(char label, float value, char out[10])
{
    uint32_t scaled;

    if (value > 999.9f) value = 999.9f;
    if (value < -999.9f) value = -999.9f;
    out[0] = label;
    out[1] = ':';
    out[2] = (value < 0.0f) ? '-' : '+';
    if (value < 0.0f) value = -value;
    scaled = (uint32_t)(value * 10.0f + 0.5f);
    out[3] = (char)('0' + ((scaled / 1000U) % 10U));
    out[4] = (char)('0' + ((scaled / 100U) % 10U));
    out[5] = (char)('0' + ((scaled / 10U) % 10U));
    out[6] = '.';
    out[7] = (char)('0' + (scaled % 10U));
    out[8] = '\0';
}

static void format_speed(char label, float value, char out[14])
{
    uint32_t scaled;

    if (value > 9.999f) value = 9.999f;
    if (value < -9.999f) value = -9.999f;
    out[0] = label;
    out[1] = ':';
    out[2] = (value < 0.0f) ? '-' : '+';
    if (value < 0.0f) value = -value;
    scaled = (uint32_t)(value * 1000.0f + 0.5f);
    out[3] = (char)('0' + ((scaled / 1000U) % 10U));
    out[4] = '.';
    out[5] = (char)('0' + ((scaled / 100U) % 10U));
    out[6] = (char)('0' + ((scaled / 10U) % 10U));
    out[7] = (char)('0' + (scaled % 10U));
    out[8] = 'm';
    out[9] = '/';
    out[10] = 's';
    out[11] = '\0';
}

bool oled_init(void)
{
    static const uint8_t init_commands[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1, 0xDB,
        0x30, 0xA4, 0xA6, 0xAF
    };
    uint8_t index;

    g_ready = false;
    g_flush_page = 0U;
    memset(g_framebuffer, 0, sizeof(g_framebuffer));
    delay_cycles(CPUCLK_FREQ / 10U);
    for (index = 0U; index < sizeof(init_commands); index++) {
        if (!oled_command(init_commands[index])) return false;
    }
    if (!oled_clear_panel()) return false;
    g_ready = true;
    return g_ready;
}

bool oled_is_ready(void)
{
    return g_ready;
}

void oled_show_status(float roll_deg, float pitch_deg, float yaw_deg,
                      float left_mps, float right_mps, bool imu_valid)
{
    char line[14];

    if (!g_ready) return;
    memset(g_framebuffer, 0, sizeof(g_framebuffer));
    if (imu_valid) {
        format_angle('R', roll_deg, line);
        draw_text(0U, 0U, line);
        format_angle('P', pitch_deg, line);
        draw_text(1U, 0U, line);
        format_angle('Y', yaw_deg, line);
        draw_text(2U, 0U, line);
    } else {
        draw_text(0U, 0U, "R:ERR");
        draw_text(1U, 0U, "P:ERR");
        draw_text(2U, 0U, "Y:ERR");
    }
    format_speed('L', left_mps, line);
    draw_text(4U, 0U, line);
    format_speed('R', right_mps, line);
    draw_text(5U, 0U, line);
}

void oled_process(void)
{
    uint8_t column;
    uint8_t count;

    if (!g_ready) return;
    if (!oled_set_position(g_flush_page, 0U)) {
        g_ready = false;
        return;
    }

    for (column = 0U; column < OLED_WIDTH;
         column = (uint8_t)(column + count)) {
        count = (uint8_t)(OLED_WIDTH - column);
        if (count > OLED_MAX_DATA_PER_WRITE)
            count = OLED_MAX_DATA_PER_WRITE;
        if (!oled_write(OLED_CONTROL_DATA,
                &g_framebuffer[(uint16_t)g_flush_page * OLED_WIDTH + column],
                count)) {
            g_ready = false;
            return;
        }
    }
    g_flush_page++;
    if (g_flush_page >= OLED_PAGE_COUNT) g_flush_page = 0U;
}
