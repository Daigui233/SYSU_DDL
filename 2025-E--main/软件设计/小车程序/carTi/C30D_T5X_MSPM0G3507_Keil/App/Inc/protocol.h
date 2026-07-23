#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    APP_MODE_STOP = 0,
    APP_MODE_REMOTE,
    APP_MODE_LINE
} AppMode;

typedef struct {
    volatile AppMode requested_mode;
    volatile float vx_mps;
    volatile float wz_radps;
    volatile uint8_t line_laps;
    volatile uint32_t last_command_tick;
    volatile bool command_ready;
} ProtocolCommand;

extern ProtocolCommand g_protocol_command;

void protocol_init(void);
void protocol_rx_byte_isr(uint8_t value);
void protocol_process(uint32_t tick);
void protocol_send_telemetry(uint32_t tick, uint16_t battery_adc);
uint8_t protocol_xor(const uint8_t *data, size_t length);

#endif
