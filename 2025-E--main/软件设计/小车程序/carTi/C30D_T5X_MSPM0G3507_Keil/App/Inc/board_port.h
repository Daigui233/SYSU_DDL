#ifndef BOARD_PORT_H
#define BOARD_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void board_init(void);
void board_poll(void);
uint32_t board_millis(void);
bool board_take_control_tick(void);

void board_motor_set(uint8_t channel, int16_t pwm_permille);
void board_all_motors_stop(void);
void board_motor_fault_shutdown(void);
void board_encoder_snapshot(int32_t *encoder_a, int32_t *encoder_b);

void board_line_adc_read(uint16_t values[6]);
uint16_t board_aux_adc_read(void);
bool board_key_pressed(uint8_t key_index);
void board_led_set(bool on);
void board_buzzer_set(bool on);

bool board_uart_send_byte(uint8_t value);
void board_uart_send(const uint8_t *data, size_t length);

void protocol_rx_byte_isr(uint8_t value);

#endif
