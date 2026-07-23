#ifndef APP_H
#define APP_H

#include <stdint.h>

void app_init(void);
void app_process(void);

/*
 * Transport-independent command entry points.
 * UART uses these now; a future CAN foreground handler can use the same API.
 * Interrupt handlers should only queue received bytes/frames and call these
 * functions later from the foreground context.
 */
void app_request_stop(void);
void app_request_line(uint8_t laps);
void app_request_motion(float vx_mps, float wz_radps, uint32_t command_tick);

#endif
