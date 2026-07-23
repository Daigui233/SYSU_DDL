#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LINE_STOP = 0,
    LINE_TRACE,
    LINE_TURN
} LineState;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_limit;
} LinePid;

typedef struct {
    uint16_t raw[6];
    int16_t normalized[6];
    float position;
    float filtered_position;
    LinePid trace_pid;
    LinePid turn_pid;
    LineState state;
    uint8_t target_laps;
    uint8_t completed_laps;
    uint8_t turn_count;
    uint16_t state_ticks;
    uint32_t last_corner_tick;
} LineFollow;

extern LineFollow g_line;

void line_follow_init(void);
void line_follow_start(uint8_t laps);
void line_follow_stop(void);
bool line_follow_update(uint32_t tick, float *left_mps, float *right_mps);
void line_follow_set_pid(float kp, float ki, float kd);

#endif
