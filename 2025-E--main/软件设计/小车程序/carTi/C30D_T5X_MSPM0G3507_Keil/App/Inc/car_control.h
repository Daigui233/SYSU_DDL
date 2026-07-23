#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float encoder_mps;
    float target_mps;
    float pwm;
    float kp;
    float ki;
    float previous_error;
} MotorParameter;

typedef struct {
    float wheel_spacing_m;
    float wheel_perimeter_m;
    float encoder_counts_per_rev;
    float move_x_mps;
    float move_z_radps;
    bool enabled;
    uint32_t fault_flags;
    MotorParameter motor[2];
} CarControl;

enum {
    CAR_FAULT_LEFT_STALL      = (1UL << 0),
    CAR_FAULT_RIGHT_STALL     = (1UL << 1),
    CAR_FAULT_LEFT_DIRECTION  = (1UL << 2),
    CAR_FAULT_RIGHT_DIRECTION = (1UL << 3)
};

extern CarControl g_car;

void car_control_init(void);
void car_control_set_motion(float vx_mps, float wz_radps);
void car_control_set_diff_targets(float left_mps, float right_mps);
void car_control_update_encoders(int32_t encoder_a, int32_t encoder_b);
void car_control_step(void);
void car_control_stop(void);
void car_control_reset_pi(void);
uint32_t car_control_fault_flags(void);

#endif
