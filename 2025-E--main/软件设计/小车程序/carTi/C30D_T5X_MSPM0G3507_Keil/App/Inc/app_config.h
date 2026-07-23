#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* T5X parameters taken from the STM32 reference project. */
#define APP_CONTROL_HZ                 100U
#define APP_TELEMETRY_HZ                20U
#define APP_COMMAND_TIMEOUT_TICKS       30U
#define APP_WHEEL_DIAMETER_M          0.065f
#define APP_WHEEL_SPACING_M           0.143f
#define APP_GEAR_RATIO                 22.5f
#define APP_ENCODER_LINES             500.0f
#define APP_ENCODER_MULTIPLIER          4.0f
#define APP_ENCODER_COUNTS_PER_REV \
    (APP_GEAR_RATIO * APP_ENCODER_LINES * APP_ENCODER_MULTIPLIER)

#define APP_PWM_LIMIT_PERMILLE         350
#define APP_MOTOR_PWM_SLEW_PER_TICK     20
#define APP_MOTOR_REVERSE_COAST_TICKS    3U
#define APP_MOTOR_STALL_PWM_COUNTS    3500
#define APP_MOTOR_STALL_TARGET_MPS      0.08f
#define APP_MOTOR_STALL_ENCODER_MPS     0.02f
#define APP_MOTOR_STALL_TICKS            15U
#define APP_MOTOR_REVERSE_FAULT_TICKS    10U
#define APP_DEFAULT_VELOCITY_KP      370.0f
#define APP_DEFAULT_VELOCITY_KI      370.0f

#define APP_LINE_SENSOR_COUNT            6U
#define APP_LINE_TARGET_POSITION      240.0f
#define APP_LINE_BASE_SPEED_MPS         0.20f
#define APP_LINE_TURN_SPEED_MPS         0.08f
#define APP_LINE_CORRECTION_MPS_PER_UNIT 0.020f

#endif
