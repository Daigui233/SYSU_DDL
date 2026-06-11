
#include "Control.h"

#define MOTOR_PI                       (3.1415926f)
#define MOTOR_CONTROL_PERIOD_S         (0.01f)
#define MOTOR_WHEEL_DIAMETER_M         (0.064f)
#define MOTOR_ENCODER_LINES            (512.0f)
#define MOTOR_ENCODER_QUADRATURE       (4.0f)
#define MOTOR_REDUCTION_RATIO          (2.7f)
#define MOTOR_ENCODER_SIGN             (1.0f)  /* Use -1.0f if forward encoder speed is negative. */
#define MOTOR_ENCODER_COUNT_TO_MPS     ((MOTOR_PI * MOTOR_WHEEL_DIAMETER_M) / (MOTOR_CONTROL_PERIOD_S * MOTOR_ENCODER_LINES * MOTOR_ENCODER_QUADRATURE * MOTOR_REDUCTION_RATIO))
#define MOTOR_TARGET_DEADBAND_MPS      (0.03f)
#define MOTOR_FEEDFORWARD_BASE_DUTY    (1000.0f)
#define MOTOR_FEEDFORWARD_SPEED_GAIN   (250.0f)
#define MOTOR_FEEDFORWARD_MAX_DUTY     (1800.0f)
#define MOTOR_PID_CORRECTION_LIMIT     (700.0f)

/* ===== State control parameters: tune here first during vehicle testing. ===== */
#define TRACK_DEFAULT_SPEED_MPS        (0.05f)
#define TRACK_MOTOR_KP                 (100.0f)
#define TRACK_MOTOR_KI                 (8.0f)
#define TRACK_MOTOR_KD                 (0.0f)
#define TRACK_SERVO_KP                 (0.55f)
#define TRACK_SERVO_KD                 (0.08f)
/* Extra output boost for large TRACK-like errors. It keeps straight-line small errors soft
 * while allowing real-pixel final_track_error in large curves to approach the steering limit. */
#define TRACK_SERVO_BOOST_START_ERROR  (80.0f)
#define TRACK_SERVO_BOOST_OUTPUT_GAIN  (0.90f)

#define AVOID_STONE_DEFAULT_SPEED_MPS  TRACK_DEFAULT_SPEED_MPS
#define AVOID_STONE_MOTOR_KP           TRACK_MOTOR_KP
#define AVOID_STONE_MOTOR_KI           TRACK_MOTOR_KI
#define AVOID_STONE_MOTOR_KD           TRACK_MOTOR_KD
#define AVOID_STONE_SERVO_KP           TRACK_SERVO_KP
#define AVOID_STONE_SERVO_KD           TRACK_SERVO_KD

#define AVOID_CAR_DEFAULT_SPEED_MPS    (0.05f)
#define AVOID_CAR_MOTOR_KP             (100.0f)
#define AVOID_CAR_MOTOR_KI             (8.0f)
#define AVOID_CAR_MOTOR_KD             (0.0f)
#define AVOID_CAR_SERVO_KP             (0.46f)
#define AVOID_CAR_SERVO_KD             (0.08f)

#define AVOID_HUMAN_DEFAULT_SPEED_MPS  (0.04f)
#define AVOID_HUMAN_MOTOR_KP           (100.0f)
#define AVOID_HUMAN_MOTOR_KI           (8.0f)
#define AVOID_HUMAN_MOTOR_KD           (0.0f)
#define AVOID_HUMAN_SERVO_KP           (0.46f)
#define AVOID_HUMAN_SERVO_KD           (0.08f)

#define COLLECT_GOLD_DEFAULT_SPEED_MPS (0.05f)
#define COLLECT_GOLD_MOTOR_KP          (100.0f)
#define COLLECT_GOLD_MOTOR_KI          (8.0f)
#define COLLECT_GOLD_MOTOR_KD          (0.0f)
#define COLLECT_GOLD_SERVO_KP          (0.38f)
#define COLLECT_GOLD_SERVO_KD          (0.06f)

#define RECOVER_LINE_DEFAULT_SPEED_MPS (0.03f)
#define RECOVER_LINE_MOTOR_KP          (100.0f)
#define RECOVER_LINE_MOTOR_KI          (8.0f)
#define RECOVER_LINE_MOTOR_KD          (0.0f)
#define RECOVER_LINE_SERVO_KP          (0.30f)
#define RECOVER_LINE_SERVO_KD          (0.04f)

control_ctx_struct control_ctx;

control_input_struct input_communication_temp;

static float control_abs_float(float input)
{
    if(input < 0.0f)
    {
        return -input;
    }

    return input;
}

static float control_sign_float(float input)
{
    if(input > 0.0f)
    {
        return 1.0f;
    }

    if(input < 0.0f)
    {
        return -1.0f;
    }

    return 0.0f;
}

static float control_track_servo_large_error_boost(car_state_enum state, float track_error)
{
    float abs_error;
    float boost;

    if(state != STATE_TRACK && state != STATE_AVOID_STONE)
    {
        return 0.0f;
    }

    abs_error = control_abs_float(track_error);
    if(abs_error <= TRACK_SERVO_BOOST_START_ERROR)
    {
        return 0.0f;
    }

    boost = (abs_error - TRACK_SERVO_BOOST_START_ERROR) * TRACK_SERVO_BOOST_OUTPUT_GAIN;
    return -control_sign_float(track_error) * boost;
}

static float control_limit_float(float input, float max, float min)
{
    if(input > max)
    {
        input = max;
    }
    else if(input < min)
    {
        input = min;
    }

    return input;
}

static uint32 control_limit_uint32(int32 input, uint32 max, uint32 min)
{
    if (input > (int32)max)
    {
        input = (int32)max;
    }
    else if (input < (int32)min)
    {
        input = (int32)min;
    }

    return (uint32)input;
}

static uint8 control_allow_speed_override(car_state_enum state)
{
    switch (state)
    {
    case STATE_TRACK:
    case STATE_AVOID_STONE:
    case STATE_AVOID_CAR:
    case STATE_AVOID_HUMAN:
    case STATE_COLLECT_GOLD:
    case STATE_RECOVER_LINE:
        return 1;

    case STATE_LINE_LOSS_SAFE_STOP:
    case STATE_TRAFFIC_LIGHT_STOP:
    case STATE_ENDSIGN_STOP:
    case STATE_SAFE_STOP:
    case STATE_IDLE:
    default:
        return 0;
    }
}

static void control_reset_input_to_safe(void)
{
    control_ctx.input.target_speed = 0.0f;
    control_ctx.input.track_error = 0.0f;
    control_ctx.input.state_cmd = (uint8)STATE_SAFE_STOP;
    control_ctx.input.flags = 0;
}

static void control_check_input_timeout(void)
{
    uint32 now_us;

    if (0 == control_ctx.input_online)
    {
        return;
    }

    now_us = system_getval_us();
    if ((uint32)(now_us - control_ctx.last_input_time_us) <= CONTROL_INPUT_TIMEOUT_US)
    {
        return;
    }

    control_ctx.input_online = 0;
    control_reset_input_to_safe();
    state_set(STATE_SAFE_STOP);
}


void control_init(void)
{
    control_reset_input_to_safe();
    control_ctx.last_input_time_us = 0;
    control_ctx.input_online = 0;
    control_ctx.current_state = state_get();
    control_apply_state_param(control_ctx.current_state);
}

void control_set_input(control_input_struct input)
{
    control_ctx.input = input;
    control_ctx.last_input_time_us = system_getval_us();
    control_ctx.input_online = 1;

    if (control_ctx.input.state_cmd < STATE_MAX)
    {
        state_set((car_state_enum)control_ctx.input.state_cmd);
    }
}

control_input_struct control_get_input(void)
{
    return control_ctx.input;
}

static void control_set_motion_param(float motor_target_speed,
                                     float motor_kp,
                                     float motor_ki,
                                     float motor_kd,
                                     float servo_kp,
                                     float servo_kd)
{
    control_ctx.param.motor_target_speed = motor_target_speed;
    control_ctx.param.motor_kp = motor_kp;
    control_ctx.param.motor_ki = motor_ki;
    control_ctx.param.motor_kd = motor_kd;
    control_ctx.param.motor_output_min = -MOTOR_PID_CORRECTION_LIMIT;
    control_ctx.param.motor_output_max = MOTOR_PID_CORRECTION_LIMIT;
    control_ctx.param.servo_kp = servo_kp;
    control_ctx.param.servo_kd = servo_kd;
    control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
    control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
}

void control_apply_state_param(car_state_enum state)
{
    control_ctx.current_state = state;

    switch (state)
    {
    case STATE_TRACK:
        control_set_motion_param(TRACK_DEFAULT_SPEED_MPS,
                                 TRACK_MOTOR_KP,
                                 TRACK_MOTOR_KI,
                                 TRACK_MOTOR_KD,
                                 TRACK_SERVO_KP,
                                 TRACK_SERVO_KD);
        break;

    case STATE_AVOID_STONE:
        control_set_motion_param(AVOID_STONE_DEFAULT_SPEED_MPS,
                                 AVOID_STONE_MOTOR_KP,
                                 AVOID_STONE_MOTOR_KI,
                                 AVOID_STONE_MOTOR_KD,
                                 AVOID_STONE_SERVO_KP,
                                 AVOID_STONE_SERVO_KD);
        break;

    case STATE_AVOID_CAR:
        control_set_motion_param(AVOID_CAR_DEFAULT_SPEED_MPS,
                                 AVOID_CAR_MOTOR_KP,
                                 AVOID_CAR_MOTOR_KI,
                                 AVOID_CAR_MOTOR_KD,
                                 AVOID_CAR_SERVO_KP,
                                 AVOID_CAR_SERVO_KD);
        break;

    case STATE_AVOID_HUMAN:
        control_set_motion_param(AVOID_HUMAN_DEFAULT_SPEED_MPS,
                                 AVOID_HUMAN_MOTOR_KP,
                                 AVOID_HUMAN_MOTOR_KI,
                                 AVOID_HUMAN_MOTOR_KD,
                                 AVOID_HUMAN_SERVO_KP,
                                 AVOID_HUMAN_SERVO_KD);
        break;

    case STATE_COLLECT_GOLD:
        control_set_motion_param(COLLECT_GOLD_DEFAULT_SPEED_MPS,
                                 COLLECT_GOLD_MOTOR_KP,
                                 COLLECT_GOLD_MOTOR_KI,
                                 COLLECT_GOLD_MOTOR_KD,
                                 COLLECT_GOLD_SERVO_KP,
                                 COLLECT_GOLD_SERVO_KD);
        break;

    case STATE_RECOVER_LINE:
        control_set_motion_param(RECOVER_LINE_DEFAULT_SPEED_MPS,
                                 RECOVER_LINE_MOTOR_KP,
                                 RECOVER_LINE_MOTOR_KI,
                                 RECOVER_LINE_MOTOR_KD,
                                 RECOVER_LINE_SERVO_KP,
                                 RECOVER_LINE_SERVO_KD);
        break;

    case STATE_LINE_LOSS_SAFE_STOP:
    case STATE_TRAFFIC_LIGHT_STOP:
    case STATE_ENDSIGN_STOP:
    case STATE_SAFE_STOP:
        control_ctx.param.motor_target_speed = 0.0f;
        control_ctx.param.motor_kp = 0.0f;
        control_ctx.param.motor_ki = 0.0f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -(float)MOTOR_PWM_DUTY_LIMIT;
        control_ctx.param.motor_output_max = (float)MOTOR_PWM_DUTY_LIMIT;
        control_ctx.param.servo_kp = 0.0f;
        control_ctx.param.servo_kd = 0.0f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;

    case STATE_IDLE:
    default:
        gpio_set_level(P21_5, 0);
        control_ctx.param.motor_target_speed = 0.0f;
        control_ctx.param.motor_kp = 2.0f;
        control_ctx.param.motor_ki = 1.0f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -(float)MOTOR_PWM_DUTY_LIMIT;
        control_ctx.param.motor_output_max = (float)MOTOR_PWM_DUTY_LIMIT;
        control_ctx.param.servo_kp = 1.0f;
        control_ctx.param.servo_kd = 1.0f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;
    }

    pid_incr_init(&control_ctx.motor_pid,
                  control_ctx.param.motor_kp,
                  control_ctx.param.motor_ki,
                  control_ctx.param.motor_kd,
                  control_ctx.param.motor_output_min,
                  control_ctx.param.motor_output_max);

    pid_pstn_init(&control_ctx.servo_pid,
                  control_ctx.param.servo_kp,
                  0.0f,
                  control_ctx.param.servo_kd,
                  control_ctx.param.servo_output_min,
                  control_ctx.param.servo_output_max,
                  0.0f);
}

static void Motor_PID_Control(void)
{
    float pid_out;
    float feedforward;
    float target_abs;
    float output;
    int32 duty;

    control_ctx.actual_speed = MOTOR_ENCODER_SIGN * ((float)motor_get_speed()) * MOTOR_ENCODER_COUNT_TO_MPS;


    control_ctx.motor_target = control_ctx.param.motor_target_speed;
    if ((control_ctx.input.flags & CONTROL_FLAG_USE_TARGET_SPEED) && control_allow_speed_override(control_ctx.current_state))
    {
        control_ctx.motor_target = control_ctx.input.target_speed;
    }

    target_abs = control_abs_float(control_ctx.motor_target);
    if (target_abs < MOTOR_TARGET_DEADBAND_MPS)
    {
        pid_incr_reset(&control_ctx.motor_pid);
        control_ctx.motor_output = 0;
        motor_set_duty(0);
        return;
    }

    pid_out = pid_incr_calc(&control_ctx.motor_pid, control_ctx.motor_target, control_ctx.actual_speed);
    pid_out = control_limit_float(pid_out, MOTOR_PID_CORRECTION_LIMIT, -MOTOR_PID_CORRECTION_LIMIT);

    feedforward = MOTOR_FEEDFORWARD_BASE_DUTY + MOTOR_FEEDFORWARD_SPEED_GAIN * target_abs;
    feedforward = control_limit_float(feedforward, MOTOR_FEEDFORWARD_MAX_DUTY, MOTOR_FEEDFORWARD_BASE_DUTY);
    if (control_ctx.motor_target < 0.0f)
    {
        feedforward = -feedforward;
    }

    output = feedforward + pid_out;
    output = control_limit_float(output, (float)MOTOR_PWM_DUTY_LIMIT, -(float)MOTOR_PWM_DUTY_LIMIT);

    duty = (int32)output;
    control_ctx.motor_output = duty;

    motor_set_duty(duty);
}

static void Servo_PID_Control(void)
{
    float pid_out;
    int32 duty;

    control_ctx.servo_target = 0.0f;

    pid_out = pid_pd_calc(&control_ctx.servo_pid, control_ctx.servo_target, control_ctx.input.track_error);
    pid_out += control_track_servo_large_error_boost(control_ctx.current_state, control_ctx.input.track_error);
    pid_out = control_limit_float(pid_out, control_ctx.param.servo_output_max, control_ctx.param.servo_output_min);

    duty = (int32)((float)SERVO_DUTY_MID + pid_out);
    duty = (int32)control_limit_uint32(duty, SERVO_DUTY_MAX, SERVO_DUTY_MIN);
    control_ctx.servo_output = (uint32)duty;

    servo_set_duty((uint32)duty);
}

void control_update(void)
{
    control_check_input_timeout();

    if (state_is_changed())
    {
        control_apply_state_param(state_get());
        state_clear_changed();
    }

    Motor_PID_Control();
    Servo_PID_Control();
}

control_ctx_struct *control_get_ctx(void)
{
    return &control_ctx;
}

void pit_set_and_enable(uint32 time)
{
    pit_init(CCU60_CH0, 1000);
    pit_enable(CCU60_CH0);
    pit_start(CCU60_CH0);
    control_ctx.periodic_interrupt_flag = 0;
    control_ctx.interrupt_count = time;
    control_ctx.interrupt_count_hold = time;
}
