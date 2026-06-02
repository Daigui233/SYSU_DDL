
#include "Control.h"

control_ctx_struct control_ctx;

control_input_struct input_communication_temp;

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
    case STATE_LIMIT_SPEED:
    case STATE_AVOID:
    case STATE_NAV_LEFT:
    case STATE_NAV_RIGHT:
        return 1;

    case STATE_WAIT_LIGHT:
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

void control_apply_state_param(car_state_enum state)
{
    control_ctx.current_state = state;

    switch (state)
    {
    case STATE_TRACK:
        control_ctx.param.motor_target_speed = 100.0f;
        control_ctx.param.motor_kp = 2.0f;
        control_ctx.param.motor_ki = 1.0f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
        control_ctx.param.servo_kp = 1.0f;
        control_ctx.param.servo_kd = 1.0f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;

    case STATE_LIMIT_SPEED:
        control_ctx.param.motor_target_speed = 0.0f;
        control_ctx.param.motor_kp = 0.0f;
        control_ctx.param.motor_ki = 0.0f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
        control_ctx.param.servo_kp = 0.0f;
        control_ctx.param.servo_kd = 0.0f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;

    case STATE_WAIT_LIGHT:
    case STATE_SAFE_STOP:
        control_ctx.param.motor_target_speed = 0.0f;
        control_ctx.param.motor_kp = 0.0f;
        control_ctx.param.motor_ki = 0.0f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
        control_ctx.param.servo_kp = 0.0f;
        control_ctx.param.servo_kd = 0.0f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;

    case STATE_AVOID:
    case STATE_NAV_LEFT:
    case STATE_NAV_RIGHT:
        control_ctx.param.motor_target_speed = 0.0f;
        control_ctx.param.motor_kp = 0.0f;
        control_ctx.param.motor_ki = 0.0f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
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
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
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
    int32 duty;

    control_ctx.actual_speed = ((float)motor_get_speed()) / 38.0 * 5.0;


    control_ctx.motor_target = control_ctx.param.motor_target_speed;
    if ((control_ctx.input.flags & CONTROL_FLAG_USE_TARGET_SPEED) && control_allow_speed_override(control_ctx.current_state))
    {
        control_ctx.motor_target = control_ctx.input.target_speed;
    }

    pid_out = pid_incr_calc(&control_ctx.motor_pid, control_ctx.motor_target, control_ctx.actual_speed);

    duty = (int32)pid_out;
    control_ctx.motor_output = duty;

    motor_set_duty(duty);
}

static void Servo_PID_Control(void)
{
    float pid_out;
    int32 duty;

    control_ctx.servo_target = 0.0f;

    pid_out = pid_pd_calc(&control_ctx.servo_pid, control_ctx.servo_target, control_ctx.input.track_error);


    duty = (int32)((float)SERVO_DUTY_MID + pid_out);
    duty = (int32)control_limit_uint32(duty, SERVO_DUTY_MAX, SERVO_DUTY_MIN);
    control_ctx.servo_output = (uint32)duty;

    servo_set_duty((uint32)duty);
}

void control_update(void)
{

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
