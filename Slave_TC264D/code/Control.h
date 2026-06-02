/*********************************************************************************************************************
 * ???????          Control
 * ????????          ?????? DDL_?????
 * ??????          TC264D
 *
 * ?????
 * ????              ????                ???
 * 2026-04-28       ljr                 ????
 * 2026-04-29       Daigui              ??????????????? Control ???
 *********************************************************************************************************************/

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "zf_common_headfile.h"
#include "State.h"
#include "Motor.h"
#include "Servo.h"
#include "PID.h"
#include "Communication.h"

#define CONTROL_FLAG_USE_TARGET_SPEED (0x01U) // ?????��???��????????????????????

    /*********************************************************************************************************************
     *                                               ???????????
     *********************************************************************************************************************/
    typedef struct control_input_struct
    {
        float target_speed;
        float track_error;
        uint8 state_cmd;
        uint8 flags;
    } control_input_struct;

    /*********************************************************************************************************************
     *                                               ?????????????
     *********************************************************************************************************************/
    typedef struct
    {
        float motor_target_speed;

        float motor_kp;
        float motor_ki;
        float motor_kd;
        float motor_output_min;
        float motor_output_max;

        float servo_kp;
        float servo_kd;
        float servo_output_min;
        float servo_output_max;
    } control_param_struct;

    /*********************************************************************************************************************
     *                                               ?????????????
     *********************************************************************************************************************/
    typedef struct
    {
        car_state_enum current_state;

        control_input_struct input;
        control_param_struct param;

        float actual_speed;
        float motor_target;
        float servo_target;

        int32 motor_output;
        uint32 servo_output;

        uint32 last_input_time_us;
        uint8 input_online;

        pid_incr_struct motor_pid;
        pid_pstn_struct servo_pid;

        // ?????��???��
        uint8 periodic_interrupt_flag;

        // ?��????
        uint32 interrupt_count;

        // ?��????????
        uint32 interrupt_count_hold;

    } control_ctx_struct;

    extern control_ctx_struct control_ctx;

    // ??????????
    extern control_input_struct input_communication_temp;

    void control_init(void);
    void control_set_input(control_input_struct input);
    control_input_struct control_get_input(void);
    void control_apply_state_param(car_state_enum state);
    void control_update(void);
    control_ctx_struct *control_get_ctx(void);

    void pit_set_and_enable(uint32 period);

#ifdef __cplusplus
}
#endif

#endif
