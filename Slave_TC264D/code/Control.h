/*********************************************************************************************************************
 * 文件名称          Control
 * 文件说明          SYSU_DDL 下位机控制模块
 * 适用平台          TC264D
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-04-28       ljr                 创建
 * 2026-04-29       Daigui              接入通信、状态和控制链路
 * 2026-06-03       Codex               整理控车反馈并保持 GBK 编码
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

#define CONTROL_FLAG_USE_TARGET_SPEED (0x01U) // 使用上位机下发目标速度
#define CONTROL_INPUT_TIMEOUT_US (500000U)    // Valid control-frame timeout for TC264D local failsafe.

#define CONTROL_FB_FLAG_INPUT_TIMEOUT      (0x0001U)
#define CONTROL_FB_FLAG_STOP_STATE         (0x0002U)
#define CONTROL_FB_FLAG_SERVO_SATURATED    (0x0004U)
#define CONTROL_FB_FLAG_MOTOR_SATURATED    (0x0008U)
#define CONTROL_FB_FLAG_TARGET_DEADBAND    (0x0010U)

    /*********************************************************************************************************************
     *                                               控制输入结构体
     *********************************************************************************************************************/
    typedef struct control_input_struct
    {
        float target_speed;
        float track_error;
        uint8 state_cmd;
        uint8 flags;
    } control_input_struct;

    /*********************************************************************************************************************
     *                                               状态参数结构体
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
     *                                               控制上下文结构体
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

        float motor_feedforward;
        float motor_pid_correction;
        float servo_raw_output;
        float servo_limited_output;
        uint16 safety_flags;
        uint32 feedback_seq;
        uint32 last_input_time_us;
        uint8 input_online;

        pid_incr_struct motor_pid;
        pid_pstn_struct servo_pid;

        // 控制周期标志。
        volatile uint8 periodic_interrupt_flag;

        // 周期计数。
        volatile uint32 interrupt_count;

        // 周期计数保持值。
        volatile uint32 interrupt_count_hold;

    } control_ctx_struct;

    extern control_ctx_struct control_ctx;

    // 串口解码后的临时控制输入。
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