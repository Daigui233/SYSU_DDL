/*********************************************************************************************************************
 * 文件名称          Control
 * 车队名称          中山大学 DDL_大电流
 * 开发平台          TC264D
 *
 * 修改记录
 * 日期              作者                备注
 * 2026-04-28       ljr                 初版
 * 2026-04-29       Daigui              基于当前工程结构整理 Control 模块
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

#define CONTROL_FLAG_USE_TARGET_SPEED (0x01U) // 使用上位机下发目标速度覆盖当前状态默认速度
#define CONTROL_INPUT_TIMEOUT_US (3000000U)   // 上位机输入超时阈值 当前按 3s 处理 参考 sasu-intelligentcar-kits/[05] 上下位机通信协议

    /*********************************************************************************************************************
     *                                               控制输入结构体
     *********************************************************************************************************************/
    typedef struct
    {
        float target_speed;
        float track_error;
        uint8 state_cmd;
        uint8 flags;
    } control_input_struct;

    /*********************************************************************************************************************
     *                                               状态控制参数结构体
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

        uint32 last_input_time_us;
        uint8 input_online;

        pid_incr_struct motor_pid;
        pid_pstn_struct servo_pid;

        // 周期中断标志位
        uint8 periodic_interrupt_flag;

        // 中断计数
        uint32 interrupt_count;

        // 中断计数保持
        uint32 interrupt_count_hold;

    } control_ctx_struct;

    extern control_ctx_struct control_ctx;

    void control_init(void);
    void control_set_input(control_input_struct input);
    control_input_struct control_get_input(void);
    void control_apply_state_param(car_state_enum state);
    void control_update(void);
    control_ctx_struct *control_get_ctx(void);

#ifdef __cplusplus
}
#endif

#endif
