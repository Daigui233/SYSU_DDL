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

#include "Control.h"

control_ctx_struct control_ctx;

/*********************************************************************************************************************
 * 函数名称          control_limit_uint32
 * 功能说明          对舵机最终输出占空比进行限幅
 * 参数说明          input           输入量
 *                   max             最大值
 *                   min             最小值
 * 返回参数          uint32          限幅后的结果
 * 使用示例          duty = control_limit_uint32(duty, SERVO_DUTY_MAX, SERVO_DUTY_MIN);
 * 备注信息          本函数为 Control 模块内部静态函数
 *********************************************************************************************************************/
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

/*********************************************************************************************************************
 * 函数名称          control_allow_speed_override
 * 功能说明          判断当前状态是否允许上位机直接覆盖目标速度
 * 参数说明          state           当前状态
 * 返回参数          uint8           1 表示允许 0 表示不允许
 * 使用示例          if(control_allow_speed_override(state))
 * 备注信息          WAIT_LIGHT SAFE_STOP IDLE 等停车状态下禁止被目标速度覆盖顶掉
 *********************************************************************************************************************/
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

/*********************************************************************************************************************
 * 函数名称          control_reset_input_to_safe
 * 功能说明          将控制输入复位为安全默认值
 * 参数说明          无
 * 返回参数          无
 * 使用示例          control_reset_input_to_safe();
 * 备注信息          输入超时后清空速度覆盖 误差与标志位 避免继续沿用旧输入
 *********************************************************************************************************************/
static void control_reset_input_to_safe(void)
{
    control_ctx.input.target_speed = 0.0f;
    control_ctx.input.track_error = 0.0f;
    control_ctx.input.state_cmd = (uint8)STATE_SAFE_STOP;
    control_ctx.input.flags = 0;
}

/*********************************************************************************************************************
 * 函数名称          control_handle_input_timeout
 * 功能说明          处理上位机输入心跳超时
 * 参数说明          无
 * 返回参数          无
 * 使用示例          control_handle_input_timeout();
 * 备注信息          超时后自动切换至安全状态 便于后续三圈停车或失联保护
 *********************************************************************************************************************/
static void control_handle_input_timeout(void)
{
    uint32 now_time;

    if (0 == control_ctx.input_online)
    {
        return;
    }

    now_time = system_getval_us();
    if ((uint32)(now_time - control_ctx.last_input_time_us) >= CONTROL_INPUT_TIMEOUT_US)
    {
        control_ctx.input_online = 0;
        control_reset_input_to_safe();
        state_set(STATE_SAFE_STOP);
    }
}

/*********************************************************************************************************************
 * 函数名称          control_init
 * 功能说明          控制上下文初始化函数
 * 参数说明          无
 * 返回参数          无
 * 使用示例          control_init();
 * 注意事项          1. 此处沿用原始 Control 的初始化思路 但对象已整理为当前工程的控制上下文
 *                   2. 电机默认采用增量式 PID 舵机默认采用 PD
 *********************************************************************************************************************/
void control_init(void)
{
    /*
     * control_ctx 为全局变量 上电后默认已完成零初始化
     * 此处不再显式整体清零 避免引入当前工程中不存在的 zf_memset 接口
     */
    control_reset_input_to_safe();
    control_ctx.last_input_time_us = 0;
    control_ctx.input_online = 0;
    control_ctx.current_state = state_get();
    control_apply_state_param(control_ctx.current_state);
}

/*********************************************************************************************************************
 * 函数名称          control_set_input
 * 功能说明          设置控制输入数据
 * 参数说明          input           控制输入结构体
 * 返回参数          无
 * 使用示例          control_set_input(rx_input);
 * 注意事项          1. 后续通信模块可将上位机解包结果直接写入本接口
 *********************************************************************************************************************/
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

/*********************************************************************************************************************
 * 函数名称          control_get_input
 * 功能说明          获取当前控制输入数据
 * 参数说明          无
 * 返回参数          control_input_struct 当前控制输入结构体
 * 使用示例          input = control_get_input();
 * 注意事项          1. 便于后续调试或通信回传时查看当前输入快照
 *********************************************************************************************************************/
control_input_struct control_get_input(void)
{
    return control_ctx.input;
}

/*********************************************************************************************************************
 * 函数名称          control_apply_state_param
 * 功能说明          按当前状态装载对应控制参数与 PID 参数
 * 参数说明          state           当前状态
 * 返回参数          无
 * 使用示例          control_apply_state_param(STATE_TRACK);
 * 注意事项          1. 当前阶段将各状态控制参数集中保存在 Control 模块内部
 *                   2. PID 模块仅保留算法职责 不直接持有状态参数
 *********************************************************************************************************************/
void control_apply_state_param(car_state_enum state)
{
    control_ctx.current_state = state;

    switch (state)
    {
    case STATE_TRACK:
        control_ctx.param.motor_target_speed = 120.0f;
        control_ctx.param.motor_kp = 8.0f;
        control_ctx.param.motor_ki = 0.2f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
        control_ctx.param.servo_kp = 3.5f;
        control_ctx.param.servo_kd = 1.2f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;

    case STATE_LIMIT_SPEED:
        control_ctx.param.motor_target_speed = 80.0f;
        control_ctx.param.motor_kp = 8.0f;
        control_ctx.param.motor_ki = 0.2f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
        control_ctx.param.servo_kp = 3.0f;
        control_ctx.param.servo_kd = 1.2f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;

    case STATE_WAIT_LIGHT:
    case STATE_SAFE_STOP:
        control_ctx.param.motor_target_speed = 0.0f;
        control_ctx.param.motor_kp = 8.0f;
        control_ctx.param.motor_ki = 0.2f;
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
        control_ctx.param.motor_target_speed = 90.0f;
        control_ctx.param.motor_kp = 8.0f;
        control_ctx.param.motor_ki = 0.2f;
        control_ctx.param.motor_kd = 0.0f;
        control_ctx.param.motor_output_min = -10000.0f;
        control_ctx.param.motor_output_max = 10000.0f;
        control_ctx.param.servo_kp = 4.0f;
        control_ctx.param.servo_kd = 1.4f;
        control_ctx.param.servo_output_min = (float)(SERVO_DUTY_MIN - SERVO_DUTY_MID);
        control_ctx.param.servo_output_max = (float)(SERVO_DUTY_MAX - SERVO_DUTY_MID);
        break;

    case STATE_IDLE:
    default:
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

/*********************************************************************************************************************
 * 函数名称          Motor_PID_Control
 * 功能说明          增量式PID控制电机输出
 * 参数说明          无
 * 返回参数          无
 * 使用示例          Motor_PID_Control();
 * 注意事项          1.不要直接使用motor_set_duty，请使用封装好的电机PID驱动函数
 *                   2.当前阶段使用 motor_get_speed() 直接作为速度反馈
 *                   3.电机默认采用增量式PID
 *********************************************************************************************************************/
static void Motor_PID_Control(void)
{
    float pid_out;
    int32 duty;

    /* 获取速度 */
    control_ctx.actual_speed = (float)motor_get_speed();

    control_ctx.motor_target = control_ctx.param.motor_target_speed;
    if ((control_ctx.input.flags & CONTROL_FLAG_USE_TARGET_SPEED) && control_allow_speed_override(control_ctx.current_state))
    {
        control_ctx.motor_target = control_ctx.input.target_speed;
    }

    /* PID计算（增量式） */
    pid_out = pid_incr_calc(&control_ctx.motor_pid, control_ctx.motor_target, control_ctx.actual_speed);

    /* 转换为PWM占空比 */
    duty = (int32)pid_out;
    control_ctx.motor_output = duty;

    /* 输出到电机 */
    motor_set_duty(duty);
}

/*********************************************************************************************************************
 * 函数名称          Servo_PID_Control
 * 功能说明          PD控制舵机输出
 * 参数说明          无
 * 返回参数          无
 * 使用示例          Servo_PID_Control();
 * 注意事项          1.不要直接使用servo_set_duty，请使用封装好的舵机PD驱动函数
 *                   2.当前阶段舵机使用赛道误差作为反馈 目标值固定为 0
 *********************************************************************************************************************/
static void Servo_PID_Control(void)
{
    float pid_out;
    int32 duty;

    control_ctx.servo_target = 0.0f;

    /* 1. PD计算 */
    pid_out = pid_pd_calc(&control_ctx.servo_pid, control_ctx.servo_target, control_ctx.input.track_error);

    /* 2. 转换为PWM占空比 */
    duty = (int32)((float)SERVO_DUTY_MID + pid_out);
    duty = (int32)control_limit_uint32(duty, SERVO_DUTY_MAX, SERVO_DUTY_MIN);
    control_ctx.servo_output = (uint32)duty;

    /* 3. 输出到舵机 */
    servo_set_duty((uint32)duty);
}

/*********************************************************************************************************************
 * 函数名称          control_update
 * 功能说明          控制模块周期更新函数
 * 参数说明          无
 * 返回参数          无
 * 使用示例          control_update();
 * 注意事项          1. 后续主循环或定时中断中可周期性调用本接口
 *                   2. 若状态发生切换 本函数将重载对应参数与 PID
 *********************************************************************************************************************/
void control_update(void)
{
    control_handle_input_timeout();

    if (state_is_changed())
    {
        control_apply_state_param(state_get());
        state_clear_changed();
    }

    Motor_PID_Control();
    Servo_PID_Control();
}

/*********************************************************************************************************************
 * 函数名称          control_get_ctx
 * 功能说明          获取控制上下文结构体指针
 * 参数说明          无
 * 返回参数          control_ctx_struct* 控制上下文结构体指针
 * 使用示例          ctx = control_get_ctx();
 * 注意事项          1. 后续调试或通信回传时可通过本接口读取当前控制结果与参数快照
 *********************************************************************************************************************/
control_ctx_struct *control_get_ctx(void)
{
    return &control_ctx;
}

/*********************************************************************************************************************
 * 函数名称          pit_set_and_enable
 * 功能说明          初始化并且使能周期中断
 * 参数说明          中断时间 单位ms
 * 返回参数          无
 * 使用示例          pit_set_and_enable(5000);
 * 注意事项
 *********************************************************************************************************************/
void pit_set_and_enable(uint32 time)
{
    pit_init(CCU60_CH0, 1000);
    pit_enable(CCU60_CH0);
    control_ctx.periodic_interrupt_flag = 0;
    control_ctx.interrupt_count = time;
    control_ctx.interrupt_count_hold = time;
}
