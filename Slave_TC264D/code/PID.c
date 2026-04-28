/*********************************************************************************************************************
* 文件名称          PID
* 车队名称          中山大学 DDL_大电流
* 开发平台          TC264D
*
* 修改记录
* 日期              作者                备注
* 2026-04-28       ljr                 初版
* 2026-04-28       Daigui              整理为当前阶段可用的 PID 模块
* 2026-04-28       Daigui              补充增量式 PID 接口并统一当前工程命名
*********************************************************************************************************************/

#include "PID.h"

/*********************************************************************************************************************
* 函数名称          pid_limit
* 功能说明          对 PID 中间量进行限幅
* 参数说明          input           输入量
*                   max             最大值
*                   min             最小值
* 返回参数          float           限幅后的结果
* 使用示例          pid_limit(out, 1000, -1000);
* 备注信息          本函数为 PID 模块内部静态函数
*********************************************************************************************************************/
static float pid_limit (float input, float max, float min)
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

/*********************************************************************************************************************
* 函数名称          pid_pstn_init
* 功能说明          初始化位置式 PID 结构体参数
* 参数说明          pid             PID 结构体指针
*                   kp              比例系数
*                   ki              积分系数
*                   kd              微分系数
*                   output_min      输出下限
*                   output_max      输出上限
*                   integral_max    积分限幅
* 返回参数          无
* 使用示例          pid_pstn_init(&motor_pid, 1.0f, 0.0f, 0.0f, -6000.0f, 6000.0f, 1000.0f);
* 备注信息          位置式 PID 当前主要保留用于学习 理解与后续和增量式做实车对比
*********************************************************************************************************************/
void pid_pstn_init (pid_pstn_struct *pid, float kp, float ki, float kd, float output_min, float output_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_max = integral_max;

    pid_pstn_reset(pid);
}

/*********************************************************************************************************************
* 函数名称          pid_pstn_calc
* 功能说明          位置式 PID 控制计算函数
* 参数说明          pid             PID 结构体指针
*                   targ            目标值
*                   now             当前反馈值
* 返回参数          float           PID 输出结果
* 使用示例          out = pid_pstn_calc(&motor_pid, target_speed, actual_speed);
* 备注信息          输出值由 P I D 三项直接相加得到 当前电机速度环默认不优先使用本形式
*********************************************************************************************************************/
float pid_pstn_calc (pid_pstn_struct *pid, float targ, float now)
{
    float derivative;

    pid->targ = targ;
    pid->now = now;

    pid->error = pid->targ - pid->now;
    pid->integral += pid->error;
    pid->integral = pid_limit(pid->integral, pid->integral_max, -pid->integral_max);

    derivative = pid->error - pid->last_error;
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;
    pid->output = pid_limit(pid->output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;

    return pid->output;
}

/*********************************************************************************************************************
* 函数名称          pid_pd_calc
* 功能说明          PD 控制计算函数
* 参数说明          pid             PID 结构体指针
*                   targ            目标值
*                   now             当前反馈值
* 返回参数          float           PD 输出结果
* 使用示例          out = pid_pd_calc(&servo_pid, servo_target, track_error);
* 备注信息          当前阶段舵机默认使用 PD 形式 避免过早引入积分累计
*********************************************************************************************************************/
float pid_pd_calc (pid_pstn_struct *pid, float targ, float now)
{
    float derivative;

    pid->targ = targ;
    pid->now = now;

    pid->error = pid->targ - pid->now;
    derivative = pid->error - pid->last_error;

    pid->output = pid->kp * pid->error + pid->kd * derivative;
    pid->output = pid_limit(pid->output, pid->output_max, pid->output_min);

    pid->last_error = pid->error;

    return pid->output;
}

/*********************************************************************************************************************
* 函数名称          pid_pstn_reset
* 功能说明          清零位置式 PID 中间变量
* 参数说明          pid             PID 结构体指针
* 返回参数          无
* 使用示例          pid_pstn_reset(&motor_pid);
* 备注信息          建议在切换模式 急停 或重新启动控制环时调用
*********************************************************************************************************************/
void pid_pstn_reset (pid_pstn_struct *pid)
{
    pid->targ = 0.0f;
    pid->now = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/*********************************************************************************************************************
* 函数名称          pid_incr_init
* 功能说明          初始化增量式 PID 结构体参数
* 参数说明          pid             PID 结构体指针
*                   kp              比例系数
*                   ki              积分系数
*                   kd              微分系数
*                   output_min      输出下限
*                   output_max      输出上限
* 返回参数          无
* 使用示例          pid_incr_init(&motor_pid, 1.0f, 0.0f, 0.0f, -6000.0f, 6000.0f);
* 备注信息          增量式 PID 为当前电机速度环默认方案 后续可再与位置式做实车对比
*********************************************************************************************************************/
void pid_incr_init (pid_incr_struct *pid, float kp, float ki, float kd, float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;

    pid_incr_reset(pid);
}

/*********************************************************************************************************************
* 函数名称          pid_incr_calc
* 功能说明          增量式 PID 控制计算函数
* 参数说明          pid             PID 结构体指针
*                   targ            目标值
*                   now             当前反馈值
* 返回参数          float           PID 输出结果
* 使用示例          out = pid_incr_calc(&motor_pid, target_speed, actual_speed);
* 备注信息          本函数先计算本周期输出增量 再累加为新的控制输出 当前阶段电机速度环默认使用本函数
*********************************************************************************************************************/
float pid_incr_calc (pid_incr_struct *pid, float targ, float now)
{
    float delta_output;

    pid->targ = targ;
    pid->now = now;

    pid->error = pid->targ - pid->now;

    delta_output = pid->kp * (pid->error - pid->last_error)
                 + pid->ki * pid->error
                 + pid->kd * (pid->error - 2.0f * pid->last_error + pid->last_last_error);

    pid->output += delta_output;
    pid->output = pid_limit(pid->output, pid->output_max, pid->output_min);

    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

/*********************************************************************************************************************
* 函数名称          pid_incr_reset
* 功能说明          清零增量式 PID 中间变量
* 参数说明          pid             PID 结构体指针
* 返回参数          无
* 使用示例          pid_incr_reset(&motor_pid);
* 备注信息          建议在切换模式 急停 或重新启动控制环时调用
*********************************************************************************************************************/
void pid_incr_reset (pid_incr_struct *pid)
{
    pid->targ = 0.0f;
    pid->now = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->last_last_error = 0.0f;
    pid->output = 0.0f;
}

/*********************************************************************************************************************
* 函数名称          pid_reset
* 功能说明          兼容当前阶段已有调用名的复位接口
* 参数说明          pid             PID 结构体指针
* 返回参数          无
* 使用示例          pid_reset(&motor_pid);
* 备注信息          当前内部等效调用 pid_pstn_reset()
*********************************************************************************************************************/
void pid_reset (pid_pstn_struct *pid)
{
    pid_pstn_reset(pid);
}

