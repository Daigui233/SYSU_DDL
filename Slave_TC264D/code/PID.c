/*********************************************************************************************************************
* 文件名称          PID
* 车队名称          中山大学 DDL_大电流
* 开发平台          TC264D
*
* 修改记录
* 日期              作者                备注
* 2026-04-28       ljr                 初版
* 2026-04-28       Daigui              整理为当前阶段可用的 PID 模块
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
*                   output_max      输出限幅
*                   integral_max    积分限幅
* 返回参数          无
* 使用示例          pid_pstn_init(&motor_pid, 1.0f, 0.0f, 0.0f, 6000.0f, 1000.0f);
* 备注信息          当前阶段电机速度环可优先使用本接口初始化
*********************************************************************************************************************/
void pid_pstn_init (pid_pstn_struct *pid, float kp, float ki, float kd, float output_max, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_max = output_max;
    pid->integral_max = integral_max;

    pid_reset(pid);
}

/*********************************************************************************************************************
* 函数名称          pid_pstn_calc
* 功能说明          位置式 PID 控制计算函数
* 参数说明          pid             PID 结构体指针
*                   targ            目标值
*                   now             当前反馈值
* 返回参数          float           PID 输出结果
* 使用示例          out = pid_pstn_calc(&motor_pid, target_speed, actual_speed);
* 备注信息          内部包含积分累加与输出限幅
*                  更适合当前阶段电机速度环等存在稳定反馈的场景
*********************************************************************************************************************/
float pid_pstn_calc (pid_pstn_struct *pid, float targ, float now)
{
    float derivative;

    pid->error = targ - now;
    pid->integral += pid->error;
    pid->integral = pid_limit(pid->integral, pid->integral_max, -pid->integral_max);

    derivative = pid->error - pid->last_error;
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * derivative;
    pid->output = pid_limit(pid->output, pid->output_max, -pid->output_max);

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
* 备注信息          当前阶段舵机反馈来自路径误差而非舵机自身编码器
*                  因此优先保留 PD 形式 避免过早引入积分累计
*********************************************************************************************************************/
float pid_pd_calc (pid_pstn_struct *pid, float targ, float now)
{
    float derivative;

    pid->error = targ - now;
    derivative = pid->error - pid->last_error;

    pid->output = pid->kp * pid->error + pid->kd * derivative;
    pid->output = pid_limit(pid->output, pid->output_max, -pid->output_max);

    pid->last_error = pid->error;

    return pid->output;
}

/*********************************************************************************************************************
* 函数名称          pid_reset
* 功能说明          清零 PID 中间变量
* 参数说明          pid             PID 结构体指针
* 返回参数          无
* 使用示例          pid_reset(&motor_pid);
* 备注信息          建议在切换模式 急停 或重新启动控制环时调用
*********************************************************************************************************************/
void pid_reset (pid_pstn_struct *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
