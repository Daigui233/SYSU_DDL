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

#ifndef CODE_PID_H_
#define CODE_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "zf_common_headfile.h"

/*********************************************************************************************************************
*                                               位置式 PID 结构体
*********************************************************************************************************************/
typedef struct
{
    float kp;
    float ki;
    float kd;

    float targ;
    float now;

    float error;
    float last_error;
    float integral;
    float output;

    float output_min;
    float output_max;
    float integral_max;
} pid_pstn_struct;

/*********************************************************************************************************************
*                                               增量式 PID 结构体
*********************************************************************************************************************/
typedef struct
{
    float kp;
    float ki;
    float kd;

    float targ;
    float now;

    float error;
    float last_error;
    float last_last_error;
    float output;

    float output_min;
    float output_max;
} pid_incr_struct;

/* 位置式 PID 接口（当前保留用于学习与实车对比） */
void  pid_pstn_init          (pid_pstn_struct *pid, float kp, float ki, float kd, float output_min, float output_max, float integral_max);
float pid_pstn_calc          (pid_pstn_struct *pid, float targ, float now);
float pid_pd_calc            (pid_pstn_struct *pid, float targ, float now);
void  pid_pstn_reset         (pid_pstn_struct *pid);

/* 增量式 PID 接口（当前电机速度环默认方案） */
void  pid_incr_init          (pid_incr_struct *pid, float kp, float ki, float kd, float output_min, float output_max);
float pid_incr_calc          (pid_incr_struct *pid, float targ, float now);
void  pid_incr_reset         (pid_incr_struct *pid);

/* 兼容当前阶段已有调用名 */
void  pid_reset              (pid_pstn_struct *pid);

#ifdef __cplusplus
}
#endif

#endif

