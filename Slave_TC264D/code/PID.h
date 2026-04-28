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

#ifndef CODE_PID_H_
#define CODE_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "zf_common_headfile.h"

typedef struct
{
    float kp;
    float ki;
    float kd;

    float error;
    float last_error;
    float integral;
    float output;

    float output_max;
    float integral_max;
} pid_pstn_struct;

void  pid_pstn_init          (pid_pstn_struct *pid, float kp, float ki, float kd, float output_max, float integral_max);
float pid_pstn_calc          (pid_pstn_struct *pid, float targ, float now);
float pid_pd_calc            (pid_pstn_struct *pid, float targ, float now);
void  pid_reset              (pid_pstn_struct *pid);

#ifdef __cplusplus
}
#endif

#endif
