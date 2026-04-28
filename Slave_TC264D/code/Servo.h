/*********************************************************************************************************************
* 文件名称          Servo
* 车队名称          中山大学 DDL_大电流
* 开发平台          TC264D
*
* 修改记录
* 日期              作者                备注
* 2026-04-28       Daigui              初版
*********************************************************************************************************************/

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_

#include "zf_common_headfile.h"

//====================================================舵机硬件参数定义====================================================
#define SERVO_PWM_FREQ               (50)                            // 舵机 PWM 频率 常用模拟舵机工作频率为 50Hz
#define SERVO_PWM_PIN                (ATOM0_CH7_P33_9)              // 当前板级推荐 IO 为 P33_9

#define SERVO_DUTY_MAX               (910)                           // 舵机最大占空比 需结合实车限位调试
#define SERVO_DUTY_MID               (730)                           // 舵机中值占空比 需结合实车回中调试
#define SERVO_DUTY_MIN               (570)                           // 舵机最小占空比 需结合实车限位调试
//====================================================舵机硬件参数定义====================================================

typedef struct
{
    uint32 duty;                                                      // 当前舵机输出占空比
} servo_info_struct;

extern servo_info_struct servo_info;

void  servo_init              (void);
void  servo_set_duty          (uint32 duty);
void  servo_center            (void);

#endif
