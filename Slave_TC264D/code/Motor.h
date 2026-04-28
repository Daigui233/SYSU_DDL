/*********************************************************************************************************************
* 文件名称          Motor
* 车队名称          中山大学 DDL_大电流
* 开发平台          TC264D
*
* 修改记录
* 日期              作者                备注
* 2026-04-28       Daigui              初版
*********************************************************************************************************************/

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"

//====================================================电机硬件参数定义====================================================
#define MOTOR_PWM_FREQ               (10000)                         // 电机 PWM 频率
#define MOTOR_PWM_DUTY_LIMIT         (10000)                         // 电机 PWM 占空比上限

#define MOTOR_FORWARD_PWM            (ATOM0_CH4_P02_4)              // 电机正转 PWM 引脚
#define MOTOR_REVERSE_PWM            (ATOM0_CH5_P02_5)              // 电机反转 PWM 引脚

#define MOTOR_ENCODER                (TIM6_ENCODER)                 // 电机编码器编号
#define MOTOR_ENCODER_CH1            (TIM6_ENCODER_CH1_P20_3)       // 电机编码器通道 1
#define MOTOR_ENCODER_CH2            (TIM6_ENCODER_CH2_P20_0)       // 电机编码器通道 2
//====================================================电机硬件参数定义====================================================

typedef struct
{
    int16 speed;                                                       // 当前周期测速值
    int32 duty;                                                        // 当前输出占空比
} motor_info_struct;

extern motor_info_struct motor_info;

void  motor_encoder_init      (void);
void  motor_init              (void);
int16 motor_get_count         (void);
int16 motor_get_speed         (void);
void  motor_set_duty          (int32 duty);
void  motor_stop              (void);

#endif
