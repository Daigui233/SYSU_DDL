#ifndef __PID_H
#define __PID_H
#include "entry.h"
typedef struct {
	float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integral;     // 积分项
    float error;
    float prev_error;   // 上一次误差
	float output;
    float output_limit; // 输出限幅
    float integral_limit; // 积分限幅
    float filtered_derivative;  // 滤波后的微分项
} tPID;

extern tPID PID_Straight;
extern tPID PID_Turn;
extern tPID PID_Trace;
void PID_Init(void);
float PID_Trace_Calc(tPID *PIDX,float measure,float target);
float velocityPIDCompute(tPID *pid, float target,float measure);

//uint16_t PID_Speed_Calc(tPID *PIDX);


#endif   
