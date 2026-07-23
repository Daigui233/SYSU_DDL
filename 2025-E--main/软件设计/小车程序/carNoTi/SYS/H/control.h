//
// Created by Lenovo on 2025/7/11.
//

#ifndef CONTROL_H
#define CONTROL_H
#include "FreeRTOS.h"
#include "task.h"

#define Dot(x)  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8, (GPIO_PinState)(x))
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integral;     // 积分项
    float error;
    float prev_error;   // 上一次误差
    float output_limit; // 输出限幅
    float integral_limit; // 积分限幅
    float filtered_derivative;  // 滤波后的微分项
} PID_Controller;

extern PID_Controller IMUPid;
extern TaskHandle_t QuestionTaskHandle;
void QuestionTask(void *argument);
void PID_Init();

void yunTaiPositionControl(float x,float y,int speed);
void yunTaiVelocityControl();
int detectionAiming();

float positionPIDCompute(PID_Controller *pid, float target,float measure);
float velocityPIDCompute(PID_Controller *pid, float target,float measure);
float IMUPIDCompute(PID_Controller *pid, float target,float measure);
// float PID_Compute(PositionVelocityPID *pid, int target,int measure);
void ledColorShow(uint8_t color);
void stateShow();
void controlBegin();
#endif //CONTROL_H
