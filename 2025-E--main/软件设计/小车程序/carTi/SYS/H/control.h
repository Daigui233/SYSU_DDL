#ifndef __CONTROL_H__
#define __CONTROL_H__
#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include "PID.h"

typedef struct {
    int speed;
    int motoPWM;
    int motoCount;
    int targetSpeed;
    float distance;
    tPID pidvel;
}moto_t;

extern moto_t leftMotor;
extern moto_t rightMotor;

void Load(int moto1,int moto2);
void Trace_Black(uint16_t inp,int speed,tPID* PIDx);
void carControlTask();
#ifdef __cplusplus
}
#endif

#endif
