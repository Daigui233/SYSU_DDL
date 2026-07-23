#ifndef __TRACE_H__
#define __TRACE_H__
#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
extern uint16_t gAdcResult[6];
extern int16_t normalized[6];
extern uint16_t inf_Min[6];
extern uint16_t inf_Max[6];
extern float inf_std;
extern uint32_t digitResult[2];
extern int16_t TraceSum;

void inf_Read(void);
uint16_t KalmanFilter(uint16_t inData);
void Scan(void);
void Normalization(void);
void Standardization(void);

#ifdef __cplusplus
}
#endif
#endif