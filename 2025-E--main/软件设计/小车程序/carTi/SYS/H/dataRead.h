#ifndef __DATAREAD_H__
#define __DATAREAD_H__
#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include "entry.h"

extern uint8_t Serial_RxFlag;
extern uint8_t YunTaiOKFlag;
extern uint8_t keyScanFlag;
bool uart0_send_char(UART_Regs *uart,char ch);
bool uart0_send_string(UART_Regs *uart,char* str); //串口0发送字符串
void Position_DataRead(const uint8_t* data,int *num1,int *num2);
void encoderFunctionA();
void encoderFunctionB();
int encoderRead(uint8_t TIMX);
void speedUpdate();
void keyScan();
#ifdef __cplusplus
}
#endif

#endif
