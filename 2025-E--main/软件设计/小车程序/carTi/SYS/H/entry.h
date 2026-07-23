#ifndef __ENTRY_H__
#define __ENTRY_H__
#ifdef __cplusplus
extern "C"{
#endif
#include <stdint.h>
#include "ti/driverlib/dl_gpio.h"
#include "ti_msp_dl_config.h"
#include <ti/driverlib/m0p/dl_interrupt.h>

#define ENCODER_RESOLUTION 14240//一圈脉冲数
#define DISTANCE_PER_PULSE 400//一圈长度（mm）
typedef enum {
    Car_Stop=0,
    Car_Trace,
    Car_Turn
}Car_State;
typedef struct{
    Car_State carState;
    int CircleNum;
    uint8_t hasTurn;
}Car_t;
typedef struct{
    int x;
    int y;
}Position_t;

extern Car_t myCar;
extern Car_t targetCar;
extern uint8_t Serial_buf[64];
extern char sendmsg[64];
extern uint8_t YunTaiMode;
void setup();
void loop();

void UART_SendString(UART_Regs * uart,const char *str);
void delay_ms(unsigned long ms);
#ifdef __cplusplus
}
#endif

#endif
