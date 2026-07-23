//
// Created by Lenovo on 2025/7/11.
//

#ifndef ENTRY_H
#define ENTRY_H
#include "control.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32h750xx.h"

#define WHITE 0
#define YELLOW 1
#define PURPLE 2
#define RED 3
#define CYAN 4
#define GREEN 5
#define BLUE 6
#define NOCOLOR 7

typedef struct {
    float Position;
    float theta;
    uint16_t speed;
    PID_Controller pidvel;
}Drap_t;
typedef struct {
    int xPosition;
    int yPosition;
}Position_t;
// 按键状态结构体
typedef struct {
    uint8_t state;            // 当前稳定状态
    uint8_t lastState;        // 上次状态
    uint32_t lastDebounceTime;// 上次状态改变时间
} Key_t;
typedef struct {
    int speed;
    int motoCount;
}moto_t;
typedef struct {
    int hasTurn;
    uint8_t sideIndex;
    float xPosition;
    float yPosition;
}Car_t;
extern float targetAngle;
extern int myQuestion;
extern moto_t leftMotor;
extern moto_t rightMotor;
extern Car_t myCar;
extern Position_t nowPosition;
extern Position_t targetPosition;
extern xQueueHandle xRecvQueue;
extern uint8_t sendmsg[64];
extern Drap_t xDrap;
extern Drap_t yDrap;
extern uint8_t stateColor;
extern uint8_t worningFlag;
extern int dir;

void setupTask(void *argument);
void loopTask(void *argument);
void KeyScanTask(void *argument);
void showBmp();
#endif //ENTRY_H
