//
// Created by Lenovo on 2025/5/3.
//

#ifndef TASKLIST_H
#define TASKLIST_H
#include "freertos/FreeRTOS.h"
typedef struct {
    int Kp;
    int Ki;
    int Kd;
}pidSend;
typedef struct {
    uint8_t colorIndex;
    uint16_t lowerH;
    uint16_t lowerS;
    uint16_t lowerV;
    uint16_t upperH;
    uint16_t upperS;
    uint16_t upperV;
    uint32_t minArea;
    uint32_t maxArea;
}color_t;
extern pidSend positionPid;
extern pidSend speedPid;
extern SemaphoreHandle_t targetSemaphore;
extern int targetx;
extern int targety;
void LVGLTask(void *pvParameters);
#endif //TASK_H
