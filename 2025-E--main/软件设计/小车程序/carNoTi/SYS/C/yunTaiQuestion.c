//
// Created by Lenovo on 2025/7/31.
//

#include "yunTaiQuestion.h"

#include <stdio.h>
#include <string.h>

#include "can_motor_control.h"
#include "control.h"
#include "entry.h"
#include "main.h"
#include "usart.h"

void aimToDot() {//瞄准后射击
    static uint8_t processFlag = 0;
    switch (processFlag) {
        case 0:
            goHome(0x01);
            processFlag = 1;
            vTaskDelay(200);
        break;
        case 1:
            yunTaiVelocityControl();
            if(detectionAiming()) processFlag = 2;
            break;
        case 2:
            Dot(1);
            vTaskDelay(250);
            Dot(0);
            QuestionTaskHandle = NULL;
            vTaskDelete(NULL);
            break;
    }
}
void aimAndDot() {
    static uint8_t processFlag = 0;
    switch(processFlag) {
        case 0:
            yunTaiVelocityControl();
            if(detectionAiming()) {
                HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
                processFlag = 1;
            }
        break;
        case 1:
            Dot(1);
            yunTaiVelocityControl();
        break;
    }
}