//
// Created by Lenovo on 2025/7/11.
//

#include "entry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_motor_control.h"
#include "control.h"
#include "dataRec.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
// 按键消抖相关定义
#define DEBOUNCE_TIME 50      // 消抖时间(毫秒)
#define KEY_PRESSED 0         // 按键按下的电平
#define KEY_RELEASED 1        // 按键释放的电平

//串口相关变量
uint8_t Serial_RxFlag7=0;
uint8_t Serial_buf7[64];
uint8_t RxData7 = 0;
int RxState7 = 0;
int pRxPacket7 = 0;

uint8_t Serial_RxFlag4=0;
uint8_t Serial_buf4[64];
uint8_t RxData4 = 0;
int RxState4 = 0;
int pRxPacket4 = 0;

uint8_t sendmsg[64];
uint8_t stateColor;
uint8_t worningFlag=0;
Drap_t xDrap;
Drap_t yDrap;
Position_t nowPosition;
Position_t targetPosition = {0,0};
xQueueHandle xRecvQueue;
int myQuestion;
Key_t keys[3] = {             // 定义4个按键
    { KEY_RELEASED, KEY_RELEASED, 0},
    {KEY_RELEASED, KEY_RELEASED, 0},
    {KEY_RELEASED, KEY_RELEASED, 0}
};
//临时测试
int dir;
int speed;
moto_t leftMotor;
moto_t rightMotor;
Car_t myCar;
void setupTask(void *argument) {
    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);
    HAL_UART_Receive_IT(&huart7,&RxData7, 1);
    HAL_UART_Receive_IT(&huart4,&RxData4, 1);
    PID_Init();
    MotorControl_Init();
    controlBegin();
    xTaskCreate(loopTask,"loopTask",256,NULL,6,NULL);
    xTaskCreate(inverseKinematicsTask,"inverseKinematicsTask",256,NULL,6,NULL);
    xTaskCreate(QueryTask,"queryTask",256,NULL,5,NULL);
    xTaskCreate(CAN_ReceiveTask,"CAN_ReceiveTask",256,NULL,5,NULL);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
    stateColor = GREEN;
    vTaskDelete(NULL);
}


void loopTask(void *argument) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();  // 获取当前系统节拍数
    static float theat;
    uint8_t sum=0,i=0;
    while(1) {
        stateShow();
         // if(worningFlag) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
         // else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
        JY901_RDDat(&myIMU);
          yunTaiVelocityControl();
        // SpeedModeRun(0x02,0,40,0);
        // speedUpdate();
        // Dot(1);
        // HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_9);
         sprintf(sendmsg,"%d\r\n",leftMotor.motoCount);
         // sprintf(sendmsg,"%.2f,%.2f,%.2f\r\n",myCar.xPosition,myCar.yPosition,SpeedInverse);
         HAL_UART_Transmit(&huart7,sendmsg,strlen(sendmsg),20);
         vTaskDelayUntil(&xLastWakeTime,15);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 确保是 USART1 的回调
    if (huart == &huart7) {
        if (RxState7 == 0) // 当前状态为 0，接收数据包包头
        {
            if (RxData7 == '@' && Serial_RxFlag7 == 0) // 判断是否为数据包包头
            {
                RxState7 = 1;         // 切换到状态 1
                pRxPacket7 = 0;       // 重置接收数据位置
            }
        }
        else if (RxState7 == 1) // 当前状态为 1，接收数据包数据，同时判断第一个包尾
        {
            if (RxData7 == '%') {
                RxState7 = 2;
            }else  // 接收到正常数据
            {
                Serial_buf7[pRxPacket7] = RxData7; // 将数据存储到接收数据包数组
                pRxPacket7++;       // 增加数据包的位置
            }
        }
        else if (RxState7 == 2) // 当前状态为 2，接收完毕
        {
            RxState7 = 0;       // 重置状态机
            Serial_buf7[pRxPacket7] = '\0';   // 添加字符串结束符
            Serial_RxFlag7 = 1 ; // 标记数据包接收成功
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(xRecvQueue,&Serial_buf7,&xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            Serial_RxFlag7 = 0;
        }
        else {//错误情况
            stateColor = RED;
        }
        HAL_UART_Receive_IT(&huart7, &RxData7, 1);
    }
    if (huart == &huart4) {
        if (RxState4 == 0) // 当前状态为 0，接收数据包包头
        {
            if (RxData4 == '@' && Serial_RxFlag4 == 0) // 判断是否为数据包包头
            {
                RxState4 = 1;         // 切换到状态 1
                pRxPacket4 = 0;       // 重置接收数据位置
            }
        }
        else if (RxState4 == 1) // 当前状态为 1，接收数据包数据，同时判断第一个包尾
        {
            if (RxData4 == '%') {
                RxState4 = 2;
            }else  // 接收到正常数据
            {
                Serial_buf4[pRxPacket4] = RxData4; // 将数据存储到接收数据包数组
                pRxPacket4++;       // 增加数据包的位置
            }
        }
        else if (RxState4 == 2) // 当前状态为 2，接收完毕
        {
            RxState4 = 0;       // 重置状态机
            Serial_buf4[pRxPacket4] = '\0';   // 添加字符串结束符
            Serial_RxFlag4 = 1 ; // 标记数据包接收成功
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(xRecvQueue,&Serial_buf4,&xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            Serial_RxFlag4 = 0;
        }
        else {//错误情况
            stateColor = RED;
        }
        HAL_UART_Receive_IT(&huart4, &RxData4, 1);
    }
}
void KeyScanTask(void *argument) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t reading;
    while(1) {
        // 扫描所有按键
        for(int i = 0; i < 3; i++) {
            switch(i) {
                case 0:
                    reading = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11);
                    break;
                case 1:
                    reading = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10);
                    break;
                case 2:
                    reading = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);
                    break;
            }
            // 如果检测到电平变化，记录时间
            if (reading != keys[i].lastState) {
                keys[i].lastDebounceTime = xTaskGetTickCount();
            }

            // 如果持续时间超过消抖时间，认为是有效变化
            if ((xTaskGetTickCount() - keys[i].lastDebounceTime) > DEBOUNCE_TIME) {
                if (reading != keys[i].state) {
                    keys[i].state = reading;

                    // 按键按下事件处理
                    if (keys[i].state == KEY_PRESSED) {
                        // 根据按键索引处理不同按键的按下事件
                        int temp=0;
                        switch(i) {
                            case 0:
                                stateColor++;
                                temp++;
                                    break;
                            case 1:
                                stateColor--;
                                temp--;
                                    break;
                            case 2:
                                stateColor = GREEN;
                                myQuestion=temp;
                                if (QuestionTaskHandle != NULL) {
                                    vTaskDelete(QuestionTaskHandle);  // 删除已有任务
                                    QuestionTaskHandle = NULL;  // 句柄置空
                                }
                                xTaskCreate(QuestionTask,"QuestionTask",512,(void*)&myQuestion,6,&QuestionTaskHandle);
                                    break;
                        }
                    }
                }
            }
            keys[i].lastState = reading;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 每10ms扫描一次
    }
}

void showBmp() {
    for(int i=0; i<70; i++) {
        xDrap.Position += 0.1;
        yDrap.Position = 0;
        yunTaiPositionControl(xDrap.Position, yDrap.Position,800);
        sprintf(sendmsg,"%d,%.2f,%.2f\r\n",i,xDrap.Position,yDrap.Position);
        HAL_UART_Transmit(&huart7, (uint8_t*)sendmsg, strlen(sendmsg), 20);
        vTaskDelay(10);
    }

    // 阶段2: 从右下角(36,0)到顶点(18,31.1769)
    for(int i=0; i<70; i++) {
        xDrap.Position -= 0.05;
        yDrap.Position += 0.0866;  // tan(60°) * 0.1 ≈ 0.0866
        yunTaiPositionControl(xDrap.Position, yDrap.Position,800);
        sprintf(sendmsg,"%d,%.2f,%.2f\r\n",i,xDrap.Position,yDrap.Position);
        HAL_UART_Transmit(&huart7, (uint8_t*)sendmsg, strlen(sendmsg), 20);
        vTaskDelay(10);
    }

    // 阶段3: 从顶点(18,31.1769)回到左下角(0,0)
    for(int i=0; i<70; i++) {
        xDrap.Position -= 0.05;    // 18/360 = 0.05
        yDrap.Position -= 0.0866;  // 31.1769/360 ≈ 0.0866
        yunTaiPositionControl(xDrap.Position, yDrap.Position,800);
        sprintf(sendmsg,"%d,%.2f,%.2f\r\n",i,xDrap.Position,yDrap.Position);
        HAL_UART_Transmit(&huart7, (uint8_t*)sendmsg, strlen(sendmsg), 20);
        vTaskDelay(10);
    }


}