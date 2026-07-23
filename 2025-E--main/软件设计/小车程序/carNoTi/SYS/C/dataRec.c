//
// Created by Lenovo on 2025/7/11.
//

#include "dataRec.h"

#include <string.h>

#include "control.h"
#include "main.h"
#include "entry.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "can_motor_control.h"
#include "i2c.h"
//1为左转且为正角度，0为右转且为负角度
#define TSL_CLK(x)		HAL_GPIO_WritePin(ccd_clk_GPIO_Port, ccd_clk_Pin, (GPIO_PinState)(x))
#define TSL_SI(x)		HAL_GPIO_WritePin(ccd_si_GPIO_Port, ccd_si_Pin, (GPIO_PinState)(x))
#define MOTOTICK 33.79
#define TARGET_X 50.0f
#define TARGET_Y 150.0f

float SpeedInverse;
void carPositionCalculation();
float calculate_omega();
int lastMode = 0;
void inverseKinematicsTask(void *argument) {
    uint8_t serialDataRec[64];
    while (1) {
        if(xQueueReceive(xRecvQueue,&serialDataRec,portMAX_DELAY)) {
            //这里写入读取串口，选择功能代码
            stateColor = CYAN;
            //Position_DataRead(serialDataRec,&num,&xDrap.Position,&yDrap.Position);
            if(strncmp(serialDataRec,"[",1) == 0) {
                int num = 4;
                Position_DataRead(serialDataRec,&num,&targetPosition.xPosition,&targetPosition.yPosition,&nowPosition.xPosition,&nowPosition.yPosition);
            }else if(strncmp(serialDataRec,"PosKp",5) == 0) {//控制pid
                int num_args = 1;
                int temp = 0;
                Position_DataRead(serialDataRec,&num_args,&temp);
                xDrap.pidvel.Kp = temp/1000.0;
                yDrap.pidvel.Kp = -temp/1000.0;
                stateColor = CYAN;
            }else if(strncmp(serialDataRec,"PosKi",5) == 0) {
                int num_args = 1;
                int temp = 0;
                Position_DataRead(serialDataRec,&num_args,&temp);
                xDrap.pidvel.Ki = temp/100.0;
                yDrap.pidvel.Ki = -temp/100.0;
                stateColor = CYAN;
            }else if(strncmp(serialDataRec,"PosKd",5) == 0) {
                int num_args = 1;
                int temp = 0;
                Position_DataRead(serialDataRec,&num_args,&temp);
                xDrap.pidvel.Kd = temp/100.0;
                yDrap.pidvel.Kd = -temp/100.0;
                stateColor = CYAN;
            }else if(strncmp(serialDataRec,"TI",2) == 0) {
                int num_args = 6;
                Position_DataRead(serialDataRec,&num_args,
                    &leftMotor.motoCount,&rightMotor.motoCount,&leftMotor.speed,&rightMotor.speed,&myCar.hasTurn,&myQuestion);
                carPositionCalculation();//解算小车信息
                SpeedInverse = calculate_omega();//解算角速度反馈
                if(myQuestion != lastMode) {
                    if (QuestionTaskHandle != NULL) {
                        vTaskDelete(QuestionTaskHandle);  // 删除已有任务
                        QuestionTaskHandle = NULL;  // 句柄置空
                    }
                    xTaskCreate(QuestionTask,"QuestionTask",512,(void*)&myQuestion,6,&QuestionTaskHandle);
                    lastMode = myQuestion;
                }
            }
        }
    }
}

void Position_DataRead(const uint8_t* data,int* n,...) {//可变参数解析坐标信息
    va_list args;
    va_start(args, n);  // 初始化可变参数列表

    int idx = 0;
    int sign = 1;
    int count = 0;     // 已解析的数字个数
    int num = 0;
    for (int i = 0; data[i] != '\0'; i++) {
        char c = data[i];

        if (c == '-') {
            sign = -1;
            continue;
        }

        if (c >= '0' && c <= '9') {
            num = num * 10 + (c - '0');
        }
        else if (c == ',' || c == ' ' || c == ']') {
            if (count < *n) {
                int* ptr = va_arg(args, int*);  // 获取   下一个指针参数
                *ptr = sign * num;              // 通过指针赋值
                count++;
            }
            idx++;
            num = 0;
            sign = 1;
        }
    }
    *n = count;  // 更新实际解析的数字个数
    va_end(args);
}

void carPositionCalculation() {
    // 计算电机平均脉冲计数（用于位置推算）
    float avgCount = (float)(leftMotor.motoCount + rightMotor.motoCount) / 2.0f;

    switch(myCar.hasTurn % 4) {
        case 0: // BA线段 (从B到A，x:100→0, y固定=100)
            myCar.sideIndex = 1;
            myCar.xPosition = 100.0f - avgCount / MOTOTICK; // 反向运动
            myCar.yPosition = 100.0f;
        break;

        case 1: // AC线段 (从A到C，x固定=0, y:100→0)
            myCar.sideIndex = 2;
            myCar.xPosition = 0.0f;
            myCar.yPosition = 100.0f - avgCount / MOTOTICK; // 向下运动
        break;

        case 2: // CD线段 (从C到D，x:0→100, y固定=0)
            myCar.sideIndex = 3;
            myCar.xPosition = avgCount / MOTOTICK; // 正向运动
            myCar.yPosition = 0.0f;
        break;

        case 3: // DB线段 (从D到B，x固定=100, y:0→100)
            myCar.sideIndex = 4;
            myCar.xPosition = 100.0f;
            myCar.yPosition = avgCount / MOTOTICK; // 向上运动
        break;
    }
}

float calculate_omega() {
    float dx = TARGET_X - myCar.xPosition;
    float dy = TARGET_Y - myCar.yPosition;
    float v = (float)(leftMotor.speed+rightMotor.speed)/2.0f*2.6f;
    float distance_sq = dx * dx + dy * dy;

    if (distance_sq < 1e-6f) return 0.0f; // 避免除以零

    switch (myCar.hasTurn % 4) {
        case 0: // BA边 (B→A)
            return (50.0f * v) / (dx * dx + 2500.0f);
        case 1: // AC边 (A→C)
            return (-50.0f * v) / (2500.0f + dy * dy);
        case 2: // CD边 (C→D)
            return (-150.0f * v) / (dx * dx + 22500.0f);
        case 3: // DB边 (D→B)
            return (-50.0f * v) / (2500.0f + dy * dy);
        default:
            return 0.0f;
    }
}


unsigned char chrTemp[30];
JY901_Angle myIMU;
/**************************************************************************
函数功能：将short数据存入寄存器
入口参数：寄存器地址，数据数组
返回  值：无
**************************************************************************/
void ShortToChar(short sData,unsigned char cData[])
{
    cData[0]=sData&0xff;
    cData[1]=sData>>8;
}

/**************************************************************************
函数功能：读取寄存器数据
入口参数：寄存器地址高一位，低本位
返回  值：读取数据值（尚未处理）
**************************************************************************/
short CharToShort(unsigned char cData[])
{
    return ((short)cData[1]<<8)|cData[0];
}
/**************************************************************************
函数功能：读取陀螺仪的值
入口参数：angle结构体
返回  值：无
**************************************************************************/
void JY901_RDDat(JY901_Angle *IMUx)
{

    HAL_I2C_Mem_Read(&hi2c1, 0x50<<1, AX, I2C_MEMADD_SIZE_8BIT, &chrTemp[0] , 24, 0xff);
    (*IMUx).a[0] = (float)CharToShort(&chrTemp[0])/32768*16;
    (*IMUx).a[1] = (float)CharToShort(&chrTemp[2])/32768*16;
    (*IMUx).a[2] = (float)CharToShort(&chrTemp[4])/32768*16;
    (*IMUx).w[0] = (float)CharToShort(&chrTemp[6])/32768*2000;
    (*IMUx).w[1] = (float)CharToShort(&chrTemp[8])/32768*2000;
    (*IMUx).w[2] = (float)CharToShort(&chrTemp[10])/32768*2000;
    (*IMUx).h[0] = CharToShort(&chrTemp[12]);
    (*IMUx).h[1] = CharToShort(&chrTemp[14]);
    (*IMUx).h[2] = CharToShort(&chrTemp[16]);
    (*IMUx).Angle[0] = (float)CharToShort(&chrTemp[18])/32768*180;
    (*IMUx).Angle[1] = (float)CharToShort(&chrTemp[20])/32768*180;
    (*IMUx).Angle[2] = (float)CharToShort(&chrTemp[22])/32768*180;

}