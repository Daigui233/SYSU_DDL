// can_motor_control.c
#include "can_motor_control.h"

#include "entry.h"
#include "fdcan.h"
#include "string.h"

// 全局变量

MotorStatus_t motorStatus[2]; // 2个电机
static osMessageQueueId_t canRxQueue;


// 初始化函数
void MotorControl_Init() {
    // 创建CAN接收队列
    canRxQueue = osMessageQueueNew(10, sizeof(CAN_Frame_t), NULL);

    // 初始化电机状态
    memset(motorStatus, 0, sizeof(motorStatus));

    // 启动CAN接收
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    uint8_t RxData[8];
    FDCAN_RxHeaderTypeDef RxHeader;
    CAN_Frame_t rxFrame;
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        // 读取FIFO0中的消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rxFrame.Data) == HAL_OK)
        {
            // 调用应用层回调函数处理消息
            rxFrame.StdId = RxHeader.Identifier;
            rxFrame.DLC = RxHeader.DataLength;
            CAN_RxCpltCallback(rxFrame);
        }
    }
}
// CAN接收中断回调
void CAN_RxCpltCallback(CAN_Frame_t rxFrame) {
    osMessageQueuePut(canRxQueue, &rxFrame, 0, 0);
}

// CAN发送函数
static HAL_StatusTypeDef CAN_SendFrame(uint32_t id, uint8_t *data, uint8_t len) {
    FDCAN_TxHeaderTypeDef TxHeader;
    HAL_StatusTypeDef status;

    // 配置发送头部

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = len;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 发送消息
    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data);
    return status;
}

// 计算CRC校验
static uint8_t CalculateCRC(uint8_t slaveAddr, uint8_t *data, uint8_t len) {
    uint32_t i;
    uint8_t check_sum;
    uint32_t sum = 0;

    for(i=0;i<len;i++)
    {
        sum += data[i];
    }
    sum += slaveAddr;
    check_sum = sum & 0xFF;
    return check_sum;
}

// 速度控制模式
HAL_StatusTypeDef SpeedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc) {
    uint8_t txBuffer[5];
    txBuffer[0] = CMD_SPEED_CONTROL;
    txBuffer[1] = (dir << 7) | ((speed >> 8) & 0x0F);
    txBuffer[2] = speed & 0xFF;
    txBuffer[3] = acc;
    txBuffer[4] = CalculateCRC(slaveAddr, txBuffer, 4);
    return CAN_SendFrame(slaveAddr, txBuffer, 5);
}

// 位置控制模式
HAL_StatusTypeDef PositionModeRun(uint8_t slaveAddr, uint8_t mode, uint8_t dir, uint16_t speed, uint8_t acc, int32_t value) {
    uint8_t txBuffer[8];
    uint8_t cmd;

    switch (mode) {
        case 1: cmd = CMD_POSITION_CONTROL1; break; // 脉冲数相对运动
        case 2: cmd = CMD_POSITION_CONTROL2; break; // 脉冲数绝对运动
        case 3: cmd = CMD_POSITION_CONTROL3; break; // 坐标值相对运动
        case 4: cmd = CMD_POSITION_CONTROL4; break; // 坐标值绝对运动
        default: return HAL_ERROR;
    }

    txBuffer[0] = cmd;

    if (mode == 1 || mode == 3) {
        // 相对运动模式需要方向参数
        txBuffer[1] = (dir << 7) | ((speed >> 8) & 0x0F);
    } else {
        // 绝对运动模式不需要方向参数
        txBuffer[1] = (speed >> 8) & 0x0F;
    }

    txBuffer[2] = speed & 0xFF;
    txBuffer[3] = acc;
    txBuffer[4] = (value >> 16) & 0xFF;
    txBuffer[5] = (value >> 8) & 0xFF;
    txBuffer[6] = value & 0xFF;
    txBuffer[7] = CalculateCRC(slaveAddr, txBuffer, 7);

    return CAN_SendFrame(slaveAddr, txBuffer, 8);
}

// 紧急停止
HAL_StatusTypeDef EmergencyStop(uint8_t slaveAddr) {
    uint8_t txBuffer[2];

    txBuffer[0] = CMD_EMERGENCY_STOP;
    txBuffer[1] = CalculateCRC(slaveAddr, txBuffer, 1);

    return CAN_SendFrame(slaveAddr, txBuffer, 2);
}

// 设置电机模式
HAL_StatusTypeDef SetMotorMode(uint8_t slaveAddr, uint8_t mode) {
    uint8_t txBuffer[3];

    txBuffer[0] = CMD_SET_MODE;
    txBuffer[1] = mode;
    txBuffer[2] = CalculateCRC(slaveAddr, txBuffer, 2);

    return CAN_SendFrame(slaveAddr, txBuffer, 3);
}

// 设置电机电流
HAL_StatusTypeDef SetMotorCurrent(uint8_t slaveAddr, uint16_t current) {
    uint8_t txBuffer[4];

    txBuffer[0] = CMD_SET_CURRENT;
    txBuffer[1] = (current >> 8) & 0xFF;
    txBuffer[2] = current & 0xFF;
    txBuffer[3] = CalculateCRC(slaveAddr, txBuffer, 3);

    return CAN_SendFrame(slaveAddr, txBuffer, 4);
}

// 设置电机使能状态
HAL_StatusTypeDef SetMotorEnable(uint8_t slaveAddr, uint8_t enable) {
    uint8_t txBuffer[3];

    txBuffer[0] = CMD_SET_ENABLE;
    txBuffer[1] = enable;
    txBuffer[2] = CalculateCRC(slaveAddr, txBuffer, 2);

    return CAN_SendFrame(slaveAddr, txBuffer, 3);
}

// 设置电机细分
HAL_StatusTypeDef SetMotorStep(uint8_t slaveAddr, uint8_t Mstep) {
    uint8_t txBuffer[3];

    txBuffer[0] = CMD_SET_MICROSTEP;
    txBuffer[1] = Mstep;
    txBuffer[2] = CalculateCRC(slaveAddr, txBuffer, 2);

    return CAN_SendFrame(slaveAddr, txBuffer, 3);
}

// 设置电机回零距离
HAL_StatusTypeDef SetHomeAngle(uint8_t slaveAddr, uint32_t retValue,uint16_t hm_ma) {
    uint8_t txBuffer[8];

    txBuffer[0] = CMD_SET_NO_LIMIT_HOME;

    // 2. 填充返回距离（字节2-5：4字节，大端模式）
    txBuffer[1] = (retValue >> 24) & 0xFF;  // 距离高8位
    txBuffer[2] = (retValue >> 16) & 0xFF;
    txBuffer[3] = (retValue >> 8) & 0xFF;
    txBuffer[4] = retValue & 0xFF;          // 距离低8位

    // 3. 填充回零电流（字节6-7：2字节，大端模式）
    txBuffer[5] = (hm_ma >> 8) & 0xFF;      // 电流高8位
    txBuffer[6] = hm_ma & 0xFF;             // 电流低8位
    txBuffer[7] = CalculateCRC(slaveAddr, txBuffer, 7);

    return CAN_SendFrame(slaveAddr, txBuffer, 8);
}
// gohome
HAL_StatusTypeDef goHome(uint8_t slaveAddr) {
    uint8_t txBuffer[2];

    txBuffer[0] = CMD_GO_HOME;
    txBuffer[1] = CalculateCRC(slaveAddr, txBuffer, 1);

    return CAN_SendFrame(slaveAddr, txBuffer, 2);
}

// 设置电机回零参数
HAL_StatusTypeDef SetHomeAll(uint8_t slaveAddr, uint16_t homespeed) {
    uint8_t txBuffer[8];

    txBuffer[0] = CMD_SET_LIMIT_PARAMS;

    // 2. 填充返回距离（字节2-5：4字节，大端模式）
    txBuffer[1] = 0;
    txBuffer[2] = 1;
    txBuffer[3] = (homespeed >> 8) & 0xFF;
    txBuffer[4] = homespeed & 0xFF;          // 距离低8位

    txBuffer[5] = 0;
    txBuffer[6] = 1;
    txBuffer[7] = CalculateCRC(slaveAddr, txBuffer, 7);

    return CAN_SendFrame(slaveAddr, txBuffer, 8);
}
// 查询电机位置
int32_t GetMotorPosition(uint8_t slaveAddr) {
    return motorStatus[slaveAddr-1].position;
}

// 查询电机速度
int16_t GetMotorSpeed(uint8_t slaveAddr) {
    return motorStatus[slaveAddr-1].speed;
}

// 查询电机状态
uint8_t GetMotorStatus(uint8_t slaveAddr) {
    return motorStatus[slaveAddr-1].status;
}

// 接收任务处理函数
void CAN_ReceiveTask(void *argument) {
    CAN_Frame_t rxFrame;
    uint8_t functionCode;

    for (;;) {
        if (osMessageQueueGet(canRxQueue, &rxFrame, NULL, osWaitForever) == osOK) {
            functionCode = rxFrame.Data[0];

            switch (functionCode) {
                case CMD_READ_MULTI_ENCODER: // 0x31
                    if (rxFrame.DLC == 8) {
                        int32_t position = (rxFrame.Data[3] << 24) |
                                           (rxFrame.Data[4] << 16) |
                                           (rxFrame.Data[5] << 8) |
                                           rxFrame.Data[6];
                        motorStatus[rxFrame.StdId-1].position = position;
                        motorStatus[rxFrame.StdId - 1].position = position;
                    }
                    break;

                case CMD_READ_SPEED: // 0x32
                    if (rxFrame.DLC == 4) {
                        int16_t speed = (rxFrame.Data[1] << 8) | rxFrame.Data[2];
                        motorStatus[rxFrame.StdId-1].speed = speed;
                    }
                    break;

                case CMD_QUERY_STATUS: // 0xF1
                    if (rxFrame.DLC == 3) {
                        motorStatus[rxFrame.StdId-1].status = rxFrame.Data[1];
                    }
                    break;
                case CMD_READ_ENCODER:
                    if (rxFrame.DLC == 8) {
                            uint16_t Encoder;  // 用int64_t存储48位值，避免溢出
                            Encoder =(rxFrame.Data[6] << 8) | rxFrame.Data[7];
                        // 组合6个字节为48位整数（假设高位在前，低位在后）


                            motorStatus[rxFrame.StdId-1].Encoder = Encoder;
                    }
                    // 可以添加更多状态解析...
            }
        }
    }
}

// 查询任务函数
void QueryTask(void *argument) {
    uint8_t txBuffer[2];
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();  // 获取当前系统节拍数

    for (;;) {
        // 查询所有电机的状态
        for (uint8_t i = 1; i <= 2; i++) {
            // 查询位置
            txBuffer[0] = CMD_READ_MULTI_ENCODER;
            txBuffer[1] = CalculateCRC(i, txBuffer, 1);
            CAN_SendFrame(i, txBuffer, 2);

            // 查询速度
            txBuffer[0] = CMD_READ_SPEED;
            txBuffer[1] = CalculateCRC(i, txBuffer, 1);
            CAN_SendFrame(i, txBuffer, 2);

            // 查询状态
            txBuffer[0] = CMD_QUERY_STATUS;
            txBuffer[1] = CalculateCRC(i, txBuffer, 1);
            CAN_SendFrame(i, txBuffer, 2);

            //查询原始编码值
            txBuffer[0] = CMD_READ_ENCODER;
            txBuffer[1] = CalculateCRC(i, txBuffer, 1);
            CAN_SendFrame(i, txBuffer, 2);

            vTaskDelay(10); // 适当延时
        }

        vTaskDelayUntil(&xLastWakeTime,200);
    }
}
