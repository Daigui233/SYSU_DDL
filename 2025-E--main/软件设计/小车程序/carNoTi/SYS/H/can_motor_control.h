// can_motor_control.h
#ifndef __CAN_MOTOR_CONTROL_H
#define __CAN_MOTOR_CONTROL_H

#include "main.h"
#include "cmsis_os.h"

// CAN帧结构体
typedef struct {
    uint32_t StdId;     // 标准ID
    uint8_t DLC;        // 数据长度
    uint8_t Data[8];    // 数据
} CAN_Frame_t;

// 电机状态结构体
typedef struct {
    int32_t position;   // 当前位置
    int16_t speed;      // 当前速度(RPM)
    uint8_t status;     // 运行状态
    uint8_t error;      // 错误状态
    uint16_t Encoder;
    uint8_t leftXianWei;//左限位
    uint8_t rightXianWei;//右限位
    float angle;
} MotorStatus_t;

// 功能码定义
#define CMD_READ_ENCODER        0x30
#define CMD_READ_MULTI_ENCODER  0x31
#define CMD_READ_SPEED          0x32
#define CMD_READ_IO_STATUS      0x34
#define CMD_READ_RAW_ENCODER    0x35
#define CMD_READ_POS_ERROR      0x39
#define CMD_READ_ENABLE_STATUS  0x3A
#define CMD_READ_HOME_STATUS    0x3B
#define CMD_STALL_RELEASE       0x3D
#define CMD_READ_STALL_STATUS   0x3E
#define CMD_RESTORE_DEFAULT     0x3F
#define CMD_READ_VERSION        0x40
#define CMD_RESET_MOTOR         0x41
#define CMD_CALIBRATE_ENCODER   0x80
#define CMD_SET_MODE            0x82
#define CMD_SET_CURRENT         0x83
#define CMD_SET_HOLD_CURRENT    0x9B
#define CMD_SET_MICROSTEP       0x84
#define CMD_SET_EN_PIN          0x85
#define CMD_SET_DIR             0x86
#define CMD_SET_AUTO_OFF        0x87
#define CMD_SET_STALL_PROTECT   0x88
#define CMD_SET_INTERPOLATION   0x89
#define CMD_SET_CAN_RATE        0x8A
#define CMD_SET_SLAVE_ADDR      0x8B
#define CMD_SET_RESPONSE_MODE   0x8C
#define CMD_SET_KEY_LOCK        0x8F
#define CMD_SET_GROUP_ADDR      0x8D
#define CMD_WRITE_IO            0x36
#define CMD_SET_LIMIT_PARAMS    0x90
#define CMD_GO_HOME             0x91
#define CMD_SET_ZERO            0x92
#define CMD_SET_NO_LIMIT_HOME   0x94
#define CMD_SET_LIMIT_REMAP     0x9E
#define CMD_SET_HOME_PARAMS     0x9A
#define CMD_SET_EN_TRIGGER_HOME 0x9D
#define CMD_QUERY_STATUS        0xF1
#define CMD_SET_ENABLE          0xF3
#define CMD_EMERGENCY_STOP      0xF7
#define CMD_SPEED_CONTROL       0xF6
#define CMD_SAVE_PARAMS         0xFF
#define CMD_POSITION_CONTROL1   0xFD
#define CMD_POSITION_CONTROL2   0xFE
#define CMD_POSITION_CONTROL3   0xF4
#define CMD_POSITION_CONTROL4   0xF5

// 方向定义
#define DIR_CW      0   // 顺时针
#define DIR_CCW     1   // 逆时针

// 模式定义
#define MODE_CR_OPEN    0x00
#define MODE_CR_CLOSE   0x01
#define MODE_CR_VFOC    0x02
#define MODE_SR_OPEN    0x03
#define MODE_SR_CLOSE   0x04
#define MODE_SR_VFOC    0x05

// 初始化函数
void MotorControl_Init(void);
void CAN_RxCpltCallback(CAN_Frame_t rxFrame);

// 控制接口
HAL_StatusTypeDef SpeedModeRun(uint8_t slaveAddr, uint8_t dir, uint16_t speed, uint8_t acc);
HAL_StatusTypeDef PositionModeRun(uint8_t slaveAddr, uint8_t mode, uint8_t dir, uint16_t speed, uint8_t acc, int32_t value);
HAL_StatusTypeDef EmergencyStop(uint8_t slaveAddr);
HAL_StatusTypeDef SetMotorMode(uint8_t slaveAddr, uint8_t mode);
HAL_StatusTypeDef SetMotorCurrent(uint8_t slaveAddr, uint16_t current);
HAL_StatusTypeDef SetMotorEnable(uint8_t slaveAddr, uint8_t enable);
HAL_StatusTypeDef SetMotorStep(uint8_t slaveAddr, uint8_t Mstep);
HAL_StatusTypeDef SetHomeAngle(uint8_t slaveAddr, uint32_t retValue,uint16_t hm_ma);
HAL_StatusTypeDef goHome(uint8_t slaveAddr);
HAL_StatusTypeDef SetHomeAll(uint8_t slaveAddr, uint16_t homespeed);

// 查询接口
int32_t GetMotorPosition(uint8_t slaveAddr);
int16_t GetMotorSpeed(uint8_t slaveAddr);
uint8_t GetMotorStatus(uint8_t slaveAddr);

extern MotorStatus_t motorStatus[2];
void CAN_ReceiveTask(void *argument);
void QueryTask(void *argument);

#endif