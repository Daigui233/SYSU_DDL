#include "dataRead.h"
#include "entry.h"
#include <stdio.h>
#include "control.h"

volatile uint8_t gEchoData = 0;
int RxState = 0;        //定义表示当前状态机状态的静态变量
int pRxPacket = 0;    //定义表示当前接收数据位置的静态变量
char RxData;
uint8_t Serial_RxFlag=0;
int32_t _encoder_1_count;
int32_t _encoder_2_count;

uint8_t YunTaiMode;
uint8_t ModeTemp;
uint8_t YunTaiOKFlag;

//串口接收中断
void ESP_INST_IRQHandler(void){
        switch (DL_UART_Main_getPendingInterrupt(ESP_INST)) {
            DL_GPIO_togglePins(myShow_PORT, myShow_LED_PIN);
        case DL_UART_IIDX_RX:
        
            if (RxState == 0) // 当前状态为 0，接收数据包包头
            {
                if (RxData == '@' && Serial_RxFlag == 0) // 判断是否为数据包包头    
                {
                    RxState = 1;         // 切换到状态 1
                    pRxPacket = 0;       // 重置接收数据位置
                }
            }else if (RxState == 1) // 当前状态为 1，接收数据包数据，同时判断第一个包尾
            {
                if (RxData == '%') {
                    RxState = 2;
                }else  // 接收到正常数据
                {
                    Serial_buf[pRxPacket] = RxData; // 将数据存储到接收数据包数组
                    pRxPacket++;       // 增加数据包的位置
                }
            }else if (RxState == 2) // 当前状态为 2，接收完毕
            {
                RxState = 0;       // 重置状态机
                Serial_buf[pRxPacket] = '\0';   // 添加字符串结束符
                Serial_RxFlag = 1 ; // 标记数据包接收成功
            }
            RxData = DL_UART_Main_receiveData(ESP_INST);
            uart0_send_char(ESP_INST,RxData);
                break;
        default:
            break;
    }
}
//串口发送
//串口发送单个字符
bool uart0_send_char(UART_Regs *uart,char ch)
{
    uint32_t time = 0;

    while( DL_UART_isBusy(uart) == true ){
        if (++time > (10 * (CPUCLK_FREQ/1000))) {
            return false; // 超时返回失败
        }
        
    }
    //发送单个字符
    DL_UART_Main_transmitData(uart, ch);
    return true;
}
//串口发送字符串
bool uart0_send_string(UART_Regs *uart,char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        if (!uart0_send_char(uart, *str++)) {
            return false; // 任意字符发送失败则整体失败
        }
    }
    return true;
}
void Position_DataRead(const uint8_t* data,int *num1,int *num2) {
    int idx = 0;
    int sign = 1;
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
            if(idx == 0){
                *num1 = sign * num;
            }else if(idx == 1){
                *num2 = sign * num;
            }
            idx++;
            num = 0;
            sign = 1;
        }
    }
}
void encoderFunctionA(){
    if(DL_GPIO_getEnabledInterruptStatus (encoder2_PORT,encoder2_leftA_PIN)){
        if (DL_GPIO_readPins (encoder2_PORT,encoder2_leftB_PIN)==0){
            _encoder_1_count++;
        }
        else{
            _encoder_1_count--;
        }

        DL_GPIO_clearInterruptStatus (encoder2_PORT,encoder2_leftA_PIN);
    }
    if(DL_GPIO_getEnabledInterruptStatus (encoder2_PORT,encoder2_leftB_PIN)){
        if (DL_GPIO_readPins (encoder2_PORT,encoder2_leftA_PIN)==0){
            _encoder_1_count--;
        }
        else{
            _encoder_1_count++;
        }

        DL_GPIO_clearInterruptStatus (encoder2_PORT,encoder2_leftB_PIN);
    }
}
void encoderFunctionB(){
    if(DL_GPIO_getEnabledInterruptStatus (encoder_PORT,encoder_RightA_PIN)){
        if (DL_GPIO_readPins (encoder_PORT,encoder_RightB_PIN)==0){
            _encoder_2_count++;
        }
        else{
            _encoder_2_count--;
        }

        DL_GPIO_clearInterruptStatus (encoder_PORT,encoder_RightA_PIN);
    }
    if(DL_GPIO_getEnabledInterruptStatus (encoder_PORT,encoder_RightB_PIN)){
        if (DL_GPIO_readPins (encoder_PORT,encoder_RightA_PIN)==0){
            _encoder_2_count--;
        }
        else{
            _encoder_2_count++;
        }

        DL_GPIO_clearInterruptStatus (encoder_PORT,encoder_RightB_PIN);
    }
}
int encoderRead(uint8_t TIMX){
    int Encoder_TIM;
    switch(TIMX)
    {
        case 1:  Encoder_TIM= _encoder_1_count;  _encoder_1_count=0;break;
        case 2:  Encoder_TIM= _encoder_2_count;  _encoder_2_count=0;break;
        default: Encoder_TIM=0;
    }
    return Encoder_TIM;
}

void speedUpdate(){//放在定时器里
    leftMotor.speed = encoderRead(1);
    rightMotor.speed = encoderRead(2);
    leftMotor.distance = leftMotor.motoCount/(float)(ENCODER_RESOLUTION*DISTANCE_PER_PULSE);
    rightMotor.distance = rightMotor.motoCount/(float)(ENCODER_RESOLUTION*DISTANCE_PER_PULSE);
}

void keyScan(){
    if (DL_GPIO_readPins (KEY_PORT,KEY_KEY1_PIN)==0)       
    {
        delay_ms(20);
        while (DL_GPIO_readPins (KEY_PORT,KEY_KEY1_PIN)==0);   
        delay_ms(20);                                         
        DL_GPIO_togglePins(myShow_PORT, myShow_LED_PIN);
        targetCar.CircleNum++;
    }
    if (DL_GPIO_readPins (KEY_PORT,KEY_KEY2_PIN)==0)          
    {
        delay_ms(20);
        while (DL_GPIO_readPins (KEY_PORT,KEY_KEY2_PIN)==0);   
        delay_ms(20);                                  
        DL_GPIO_togglePins(myShow_PORT, myShow_LED_PIN);
        ModeTemp++;
        ModeTemp%=7;
    }
    if (DL_GPIO_readPins (KEY_PORT,KEY_KEY3_PIN)==0)           
    {
        delay_ms(20);
        while (DL_GPIO_readPins (KEY_PORT,KEY_KEY3_PIN)==0);    
        delay_ms(20);
        DL_GPIO_togglePins(myShow_PORT, myShow_LED_PIN);
        YunTaiMode = ModeTemp;
        if(targetCar.CircleNum!=0){
            if(YunTaiMode==4||YunTaiMode==5){//请求解析
                YunTaiOKFlag = 1;
            }else{
                myCar.carState = Car_Trace;
            }
            
        }
        ModeTemp = 0;
    }
}