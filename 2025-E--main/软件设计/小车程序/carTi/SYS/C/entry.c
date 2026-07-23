
#include "stdio.h"
#include "string.h"
#include "entry.h"
#include "dataRead.h"
#include "control.h"
#include "Trace.h"

char sendmsg[64];
uint8_t Serial_buf[64];
int result;
int num1=0,num2=0;
Car_t myCar;
Car_t targetCar;
//临时测试
int speed = 15;
Position_t nowPosition;
uint8_t lastCarState;
 uint8_t test;
void pidTest();
int32_t lastWakeTime;
int32_t globatime;

void setup(){
        
        //  NVIC_ClearPendingIRQ(TOF_INST_INT_IRQN);
        // NVIC_EnableIRQ(TOF_INST_INT_IRQN);
        NVIC_EnableIRQ(GPIOB_INT_IRQn);
        NVIC_EnableIRQ(GPIOA_INT_IRQn);
        DL_ADC12_startConversion(ADC_INST);
        PID_Init();
        
        DL_GPIO_setPins(myShow_PORT, myShow_LED_PIN);
        myCar.carState = Car_Stop;
        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    }

void loop(){
    //  DL_GPIO_togglePins(myShow_PORT, myShow_LED_PIN);
    //  sprintf(sendmsg,"%d,%d,%d,%d\r\n",leftMotor.speed,leftMotor.motoCount,rightMotor.motoCount,rightMotor.motoCount);
    keyScan();
    if(Serial_RxFlag){
         pidTest();
         //Position_DataRead(Serial_buf,&num1,&num2);
        Serial_RxFlag = 0;
    }
    // if(YunTaiOKFlag){
    //     if(DL_GPIO_readPins(myShow_PORT,myShow_YunTai_PIN)){
    //         myCar.carState = Car_Trace;
    //     }
    // }
    test = (DL_GPIO_readPins(ccd_PORT,ccd_YunTai_PIN)>>28)&0x01;
    if(YunTaiOKFlag){
        if(test==1){
            myCar.carState = Car_Trace;
            YunTaiOKFlag = 0;
        }
    }
    //sprintf(sendmsg,"%d,%d\r\n",num1,num2);
    //  sprintf(sendmsg,"%d,%d,%d,%d\r\n",num1,num2,leftMotor.speed,rightMotor.speed);
    // sprintf(sendmsg,"@TI:%d,%d,%d,%d,%d]%%\r\n",leftMotor.motoCount,rightMotor.motoCount,leftMotor.speed,rightMotor.speed,myCar.hasTurn);
  //sprintf(sendmsg,"%.3f,%.3f,%.3f\r\n",PID_Trace.Kp,PID_Trace.Ki,PID_Trace.Kd);
    // sprintf(sendmsg,"%d,%d,%d,%.2f\r\n",myCar.carState,globatime,lastWakeTime,inf_std);
    // Load(num1,num2);
    // sprintf(sendmsg,"%.2f,%d\r\n",inf_std,myCar.carState);
    sprintf(sendmsg,"@TI:%d,%d,%d,%d,%d,%d]%%\r\n",leftMotor.motoCount,rightMotor.motoCount,leftMotor.speed,rightMotor.speed,myCar.hasTurn,YunTaiMode);
    uart0_send_string(TOF_INST, sendmsg);
    DL_GPIO_togglePins(mosSW_PORT, mosSW_mos1_PIN);
    DL_GPIO_togglePins(mosSW_PORT, mosSW_mos2_PIN);
    delay_ms(15);
}

void TIMG0_IRQHandler(void){
    static int timeslice = 0;
    globatime++;
    inf_Read();
    speedUpdate();
    if(myCar.carState!=Car_Stop){
        if(myCar.carState==Car_Trace){
            timeslice++;
            if(timeslice>3){
                speed++;
                if(speed>16) speed = 16;
                timeslice = 0;
            }
            Trace_Black(inf_std, speed, &PID_Trace);
            leftMotor.motoCount += leftMotor.speed;
            rightMotor.motoCount += rightMotor.speed;
            if(inf_std<=210&&(globatime-lastWakeTime)>200){//转弯
            // 进入转弯状态
                myCar.carState = Car_Turn;
                lastWakeTime = globatime;
                if(myCar.hasTurn % 4 == 0&&myCar.hasTurn!=0) {
                myCar.CircleNum++;
                // 达到目标圈数，停止
                if(myCar.CircleNum >= targetCar.CircleNum) {
                    myCar.carState = Car_Stop;
                    myCar.hasTurn = 0;  // 重置转弯计数
                    myCar.CircleNum = 0; // 重置圈数计数
                    targetCar.CircleNum = 0;// 重置目标圈数
                    sprintf(sendmsg,"@TraceOK%%\r\n");
                    uart0_send_string(TOF_INST, sendmsg); 
                    DL_TimerA_setCaptureCompareValue(carPWM_INST, (uint32_t)0, DL_TIMER_CC_0_INDEX);
                    DL_TimerA_setCaptureCompareValue(car1N_INST, (uint32_t)10, DL_TIMER_CC_1_INDEX);
                    DL_TimerA_setCaptureCompareValue(carPWM_INST, (uint32_t)10, DL_TIMER_CC_1_INDEX);
                    DL_TimerA_setCaptureCompareValue(car2N_INST, (uint32_t)0, DL_TIMER_CC_0_INDEX);

                }
                }
            }
        }
        else if(myCar.carState == Car_Turn){
            timeslice++;
            speed = 2;
            Trace_Black(inf_std, speed, &PID_Turn);
            if(timeslice==30){
                leftMotor.motoCount = 0;
                rightMotor.motoCount = 0;
            }
            if(timeslice>=30&&inf_std>=230){
                timeslice = 0;
                DL_GPIO_togglePins(myShow_PORT, myShow_LED_PIN);
                myCar.hasTurn++;
                myCar.carState = Car_Trace;
            }
        }
    }
}
void GROUP1_IRQHandler(void){
    switch(DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)){
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            encoderFunctionA();
            break;
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
            encoderFunctionB();
            break;
        default:
            break;
    }

}

void delay_ms(unsigned long ms) 
{
    while(ms--)
	    delay_cycles(CPUCLK_FREQ/1000);
}
void pidTest(){
    if(strncmp((char*)Serial_buf,"PosKp",5) == 0) {//控制pid
        int num_args = 1;
        int temp = 0;
        Position_DataRead(Serial_buf,&temp,&num_args);
        PID_Trace.Kp = temp / 1000.0f;
    }else if(strncmp((char*)Serial_buf,"PosKi",5) == 0) {
        int num_args = 1;
        int temp = 0;
        Position_DataRead(Serial_buf,&temp,&num_args);
        PID_Trace.Ki = temp/100.0;
    }else if(strncmp((char*)Serial_buf,"PosKd",5) == 0) {
        int num_args = 1;
        int temp = 0;
        Position_DataRead(Serial_buf,&temp,&num_args);
        PID_Trace.Kd = temp/100.0f;
    }else if(strncmp((char *)Serial_buf, "[", 1)==0){
        Position_DataRead(Serial_buf,&num1,&num2);
    }else if(strncmp((char *)Serial_buf,"OK",2)==0){
        if(YunTaiOKFlag){
            myCar.carState = Car_Trace;
            YunTaiOKFlag = 0;
        }
    }
}