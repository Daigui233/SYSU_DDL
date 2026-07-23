#include "control.h"
#include "entry.h"
#include "PID.h"
#include "Trace.h"

moto_t leftMotor;
moto_t rightMotor;
float Final=0;
void Load(int moto1,int moto2){
    leftMotor.motoPWM = velocityPIDCompute(&leftMotor.pidvel,(float)moto1,(float)leftMotor.speed);
    rightMotor.motoPWM = velocityPIDCompute(&rightMotor.pidvel,(float)moto2,(float)rightMotor.speed);
    // sprintf(sendmsg,"%d,%d,%.2f\r\n",leftMotor.motoPWM,rightMotor.motoPWM,Final);

    if(leftMotor.motoPWM>0){
        DL_TimerA_setCaptureCompareValue(carPWM_INST, (uint32_t)0, DL_TIMER_CC_1_INDEX);
        DL_TimerA_setCaptureCompareValue(car2N_INST, (uint32_t)leftMotor.motoPWM*5, DL_TIMER_CC_0_INDEX);
    }else if(leftMotor.motoPWM<=0){
        DL_TimerA_setCaptureCompareValue(carPWM_INST, -(uint32_t)leftMotor.motoPWM, DL_TIMER_CC_1_INDEX);//AIN1fan
        DL_TimerA_setCaptureCompareValue(car2N_INST, (uint32_t)0, DL_TIMER_CC_0_INDEX);//AIN2fan
    }
        if(rightMotor.motoPWM>0){
        DL_TimerA_setCaptureCompareValue(carPWM_INST, (uint32_t)rightMotor.motoPWM, DL_TIMER_CC_0_INDEX);
        DL_TimerA_setCaptureCompareValue(car1N_INST, (uint32_t)0, DL_TIMER_CC_1_INDEX);
    }else if(rightMotor.motoPWM<=0){
        DL_TimerA_setCaptureCompareValue(carPWM_INST, (uint32_t)0, DL_TIMER_CC_0_INDEX);//BIN1
        DL_TimerA_setCaptureCompareValue(car1N_INST, (uint32_t)-rightMotor.motoPWM, DL_TIMER_CC_1_INDEX);//BIN2
    }
}
/**************************************************************************
函数功能：循迹
入口参数：循迹的权重中值
返回  值：无
**************************************************************************/
void Trace_Black(uint16_t inp,int speed,tPID* PIDx){
    Final=PID_Trace_Calc(PIDx, inp, 240.0f);
    Load(speed-(int)Final,speed+(int)Final);
}
void carControlTask(){
    static uint8_t Process_Flag;
    switch(Process_Flag){
        case 1:
            Trace_Black(inf_std, 10, &PID_Trace);
            if(leftMotor.motoCount+rightMotor.motoCount>=28480){
                myCar.CircleNum++;
                if(myCar.CircleNum==targetCar.CircleNum){
                    myCar.carState=Car_Stop;
                }
            }
            break;
    }
}
