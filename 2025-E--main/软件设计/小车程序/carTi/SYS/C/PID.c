#include "PID.h"
#include "entry.h"
#include "control.h"


/*BianLiang ShengMing Begin*/
float Velocity,Encoder_Least,Encoder,Movement;
float Encoder_Integral,Target_Velocity;
tPID PID_Straight;
tPID PID_Trace;
tPID PID_Turn;

/*BianLiang ShengMing End*/

/**************************************************************************
函数功能：pid初始化
入口参数：pid结构体指针
返回  值：无
**************************************************************************/
void PID_Init(){
    leftMotor.pidvel.Kp = 77.8f;
    leftMotor.pidvel.Ki = 20.8f;
    leftMotor.pidvel.integral_limit = 2400.0f;
    leftMotor.pidvel.output_limit = 2400.0f;

    rightMotor.pidvel.Kp = 77.8f;
    rightMotor.pidvel.Ki = 20.8f;
    rightMotor.pidvel.integral_limit = 2400.0f;
    rightMotor.pidvel.output_limit = 2400.0f;

    PID_Trace.Kp=0.030;//0.7
    PID_Trace.Ki = 0.0;
    PID_Trace.Kd=1.6;//20 

    PID_Trace.error=0;
    PID_Trace.prev_error=0;
    PID_Trace.output_limit = 7;

    PID_Turn.Kp = 0.473;
    PID_Turn.Ki = 0.009;
    PID_Turn.Kd = 0.0;
    PID_Turn.integral_limit = 2;
    PID_Turn.output_limit = 4;

    
}
/**************************************************************************
函数功能：循迹pid的计算
入口参数：pid结构体指针，循迹目标的偏差
返回  值：pid计算结果，返回给电机
**************************************************************************/
float PID_Trace_Calc(tPID *PIDX,float measure,float target){
	PIDX->prev_error=PIDX->error;
	PIDX->error=target-measure;
    PIDX->integral +=PIDX->error;
    if(PIDX->integral>PIDX->integral_limit) PIDX->integral = PIDX->integral_limit;
    else if(PIDX->integral<-PIDX->integral_limit) PIDX->integral = -PIDX->integral_limit;
	PIDX->output=PIDX->Kp*PIDX->error+PIDX->Kd*(PIDX->error-PIDX->prev_error)+PIDX->Ki * PIDX->integral; 

	if(PIDX->output > PIDX->output_limit )   PIDX->output = PIDX->output_limit;
	if(PIDX->output < -PIDX->output_limit )  PIDX->output = -PIDX->output_limit;

	return PIDX->output;
}

float velocityPIDCompute(tPID *pid, float target,float measure) {
    pid->error = target - measure;
    //积分限幅
    pid->integral += pid->error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // 计算微分项并应用低通滤波
    float derivative = pid->error - pid->prev_error;
    pid->prev_error = pid->error;  // 更新上次误差
    //输出并限幅
    float velocityOutput = pid->Kp * pid->error + pid->Ki * pid->integral +  pid->Kd * derivative;
    if(velocityOutput >pid->output_limit) velocityOutput = pid->output_limit;
    if(velocityOutput <-pid->output_limit) velocityOutput = -pid->output_limit;
    // if(target==0&&abs(measure)<50) return 0;
    return velocityOutput;
}
