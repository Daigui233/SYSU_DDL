#include "control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_motor_control.h"
#include "dataRec.h"
#include "entry.h"
#include "main.h"
#include "task.h"
#include "usart.h"
#include "yunTaiQuestion.h"

TaskHandle_t QuestionTaskHandle;
void QuestionTask(void *argument) {
    while(1) {
        switch(myQuestion) {
            case 1://瞄准后打点
                aimToDot();
                break;
            case 2://持续瞄准打点
                aimAndDot();
                break;
            case 3://持续瞄准并画圆
                break;
            default://选了错误的题目，啥都不做
                break;
        }
        vTaskDelay(10);
    }
}

void yunTaiPositionControl(float x,float y,int speed) {
    xDrap.theta = atan(x/100.0f);
    xDrap.theta = xDrap.theta * (180.0f / M_PI);
    yDrap.theta = atan(y/100.0f);
    yDrap.theta = yDrap.theta * (180.0f / M_PI);
    PositionModeRun(0x02,4,1,speed,1,(int32_t)(-xDrap.theta*ANGLETOCLK));
    PositionModeRun(0x01,4,1,speed,1,(int32_t)(yDrap.theta*ANGLETOCLK));
}
void yunTaiVelocityControl() {
    float speedx=0;
    int speedy;
    speedx = velocityPIDCompute(&xDrap.pidvel,(float)targetPosition.xPosition,(float)nowPosition.xPosition);
    speedy = velocityPIDCompute(&yDrap.pidvel,(float)targetPosition.yPosition,(float)nowPosition.yPosition);

     speedx += myIMU.w[2]*2.4f;
    speedx += SpeedInverse*95.7f;
    sprintf(sendmsg,"%.2f,%.2f,%d,%d,%d\r\n",myCar.xPosition,myCar.yPosition,leftMotor.speed,rightMotor.speed,myCar.hasTurn);
    if(speedx < 0) {
        SpeedModeRun(0x02,0,-(int)(speedx),0);
    }else {
        SpeedModeRun(0x02,1,(int)(speedx),0);
    }
    if(speedy < 0) {
        SpeedModeRun(0x01,0,-speedy,0);
    }else {
        SpeedModeRun(0x01,1,speedy,0);
    }
}
int detectionAiming() {
    if(abs(targetPosition.xPosition-nowPosition.xPosition)<=3 && abs(targetPosition.yPosition-nowPosition.yPosition)<=3) {
        return 1;
    }
    return 0;
}
// PID控制器初始化函数
void PID_Init() {
    xDrap.pidvel.Kp = 0.532;
    xDrap.pidvel.Ki = 0.0;
    xDrap.pidvel.Kd = 1.49;
    xDrap.pidvel.output_limit = 3000.0f;

    yDrap.pidvel.Kp = -0.532;
    yDrap.pidvel.Ki = 0.0;
    yDrap.pidvel.Kd = -1.49;
    yDrap.pidvel.output_limit = 3000.0f;
}

float positionPIDCompute(PID_Controller *pid, float target,float measure) {//位置环控制，为20ms周期
    pid->error = target - measure;
    if(pid->error<10&&pid->error>-10) pid->error = 0;
    if(pid->error<50&&pid->error>-50) {
            //积分限幅
        pid->integral += (pid->error * pid->Ki);
        if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
        if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }


    float derivative = pid->error - pid->prev_error;//微分项
    pid->prev_error = pid->error;//更新上次误差
    //输出并限幅
    float positionOutput = pid->Kp * pid->error + pid->integral+ pid->Kd * derivative;
    if(positionOutput >pid->output_limit) positionOutput = pid->output_limit;
    if(positionOutput <-pid->output_limit) positionOutput = -pid->output_limit;
    //sprintf(sendmsg,"com:%.2f\r\n",pid->integral);
    HAL_UART_Transmit(&huart7,sendmsg,strlen(sendmsg),100);
    return positionOutput;
}
float velocityPIDCompute(PID_Controller *pid, float target,float measure) {
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

// 闪烁状态结构体：记录当前颜色的闪烁阶段
typedef struct {
    uint8_t current_color;    // 当前正在显示的颜色（用于检测是否切换）
    uint8_t stage;            // 闪烁阶段：0=亮灯阶段，1=灭灯阶段
    TickType_t stage_start;   // 当前阶段的开始时间（用于计算是否到切换时间）
} BlinkState;

// 全局闪烁状态（初始化）
static BlinkState blink_state = {
    .current_color = NOCOLOR,
    .stage = 0,
    .stage_start = 0
};

// 定义每个颜色的闪烁参数（亮灯时间、灭灯时间，单位：ms）
typedef struct {
    uint32_t on_time;   // 亮灯时长
    uint32_t off_time;  // 灭灯时长
} BlinkParam;

// 颜色-闪烁参数映射表（与case对应）
static const BlinkParam blink_params[] = {
    [WHITE]   = {1000, 0},  // 常量
    [YELLOW]   = {1000, 1000},   // 亮2000ms，灭500ms
    [PURPLE]  = {0, 1000},    // 亮500ms，灭500ms
    [RED]     = {200, 200},    // 亮200ms，灭200ms
    [CYAN]    = {100, 400},    // 亮400ms，灭600ms
    [GREEN]   = {2000, 500},   // 亮2000ms，灭500ms
    [BLUE]    = {700, 300},    // 亮700ms，灭300ms
    [NOCOLOR] = {0, 0}         // 常灭
};
// 蜂鸣器状态：记录是否正在鸣叫及开始时间
typedef struct {
    uint8_t is_buzzing;       // 0：未鸣叫；1：正在鸣叫
    TickType_t buzz_start;    // 鸣叫开始时间（用于计算500ms）
} BuzzerState;

// 全局蜂鸣器状态（初始化）
static BuzzerState buzzer_state = {
    .is_buzzing = 0,
    .buzz_start = 0
};

// 蜂鸣器控制（用户提供的GPIO操作，假设低电平响，高电平关）
#define BUZZ_ON()  do { worningFlag = 1; } while(0)  // 响（根据实际电平调整）
#define BUZZ_OFF() do { worningFlag = 0; } while(0)  // 关

void ledColorShow(uint8_t color) {
    switch(color) {
        case 0:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,0);
        break;
        case 1:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,0);
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
            break;
        case 4:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,0);
            break;
        case 5:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
            break;
        case 6:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,0);
            break;
        case 7:
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,1);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,1);
            break;
    }
}

void stateShow() {
    // --------------------------
    // 第一步：检查蜂鸣器是否需要关闭（非阻塞计时）
    // --------------------------
    if (buzzer_state.is_buzzing) {
        TickType_t current_tick = xTaskGetTickCount();
        // 计算蜂鸣已持续的时间（溢出安全处理）
        TickType_t elapsed = current_tick - buzzer_state.buzz_start;
        if (elapsed >= pdMS_TO_TICKS(500)) {  // 500ms到了
            BUZZ_OFF();                       // 关闭蜂鸣器
            stateColor = BLUE;
            buzzer_state.is_buzzing = 0;      // 重置蜂鸣状态
        }
    }

    // --------------------------
    // 第二步：处理颜色切换（核心逻辑）
    // --------------------------
    // 检测颜色是否切换（与上一次记录的颜色不同）
    if (stateColor != blink_state.current_color) {
        // 颜色切换：立即触发蜂鸣（仅在非鸣叫状态时触发，避免重复触发）
        if (!buzzer_state.is_buzzing&&stateColor == CYAN) {
            BUZZ_ON();                          // 打开蜂鸣器
            buzzer_state.is_buzzing = 1;         // 标记正在鸣叫
            buzzer_state.buzz_start = xTaskGetTickCount();  // 记录开始时间
        }

        // 重置颜色闪烁状态（立即更新为新颜色）
        blink_state.current_color = stateColor;
        blink_state.stage = 0;                   // 从亮灯阶段开始
        blink_state.stage_start = xTaskGetTickCount();
        ledColorShow(stateColor);                // 立即显示新颜色
        return;
    }

    // --------------------------
    // 第三步：颜色未切换，按当前阶段更新闪烁（非阻塞）
    // --------------------------
    if (stateColor == NOCOLOR) {
        ledColorShow(NOCOLOR);
        return;
    }

    BlinkParam param = blink_params[stateColor];
    TickType_t current_tick = xTaskGetTickCount();
    TickType_t elapsed = current_tick - blink_state.stage_start;

    if (blink_state.stage == 0) {  // 亮灯阶段
        if (elapsed >= pdMS_TO_TICKS(param.on_time)) {
            ledColorShow(NOCOLOR);
            blink_state.stage = 1;
            blink_state.stage_start = current_tick;
        }
    } else {  // 灭灯阶段
        if (elapsed >= pdMS_TO_TICKS(param.off_time)) {
            ledColorShow(stateColor);
            blink_state.stage = 0;
            blink_state.stage_start = current_tick;
        }
    }
}


void controlBegin() {
    int periadTime = 400;
    ledColorShow(WHITE);
    HAL_Delay(periadTime);
    ledColorShow(RED);
    HAL_Delay(periadTime);
    ledColorShow(BLUE);
    HAL_Delay(periadTime);
    ledColorShow(CYAN);
    HAL_Delay(periadTime);
    ledColorShow(PURPLE);
    HAL_Delay(periadTime);
    ledColorShow(GREEN);
    HAL_Delay(periadTime);
}