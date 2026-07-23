
#include <esp_log.h>
#include <gui_guider.h>
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include <TaskList.h>
#include <esp_http_client.h>
#include <esp_now.h>
#include <interface.h>

const char *TAG = "TaskList";
SemaphoreHandle_t targetSemaphore = NULL;
int targetx;
int targety;
pidSend positionPid={0,0,0};
pidSend speedPid={0,0,0};
float sendx;
float sendy;
/*LVGL的响应函数*/
void LVGLTask(void *pvParameters) {
    targetSemaphore = xSemaphoreCreateBinary();
    static int newx=150;
    static int newy=150;
    static char sendMsg[64];
    while(1) {
        //绘画目标点
        if(xSemaphoreTake(targetSemaphore, 2) == pdPASS) {
            newx +=(targetx-100)/10;
            newy +=(targety-100)/10;
            if(newx<0) newx=0;
            else if(newx>300) newx=300;
            if (newy<0) newy=0;
            else if(newy>300) newy=300;
            sendx = (float)newx*400.0f/300.0f+120.0f;
            sendy = 435.0f-(float)newy*400.0f/300.0f;
            lv_obj_set_x(guider_ui.screen_device_btn_3, (newx+450)-8);
            lv_obj_set_y(guider_ui.screen_device_btn_3, (newy+150)-8);
            sprintf(sendMsg,"%d,%d\r\n",(int)sendy,(int)sendx);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
        }
        lv_timer_handler();//处理触发信息
    }
}

