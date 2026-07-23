/*
* Copyright 2025 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "events_init.h"

#include <esp_now.h>
#include <interface.h>
#include <math.h>
#include <stdio.h>
#include <TaskList.h>
#include <freertos/semphr.h>

#include "lvgl.h"

#if LV_USE_GUIDER_SIMULATOR && LV_USE_FREEMASTER
#include "freemaster_client.h"
#endif

#define BEGINX 95
#define BEGINY 194
#define MYR 100

char sendMsg[64];
uint8_t target_macIndx;
color_t colorArray[3]={
    {0,50,70,120,60,40,30,10000,20000},
    {1,0,0,0,0,0,0,0,0},
    {2,0,0,0,0,0,0,0,0}
};
uint8_t nowColorIndex=0;
static void screen_device_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_SCREEN_LOADED:
    {
        lv_obj_set_style_radius(guider_ui.screen_device_btn_1, 90, LV_PART_MAIN);
        lv_obj_set_style_radius(guider_ui.screen_device_cont_2, 90, LV_PART_MAIN);
        break;
    }
    default:
        break;
    }
}

static void screen_device_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        break;
    }
    default:
        break;
    }
}
static void screen_device_cont_2_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
        case LV_EVENT_PRESSING:
        {
            // 获取触摸位置
            lv_point_t pos;
            lv_indev_get_point(lv_indev_get_act(), &pos);

            int dx = pos.x - (BEGINX + MYR);
            int dy = pos.y - (BEGINY + MYR);
            int distance_squared = dx * dx + dy * dy; // 避免使用pow函数，直接用乘法

            // 如果在圆内，直接跟随触摸位置
            if (distance_squared <= MYR*MYR) {
                lv_obj_set_x(guider_ui.screen_device_btn_1, pos.x - lv_obj_get_width(guider_ui.screen_device_btn_1) / 2);
                lv_obj_set_y(guider_ui.screen_device_btn_1, pos.y - lv_obj_get_height(guider_ui.screen_device_btn_1) / 2);
                targetx = pos.x-BEGINX;
                targety = pos.y-BEGINY;
            }
            // 如果在圆外，锁定在圆边缘
            else {
                // 计算单位方向向量
                float distance = sqrtf(distance_squared);
                float unit_x = (float)dx / distance;
                float unit_y = (float)dy / distance;

                // 计算圆边缘点的坐标
                int edge_x = (BEGINX+MYR) + unit_x * MYR; // 100是圆的半径(√10000)
                int edge_y = (BEGINY+MYR) + unit_y * MYR;

                // 设置按钮位置（居中对齐）
                lv_obj_set_x(guider_ui.screen_device_btn_1, edge_x - lv_obj_get_width(guider_ui.screen_device_btn_1) / 2);
                lv_obj_set_y(guider_ui.screen_device_btn_1, edge_y - lv_obj_get_height(guider_ui.screen_device_btn_1) / 2);
                targetx = edge_x-BEGINX;
                targety = edge_y-BEGINY;
            }
            xSemaphoreGive(targetSemaphore);
            break;
        }
        case LV_EVENT_RELEASED :
            // 释放后小圆盘回到初始位置(93,333)
            lv_obj_set_x(guider_ui.screen_device_btn_1, 170);
            lv_obj_set_y(guider_ui.screen_device_btn_1, 272);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
        break;
        default:
            break;
    }
}
static void screen_device_btn_5_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_pid, guider_ui.screen_pid_del, &guider_ui.screen_device_del, setup_scr_screen_pid, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
        break;
    }
    default:
        break;
    }
}

static void screen_device_btn_7_event_handler (lv_event_t *e)//shut
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
        case LV_EVENT_CLICKED:
        {
            sprintf(sendMsg, "@shut%%\r\n");
            lv_slider_set_value(guider_ui.screen_device_slider_1,50,true);
            lv_slider_set_value(guider_ui.screen_device_slider_2,50,true);
            lv_slider_set_value(guider_ui.screen_device_slider_3,50,true);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_device_btn_8_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_center, guider_ui.screen_center_del, &guider_ui.screen_device_del, setup_scr_screen_center, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
        break;
    }
    default:
        break;
    }
}

int x=0,y=0,z=0;
static void screen_device_slider_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            uint32_t value = lv_slider_get_value(slider);
            x = value;
            x = (x-50)/2;
            sprintf(sendMsg, "@Test[%d,%d,%d]%%\r\n",x,y,z);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            lv_label_set_text(guider_ui.screen_device_label_1, sendMsg);
            break;
        }
        default:
            break;
    }
}
static void screen_device_slider_2_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            uint32_t value = lv_slider_get_value(slider);
            y = value;
            y = (y-50)/2;
            sprintf(sendMsg, "@Test[%d,%d,%d]%%\r\n",x,y,z);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            lv_label_set_text(guider_ui.screen_device_label_1, sendMsg);
            break;
        }
        default:
            break;
    }
}
static void screen_device_slider_3_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            uint32_t value = lv_slider_get_value(slider);
            z = value-50;
            sprintf(sendMsg, "@Test[%d,%d,%d]%%\r\n",x,y,z);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            lv_label_set_text(guider_ui.screen_device_label_1, sendMsg);
            break;
        }
        default:
            break;
    }
}
static void screen_device_ddlist_1_event_handler (lv_event_t *e)//选题
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ddlist = lv_event_get_target(e);
    uint32_t selected_index = lv_dropdown_get_selected(ddlist);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
             sprintf(sendMsg, "@Question%ld%%\r\n",selected_index);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
void events_init_screen_device (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_device, screen_device_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_btn_1, screen_device_btn_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_btn_5, screen_device_btn_5_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_btn_8, screen_device_btn_8_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_cont_2, screen_device_cont_2_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_btn_7, screen_device_btn_7_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_slider_1, screen_device_slider_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_slider_2, screen_device_slider_2_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_slider_3, screen_device_slider_3_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_device_ddlist_1, screen_device_ddlist_1_event_handler, LV_EVENT_ALL, ui);
}

static void screen_pid_spinbox_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *spinbox = lv_event_get_target(e);
    int32_t value = lv_spinbox_get_value(spinbox);
    positionPid.Kp = value;
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            sprintf(sendMsg,"@PosKp%ld]%%\r\n",value);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pid_spinbox_2_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *spinbox = lv_event_get_target(e);
    int32_t value = lv_spinbox_get_value(spinbox);
    positionPid.Ki = value;
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            sprintf(sendMsg,"@PosKi%ld]%%\r\n",value);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pid_spinbox_3_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *spinbox = lv_event_get_target(e);
    int32_t value = lv_spinbox_get_value(spinbox);
    positionPid.Kd = value;
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            sprintf(sendMsg,"@PosKd%ld]%%\r\n",value);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pid_spinbox_4_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *spinbox = lv_event_get_target(e);
    int32_t value = lv_spinbox_get_value(spinbox);
    speedPid.Kp = value;
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            sprintf(sendMsg,"@velKp%ld]%%\r\n",value);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pid_spinbox_5_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *spinbox = lv_event_get_target(e);
    int32_t value = lv_spinbox_get_value(spinbox);
    speedPid.Ki = value;
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            sprintf(sendMsg,"@velKi%ld]%%\r\n",value);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pid_spinbox_6_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *spinbox = lv_event_get_target(e);
    int32_t value = lv_spinbox_get_value(spinbox);
    speedPid.Kd = value;
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            sprintf(sendMsg,"@velKd%ld]%%\r\n",value);
            esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pid_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_device, guider_ui.screen_device_del, &guider_ui.screen_pid_del, setup_scr_screen_device, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
        break;
    }
    default:
        break;
    }
}

void events_init_screen_pid (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_pid_btn_1, screen_pid_btn_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_1, screen_pid_spinbox_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_2, screen_pid_spinbox_2_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_3, screen_pid_spinbox_3_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_4, screen_pid_spinbox_4_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_5, screen_pid_spinbox_5_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_6, screen_pid_spinbox_6_event_handler, LV_EVENT_ALL, ui);
}

static void screen_center_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
        case LV_EVENT_CLICKED:
        {
            ui_load_scr_animation(&guider_ui, &guider_ui.screen_pi, guider_ui.screen_pi_del, &guider_ui.screen_center_del, setup_scr_screen_pi, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
            break;
        }
        default:
            break;
    }
}
static void screen_center_btn_2_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_device, guider_ui.screen_device_del, &guider_ui.screen_center_del, setup_scr_screen_device, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
        break;
    }
    default:
        break;
    }
}
static void screen_center_btn_3_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
        case LV_EVENT_CLICKED:
        {
            ui_load_scr_animation(&guider_ui, &guider_ui.screen_msg, guider_ui.screen_msg_del, &guider_ui.screen_center_del, setup_scr_screen_msg, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
            break;
        }
        default:
            break;
    }
}

void events_init_screen_center (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_center_btn_1, screen_center_btn_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_center_btn_2, screen_center_btn_2_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_center_btn_3, screen_center_btn_3_event_handler, LV_EVENT_ALL, ui);
}

static void screen_msg_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        ui_load_scr_animation(&guider_ui, &guider_ui.screen_center, guider_ui.screen_center_del, &guider_ui.screen_msg_del, setup_scr_screen_center, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
        break;
    }
    default:
        break;
    }
}
static void screen_msg_ta_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_READY:
        {
            const char * txt = lv_textarea_get_text(ta);
            switch(target_macIndx) {
                case 0:
                    sprintf(sendMsg, "@%s%%\r\n", txt);
                    esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
                    break;
                case 1:
                    sprintf(sendMsg, "%s\r\n", txt);
                    esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
                break;
                case 2:
                    sprintf(sendMsg, "@%s%%\r\n", txt);
                    esp_now_send(device_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
                    sprintf(sendMsg, "%s\r\n", txt);
                    esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
                break;
            }
            break;
        }
        default:
            break;
    }
}
static void screen_msg_ddlist_1_event_handler (lv_event_t *e)//选题
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ddlist = lv_event_get_target(e);
    uint32_t selected_index = lv_dropdown_get_selected(ddlist);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
             sprintf(sendmsg,"@Question%ld%%\r\n", selected_index);
            esp_now_send(device_mac,(uint8_t*)sendmsg,strlen(sendmsg)+1);
            break;
        }
        default:
            break;
    }
}
void events_init_screen_msg (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_msg_btn_1, screen_msg_btn_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_msg_ta_1, screen_msg_ta_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_msg_ddlist_1, screen_msg_ddlist_1_event_handler, LV_EVENT_ALL, ui);
}

static void screen_pi_btn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
        case LV_EVENT_CLICKED:
        {
            ui_load_scr_animation(&guider_ui, &guider_ui.screen_center, guider_ui.screen_center_del, &guider_ui.screen_pi_del, setup_scr_screen_center, LV_SCR_LOAD_ANIM_FADE_ON, 200, 200, false, true);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_ddlist_1_event_handler(lv_event_t *e)//选目标颜色
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ddlist = lv_event_get_target(e);
    uint32_t selected_index = lv_dropdown_get_selected(ddlist);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            nowColorIndex = selected_index;
            lv_slider_set_value(guider_ui.screen_pi_slider_1,colorArray[nowColorIndex].lowerH,LV_ANIM_ON);
            lv_slider_set_value(guider_ui.screen_pi_slider_2,colorArray[nowColorIndex].lowerS,LV_ANIM_ON);
            lv_slider_set_value(guider_ui.screen_pi_slider_3,colorArray[nowColorIndex].lowerV,LV_ANIM_ON);
            lv_slider_set_value(guider_ui.screen_pi_slider_4,colorArray[nowColorIndex].upperH,LV_ANIM_ON);
            lv_slider_set_value(guider_ui.screen_pi_slider_5,colorArray[nowColorIndex].upperS,LV_ANIM_ON);
            lv_slider_set_value(guider_ui.screen_pi_slider_6,colorArray[nowColorIndex].upperV,LV_ANIM_ON);
            lv_slider_set_value(guider_ui.screen_pi_slider_7,colorArray[nowColorIndex].minArea,LV_ANIM_ON);
            lv_slider_set_value(guider_ui.screen_pi_slider_8,colorArray[nowColorIndex].maxArea,LV_ANIM_ON);
            sprintf(numbertext,"%d",colorArray[nowColorIndex].lowerH);
            lv_label_set_text(guider_ui.screen_pi_label_9,numbertext);
            sprintf(numbertext,"%d",colorArray[nowColorIndex].lowerS);
            lv_label_set_text(guider_ui.screen_pi_label_10,numbertext);
            sprintf(numbertext,"%d",colorArray[nowColorIndex].lowerV);
            lv_label_set_text(guider_ui.screen_pi_label_11,numbertext);
            sprintf(numbertext,"%d",colorArray[nowColorIndex].upperH);
            lv_label_set_text(guider_ui.screen_pi_label_12,numbertext);
            sprintf(numbertext,"%d",colorArray[nowColorIndex].upperS);
            lv_label_set_text(guider_ui.screen_pi_label_13,numbertext);
            sprintf(numbertext,"%d",colorArray[nowColorIndex].upperV);
            lv_label_set_text(guider_ui.screen_pi_label_14,numbertext);
            sprintf(numbertext,"%ld",colorArray[nowColorIndex].minArea);
            lv_label_set_text(guider_ui.screen_pi_label_15,numbertext);
            sprintf(numbertext,"%ld",colorArray[nowColorIndex].maxArea);
            lv_label_set_text(guider_ui.screen_pi_label_16,numbertext);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_1_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].lowerH = value;
            sprintf(numbertext, "%d\r\n",colorArray[nowColorIndex].lowerH);
            lv_label_set_text(guider_ui.screen_pi_label_9,numbertext);

            sprintf(sendMsg, "%d:%d\r\n",nowColorIndex*8,colorArray[nowColorIndex].lowerH);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_2_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].lowerS = value;
            sprintf(numbertext, "%d\r\n",colorArray[nowColorIndex].lowerS);
            lv_label_set_text(guider_ui.screen_pi_label_10,numbertext);

            sprintf(sendMsg, "%d:%d\r\n",nowColorIndex*8+1,colorArray[nowColorIndex].lowerS);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_3_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].lowerV = value;
            sprintf(numbertext, "%d\r\n",colorArray[nowColorIndex].lowerV);
            lv_label_set_text(guider_ui.screen_pi_label_11,numbertext);

            sprintf(sendMsg, "%d:%d\r\n",nowColorIndex*8+2,colorArray[nowColorIndex].lowerV);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_4_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].upperH = value;
            sprintf(numbertext, "%d\r\n",colorArray[nowColorIndex].upperH);
            lv_label_set_text(guider_ui.screen_pi_label_12,numbertext);

            sprintf(sendMsg, "%d:%d\r\n",nowColorIndex*8+3,colorArray[nowColorIndex].upperH);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_5_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].upperS = value;
            sprintf(numbertext, "%d\r\n",colorArray[nowColorIndex].upperS);
            lv_label_set_text(guider_ui.screen_pi_label_13,numbertext);

            sprintf(sendMsg, "%d:%d\r\n",nowColorIndex*8+4,colorArray[nowColorIndex].upperS);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_6_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].upperV = value;
            sprintf(numbertext, "%d\r\n",colorArray[nowColorIndex].upperV);
            lv_label_set_text(guider_ui.screen_pi_label_14,numbertext);

            sprintf(sendMsg, "%d:%d\r\n",nowColorIndex*8+5,colorArray[nowColorIndex].upperV);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_7_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].minArea = value;
            sprintf(numbertext, "%ld\r\n",colorArray[nowColorIndex].minArea);
            lv_label_set_text(guider_ui.screen_pi_label_15,numbertext);

            sprintf(sendMsg, "%d:%ld\r\n",nowColorIndex*8+6,colorArray[nowColorIndex].minArea);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}
static void screen_pi_slider_8_event_handler (lv_event_t *e)//clean
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * slider = lv_event_get_target(e);
    switch (code) {
        case LV_EVENT_VALUE_CHANGED:
        {
            char numbertext[12];
            uint32_t value = lv_slider_get_value(slider);
            colorArray[nowColorIndex].maxArea = value;
            sprintf(numbertext, "%ld\r\n",colorArray[nowColorIndex].maxArea);
            lv_label_set_text(guider_ui.screen_pi_label_16,numbertext);

            sprintf(sendMsg, "%d:%ld\r\n",nowColorIndex*8+7,colorArray[nowColorIndex].maxArea);
            esp_now_send(Pi_mac,(uint8_t*)sendMsg,strlen(sendMsg)+1);
            break;
        }
        default:
            break;
    }
}

void events_init_screen_pi(lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen_pi_slider_1, screen_pi_slider_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_slider_2, screen_pi_slider_2_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_slider_3, screen_pi_slider_3_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_slider_4, screen_pi_slider_4_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_slider_5, screen_pi_slider_5_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_slider_6, screen_pi_slider_6_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_slider_7, screen_pi_slider_7_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_slider_8, screen_pi_slider_8_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_ddlist_1, screen_pi_ddlist_1_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_pi_btn_1, screen_pi_btn_1_event_handler, LV_EVENT_ALL, ui);
}

void events_init(lv_ui *ui)
{

}
