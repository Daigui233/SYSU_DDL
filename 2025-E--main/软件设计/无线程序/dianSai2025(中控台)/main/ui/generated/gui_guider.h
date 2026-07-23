/*
* Copyright 2025 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#ifndef GUI_GUIDER_H
#define GUI_GUIDER_H
#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

typedef struct
{
  
	lv_obj_t *screen_device;
	bool screen_device_del;
	lv_obj_t *screen_device_cont_1;
	lv_obj_t *screen_device_cont_2;
	lv_obj_t *screen_device_btn_1;
	lv_obj_t *screen_device_btn_1_label;
	lv_obj_t *screen_device_ddlist_1;
	lv_obj_t *screen_device_btn_2;
	lv_obj_t *screen_device_btn_2_label;
	lv_obj_t *screen_device_btn_3;
	lv_obj_t *screen_device_btn_3_label;
	lv_obj_t *screen_device_btn_4;
	lv_obj_t *screen_device_btn_4_label;
	lv_obj_t *screen_device_btn_5;
	lv_obj_t *screen_device_btn_5_label;
	lv_obj_t *screen_device_btn_7;
	lv_obj_t *screen_device_btn_7_label;
	lv_obj_t *screen_device_slider_1;
	lv_obj_t *screen_device_slider_2;
	lv_obj_t *screen_device_slider_3;
	lv_obj_t *screen_device_label_1;
	lv_obj_t *screen_device_btn_8;
	lv_obj_t *screen_device_btn_8_label;
	lv_obj_t *screen_pid;
	bool screen_pid_del;
	lv_obj_t *screen_pid_spinbox_1;
	lv_obj_t *screen_pid_spinbox_1_btn_plus;
	lv_obj_t *screen_pid_spinbox_1_btn_minus;
	lv_obj_t *screen_pid_label_1;
	lv_obj_t *screen_pid_spinbox_2;
	lv_obj_t *screen_pid_spinbox_2_btn_plus;
	lv_obj_t *screen_pid_spinbox_2_btn_minus;
	lv_obj_t *screen_pid_label_2;
	lv_obj_t *screen_pid_spinbox_3;
	lv_obj_t *screen_pid_spinbox_3_btn_plus;
	lv_obj_t *screen_pid_spinbox_3_btn_minus;
	lv_obj_t *screen_pid_label_3;
	lv_obj_t *screen_pid_spinbox_4;
	lv_obj_t *screen_pid_spinbox_4_btn_plus;
	lv_obj_t *screen_pid_spinbox_4_btn_minus;
	lv_obj_t *screen_pid_label_4;
	lv_obj_t *screen_pid_label_5;
	lv_obj_t *screen_pid_spinbox_5;
	lv_obj_t *screen_pid_spinbox_5_btn_plus;
	lv_obj_t *screen_pid_spinbox_5_btn_minus;
	lv_obj_t *screen_pid_label_6;
	lv_obj_t *screen_pid_spinbox_6;
	lv_obj_t *screen_pid_spinbox_6_btn_plus;
	lv_obj_t *screen_pid_spinbox_6_btn_minus;
	lv_obj_t *screen_pid_btn_1;
	lv_obj_t *screen_pid_btn_1_label;
	lv_obj_t *screen_pid_btn_2;
	lv_obj_t *screen_pid_btn_2_label;
	lv_obj_t *screen_pid_btn_3;
	lv_obj_t *screen_pid_btn_3_label;
	lv_obj_t *screen_center;
	bool screen_center_del;
	lv_obj_t *screen_center_img_1;
	lv_obj_t *screen_center_btn_1;
	lv_obj_t *screen_center_btn_1_label;
	lv_obj_t *screen_center_btn_2;
	lv_obj_t *screen_center_btn_2_label;
	lv_obj_t *screen_center_btn_3;
	lv_obj_t *screen_center_btn_3_label;
	lv_obj_t *screen_center_btn_4;
	lv_obj_t *screen_center_btn_4_label;
	lv_obj_t *screen_msg;
	bool screen_msg_del;
	lv_obj_t *screen_msg_ta_1;
	lv_obj_t *screen_msg_ddlist_1;
	lv_obj_t *screen_msg_label_1;
	lv_obj_t *screen_msg_label_2;
	lv_obj_t *screen_msg_btn_1;
	lv_obj_t *screen_msg_btn_1_label;
	lv_obj_t *screen_msg_label_3;
	lv_obj_t *screen_msg_label_4;
	lv_obj_t *screen_pi;
	bool screen_pi_del;
	lv_obj_t *screen_pi_label_1;
	lv_obj_t *screen_pi_slider_1;
	lv_obj_t *screen_pi_slider_2;
	lv_obj_t *screen_pi_label_2;
	lv_obj_t *screen_pi_label_3;
	lv_obj_t *screen_pi_label_4;
	lv_obj_t *screen_pi_label_5;
	lv_obj_t *screen_pi_label_6;
	lv_obj_t *screen_pi_label_7;
	lv_obj_t *screen_pi_label_8;
	lv_obj_t *screen_pi_slider_3;
	lv_obj_t *screen_pi_slider_4;
	lv_obj_t *screen_pi_slider_5;
	lv_obj_t *screen_pi_slider_6;
	lv_obj_t *screen_pi_slider_7;
	lv_obj_t *screen_pi_slider_8;
	lv_obj_t *screen_pi_ddlist_1;
	lv_obj_t *screen_pi_btn_1;
	lv_obj_t *screen_pi_btn_1_label;
	lv_obj_t *screen_pi_label_9;
	lv_obj_t *screen_pi_label_10;
	lv_obj_t *screen_pi_label_11;
	lv_obj_t *screen_pi_label_12;
	lv_obj_t *screen_pi_label_13;
	lv_obj_t *screen_pi_label_14;
	lv_obj_t *screen_pi_label_15;
	lv_obj_t *screen_pi_label_16;
	lv_obj_t *g_kb_top_layer;
}lv_ui;

typedef void (*ui_setup_scr_t)(lv_ui * ui);

void ui_init_style(lv_style_t * style);

void ui_load_scr_animation(lv_ui *ui, lv_obj_t ** new_scr, bool new_scr_del, bool * old_scr_del, ui_setup_scr_t setup_scr,
                           lv_scr_load_anim_t anim_type, uint32_t time, uint32_t delay, bool is_clean, bool auto_del);

void ui_animation(void * var, int32_t duration, int32_t delay, int32_t start_value, int32_t end_value, lv_anim_path_cb_t path_cb,
                       uint16_t repeat_cnt, uint32_t repeat_delay, uint32_t playback_time, uint32_t playback_delay,
                       lv_anim_exec_xcb_t exec_cb, lv_anim_start_cb_t start_cb, lv_anim_ready_cb_t ready_cb, lv_anim_deleted_cb_t deleted_cb);


void init_scr_del_flag(lv_ui *ui);

void setup_ui(lv_ui *ui);

void init_keyboard(lv_ui *ui);

extern lv_ui guider_ui;


void setup_scr_screen_device(lv_ui *ui);
void setup_scr_screen_pid(lv_ui *ui);
void setup_scr_screen_center(lv_ui *ui);
void setup_scr_screen_msg(lv_ui *ui);
LV_IMG_DECLARE(_8872841B76C3FAD83BE08EBB3A545CD4_alpha_800x480);

LV_FONT_DECLARE(lv_font_montserratMedium_16)
LV_FONT_DECLARE(lv_font_montserratMedium_12)
LV_FONT_DECLARE(lv_font_montserratMedium_28)
LV_FONT_DECLARE(lv_font_montserratMedium_25)
LV_FONT_DECLARE(lv_font_SourceHanSerifSC_Regular_18)


#ifdef __cplusplus
}
#endif
#endif
