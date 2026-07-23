/*
* Copyright 2025 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"



void setup_scr_screen_device(lv_ui *ui)
{
    //Write codes screen_device
    ui->screen_device = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_device, 800, 480);
    lv_obj_set_scrollbar_mode(ui->screen_device, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen_device, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_device_cont_1
    ui->screen_device_cont_1 = lv_obj_create(ui->screen_device);
    lv_obj_set_pos(ui->screen_device_cont_1, 450, 150);
    lv_obj_set_size(ui->screen_device_cont_1, 300, 300);
    lv_obj_set_scrollbar_mode(ui->screen_device_cont_1, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen_device_cont_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_device_cont_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_device_cont_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_device_cont_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_device_cont_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_device_cont_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_cont_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_cont_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_device_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_device_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_device_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_device_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_cont_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    ///Write codes screen_cont_2
    ui->screen_device_cont_2 = lv_obj_create(ui->screen_device);
    lv_obj_set_pos(ui->screen_device_cont_2, 95, 194);
    lv_obj_set_size(ui->screen_device_cont_2, 200, 200);
    lv_obj_set_scrollbar_mode(ui->screen_device_cont_2, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen_device_cont_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_device_cont_2, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_device_cont_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_device_cont_2, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_device_cont_2, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_cont_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_device_cont_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_cont_2, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_cont_2, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_device_cont_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_device_cont_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_device_cont_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_device_cont_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_cont_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_cont_2, 90, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_btn_1
    ui->screen_device_btn_1 = lv_btn_create(ui->screen_device);
    ui->screen_device_btn_1_label = lv_label_create(ui->screen_device_btn_1);
    lv_label_set_text(ui->screen_device_btn_1_label, "\n");
    lv_label_set_long_mode(ui->screen_device_btn_1_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_device_btn_1_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_device_btn_1, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_device_btn_1_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_device_btn_1, 170, 272);
    lv_obj_set_size(ui->screen_device_btn_1, 50, 50);

    //Write style for screen_device_btn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_btn_1, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_btn_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_device_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_1, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_btn_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_btn_1, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_btn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_1, 90, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_device_ddlist_1
    ui->screen_device_ddlist_1 = lv_dropdown_create(ui->screen_device);
    lv_dropdown_set_options(ui->screen_device_ddlist_1, "Question1\nQuestion2\nQuestion3\nQuestion4\nQuestion5\nQuestion6\nQuestion7");
    lv_obj_set_pos(ui->screen_device_ddlist_1, 0, 0);
    lv_obj_set_size(ui->screen_device_ddlist_1, 130, 30);

    //Write style for screen_device_ddlist_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_device_ddlist_1, 1, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_device_ddlist_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_device_ddlist_1, lv_color_hex(0xe1e6ee), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_device_ddlist_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_ddlist_1, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_device_ddlist_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_ddlist_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_ddlist_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_device_ddlist_1, 8, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_device_ddlist_1, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_device_ddlist_1, 6, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_ddlist_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_ddlist_1, lv_color_hex(0x0D3055), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_ddlist_1, &lv_font_montserratMedium_12, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_ddlist_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_CHECKED for &style_screen_device_ddlist_1_extra_list_selected_checked
    static lv_style_t style_screen_device_ddlist_1_extra_list_selected_checked;
    ui_init_style(&style_screen_device_ddlist_1_extra_list_selected_checked);

    lv_style_set_border_width(&style_screen_device_ddlist_1_extra_list_selected_checked, 1);
    lv_style_set_border_opa(&style_screen_device_ddlist_1_extra_list_selected_checked, 255);
    lv_style_set_border_color(&style_screen_device_ddlist_1_extra_list_selected_checked, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_device_ddlist_1_extra_list_selected_checked, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_device_ddlist_1_extra_list_selected_checked, 3);
    lv_style_set_bg_opa(&style_screen_device_ddlist_1_extra_list_selected_checked, 255);
    lv_style_set_bg_color(&style_screen_device_ddlist_1_extra_list_selected_checked, lv_color_hex(0x00a1b5));
    lv_style_set_bg_grad_dir(&style_screen_device_ddlist_1_extra_list_selected_checked, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_device_ddlist_1), &style_screen_device_ddlist_1_extra_list_selected_checked, LV_PART_SELECTED|LV_STATE_CHECKED);

    //Write style state: LV_STATE_DEFAULT for &style_screen_device_ddlist_1_extra_list_main_default
    static lv_style_t style_screen_device_ddlist_1_extra_list_main_default;
    ui_init_style(&style_screen_device_ddlist_1_extra_list_main_default);

    lv_style_set_max_height(&style_screen_device_ddlist_1_extra_list_main_default, 90);
    lv_style_set_text_color(&style_screen_device_ddlist_1_extra_list_main_default, lv_color_hex(0x0D3055));
    lv_style_set_text_font(&style_screen_device_ddlist_1_extra_list_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_device_ddlist_1_extra_list_main_default, 255);
    lv_style_set_border_width(&style_screen_device_ddlist_1_extra_list_main_default, 1);
    lv_style_set_border_opa(&style_screen_device_ddlist_1_extra_list_main_default, 255);
    lv_style_set_border_color(&style_screen_device_ddlist_1_extra_list_main_default, lv_color_hex(0xe1e6ee));
    lv_style_set_border_side(&style_screen_device_ddlist_1_extra_list_main_default, LV_BORDER_SIDE_FULL);
    lv_style_set_radius(&style_screen_device_ddlist_1_extra_list_main_default, 3);
    lv_style_set_bg_opa(&style_screen_device_ddlist_1_extra_list_main_default, 255);
    lv_style_set_bg_color(&style_screen_device_ddlist_1_extra_list_main_default, lv_color_hex(0xffffff));
    lv_style_set_bg_grad_dir(&style_screen_device_ddlist_1_extra_list_main_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_device_ddlist_1), &style_screen_device_ddlist_1_extra_list_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_device_ddlist_1_extra_list_scrollbar_default
    static lv_style_t style_screen_device_ddlist_1_extra_list_scrollbar_default;
    ui_init_style(&style_screen_device_ddlist_1_extra_list_scrollbar_default);

    lv_style_set_radius(&style_screen_device_ddlist_1_extra_list_scrollbar_default, 3);
    lv_style_set_bg_opa(&style_screen_device_ddlist_1_extra_list_scrollbar_default, 255);
    lv_style_set_bg_color(&style_screen_device_ddlist_1_extra_list_scrollbar_default, lv_color_hex(0x00ff00));
    lv_style_set_bg_grad_dir(&style_screen_device_ddlist_1_extra_list_scrollbar_default, LV_GRAD_DIR_NONE);
    lv_obj_add_style(lv_dropdown_get_list(ui->screen_device_ddlist_1), &style_screen_device_ddlist_1_extra_list_scrollbar_default, LV_PART_SCROLLBAR|LV_STATE_DEFAULT);

    //Write codes screen_device_btn_2
    ui->screen_device_btn_2 = lv_btn_create(ui->screen_device);
    ui->screen_device_btn_2_label = lv_label_create(ui->screen_device_btn_2);
    lv_label_set_text(ui->screen_device_btn_2_label, "Reset");
    lv_label_set_long_mode(ui->screen_device_btn_2_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_device_btn_2_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_device_btn_2, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_device_btn_2_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_device_btn_2, 87, 98);
    lv_obj_set_size(ui->screen_device_btn_2, 50, 50);

    //Write style for screen_device_btn_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_device_btn_2, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_btn_2, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_btn_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_btn_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_device_btn_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_2, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_device_btn_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_btn_2, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_btn_2, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_btn_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes 目标点
    ui->screen_device_btn_3 = lv_btn_create(ui->screen_device);
    ui->screen_device_btn_3_label = lv_label_create(ui->screen_device_btn_3);
    lv_label_set_text(ui->screen_device_btn_3_label, "");
    lv_label_set_long_mode(ui->screen_device_btn_3_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_device_btn_3_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_device_btn_3, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_device_btn_3_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_device_btn_3, 600, 300);
    lv_obj_set_size(ui->screen_device_btn_3, 15, 15);

    //Write style for screen_device_btn_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_btn_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_btn_3, lv_color_hex(0xd50c0c), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_btn_3, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_device_btn_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_3, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_btn_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_btn_3, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_btn_3, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_btn_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_btn_3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_3, 90, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_device_btn_4
    ui->screen_device_btn_4 = lv_btn_create(ui->screen_device);
    ui->screen_device_btn_4_label = lv_label_create(ui->screen_device_btn_4);
    lv_label_set_text(ui->screen_device_btn_4_label, "");
    lv_label_set_long_mode(ui->screen_device_btn_4_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_device_btn_4_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_device_btn_4, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_device_btn_4_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_device_btn_4, 348, 103);
    lv_obj_set_size(ui->screen_device_btn_4, 15, 15);

    //Write style for screen_device_btn_4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_btn_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_device_btn_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_4, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_btn_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_btn_4, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_btn_4, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_btn_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_btn_4, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_device_btn_5
    ui->screen_device_btn_5 = lv_btn_create(ui->screen_device);
    ui->screen_device_btn_5_label = lv_label_create(ui->screen_device_btn_5);
    lv_label_set_text(ui->screen_device_btn_5_label, "PID\n");
    lv_label_set_long_mode(ui->screen_device_btn_5_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_device_btn_5_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_device_btn_5, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_device_btn_5_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_device_btn_5, 185, 98);
    lv_obj_set_size(ui->screen_device_btn_5, 50, 50);

    //Write style for screen_device_btn_5, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_btn_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_btn_5, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_btn_5, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_device_btn_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_5, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_btn_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_btn_5, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_btn_5, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_btn_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_btn_5, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_device_btn_7
    ui->screen_device_btn_7 = lv_btn_create(ui->screen_device);
    ui->screen_device_btn_7_label = lv_label_create(ui->screen_device_btn_7);
    lv_label_set_text(ui->screen_device_btn_7_label, "Shut\n");
    lv_label_set_long_mode(ui->screen_device_btn_7_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_device_btn_7_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_device_btn_7, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_device_btn_7_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_device_btn_7, 275, 98);
    lv_obj_set_size(ui->screen_device_btn_7, 50, 50);

    //Write style for screen_device_btn_7, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_btn_7, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_btn_7, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_btn_7, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_device_btn_7, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_7, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_btn_7, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_btn_7, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_btn_7, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_btn_7, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_btn_7, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_device_slider_1
    ui->screen_device_slider_1 = lv_slider_create(ui->screen_device);
    lv_slider_set_range(ui->screen_device_slider_1, 0, 100);
    lv_slider_set_mode(ui->screen_device_slider_1, LV_SLIDER_MODE_NORMAL);
    lv_slider_set_value(ui->screen_device_slider_1, 50, LV_ANIM_OFF);
    lv_obj_set_pos(ui->screen_device_slider_1, 450, 16);
    lv_obj_set_size(ui->screen_device_slider_1, 160, 8);

    //Write style for screen_device_slider_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_1, 60, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_1, 50, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_slider_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui->screen_device_slider_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_device_slider_1, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_1, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_1, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_1, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_1, 50, LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write style for screen_device_slider_1, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_1, 255, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_1, lv_color_hex(0x2195f6), LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_1, LV_GRAD_DIR_NONE, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_1, 50, LV_PART_KNOB|LV_STATE_DEFAULT);

    //Write codes screen_device_slider_2
    ui->screen_device_slider_2 = lv_slider_create(ui->screen_device);
    lv_slider_set_range(ui->screen_device_slider_2, 0, 100);
    lv_slider_set_mode(ui->screen_device_slider_2, LV_SLIDER_MODE_NORMAL);
    lv_slider_set_value(ui->screen_device_slider_2, 50, LV_ANIM_OFF);
    lv_obj_set_pos(ui->screen_device_slider_2, 450, 58);
    lv_obj_set_size(ui->screen_device_slider_2, 160, 8);

    //Write style for screen_device_slider_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_2, 60, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_2, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_2, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_2, 50, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui->screen_device_slider_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_slider_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_device_slider_2, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_2, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_2, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_2, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_2, 50, LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write style for screen_device_slider_2, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_2, 255, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_2, lv_color_hex(0x2195f6), LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_2, LV_GRAD_DIR_NONE, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_2, 50, LV_PART_KNOB|LV_STATE_DEFAULT);

    //Write codes screen_device_slider_3
    ui->screen_device_slider_3 = lv_slider_create(ui->screen_device);
    lv_slider_set_range(ui->screen_device_slider_3, 0, 100);
    lv_slider_set_mode(ui->screen_device_slider_3, LV_SLIDER_MODE_NORMAL);
    lv_slider_set_value(ui->screen_device_slider_3, 50, LV_ANIM_OFF);
    lv_obj_set_pos(ui->screen_device_slider_3, 450, 106);
    lv_obj_set_size(ui->screen_device_slider_3, 160, 8);

    //Write style for screen_device_slider_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_3, 60, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_3, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_3, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_3, 50, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_outline_width(ui->screen_device_slider_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_slider_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_device_slider_3, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_3, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_3, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_3, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_3, 50, LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write style for screen_device_slider_3, Part: LV_PART_KNOB, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_slider_3, 255, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_slider_3, lv_color_hex(0x2195f6), LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_slider_3, LV_GRAD_DIR_NONE, LV_PART_KNOB|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_slider_3, 50, LV_PART_KNOB|LV_STATE_DEFAULT);

    //Write codes screen_device_label_1
    ui->screen_device_label_1 = lv_label_create(ui->screen_device);
    lv_label_set_text(ui->screen_device_label_1, "");
    lv_label_set_long_mode(ui->screen_device_label_1, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_device_label_1, 191, 24);
    lv_obj_set_size(ui->screen_device_label_1, 131, 25);

    //Write style for screen_device_label_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_label_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_label_1, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_label_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_device_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_device_btn_8
    ui->screen_device_btn_8 = lv_btn_create(ui->screen_device);
    ui->screen_device_btn_8_label = lv_label_create(ui->screen_device_btn_8);
    lv_label_set_text(ui->screen_device_btn_8_label, "Back");
    lv_label_set_long_mode(ui->screen_device_btn_8_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_device_btn_8_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_device_btn_8, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_device_btn_8_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_device_btn_8, 275, 34);
    lv_obj_set_size(ui->screen_device_btn_8, 50, 50);

    //Write style for screen_device_btn_8, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_device_btn_8, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_device_btn_8, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_device_btn_8, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_device_btn_8, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_device_btn_8, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_device_btn_8, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_device_btn_8, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_device_btn_8, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_device_btn_8, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_device_btn_8, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //The custom code of screen_device.


    //Update current screen layout.
    lv_obj_update_layout(ui->screen_device);

    //Init events for screen.
    events_init_screen_device(ui);
}
