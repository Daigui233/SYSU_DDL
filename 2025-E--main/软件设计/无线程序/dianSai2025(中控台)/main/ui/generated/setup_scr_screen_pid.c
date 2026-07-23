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



void setup_scr_screen_pid(lv_ui *ui)
{
    //Write codes screen_pid
    ui->screen_pid = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen_pid, 800, 480);
    lv_obj_set_scrollbar_mode(ui->screen_pid, LV_SCROLLBAR_MODE_OFF);

    //Write style for screen_pid, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_spinbox_1
    ui->screen_pid_spinbox_1 = lv_spinbox_create(ui->screen_pid);
    lv_obj_set_pos(ui->screen_pid_spinbox_1, 75, 60);
    lv_obj_set_width(ui->screen_pid_spinbox_1, 120);
    lv_obj_set_height(ui->screen_pid_spinbox_1, 60);
    lv_spinbox_set_digit_format(ui->screen_pid_spinbox_1, 5, 2);
    lv_spinbox_set_range(ui->screen_pid_spinbox_1, -99999, 99999);
    lv_coord_t screen_pid_spinbox_1_h = lv_obj_get_height(ui->screen_pid_spinbox_1);
    ui->screen_pid_spinbox_1_btn_plus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_1_btn_plus, screen_pid_spinbox_1_h, screen_pid_spinbox_1_h);
    lv_obj_align_to(ui->screen_pid_spinbox_1_btn_plus, ui->screen_pid_spinbox_1, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_1_btn_plus, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_1_btn_plus, lv_screen_pid_spinbox_1_increment_event_cb, LV_EVENT_ALL, NULL);
    ui->screen_pid_spinbox_1_btn_minus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_1_btn_minus, screen_pid_spinbox_1_h, screen_pid_spinbox_1_h);
    lv_obj_align_to(ui->screen_pid_spinbox_1_btn_minus, ui->screen_pid_spinbox_1, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_1_btn_minus, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_1_btn_minus, lv_screen_pid_spinbox_1_decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(ui->screen_pid_spinbox_1, 75, 60);

    //Write style for screen_pid_spinbox_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_1, 123, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_1, lv_color_hex(0xd000ff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_spinbox_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_pid_spinbox_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_pid_spinbox_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_pid_spinbox_1, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_spinbox_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_spinbox_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_spinbox_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_spinbox_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_1, &lv_font_montserratMedium_28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_spinbox_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_spinbox_1, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_spinbox_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_pid_spinbox_1, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_1, lv_color_hex(0xffffff), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_1, &lv_font_montserratMedium_12, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_1, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_1, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_1, lv_color_hex(0x2195f6), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_1, LV_GRAD_DIR_NONE, LV_PART_CURSOR|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_pid_spinbox_1_extra_btns_main_default
    static lv_style_t style_screen_pid_spinbox_1_extra_btns_main_default;
    ui_init_style(&style_screen_pid_spinbox_1_extra_btns_main_default);

    lv_style_set_text_color(&style_screen_pid_spinbox_1_extra_btns_main_default, lv_color_hex(0xffffff));
    lv_style_set_text_font(&style_screen_pid_spinbox_1_extra_btns_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_pid_spinbox_1_extra_btns_main_default, 255);
    lv_style_set_bg_opa(&style_screen_pid_spinbox_1_extra_btns_main_default, 255);
    lv_style_set_bg_color(&style_screen_pid_spinbox_1_extra_btns_main_default, lv_color_hex(0x2195f6));
    lv_style_set_bg_grad_dir(&style_screen_pid_spinbox_1_extra_btns_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_screen_pid_spinbox_1_extra_btns_main_default, 0);
    lv_style_set_radius(&style_screen_pid_spinbox_1_extra_btns_main_default, 5);
    lv_style_set_shadow_width(&style_screen_pid_spinbox_1_extra_btns_main_default, 0);
    lv_obj_add_style(ui->screen_pid_spinbox_1_btn_plus, &style_screen_pid_spinbox_1_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(ui->screen_pid_spinbox_1_btn_minus, &style_screen_pid_spinbox_1_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_label_1
    ui->screen_pid_label_1 = lv_label_create(ui->screen_pid);
    lv_label_set_text(ui->screen_pid_label_1, "PositionKp");
    lv_label_set_long_mode(ui->screen_pid_label_1, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_pid_label_1, 58, 138);
    lv_obj_set_size(ui->screen_pid_label_1, 150, 40);

    //Write style for screen_pid_label_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_label_1, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_label_1, &lv_font_montserratMedium_25, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_label_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_spinbox_2
    ui->screen_pid_spinbox_2 = lv_spinbox_create(ui->screen_pid);
    lv_obj_set_pos(ui->screen_pid_spinbox_2, 344, 60);
    lv_obj_set_width(ui->screen_pid_spinbox_2, 120);
    lv_obj_set_height(ui->screen_pid_spinbox_2, 60);
    lv_spinbox_set_digit_format(ui->screen_pid_spinbox_2, 5, 3);
    lv_spinbox_set_range(ui->screen_pid_spinbox_2, -99999, 99999);
    lv_coord_t screen_pid_spinbox_2_h = lv_obj_get_height(ui->screen_pid_spinbox_2);
    ui->screen_pid_spinbox_2_btn_plus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_2_btn_plus, screen_pid_spinbox_2_h, screen_pid_spinbox_2_h);
    lv_obj_align_to(ui->screen_pid_spinbox_2_btn_plus, ui->screen_pid_spinbox_2, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_2_btn_plus, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_2_btn_plus, lv_screen_pid_spinbox_2_increment_event_cb, LV_EVENT_ALL, NULL);
    ui->screen_pid_spinbox_2_btn_minus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_2_btn_minus, screen_pid_spinbox_2_h, screen_pid_spinbox_2_h);
    lv_obj_align_to(ui->screen_pid_spinbox_2_btn_minus, ui->screen_pid_spinbox_2, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_2_btn_minus, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_2_btn_minus, lv_screen_pid_spinbox_2_decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(ui->screen_pid_spinbox_2, 344, 60);

    //Write style for screen_pid_spinbox_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_2, 123, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_2, lv_color_hex(0xd000ff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_2, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_spinbox_2, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_pid_spinbox_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_pid_spinbox_2, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_pid_spinbox_2, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_spinbox_2, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_spinbox_2, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_spinbox_2, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_spinbox_2, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_2, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_2, &lv_font_montserratMedium_28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_spinbox_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_spinbox_2, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_spinbox_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_pid_spinbox_2, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_2, lv_color_hex(0xffffff), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_2, &lv_font_montserratMedium_12, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_2, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_2, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_2, lv_color_hex(0x2195f6), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_2, LV_GRAD_DIR_NONE, LV_PART_CURSOR|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_pid_spinbox_2_extra_btns_main_default
    static lv_style_t style_screen_pid_spinbox_2_extra_btns_main_default;
    ui_init_style(&style_screen_pid_spinbox_2_extra_btns_main_default);

    lv_style_set_text_color(&style_screen_pid_spinbox_2_extra_btns_main_default, lv_color_hex(0xffffff));
    lv_style_set_text_font(&style_screen_pid_spinbox_2_extra_btns_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_pid_spinbox_2_extra_btns_main_default, 255);
    lv_style_set_bg_opa(&style_screen_pid_spinbox_2_extra_btns_main_default, 255);
    lv_style_set_bg_color(&style_screen_pid_spinbox_2_extra_btns_main_default, lv_color_hex(0x2195f6));
    lv_style_set_bg_grad_dir(&style_screen_pid_spinbox_2_extra_btns_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_screen_pid_spinbox_2_extra_btns_main_default, 0);
    lv_style_set_radius(&style_screen_pid_spinbox_2_extra_btns_main_default, 5);
    lv_style_set_shadow_width(&style_screen_pid_spinbox_2_extra_btns_main_default, 0);
    lv_obj_add_style(ui->screen_pid_spinbox_2_btn_plus, &style_screen_pid_spinbox_2_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(ui->screen_pid_spinbox_2_btn_minus, &style_screen_pid_spinbox_2_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_label_2
    ui->screen_pid_label_2 = lv_label_create(ui->screen_pid);
    lv_label_set_text(ui->screen_pid_label_2, "PositionKi\n");
    lv_label_set_long_mode(ui->screen_pid_label_2, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_pid_label_2, 328, 138);
    lv_obj_set_size(ui->screen_pid_label_2, 150, 40);

    //Write style for screen_pid_label_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_label_2, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_label_2, &lv_font_montserratMedium_25, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_label_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_label_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_spinbox_3
    ui->screen_pid_spinbox_3 = lv_spinbox_create(ui->screen_pid);
    lv_obj_set_pos(ui->screen_pid_spinbox_3, 611, 60);
    lv_obj_set_width(ui->screen_pid_spinbox_3, 120);
    lv_obj_set_height(ui->screen_pid_spinbox_3, 60);
    lv_spinbox_set_digit_format(ui->screen_pid_spinbox_3, 5, 3);
    lv_spinbox_set_range(ui->screen_pid_spinbox_3, -99999, 99999);
    lv_coord_t screen_pid_spinbox_3_h = lv_obj_get_height(ui->screen_pid_spinbox_3);
    ui->screen_pid_spinbox_3_btn_plus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_3_btn_plus, screen_pid_spinbox_3_h, screen_pid_spinbox_3_h);
    lv_obj_align_to(ui->screen_pid_spinbox_3_btn_plus, ui->screen_pid_spinbox_3, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_3_btn_plus, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_3_btn_plus, lv_screen_pid_spinbox_3_increment_event_cb, LV_EVENT_ALL, NULL);
    ui->screen_pid_spinbox_3_btn_minus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_3_btn_minus, screen_pid_spinbox_3_h, screen_pid_spinbox_3_h);
    lv_obj_align_to(ui->screen_pid_spinbox_3_btn_minus, ui->screen_pid_spinbox_3, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_3_btn_minus, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_3_btn_minus, lv_screen_pid_spinbox_3_decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(ui->screen_pid_spinbox_3, 611, 60);

    //Write style for screen_pid_spinbox_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_3, 125, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_3, lv_color_hex(0xd000ff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_3, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_spinbox_3, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_pid_spinbox_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_pid_spinbox_3, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_pid_spinbox_3, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_spinbox_3, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_spinbox_3, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_spinbox_3, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_spinbox_3, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_3, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_3, &lv_font_montserratMedium_28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_spinbox_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_spinbox_3, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_spinbox_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_pid_spinbox_3, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_3, lv_color_hex(0xffffff), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_3, &lv_font_montserratMedium_12, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_3, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_3, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_3, lv_color_hex(0x2195f6), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_3, LV_GRAD_DIR_NONE, LV_PART_CURSOR|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_pid_spinbox_3_extra_btns_main_default
    static lv_style_t style_screen_pid_spinbox_3_extra_btns_main_default;
    ui_init_style(&style_screen_pid_spinbox_3_extra_btns_main_default);

    lv_style_set_text_color(&style_screen_pid_spinbox_3_extra_btns_main_default, lv_color_hex(0xffffff));
    lv_style_set_text_font(&style_screen_pid_spinbox_3_extra_btns_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_pid_spinbox_3_extra_btns_main_default, 255);
    lv_style_set_bg_opa(&style_screen_pid_spinbox_3_extra_btns_main_default, 255);
    lv_style_set_bg_color(&style_screen_pid_spinbox_3_extra_btns_main_default, lv_color_hex(0x2195f6));
    lv_style_set_bg_grad_dir(&style_screen_pid_spinbox_3_extra_btns_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_screen_pid_spinbox_3_extra_btns_main_default, 0);
    lv_style_set_radius(&style_screen_pid_spinbox_3_extra_btns_main_default, 5);
    lv_style_set_shadow_width(&style_screen_pid_spinbox_3_extra_btns_main_default, 0);
    lv_obj_add_style(ui->screen_pid_spinbox_3_btn_plus, &style_screen_pid_spinbox_3_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(ui->screen_pid_spinbox_3_btn_minus, &style_screen_pid_spinbox_3_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_label_3
    ui->screen_pid_label_3 = lv_label_create(ui->screen_pid);
    lv_label_set_text(ui->screen_pid_label_3, "PositionKd");
    lv_label_set_long_mode(ui->screen_pid_label_3, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_pid_label_3, 601, 138);
    lv_obj_set_size(ui->screen_pid_label_3, 150, 40);

    //Write style for screen_pid_label_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_label_3, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_label_3, &lv_font_montserratMedium_25, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_label_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_label_3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_label_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_spinbox_4
    ui->screen_pid_spinbox_4 = lv_spinbox_create(ui->screen_pid);
    lv_obj_set_pos(ui->screen_pid_spinbox_4, 75, 307);
    lv_obj_set_width(ui->screen_pid_spinbox_4, 120);
    lv_obj_set_height(ui->screen_pid_spinbox_4, 60);
    lv_spinbox_set_digit_format(ui->screen_pid_spinbox_4, 5, 3);
    lv_spinbox_set_range(ui->screen_pid_spinbox_4, -99999, 99999);
    lv_coord_t screen_pid_spinbox_4_h = lv_obj_get_height(ui->screen_pid_spinbox_4);
    ui->screen_pid_spinbox_4_btn_plus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_4_btn_plus, screen_pid_spinbox_4_h, screen_pid_spinbox_4_h);
    lv_obj_align_to(ui->screen_pid_spinbox_4_btn_plus, ui->screen_pid_spinbox_4, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_4_btn_plus, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_4_btn_plus, lv_screen_pid_spinbox_4_increment_event_cb, LV_EVENT_ALL, NULL);
    ui->screen_pid_spinbox_4_btn_minus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_4_btn_minus, screen_pid_spinbox_4_h, screen_pid_spinbox_4_h);
    lv_obj_align_to(ui->screen_pid_spinbox_4_btn_minus, ui->screen_pid_spinbox_4, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_4_btn_minus, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_4_btn_minus, lv_screen_pid_spinbox_4_decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(ui->screen_pid_spinbox_4, 75, 307);

    //Write style for screen_pid_spinbox_4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_4, 136, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_4, lv_color_hex(0xd000ff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_4, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_spinbox_4, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_pid_spinbox_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_pid_spinbox_4, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_pid_spinbox_4, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_spinbox_4, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_spinbox_4, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_spinbox_4, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_spinbox_4, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_4, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_4, &lv_font_montserratMedium_28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_spinbox_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_spinbox_4, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_spinbox_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_pid_spinbox_4, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_4, lv_color_hex(0xffffff), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_4, &lv_font_montserratMedium_12, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_4, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_4, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_4, lv_color_hex(0x2195f6), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_4, LV_GRAD_DIR_NONE, LV_PART_CURSOR|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_pid_spinbox_4_extra_btns_main_default
    static lv_style_t style_screen_pid_spinbox_4_extra_btns_main_default;
    ui_init_style(&style_screen_pid_spinbox_4_extra_btns_main_default);

    lv_style_set_text_color(&style_screen_pid_spinbox_4_extra_btns_main_default, lv_color_hex(0xffffff));
    lv_style_set_text_font(&style_screen_pid_spinbox_4_extra_btns_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_pid_spinbox_4_extra_btns_main_default, 255);
    lv_style_set_bg_opa(&style_screen_pid_spinbox_4_extra_btns_main_default, 255);
    lv_style_set_bg_color(&style_screen_pid_spinbox_4_extra_btns_main_default, lv_color_hex(0x2195f6));
    lv_style_set_bg_grad_dir(&style_screen_pid_spinbox_4_extra_btns_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_screen_pid_spinbox_4_extra_btns_main_default, 0);
    lv_style_set_radius(&style_screen_pid_spinbox_4_extra_btns_main_default, 5);
    lv_style_set_shadow_width(&style_screen_pid_spinbox_4_extra_btns_main_default, 0);
    lv_obj_add_style(ui->screen_pid_spinbox_4_btn_plus, &style_screen_pid_spinbox_4_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(ui->screen_pid_spinbox_4_btn_minus, &style_screen_pid_spinbox_4_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_label_4
    ui->screen_pid_label_4 = lv_label_create(ui->screen_pid);
    lv_label_set_text(ui->screen_pid_label_4, "velocityKp");
    lv_label_set_long_mode(ui->screen_pid_label_4, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_pid_label_4, 58, 393);
    lv_obj_set_size(ui->screen_pid_label_4, 150, 40);

    //Write style for screen_pid_label_4, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_label_4, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_label_4, &lv_font_montserratMedium_25, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_label_4, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_label_4, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_label_4, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_label_5
    ui->screen_pid_label_5 = lv_label_create(ui->screen_pid);
    lv_label_set_text(ui->screen_pid_label_5, "velocityKi");
    lv_label_set_long_mode(ui->screen_pid_label_5, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_pid_label_5, 328, 398);
    lv_obj_set_size(ui->screen_pid_label_5, 150, 40);

    //Write style for screen_pid_label_5, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_label_5, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_label_5, &lv_font_montserratMedium_25, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_label_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_label_5, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_label_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_spinbox_5
    ui->screen_pid_spinbox_5 = lv_spinbox_create(ui->screen_pid);
    lv_obj_set_pos(ui->screen_pid_spinbox_5, 344, 307);
    lv_obj_set_width(ui->screen_pid_spinbox_5, 120);
    lv_obj_set_height(ui->screen_pid_spinbox_5, 60);
    lv_spinbox_set_digit_format(ui->screen_pid_spinbox_5, 5, 3);
    lv_spinbox_set_range(ui->screen_pid_spinbox_5, -99999, 99999);
    lv_coord_t screen_pid_spinbox_5_h = lv_obj_get_height(ui->screen_pid_spinbox_5);
    ui->screen_pid_spinbox_5_btn_plus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_5_btn_plus, screen_pid_spinbox_5_h, screen_pid_spinbox_5_h);
    lv_obj_align_to(ui->screen_pid_spinbox_5_btn_plus, ui->screen_pid_spinbox_5, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_5_btn_plus, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_5_btn_plus, lv_screen_pid_spinbox_5_increment_event_cb, LV_EVENT_ALL, NULL);
    ui->screen_pid_spinbox_5_btn_minus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_5_btn_minus, screen_pid_spinbox_5_h, screen_pid_spinbox_5_h);
    lv_obj_align_to(ui->screen_pid_spinbox_5_btn_minus, ui->screen_pid_spinbox_5, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_5_btn_minus, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_5_btn_minus, lv_screen_pid_spinbox_5_decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(ui->screen_pid_spinbox_5, 344, 307);

    //Write style for screen_pid_spinbox_5, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_5, 134, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_5, lv_color_hex(0xd000ff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_5, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_spinbox_5, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_pid_spinbox_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_pid_spinbox_5, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_pid_spinbox_5, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_spinbox_5, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_spinbox_5, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_spinbox_5, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_spinbox_5, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_5, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_5, &lv_font_montserratMedium_28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_5, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_spinbox_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_spinbox_5, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_spinbox_5, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_pid_spinbox_5, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_5, lv_color_hex(0xffffff), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_5, &lv_font_montserratMedium_12, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_5, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_5, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_5, lv_color_hex(0x2195f6), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_5, LV_GRAD_DIR_NONE, LV_PART_CURSOR|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_pid_spinbox_5_extra_btns_main_default
    static lv_style_t style_screen_pid_spinbox_5_extra_btns_main_default;
    ui_init_style(&style_screen_pid_spinbox_5_extra_btns_main_default);

    lv_style_set_text_color(&style_screen_pid_spinbox_5_extra_btns_main_default, lv_color_hex(0xffffff));
    lv_style_set_text_font(&style_screen_pid_spinbox_5_extra_btns_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_pid_spinbox_5_extra_btns_main_default, 255);
    lv_style_set_bg_opa(&style_screen_pid_spinbox_5_extra_btns_main_default, 255);
    lv_style_set_bg_color(&style_screen_pid_spinbox_5_extra_btns_main_default, lv_color_hex(0x2195f6));
    lv_style_set_bg_grad_dir(&style_screen_pid_spinbox_5_extra_btns_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_screen_pid_spinbox_5_extra_btns_main_default, 0);
    lv_style_set_radius(&style_screen_pid_spinbox_5_extra_btns_main_default, 5);
    lv_style_set_shadow_width(&style_screen_pid_spinbox_5_extra_btns_main_default, 0);
    lv_obj_add_style(ui->screen_pid_spinbox_5_btn_plus, &style_screen_pid_spinbox_5_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(ui->screen_pid_spinbox_5_btn_minus, &style_screen_pid_spinbox_5_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_label_6
    ui->screen_pid_label_6 = lv_label_create(ui->screen_pid);
    lv_label_set_text(ui->screen_pid_label_6, "velocityKd");
    lv_label_set_long_mode(ui->screen_pid_label_6, LV_LABEL_LONG_WRAP);
    lv_obj_set_pos(ui->screen_pid_label_6, 601, 393);
    lv_obj_set_size(ui->screen_pid_label_6, 150, 40);

    //Write style for screen_pid_label_6, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_label_6, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_label_6, &lv_font_montserratMedium_25, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_label_6, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_label_6, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_label_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_spinbox_6
    ui->screen_pid_spinbox_6 = lv_spinbox_create(ui->screen_pid);
    lv_obj_set_pos(ui->screen_pid_spinbox_6, 611, 307);
    lv_obj_set_width(ui->screen_pid_spinbox_6, 120);
    lv_obj_set_height(ui->screen_pid_spinbox_6, 60);
    lv_spinbox_set_digit_format(ui->screen_pid_spinbox_6, 5, 3);
    lv_spinbox_set_range(ui->screen_pid_spinbox_6, -99999, 99999);
    lv_coord_t screen_pid_spinbox_6_h = lv_obj_get_height(ui->screen_pid_spinbox_6);
    ui->screen_pid_spinbox_6_btn_plus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_6_btn_plus, screen_pid_spinbox_6_h, screen_pid_spinbox_6_h);
    lv_obj_align_to(ui->screen_pid_spinbox_6_btn_plus, ui->screen_pid_spinbox_6, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_6_btn_plus, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_6_btn_plus, lv_screen_pid_spinbox_6_increment_event_cb, LV_EVENT_ALL, NULL);
    ui->screen_pid_spinbox_6_btn_minus = lv_btn_create(ui->screen_pid);
    lv_obj_set_size(ui->screen_pid_spinbox_6_btn_minus, screen_pid_spinbox_6_h, screen_pid_spinbox_6_h);
    lv_obj_align_to(ui->screen_pid_spinbox_6_btn_minus, ui->screen_pid_spinbox_6, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(ui->screen_pid_spinbox_6_btn_minus, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(ui->screen_pid_spinbox_6_btn_minus, lv_screen_pid_spinbox_6_decrement_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(ui->screen_pid_spinbox_6, 611, 307);

    //Write style for screen_pid_spinbox_6, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_6, 121, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_6, lv_color_hex(0xd000ff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_6, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_spinbox_6, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui->screen_pid_spinbox_6, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui->screen_pid_spinbox_6, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_side(ui->screen_pid_spinbox_6, LV_BORDER_SIDE_FULL, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->screen_pid_spinbox_6, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->screen_pid_spinbox_6, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->screen_pid_spinbox_6, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->screen_pid_spinbox_6, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_6, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_6, &lv_font_montserratMedium_28, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_6, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->screen_pid_spinbox_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_spinbox_6, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_spinbox_6, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for screen_pid_spinbox_6, Part: LV_PART_CURSOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_text_color(ui->screen_pid_spinbox_6, lv_color_hex(0xffffff), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_spinbox_6, &lv_font_montserratMedium_12, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_spinbox_6, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->screen_pid_spinbox_6, 255, LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_spinbox_6, lv_color_hex(0x2195f6), LV_PART_CURSOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_spinbox_6, LV_GRAD_DIR_NONE, LV_PART_CURSOR|LV_STATE_DEFAULT);

    //Write style state: LV_STATE_DEFAULT for &style_screen_pid_spinbox_6_extra_btns_main_default
    static lv_style_t style_screen_pid_spinbox_6_extra_btns_main_default;
    ui_init_style(&style_screen_pid_spinbox_6_extra_btns_main_default);

    lv_style_set_text_color(&style_screen_pid_spinbox_6_extra_btns_main_default, lv_color_hex(0xffffff));
    lv_style_set_text_font(&style_screen_pid_spinbox_6_extra_btns_main_default, &lv_font_montserratMedium_12);
    lv_style_set_text_opa(&style_screen_pid_spinbox_6_extra_btns_main_default, 255);
    lv_style_set_bg_opa(&style_screen_pid_spinbox_6_extra_btns_main_default, 255);
    lv_style_set_bg_color(&style_screen_pid_spinbox_6_extra_btns_main_default, lv_color_hex(0x2195f6));
    lv_style_set_bg_grad_dir(&style_screen_pid_spinbox_6_extra_btns_main_default, LV_GRAD_DIR_NONE);
    lv_style_set_border_width(&style_screen_pid_spinbox_6_extra_btns_main_default, 0);
    lv_style_set_radius(&style_screen_pid_spinbox_6_extra_btns_main_default, 5);
    lv_style_set_shadow_width(&style_screen_pid_spinbox_6_extra_btns_main_default, 0);
    lv_obj_add_style(ui->screen_pid_spinbox_6_btn_plus, &style_screen_pid_spinbox_6_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_add_style(ui->screen_pid_spinbox_6_btn_minus, &style_screen_pid_spinbox_6_extra_btns_main_default, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_btn_1
    ui->screen_pid_btn_1 = lv_btn_create(ui->screen_pid);
    ui->screen_pid_btn_1_label = lv_label_create(ui->screen_pid_btn_1);
    lv_label_set_text(ui->screen_pid_btn_1_label, "Back\n");
    lv_label_set_long_mode(ui->screen_pid_btn_1_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_pid_btn_1_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_pid_btn_1, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_pid_btn_1_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_pid_btn_1, 38, 209);
    lv_obj_set_size(ui->screen_pid_btn_1, 50, 50);

    //Write style for screen_pid_btn_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_btn_1, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_btn_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_btn_1, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_btn_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_btn_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_btn_1, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_btn_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_btn_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_btn_2
    ui->screen_pid_btn_2 = lv_btn_create(ui->screen_pid);
    ui->screen_pid_btn_2_label = lv_label_create(ui->screen_pid_btn_2);
    lv_label_set_text(ui->screen_pid_btn_2_label, "X\n\n");
    lv_label_set_long_mode(ui->screen_pid_btn_2_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_pid_btn_2_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_pid_btn_2, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_pid_btn_2_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_pid_btn_2, 134, 209);
    lv_obj_set_size(ui->screen_pid_btn_2, 50, 50);

    //Write style for screen_pid_btn_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_btn_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_btn_2, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_btn_2, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_btn_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_btn_2, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_btn_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_btn_2, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_btn_2, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_btn_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_btn_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes screen_pid_btn_3
    ui->screen_pid_btn_3 = lv_btn_create(ui->screen_pid);
    ui->screen_pid_btn_3_label = lv_label_create(ui->screen_pid_btn_3);
    lv_label_set_text(ui->screen_pid_btn_3_label, "Y\n\n");
    lv_label_set_long_mode(ui->screen_pid_btn_3_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(ui->screen_pid_btn_3_label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_pad_all(ui->screen_pid_btn_3, 0, LV_STATE_DEFAULT);
    lv_obj_set_width(ui->screen_pid_btn_3_label, LV_PCT(100));
    lv_obj_set_pos(ui->screen_pid_btn_3, 220, 209);
    lv_obj_set_size(ui->screen_pid_btn_3, 50, 50);

    //Write style for screen_pid_btn_3, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->screen_pid_btn_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->screen_pid_btn_3, lv_color_hex(0x161718), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->screen_pid_btn_3, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui->screen_pid_btn_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->screen_pid_btn_3, 5, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->screen_pid_btn_3, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->screen_pid_btn_3, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->screen_pid_btn_3, &lv_font_montserratMedium_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->screen_pid_btn_3, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->screen_pid_btn_3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);

    //The custom code of screen_pid.


    //Update current screen layout.
    lv_obj_update_layout(ui->screen_pid);

    //Init events for screen.
    events_init_screen_pid(ui);
}
