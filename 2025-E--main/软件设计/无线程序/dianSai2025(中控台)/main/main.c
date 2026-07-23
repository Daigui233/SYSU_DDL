/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "tca9554.h"
#include <bsp_ext_io.h>
#include <esp_mac.h>
#include <esp_now.h>

#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include <esp_timer.h>
#include <gui_guider.h>
#include <esp_wifi.h>
#include <interface.h>
#include <nvs_flash.h>
#include <TaskList.h>

#define LV_LVGL_H_INCLUDE_SIMPLE
static const char *TAG = "main";
lv_ui guider_ui;


#define LVGL_TICK_PERIOD_MS    1
static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}


void app_main(void)
{
    // 初始化 NVS 文件系统ine

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // 如果 NVS 分区被截断，需要擦除后重试
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    /* Initialize I2C 400KHz */
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));

    /* Init TCA9554 */
    ESP_ERROR_CHECK(tca9554_init());
    ext_io_t io_conf = BSP_EXT_IO_DEFAULT_CONFIG();
    ext_io_t io_level = BSP_EXT_IO_DEFAULT_LEVEL();
    ESP_ERROR_CHECK(tca9554_set_configuration(io_conf.val));
    ESP_ERROR_CHECK(tca9554_write_output_pins(io_level.val));

    /* init LVGL tick timer */
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));  //

    /* LVGL init */
    lv_init();                  //内核初始化
    lv_port_disp_init();	    //接口初始化
    ESP_LOGW(TAG, "Increase lvgl indev");
    lv_port_indev_init();       //输入设备初始化
    // lv_port_fs_init();       //文件系统初始化

    /* example lvgl demos */
    //lv_demo_music();

    // lv_demo_keypad_encoder();
     //lv_demo_benchmark();
    // lv_demo_stress();
    setup_ui(&guider_ui);
    xTaskCreatePinnedToCore(LVGLTask,"LVGL",1024*4,NULL,3,NULL,0);
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init();
    espnow_app_start();

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(100));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        //lv_timer_handler();
    }
}
