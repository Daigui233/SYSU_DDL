#include "interface.h"
#include <esp_event_base.h>
#include <esp_log.h>
#include <esp_wifi_types_generic.h>
#include <gui_guider.h>
#include <mqtt_client.h>
#include <TaskList.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include <esp_http_client.h>
#include <esp_now.h>
#include <esp_tls.h>
#define TOPIC_MAX_LEN 64
#define PAYLOAD_MAX_LEN 256

static const char *TAG = "interface";
esp_mqtt_client_handle_t client;
esp_http_client_handle_t http_client;
char* mqtt_topic;
char* mqtt_payload;
const char WIFI_SSID[] = "Async-Lab";
const char WIFI_PASSWORD[] = "Asynchronous";
uint8_t device_mac[]={0xC4,0xD8,0xD5,0x0B,0x7F,0x42};//device的mac
uint8_t Pi_mac[] = {0xE0,0x98,0x06,0x88,0x13,0x9E};//树莓派的mac
char recvInfo[64];
enum WIFI_Status WiFi = WIFI_DISCONNECTED;

// 安全拷贝函数
void safe_strncpy(char* dest, const char* src, size_t src_len, size_t dest_max) {
    if (!dest || !src || dest_max == 0) return;
    size_t len = (src_len < dest_max - 1) ? src_len : dest_max - 1;
    if (len > 0) memcpy(dest, src, len);
    dest[len] = '\0';
}
/* Wi-Fi 事件处理 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi 断开，尝试重连...");
        WiFi = WIFI_DISCONNECTED;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi 连接成功，IP: " IPSTR, IP2STR(&event->ip_info.ip));
        WiFi = WIFI_CONNECTED;
    }
}

/* 初始化 Wi-Fi */
void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册 Wi-Fi 事件
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",//Asynclab
            .password = "",//"Asynchronous",
            .threshold={
                .authmode = WIFI_AUTH_WPA2_PSK,
            }
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());
}

static void Data_rec(const esp_now_recv_info_t *esp_now_info,const uint8_t *data,int data_len)
{
    ESP_LOGI(TAG,"%s",(char*)data);
    // if(strncmp((char*)esp_now_info->src_addr,device_mac,6)==0) {
    //     sprintf(recvInfo,"%s,",(char*));
    // }
    ESP_LOGI(TAG,"%s",(char*)esp_now_info->src_addr);
    lv_label_set_text(guider_ui.screen_msg_label_2, (char *)esp_now_info->src_addr);
    lv_label_set_text(guider_ui.screen_msg_label_4, (char*)data);
}
esp_err_t espnow_app_start(void) {
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW初始化失败: %d", err);
        return err;
    }
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    wifi_ap_record_t ap_info;
    esp_wifi_sta_get_ap_info(&ap_info);
    uint8_t router_channel = ap_info.primary;
    ESP_LOGI(TAG,"router_channel=%d", router_channel);

    // 配置目标1
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    peer.channel = router_channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    peer.channel = 6;
    memcpy(peer.peer_addr, device_mac, 6);
    // 添加第一个peer
    err = esp_now_add_peer(&peer);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加peer 1失败: %d", err);
        esp_now_deinit();
        return err;
    }

    // 配置目标1
    esp_now_peer_info_t peer2;
    memset(&peer, 0, sizeof(peer2));
    peer.channel = router_channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    peer.channel = 6;
    memcpy(peer.peer_addr, Pi_mac, 6);
    // 添加第一个peer
    err = esp_now_add_peer(&peer);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加peer 1失败: %d", err);
        esp_now_deinit();
        return err;
    }
    esp_now_register_recv_cb(Data_rec);
    ESP_LOGI(TAG,"ESPNOW_OK");
    return ESP_OK;
}
