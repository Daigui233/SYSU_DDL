
#ifndef INTERFACE_H
#define INTERFACE_H
#ifdef __cplusplus
extern "C"{
#endif
#include <mqtt_client.h>
#include <esp_http_client.h>

    extern esp_mqtt_client_handle_t client;
    extern char* mqtt_server;
    extern esp_http_client_handle_t http_client;
    extern uint8_t device_mac[];
    extern uint8_t Pi_mac[];
enum WIFI_Status
    { WIFI_DISCONNECTED = 0,
      WIFI_CONNECTED };

    void wifi_init(void);
    esp_err_t espnow_app_start(void);

#ifdef __cplusplus
}
#endif
#endif //INTERFACE_H
