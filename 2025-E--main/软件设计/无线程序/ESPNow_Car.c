#include <ESP8266WIFI.h>
#include <Arduino.h>
#include "ESPNowW.h"
uint8_t Car_mac[]={0xC4,0xD8,0xD5,0x0B,0x7F,0x42};//Car的mac
uint8_t Pi_mac[] = {0xE0,0x98,0x06,0x88,0x13,0x9E};//树莓派的mac
uint8_t computer_mac[]={0xC4,0xD8,0xD5,0x0B,0x7D,0x4D};//computer的mac
uint8_t LVGL_mac[]={0x7C,0xDF,0xA1,0xE6,0xFB,0x18};
WiFiClientSecure net;

int32_t tes=0;
uint8_t msg[249]; 
int test;
void ESPSend(uint8_t* mac,uint8_t* data){
  uint8_t len = strlen((char*)data) + 1;
  ESPNow.send_message(mac, data, len);
}

void onRecv(unsigned char *mac_addr, unsigned char *data, uint8_t data_len) {
  if (data[data_len - 1] == 0)
      Serial.printf("%s\n", data);

}
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(2,OUTPUT);
   wifi_set_channel(6);
  ESPNow.init();
ESPNow.add_peer(Pi_mac);
ESPNow.add_peer(LVGL_mac);
ESPNow.add_peer(computer_mac);
  // 注册串口接收回调函数
  digitalWrite(2,LOW);
  ESPNow.reg_recv_cb(onRecv);
}


void loop() {
  static uint8_t buffer[256];    
  static size_t index = 0;     
  static unsigned long lastReceiveTime = millis();  // 记录最后一次接收时间

  // 检查串口数据
  while (Serial.available() > 0) {
    char c = Serial.read();
    lastReceiveTime = millis();  // 更新最后一次接收时间

    if (c == '\n') {
      buffer[index] = '\0';
      index = 0;
      digitalWrite(2, LOW);
      ESPSend(computer_mac, buffer);
    } else {
      buffer[index++] = c; 
    }
  }

  // 超时检测（例如 100ms 未收到数据）
  if (millis() - lastReceiveTime > 100) {
    digitalWrite(2, HIGH);  
  }
}