#include <ESP8266WIFI.h>
#include "ESPNowW.h"


uint8_t Car_mac[]={0xC4,0xD8,0xD5,0x0B,0x7F,0x3F};//Car的mac
uint8_t Pi_mac[] = {0xE0,0x98,0x06,0x88,0x13,0x9E};//树莓派的mac
WiFiClientSecure net;

WiFiClient client;
int32_t test=0;
uint8_t msg[64]; 
int timex=0;

void ESPSend(uint8_t* mac,uint8_t* data){
  uint8_t len = strlen((char*)data) + 1; 
  ESPNow.send_message(mac, data, len);
}

void onRecv(unsigned char *mac_addr, unsigned char *data, uint8_t data_len) {
    Serial.printf("%s\n", data);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(2,OUTPUT);
  WiFi.disconnect();
  wifi_set_channel(6);
  ESPNow.init();
  ESPNow.add_peer(Pi_mac);
  ESPNow.add_peer(Car_mac);
  digitalWrite(2,LOW);
  ESPNow.reg_recv_cb(onRecv);

}

void loop() {
    static uint8_t buffer[256];    
  static size_t index = 0;     

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      buffer[index] = '\0';
      index = 0;
      ESPSend(Pi_mac,buffer);
      ESPSend(Car_mac,buffer);
    } else {
      buffer[index++] = c; 
    }
    
  }

}