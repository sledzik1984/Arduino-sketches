#include <ESP8266WiFi.h>
#include <espnow.h>
ADC_MODE(ADC_VCC);

// Replace with receiver MAC address
uint8_t receiverMAC[] = {0x2D, 0xA4, 0x31, 0x2C, 0x6B, 0x47}; //Odbiornik DOM
uint8_t receiver2MAC[] = {0x2D, 0xA4, 0x31, 0x2C, 0x6B, 0x47}; //Odbiornik Magazyn

typedef struct dataStructure {
  int n;
  float voltage;
} dataStructure;
dataStructure myData;

void setup() {
  float vcc = ESP.getVcc() / 1000.0 + 0.7;
  myData.voltage=vcc;

  WiFi.mode(WIFI_STA);
  pinMode(3,INPUT_PULLUP);

  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(receiver2MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  if(digitalRead(3)==HIGH){ //press left button
    myData.n=1;
    esp_now_send(receiverMAC, (uint8_t *) &myData, sizeof(myData));
  }
  else{ //hold right button and press the left one
    myData.n=0;
    esp_now_send(receiverMAC, (uint8_t *) &myData, sizeof(myData));
  }
}

void loop() {
  //hold actions
  /*
  delay(500);
  myData.n+=10;
  esp_now_send(receiverMAC, (uint8_t *) &myData, sizeof(myData));
  */
}