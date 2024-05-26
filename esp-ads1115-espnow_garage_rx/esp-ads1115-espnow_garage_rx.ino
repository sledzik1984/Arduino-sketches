/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h> //For ESP32
#include <WiFi.h> //For ESP32

#define LED_PIN 10


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  //char a[32];
  int b;
  //float c;
  //bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print("Bytes received: ");
  //S/erial.println(len);
  //Serial.print("Char: ");
  //Serial.println(myData.a);

  //Serial.print("Float: ");
  //Serial.println(myData.c);
  //Serial.print("Bool: ");
  //Serial.println(myData.d);
  //Serial.println();

  if (myData.b == 0) {

  } else if (myData.b == 1) {
  
    digitalWrite (LED_PIN, HIGH);
    delay(100);
    Serial.println("LONG!");
    digitalWrite (LED_PIN, LOW);
    delay(100);
    digitalWrite (LED_PIN, HIGH);

  } else if (myData.b == 2) {

    digitalWrite (LED_PIN, HIGH);
    delay(100);
    Serial.println("SHORT!");
    digitalWrite (LED_PIN, LOW);
    delay(100);
    digitalWrite (LED_PIN, HIGH);
    
  }

}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(LED_PIN, OUTPUT);

}

void loop() {

  //Serial.println("LOOP");

}
