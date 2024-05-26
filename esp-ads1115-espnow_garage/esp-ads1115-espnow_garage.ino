//https://docs.arduino.cc/tutorials/nano-esp32/esp-now/

#include <EasyButton.h>
int duration = 1000;
#define BUTTON_PIN 23
#define EXT_LED 22

int btn_to_send; 

#include <esp_now.h>
#include <WiFi.h>
int esp_now_mesh_id;
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


EasyButton button(BUTTON_PIN);


// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  //char a[32];
  int b;
  //float c;
  //bool d;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onPressedForDuration() {
    Serial.println("Button has been pressed for the given duration!");
    btn_to_send = 1;
}
void onPressed() {
    Serial.println("Button has been pressed!");
    btn_to_send = 2;
}

void led_sent_ok() {
    Serial.println("LED TRIGG SENT OK");
    digitalWrite(EXT_LED, HIGH);
    delay(150);
    digitalWrite(EXT_LED, LOW);
    delay(150);
    digitalWrite(EXT_LED, HIGH);
    delay(150);
    digitalWrite(EXT_LED, LOW);
    delay(150);
    digitalWrite(EXT_LED, HIGH);
    delay(300);
    digitalWrite(EXT_LED, LOW);
    
}


void init_ok() {
    Serial.println("LED TRIGG SENT OK");
    digitalWrite(EXT_LED, HIGH);
    delay(50);
    digitalWrite(EXT_LED, LOW);
    delay(50);
    digitalWrite(EXT_LED, HIGH);
    delay(50);
    digitalWrite(EXT_LED, LOW);
    delay(50);
    digitalWrite(EXT_LED, HIGH);
    delay(50);
    digitalWrite(EXT_LED, LOW);
    
}

void setup(void)
{
  Serial.begin(115200);
  btStop();  // make sure BT is disabled
  Serial.println("Disabling BT");
  setCpuFrequencyMhz(80);
  Serial.println("CPU Set to 80Mhz");

  pinMode(EXT_LED, OUTPUT);
  


  // Initialize the button.
  button.begin();
  // Attach callback.
  button.onPressedFor(duration, onPressedForDuration);
  // Attach callback.
  button.onPressed(onPressed);




   if (!initESPNow()) {
   debugln(" SYSTEM......... FAIL");
   debugln(" ... restarting");
   delay(60 * 1000);
   ESP.restart();

   }

}

bool initESPNow() {

  debugln(" ESPNOW............INIT");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    debugln(" ESPNOW.............FAIL");
    return false;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    debugln(" ESPNOW.............FAIL");
    return false;
  }
  debugln(" ESPNOW..............OK");

  esp_now_mesh_id = getMeshID();
  debug(" MESH ID..........");
  debugln(esp_now_mesh_id);
  init_ok();

  return true;
}

int getMeshID() {

  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  uint32_t uniqueID = 0;
  for (int i = 2; i < 6; i++) {
    uniqueID <<= 8;
    uniqueID |= baseMac[i];
  }
  // Limit the integer to the maximum value of an int (32,767)
  uniqueID %= 32768;
  return (int)uniqueID;
}


void loop(void)
{
 btn_to_send = 0;
 
 button.read();
 
 
 if (btn_to_send == 0) {
 

 } else if (btn_to_send == 1) {

  myData.b = btn_to_send;
 
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent 1 with success");
    led_sent_ok();

  }
  else {
    Serial.println("Error sending the data");
  }

 } else if (btn_to_send == 2) {

  myData.b = btn_to_send;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent 2 with success");
    led_sent_ok();

  }
  else {
    Serial.println("Error sending the data");
  }

 }

}
