#include <OneWire.h>
#include <string.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "cma"; // Enter your WiFi name
const char *password = "@@cmacma@@";  // Enter WiFi password


//Dioda w Czytniku
#define LED 12

// Onewire lib is here http://www.pjrc.com/teensy/td_libs_OneWire.html

OneWire  ds(0);  // on pin 8 here, don't forget 4.7K resistor between +5V and DQ pin

//byte PiotrekG[]={0x01,0x5B,0xBD,0x42,0x01,0x00,0x00,0x04};
byte PiotrekG[]={0x01,0x5B,0xBD,0x42,0x01,0x00,0x00,0x04};



void setup(void) {
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }


}

void loop(void) {
  //Zapal diodę na początku pętli
  client.publish(topic, "Znaleziono klucz PiotrekG");

  digitalWrite(LED, HIGH);

  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    Serial.println("--------------");
    Serial.println(".");
    Serial.println(".");
    Serial.println("END OF SCAN.");


    Serial.println(".");
    Serial.println();

    ds.reset_search();
    delay(1000);
    return;
  }

  Serial.println("--------------");



  Serial.print(" ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    if ( addr[i]<16) {
      Serial.print("0");
    }
    Serial.print(addr[i], HEX);




  }

  
      //https://forum.arduino.cc/t/how-to-compare-arrays/76556/7

if ( memcmp( (const void *)addr, (const void *)PiotrekG, sizeof(addr)) == 0)
{
    Serial.println("Znaleziono klucz PiotrekG");
    //Znaleziono klucz - zaświeć diodę
    digitalWrite(LED, LOW);
    delay(2000); 
}



  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  
  Serial.println();


  Serial.print("CHIP FAMILY ");
  Serial.print(addr[0],HEX);
  // the first ROM byte indicates which chip
  Serial.print(" =  ");
  switch (addr[0]) {

  case 0x01:
    Serial.println(" DS1990 DS2401");  // 
    break;
  case 0x02:
    Serial.println(" DS1991");  // 
    break;
  case 0x04:
    Serial.println(" DS1994");  // 
    break;
  case 0x05:
    Serial.println(" DS2405");  // 
    break;
  case 0x06:
    Serial.println(" DS1992");  // 
    break;
  case 0x08:
    Serial.println(" DS1993");  // 
    break;
  case 0x0B:
    Serial.println(" DS1985");  
    break;
  case 0x10:
    Serial.println(" DS1820 DS18S20 DS1920");  
    break;
  case 0x12:
    Serial.println(" DS2406");  
    break;
  case 0x21:
    Serial.println(" DS1921");
    break;
  case 0x22:
    Serial.println(" DS1822");
    break;
  case 0x24:
    Serial.println(" DS1904");
    break;
  case 0x28:
    Serial.println(" DS18B20");
    break;
  case 0x29:
    Serial.println(" DS2408");  
    break;
  case 0x36:
    Serial.println(" DS2740");  
    break;
  case 0x3B:
    Serial.println(" DS1825");  
    break;
  case 0x41:
    Serial.println(" DS1923");  
    break;

  default:
    Serial.println(" is not listed.");

    return;
  } 

}
