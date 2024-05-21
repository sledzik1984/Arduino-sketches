#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Adafruit_ADS1015 ads;     /* Use this for the 12-bit version */

#define SDA_PIN 21 
#define SCL_PIN 22

unsigned long inStateAtMs = millis() ;
int buttonState ;  // 0 = waiting for press, 1= waiting for release 2= differentiate single or double 3= wait button release (double)




void setup(void)
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  ads.begin();
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

 
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  //ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}
 
void loop(void)
{
  int16_t adc0;
  float volts0;
 
  adc0 = ads.readADC_SingleEnded(0);
  
  volts0 = ads.computeVolts(adc0);
  
  //Serial.println("-----------------------------------------------------------");
  //Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  
  //if ( volts0 > 3) {



 if ( buttonState == 0 ) {
   // wait for button press
   //Serial.println("Sleep");
   if ( millis() - inStateAtMs > 300 ) {
     if ( volts0 > 3) {
       buttonState = 1 ;
       Serial.println("buttonState = 1 PUSHED ONCE!");
       inStateAtMs = millis();
     }
   }
 }
 else if ( buttonState == 1 ) {
   // wait for stable button release 1
   if ( millis() - inStateAtMs > 300 && volts0 < 3 ) {
     buttonState = 2 ;
     
     inStateAtMs = millis();
   }
 }
 else if ( buttonState == 2 ) {
   // differentiate between single and double press
   if ( millis() - inStateAtMs > 1000 ) {
     // timeout - is a single press
     Serial.println("single");
     buttonState = 0 ;
     inStateAtMs = millis() ;
   }
   else if (  ( volts0 > 3 ) && (millis() - inStateAtMs > 300 ) ) {
     // got second press within timeout - double
     Serial.println("double");
     buttonState = 3 ;
     inStateAtMs = millis() ;
   }
 }
 else if ( buttonState == 3 ) {
   // wait for stable button release 2
   if ( millis() - inStateAtMs > 300 && volts0 < 3 ) {
     buttonState = 0 ;
     Serial.println("Btn Released");
     inStateAtMs = millis();
   }
 }




}
