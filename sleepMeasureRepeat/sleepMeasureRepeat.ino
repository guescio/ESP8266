/* 
  This sketch reads the ambient temperature, humidity and differential pressure, prints them to serial output and then goes to sleep.
  Temperature and humidity can be read from two different sensors.
  Don't forget to connect pin 16 with the reset pin.
  To enable/disable a sensor, uncomment/comment its name in the definitions section below.
  Author: Guescio.
*/

//******************************************
//definitions
#define SHT35A //0x44 address
#define SHT35B //0x45 address
#define SDP610
#define SLEEPTIME (30)//s

//******************************************
#ifdef SHT35A || SHT35B
#include "Adafruit_SHT31.h"
#endif
#ifdef SDP610
#include <Wire.h>
#include "SDP6x.h"
#endif

//******************************************
//SHT35 A
#ifdef SHT35A
#define SHT35AADDR (0x44)
Adafruit_SHT31 sht31a = Adafruit_SHT31();
#endif

//SHT35 B
#ifdef SHT35B
#define SHT35BADDR (0x45)
Adafruit_SHT31 sht31b = Adafruit_SHT31();
#endif

//SDP610 does not need initialization

//******************************************
void setup(){
  Serial.begin(115200);
  while(!Serial){}

  //initialize SHT35s
  #ifdef SHT35A
  sht31a.begin(SHT35AADDR);
  #endif
  #ifdef SHT35B
  sht31b.begin(SHT35BADDR);
  #endif

  //initialize SDP610
  #ifdef SDP610
  Wire.begin();
  Wire.setClockStretchLimit(1e4);//µs
  #endif

  //print info
  Serial.println();
  /*
  Serial.println("sleep, measure, repeat");
  #ifdef SHT35A
  Serial.print(" t_a[C] RH_a[%]");
  #endif
  #ifdef SHT35A
  Serial.print(" t_b[C] RH_b[%]");
  #endif
  #ifdef SDP610
  Serial.print(" dP[Pa]");
  #endif
  Serial.println();
  */
  
  //print measurements to serial output
  #ifdef SHT35A
  Serial.print(sht31a.readTemperature());//temperature
  Serial.print(" ");
  Serial.print(sht31a.readHumidity());//humidity
  Serial.print(" ");
  #endif

  #ifdef SHT35B
  Serial.print(sht31b.readTemperature());//temperature
  Serial.print(" ");
  Serial.print(sht31b.readHumidity());//humidity
  Serial.print(" ");
  #endif

  #ifdef SDP610
  Serial.print(SDP6x.GetPressureDiff());//differential pressure
  #endif

  Serial.println();
  
  //deep sleep
  ESP.deepSleep(SLEEPTIME*1e6);//µs
}

//******************************************
//loop() is empty since the ESP8266 is sent to deep sleep at the end of setup()
void loop(){}

