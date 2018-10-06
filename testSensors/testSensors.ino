/* 
  This sketch reads the the values from a bunch of different sensors and prints them to serial output.
  It currently supports SHT3* and SDP6* sensors.
  To enable/disable a sensor, uncomment/comment its name in the definitions section below.
  Author: Guescio.
*/

//******************************************
//definitions
#define SHT35A //0x44
//#define SHT35B //0x45
//#define SDP610
#define INTERVAL (2e3)//ms

//******************************************
#if defined(SHT35A) || defined(SHT35B)
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

  Serial.println();
  Serial.println("test sensors v. 0.0");

  //SHT35 A
  #ifdef SHT35A
  sht31a.begin(SHT35AADDR);
  Serial.print("t_a[C] RH_b[%] ");
  #endif

  //SHT35 B
  #ifdef SHT35B
  sht31b.begin(SHT35BADDR);
  Serial.print("t_b[C] RH_b[%] ");
  #endif

  //SDP610
  #ifdef SDP610
  Wire.begin();
  Wire.setClockStretchLimit(1e4);//µs
  Serial.print("dP[Pa] ");
  #endif
  
  Serial.println();
}

//******************************************
void loop(){
  
  //SHT35 A
  #ifdef SHT35A
  Serial.print(sht31a.readTemperature());//temperature
  Serial.print("  ");
  Serial.print(sht31a.readHumidity());//humidity
  Serial.print("   ");
  #endif

  //SHT35 B
  #ifdef SHT35B
  Serial.print(sht31b.readTemperature());//temperature
  Serial.print("  ");
  Serial.print(sht31b.readHumidity());//humidity
  Serial.print("   ");
  #endif

  //SDP610
  #ifdef SDP610
  Serial.print(SDP6x.GetPressureDiff());//differential pressure
  Serial.print("   ");
  #endif

  Serial.println();
  
  delay(INTERVAL);
}


