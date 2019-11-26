/*
  This sketch reads the ambient temperature, humidity, differential pressure and particle count,
  prints them to serial output, posts them online and then goes to sleep (unless caffeine is taken).
  Both ESP8266 and ESP32 boards are supported.
  The values measured are posted through MQTT.
  Temperature and humidity can be read from SHT35 and SHT85 sensors.
  Temperature can be read from up to 5 NTCs.
  The dust particle count is measured by a SPS30 sensor.
  On ESP8266 boards, don't forget to connect pin 16 to the reset pin to wake up from deep sleep (again, unless caffeine is taken).
  Printing and posting can be enabled/disabled by uncommenting/commenting the relative options in config.h.
  To enable/disable a sensor, uncomment/comment its name in config.h.
  Author: Guescio.
*/

//******************************************
//libraries
#include "config.h"//NOTE: this is where alle the parameters are set, edit this
#include "limits.h"
#include "ArduinoJson.h"
#include "guescio.h"

#if defined(SHTA) || defined(SHTB)
#include "Adafruit_SHT31.h"
#endif

#ifdef SPS30
#include "sps30.h"
#endif

#ifdef SDP610
#include "Wire.h"
#include "SDP6x.h"
#endif

#ifdef POST
#ifdef ESP32
#include "WiFi.h" //ESP32
#else
#include "ESP8266WiFi.h" //ESP8266
#endif
#include "PubSubClient.h"
#endif

//******************************************
//initilization
//SHT A
#ifdef SHTA
#define SHTAADDR (0x44)
Adafruit_SHT31 sht31a = Adafruit_SHT31();
#endif

//SHT B
#ifdef SHTB
#define SHTBADDR (0x45)
Adafruit_SHT31 sht31b = Adafruit_SHT31();
#endif

//SPS30
#ifdef SPS30
s16 ret;
u8 auto_clean_days = 2;
u32 auto_clean;
bool sps30available = false;
char serial[SPS_MAX_SERIAL_LEN];
#endif //SPS30

//******************************************
//wifi and MQTT setup
#ifdef POST
//wifi
const char* ssid     = WIFISSID;
const char* password = WIFIPASSWORD;
//initialize wifi client library
WiFiClient WiFiclient;
#endif //POST

#if defined(POST) || defined(VERBOSE)
//MQTT
char MQTTServer[] = MQTTSERVER;
long MQTTPort = 1883;
char MQTTUsername[] = MQTTUSER;
char MQTTPassword[] = MQTTPASSWORD;
//NOTE: need a random client ID for posting
static const char alphanum[] = "0123456789"
                               "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                               "abcdefghijklmnopqrstuvwxyz";
#endif //POST || VERBOSE
                               
#ifdef POST 
//initialize PuBSubClient library
PubSubClient MQTTClient(WiFiclient);
#endif //POST

//******************************************
//measurements
float ta = std::numeric_limits<double>::quiet_NaN();//temperature A [C]
float rha = std::numeric_limits<double>::quiet_NaN();//relative humidity A [%]
float dpa = std::numeric_limits<double>::quiet_NaN();//dew point A [C]
float tb = std::numeric_limits<double>::quiet_NaN();//temperature B [C]
float rhb = std::numeric_limits<double>::quiet_NaN();//relative humidity B [%]
float dpb = std::numeric_limits<double>::quiet_NaN();//dew point B [C]

float dp = std::numeric_limits<double>::quiet_NaN();//differential pressure [Pa]

int adc0 = std::numeric_limits<double>::quiet_NaN();//ADC0 []
int adc1 = std::numeric_limits<double>::quiet_NaN();//ADC1 []
int adc2 = std::numeric_limits<double>::quiet_NaN();//ADC2 []
int adc3 = std::numeric_limits<double>::quiet_NaN();//ADC3 []
int adc4 = std::numeric_limits<double>::quiet_NaN();//ADC4 []

float tntc0 = std::numeric_limits<double>::quiet_NaN();//temperature NTC0 [C]
float tntc1 = std::numeric_limits<double>::quiet_NaN();//temperature NTC1 [C]
float tntc2 = std::numeric_limits<double>::quiet_NaN();//temperature NTC2 [C]
float tntc3 = std::numeric_limits<double>::quiet_NaN();//temperature NTC3 [C]
float tntc4 = std::numeric_limits<double>::quiet_NaN();//temperature NTC4 [C]

float dustnc0p5 = std::numeric_limits<double>::quiet_NaN();//>0.5 um particles concentration [cm^-3]
float dustnc1p0 = std::numeric_limits<double>::quiet_NaN();//>1.0 um particles concentration [cm^-3]
float dustnc2p5 = std::numeric_limits<double>::quiet_NaN();//>2.5 um particles concentration [cm^-3]
float dustnc4p0 = std::numeric_limits<double>::quiet_NaN();//>4.0 um particles concentration [cm^-3]
float nc0p5 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <0.5 um particles concentration [cm^-3]
float nc1p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <1.0 um particles concentration [cm^-3]
float nc2p5 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <2.5 um particles concentration [cm^-3]
float nc4p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <4.0 um particles concentration [cm^-3]
float nc10p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <10.0 um particles concentration [cm^-3]
float dustmc1p0 = std::numeric_limits<double>::quiet_NaN();//>1.0 um particles mass concentration [µg/m^-3]
float dustmc2p5 = std::numeric_limits<double>::quiet_NaN();//>2.5 um particles mass concentration [µg/m^-3]
float dustmc4p0 = std::numeric_limits<double>::quiet_NaN();//>4.0 um particles mass concentration [µg/m^-3]
float mc1p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <1.0 um particles mass concentration [µg/m^-3]
float mc2p5 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <2.5 um particles mass concentration [µg/m^-3]
float mc4p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <4.0 um particles mass concentration [µg/m^-3]
float mc10p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <10.0 um particles mass concentration [µg/m^-3]
float dustsize = std::numeric_limits<double>::quiet_NaN();//average dust particle size [µm]

//******************************************
//setup
void setup() {

  //------------------------------------------
  Serial.begin(115200);
  while (!Serial) {}

  //------------------------------------------
  #if defined(PRINTSERIAL) || defined(VERBOSE)
  Serial.println("\nsleep, measure, post, repeat");
  #ifdef CAFFEINE
  Serial.println("caffeine!");
  #endif //CAFFEINE
  #endif //PRINTSERIAL or VERBOSE

  //------------------------------------------
  //initialize SHT sensors
  #ifdef SHTA
  sht31a.begin(SHTAADDR);
  #endif

  #ifdef SHTB
  sht31b.begin(SHTBADDR);
  #endif

  //------------------------------------------
  //initialize SPS30
  #ifdef SPS30

  #ifdef VERBOSE
  Serial.println("\ninitialising SPS30");
  #endif

  //probe SPS30
  for (int ii = 0; ii < 10; ii++) {
    if (sps30_probe() != 0) {
      #ifdef VERBOSE
      Serial.println("SPS30 sensor probing failed");
      #endif //VERBOSE
    } else {
      sps30available = true;
      break;
    }
    //wait 500 ms between trials
    delay(500);//ms
  }

  //SPS30 serial number
  #ifdef VERBOSE
  if (sps30available) {
    ret = sps30_get_serial(serial);
    if (ret) {
      Serial.println("could not retrieve SPS30 serial number");
    } else {
      Serial.print("SPS30 serial number: ");
      Serial.println(serial);
    }
  }
  #endif //VERBOSE
  
  //set auto-cleaning
  if (sps30available) {
    ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
    if (ret) {
      #ifdef VERBOSE
      Serial.print("SPS30 error setting the auto-clean interval: ");
      Serial.println(ret);
      #endif //VERBOSE
    }
  }

  //start measurement
  if (sps30available) {
    ret = sps30_start_measurement();
    if (ret < 0) {
      #ifdef VERBOSE
      Serial.print("SPS30 error starting measurement\n");
      #endif //verbose
      sps30available = false;
    }
  }

  #ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
  Serial.println("The hardware has a limitation that only");
  Serial.println("  allows reading the mass concentrations.");
  Serial.println("  For more information, please check:");
  Serial.println("  https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support\n");
  #endif //SPS30_LIMITED_I2C_BUFFER_SIZE
  #endif //SPS30

  //------------------------------------------
  //initialize SDP610
  #ifdef SDP610
  Wire.begin();
  Wire.setClockStretchLimit(1e4);//µs
  #endif

  //------------------------------------------
  //if needed, connect to wifi already
  #if defined(CAFFEINE) && defined(POST)
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnect();
  }
  #endif //CAFFEINE and POST

  //------------------------------------------
  #ifndef CAFFEINE
  //read, print and post values
  readPrintPost();
  
  //deep sleep
  ESP.deepSleep(SLEEPTIME * 1e6); //µs
  #endif //CAFFEINE
}

//******************************************
void loop() {

  //read, print and post values
  readPrintPost();
  
  //delay but do not sleep
  delay(SLEEPTIME * 1e3); //ms
}

//******************************************
//read, print and post values
void readPrintPost() {

  //read values
  readValues();

  //print values to serial
  #ifdef PRINTSERIAL
  printValues();
  #endif //PRINTSERIAL

  //post values
  #if defined(POST) || defined(VERBOSE)
  postValues();
  #endif //POST or VERBOSE
  
  return;
}

//******************************************
//read values from enabled sensors
void readValues() {

  //------------------------------------------
  //set values to NaN
  ta = std::numeric_limits<double>::quiet_NaN();//temperature A [C]
  rha = std::numeric_limits<double>::quiet_NaN();//relative humidity A [%]
  dpa = std::numeric_limits<double>::quiet_NaN();//dew point A [C]
  tb = std::numeric_limits<double>::quiet_NaN();//temperature B [C]
  rhb = std::numeric_limits<double>::quiet_NaN();//relative humidity B [%]
  dpb = std::numeric_limits<double>::quiet_NaN();//dew point B [C]

  dp = std::numeric_limits<double>::quiet_NaN();//differential pressure [Pa]
  
  adc0 = std::numeric_limits<double>::quiet_NaN();//ADC0 []
  adc1 = std::numeric_limits<double>::quiet_NaN();//ADC1 []
  adc2 = std::numeric_limits<double>::quiet_NaN();//ADC2 []
  adc3 = std::numeric_limits<double>::quiet_NaN();//ADC3 []
  adc4 = std::numeric_limits<double>::quiet_NaN();//ADC4 []
  
  tntc0 = std::numeric_limits<double>::quiet_NaN();//temperature NTC0 [C]
  tntc1 = std::numeric_limits<double>::quiet_NaN();//temperature NTC1 [C]
  tntc2 = std::numeric_limits<double>::quiet_NaN();//temperature NTC2 [C]
  tntc3 = std::numeric_limits<double>::quiet_NaN();//temperature NTC3 [C]
  tntc4 = std::numeric_limits<double>::quiet_NaN();//temperature NTC4 [C]
  
  dustnc0p5 = std::numeric_limits<double>::quiet_NaN();//>0.5 um particles concentration [cm^-3]
  dustnc1p0 = std::numeric_limits<double>::quiet_NaN();//>1.0 um particles concentration [cm^-3]
  dustnc2p5 = std::numeric_limits<double>::quiet_NaN();//>2.5 um particles concentration [cm^-3]
  dustnc4p0 = std::numeric_limits<double>::quiet_NaN();//>4.0 um particles concentration [cm^-3]
  nc0p5 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <0.5 µm particles concentration [cm^-3]
  nc1p0 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <1.0 µm particles concentration [cm^-3]
  nc2p5 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <2.5 µm particles concentration [cm^-3]
  nc4p0 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <4.0 µm particles concentration [cm^-3]
  nc10p0 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <10.0 µm particles concentration [cm^-3]
  dustmc1p0 = std::numeric_limits<double>::quiet_NaN();//>1.0 µm particles mass concentration [µg/m^-3]
  dustmc2p5 = std::numeric_limits<double>::quiet_NaN();//>2.5 µm particles mass concentration [µg/m^-3]
  dustmc4p0 = std::numeric_limits<double>::quiet_NaN();//>4.0 µm particles mass concentration [µg/m^-3]
  mc1p0 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <1.0 µm particles mass concentration [µg/m^-3]
  mc2p5 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <2.5 µm particles mass concentration [µg/m^-3]
  mc4p0 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <4.0 µm particles mass concentration [µg/m^-3]
  mc10p0 = std::numeric_limits<double>::quiet_NaN();//0.3µm< size <10.0 µm particles mass concentration [µg/m^-3]
  dustsize = std::numeric_limits<double>::quiet_NaN();//typical dust particle size

  //------------------------------------------
  #ifdef SHTA
  ta = sht31a.readTemperature();
  rha = sht31a.readHumidity();
  #endif //SHTA

  #ifdef SHTB
  tb = sht31b.readTemperature();
  rhb = sht31b.readHumidity();
  #endif //SHTB

  //------------------------------------------
  #ifdef SPS30
  if (sps30available) {

    struct sps30_measurement m;
    u16 data_ready;

    //try to read data from SPS30
    for (int ii = 0; ii < 10; ii++) {
      ret = sps30_read_data_ready(&data_ready);
      if (ret < 0) {
        #ifdef VERBOSE
        Serial.print("SPS30 error reading data-ready flag: ");
        Serial.println(ret);
        #endif //VERBOSE
      } else if (!data_ready) {
        #ifdef VERBOSE
        Serial.println("SPS30 data not ready, no new measurement available");
        #endif //VERBOSE
      } else {
        break;
      }
      //wait 100 ms between trials
      delay(100);//ms
    };

    //read SPS30 measurement
    ret = sps30_read_measurement(&m);
    if (ret < 0) {
      #ifdef VERBOSE
      Serial.println("SPS30 error reading measurement");
      #endif //VERBOSE
    } else {
      dustnc0p5 = (m.nc_10p0 - m.nc_0p5); //particles/cm^3
      dustnc1p0 = (m.nc_10p0 - m.nc_1p0); //particles/cm^3
      dustnc2p5 = (m.nc_10p0 - m.nc_2p5); //particles/cm^3
      dustnc4p0 = (m.nc_10p0 - m.nc_4p0); //particles/cm^3
      nc0p5 = m.nc_0p5; //particles/cm^3
      nc1p0 = m.nc_1p0; //particles/cm^3
      nc2p5 = m.nc_2p5; //particles/cm^3
      nc4p0 = m.nc_4p0; //particles/cm^3
      nc10p0 = m.nc_10p0; //particles/cm^3
      dustmc1p0 = (m.mc_10p0 - m.mc_1p0); //µg/m^3
      dustmc2p5 = (m.mc_10p0 - m.mc_2p5); //µg/m^3
      dustmc4p0 = (m.mc_10p0 - m.mc_4p0); //µg/m^3
      mc1p0 = m.mc_1p0; //µg/m^3
      mc2p5 = m.mc_2p5; //µg/m^3
      mc4p0 = m.mc_4p0; //µg/m^3
      mc10p0 = m.mc_10p0; //µg/m^3
      dustsize = m.typical_particle_size; //µm
      
      //Serial.print("particle concentration (>0.5 µm and <10 µm): ");
      //Serial.print(dust0p5);
      //Serial.println(" cm^-3");
      //Serial.print("particle concentration (>0.5 µm and <10 µm): ");
      //Serial.print(dust0p5*pow(30.48, 3));
      //Serial.println(" ft^-3");
      //Serial.print("typical partical size: ");
      //Serial.print(m.typical_particle_size);
      //Serial.println(" µm");
    }

  }//sps30available
  #endif //SPS30

  //------------------------------------------
  #ifdef SDP610
  dp = SDP6x.GetPressureDiff();
  #endif //SDP610

  //------------------------------------------
  #if defined(ADC0) or defined (NTC0)
  #ifdef ESP32
  adc0 = getADC(A2); //ESP32 A2/34
  #else
  adc0 = getADC(A0); //ESP8266 A0/ADC
  #endif
  #endif //ADC0 || NTC0

  #ifdef ESP32
  
  #if defined(ADC1) or defined (NTC1)
  adc1 = getADC(A3); //ESP32 A3/39
  #endif //ADC1 || NTC1

  #if defined(ADC2) or defined (NTC2)
  adc2 = getADC(A4); //ESP32 A4/36
  #endif //ADC2 || NTC2

  #if defined(ADC3) or defined (NTC3)
  adc3 = getADC(A7); //ESP32 A7/32
  #endif //ADC3 || NTC3

  #if defined(ADC4) or defined (NTC4)
  adc4 = getADC(A9); //ESP32 A9/33
  #endif //ADC4 || NTC4

  #endif //ESP32

  //------------------------------------------  
  #ifdef NTC0
  tntc0 = getTNTC(adc0, ADC0RDIV, NTC0T0, NTC0B, NTC0R0);
  #endif //NTC0

  #ifdef ESP32
  
  #ifdef NTC1
  tntc1 = getTNTC(adc1, ADC1RDIV, NTC1T0, NTC1B, NTC1R0);
  #endif //NTC1

  #ifdef NTC2
  tntc2 = getTNTC(adc2, ADC2RDIV, NTC2T0, NTC2B, NTC2R0);
  #endif //NTC2

  #ifdef NTC3
  tntc3 = getTNTC(adc3, ADC3RDIV, NTC3T0, NTC3B, NTC3R0);
  #endif //NTC3

  #ifdef NTC4
  tntc4 = getTNTC(adc4, ADC4RDIV, NTC4T0, NTC4B, NTC4R0);
  #endif //NTC4

  #endif //ESP32

  return;
}

//******************************************
//get dew point
float getDewPoint(float t, float rh) {
  if (! isnan(t) and ! isnan(rh)) {
    float h = (log10(rh) - 2) / 0.4343 + (17.62 * t) / (243.12 + t);
    return 243.12 * h / (17.62 - h);
  }
  else return std::numeric_limits<double>::quiet_NaN();
}

//******************************************
//get ADC value
float getADC(int pin) {
  #ifdef ESP32
  analogReadResolution(NBITS);
  analogSetWidth(NBITS);
  analogSetAttenuation(ADC_0db);
  #endif //ESP32
  int value = 0;
  for (int ii = 0; ii < NADCREADINGS; ii++) {
    value += analogRead(pin);
    delay(10);//ms
  }
  if (! isnan(value)) {
    return value / NADCREADINGS;
  }
  else return std::numeric_limits<double>::quiet_NaN();
}

//******************************************
//get NTC temperature
float getTNTC(int adc, float adcrdiv, float ntct0, float ntcb, float ntcr0) {
  if (! isnan(adc)) {
    float rntc = adcrdiv / ((pow(2, NBITS) - 1.) / adc * VDD / ADCVREF - 1.); //Ohm
    return (1. / ((1. / ntct0) + (1. / ntcb) * log((rntc / ntcr0))) - 273.15); //C
  }
  else return std::numeric_limits<double>::quiet_NaN();
}

//******************************************
//print values to serial output
void printValues() {

  Serial.println();

  #ifdef SHTA
  Serial.print("t_a[C] RH_a[%] DP_a[C] ");
  #endif
  #ifdef SHTB
  Serial.print("t_b[C] RH_b[%] DP_b[C] ");
  #endif

  #ifdef SPS30
  Serial.print("dust[cm^-3] NC0.5[cm^-3] NC1.0[cm^-3] NC2.5[cm^-3] NC4.0[cm^-3] NC10.0[cm^-3] ");
  Serial.print("dust[µg/m^-3] MC0.5[µg/m^-3] MC1.0[µg/m^-3] MC2.5[µg/m^-3] MC4.0[µg/m^-3] MC10.0[µg/m^-3] dust size[µm] ");
  #endif

  #ifdef SDP610
  Serial.print("dP[Pa] ");
  #endif

  #ifdef ADC0
  Serial.print("ADC0 ");
  #endif
  #ifdef ADC1
  Serial.print("ADC1 ");
  #endif
  #ifdef ADC2
  Serial.print("ADC2 ");
  #endif
  #ifdef ADC3
  Serial.print("ADC3 ");
  #endif
  #ifdef ADC4
  Serial.print("ADC4 ");
  #endif

  #ifdef NTC0
  Serial.print("t_NTC0[C] ");
  #endif
  #ifdef NTC1
  Serial.print("t_NTC1[C] ");
  #endif
  #ifdef NTC2
  Serial.print("t_NTC2[C] ");
  #endif
  #ifdef NTC3
  Serial.print("t_NTC3[C] ");
  #endif
  #ifdef NTC4
  Serial.print("t_NTC4[C] ");
  #endif
  
  Serial.println();

  //print measurements to serial output
  #ifdef SHTA
  Serial.print(ta);//temperature
  Serial.print("  ");
  Serial.print(rha);//humidity
  Serial.print("   ");
  Serial.print(getDewPoint(ta, rha)); //dew point
  Serial.print("   ");
  #endif

  #ifdef SHTB
  Serial.print(tb);//temperature
  Serial.print("  ");
  Serial.print(rhb);//humidity
  Serial.print("   ");
  Serial.print(getDewPoint(tb, rhb)); //dew point
  Serial.print("   ");
  #endif

  #ifdef SPS30
  Serial.print(dustnc0p5,3);//particle concentration (>0.5 um)
  Serial.print("       ");
  Serial.print(nc0p5,3);//particle concentration (<0.5 um)
  Serial.print("        ");
  Serial.print(nc1p0,3);//particle concentration (<1.0 um)
  Serial.print("        ");
  Serial.print(nc2p5,3);//particle concentration (<2.5 um)
  Serial.print("        ");
  Serial.print(nc4p0,3);//particle concentration (<4.0 um)
  Serial.print("        ");
  Serial.print(nc10p0,3);//particle concentration (<10.0 um)
  Serial.print("         ");
  Serial.print(dustmc1p0,3);//particle concentration (>0.5 um µg/m^3)
  Serial.print("       ");
  Serial.print(mc1p0,3);//particle concentration (<1.0 um µg/m^3)
  Serial.print("        ");
  Serial.print(mc2p5,3);//particle concentration (<2.5 um µg/m^3)
  Serial.print("        ");
  Serial.print(mc4p0,3);//particle concentration (<4.0 um µg/m^3)
  Serial.print("        ");
  Serial.print(mc10p0,3);//particle concentration (<10.0 um µg/m^3)
  Serial.print("         ");
  Serial.print(dustsize,3);//typical dust particle size (µm)
  Serial.print("         ");
  #endif

  #ifdef SDP610
  Serial.print(dp);//differential pressure
  Serial.print("  ");
  #endif

  #ifdef ADC0
  Serial.print(adc0);//ADC0
  Serial.print("  ");
  #endif
  #ifdef ADC1
  Serial.print(adc1);//ADC1
  Serial.print("  ");
  #endif
  #ifdef ADC2
  Serial.print(adc2);//ADC2
  Serial.print("  ");
  #endif
  #ifdef ADC3
  Serial.print(adc3);//ADC3
  Serial.print("  ");
  #endif
  #ifdef ADC4
  Serial.print(adc4);//ADC4
  Serial.print("  ");
  #endif
  
  #ifdef NTC0
  Serial.print(tntc0);//NTC0
  Serial.print("  ");
  #endif
  #ifdef NTC1
  Serial.print(tntc1);//NTC1
  Serial.print("  ");
  #endif
  #ifdef NTC2
  Serial.print(tntc2);//NTC2
  Serial.print("  ");
  #endif
  #ifdef NTC3
  Serial.print(tntc3);//NTC3
  Serial.print("  ");
  #endif
  #ifdef NTC4
  Serial.print(tntc4);//NTC4
  Serial.print("  ");
  #endif

  Serial.println();
}

//******************************************
//post values from enbled sensors
void postValues() {

  //------------------------------------------
  //MQTT message
  StaticJsonDocument<2056> doc;
  getMQTTMessage(doc);

  //convert message JSON to char array
  char message[measureJson(doc)+1];
  serializeJson(doc, message, measureJson(doc)+1);

  //print message
  #ifdef VERBOSE
  Serial.println();
  Serial.println("JSON: ");
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.print("message: ");
  Serial.println(message);
  Serial.print("message length: ");
  Serial.println(measureJson(doc));
  #endif //VERBOSE

  //------------------------------------------
  //MQTT topic
  String topicString = MQTTTOPIC;

  //append institute
  #ifdef INSTITUTE
  topicString += "/";
  topicString += INSTITUTE;
  #endif //INSTITUTE

  //convert to char
  char topic[topicString.length()];
  topicString.toCharArray(topic, topicString.length() + 1);

  //print topic
  #ifdef VERBOSE
  Serial.print("topic: ");
  Serial.println(topic);
  #endif //VERBOSE

  //------------------------------------------
  //MQTT details
  #ifdef VERBOSE
  Serial.print("server: ");
  Serial.println(MQTTServer);
  Serial.print("user: ");
  Serial.println(MQTTUsername);
  Serial.print("auth: ");
  Serial.println(MQTTPassword);
  #endif //VERBOSE
  
  //------------------------------------------
  //post
  
  #ifdef POST
  //set MQTT server
  MQTTClient.setServer(MQTTServer, MQTTPort);

  //------------------------------------------
  //connect to wifi
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnect();
  }
  
  //------------------------------------------
  //post data
  if (WiFi.status() == WL_CONNECTED) {

    //------------------------------------------
    //connect to MQTT broker
    MQTTConnect();
    if (MQTTClient.connected()) {

      //------------------------------------------
      //call the loop continuously to establish connection to the server
      MQTTClient.loop();      
      
      //------------------------------------------
      //publish
      if (MQTTClient.publish(topic, message)) {
        #ifdef VERBOSE
        Serial.println("success");
        #endif //VERBOSE
      } else {
        #ifdef VERBOSE
        Serial.println("fail");
        #endif //VERBOSE
      }
      Serial.println();

      //------------------------------------------
      //disconnect
      MQTTClient.disconnect();
    }
  }

  //------------------------------------------
  #ifndef CAFFEINE
  //disconnect before leaving
  WiFi.disconnect();
  #endif //CAFFEINE
  
  #endif //POST
  return;
}

//******************************************
//connect to wifi
#ifdef POST
void wifiConnect() {

  //https://github.com/esp8266/Arduino/issues/2702
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  #ifdef VERBOSE
  Serial.println();
  Serial.print("wifi connecting to ");
  Serial.println(ssid);
  #endif //VERBOSE

  //try connecting for 20 times with 1 s intervals
  for (int ii = 0; ii < 20; ii++) {

    //status
    #ifdef VERBOSE
    Serial.print("status: ");
    switch (WiFi.status()) {
      case 0:
        Serial.println("idle");
        break;
      case 1:
        Serial.println("no SSID");
        break;
      case 2:
        Serial.println("scan compl.");
        break;
      case 3:
        Serial.println("connected");
        break;
      case 4:
        Serial.println("connection failed");
        break;
      case 5:
        Serial.println("connection lost");
        break;
      case 6:
        Serial.println("disconnected");
        delay(1000);//ms
        break;
      default:
        Serial.println("unknown");
        break;
    }
    #endif //VERBOSE

    if (WiFi.status() == WL_CONNECTED) {
      #ifdef VERBOSE
      Serial.println();
      Serial.print("wifi connected to ");
      Serial.println(ssid);
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI());
      Serial.println();
      #endif //VERBOSE
      break;
    }

    //delay between trials
    delay(1000);//ms
  }

  #ifdef VERBOSE
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("could not connect to ");
    Serial.println(ssid);
    Serial.println("abort");
    Serial.println();
  }
  #endif
}
#endif //POST

//******************************************
//connect to MQTT broker
#ifdef POST
void MQTTConnect() {

  //show status
  #ifdef VERBOSE
  Serial.print("connecting to MQTT server ");
  Serial.print(MQTTServer);
  Serial.println("...");
  #endif

  //random clientID
  char clientID[10];

  //try connecting for 10 times with 1 s intervals
  for (int ii = 0; ii < 10; ii++) {

    //generate random clientID
    for (int i = 0; i < 10; i++) {
      clientID[i] = alphanum[random(51)];
    }

    //https://community.thingspeak.com/forum/esp8266-wi-fi/problem-rc-4-using-library-pubsub/
    clientID[10] = '\0';

    //connect
    MQTTClient.connect(clientID, MQTTUsername, MQTTPassword);

    //connection status
    //print to know why the connection failed
    //see http://pubsubclient.knolleary.net/api.html#state for the failure code explanation
    #ifdef VERBOSE
    Serial.print("status: ");

    switch (MQTTClient.state()) {
      case -4:
        Serial.println("timeout");
        break;
      case -3:
        Serial.println("lost");
        break;
      case -2:
        Serial.println("failed");
        break;
      case -1:
        Serial.println("disconnected");
        break;
      case 0:
        Serial.println("connected");
        break;
      case 1:
        Serial.println("bad protocol");
        break;
      case 2:
        Serial.println("bad client ID");
        break;
      case 3:
        Serial.println("unavailable");
        break;
      case 4:
        Serial.println("bad credentials");
        break;
      case 5:
        Serial.println("unauthorized");
        break;
      default:
        Serial.println("unknown");
        break;
    }
    #endif

    //upon successful connection
    if (MQTTClient.connected()) {
      #ifdef VERBOSE
      Serial.print("MQTT connected to ");
      Serial.println(MQTTServer);
      #endif
      break;
    }

    //delay between trials
    delay(1000);//ms
  }//END connecting loop
}
#endif //POST

//******************************************
//set JSON document T, RH and dew point
void setDocTRH(JsonDocument &doc, float t, float rh) {

  //temperature
  if ( ! isnan(t)) doc["temp"]=t;
  else doc["temp"]="\"NaN\"";

  //relative humidity
  if ( ! isnan(rh)) doc["rh"]=rh;
  else doc["rh"]="\"NaN\"";
  
  //dew point
  if ( ! isnan(getDewPoint(t, rh))) {
    doc["dewpoint"]=getDewPoint(t, rh);
    if (t > getDewPoint(t, rh)) doc["dewpointstatus"]=1;
    else doc["dewpointstatus"]=0;
  } else {
    doc["dewpoint"]="\"NaN\"";
    doc["dewpointstatus"]="\"NaN\"";
  }
  
}

//******************************************
//add metadata to JSON document: institute, room, location and name
void addMetaData(JsonDocument &doc) {
  #ifdef INSTITUTE
  doc["instutute"]=INSTITUTE;
  #endif //INSTITUTE
  #ifdef ROOM
  doc["room"]=ROOM;
  #endif //ROOM
  #ifdef LOCATION
  doc["location"]=LOCATION;
  #endif //LOCATION
  #ifdef NAME
  doc["name"]=NAME;
  #endif //NAME
}

//******************************************
void getMQTTMessage(JsonDocument &doc) {

  //------------------------------------------
  //SHTA
  #ifdef SHTA
  StaticJsonDocument<160> docSHTA;
  setDocTRH(docSHTA, ta, rha);
  doc["sensor"]="SHTA";
  addMetaData(docSHTA);
  doc.add(docSHTA);
  #endif //SHTA

  //------------------------------------------
  //SHTB
  #ifdef SHTB
  StaticJsonDocument<160> docSHTB;
  setDocTRH(docSHTB, tb, rhb);
  docSHTB["sensor"]="SHTB";
  addMetaData(docSHTB);
  doc.add(docSHTB);
  #endif //SHTB

  //------------------------------------------
  //SPS30128
  #ifdef SPS30
  StaticJsonDocument<128> docSPS30;
  if ( ! isnan(dustnc0p5)) docSPS30["dustnc0p5"]=dustnc0p5;
  else docSPS30["dustnc0p5"]="\"NaN\"";
  docSPS30["sensor"]="SPS30";
  addMetaData(docSPS30);
  doc.add(docSPS30);
  #endif //SPS30
  /*
  #ifdef SPS30
  data += String("{");
  if ( ! isnan(dustnc0p5))
    data += String("\"dustnc0p5\":" + String(dustnc0p5,6));
  else
    data += String("\"dustnc0p5\":\"NaN\"");
  if ( ! isnan(dustnc1p0))
    data += String(",\"dustnc1p0\":" + String(dustnc1p0,6));
  else
    data += String(",\"dustnc1p0\":\"NaN\"");
  if ( ! isnan(dustnc2p5))
    data += String(",\"dustnc2p5\":" + String(dustnc2p5,6));
  else
    data += String(",\"dustnc2p5\":\"NaN\"");
  if ( ! isnan(dustnc4p0))
    data += String(",\"dustnc4p0\":" + String(dustnc4p0,6));
  else
    data += String(",\"dustnc4p0\":\"NaN\"");
  if ( ! isnan(nc0p5))
    data += String(",\"nc0p5\":" + String(nc0p5,3));
  else
    data += String(",\"nc0p5\":\"NaN\"");
  if ( ! isnan(nc1p0))
    data += String(",\"nc1p0\":" + String(nc1p0,3));
  else
    data += String(",\"nc1p0\":\"NaN\"");
  if ( ! isnan(nc2p5))
    data += String(",\"nc2p5\":" + String(nc2p5,3));
  else
    data += String(",\"nc2p5\":\"NaN\"");
  if ( ! isnan(nc4p0))
    data += String(",\"nc4p0\":" + String(nc4p0,3));
  else
    data += String(",\"nc4p0\":\"NaN\"");
  if ( ! isnan(nc10p0))
    data += String(",\"nc10p0\":" + String(nc10p0,3));
  else
    data += String(",\"nc10p0\":\"NaN\"");
  if ( ! isnan(dustmc1p0))
    data += String(",\"dustmc1p0\":" + String(dustmc1p0,6));
  else
    data += String(",\"dustmc1p0\":\"NaN\"");
  if ( ! isnan(dustmc2p5))
    data += String(",\"dustmc2p5\":" + String(dustmc2p5,6));
  else
    data += String(",\"dustmc2p5\":\"NaN\"");
  if ( ! isnan(dustmc4p0))
    data += String(",\"dustmc4p0\":" + String(dustmc4p0,6));
  else
    data += String(",\"dustmc4p0\":\"NaN\"");
  if ( ! isnan(mc1p0))
    data += String(",\"mc1p0\":" + String(mc1p0,3));
  else
    data += String(",\"mc1p0\":\"NaN\"");
  if ( ! isnan(mc2p5))
    data += String(",\"mc2p5\":" + String(mc2p5,3));
  else
    data += String(",\"mc2p5\":\"NaN\"");
  if ( ! isnan(mc4p0))
    data += String(",\"mc4p0\":" + String(mc4p0,3));
  else
    data += String(",\"mc4p0\":\"NaN\"");
  if ( ! isnan(mc10p0))
    data += String(",\"mc10p0\":" + String(mc10p0,3));
  else
    data += String(",\"mc10p0\":\"NaN\"");
  if ( ! isnan(dustsize))
    data += String(",\"dustsize\":" + String(dustsize,3));
  else
    data += String(",\"dustsize\":\"NaN\"");
  data += String(",\"sensor\":\"SPS30\"");
  data += getMetadataString();
  data += String("},");
  #endif //SPS30
  */

  //------------------------------------------
  //SDP610
  #ifdef SDP610
  StaticJsonDocument<128> docSDP610;
  if ( ! isnan(dp)) docSDP610["differentialpressure"]=dp;
  else docSDP610["differentialpressure"]="\"NaN\"";
  docSDP610["sensor"]="SDP610";
  addMetaData(docSDP610);
  doc.add(docSDP610);
  #endif //SDP610

  //------------------------------------------
  //ADC0
  #ifdef ADC0
  StaticJsonDocument<128> docADC0;
  if ( ! isnan(adc0)) docADC0["adc"]=adc0;
  else docADC0["adc"]="\"NaN\"";
  docADC0["sensor"]="ADC0";
  addMetaData(docADC0);
  doc.add(docADC0);
  #endif //ADC0
  
  //NTC0
  #ifdef NTC0
  StaticJsonDocument<128> docNTC0;
  if ( ! isnan(tntc0)) docNTC0["temp"]=tntc0;
  else docNTC0["temp"]="\"NaN\"";
  docNTC0["sensor"]="NTC0";
  addMetaData(docNTC0);
  doc.add(docNTC0);
  #endif //NTC0

  //------------------------------------------
  //ADC1
  #ifdef ADC1
  StaticJsonDocument<128> docADC1;
  if ( ! isnan(adc1)) docADC1["adc"]=adc1;
  else docADC1["adc"]="\"NaN\"";
  docADC1["sensor"]="ADC1";
  addMetaData(docADC1);
  doc.add(docADC1);
  #endif //ADC1
  
  //NTC1
  #ifdef NTC1
  StaticJsonDocument<128> docNTC1;
  if ( ! isnan(tntc1)) docNTC1["temp"]=tntc1;
  else docNTC1["temp"]="\"NaN\"";
  docNTC1["sensor"]="NTC1";
  addMetaData(docNTC1);
  doc.add(docNTC1);
  #endif //NTC1

  //------------------------------------------
  //ADC2
  #ifdef ADC2
  StaticJsonDocument<128> docADC2;
  if ( ! isnan(adc2)) docADC2["adc"]=adc2;
  else docADC2["adc"]="\"NaN\"";
  docADC2["sensor"]="ADC2";
  addMetaData(docADC2);
  doc.add(docADC2);
  #endif //ADC2
  
  //NTC2
  #ifdef NTC2
  StaticJsonDocument<128> docNTC2;
  if ( ! isnan(tntc2)) docNTC2["temp"]=tntc2;
  else docNTC2["temp"]="\"NaN\"";
  docNTC2["sensor"]="NTC2";
  addMetaData(docNTC2);
  doc.add(docNTC2);
  #endif //NTC2

  //------------------------------------------
  //ADC3
  #ifdef ADC3
  StaticJsonDocument<128> docADC3;
  if ( ! isnan(adc3)) docADC3["adc"]=adc3;
  else docADC3["adc"]="\"NaN\"";
  docADC3["sensor"]="ADC3";
  addMetaData(docADC3);
  doc.add(docADC3);
  #endif //ADC3
  
  //NTC3
  #ifdef NTC3
  StaticJsonDocument<128> docNTC3;
  if ( ! isnan(tntc3)) docNTC3["temp"]=tntc3;
  else docNTC3["temp"]="\"NaN\"";
  docNTC3["sensor"]="NTC3";
  addMetaData(docNTC3);
  doc.add(docNTC3);
  #endif //NTC3

  //------------------------------------------
  //ADC4
  #ifdef ADC4
  StaticJsonDocument<128> docADC4;
  if ( ! isnan(adc4)) docADC4["adc"]=adc4;
  else docADC4["adc"]="\"NaN\"";
  docADC4["sensor"]="ADC4";
  addMetaData(docADC4);
  doc.add(docADC4);
  #endif //ADC4
  
  //NTC4
  #ifdef NTC
  StaticJsonDocument<128> docNTC4;
  if ( ! isnan(tntc4)) docNTC4["temp"]=tntc4;
  else docNTC4["temp"]="\"NaN\"";
  docNTC4["sensor"]="NTC4";
  addMetaData(docNTC4);
  doc.add(docNTC4);
  #endif //NTC4

  //------------------------------------------
  return;
}

