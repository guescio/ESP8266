/*
  This sketch reads the ambient temperature, humidity, differential pressure and particle count, prints them to serial output, posts them online and then goes to sleep.
  The values measured are posted through MQTT.
  Temperature and humidity can be read from SHT35 and SHT85 sensors.
  Don't forget to connect pin 16 with the reset pin.
  Printing and posting can be enabled/disabled by uncommenting/commenting the relative options in the definitions section below.
  To enable/disable a sensor, uncomment/comment its name in the definitions section below.
  Author: Guescio.
*/

//******************************************
//definitions
#define SHTA //0x44 address //SHT35A or SHT85
#define SHTB //0x45 address //SHT35B
#define SPS30
//#define SDP610
//#define ADC//NOTE: set also the voltage divider resistor value
#define ADCRDIV (84.5e3)//ADC voltage resistor value [Ohm]
//#define NTC//NOTE: set also the voltage divider resistor value and NTC R0, T0 and B values
#define NTCR0 (1e4)//NTC R0 [Ohm]
#define NTCT0 (298.15)//NTC T0 [K]
#define NTCB (3435)//NTC B
#define SLEEPTIME (60)//s
//#define PRINTSERIAL //print measurements to serial output
#define POST //connect and post measurements online
//#define QUIET //connect to broker but ldo not post
//#define VERBOSE //print connection status and posting details
//#define MQTTSERVER ("127.0.0.1")
//#define MQTTUSER ("user")
//#define MQTTPASSWORD ("password")
//#define MQTTTOPIC ("topic") //MQTT topic
//#define INSTITUTE ("institute") //institute of the measurement device //NOTE this is used also for the MQTT topic
//#define ROOM("room") //room of the measurement device
//#define LOCATION ("location") //location of the measurement device
//#define NAME("name") //name of the measurement device

//******************************************
//libraries
#if defined(SHTA) || defined(SHTB)
#include "Adafruit_SHT31.h"
#endif

#ifdef SPS30
#include "sps30.h"
#endif

#ifdef SDP610
#include <Wire.h>
#include "SDP6x.h"
#endif

#ifdef POST
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <guescio.h>
#endif

#include <limits>

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

//SPS30 does not need initializaiton
//SDP610 does not need initialization

//******************************************
//wifi and MQTT setup
#ifdef POST
//wifi
const char* ssid     = WIFISSID;
const char* password = WIFIPASSWORD;
//initialize wifi client library
WiFiClient WiFiclient;

//MQTT
char MQTTServer[] = MQTTSERVER;
long MQTTPort = 1883;
char MQTTUsername[] = MQTTUSER;
char MQTTPassword[] = MQTTPASSWORD;
//NOTE: need a random client ID for posting
static const char alphanum[] = "0123456789"
                               "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                               "abcdefghijklmnopqrstuvwxyz";
//initialize PuBSubClient library
PubSubClient MQTTClient(WiFiclient);
#endif //POST

//******************************************
//setup
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

#if defined(PRINTSERIAL) || defined(VERBOSE)
  Serial.println("\nsleep, measure, post, repeat");
#endif

  //initialize SHTs
#ifdef SHTA
  sht31a.begin(SHTAADDR);
#endif

#ifdef SHTB
  sht31b.begin(SHTBADDR);
#endif

  //initialize SPS30
#ifdef SPS30

#ifdef VERBOSE
  Serial.println("\ninitialising SPS30");
#endif

  s16 ret;
  u8 auto_clean_days = 4;
  u32 auto_clean;
  bool sps30available = false;

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

#ifdef defined(SPS30_LIMITED_I2C_BUFFER_SIZE)
  Serial.println("The hardware has a limitation that only");
  Serial.println("  allows reading the mass concentrations.");
  Serial.println("  For more information, please check:");
  Serial.println("  https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support\n");
#endif //SPS30_LIMITED_I2C_BUFFER_SIZE
#endif //SPS30

  //initialize SDP610
#ifdef SDP610
  Wire.begin();
  Wire.setClockStretchLimit(1e4);//µs
#endif

  //set initial values
  float ta = std::numeric_limits<double>::quiet_NaN();//temperature A [C]
  float rha = std::numeric_limits<double>::quiet_NaN();//relative humidity A [%]
  float dpa = std::numeric_limits<double>::quiet_NaN();//dew point A [C]
  float tb = std::numeric_limits<double>::quiet_NaN();//temperature B [C]
  float rhb = std::numeric_limits<double>::quiet_NaN();//relative humidity B [%]
  float dpb = std::numeric_limits<double>::quiet_NaN();//dew point B [C]
  float dp = std::numeric_limits<double>::quiet_NaN();//differential pressure [Pa]
  float dt = std::numeric_limits<double>::quiet_NaN();//temperature difference [C]
  int adc = std::numeric_limits<double>::quiet_NaN();//ADC []
  float tntc = std::numeric_limits<double>::quiet_NaN();//temperature NTC [C]
  float dust0p5 = std::numeric_limits<double>::quiet_NaN();//>0.5 um particles concentration [ft^-3]
  float pm0p5 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <0.5 um particles concentration [ft^-3]
  float pm1p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <1.0 um particles concentration [ft^-3]
  float pm2p5 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <2.5 um particles concentration [ft^-3]
  float pm4p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <4.0 um particles concentration [ft^-3]
  float pm10p0 = std::numeric_limits<double>::quiet_NaN();//0.3um< size <10.0 um particles concentration [ft^-3]

  //read values before the ESP8266 heats up
#ifdef SHTA
  ta = sht31a.readTemperature();
  rha = sht31a.readHumidity();
#endif //SHTA

#ifdef SHTB
  tb = sht31b.readTemperature();
  rhb = sht31b.readHumidity();
#endif //SHTB

#ifdef SPS30
  if (sps30available) {

    struct sps30_measurement m;
    char serial[SPS_MAX_SERIAL_LEN];
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
      dust0p5 = (m.nc_10p0 - m.nc_0p5) * pow(30.48, 3); //particles/ft^3
      pm0p5 = m.nc_0p5 * pow(30.48, 3); //particles/ft^3
      pm1p0 = m.nc_1p0 * pow(30.48, 3); //particles/ft^3
      pm2p5 = m.nc_2p5 * pow(30.48, 3); //particles/ft^3
      pm4p0 = m.nc_4p0 * pow(30.48, 3); //particles/ft^3
      pm10p0 = m.nc_10p0 * pow(30.48, 3); //particles/ft^3
      //Serial.print("particle concentration (>0.5 um and <10 um): ");
      //Serial.print(dust0p5/pow(30.48, 3));
      //Serial.println(" cm^-3");
      //Serial.print("particle concentration (>0.5 um and <10 um): ");
      //Serial.print(dust0p5);
      //Serial.println(" ft^-3");
      //Serial.print("typical partical size: ");
      //Serial.print(m.typical_pisoarticle_size);
      //Serial.println(" um");
    }

  }//sps30available
#endif //SPS30

#ifdef SDP610
  dp = SDP6x.GetPressureDiff();
#endif //SDP610

#if defined(SHTA) && defined(SHTB)
  dt = ta - tb;
#endif //SHTA && SHTB

#if defined(ADC) or defined (NTC)
  adc = getADC();
#endif //ADC

#ifdef NTC
  tntc = getTNTC(adc);
#endif //NTC

  //print serial
#ifdef PRINTSERIAL
  printSerial(ta, rha, tb, rhb, dp, dt, adc, tntc, dust0p5, pm0p5, pm1p0, pm2p5, pm4p0, pm10p0);
#endif //PRINTSERIAL

  //connect to wifi and post data
#ifdef POST
  //set MQTT server
  MQTTClient.setServer(MQTTServer, MQTTPort);

  //connect to wifi
  wifiConnect();

  //post data
  if (WiFi.status() == WL_CONNECTED) {
    postData(ta, rha, tb, rhb, dp, dt, adc, tntc, dust0p5, pm0p5, pm1p0, pm2p5, pm4p0, pm10p0);
  }

  //disconnect before leaving
  WiFi.disconnect();
#endif //POST

  //deep sleep
  ESP.deepSleep(SLEEPTIME * 1e6); //µs
}

//******************************************
//loop() is empty since the ESP8266 is sent to deep sleep at the end of setup()
void loop() {}

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
#if defined(ADC) or defined(NTC)
float getADC() {
  int value = 0;
  int nreadings = 10;
  for (int ii = 0; ii < nreadings; ii++) {
    value += analogRead(A0);
    delay(10);//ms
  }
  if (! isnan(value)) {
    return value / nreadings;
  }
  else return std::numeric_limits<double>::quiet_NaN();
}
#endif //ADC or NTC

//******************************************
//get NTC temperature
#ifdef NTC
float getTNTC(int adc) {
  if (! isnan(adc)) {
    float rntc = ADCRDIV / (1023 / adc - 1); //Ohm
    return (1. / ((1. / NTCT0) + (1. / NTCB) * log((rntc / NTCR0))) - 273.15); //C
  }
  else return std::numeric_limits<double>::quiet_NaN();
}
#endif //NTC

//******************************************
//print measurements to serial output
void printSerial(float ta, float rha, float tb, float rhb, float dp, float dt, int adc, float tntc, float dust0p5, float pm0p5, float pm1p0, float pm2p5, float pm4p0, float pm10p0) {

  Serial.println();

#ifdef SHTA
  Serial.print("t_a[C] RH_a[%] DP_a[C]");
#endif
#ifdef SHTB
  Serial.print(" t_b[C] RH_b[%] DP_b[C]");
#endif
#ifdef SPS30
  Serial.print(" dust[ft^-3] PM0.5[ft^-3] PM1.0[ft^-3] PM2.5[ft^-3] PM4.0[ft^-3] PM10.0[ft^-3]");
#endif
#ifdef SDP610
  Serial.print(" dP[Pa]");
#endif
#if defined(SHTA) && defined(SHTB)
  Serial.print(" dt[C]");
#endif
#ifdef ADC
  Serial.print(" ADC");
#endif
#ifdef NTC
  Serial.print(" t_NTC[C]");
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
  Serial.print(dust0p5);//particle concentration (>0.5 um)
  Serial.print("   ");
  Serial.print(pm0p5);//particle concentration (<0.5 um)
  Serial.print("   ");
  Serial.print(pm1p0);//particle concentration (<1.0 um)
  Serial.print("   ");
  Serial.print(pm2p5);//particle concentration (<2.5 um)
  Serial.print("   ");
  Serial.print(pm4p0);//particle concentration (<4.0 um)
  Serial.print("   ");
  Serial.print(pm10p0);//particle concentration (<10.0 um)
  Serial.print("    ");
#endif

#ifdef SDP610
  Serial.print(dp);//differential pressure
  Serial.print("  ");
#endif

#if defined(SHTA) && defined(SHTB)
  Serial.print(dt);//temperature difference
  Serial.print("  ");
#endif

#ifdef ADC
  Serial.print(adc);//ADC
  Serial.print("  ");
#endif

#ifdef NTC
  Serial.print(tntc);//temperature
  Serial.print("  ");
#endif

  Serial.println();
}

//******************************************
//connect to wifi
//try connecting for 20 times with 1 s intervals
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

  //connect
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
//post data online using MQTT protocol
#ifdef POST
void postData(float ta, float rha, float tb, float rhb, float dp, float dt, int adc, float tntc, float dust0p5, float pm0p5, float pm1p0, float pm2p5, float pm4p0, float pm10p0) {
  //connect to MQTT broker
  MQTTConnect();

  if (MQTTClient.connected()) {
    //call the loop continuously to establish connection to the server
    MQTTClient.loop();

    //publish data
    MQTTPublish(ta, rha, tb, rhb, dp, dt, adc, tntc, dust0p5, pm0p5, pm1p0, pm2p5, pm4p0, pm10p0);

    //disconnect
    MQTTClient.disconnect();
  }
}
#endif //POST

//******************************************
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

  //try connecting for 5 times with 2 s intervals
  for (int ii = 0; ii < 5; ii++) {

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
    delay(2000);//ms
  }//END connecting loop
}
#endif //POST

//******************************************
//string to post with T, RH and dew point
String getTRHString(float t, float rh) {
  String data = String("");

  //temperature
  if ( ! isnan(t))
    data += String("\"temp\":" + String(t));
  else
    data += String("\"temp\":\"NaN\"");

  //relative humidity
  if ( ! isnan(rh))
    data += String(",\"rh\":" + String(rh));
  else
    data += String(",\"rh\":\"NaN\"");

  //dew point
  if ( ! isnan(getDewPoint(t, rh))) {
    data += String(",\"dewpoint\":" + String(getDewPoint(t, rh)));
    if (t > getDewPoint(t, rh)) data += String(",\"dewpstatus\":1");
    else data += String(",\"dewpointstatus\":0");
  } else {
    data += String(",\"dewpoint\":\"NaN\"");
    data += String(",\"dewpointstatus\":\"NaN\"");
  }

  return data;
}

//******************************************
//metadata string with institute, room, location and name
String getMetadataString() {
  String data = String("");
#ifdef INSTITUTE
  data += String(",\"institute\":\"");
  data += INSTITUTE;
  data += String("\"");
#endif //INSTITUTE
#ifdef ROOM
  data += String(",\"room\":\"");
  data += ROOM;
  data += String("\"");
#endif //ROOM
#ifdef LOCATION
  data += String(",\"location\":\"");
  data += LOCATION;
  data += String("\"");
#endif //LOCATION
#ifdef NAME
  data += String(",\"name\":\"");
  data += NAME;
  data += String("\"");
#endif //NAME
  return data;
}

//******************************************
#ifdef POST
void MQTTPublish(float ta, float rha, float tb, float rhb, float dp, float dt, int adc, float tntc, float dust0p5, float pm0p5, float pm1p0, float pm2p5, float pm4p0, float pm10p0) {

  //print
#ifdef VERBOSE
  Serial.println("posting data");
#endif

  //create data string to send to MQTT broker
  //NOTE the string is a JSON array
  String data = String("[");

  //SHTA
#ifdef SHTA
  data += String("{");
  data += getTRHString(ta, rha);
  data += String(",\"sensor\":\"SHTA\"");
  data += getMetadataString();
  data += String("},");
#endif //SHTA

  //SHTB
#ifdef SHTB
  data += String("{");
  data += getTRHString(tb, rhb);
  data += String(",\"sensor\":\"SHTB\"");
  data += getMetadataString();
  data += String("},");
#endif //SHTB

  //SPS30
#ifdef SPS30
  data += String("{");
  if ( ! isnan(dust0p5))
    data += String("\"dust0p5\":" + String(dust0p5));
  else
    data += String("\"dust0p5\":\"NaN\"");
  if ( ! isnan(pm0p5))
    data += String(",\"pm0p5\":" + String(pm0p5));
  else
    data += String(",\"pm0p5\":\"NaN\"");
  if ( ! isnan(pm1p0))
    data += String(",\"pm1p0\":" + String(pm1p0));
  else
    data += String(",\"pm1p0\":\"NaN\"");
  if ( ! isnan(pm2p5))
    data += String(",\"pm2p5\":" + String(pm2p5));
  else
    data += String(",\"pm2p5\":\"NaN\"");
  if ( ! isnan(pm4p0))
    data += String(",\"pm4p0\":" + String(pm4p0));
  else
    data += String(",\"pm4p0\":\"NaN\"");
  if ( ! isnan(pm10p0))
    data += String(",\"pm10p0\":" + String(pm10p0));
  else
    data += String(",\"pm10p0\":\"NaN\"");
  data += String(",\"sensor\":\"SPS30\"");
  data += getMetadataString();
  data += String("},");
#endif //SPS30

  //SDP610
#ifdef SDP610
  data += String("{");
  if ( ! isnan(dp))
    data += String("\"differentialpressure\":" + String(dp));
  else
    data += String("\"differentialpressure\":\"NaN\"");
  data += String(",\"sensor\":\"SDP610\"");
  data += getMetadataString();
  data += String("},");
#endif //SDP610

  //ADC
#ifdef ADC
  data += String("{");
  if ( ! isnan(adc))
    data += String("\"adc\":" + String(adc));
  else
    data += String("\"adc\":\"NaN\"");
  data += String(",\"sensor\":\"ADC\"");
  data += getMetadataString();
  data += String("},");
#endif //ADC

  //NTC
#ifdef NTC
  data += String("{");
  data += getTRHString(tntc, std::numeric_limits<double>::quiet_NaN());
  data += String(",\"sensor\":\"NTC\"");
  data += getMetadataString();
  data += String("},");
#endif //NTC

  //remove trailing ","
  if (data.endsWith(",")) {
    data.remove(data.length() - 1);
  }

  //end of array
  data += String("]");

  //convert message string to char array
  int length = data.length();
  char msgBuffer[length];
  data.toCharArray(msgBuffer, length + 1);

  //print message
#ifdef VERBOSE
  Serial.print("message: ");
  Serial.println(msgBuffer);
  Serial.print("message length: ");
  Serial.println(length);
#endif

  //create topic string
  String topicString = MQTTTOPIC;

  //append institute to topic
#ifdef INSTITUTE
  topicString += "/";
  topicString += INSTITUTE;
#endif //INSTITUTE

  length = topicString.length();
  char topicBuffer[length];
  topicString.toCharArray(topicBuffer, length + 1);
#ifdef VERBOSE
  Serial.print("topic: ");
  Serial.println(topicString);
#endif

  //publish
#ifndef QUIET
  if (MQTTClient.publish(topicBuffer, msgBuffer)) {
#ifdef VERBOSE
    Serial.println("success");
#endif //VERBOSE
  } else {
#ifdef VERBOSE
    Serial.println("fail");
#endif //VERBOSE
  }
  Serial.println();
#endif //QUIET
}
#endif //POST

