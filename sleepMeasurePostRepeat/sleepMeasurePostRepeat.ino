/*
  This sketch reads the ambient temperature, humidity, differential pressure and particle count,
  prints them to serial output, posts them online and then goes to sleep (unless caffeine is taken).
  The values measured are posted through MQTT.
  Temperature and humidity can be read from SHT35 and SHT85 sensors.
  Temperature can be read from an NTC.
  The dust particle count is measured by a SPS30 sensor.
  Don't forget to connect pin 16 to the reset pin to wake up from deep sleep (again, unless caffeine is taken).
  Printing and posting can be enabled/disabled by uncommenting/commenting the relative options in the definitions section below.
  To enable/disable a sensor, uncomment/comment its name in the definitions section below.
  Author: Guescio.
*/

//******************************************
//definitions
#define CAFFEINE //do not go to deep sleep
#define SHTA //SHT35A or SHT85 (0x44 address)
#define SHTB //SHT35B (0x45 address)
#define SPS30
//#define SDP610
//#define ADC//NOTE: set also the voltage divider resistor value
#define ADCRDIV (84.5e3)//ADC voltage resistor value [Ohm]
//#define NTC//NOTE: set also the voltage divider resistor value and NTC R0, T0 and B values
#define NTCR0 (1e4)//NTC R0 [Ohm]
#define NTCT0 (298.15)//NTC T0 [K]
#define NTCB (3435)//NTC B
#define ADCVREF (1.0)//ADC reference voltage [V]
#define VDD (3.3)//voltage supply [V]
#define SLEEPTIME (60)//s
//#define PRINTSERIAL //print measurements to serial output
#define POST //connect and post measurements online
//#define QUIET //connect to broker but do not post
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
#endif

#include <limits>
#include <guescio.h>

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

#if defined(POST) || defined(QUIET)
//MQTT
char MQTTServer[] = MQTTSERVER;
long MQTTPort = 1883;
char MQTTUsername[] = MQTTUSER;
char MQTTPassword[] = MQTTPASSWORD;
//NOTE: need a random client ID for posting
static const char alphanum[] = "0123456789"
                               "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                               "abcdefghijklmnopqrstuvwxyz";
#endif //POST || QUIET
                               
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
float dt = std::numeric_limits<double>::quiet_NaN();//temperature difference [C]
int adc = std::numeric_limits<double>::quiet_NaN();//ADC []
float tntc = std::numeric_limits<double>::quiet_NaN();//temperature NTC [C]
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
  Serial.begin(115200);
  while (!Serial) {}

#if defined(PRINTSERIAL) || defined(VERBOSE)
  Serial.println("\nsleep, measure, post, repeat");
  #ifdef CAFFEINE
  Serial.println("caffeine!");
  #endif //CAFFEINE
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

  //initialize SDP610
#ifdef SDP610
  Wire.begin();
  Wire.setClockStretchLimit(1e4);//µs
#endif

#if defined(CAFFEINE) && defined(POST)
  //------------------------------------------
  //connect to wifi already
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnect();
  }
#endif //CAFFEINE and POST

#ifndef CAFFEINE

  //------------------------------------------
  //do it all: read values, print them and post them
  readPrintPost();
  
  //------------------------------------------
  //deep sleep
  ESP.deepSleep(SLEEPTIME * 1e6); //µs
#endif //CAFFEINE
}

//******************************************
void loop() {
  
  //------------------------------------------
  //do it all: read values, print them and post them
  readPrintPost();

  //------------------------------------------
  //delay but do not sleep
  delay(SLEEPTIME * 1e3); //ms
}

//******************************************
//do it all: read values, print them and post them
void readPrintPost() {

  //------------------------------------------
  //read values
  readAllValues();

  //------------------------------------------
  //print serial
#ifdef PRINTSERIAL
  printSerial();
#endif //PRINTSERIAL

  //------------------------------------------
  //connect to wifi and post data
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

    //connect to MQTT broker
    MQTTConnect();
    if (MQTTClient.connected()) {
      
      //call the loop continuously to establish connection to the server
      MQTTClient.loop();

      //publish data
      //NOTE use even when POST flag is not defined to print MQTT info
      MQTTPublish();

      //disconnect
      MQTTClient.disconnect();
    }
  }

  //------------------------------------------
  #ifndef CAFFEINE
  //disconnect before leaving
  WiFi.disconnect();
  #endif //CAFFEINE
  
#elif defined(QUIET)
  //------------------------------------------
  //simply print MQTT message info without actually connecting
  MQTTPublish();

  //------------------------------------------
  //print MQTT connection details
  #ifdef VERBOSE
  Serial.print("server: ");
  Serial.println(MQTTServer);
  Serial.print("user: ");
  Serial.println(MQTTUsername);
  Serial.print("auth: ");
  Serial.println(MQTTPassword);
  #endif //VERBOSE
#endif //POST or QUIET

  //------------------------------------------
  return;
}

//******************************************
//read values from all sensors enabled
void readAllValues() {

  //set values to NaN
  ta = std::numeric_limits<double>::quiet_NaN();//temperature A [C]
  rha = std::numeric_limits<double>::quiet_NaN();//relative humidity A [%]
  dpa = std::numeric_limits<double>::quiet_NaN();//dew point A [C]
  tb = std::numeric_limits<double>::quiet_NaN();//temperature B [C]
  rhb = std::numeric_limits<double>::quiet_NaN();//relative humidity B [%]
  dpb = std::numeric_limits<double>::quiet_NaN();//dew point B [C]
  dp = std::numeric_limits<double>::quiet_NaN();//differential pressure [Pa]
  dt = std::numeric_limits<double>::quiet_NaN();//temperature difference [C]
  adc = std::numeric_limits<double>::quiet_NaN();//ADC []
  tntc = std::numeric_limits<double>::quiet_NaN();//temperature NTC [C]
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
    float rntc = ADCRDIV / (1023. / adc * VDD / ADCVREF - 1.); //Ohm
    return (1. / ((1. / NTCT0) + (1. / NTCB) * log((rntc / NTCR0))) - 273.15); //C
  }
  else return std::numeric_limits<double>::quiet_NaN();
}
#endif //NTC

//******************************************
//print measurements to serial output
void printSerial() {

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
#ifdef ADC
  Serial.print("ADC ");
#endif
#ifdef NTC
  Serial.print("t_NTC[C] ");
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
#if defined(POST) || defined(QUIET)
void MQTTPublish() {

  //print
#if defined(POST) && !defined(QUIET) && defined(VERBOSE)
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

