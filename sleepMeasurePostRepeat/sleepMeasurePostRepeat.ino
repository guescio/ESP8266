/* 
  This sketch reads the ambient temperature, humidity and differential pressure, prints them to serial output, posts them online and then goes to sleep.
  The values measured are posted on ThingSpeak through MQTT.
  Temperature and humidity can be read from two different sensors.
  Don't forget to connect pin 16 with the reset pin.
  Printing and posting can be enabled/disabled by uncommenting/commenting the relative options in the definitions section below.
  To enable/disable a sensor, uncomment/comment its name in the definitions section below.
  NOTE: the maximum number of fields that can be uploaded seems to be six.
  Author: Guescio.
*/

//******************************************
//definitions
#define SHT35A //0x44 address
//#define SHT35B //0x45 address
//#define SDP610
#define SLEEPTIME (60)//s
//#define PRINTSERIAL //print measurements to serial output
#define POST //post measurements online
//#define QUIET //test posting but do not post
#define FALKOR //post to falkor instead of ThingSpeak
//#define VERBOSE //print connection status and posting details

//******************************************
#if defined(SHT35A) || defined(SHT35B)
#include "Adafruit_SHT31.h"
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
//wifi
#ifdef POST
const char* ssid     = WIFISSID;
const char* password = WIFIPASSWORD;
#endif

//******************************************
//MQTT and wi-fi setup
#ifdef POST
  #ifdef FALKOR
    char MQTTServer[] = "falkor.triumf.ca";
    long MQTTPort = 1883;
    char MQTTUsername[] = FALKORUSER;
    char MQTTPassword[] = FALKORPASSWORD;
  #else
    //ThingSpeak
    char MQTTServer[] = "mqtt.thingspeak.com";
    long MQTTPort = 1883;//NOTE 1883 without SSL, 8883 with SSL
    char MQTTUsername[] = TSWRITEAPIKEY;//TS channel write API key
    long MQTTUsername = TSCHANNELID;
  #endif
  //NOTE need a random client ID for posting
  static const char alphanum[] ="0123456789"
                                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                "abcdefghijklmnopqrstuvwxyz";
  WiFiClient WiFiclient;//initialize the wifi client library
  PubSubClient MQTTClient(WiFiclient);//initialize PuBSubClient library
#endif //POST

//******************************************
void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Serial.println();

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

  //read values before the ESP8266 heats up
  float ta = std::numeric_limits<double>::quiet_NaN();
  float rha = std::numeric_limits<double>::quiet_NaN();
  float dpa = std::numeric_limits<double>::quiet_NaN();
  float tb = std::numeric_limits<double>::quiet_NaN();
  float rhb = std::numeric_limits<double>::quiet_NaN();
  float dpb = std::numeric_limits<double>::quiet_NaN();
  float dp = std::numeric_limits<double>::quiet_NaN();
  float dt = std::numeric_limits<double>::quiet_NaN();
  #ifdef SHT35A
  ta = sht31a.readTemperature();
  rha = sht31a.readHumidity();
  #endif
  #ifdef SHT35B
  tb = sht31b.readTemperature();
  rhb = sht31b.readHumidity();
  #endif
  #ifdef SDP610
  dp = SDP6x.GetPressureDiff();
  #endif

  #if defined(SHT35A) && defined(SHT35B)
  dt = ta-tb;
  #endif

  //print serial
  #ifdef PRINTSERIAL
  printSerial(ta, rha, tb, rhb, dp, dt);
  #endif
  
  //post data online
  #ifdef POST
  MQTTClient.setServer(MQTTServer, MQTTPort);
  postData(ta, rha, tb, rhb, dp, dt);
  #endif

  //deep sleep
  ESP.deepSleep(SLEEPTIME*1e6);//µs
}

//******************************************
//loop() is empty since the ESP8266 is sent to deep sleep at the end of setup()
void loop(){}

//******************************************
//get dew point
float getDewPoint(float t, float rh){
  if (! isnan(t) and ! isnan(rh)){
    float h = (log10(rh)-2)/0.4343 + (17.62*t)/(243.12+t);
    return 243.12*h/(17.62-h);
  }
  else return std::numeric_limits<double>::quiet_NaN();
}

//******************************************
//print measurements to serial output
void printSerial(float ta, float rha, float tb, float rhb, float dp, float dt){
  /*
  Serial.println();
  Serial.println("sleep, measure, post, repeat");
  #ifdef SHT35A
  Serial.print(" t_a[C] RH_a[%]");
  #endif
  #ifdef SHT35A
  Serial.print(" t_b[C] RH_b[%]");
  #endif
  #ifdef SDP610
  Serial.print(" dP[Pa]");
  #endif
  #if defined(SHT35A) && defined(SHT35B)
  Serial.print(" dt[C]");
  #endif
  Serial.println();
  */
  
  //print measurements to serial output
  #ifdef SHT35A
  Serial.print(ta);//temperature
  Serial.print(" ");
  Serial.print(rha);//humidity
  Serial.print(" ");
  Serial.print(getDewPoint(ta,rha));//dew point
  Serial.print(" ");
  #endif

  #ifdef SHT35B
  Serial.print(tb);//temperature
  Serial.print(" ");
  Serial.print(rhb);//humidity
  Serial.print(" ");
  Serial.print(getDewPoint(tb,rhb));//dew point
  Serial.print(" ");
  #endif

  #ifdef SDP610
  Serial.print(dp);//differential pressure
  Serial.print(" ");
  #endif

  #if defined(SHT35A) && defined(SHT35B)
  Serial.print(dt);//temperature difference
  Serial.print(" ");
  #endif

  Serial.println();
}

//******************************************
//post data online using MQTT protocol
#ifdef POST
void postData(float ta, float rha, float tb, float rhb, float dp, float dt){

  //------------------------------------------
  //connect to the wifi
  //https://github.com/esp8266/Arduino/issues/2702
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  //try connecting for ~20 seconds
  for (int ii=0;ii<20;ii++){

    //status
    #ifdef VERBOSE
    Serial.println();
    Serial.print("wifi connecting to ");
    Serial.println(ssid);
    
    Serial.print("status: ");
    switch (WiFi.status()){
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
        break;
      default:
        Serial.println("unknown");
        break;
    }
    #endif
    
    if (WiFi.status() == WL_CONNECTED) break;
    delay(1000);
  }

  //------------------------------------------
  if (WiFi.status() == WL_CONNECTED){

    #ifdef VERBOSE
    Serial.print("wifi connected to ");
    Serial.println(ssid);
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    #endif
    
    //------------------------------------------
    //post data using MQTT protocol
    //connect MQTT
    MQTTConnect();
    if (MQTTClient.connected()){
      //call the loop continuously to establish connection to the server
      MQTTClient.loop();
      //publish data to ThingSpeak
      MQTTPublish(ta, rha, tb, rhb, dp, dt);
      //disconnect
      MQTTClient.disconnect();
    }
  }
  //------------------------------------------
  else {
    #ifdef VERBOSE
    Serial.print("could not connect to ");
    Serial.println(ssid);
    Serial.println("abort");
    Serial.println();
    #endif
  }

  //------------------------------------------
  //disconnect before leaving
  WiFi.disconnect();
}
#endif //POST

//******************************************
#ifdef POST
void MQTTConnect(){

  //show status
  #ifdef VERBOSE
  Serial.print("connecting to MQTT server ");
  Serial.print(MQTTServer);
  Serial.println("...");
  #endif

  //random clientID
  char clientID[10];
  
  //try connecting for 5 times
  //using 2 second intervals
  for (int ii=0;ii<5;ii++){

    //generate random clientID
    for (int i = 0; i<10; i++) {
        clientID[i] = alphanum[random(51)];
    }

    //https://community.thingspeak.com/forum/esp8266-wi-fi/problem-rc-4-using-library-pubsub/
    clientID[10]='\0';

    //connect
    MQTTClient.connect(clientID, MQTTUsername, MQTTPassword);
    
    //connection status
    //print to know why the connection failed
    //see http://pubsubclient.knolleary.net/api.html#state for the failure code explanation
    #ifdef VERBOSE
    Serial.print("status: ");
    
    switch (MQTTClient.state()){
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
    if (MQTTClient.connected()){
      #ifdef VERBOSE
      Serial.print("MQTT connected to ");
      Serial.println(MQTTServer);
      #endif
      break;
    }
    else delay(2000);
  }//END connecting loop
}
#endif //POST

//******************************************
#ifdef POST
void MQTTPublish(float ta, float rha, float tb, float rhb, float dp, float dt){

  //print
  #ifdef VERBOSE
  Serial.println("posting data");
  #endif

  //create data string to send to MQTT broker
  #ifdef FALKOR
    //falkor
    #if defined(SHT35A) and defined(SHT35B)
      String data = String("{"+
                         "[{" +
                         "\"temperature\":" + String(ta) +
                         ",\"relativehumidity\":" + String(rha) +
                         //",\"dewpoint\":" + String(getDewPoint(ta,rha)) +
                         "\"address\":\"A\""}]" +
                         "}, {" +
                         \"temperature\":" + String(tb) +
                         ",\"relativehumidity\":" + String(rhb) +
                         //",\"dewpoint\":" + String(getDewPoint(tb,rhb)) +
                         "\"address\":\"B\""}]" +
                         ",\"temperaturedifference\":" + String(dt));
      #ifdef SDP610
        data += String(",\"differentialpressure\":" + String(dp));
      #endif
      data+=String("}");
    #elif defined(SHT35A)
      String data = String("{\"temperature\":" + String(ta));
      data += String(",\"relativehumidity\":" + String(rha));
      //data += String(",\"dewpoint\":" + String(getDewPoint(ta,rha)));
      #ifdef SDP610
        data += String(",\"differentialpressure\":" + String(dp));
      #endif
      data+=String("}");
    #elif defined(SHT35B)
      String data = String("{\"temperature\":" + String(tb));
      data += String(",\"relativehumidity\":" + String(rhb));
      //data += String(",\"dewpoint\":" + String(getDewPoint(tb,rhb)));
      #ifdef SDP610
        data += String(",\"differentialpressure\":" + String(dp));
      #endif
      data+=String("}");
    #endif
  #else
    //ThingSpeak
    #if defined(SHT35A) and defined(SHT35B)
    String data = String("field1="  + String(ta) +
                         "&field2=" + String(rha) +
                         //"&field3=" + String(getDewPoint(ta,rha)) +
                         "&field4=" + String(dp)+
                         "&field5=" + String(tb) +
                         "&field6=" + String(rhb) +
                         //"&field7=" + String(getDewPoint(tb,rhb)) +
                         "&field8=" + String(dt));         
    #elif defined(SHT35A)
    String data = String("field1="  + String(ta) +
                         "&field2=" + String(rha) +
                         "&field3=" + String(getDewPoint(ta,rha)) +
                         "&field4=" + String(dp));
    #elif defined(SHT35B)
    String data = String("field1="  + String(tb) +
                         "&field2=" + String(rhb) +
                         "&field3=" + String(getDewPoint(tb,rhb)) +
                         "&field4=" + String(dp));
    #endif
  #endif
  
  int length = data.length();
  char msgBuffer[length];
  data.toCharArray(msgBuffer,length+1);
  #ifdef VERBOSE
  Serial.println(msgBuffer);
  #endif

  //create a topic string and publish data to channel feed
  #ifdef FALKOR
  //falkor
  String topicString ="test";
  #else
  //ThingSpeak
  String topicString ="channels/" + String(channelID) + "/publish/"+String(MQTTPassword);
  #endif
  length=topicString.length();
  char topicBuffer[length];
  topicString.toCharArray(topicBuffer,length+1);
  #ifdef VERBOSE
  Serial.println(topicString);
  #endif
  
  //publish
  #ifndef QUIET
    if (MQTTClient.publish(topicBuffer, msgBuffer)) {
      #ifdef VERBOSE
      Serial.println("success");
      #endif
    } else {
      #ifdef VERBOSE
      Serial.println("fail");
      #endif
    }
  Serial.println();
  #endif
}
#endif //POST

