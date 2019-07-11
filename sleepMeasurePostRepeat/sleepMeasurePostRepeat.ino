/* 
  This sketch reads the ambient temperature, humidity and differential pressure, prints them to serial output, posts them online and then goes to sleep.
  The values measured are posted through MQTT.
  Temperature and humidity can be read from two different sensors.
  Don't forget to connect pin 16 with the reset pin.
  Printing and posting can be enabled/disabled by uncommenting/commenting the relative options in the definitions section below.
  To enable/disable a sensor, uncomment/comment its name in the definitions section below.
  Author: Guescio.
*/

//******************************************
//definitions
#define SHTA //0x44 address //SHT35A or SHT85
#define SHTB //0x45 address //SHT35B
#define SDP610
#define SLEEPTIME (30)//s
#define PRINTSERIAL //print measurements to serial output
#define POST //connect and post measurements online
//#define QUIET //connect to broker but do not post
#define VERBOSE //print connection status and posting details
//#define LOCATION ("location") //location of the measurement device

//******************************************
#if defined(SHTA) || defined(SHTB)
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

//SDP610 does not need initialization

//******************************************
//

#ifdef POST
  const char* ssid     = WIFISSID;
  const char* password = WIFIPASSWORD;
#endif

//******************************************
//MQTT and wi-fi setup
#ifdef POST
  char MQTTServer[] = MQTTSERVER;
  long MQTTPort = 1883;
  char MQTTUsername[] = MQTTUSER;
  char MQTTPassword[] = MQTTPASSWORD;
  //NOTE need a random client ID for posting
  static const char alphanum[] ="0123456789"
                                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                "abcdefghijklmnopqrstuvwxyz";
  WiFiClient WiFiclient;//initialize the wifi client library
  PubSubClient MQTTClient(WiFiclient);//initialize PuBSubClient library
#endif //POST

//******************************************
//setup
void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Serial.println();

  //initialize SHTs
  #ifdef SHTA
    sht31a.begin(SHTAADDR);
  #endif
  
  #ifdef SHTB
    sht31b.begin(SHTBADDR);
  #endif

  //initialize SDP610
  #ifdef SDP610
    Wire.begin();
    Wire.setClockStretchLimit(1e4);//µs
  #endif

  //set initial values
  float ta = std::numeric_limits<double>::quiet_NaN();//temeprature A
  float rha = std::numeric_limits<double>::quiet_NaN();//relative humidity A
  float dpa = std::numeric_limits<double>::quiet_NaN();//dew point A
  float tb = std::numeric_limits<double>::quiet_NaN();//temeprature B
  float rhb = std::numeric_limits<double>::quiet_NaN();//relative humidity B
  float dpb = std::numeric_limits<double>::quiet_NaN();//dew point B
  float dp = std::numeric_limits<double>::quiet_NaN();//differential pressure
  float dt = std::numeric_limits<double>::quiet_NaN();//temperature difference
  
  //read values before the ESP8266 heats up
  #ifdef SHTA
    ta = sht31a.readTemperature();
    rha = sht31a.readHumidity();
  #endif //SHTA
  
  #ifdef SHTB
    tb = sht31b.readTemperature();
    rhb = sht31b.readHumidity();
  #endif //SHTB
  
  #ifdef SDP610
    dp = SDP6x.GetPressureDiff();
  #endif //SDP610

  #if defined(SHTA) && defined(SHTB)
    dt = ta-tb;
  #endif //SHTA && SHTB

  //print serial
  #ifdef PRINTSERIAL
    printSerial(ta, rha, tb, rhb, dp, dt);
  #endif //PRINTSERIAL
  
  //connect to wifi and post data
  #ifdef POST
    //set MQTT server
    MQTTClient.setServer(MQTTServer, MQTTPort);
  
    //connect to wifi
    wifiConnect();
  
    //post data
    if (WiFi.status() == WL_CONNECTED){
      postData(ta, rha, tb, rhb, dp, dt);
    }
  
    //disconnect before leaving
    WiFi.disconnect();
  #endif //POST

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

  Serial.println();
  Serial.println("sleep, measure, post, repeat");
  Serial.println();
  #ifdef SHTA
    Serial.print("t_a[C] RH_a[%] DP_a[C]");
  #endif
  #ifdef SHTB
    Serial.print(" t_b[C] RH_b[%] DP_b[C]");
  #endif
  #ifdef SDP610
    Serial.print(" dP[Pa]");
  #endif
  #if defined(SHTA) && defined(SHTB)
    Serial.print(" dt[C]");
  #endif
  Serial.println();
  
  //print measurements to serial output
  #ifdef SHTA
    Serial.print(ta);//temperature
    Serial.print("  ");
    Serial.print(rha);//humidity
    Serial.print("   ");
    Serial.print(getDewPoint(ta,rha));//dew point
    Serial.print("   ");
  #endif

  #ifdef SHTB
    Serial.print(tb);//temperature
    Serial.print("  ");
    Serial.print(rhb);//humidity
    Serial.print("   ");
    Serial.print(getDewPoint(tb,rhb));//dew point
    Serial.print("   ");
  #endif

  #ifdef SDP610
    Serial.print(dp);//differential pressure
    Serial.print("  ");
  #endif

  #if defined(SHTA) && defined(SHTB)
    Serial.print(dt);//temperature difference
  #endif

  Serial.println();
}

//******************************************
//connect to wifi
//try connecting for 20 times with 1 s intervals
#ifdef POST
void wifiConnect(){

  //https://github.com/esp8266/Arduino/issues/2702
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.print("wifi connecting to ");
  Serial.println(ssid);

  //connect
  for (int ii=0;ii<20;ii++){

    //status
    #ifdef VERBOSE
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
        delay(1000);//ms
        break;
      default:
        Serial.println("unknown");
        break;
    }
    #endif //VERBOSE
    
    if (WiFi.status() == WL_CONNECTED){
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
    if (WiFi.status() != WL_CONNECTED){
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
void postData(float ta, float rha, float tb, float rhb, float dp, float dt){
  //connect to MQTT broker
  MQTTConnect();
  
  if (MQTTClient.connected()){
    //call the loop continuously to establish connection to the server
    MQTTClient.loop();
    
    //publish data
    MQTTPublish(ta, rha, tb, rhb, dp, dt);
    
    //disconnect
    MQTTClient.disconnect();
  }
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
  
  //try connecting for 5 times with 2 s intervals
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

    //delay between trials
    delay(2000);//ms
  }//END connecting loop
}
#endif //POST

//******************************************
//string to post with T, RH and dew point
String getStringToPost(float t, float rh){
      String data = String("");

      //temperature
      data += String("\"temperature\":" + String(t));
      
      //relative humidity
      data += String(",\"relativehumidity\":" + String(rh));
      
      //dew point
      if ( ! isnan(getDewPoint(t,rh))){
        data += String(",\"dewpoint\":" + String(getDewPoint(t,rh)));
        if (t > getDewPoint(t,rh)) data += String(",\"dewpointstatus\":1");
        else data += String(",\"dewpointstatus\":0");
      }
      else {
        data += String(",\"dewpoint\":\"NaN\"");
        data += String(",\"dewpointstatus\":\"NaN\"");
      }
      
      return data;
}

//******************************************
#ifdef POST
void MQTTPublish(float ta, float rha, float tb, float rhb, float dp, float dt){

  //print
  #ifdef VERBOSE
    Serial.println("posting data");
  #endif

  //create data string to send to MQTT broker
  //SHTA and SHTB
  #if defined(SHTA) and defined(SHTB)
    String data = String("");
    data += String("{[{");
    data += getStringToPost(ta,rha);
    data += String(",\"address\":\"A\"}, {");
    data += getStringToPost(tb,rhb);
    data += String(",\"address\":\"B\"}]");
    data += String(",\"temperaturedifference\":" + String(dt));
    #ifdef SDP610
      data += String(",\"differentialpressure\":" + String(dp));
    #endif //SDP610
    data += String("}");

  //SHTA
  #elif defined(SHTA)
    String data = ("{");
    data += getStringToPost(ta,rha);
    #ifdef SDP610
      data += String(",\"differentialpressure\":" + String(dp));
    #endif //SDP610
    data += String(",\"address\":\"A\"");
    #ifdef LOCATION
      data += String(",\"location\":\"");
      data += LOCATION;
      data += String("\"");
    #endif //LOCATION
    data += String("}");

  //SHTB
  #elif defined(SHTB)
    String data = ("{");
    data += getStringToPost(tb,rhb);
    #ifdef SDP610
      data += String(",\"differentialpressure\":" + String(dp));
    #endif //SDP610
    data += String(",\"address\":\"B\"");
    #ifdef LOCATION
      data += String(",\"location\":\"");
      data += LOCATION;
      data += String("\"");
    #endif //LOCATION
    data += String("}");
  
  #endif //SHTA and/or SHTB definition

  //convert message string to char array
  int length = data.length();
  char msgBuffer[length];
  data.toCharArray(msgBuffer,length+1);

  //print message
  #ifdef VERBOSE
    Serial.print("message: ");
    Serial.println(msgBuffer);
  #endif

  //create topic string
  String topicString = MQTTTOPIC;
  
  //append location to topic
  #ifdef LOCATION
    topicString += "/";
    topicString += LOCATION;
  #endif //LOCATION
  
  length=topicString.length();
  char topicBuffer[length];
  topicString.toCharArray(topicBuffer,length+1);
  #ifdef VERBOSE
    Serial.print("topic: ");
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

