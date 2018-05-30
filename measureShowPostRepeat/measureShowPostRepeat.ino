/* 
  This sketch reads the ambient temperature and humidity and monitors the battery status.
  The values measured are shown on the OLED screen and posted on ThingSpeak through MQTT.
  Author: Guescio.
*/

//******************************************
//definitions
#define SCREENINTERVAL (10000)//ms
#define POSTINTERVAL (60000)//ms
#define POST //post data online
#define PRINTSERIAL //print measurements to serial output
#define VERBOSE //print connection status and posting details

//******************************************
#include <ESP8266WiFi.h>
#include "Adafruit_SHT31.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <limits>
#include <elapsedMillis.h>
#include <PubSubClient.h>
#include <guescio.h>

//******************************************
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SSD1306 display = Adafruit_SSD1306();

//******************************************
#if defined(ESP8266)
  #define BUTTON_A 0
  #define BUTTON_B 16
  #define BUTTON_C 2
#endif

//******************************************
#if (SSD1306_LCDHEIGHT != 32)
 #error("screen height incorrect, please fix Adafruit_SSD1306.h");
#endif

//******************************************
//wi-fi
const char* ssid     = WIFISSID;
const char* password = WIFIPASSWORD;

//******************************************
//ThingSpeak MQTT
#ifdef POST
  char MQTTServer[] = "mqtt.thingspeak.com";
  long MQTTPort = 1883;//NOTE 1883 without SSL, 8883 with SSL
  char TSChannelWriteAPIKey[] = TSWRITEAPIKEY;//TS channel write API key
  long channelID = TSCHANNELID;
  WiFiClient WiFiclient;//initialize the wifi client library
  PubSubClient MQTTClient(WiFiclient);//initialize PuBSubClient library
  //NOTE need a random client ID for posting
  static const char alphanum[] ="0123456789"
                                "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                "abcdefghijklmnopqrstuvwxyz";
#endif //POST

//******************************************
//time keeper
elapsedMillis screenTimeElapsed;
elapsedMillis postTimeElapsed;

//voltage divider resistors
const float r1 = 10e3;//Ohm
const float r2 = 51e3;//Ohm

//ADC
const float adcmin=457;
const float adcmax=621;

//buttons and screens
bool buttonPressed=false;
char screen='a';
int acount=1;
bool toggleWiFi=true;

//******************************************
void setup(){
  Serial.begin(115200);
  while(!Serial){}

  //initialize OLED (address 0x3C)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  //print
  #ifdef PRINTSERIAL
    Serial.println("mTRHV v. 0.7");
    Serial.println("t[C] RH[%] DP[C] V[V] charge[%] ADC");
  #endif
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  display.println("mTRHV");
  display.setTextSize(1);
  display.println("");
  display.println("v. 0.7");
  display.display();
  
  //initialize SHT31 (address 0x44)
  //set to 0x45 for alternate i2c address
  sht31.begin(0x44);

  //set buttons
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  //disconnect wifi
  //https://github.com/esp8266/Arduino/issues/2702
  WiFi.disconnect();

  //set MQTT server
  #ifdef MQTT
    MQTTClient.setServer(MQTTServer, MQTTPort);
  #endif MQTT
  
  delay(1000);
}

//******************************************
//get voltage
float getVoltage(){
  //read raw analog value form ADC
  int raw = analogRead(A0);
  //convert ADC to voltage
  return (float) raw/1023. * (r1+r2)/r1;
}

//******************************************
//get battery charge status
int getBatteryStatus(){
  //read raw analog value form ADC
  int raw = analogRead(A0);
  //convert ADC to percent
  int level = map(raw, adcmin, adcmax, 0, 100);
  return level < 150 ? level : 100;
}

//******************************************
//get dew point - no T nor RH
float getDewPoint(){
  float t = sht31.readTemperature();
  float rh = sht31.readHumidity();
  return getDewPoint(t,rh);
}

//******************************************
//get dew point - given T and RH
float getDewPoint(float t, float rh){
  if (! isnan(t) and ! isnan(rh)){
    float h = (log10(rh)-2)/0.4343 + (17.62*t)/(243.12+t);
    return 243.12*h/(17.62-h);
  }
  else return std::numeric_limits<double>::quiet_NaN();
}

//******************************************
//print measurements to serial output
void printSerial(){
  
  //temperature
  float t = sht31.readTemperature();
  Serial.print(t);
  Serial.print(" ");

  //humidity
  float rh = sht31.readHumidity();
  Serial.print(rh);
  Serial.print(" ");

  //dew point
  Serial.print(getDewPoint(t,rh));
  Serial.print(" ");

  //voltage
  Serial.print(getVoltage());
  Serial.print(" ");

  //battery charge status
  Serial.print(getBatteryStatus());
  Serial.print(" ");
  
  //ADC
  Serial.print(analogRead(A0));

  //finally print line
  Serial.println();
}

//******************************************
//screen A0: print T, RH, V, %
void printScreenA0(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  
  //temperature
  float t = sht31.readTemperature();
  display.print(t);
  display.println(" C");

  //humidity
  float rh = sht31.readHumidity();
  display.print(rh);
  display.print("% RH  ");

  //dew point
  display.print(getDewPoint(t,rh));
  display.println(" C DP");
  
  //voltage
  display.print(getVoltage());
  display.print(" V     ");

  //battery charge status
  display.print(getBatteryStatus());
  display.println("%");

  //ADC
  display.print(analogRead(A0));
  display.print(" ADC");

  //actually print
  display.display();
}

//******************************************
//screen A1: print T and RH
void printScreenA1(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  
  //temperature
  display.print(sht31.readTemperature());
  display.println(" C");

  //humidity
  display.print(sht31.readHumidity());
  display.println("% RH");

  display.display();
}

//******************************************
//screen A2: print T and DP
void printScreenA2(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  
  //temperature
  float t = sht31.readTemperature();
  display.print(t);
  display.println(" C");

  //dew point
  float rh = sht31.readHumidity();
  display.print(getDewPoint(t,rh));
  display.println(" C DP");
  
  display.display();
}

//******************************************
//screen B0: print voltage and battery charge status
void printScreenB0(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);
  
  //voltage
  display.print(getVoltage());
  display.println(" V");

  //battery charge status
  display.print(getBatteryStatus());
  display.println("%");
  
  display.display();
}

//******************************************
//screen C0: wi-fi
void printScreenC0(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(2);

  //turn on/off
  display.print("wifi: ");
  if (toggleWiFi){
    display.println("on");
    //if wifi is off, turn it on
  } else {
    display.println("off");
    //if wifi is on, turn it off
  }

  display.setTextSize(1);
  display.println("SSID: ");
  display.println(ssid);
  display.display();
}

//******************************************
//post data online using MQTT protocol
#ifdef POST
void postDataOnline(float t, float rh){

  //------------------------------------------
  //connect to the wifi
  //https://github.com/esp8266/Arduino/issues/2702
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  //try connecting for ~20 seconds
  for (int ii=0;ii<20;ii++){

    //show status
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.println("wifi connecting to");
    display.println(ssid);

    display.print("status: ");
    switch (WiFi.status()){
      case 0:
        display.println("idle");
        break;
      case 1:
        display.println("no SSID");
        break;
      case 2:
        display.println("scan compl.");
        break;
      case 3:
        display.println("connected");
        break;
      case 4:
        display.println("connection failed");
        break;
      case 5:
        display.println("connection lost");
        break;
      case 6:
        display.println("disconnected");
        break;
      default:
        display.println("unknown");
        break;
    }

    //connection progress
    for (int jj=0;jj<ii;jj++){
      display.print(".");
    }
    display.println();
    display.display();

    if (WiFi.status() == WL_CONNECTED) break;
    delay(1000);
  }

  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);

  //------------------------------------------
  if (WiFi.status() == WL_CONNECTED){
  
    display.println("wifi connected to");
    display.println(ssid);
    display.print("IP: ");
    display.println(WiFi.localIP());
    //display.println("posting data");
    display.display();
    delay(1000);
    
    //------------------------------------------
    //post data using MQTT protocol
    //connect MQTT
    MQTTConnect();
    if (MQTTClient.connected()){
      //call the loop continuously to establish connection to the server
      MQTTClient.loop();
      //publish data to ThingSpeak
      MQTTPublish(t, rh);
      //disconnect
      MQTTClient.disconnect();
    }
  }
  //------------------------------------------
  else {
    //toggleWiFi=false;
    display.println("could not connect to");
    display.println(ssid);
    display.println("abort");
    display.display();
    delay(1000);
  }

  //------------------------------------------
  //disconnect before leaving
  WiFi.disconnect();
}
#endif //POST

//******************************************
void loop(){

  if (! digitalRead(BUTTON_A)){
    if (screen=='a') acount++;
    acount=acount%3;
    screen='a';
    buttonPressed=true;
  }

  if (! digitalRead(BUTTON_B)){
    screen='b';
    buttonPressed=true;
  }

  if (! digitalRead(BUTTON_C)){
    toggleWiFi= screen=='c'? !toggleWiFi : toggleWiFi;
    screen='c';
    buttonPressed=true;
  }

  //update screen
  if(screenTimeElapsed > SCREENINTERVAL or buttonPressed){

    //print serial
    #ifdef PRINTSERIAL
      printSerial();
    #endif

    //print on screen
    switch (screen){
    
      //screen A
      case 'a':
        switch (acount){
          case 0:
            printScreenA0();
            break;
          case 1:
            printScreenA1();
            break;
          case 2:
            printScreenA2();
            break;
          default:
            printScreenA0();
            break;
        }
        break;
    
      //screen B
      case 'b':
        printScreenB0();
        break;

      //screen C
      case 'c':
        printScreenC0();
        screen='c';
        break;

      //default: screen A1
      default:
        printScreenA1();
        break;
    }

    //delay to reduce double press
    delay(200);//ms

    //reset button flag and elapsed time clock
    buttonPressed=false;
    screenTimeElapsed=0;

    }

  //post data online
  if(postTimeElapsed > POSTINTERVAL && toggleWiFi){
    #ifdef POST
      //read temperature and RH before the ESP8266 heats up while connecting to the wifi
      postDataOnline(
        sht31.readTemperature(),
        sht31.readHumidity());
    #endif //POST
    //reset button flag and elapsed time clock
    postTimeElapsed=0;
  }

  //yield to background functions before long delays
  yield();
}

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

    //show status
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.println("MQTT connecting to");
    display.println(MQTTServer);

    //generate random clientID
    for (int i = 0; i<10; i++) {
        clientID[i] = alphanum[random(51)];
    }

    //connect
    MQTTClient.connect(clientID, TSUSERNAME, TSMQTTAPIKEY);
    
    //connection status
    //print to know why the connection failed
    //see http://pubsubclient.knolleary.net/api.html#state for the failure code explanation
    display.print("status: ");
    #ifdef VERBOSE
      Serial.print("\tstatus: ");
    #endif
    
    switch (MQTTClient.state()){
      case -4:
        display.println("timeout");
        #ifdef VERBOSE
          Serial.println("timeout");
        #endif
        break;
      case -3:
        display.println("lost");
        #ifdef VERBOSE
          Serial.println("lost");
        #endif
        break;
      case -2:
        display.println("failed");
        #ifdef VERBOSE
          Serial.println("failed");
        #endif
        break;
      case -1:
        display.println("disconnected");
        #ifdef VERBOSE
          Serial.println("disconnected");
        #endif
        break;
      case 0:
        display.println("connected");
        #ifdef VERBOSE
          Serial.println("connected");
        #endif
        break;
      case 1:
        display.println("bad protocol");
        #ifdef VERBOSE
          Serial.println("bad protocol");
        #endif
        break;
      case 2:
        display.println("bad client ID");
        #ifdef VERBOSE
          Serial.println("bad client ID");
        #endif
        break;
      case 3:
        display.println("unavailable");
        #ifdef VERBOSE
          Serial.println("unavailable");
        #endif
        break;
      case 4:
        display.println("bad credentials");
        #ifdef VERBOSE
          Serial.println("bad credentials");
        #endif
        break;
      case 5:
        display.println("unauthorized");
        #ifdef VERBOSE
          Serial.println("unauthorized");
        #endif
        break;
      default:
        display.println("unknown");
        #ifdef VERBOSE
          Serial.println("unknown");
        #endif
        break;
    }

    //connection progress
    for (int jj=0;jj<=ii;jj++){
      display.print(".");
    }
    display.println();
    display.display();

    //upon successful connection
    if (MQTTClient.connected()){
      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(1);
      display.println("MQTT connected to");
      display.println(MQTTServer);
      display.display();
      #ifdef VERBOSE
        Serial.println("connected");
      #endif
      break;
    }
    else delay(2000);
  }//END connecting loop
}
#endif //POST

//******************************************
#ifdef POST
void MQTTPublish(float t, float rh){

  //print
  #ifdef VERBOSE
    Serial.println("posting data");
  #endif
  display.println("posting data");
 
  //create data string to send to ThingSpeak
  String data = String("field1="  + String(t) +
                       "&field2=" + String(rh) +
                       "&field3=" + String(getDewPoint(t,rh)) +
                       "&field4=" + String(getVoltage()) +
                       "&field5=" + String(getBatteryStatus()) +
                       "&field6=" + String(analogRead(A0)));
  int length = data.length();
  char msgBuffer[length];
  data.toCharArray(msgBuffer,length+1);
  #ifdef VERBOSE
    Serial.println(msgBuffer);
  #endif

  //create a topic string and publish data to ThingSpeak channel feed
  String topicString ="channels/" + String( channelID ) + "/publish/"+String(TSChannelWriteAPIKey);
  length=topicString.length();
  char topicBuffer[length];
  topicString.toCharArray(topicBuffer,length+1);
  #ifdef VERBOSE
    Serial.println(topicString);
  #endif
  
  //publish
  if (MQTTClient.publish(topicBuffer, msgBuffer)) {
    display.println("success");
    display.display();
    #ifdef VERBOSE
      Serial.println("success");
    #endif
  } else {
    display.println("fail");
    display.display();
    #ifdef VERBOSE
      Serial.println("fail");
    #endif
  }
  delay(1000);
}
#endif //POST

