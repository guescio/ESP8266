/*
  This sketch tests the wifi connection of the ESP8266 module by connecting to a web host and requesting a web page.
  This is based on the Adafruit example available at:
  https://learn.adafruit.com/adafruit-huzzah-esp8266-breakout/using-arduino-ide
  Minor modifications by Guescio.
*/

//******************************************
//definitions
//delay between requests
#define DELAY (5000)//ms

//******************************************
#include <ESP8266WiFi.h>
#include <guescio.h>

//******************************************
//wifi
const char* ssid     = WIFISSID;
const char* password = WIFIPASSWORD;

//webhost
const char* host = "wifitest.adafruit.com";

//******************************************
void setup() {
  Serial.begin(115200);
  delay(100);

  //print MAC address
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  //connect to wifi network
  Serial.print("connecting to ");
  Serial.println(ssid);
  //https://github.com/esp8266/Arduino/issues/2702
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //print info
  Serial.println("");
  Serial.println("wifi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("netmask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
}

//******************************************
int value = 0;

//******************************************
void loop() {
  delay(DELAY);
  ++value;

  Serial.println();
  Serial.println();
  Serial.print("connecting to ");
  Serial.println(host);
  
  //use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    return;
  }
  
  //create URI for the request
  String url = "/testwifi/index.html";
  Serial.print("requesting URL: ");
  Serial.println(url);
  
  //send request to the server
  Serial.println();
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "host: " + host + "\r\n" + 
               "connection: close\r\n\r\n");
  delay(500);
  
  //read reply from server and print it to serial
  while(client.available()){
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
  
  Serial.println();
  Serial.println();
  Serial.println("closing connection");
}
