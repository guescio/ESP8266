/*
  This sketch iteratively sends the ESP8266 to deep sleep and then wakes it up.
  Autor: Guescio.
*/

void setup() {
  Serial.begin(115200);
  while(!Serial){}
  
  Serial.println();
  Serial.println("I'm awake! Zzz...");
  ESP.deepSleep(60e6);//µs
}

//loop() is empty since the ESP8266 is sent to deep sleep at the end of setup()
void loop() {}

