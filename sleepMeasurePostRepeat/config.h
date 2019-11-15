//******************************************
//definitions
#define CAFFEINE //do not go to deep sleep
//#define SHTA //SHT35A or SHT85 (0x44 address)
#define SHTB //SHT35B (0x45 address)
//#define SPS30
//#define SDP610
//#define ADC//NOTE: set also the voltage divider resistor value
#define ADCRDIV (1e4)//ADC voltage resistor value [Ohm] //84.5e3
#define NTC//NOTE: set also the voltage divider resistor value and NTC R0, T0 and B values
#define NTCR0 (2.2e3)//NTC R0 [Ohm] //1e4
#define NTCT0 (298.15)//NTC T0 [K]
#define NTCB (4000)//NTC B //3435
#define ADCVREF (1.0)//ADC reference voltage [V]
#define VDD (3.3)//voltage supply [V]
#define SLEEPTIME (10)//s
#define PRINTSERIAL //print measurements to serial output
//#define POST //connect and post measurements online
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
#define ESP32 //use ESP32 instead of ESP8266

