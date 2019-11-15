//******************************************
//setup
#define CAFFEINE //do not go to sleep
#define ESP32 //use an ESP32 board instead of an ESP8266 board
#define SLEEPTIME (10)//s
#define PRINTSERIAL //print measurements to serial output
//#define POST //connect and post measurements online
//#define QUIET //connect to broker but do not post
//#define VERBOSE //print connection status and posting details

//******************************************
//temperature and humidity sensors
//#define SHTA //SHT35A or SHT85 (0x44 address)
#define SHTB //SHT35B (0x45 address)

//******************************************
//dust particle sensors
//#define SPS30

//******************************************
//pressure sensors
//#define SDP610

//******************************************
//ADCs and NTCs
//NOTE: ESP8266 supports only ADC0, ESP32 supports ADC0-4
#define VDD (3.3)//voltage supply [V]
#define ADCVREF (1.0)//ADC reference voltage [V]

//------------------------------------------
//ADC0/NTC0
//#define ADC0//NOTE: set also the voltage divider resistor value
#define ADC0RDIV (84.5e3)//ADC voltage resistor value [Ohm] //84.5e3
//#define NTC0//NOTE: set also the voltage divider resistor value and NTC R0, T0 and B values
#define NTC0R0 (1e4)//NTC R0 [Ohm] //1e4
#define NTC0T0 (298.15)//NTC T0 [K] //298.15
#define NTC0B (3435)//NTC B //3435

//------------------------------------------
//ADC1/NTC1
//#define ADC1
#define ADC1RDIV (10e3)
#define NTC1
#define NTC1R0 (1e3)
#define NTC1T0 (298.15)
#define NTC1B (4000)

//------------------------------------------
//ADC2/NCT2
//#define ADC2
#define ADC2RDIV (10e3)
#define NTC2
#define NTC2R0 (2.2e3)
#define NTC2T0 (298.15)
#define NTC2B (4000)

//------------------------------------------
//ADC3/NTC3
//#define ADC3
#define ADC3RDIV (20e3)
#define NTC3
#define NTC3R0 (4.7e3)
#define NTC3T0 (298.15)
#define NTC3B (3650)

//------------------------------------------
//ADC4/NTC4
//#define ADC4
#define ADC4RDIV (51e3)
#define NTC4
#define NTC4R0 (22e3)
#define NTC4T0 (298.15)
#define NTC4B (4000)

//******************************************
//wifi
//#define WIFISSID ("ssid")
//#define WIFIPASSWORD ("password")

//******************************************
//MQTT
//#define MQTTSERVER ("127.0.0.1")
//#define MQTTUSER ("user")
//#define MQTTPASSWORD ("password")
//#define MQTTTOPIC ("topic") //MQTT topic
//#define INSTITUTE ("institute") //institute of the measurement device //NOTE this is used also for the MQTT topic
//#define ROOM("room") //room of the measurement device
//#define LOCATION ("location") //location of the measurement device
//#define NAME("name") //name of the measurement device

