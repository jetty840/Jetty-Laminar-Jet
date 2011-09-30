// Copyright: Jetty, 30th September 2011
// License: License: Creative Commons Attribution-NonCommerical-ShareAlike
// http://creativecommons.org/licenses/by-nc-sa/3.0/
//
// Laminar Jet Control Software

#include <MemoryFree.h>
#include <WiServer.h>
#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <Statistic.h>

#define STEPPER
#define TEMPERATURE
#define BLINKM
#define WATER_LEVEL
#define PUMPS
#define THUMPER
#define WIND
#define WISERVER


//Pins in use
//D0 Adruino Serial
//D1 Arduino Serial
//D2 CUPPERHEAD WiShield INT0
//D3 Stepper 1
//D4 Stepper 1
//D5 OneWire Bus (Temp Sensors)
//D6 Stepper 2
//D7 Stepper 2  
//D8 Pumps
//D9 CUPPERHEAD WiShield successful wireless connection LED
//D10 CUPPERHEAD WiShield Slave Select (SS)                        Jumpered to D53
//D11 CUPPERHEAD WiShield Master Out, Slave In (MOSI)              Jumpered to D51
//D12 CUPPERHEAD WiShield Master In, Slave Out (MISO)              Jumpered to D50
//D13 CUPPERHEAD WiShield Clock (SCK)                              Jumpered to D52
//D48 Stepper Enable
//D50 2560 MEGA - CUPPERHEAD WiShield Master In, Slave Out (MISO)  Jumpered to D12
//D51 2560 MEGA - CUPPERHEAD WiShield Master Out, Slave In (MOSI)  Jumpered to D11
//D52 2560 MEGA - CUPPERHEAD WiShield Clock (SCK)                  Jumpered to D13
//D53 2560 MEGA - CUPPERHEAD WiShield Slave Select (SS)            Jumpered to D10

//A0 Thumper
//A1 Windspeed
//A2 Water Level 2
//A3 Water Level 1
//A4 BlinkM I2C
//A5 BlinkM I2C



//Constants for digital pins
#define D0   0
#define D1   1
#define D2   2
#define D3   3
#define D4   4
#define D5   5
#define D6   6
#define D7   7
#define D8   8
#define D9   9
#define D10  10
#define D11  11
#define D12  12
#define D13  13
#define D20  20
#define D21  21
#define D48  48





// ***** WISERVER CONFIRGUATION *****
#ifdef WISERVER

#define LAMINAR_INFRA  //Infrastructure or adhoc

#ifdef LAMINAR_INFRA
unsigned char local_ip[]    = {
  192,168,20,137}; // IP address of WiShield
unsigned char gateway_ip[]  = {
  192,168,20,1};   // router or gateway IP address
unsigned char subnet_mask[] = {
  255,255,255,0};  // subnet mask for the local network
char ssid[]                 = {
  "YOUR WIFI NETWORK SSID GOES HERE"};          // max 32 bytes
#else
unsigned char local_ip[]    = {
  169,254,74,137};	// IP address of WiShield
unsigned char gateway_ip[]  = {
  169,254,1,1};    // router or gateway IP address
unsigned char subnet_mask[] = {
  255,255,0,0};	// subnet mask for the local network
char ssid[]  = {
  "lam"};		                // max 32 bytes
#endif

#ifdef LAMINAR_INFRA
unsigned char security_type = 5;	// 0 - open; 1 - WEP; 2 - WPA; 3 - WPA2; 4 - WPA Precalc; 5 - WPA2 Precalc
#else
unsigned char security_type     = 0;
#endif

// Depending on your security_type, uncomment the appropriate type of security_data
// 0 - None (open)
//const prog_char security_data[] PROGMEM = {};

// 1 - WEP 
// UIP_WEP_KEY_LEN. 5 bytes for 64-bit key, 13 bytes for 128-bit key
// Only supply the appropriate key, do not specify 4 keys and then try to specify which to use
//const prog_char security_data[] PROGMEM = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, };

// 2, 3 - WPA/WPA2 Passphrase
// 8 to 63 characters which will be used to generate the 32 byte calculated key
// Expect the g2100 to take 30 seconds to calculate the key from a passphrase
//const prog_char security_data[] PROGMEM = {"12345678"};

// 4, 5 - WPA/WPA2 Precalc
// The 32 byte precalculate WPA/WPA2 key. This can be calculated in advance to save boot time
// http://jorisvr.nl/wpapsk.html
#error FILL OUT THE FOLLOWING INFORMATION IF REQUIRED
const prog_char security_data[] PROGMEM = {
  0x00, 0x000 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x000 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


// setup the wireless mode
// WIRELESS_MODE_INFRA - connect to AP
// WIRELESS_MODE_ADHOC - connect to another WiFi device
#ifdef LAMINAR_INFRA
unsigned char wireless_mode = WIRELESS_MODE_INFRA;
#else
unsigned char wireless_mode = WIRELESS_MODE_ADHOC;
#endif

#endif
// ***** END WISERVER CONFIRGUATION *****







// ***** BLINKM MAXM CONFIRGUATION *****
#ifdef BLINKM
#include "Wire.h"  //I2C
#include "BlinkM_funcs.h"

#define BLINKM_MAXM1  0x01
#define BLINKM_MAXM2  0x02

int activeScript;

// ***** END BLINKM MAXM CONFIRGUATION *****
#endif BLINKM




#ifdef STEPPER
// ***** STEPPER MOTOR CONFIGURATION *****
AccelStepper stepper1(2, 3, 4);  //Stepper motor, 2 pins, controlled by pins 3 & 4
AccelStepper stepper2(2, 6, 7);  //Stepper motor, 2 pins, controlled by pins 6 & 7

#define stepperDisablePin      D48

#define stepperFastSpeed   800.0   //Steps per second
#define stepperSlowSpeed   400.0    //Steps per second

#define stepper1CorrectionToCenter  17
#define stepper2CorrectionToCenter  17

#define stepper1Position1    -15
#define stepper1PositionCut  0
#define stepper1Position2    15

#define stepper1PositionFlash  -20

#define stepper2Position1    -15
#define stepper2PositionCut  0
#define stepper2Position2    15

boolean stepper1Cut           = true;
int stepper1Vibration         = 0;
int stepper1VibrationPosition = 0;

// ***** END STEPPER MOTOR CONFIGURATION *****
#endif





#ifdef TEMPERATURE
// ***** TEMPERATURE CONFIGURATION *****
#define ONE_WIRE_BUS           D5
#define TEMPERATURE_PRECISION  12

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature
DallasTemperature sensors(&oneWire);

// Device address of the temperature sensors so that we know which is which
DeviceAddress electronicsThermometer = { 0x28, 0xFF, 0x6E, 0x01, 0x03, 0x00, 0x00, 0x90 };
DeviceAddress outsideThermometer     = { 0x28, 0xC0, 0x9A, 0x01, 0x03, 0x00, 0x00, 0xE0 };
DeviceAddress stepper1Thermometer    = { 0x28, 0x1E, 0xA4, 0x01, 0x03, 0x00, 0x00, 0x11 };
DeviceAddress stepper2Thermometer    = { 0x28, 0x64, 0x7B, 0x01, 0x03, 0x00, 0x00, 0x77 };
DeviceAddress led1Thermometer        = { 0x28, 0x7C, 0x94, 0x01, 0x03, 0x00, 0x00, 0x38 };
DeviceAddress led2Thermometer        = { 0x28, 0x50, 0x74, 0x01, 0x03, 0x00, 0x00, 0x29 };
  
#define OUTSIDE_TEMP_LOWER_LIMIT     0
#define OUTSIDE_TEMP_UPPER_LIMIT     40
#define ELECTRONICS_TEMP_LOWER_LIMIT 0
#define ELECTRONICS_TEMP_UPPER_LIMIT 50
#define STEPPER_TEMP_LOWER_LIMIT     0
#define STEPPER_TEMP_UPPER_LIMIT     60
#define LED_TEMP_LOWER_LIMIT         0
#define LED_TEMP_UPPER_LIMIT         70


// ***** END TEMPERATURE CONFIGURATION *****
#endif




// ***** WATER LEVEL CONFIGURATION *****
#ifdef WATER_LEVEL
#define WATER_LEVEL1_PIN  A3
#define WATER_LEVEL2_PIN  A2

#define WATER_LEVEL1_ZERO_PERCENT  191
#define WATER_LEVEL1_100_PERCENT   555
#define WATER_LEVEL1_LOW_LIMIT     50.0    //Percent
#define WATER_LEVEL1_HIGH_LIMIT    110.0   //Percent

#define WATER_LEVEL2_ZERO_PERCENT  100
#define WATER_LEVEL2_100_PERCENT   300
#define WATER_LEVEL2_LOW_LIMIT     30.0    //Percent
#define WATER_LEVEL2_HIGH_LIMIT    130.0   //Percent
#endif
// ***** END WATER LEVEL CONFIGURATION *****





// ***** WIND CONFIGURATION *****
#ifdef WIND
#define WIND_PIN  A1
#endif
// ***** END WIND CONFIGURATION *****





// ***** PUMPS CONFIGURATION *****
#ifdef PUMPS
#define PUMPS_PIN  D8
#endif
// ***** END PUMPS CONFIGURATION *****





// ***** THUMPER CONFIGURATION *****
#ifdef THUMPER
#define THUMPER_PIN  A0
#endif
// ***** END THUMPER CONFIGURATION *****



#define PUMP_STARTUP_TIME    30  //Seconds
#define PUMP_SHUTDOWN_TIME   2  //Seconds

typedef enum _LaminarState
{
  LSTATE_TRANSITION_OFF = 0,             //Transition state to set everything off
  LSTATE_OFF,                            //Everything off (rest state)
  LSTATE_TRANSITION_STARTUP,             //Transition to the startup state
  LSTATE_STARTUP,                        //Pump has started, but jet not active yet
  LSTATE_TRANSITION_SHUTDOWN,            //Transition to the shutdown state
  LSTATE_SHUTDOWN,                       //Activate cutter, wait a bit then shutdown pump
  LSTATE_AUTO_SHUTDOWN_OVER_TEMP,        //Auto shutdown due to over temperature
  LSTATE_AUTO_SHUTDOWN_UNDER_TEMP,       //Auto shutdown due to under temperature
  LSTATE_AUTO_SHUTDOWN_NO_TEMP,          //Auto shutdown due to no temperature reading
  LSTATE_AUTO_SHUTDOWN_WATER_LEVEL_LOW,  //Auto shutdown due to water level being too low
  LSTATE_AUTO_SHUTDOWN_WATER_LEVEL_HIGH, //Auto shutdown due to water level being too high
  LSTATE_AUTO_SHUTDOWN_WIND,             //Auto shutdown due to wind
  LSTATE_SCENE_RUNNING,                  //Running a scene
} 
LaminarState;

PROGMEM prog_char string_laminar_state_0[]  = "Transition Off";
PROGMEM prog_char string_laminar_state_1[]  = "Off";
PROGMEM prog_char string_laminar_state_2[]  = "Transition Startup";
PROGMEM prog_char string_laminar_state_3[]  = "Startup";
PROGMEM prog_char string_laminar_state_4[]  = "Transition Shutdown";
PROGMEM prog_char string_laminar_state_5[]  = "Shutdown";
PROGMEM prog_char string_laminar_state_6[]  = "Auto Shutdown Over Temp";
PROGMEM prog_char string_laminar_state_7[]  = "Auto Shutdown Under Temp";
PROGMEM prog_char string_laminar_state_8[]  = "Auto Shutdown No Temp";
PROGMEM prog_char string_laminar_state_9[]  = "Auto Shutdown Water Level Low";
PROGMEM prog_char string_laminar_state_10[] = "Auto Shutdown Water Level High";
PROGMEM prog_char string_laminar_state_11[] = "Auto Shutdown Wind";
PROGMEM prog_char string_laminar_state_12[] = "Scene Running";

const prog_char *string_laminar_state_table[] = 
{
  string_laminar_state_0,
  string_laminar_state_1,
  string_laminar_state_2,
  string_laminar_state_3,
  string_laminar_state_4,
  string_laminar_state_5,
  string_laminar_state_6,
  string_laminar_state_7,
  string_laminar_state_8,
  string_laminar_state_9,
  string_laminar_state_10,
  string_laminar_state_11,
  string_laminar_state_12,
};



typedef enum _LaminarScene
{
  LSCENE_NONE = 0,
  LSCENE_TEST,
  LSCENE_NIGHT_PLAIN,
  LSCENE_NIGHT_VIBRATION,
  LSCENE_NIGHT_REGULAR_FLASH,
  LSCENE_NIGHT_RANDOM_FLASH,
  LSCENE_DAY_PLAIN,
  LSCENE_DAY_SLOW_CUT,
  LSCENE_DAY_FAST_CUT,
  LSCENE_DAY_RANDOM_CUT,
}
LaminarScene;

#define LSCENE_LAST LSCENE_DAY_RANDOM_CUT


PROGMEM prog_char string_laminar_scene_0[]  = "None";
PROGMEM prog_char string_laminar_scene_1[]  = "Test";
PROGMEM prog_char string_laminar_scene_2[]  = "Night - Plain";
PROGMEM prog_char string_laminar_scene_3[]  = "Night - Vibration";
PROGMEM prog_char string_laminar_scene_4[]  = "Night - Regular Flash";
PROGMEM prog_char string_laminar_scene_5[]  = "Night - Random Flash";
PROGMEM prog_char string_laminar_scene_6[]  = "Day - Plain";
PROGMEM prog_char string_laminar_scene_7[]  = "Day - Slow Cut";
PROGMEM prog_char string_laminar_scene_8[]  = "Day - Fast Cut";
PROGMEM prog_char string_laminar_scene_9[]  = "Day - Random Cut";

const prog_char *string_laminar_scene_table[] = 
{
  string_laminar_scene_0,
  string_laminar_scene_1,
  string_laminar_scene_2,
  string_laminar_scene_3,
  string_laminar_scene_4,
  string_laminar_scene_5,
  string_laminar_scene_6,
  string_laminar_scene_7,
  string_laminar_scene_8,
  string_laminar_scene_9,
};



#define LOOP_DELAY  10                    //Milliseconds
#define TEMPERATURE_READ_INTERVAL  10    //How often we poll the temperature sensors (seconds)

#define ANALOG_SENSOR_READ_FREQUENCY  5  //How often to check sensors - seconds
#define SENSOR_CHECK_FREQUENCY        1  //How often to check that sensors are within permitted ranges - seconds

#define ANALOG_SENSORS_SIZE 3
const  int analogSensorsPins[ANALOG_SENSORS_SIZE]     = {WATER_LEVEL1_PIN, WATER_LEVEL2_PIN, WIND_PIN};
int        analogSensorsValues[ANALOG_SENSORS_SIZE]   = {-1, -1, -1};
int        analogSensorsIndex                         = 0;
#define WATER_LEVEL1_SENSOR_INDEX 0
#define WATER_LEVEL2_SENSOR_INDEX 1
#define WIND_SENSOR_INDEX         2


LaminarState     state = LSTATE_TRANSITION_OFF;
time_t           stateTimer;
time_t           sceneTimer;

LaminarScene     laminarScene = LSCENE_DAY_PLAIN;


#define MAX_WIND_SENSOR_HISTORY 20  //20 = approx 5 mins of reading
int       windSensorHistory[MAX_WIND_SENSOR_HISTORY];
int       windSensorHistoryNextWriteLocation = 0;
Statistic windSensorStats;

AlarmID_t lastCutAlarm, lastFlashAlarm;


#define NIGHT_FLASH_REGULAR_PERIOD    1    //Seconds
#define NIGHT_FLASH_LOWER_RANDOM      1    //Seconds
#define NIGHT_FLASH_UPPER_RANDOM      10   //Seconds
#define STEPPER_FLASH_AMOUNT          10
#define DAY_SLOW_CUT_PERIOD           5    //Seconds
#define DAY_FAST_CUT_PERIOD           2    //Seconds
#define DAY_RANDOM_CUT_LOWER_RANDOM   1    //Seconds
#define DAY_RANDOM_CUT_UPPER_RANDOM   10   //Seconds



//Serial print for progmem strings

void SerialPrint_P(const prog_char str[])
{
  char c;

  if(!str) return;

  while((c = pgm_read_byte(str++)))  Serial.print(c,BYTE);
}



//Serial println for progmem strings

void SerialPrintln_P(const prog_char str[])
{
  SerialPrint_P(str);
  Serial.println();
}



PROGMEM prog_char string_general_0[] = "1";
PROGMEM prog_char string_general_1[] = "2";
PROGMEM prog_char string_general_2[] = "% (";
PROGMEM prog_char string_general_3[] = ")";
PROGMEM prog_char string_general_4[]  = "ON";
PROGMEM prog_char string_general_5[]  = "OFF";
PROGMEM prog_char string_general_6[]  = "<B>Temperatures:</B>";
PROGMEM prog_char string_general_7[]  = "<B>Water Levels:</B>";
PROGMEM prog_char string_general_8[]  = "<B>Wind:</B>";
PROGMEM prog_char string_general_9[]  = "\t";
PROGMEM prog_char string_general_10[]  = ":\t";
PROGMEM prog_char string_general_11[]  = "C";
PROGMEM prog_char string_general_12[]  = "-";
PROGMEM prog_char string_general_13[]  = "done";
PROGMEM prog_char string_general_14[]  = "Tank";
PROGMEM prog_char string_general_15[] = "Starting wifi and web server...";
PROGMEM prog_char string_general_16[] = "Starting BlinkM MaxM's...";
PROGMEM prog_char string_general_17[] = "Ready";
PROGMEM prog_char string_general_18[] = "<B>Status:</B>";
PROGMEM prog_char string_general_19[] = "Auto Shutdown Over Temp";
PROGMEM prog_char string_general_20[] = "Auto Shutdown Water Level Low";
PROGMEM prog_char string_general_21[] = "Auto Shutdown Water Level High";
PROGMEM prog_char string_general_22[] = "Auto Shutdown Wind";
PROGMEM prog_char string_general_23[] = "Manual";
PROGMEM prog_char string_general_24[] = "<B>Last Shutdown Reason:</B>";
PROGMEM prog_char string_general_25[] = "Auto Shutdown Under Temp";
PROGMEM prog_char string_general_26[] = "Auto Shutdown No Temp";


prog_char  *lastShutdownReason = string_general_23;
char lastShutdownReasonDetail[80];

#ifdef WISERVER
PROGMEM prog_char string_wiserver_0[]  = "<html><head>";
PROGMEM prog_char string_wiserver_1[]  = "<TABLE border=\"1\">";
PROGMEM prog_char string_wiserver_2[]  = "<TR><TD>";
PROGMEM prog_char string_wiserver_3[]  = "</TD><TD>";
PROGMEM prog_char string_wiserver_4[]  = "&#8451;";
PROGMEM prog_char string_wiserver_5[]  = "</TABLE>";
PROGMEM prog_char string_wiserver_6[]  = "</div></body></html>";
PROGMEM prog_char string_wiserver_7[]  = "</TD></TR>";
PROGMEM prog_char string_wiserver_8[]  = "<P />";
PROGMEM prog_char string_wiserver_9[]  = "<BLOCKQUOTE>";
PROGMEM prog_char string_wiserver_10[]  = "</BLOCKQUOTE>";
PROGMEM prog_char string_wiserver_13[] = "Not Connected";
PROGMEM prog_char string_wiserver_19[] = "Cutter Flash:";
PROGMEM prog_char string_wiserver_20[] = "f";
PROGMEM prog_char string_wiserver_22[] = "Cutter:";
PROGMEM prog_char string_wiserver_23[] = "c";
PROGMEM prog_char string_wiserver_24[] = "Cutter Vibration:";
PROGMEM prog_char string_wiserver_25[] = "v";
PROGMEM prog_char string_wiserver_26[] = "<meta http-equiv='REFRESH' content='0;url=/'>";
PROGMEM prog_char string_wiserver_27[] = "</head><body style=\"background-color:#";
PROGMEM prog_char string_wiserver_28[] = ";\"><div style=\"background-color:#CCCCCC;margin:20pt;\">";
PROGMEM prog_char string_wiserver_30[] = "<form name=\"control\" action=\"/\" method=\"GET\">";
PROGMEM prog_char string_wiserver_31[] = "<TABLE border=\"0\">";
PROGMEM prog_char string_wiserver_32[] = "Pumps:";
PROGMEM prog_char string_wiserver_33[] = "</FORM>";
PROGMEM prog_char string_wiserver_34[] = "<SELECT name=\"";
PROGMEM prog_char string_wiserver_35[] = "<OPTION value=\"";
PROGMEM prog_char string_wiserver_36[] = "\"";
PROGMEM prog_char string_wiserver_37[] = " selected";
PROGMEM prog_char string_wiserver_38[] = "</OPTION>";
PROGMEM prog_char string_wiserver_39[] = "</SELECT>";
PROGMEM prog_char string_wiserver_40[] = ">";
PROGMEM prog_char string_wiserver_41[] = "Thumper:";
PROGMEM prog_char string_wiserver_42[] = "\" onChange=\"control.submit();\">";
PROGMEM prog_char string_wiserver_43[] = "p";
PROGMEM prog_char string_wiserver_44[] = "t";
PROGMEM prog_char string_wiserver_45[] = "l";
PROGMEM prog_char string_wiserver_46[] = "LED:";
PROGMEM prog_char string_wiserver_47[] = "Scene:";
PROGMEM prog_char string_wiserver_48[] = "s";
PROGMEM prog_char string_wiserver_49[] = "<CENTER><U>Tank ";
PROGMEM prog_char string_wiserver_50[] = "</U><BR><BR>";
PROGMEM prog_char string_wiserver_51[] = "%<table width=75 cellspacing=0 cellpadding=0 border=1><tr><td bgcolor=#FFFFFF height=";
PROGMEM prog_char string_wiserver_52[] = "></td></tr><tr><td bgcolor=#00FFFF height=";
PROGMEM prog_char string_wiserver_53[] = "></td></tr>";
PROGMEM prog_char string_wiserver_55[] = "</CENTER>";
PROGMEM prog_char string_wiserver_56[] = "<TR>";
PROGMEM prog_char string_wiserver_57[] = "</TR>";
PROGMEM prog_char string_wiserver_65[] = "Default";              //0
PROGMEM prog_char string_wiserver_66[] = "RGB";                  //1
PROGMEM prog_char string_wiserver_67[] = "White Flash";          //2
PROGMEM prog_char string_wiserver_68[] = "Red Flash";            //3
PROGMEM prog_char string_wiserver_69[] = "Green Flash";          //4 
PROGMEM prog_char string_wiserver_70[] = "Blue Flash";           //5
PROGMEM prog_char string_wiserver_71[] = "Cyan Flash";           //6
PROGMEM prog_char string_wiserver_72[] = "Magenta Flash";        //7
PROGMEM prog_char string_wiserver_73[] = "Yellow Flash";         //8
PROGMEM prog_char string_wiserver_74[] = "Black";                //9
PROGMEM prog_char string_wiserver_75[] = "Hue Cycle";            //10
PROGMEM prog_char string_wiserver_76[] = "Mood Light";           //11
PROGMEM prog_char string_wiserver_77[] = "Virtual Candle";       //12
PROGMEM prog_char string_wiserver_78[] = "Water Reflections";    //13
PROGMEM prog_char string_wiserver_79[] = "Old Neon";             //14
PROGMEM prog_char string_wiserver_80[] = "The Seasons";          //15
PROGMEM prog_char string_wiserver_81[] = "Thunderstorm";         //16
PROGMEM prog_char string_wiserver_82[] = "Stop Light";           //17
PROGMEM prog_char string_wiserver_83[] = "Morse Code";           //18
PROGMEM prog_char string_wiserver_84[] = "White";                //40
PROGMEM prog_char string_wiserver_85[] = "Off";                  //41
#endif

#ifdef BLINKM
PROGMEM prog_char string_scanfunc_0[] = "\tFound address: ";
PROGMEM prog_char string_scanfunc_1[] = "  version: ";
PROGMEM prog_char string_scanfunc_2[] = ".";
#endif

#ifdef TEMPERATURE
PROGMEM prog_char string_temperature_1[]  = "Temperature Sensors:";
PROGMEM prog_char string_temperature_2[]  = "      Starting...";
PROGMEM prog_char string_temperature_3[]  = "      Locating...";
PROGMEM prog_char string_temperature_4[]  = " devices found";
PROGMEM prog_char string_temperature_5[]  = "      Parasite power is: ";
PROGMEM prog_char string_temperature_8[]  = "      Setting resolution to: ";
PROGMEM prog_char string_temperature_9[]  = " bits...";
PROGMEM prog_char string_temperature_11[] = "      Reading initial...";
PROGMEM prog_char string_temperature_12[] = "Reading temp sensors...";
PROGMEM prog_char string_temperature_13[] = ": ";
PROGMEM prog_char string_temperature_14[] = "Outside";
PROGMEM prog_char string_temperature_15[] = "Electronics";
PROGMEM prog_char string_temperature_16[] = "Stepper 1";
PROGMEM prog_char string_temperature_17[] = "Stepper 2";
PROGMEM prog_char string_temperature_18[] = "LED Sink 1";
PROGMEM prog_char string_temperature_19[] = "LED Sink 2";
#endif



//Converts a float to a char with precision

char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}



#ifdef WISERVER

#ifdef TEMPERATURE
void webServerPrintTemperature(const prog_char *sensorName, DeviceAddress addr)
{
  WiServer.println_P(string_wiserver_2);
  WiServer.print_P(sensorName);
  WiServer.println_P(string_wiserver_3);

  float temperature = getTemperature(addr);

  if ( temperature == -127.00 )  WiServer.print_P(string_general_12);
  else
  {
    WiServer.print(temperature);
    WiServer.print_P(string_wiserver_4);    
  }

  WiServer.println_P(string_wiserver_7);    
}
#endif


#ifdef WATER_LEVEL
void webServerPrintWaterLevel(const prog_char *sensorName, int pin)
{
  int waterLevelOrig = getWaterAnalogSensorValue(pin);
  float waterLevel   = convertWaterLevelToPercent(waterLevelOrig, pin);

  WiServer.println("<TD>");

  if ( waterLevelOrig != -1 )
  {
    //Print the water level graphically
    int tankLevel = constrain(waterLevel, 0, 100);
    
    WiServer.print_P(string_wiserver_49);
    WiServer.print_P(sensorName);
    WiServer.print_P(string_wiserver_50);
    WiServer.print(waterLevel);
    WiServer.print_P(string_wiserver_51);
    WiServer.print(100 - tankLevel);
    WiServer.print_P(string_wiserver_52);
    WiServer.print(tankLevel);
    WiServer.print_P(string_wiserver_53);
    WiServer.print_P(string_wiserver_5);
    WiServer.print(waterLevelOrig);
    WiServer.println_P(string_wiserver_55);
  }
  
  
  WiServer.println("</TD>");    
}
#endif


void webServerOption(int value, const prog_char *valueStr, boolean selected)
{      
  WiServer.print_P(string_wiserver_35);

  WiServer.print(value);

  WiServer.print_P(string_wiserver_36);

  if ( selected )  WiServer.print_P(string_wiserver_37);

  WiServer.print_P(string_wiserver_40);

  WiServer.print_P(valueStr);

  WiServer.println_P(string_wiserver_38);
}



void webServerSelect(const prog_char *name)
{
  WiServer.print_P(string_wiserver_34);
  WiServer.print_P(name);
  WiServer.println_P(string_wiserver_42);
}



void actionForm(char *URL)
{
  char cmd;
  int value;
  char *p = URL;

  while ( *p )
  {
    cmd = *p ++;
    p ++;  //Remove =
    sscanf(p, "%d", &value);

    //Now remove numbers
    while (( *p >= '0') && ( *p <= '9' ))  p ++;

    if ( *p == '&' ) p ++;

    switch ( cmd )
    {
#ifdef PUMPS
    case 'p':
      if      (( value ) && (! isPumpsOn())) state = LSTATE_TRANSITION_STARTUP;
      else if (( ! value ) && ( isPumpsOn()))          
      {
        lastShutdownReason = string_general_23;
        strcpy(lastShutdownReasonDetail, "");
        state = LSTATE_TRANSITION_SHUTDOWN;
      }
      break;
#endif
#ifdef STEPPER
    case 'c':
      if ( value )    stepperCut(&stepper1, true);
      else            stepperCut(&stepper1, false);
      break;
    case 'v':
      if ( value )    stepperVibration(&stepper1, 3);
      else            stepperVibration(&stepper1, 0);
      break;
    case 'f':
      if ( value )    stepperFlash(&stepper1, 10);
      break;
#endif
#ifdef THUMPER
    case 't':
      if ( value )  thumperSwitch(true);
      else          thumperSwitch(false);
      break;
#endif
#ifdef BLINKM
    case 'l':
      if      ( value == 40 )
      {
        blinkMSetTestColor(BLINKM_MAXM1);
        blinkMSetTestColor(BLINKM_MAXM2);
      } 
      else if ( value == 41 )
        blinkMSwitchAllOff();
      else
      {
        blinkMPlayScript(BLINKM_MAXM1, value);
        blinkMPlayScript(BLINKM_MAXM2, value);
      }
      break;
#endif
    case 's':
      if ( laminarScene != (LaminarScene)value )
      {
        sceneShutdown();
        laminarScene = (LaminarScene)value;
        sceneSetup();
      }
      break;
    }
  }      
}



void printHex(byte b)
{
  if ( b < 16 )  WiServer.print("0");
  WiServer.print(b, HEX);
}



//Called multiple times due to limited buffer space

boolean sendMyPage(char *URL)
{
  Serial.print("URL: ");
  Serial.println(URL);

  if ( strncmp(URL, "/?",2) == 0)
  {
    actionForm(URL+2);

    WiServer.print_P(string_wiserver_0);

    WiServer.print_P(string_wiserver_26);

    WiServer.print_P(string_wiserver_27);
    WiServer.print_P("FFFFFF");
    WiServer.println_P(string_wiserver_28);
    WiServer.println_P(string_wiserver_6);

    return true;
  }

  // Check if the requested URL matches "/"
  if (strcmp(URL, "/") == 0)
  {
    WiServer.print_P(string_wiserver_0);

    WiServer.print_P(string_wiserver_27);

#ifdef BLINKM
    byte r, g, b;
    BlinkM_getRGBColor(BLINKM_MAXM1, &r, &g, &b);
    printHex(r);
    printHex(g);
    printHex(b);
#endif

    WiServer.println_P(string_wiserver_28);
    
    //Print the status
    WiServer.print_P(string_general_18);
    WiServer.print_P(string_general_9);
    WiServer.println_P(string_laminar_state_table[state]);
    WiServer.println_P(string_wiserver_8);

    //Print the last shutdown
    WiServer.print_P(string_general_24);
    WiServer.print_P(string_general_9);
    WiServer.print_P(lastShutdownReason);
    WiServer.print(": ");
    WiServer.print(lastShutdownReasonDetail);
    WiServer.println_P(string_wiserver_8);

#ifdef TEMPERATURE
    //Temperatures
    float temperature;
    WiServer.println_P(string_general_6);    
    WiServer.println_P(string_wiserver_9);
    WiServer.println_P(string_wiserver_1);
    webServerPrintTemperature(string_temperature_14, outsideThermometer); 
    webServerPrintTemperature(string_temperature_15, electronicsThermometer);
    webServerPrintTemperature(string_temperature_16, stepper1Thermometer);
    webServerPrintTemperature(string_temperature_17, stepper2Thermometer);
    webServerPrintTemperature(string_temperature_18, led1Thermometer);
    webServerPrintTemperature(string_temperature_19, led2Thermometer);
    WiServer.println_P(string_wiserver_5);
    WiServer.println_P(string_wiserver_10);

    WiServer.println_P(string_wiserver_8);
#endif

#ifdef WATER_LEVEL
    //Water levels
    WiServer.println_P(string_general_7);
    WiServer.println_P(string_wiserver_9);
    WiServer.println_P(string_wiserver_31);
    
    WiServer.println_P(string_wiserver_56);
    webServerPrintWaterLevel(string_general_0, WATER_LEVEL1_PIN);
    webServerPrintWaterLevel(string_general_1, WATER_LEVEL2_PIN);
    WiServer.println_P(string_wiserver_57);
    WiServer.println_P(string_wiserver_5);  
    WiServer.println_P(string_wiserver_10);

    WiServer.println_P(string_wiserver_8);
#endif

#ifdef WIND
    //Wind
    WiServer.print_P(string_general_8);
    WiServer.print_P(string_general_9);
    int windSpeed = getAnalogSensorValue(WIND_SENSOR_INDEX);
    if ( windSpeed == -1 )  WiServer.println_P(string_general_12);
    else
    {
       WiServer.println(windSpeed);
       
       //Calculate the statistcs
       calcWindSensorStats();
       WiServer.print(" (#");
       WiServer.print(windSensorStats.count());
       WiServer.print(" Avg:");
       WiServer.print(windSensorStats.average());
       WiServer.print(" Gust(5mins):");
       WiServer.print(windSensorStats.maximum());
       WiServer.print(" PopStdDev:");
       WiServer.print(windSensorStats.pop_stdev());
       WiServer.print(" UnbiasedStdDev:");
       WiServer.println(windSensorStats.unbiased_stdev());
    }
#endif

    //Form
    WiServer.println_P(string_wiserver_30);
    WiServer.println_P(string_wiserver_31);


#ifdef PUMPS
    //Pumps
    WiServer.println_P(string_wiserver_2);
    WiServer.println_P(string_wiserver_32);
    WiServer.println_P(string_wiserver_3);
    webServerSelect(string_wiserver_43);

    webServerOption(0, string_general_5, ! isPumpsOn() );  //Option Off
    webServerOption(1, string_general_4, isPumpsOn() );    //Option On

      WiServer.println_P(string_wiserver_39);
    WiServer.println_P(string_wiserver_7);
#endif

/*
#ifdef THUMPER
    //Thumper
    WiServer.println_P(string_wiserver_2);
    WiServer.println_P(string_wiserver_41);
    WiServer.println_P(string_wiserver_3);
    webServerSelect(string_wiserver_44);

    webServerOption(0, string_general_5, ! isThumperOn() );  //Option Off
    webServerOption(1, string_general_4, isThumperOn() );    //Option On

    WiServer.println_P(string_wiserver_39);
    WiServer.println_P(string_wiserver_7);
#endif
*/

/*
#ifdef STEPPER
    //Cutter
    WiServer.println_P(string_wiserver_2);
    WiServer.println_P(string_wiserver_22);

    WiServer.println_P(string_wiserver_3);
    webServerSelect(string_wiserver_23);

    webServerOption(0, string_general_5, ! stepper1Cut );  //Option Off
    webServerOption(1, string_general_4, stepper1Cut );    //Option On

    WiServer.println_P(string_wiserver_39);
    WiServer.println_P(string_wiserver_7);

    //Cutter Vibration
    WiServer.println_P(string_wiserver_2);
    WiServer.println_P(string_wiserver_24);

    WiServer.println_P(string_wiserver_3);
    webServerSelect(string_wiserver_25);

    webServerOption(0, string_general_5, ! stepper1Vibration );  //Option Off
    webServerOption(1, string_general_4, stepper1Vibration );    //Option On

      WiServer.println_P(string_wiserver_39);
    WiServer.println_P(string_wiserver_7);

    //Cutter Flash
    WiServer.println_P(string_wiserver_2);
    WiServer.println_P(string_wiserver_19);

    WiServer.println_P(string_wiserver_3);
    webServerSelect(string_wiserver_20);

    webServerOption(0, string_general_5, true );  //Option Off
    webServerOption(1, string_general_4, false );    //Option On

    WiServer.println_P(string_wiserver_39);
    WiServer.println_P(string_wiserver_7);
#endif
*/

#ifdef BLINKM
    //LED
    WiServer.println_P(string_wiserver_2);
    WiServer.println_P(string_wiserver_46);
    WiServer.println_P(string_wiserver_3);
    webServerSelect(string_wiserver_45);

    webServerOption(0,  string_wiserver_65, (activeScript == 0));
    webServerOption(1,  string_wiserver_66, (activeScript == 1));
    webServerOption(2,  string_wiserver_67, (activeScript == 2));
    webServerOption(3,  string_wiserver_68, (activeScript == 3));
    webServerOption(4,  string_wiserver_69, (activeScript == 4));
    webServerOption(5,  string_wiserver_70, (activeScript == 5));
    webServerOption(6,  string_wiserver_71, (activeScript == 6));
    webServerOption(7,  string_wiserver_72, (activeScript == 7));
    webServerOption(8,  string_wiserver_73, (activeScript == 8));
    webServerOption(9,  string_wiserver_74, (activeScript == 9));
    webServerOption(10, string_wiserver_75, (activeScript == 10));
    webServerOption(11, string_wiserver_76, (activeScript == 11));
    webServerOption(12, string_wiserver_77, (activeScript == 12));
    webServerOption(13, string_wiserver_78, (activeScript == 13));
    webServerOption(14, string_wiserver_79, (activeScript == 14));
    webServerOption(15, string_wiserver_80, (activeScript == 15));
    webServerOption(16, string_wiserver_81, (activeScript == 16));
    webServerOption(17, string_wiserver_82, (activeScript == 17));
    webServerOption(18, string_wiserver_83, (activeScript == 18));
    webServerOption(40, string_wiserver_84, (activeScript == 40));
    webServerOption(41, string_wiserver_85, (activeScript == 41));

    WiServer.println_P(string_wiserver_39);
    WiServer.println_P(string_wiserver_7);
#endif

    //Laminar Scene
    WiServer.println_P(string_wiserver_2);
    WiServer.println_P(string_wiserver_47);
    WiServer.println_P(string_wiserver_3);
    webServerSelect(string_wiserver_48);

    for ( int i = 0; i <= LSCENE_LAST; i ++ )
      webServerOption(i, string_laminar_scene_table[i], (laminarScene == i));

    WiServer.println_P(string_wiserver_39);
    WiServer.println_P(string_wiserver_7);
    //End Laminar Scene

    //End table and close form
    WiServer.println_P(string_wiserver_5);
    WiServer.println_P(string_wiserver_33);


    WiServer.println_P(string_wiserver_6);    

    // URL was recognized
    return true;
  }

  // URL not found
  return false;
}

#endif



#ifdef BLINKM

//I2C devices tend to hang if the arduino is reset whilst data is being sent.
//This will unhang the I2C bus.
//
//http://forums.freescale.com/t5/16-Bit-Microcontrollers/I2C-Bus-hangs-after-reset/td-p/26772

void resetI2CBus(void)
{
  //Set I2C bus (SDA / SCL) to output
  pinMode(D20, OUTPUT);
  pinMode(D21, OUTPUT);
  
  //Hold SDA / SCL high
  digitalWrite(D20, HIGH);
  digitalWrite(D21, HIGH);
  
  delay(100);
  
  //Set I2C bus (SDA / SCL ) back to input
  pinMode(D20, INPUT);
  pinMode(D21, INPUT);
}



// called when address is found in BlinkM_scanI2CBus()

void scanfunc( byte addr, byte result )
{
  if ( result )  return;
  SerialPrint_P(string_scanfunc_0);
  Serial.print(addr,DEC);
  SerialPrint_P(string_scanfunc_1);
  int version = BlinkM_getVersion(addr);
  Serial.print(version>>8);
  SerialPrint_P(string_scanfunc_2);
  Serial.println(version&0xFF);
}



void blinkMPlayScript(int blinkm_id, int script_id)
{
  BlinkM_playScript(blinkm_id, script_id, 0, 0);
  activeScript = script_id;
}



void blinkMSwitchOff(int blinkm_id)
{
  BlinkM_stopScript(blinkm_id);  
  blinkMPlayScript(blinkm_id, 9);
  activeScript = 41;
}



void blinkMSwitchAllOff()
{
  blinkMSwitchOff(BLINKM_MAXM1);
  blinkMSwitchOff(BLINKM_MAXM2);
}



void blinkMSetTestColor(int blinkm_id)
{
  BlinkM_stopScript(blinkm_id); 
  BlinkM_setRGB(blinkm_id, 255, 255, 255);  
  activeScript = 40;
}

#endif


#ifdef STEPPER

void disableSteppers()
{
  digitalWrite(stepperDisablePin, LOW);
  stepper1.disableOutputs();
}



void enableSteppers()
{
  digitalWrite(stepperDisablePin, HIGH);
  stepper1.enableOutputs();
}



void setupStepper(AccelStepper *stepper, int index)
{
  Serial.print("Setting up stepper ");
  Serial.print(index, DEC);
  Serial.print("...");

  stepper->setMaxSpeed(stepperFastSpeed);
  stepper->setSpeed(stepperFastSpeed);
  stepper->setAcceleration(30.0);

  SerialPrintln_P(string_general_13);
}



void findStepperZero(AccelStepper *stepper, int index, long correctionToCenter)
{
  Serial.print("Finding zero point for stepper: ");
  Serial.print(index, DEC);
  Serial.print("...");

  stepper->setCurrentPosition(0);
  stepper->runToNewPosition(0);

  //Find to the extreme (rest against stop)
  stepper1.setAcceleration(100.0);
  stepper->moveTo(-100);
  stepper->runToPosition();
  stepper->setCurrentPosition(0);
  stepper->runToNewPosition(0);

  //Move to the center
  stepper1.setAcceleration(100.0);
  stepper->moveTo(correctionToCenter);
  stepper->runToPosition();

  Serial.println("found");
  stepper->setCurrentPosition(0);  //Set the zero point
  stepper->setSpeed(stepperFastSpeed);
}



void stepperCut(AccelStepper *stepper, boolean on)
{
  if ( stepper->distanceToGo() == 0 )
  {
    enableSteppers();

    stepper1Cut = on;

    stepper->setSpeed(stepperFastSpeed);
    stepper->setAcceleration(6000.0);

    stepper->moveTo( (on) ? stepper1PositionCut : stepper1Position1);

    stepper->runToPosition();

    disableSteppers();
  }
}



void stepperFlash(AccelStepper *stepper, int amount)
{
  enableSteppers();

  stepperCut(stepper, false);

  stepper->setSpeed(stepperFastSpeed);
  stepper->setAcceleration(6000.0);

  stepper->moveTo(stepper1PositionFlash - amount);
  stepper->runToPosition();

  stepper->move(stepper1CorrectionToCenter + stepper1Position1);
  stepper->runToPosition();

  //Setup the center again, as we lost it when we hit the stop
  stepper->setCurrentPosition(stepper1Position1);    

  disableSteppers();
}



void stepperVibration(AccelStepper *stepper, int amount)
{
  stepper1Vibration = amount;

  if ( stepper1Vibration )  enableSteppers();

  //   if ( stepper1Vibration == 0 )  stepperCut(stepper, false);
}



void stepperVibrationPoll(AccelStepper *stepper)
{
  if (( ! stepper1Vibration ) && ( stepper->distanceToGo() == 0 ))  disableSteppers();

  if (( stepper1Vibration ) && ( stepper->distanceToGo() == 0 ))
  {
    stepper->setSpeed(stepperFastSpeed);
    stepper->setAcceleration(1000.0);

    if ( stepper1VibrationPosition == 0 )
    {
      stepper->moveTo(stepper1Position1 + stepper1Vibration);
      stepper1VibrationPosition = 1;
    }
    else                                   
    {
      stepper->moveTo(stepper1Position1);
      stepper1VibrationPosition = 0;
    } 
  }
}


#endif



#ifdef TEMPERATURE

float getTemperature(DeviceAddress deviceAddress)
{
  return sensors.getTempC(deviceAddress);
}


// function to print the temperature for a device
void printTemperature(const prog_char *sensorName, DeviceAddress deviceAddress)
{
  float tempC = getTemperature(deviceAddress);
  SerialPrint_P(string_general_9);
  SerialPrint_P(sensorName);
  SerialPrint_P(string_general_10);

  if ( tempC == -127.00 )  SerialPrintln_P(string_general_12);
  else
  {
    Serial.print(tempC);
    Serial.print(176, BYTE);  //Print degree character
    SerialPrintln_P(string_general_11);
  }
}



void setupTemperatures()
{

  //Startup temperature sensors
  SerialPrintln_P(string_temperature_1);
  SerialPrint_P(string_temperature_2);
  sensors.begin();
  SerialPrintln_P(string_general_13);

  //Figure out how many temperature devices are on the 1 wire bus 
  SerialPrint_P(string_temperature_3);
  int numDevices = sensors.getDeviceCount();
  Serial.print(numDevices, DEC);
  SerialPrintln_P(string_temperature_4);

  // report parasite power requirements
  SerialPrint_P(string_temperature_5); 
  if (sensors.isParasitePowerMode()) SerialPrintln_P(string_general_4);
  else                               SerialPrintln_P(string_general_5);

  //Set resolution
  SerialPrint_P(string_temperature_8);
  Serial.print(TEMPERATURE_PRECISION);
  SerialPrint_P(string_temperature_9);

  sensors.setResolution(electronicsThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(outsideThermometer,     TEMPERATURE_PRECISION);
  sensors.setResolution(led1Thermometer,        TEMPERATURE_PRECISION); 
  //  sensors.setResolution(led2Thermometer,      TEMPERATURE_PRECISION);
  sensors.setResolution(stepper1Thermometer,    TEMPERATURE_PRECISION);
  //  sensors.setResolution(stepper2Thermometer,  TEMPERATURE_PRECISION);

  SerialPrintln_P(string_general_13);

  SerialPrint_P(string_temperature_11);
  sensors.requestTemperatures();
  SerialPrintln_P(string_general_13);
}



void readTemperatures()
{
  SerialPrint_P(string_temperature_12);

  sensors.requestTemperatures();

  SerialPrintln_P(string_general_13);
}


void printTemperaturesToSerial()
{
  SerialPrintln_P(string_general_6);  

  printTemperature(string_temperature_14, outsideThermometer);
  printTemperature(string_temperature_15, electronicsThermometer);
  printTemperature(string_temperature_16, stepper1Thermometer);
  printTemperature(string_temperature_17, stepper2Thermometer);    
  printTemperature(string_temperature_18, led1Thermometer);
  printTemperature(string_temperature_19, led2Thermometer);
}

#endif



int getAnalogSensorValue(int sensorIndex)
{
  return analogSensorsValues[sensorIndex];
}



#ifdef WATER_LEVEL

int getWaterAnalogSensorValue(int pin)
{
  int sensorIndex;
  
  if      ( pin == WATER_LEVEL1_PIN )  sensorIndex = WATER_LEVEL1_SENSOR_INDEX;
  else if ( pin == WATER_LEVEL2_PIN )  sensorIndex = WATER_LEVEL2_SENSOR_INDEX;
  else                                 return -1;

  return getAnalogSensorValue(sensorIndex);
}



float convertWaterLevelToPercent(int waterLevel, int pin)
{
  float lower, upper;

  if      ( pin == WATER_LEVEL1_PIN )
  {
    lower = WATER_LEVEL1_ZERO_PERCENT;
    upper = WATER_LEVEL1_100_PERCENT;
  }
  else if ( pin == WATER_LEVEL2_PIN )
  {
    lower = WATER_LEVEL2_ZERO_PERCENT;
    upper = WATER_LEVEL2_100_PERCENT;
  }
  else
  {
    lower = -200;
    upper = -100;
  }

  return map(waterLevel, lower, upper, 0, 100);
}



void checkWaterTankWithinLevels(int pin)
{
  if ( ( state == LSTATE_TRANSITION_SHUTDOWN ) || ( state == LSTATE_SHUTDOWN ) ||
       ( state == LSTATE_TRANSITION_OFF )      || ( state == LSTATE_OFF ))  return;
  
  float lower, upper;
  int waterLevelOrig = getWaterAnalogSensorValue(pin);
  float waterLevel   = convertWaterLevelToPercent(waterLevelOrig, pin);

  if ( waterLevelOrig == -1 )  return;  //If we don't have a sensor reading yet, we return

  if      ( pin == WATER_LEVEL1_PIN )
  {
    lower = WATER_LEVEL1_LOW_LIMIT;
    upper = WATER_LEVEL1_HIGH_LIMIT;
  }
  else if ( pin == WATER_LEVEL2_PIN )
  {
    lower = WATER_LEVEL2_LOW_LIMIT;
    upper = WATER_LEVEL2_HIGH_LIMIT;
  }
  else
  {
    lower = 40.0;
    upper = 60.0;
  }
  
  LaminarState tempState = (LaminarState)0;
  
  if      ( waterLevel > upper )  tempState = LSTATE_AUTO_SHUTDOWN_WATER_LEVEL_HIGH;
  else if ( waterLevel < lower )  tempState = LSTATE_AUTO_SHUTDOWN_WATER_LEVEL_LOW;
  
  if ( tempState )
  {
    state = tempState;

    char waterLevelStr[20], lowerStr[20], upperStr[20];
    ftoa(waterLevelStr, (double)waterLevel, 2);
    ftoa(lowerStr, (double)lower, 2);
    ftoa(upperStr, (double)upper, 2);
    sprintf(lastShutdownReasonDetail, "Tank on Pin: %d: %s (%s - %s)", pin, waterLevelStr, lowerStr, upperStr);
  }
}



#endif



#ifdef WIND

void writeWindSensorValueToHistory(int value)
{
  windSensorHistory[windSensorHistoryNextWriteLocation++] = value;
  if ( windSensorHistoryNextWriteLocation == MAX_WIND_SENSOR_HISTORY )  windSensorHistoryNextWriteLocation = 0;
}


void clearWindSensorHistory()
{
  for (int i = 0; i < MAX_WIND_SENSOR_HISTORY; i ++ )  windSensorHistory[i] = -1;
}



void calcWindSensorStats()
{
    windSensorStats.clear();
    
    for (int i = 0; i < MAX_WIND_SENSOR_HISTORY; i ++ )
    {
      if ( windSensorHistory[i] != -1 )  windSensorStats.add(windSensorHistory[i]);
    }
}

#endif



#ifdef PUMPS

void pumpsSwitch(boolean on)
{
  digitalWrite(PUMPS_PIN, (on)?HIGH:LOW);
}



boolean isPumpsOn()
{
  if ( ( state == LSTATE_OFF ) || (state == LSTATE_TRANSITION_SHUTDOWN ) || ( state == LSTATE_SHUTDOWN ))  return false;
  
  return true;
}



#endif



#ifdef THUMPER

void thumperSwitch(boolean on)
{
  digitalWrite(THUMPER_PIN, (on)?HIGH:LOW);
}



boolean isThumperOn()
{
  return digitalRead(THUMPER_PIN);
}



void thumperToggle()
{
  if ( isThumperOn() )  thumperSwitch(false);
  else                  thumperSwitch(true);
}

#endif



// Power everything down

void switchEverythingOff()
{
#ifdef PUMPS
  pumpsSwitch(false);
#endif

#ifdef THUMPER
  thumperSwitch(false);
#endif
  
#ifdef STEPPER   
  stepperVibration(&stepper1, 0);
  stepperCut(&stepper1, true);
  disableSteppers();
#endif
      
#ifdef BLINKM
  blinkMSwitchAllOff();
#endif
}



void freeMemoryReport()
{
    Serial.print("freeMemory()=");
    Serial.println(freeMemory());
}



//The water level sensors are high impedeance input sources and the arduino has on ADC that is multiplexed for all the analog pins
//Therefore the high impedence input source may take a while to drive the pin to the correct voltage level, so we switch
//to the pin first by using analogRead, delay for 5 seconds and then analogRead the value
//http://forums.adafruit.com/viewtopic.php?f=25&t=11597
//This routine is non-blocking and reads water level sensors and wind sensor in round robin fashion

void analogSensorRead()
{
  //Serial.print("Sensor Read: ");
  //Serial.println(analogSensorsIndex);
  
  analogSensorsValues[analogSensorsIndex] = analogRead(analogSensorsPins[analogSensorsIndex]);
  
  if ( analogSensorsIndex == WIND_SENSOR_INDEX )  writeWindSensorValueToHistory(analogSensorsValues[analogSensorsIndex]);

  //for (int i = 0; i < ANALOG_SENSORS_SIZE; i ++ )
  //{
  //  Serial.print("Sensor: ");
  //  Serial.print(i);
  //  Serial.print(" = ");
  //  Serial.println(analogSensorsValues[i]);  
  //}
  
  analogSensorsIndex ++;
  if ( analogSensorsIndex >= ANALOG_SENSORS_SIZE )  analogSensorsIndex = 0;
  
  //Setup the next pin to read to give it time to reach a correct level
  analogRead(analogSensorsPins[analogSensorsIndex]);
}



void sensorCheckTempWithinRange(prog_char *sensorName, DeviceAddress deviceAddress, float lowerLimit, float upperLimit, boolean shutdownOnNoConnection)
{
  if ( ( state == LSTATE_TRANSITION_SHUTDOWN ) || ( state == LSTATE_SHUTDOWN ) ||
       ( state == LSTATE_TRANSITION_OFF )      || ( state == LSTATE_OFF ))  return;

  float temp = getTemperature(deviceAddress);
  
  LaminarState tempState = (LaminarState)0;

  if      (( temp == -127.00 ) && ( shutdownOnNoConnection ))   tempState = LSTATE_AUTO_SHUTDOWN_NO_TEMP;
  else if (( temp <= lowerLimit ) && ( temp != -127.00 ))       tempState = LSTATE_AUTO_SHUTDOWN_UNDER_TEMP;
  else if ( temp >= upperLimit )                                tempState = LSTATE_AUTO_SHUTDOWN_OVER_TEMP;
  
  if ( tempState )
  {
    state = tempState;
    strcpy_P(lastShutdownReasonDetail, sensorName);
    
    char tmp[40];
    
    char tempStr[20], lowerLimitStr[20], upperLimitStr[20];
    ftoa(tempStr, (double)temp, 2);
    ftoa(lowerLimitStr, (double)lowerLimit, 2);
    ftoa(upperLimitStr, (double)upperLimit, 2);

    sprintf(tmp, ": %s (%s - %s)", tempStr, lowerLimitStr, upperLimitStr);
    strcat(lastShutdownReasonDetail, tmp);
  }
}



//Checks water, temperature and wind sensors, and shuts down the system if not within levels
//Called every second

void sensorCheck()
{
  checkWaterTankWithinLevels(WATER_LEVEL1_PIN);
  checkWaterTankWithinLevels(WATER_LEVEL2_PIN);
  
  sensorCheckTempWithinRange(string_temperature_14, outsideThermometer,    OUTSIDE_TEMP_LOWER_LIMIT,    OUTSIDE_TEMP_UPPER_LIMIT,    false);
  sensorCheckTempWithinRange(string_temperature_15, electronicsThermometer,ELECTRONICS_TEMP_LOWER_LIMIT,ELECTRONICS_TEMP_UPPER_LIMIT,false);
  sensorCheckTempWithinRange(string_temperature_16, stepper1Thermometer,   STEPPER_TEMP_LOWER_LIMIT,    STEPPER_TEMP_UPPER_LIMIT,    false);
//sensorCheckTempWithinRange(string_temperature_17, stepper2Thermometer,   STEPPER_TEMP_LOWER_LIMIT,    STEPPER_TEMP_UPPER_LIMIT,    true);
  sensorCheckTempWithinRange(string_temperature_18, led1Thermometer,       LED_TEMP_LOWER_LIMIT,        LED_TEMP_UPPER_LIMIT,        false);
//sensorCheckTempWithinRange(string_temperature_19, led2Thermometer,       LED_TEMP_LOWER_LIMIT,        LED_TEMP_UPPER_LIMIT,        true);
}



void sceneFlashTimer()
{
  lastFlashAlarm = dtINVALID_ALARM_ID;
  
  if      ( state != LSTATE_SCENE_RUNNING )  return;
  else if ( laminarScene == LSCENE_NIGHT_REGULAR_FLASH )
  {
    stepperFlash(&stepper1, STEPPER_FLASH_AMOUNT);
    lastFlashAlarm = Alarm.timerOnce(NIGHT_FLASH_REGULAR_PERIOD, sceneFlashTimer);
  }      
  else if ( laminarScene == LSCENE_NIGHT_RANDOM_FLASH )
  {
    stepperFlash(&stepper1, STEPPER_FLASH_AMOUNT);
    lastFlashAlarm = Alarm.timerOnce(random(NIGHT_FLASH_LOWER_RANDOM, NIGHT_FLASH_UPPER_RANDOM), sceneFlashTimer);
  }
}



void sceneCutTimer()
{
  lastCutAlarm = dtINVALID_ALARM_ID;

  if      ( state != LSTATE_SCENE_RUNNING )  return;
  else if ( laminarScene == LSCENE_DAY_SLOW_CUT )
  {
    stepperCut(&stepper1, false);
    Alarm.delay(500);
    stepperCut(&stepper1, true);
    lastCutAlarm = Alarm.timerOnce(DAY_SLOW_CUT_PERIOD, sceneCutTimer);
  }      
  else if ( laminarScene == LSCENE_DAY_FAST_CUT )
  {
    stepperCut(&stepper1, false);
    Alarm.delay(500);
    stepperCut(&stepper1, true);
    lastCutAlarm = Alarm.timerOnce(DAY_FAST_CUT_PERIOD, sceneCutTimer);
  }      
  else if ( laminarScene ==  LSCENE_DAY_RANDOM_CUT )
  {
    stepperCut(&stepper1, false);
    Alarm.delay(500);
    stepperCut(&stepper1, true);
    lastCutAlarm = Alarm.timerOnce(random( DAY_RANDOM_CUT_LOWER_RANDOM,  DAY_RANDOM_CUT_UPPER_RANDOM), sceneCutTimer);
  }
}



void sceneSetup()
{
  switch ( laminarScene )
  {
    case LSCENE_NONE:
      //Do nothing
      break;
      
    case LSCENE_TEST:
      //Do nothing
      break;
      
    case LSCENE_NIGHT_PLAIN:
      //Do nothing
      break;
      
    case LSCENE_NIGHT_VIBRATION:
      stepperVibration(&stepper1, 3);
      break;
      
    case LSCENE_NIGHT_REGULAR_FLASH:
      sceneFlashTimer();
      break;
      
    case LSCENE_NIGHT_RANDOM_FLASH:
      sceneFlashTimer();
      break;
      
    case LSCENE_DAY_PLAIN:
      //Switch LED off
      #ifdef BLINKM
        blinkMSwitchAllOff();
      #endif
      break;
      
    case LSCENE_DAY_SLOW_CUT:
      //Switch LED off
      #ifdef BLINKM
        blinkMSwitchAllOff();
      #endif
      sceneCutTimer();
      break;
      
    case LSCENE_DAY_FAST_CUT:
      //Switch LED off
      #ifdef BLINKM
        blinkMSwitchAllOff();
      #endif
      sceneCutTimer();
      break;
      
    case LSCENE_DAY_RANDOM_CUT:
      //Switch LED off
      #ifdef BLINKM
        blinkMSwitchAllOff();
      #endif
      sceneCutTimer();
      break;
      
    default:
      //Should never be reached
      break;
  }
}



void sceneShutdown()
{
  laminarScene = LSCENE_NONE;
  
  //Kill any pending alarms for cutting or flashing
  if ( lastCutAlarm != dtINVALID_ALARM_ID )
  {
    Alarm.disable(lastCutAlarm);
    lastCutAlarm = dtINVALID_ALARM_ID;
  }
  
  if ( lastFlashAlarm != dtINVALID_ALARM_ID )
  {
    Alarm.disable(lastFlashAlarm);
    lastFlashAlarm = dtINVALID_ALARM_ID;
  }
  
  //Switch vibration off, open up the stream and disable steppers temporarily
#ifdef STEPPER   
  if ( state == LSTATE_SCENE_RUNNING )
  {
    stepperVibration(&stepper1, 0);
    stepperCut(&stepper1, false);
    disableSteppers();
  }
#endif
}



void doState()
{
  switch ( state )
  {
    case LSTATE_TRANSITION_OFF:
      //Transition state to set everything off
      switchEverythingOff();
      state = LSTATE_OFF;
      break;

    case LSTATE_OFF:                      
      //Everything off - So Do Nothing
      break;
    
    case LSTATE_TRANSITION_STARTUP:
      //Transition to the startup state
      //Switch pump on and cut the stream
      stateTimer = now();
      
      #ifdef PUMPS
        pumpsSwitch(true);
      #endif
      #ifdef STEPPER
        stepperCut(&stepper1, true);
      #endif
      
      state = LSTATE_STARTUP;
      break;
    
    case LSTATE_STARTUP:                  
      //Pump has started, but jet not active yet
      if ( now() > (stateTimer +  PUMP_STARTUP_TIME))
      {
        #ifdef STEPPER
          stepperCut(&stepper1, false);
        #endif
        state = LSTATE_SCENE_RUNNING;
        sceneSetup();
      }
      break;
    
    case LSTATE_TRANSITION_SHUTDOWN:
      //Transition to the shutdown state
      
      sceneShutdown();
      //Cut the stream, wait for delay then switch the pump off
      stateTimer = now();
      
      #ifdef STEPPER
        stepperCut(&stepper1, true);
      #endif
      
      state = LSTATE_SHUTDOWN;
      break;
      
    case LSTATE_SHUTDOWN:               
      //Activate cutter, wait a bit then shutdown pump
      if ( now() > (stateTimer +  PUMP_SHUTDOWN_TIME))
      {
        #ifdef PUMPS
          pumpsSwitch(false);
        #endif
        
        state = LSTATE_TRANSITION_OFF;
      }
      break;
      
    case LSTATE_AUTO_SHUTDOWN_OVER_TEMP:
      //Auto shutdown due to over temperature
      lastShutdownReason = string_general_19; 
      state = LSTATE_TRANSITION_SHUTDOWN;
      break;

    case LSTATE_AUTO_SHUTDOWN_UNDER_TEMP:
      //Auto shutdown due to under temperature
      lastShutdownReason = string_general_25; 
      state = LSTATE_TRANSITION_SHUTDOWN;
      break;
      
    case LSTATE_AUTO_SHUTDOWN_NO_TEMP:
      //Auto shutdown due to no temperature reading
      lastShutdownReason = string_general_26; 
      state = LSTATE_TRANSITION_SHUTDOWN;
      break;
      
    case LSTATE_AUTO_SHUTDOWN_WATER_LEVEL_LOW:
      //Auto shutdown due to water level being too low
      lastShutdownReason = string_general_20; 
      state = LSTATE_TRANSITION_SHUTDOWN;
      break;

    case LSTATE_AUTO_SHUTDOWN_WATER_LEVEL_HIGH:
      //Auto shutdown due to water level being too high
      lastShutdownReason = string_general_21; 
      state = LSTATE_TRANSITION_SHUTDOWN;
      break;
      
    case LSTATE_AUTO_SHUTDOWN_WIND:
      //Auto shutdown due to wind
      lastShutdownReason = string_general_22; 
      state = LSTATE_TRANSITION_SHUTDOWN;
      break;
      
    case LSTATE_SCENE_RUNNING:
      //Running a scene
      break;
      
    default:
      //Should never happen
      break;
  }
}



void setup()
{
  Serial.begin(57600);

  //So we can use the WiShield on the Mega 2560.  We need put D10, D11, D12, D13 to high impedence
  //D10, D11, D12, D13 are then jumpered to the I2C bus on the Mega which is on pins D50-D53
  pinMode(D10, INPUT);
  pinMode(D11, INPUT);
  pinMode(D12, INPUT);
  pinMode(D13, INPUT);

#ifdef WISERVER
  SerialPrint_P(string_general_15);

  // Initialize WiServer and have it use the sendMyPage function to serve pages
  WiServer.init(sendMyPage);  //CALL BEFORE Serial.begin !!

  SerialPrintln_P(string_general_13);
#endif

#ifdef TEMPERATURE
  //Startup temperatures
  setupTemperatures();
#endif

#ifdef WATER_LEVEL
  //Startup water level
  pinMode(WATER_LEVEL1_PIN, INPUT);
  pinMode(WATER_LEVEL2_PIN, INPUT);
#endif

#ifdef WIND
  //Start wind monitoring
  pinMode(WIND_PIN, INPUT);
  clearWindSensorHistory();
#endif

#ifdef PUMPS
  //Startup pumps
  pinMode(PUMPS_PIN, OUTPUT);
  pumpsSwitch(false);
#endif

#ifdef THUMPER
  //Startup thumper
  pinMode(THUMPER_PIN, OUTPUT);
  thumperSwitch(false);
#endif

#ifdef BLINKM
  SerialPrintln_P(string_general_16);

  resetI2CBus();

  BlinkM_begin();
  delay(100);  //wait a while for BlinkM to stabilize

  //Uncomment to set BlinkMAddress (make sure only 1 BlinkM is connect to the I2C bus
  //BlinkM_setAddress(0x01);

  BlinkM_scanI2CBus(1,60, scanfunc);

  blinkMSwitchAllOff();
#endif

#ifdef STEPPER
  //Setup the stepper motors

  pinMode(stepperDisablePin, OUTPUT);

  enableSteppers();

  setupStepper(&stepper1, 1);
  //    setupStepper(&stepper2, 2);

  findStepperZero(&stepper1, 1, stepper1CorrectionToCenter);
  //    findStepperZero(&stepper2, 2, stepper2CorrectionToCenter);

  disableSteppers();
#endif

  strcpy(lastShutdownReasonDetail, "");

  setTime(0);
  
  //Setup the analog sensor reading
  analogRead(analogSensorsPins[0]);
  Alarm.timerRepeat(ANALOG_SENSOR_READ_FREQUENCY, analogSensorRead);	    // timer to read sensors every 15 seconds
  Alarm.timerRepeat(SENSOR_CHECK_FREQUENCY,       sensorCheck);	            // timer to check sensors are valid every second
#ifdef TEMPERATURE
  Alarm.timerRepeat(TEMPERATURE_READ_INTERVAL,    readTemperatures);       // timer to read the temperatures every TEMPERATURE_READ_INTERVAL seconds
#endif
  Alarm.timerRepeat(TEMPERATURE_READ_INTERVAL,    freeMemoryReport);

  SerialPrintln_P(string_general_17);
}



void loop()
{  
#ifdef WISERVER
  // Run WiServer
  WiServer.server_task();
#endif

#ifdef STEPPER
  stepperVibrationPoll(&stepper1);
  stepper1.run();
  //stepper2.run();
#endif
  
  doState();

  Alarm.delay(LOOP_DELAY);
}



