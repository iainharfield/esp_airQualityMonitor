// Useful sites:
// https://uk-air.defra.gov.uk/air-pollution/daqi?view=more-info&pollutant=pm10#pollutant

// https://wiki.dfrobot.com/SHT31_Temperature_Humidity_Sensor_Weatherproof_SKU_SEN0385



#include <ArduinoOTA.h>
#include <Ticker.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient_Generic.hpp>

#include <DFRobot_SHT3x.h>
#if defined(ESP32)
    #define ESP_DRD_USE_LITTLEFS  true
    #define ESP_DRD_USE_SPIFFS    false
    #define ESP_DRD_USE_EEPROM    false
#else
  #define USE_LITTLEFS            true
  #define ESP_DRD_USE_LITTLEFS    true
#endif
#define DOUBLERESETDETECTOR_DEBUG true
#define DRD_TIMEOUT 4
#define DRD_ADDRESS 0
#include <ESP_DoubleResetDetector.h>
#include "PMS.h"
#include <SoftwareSerial.h>

#include "hh_defines.h"   //in the deiceFramework library
#include "hh_utilities.h"
#include "hh_cntrl.h"

//***********************
// Application functions
//**********************
void getSHT3xReadings();
void readPMS();
void wakeupPMS();
String readData();
boolean readPMSdata(Stream *s);
String createJSONmessage();
void publishResults();

//*******************************************************************
//functions that MUST exist here even if not used by this app.
//*******************************************************************
bool onMqttMessageAppExt(char *, char *, const AsyncMqttClientMessageProperties &, const size_t &, const size_t &, const size_t &); 
bool onMqttMessageAppCntrlExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total);
void appMQTTTopicSubscribe();
void telnet_extension_1(char);      // Required by template
void telnet_extension_2(char);      // Required by template
void telnet_extensionHelp(char);    // Required by template
void processTOD_Ext();              // Required by template

//****************************************
// functions that can be calleed in hh_asyncConnect.cpp
//***************************************
extern void platform_setup(bool);
extern void printTelnet(String);
extern void handleTelnet();
extern AsyncMqttClient mqttClient;

DFRobot_SHT3x sht3x(&Wire,/*address=*/0x44,/*RST=*/4);
DoubleResetDetector *drd;
devConfig espDevice;
Ticker sht3xReadTimer;
Ticker pmsRead;
Ticker pmsDelay;
Ticker aqmPublish;

// Application Specific MQTT Topics and config
const char *oh3StateValue = "/house/aqm/aqm-1/value";  //FIXTHIS
//const char *oh3StateValue = "/house/aqm/aqm-1/value";  //FIXTHIS
String deviceName = "aqm-1";
String deviceType = "AQM";

float sht3xTemp = 0;
float sht3xHumidity = 0;

//****************
// PSM5003
//****************
#define UARTRX 34       // goes to PSM5003 TXD, GPIO34 , A2
#define UARTTX 26       // goes to PSM5003 RXD, GPIO26 , A0 Outout capable pin
#define PMSRST 13       // PSM wake/sleep control
#define PMSWARMUP 30    // warmup time in seconds
#define PMSSLEEP  120   // put senor to sleep, to increase life time in sconds 
#define AQMPUBLISH 60   // Publish results timer
#define SHTREAD    30   // Read the SHt3x sesor to get Temp and humidity
//PSM5003 serial data format
struct pms5003data 
{
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data psm5003data = {0};

SoftwareSerial pmsSerial(UARTRX, UARTTX);  
PMS pms(pmsSerial);
PMS::DATA psmdata;
boolean softControl = false;  //false means use GPIO contol, true means use the software commands

//*******************************************************
// Setup the device and sensors. 
//*******************************************************
void setup(void)
{

    bool configWiFi = false;
    Serial.begin(115200);
    while (!Serial)
        delay(1000);
    
    // Set up DRD
    drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);
    if (drd->detectDoubleReset())
    {
        configWiFi = true;
        Serial.println("App: Double Reset detected");
    }

    // Set this deviceName and Type
    espDevice.setup(deviceName, deviceType);
    // Platform setup: Set up and manage all services: LittleFS, WiFi, MQTT and Telnet
    // The first time, there will be no WiFi confif so look foe SSID and configure device.
    platform_setup(configWiFi);
    //delay(5000);

    // Setup PMS5003
    pinMode(UARTTX, OUTPUT);
    pinMode(PMSRST, OUTPUT);
    
    pmsSerial.begin(9600);
    pms.passiveMode();

    // Default state after sensor power, but undefined after ESP restart e.g. by OTA flash, so we have to manually wake up the sensor for sure.
    // Some logs from bootloader is sent via Serial port to the sensor after power up. This can cause invalid first read or wake up so be patient and wait for next read cycle.
    if (softControl) 
    {
      pms.wakeUp();
    }
    else
    {
      digitalWrite(PMSRST, HIGH);
    }

    pmsRead.once(PMSWARMUP,readPMS);

    // Setup SHT3x
    while (sht3x.begin() != 0) 
    {
      Serial.println("Failed to initialize the chip, please confirm the chip connection");
      delay(1000);
    }   

  if(!sht3x.softReset())
  {
    Serial.println("Failed to reset the chip");
  }
  else
  {
    Serial.print("chip serial number: ");
    Serial.println(sht3x.readSerialNumber());
  }
  sht3xReadTimer.attach(SHTREAD,getSHT3xReadings); // read every xx second
  aqmPublish.attach(AQMPUBLISH,publishResults);
}

/*
 * Main function, get and show the temperature
 */
void loop(void)
{ 
    drd->loop();

   // Go look for OTA request
    ArduinoOTA.handle();

    handleTelnet();

    delay(1000);            // Seems to be required for DRD TO WORK when used with Ticker ???? - False DRD detection
}
 

// Invoked by Ticker Event timer
void getSHT3xReadings()
{
  char logString[MAX_LOGSTRING_LENGTH];

  sht3xTemp = sht3x.getTemperatureC();
  sht3xHumidity = sht3x.getHumidityRH();
  Serial.print("Ambient Temperature(°C):");
  Serial.print(sht3xTemp);
  Serial.print(" ");
  Serial.print("Relative Humidity(%):");
  Serial.println(sht3xHumidity);

  //publishResults();
}

void readPMS()
{
    readData(); // read the data from PMS
    Serial.println("Put PSM to sleep.");
    if (softControl) { pms.sleep(); }
    else {digitalWrite(PMSRST, LOW);}
    
    pmsDelay.once(PMSSLEEP, wakeupPMS);  //FIX THIS PSM PMS
}
void wakeupPMS()
{
    //readData();
    Serial.println("Warmup PSM and start 30s timer before reading.");
    //readData();  // this should fail !
    if (softControl) 
    {
      pms.wakeUp();
    }
    else 
    {
      digitalWrite(PMSRST, HIGH);
    }
    
    pmsRead.once(PMSWARMUP,readPMS);
}
String readData()
{
  PMS::DATA data;

  // Clear buffer (removes potentially old data) before read. Some data could have been also sent before switching to passive mode.
  while (Serial.available()) { Serial.read(); }
  Serial.println("App: read PMS5003.");

  //Serial.println("Send read request...");

  pms.requestRead();
  delay(300);
  

  //while (readPMSdata(&pmsSerial) == false) // fixthis - need to exit
  if (readPMSdata(&pmsSerial) == true) 
  {
    return (createJSONmessage());
    /*
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(psm5003data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(psm5003data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(psm5003data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(psm5003data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(psm5003data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(psm5003data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(psm5003data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(psm5003data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(psm5003data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(psm5003data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(psm5003data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(psm5003data.particles_100um);
    Serial.println("---------------------------------------");
  */
  }
  else
  {
    Serial.println("......No data.......");
    return "No Data";  //FIXTHIS
  }
}



boolean readPMSdata(Stream *s) 
{
  Serial.println("Reading data...");

  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
  /*
  // debugging
  Serial.println("Debug data");
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  //end debugging
  */
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) 
  {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into struct :)
  memcpy((void *)&psm5003data, (void *)buffer_u16, 30);
 
  if (sum != psm5003data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  Serial.println("returning true");
  return true;
}

void publishResults()
{
    String output;
    /*
    char buf[6];
    char logString[MAX_LOGSTRING_LENGTH];

    memset(logString, 0, sizeof logString);
    //convert float to string
    dtostrf(sht3xTemp,5,2,buf);
    //sprintf(logString, "%s%s\r", "Temperature °C:\t\t\t", buf);


    sprintf(logString, "%s,%u,%u,%u,%u,%u,%u",buf,
                        psm5003data.pm10_standard,psm5003data.pm25_standard,psm5003data.pm100_standard, 
                        psm5003data.pm10_env,psm5003data.pm25_env,psm5003data.pm100_env);
    //printTelnet((String)logString); 
  */
    output = createJSONmessage();
    mqttClient.publish(oh3StateValue, 0, true, output.c_str());
} 


String createJSONmessage()
{
  String output;
  StaticJsonDocument<270> doc;
  doc["temprature"] = sht3xTemp;
  doc["humidity"] = sht3xHumidity;
  doc["standadPM10"] = psm5003data.pm10_standard;
  doc["standadPM25"] = psm5003data.pm25_standard;
  doc["standadPM100"] = psm5003data.pm100_standard;
  doc["environmentalPM10"] = psm5003data.pm10_env;
  doc["environmentalPM25"] = psm5003data.pm25_env;
  doc["environmentalPM100"] = psm5003data.pm100_env;
  doc["particles03"] = psm5003data.particles_03um;
  doc["particles05"] = psm5003data.particles_05um;
  doc["particles10"] = psm5003data.particles_10um;
  doc["particles25"] = psm5003data.particles_25um;
  doc["particles50"] = psm5003data.particles_50um;
  doc["particles100"] = psm5003data.particles_100um;

  serializeJsonPretty(doc, Serial);
  Serial.println();
  serializeJson(doc, output);
  return(output);
}



//************************************************************
// Process any application specific inbound MQTT messages
// Return False if none
// Return true if an MQTT message was handled here
//************************************************************
bool onMqttMessageAppExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total)
{
    Serial.println("onMqttMessageAppExt: User exit enabling App to do stuff");
    return false;
}
bool onMqttMessageAppCntrlExt(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties, const size_t &len, const size_t &index, const size_t &total)
{
  Serial.println("onMqttMessageAppCntrlExt: User exit enabling App to het Controller to do stuff");
	return false;
}
//***********************************************************
// Subscribe to application specific topics
//***********************************************************
void appMQTTTopicSubscribe()
{
    // mqttTopicsubscribe(oh3StateValue, 2);
}

//********************************************************************
// Write out ove telnet session and application specific infomation
//********************************************************************
void telnet_extension_1(char c)
{
    char buf[6];
    char logString[MAX_LOGSTRING_LENGTH];
    memset(logString, 0, sizeof logString);
    //convert float to string
    dtostrf(sht3xTemp,5,2,buf);
    sprintf(logString, "%s%s\r", "Temperature (°C):\t\t", buf);
    printTelnet((String)logString);
    dtostrf(sht3xHumidity,5,2,buf);
    sprintf(logString, "%s%s\r", "Relative Humidity (%):\t\t", buf);
    printTelnet((String)logString);
    sprintf(logString, "%s%s%u,%s%u,%s%u\r","Standard Concentration:\t\t","PM1.0 = ",psm5003data.pm10_standard," PM2.5 = ",psm5003data.pm25_standard," PM10 = ",psm5003data.pm100_standard );
    printTelnet((String)logString); 
    sprintf(logString, "%s%s%u,%s%u,%s%u\n\r","Environmental Concentration:\t","PM1.0 = ",psm5003data.pm10_env," PM2.5 = ",psm5003data.pm25_env," PM10 = ",psm5003data.pm100_env );
    printTelnet((String)logString); 


    sprintf(logString, "%s%u\r","Particles > 0.3um / 0.1L air:\t",psm5003data.particles_03um);
    printTelnet((String)logString); 
    sprintf(logString, "%s%u\r","Particles > 0.5um / 0.1L air:\t",psm5003data.particles_05um);
    printTelnet((String)logString); 
    sprintf(logString, "%s%u\r","Particles > 1.0um / 0.1L air:\t",psm5003data.particles_10um);
    printTelnet((String)logString); 
    sprintf(logString, "%s%u\r","Particles > 2.5um / 0.1L air:\t",psm5003data.particles_25um);
    printTelnet((String)logString); 
    sprintf(logString, "%s%u\r","Particles > 5.0um / 0.1L air:\t",psm5003data.particles_50um);
    printTelnet((String)logString); 
    sprintf(logString, "%s%u\n\r","Particles > 10.0um / 0.1L air:\t",psm5003data.particles_100um);
    printTelnet((String)logString); 

}

//*****************************************************
// Process any application specific telnet commannds
//*****************************************************
void telnet_extension_2(char c)
{
    printTelnet((String)c);
}

//************************************************************************************************
// Prints out any application specific telenet command help instructions this app has implemented
//************************************************************************************************
void telnet_extensionHelp(char c)
{
    printTelnet((String) "x\t\tSome description");
}

void drdDetected()
{
    Serial.println("Double reset detected");
}

//************************************************************************
// Implement any Application Specific TOD behaviour
// This exit also calls the created controller TOD functions
// For example if you have created three time contollers you need to call
// <cntrlname1>.processCntrl_Ext();
// <cntrlname2>.processCntrl_Ext();
// <cntrlname3>.processCntrl_Ext();
//
// FIXTHIS - Must be a better way
//************************************************************************
void processCntrlTOD_Ext()
{
    mqttLog("Application Processing Controller TOD functions", true, true);
    // Call the TOD function for each controller crteated by this app
    //USCntrlState.processCntrlTOD_Ext();
}