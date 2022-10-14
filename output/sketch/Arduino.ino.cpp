#include <Arduino.h>
#line 1 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
/*=================================================================================================
 * File:            17553_ArdLogger.ino
 * Author:          G. Seigers / J. de Pagter
 * Created date:    -
 * Description:     Arduino program to run the MKR1500 NB with GPRS to Azure cloud (MQTT) to read
 * some sensors for remote monitoring
=================================================================================================*/

#include <MKRNB.h>
#include <ArduinoECCX08.h>
#include <ArduinoBearSSL.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_ADS1X15.h>
#include <Modem.h>
#include <utility/ECCX08SelfSignedCert.h>
#include "iot_configs.h"
#include "Params.h"
#include "BootMenu.h"
#include "DataRecord.h"
#include "Sodaq_wdt.h"
#include <ArduinoJson.h>
#include <cstring>
#include <SPI.h>
#include <SD.h>
#include <time.h>

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

#define PROJECT_NAME "Project Marien"
#define VERSION "1.1.0"
#define STARTUP_DELAY 20000 // 20 seconden om te booten
#define CONSOLE_STREAM SerialUSB

#define MAX_NUMBER_OF_MEASUREMENTS 15 // Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS
#define DEFAULT_MEASUREMENT_INTERVAL   8000
#define DEFAULT_NUMBER_OF_MEASUREMENTS 1
#define DEFAULT_REPEATS 60000

#define DEBUG 1
#define REGISTERED 1
#define DEVICE_NAME "A04072210" // THIS CODE MUST CHANGED FOR EVERY ARDUIO !!!!!
#define MQTT_BROKER "euw-iothub-rdfv-pr.azure-devices.net"
#define USE_GPS 1
#define USE_LED 1
#define PIN_NUMBER "915684"
#define APN_A "iot.t-mobile.nl"

#define MIN_SIGNAL_STRENGTH                 15      // get from modem the detected carrier (31 > 51 dBm)
#define NO_SIGNAL_STRENGTH                  99      // What signal is detected when modem starts with no antenne or still initializing

//#define PORT1 "T"
#define PORT2 "CO2"
#define PORT3 "H"
#define PORT4 "T"
//#define PORT5 "G"
#define PORT6 "NH3"
// #define PORT7 ""
// #define PORT8 ""

#define USEADC1 1
#define USEADC2 1

#define GAIN1 2 // Range for 0 to 2048 mV
#define GAIN2 2 // Range from 0 to 2048 mV

//Used for SD card
const int chipSelect = 4;

// Translate iot-configs.h defines into variabels
static const char *host = IOT_CONFIG_IOTHUB_FQDN;
static const char *device_id = IOT_CONFIG_DEVICE_ID;
static const char *apn = LTE_APN;

/* Define the constants for the IOTHub and T-Mobile */
const char PINNUMBER[] = "031591"; // PIN Number
// const char APN[]              = "iot.t-mobile.nl";                                               // APN T-Mobile

const int delayBetweenMessages = 500; // Wait time im ms
String response;
static uint8_t lastResetCause; // Veld waarin staat wat de oorzaak is van de laatste reset.
String IMEI = "";
String DeviceName; 
String MQTTBroker = "";
String Error = "";
char sendBuffer[500];
bool sdPresent = false;

// DataRecord measurements[MAX_NUMBER_OF_MEASUREMENTS];
// int measurementPointer = 0; // Actueel aantal metingen dat gedaan is.

//Variables for reading the ports
int16_t  P1;
int16_t  CO2;                //Port2 p1_2
int16_t  H;                  //Port3 p1_3 
int16_t  T;                  //Port4 p1_4
int16_t  P5; 
int16_t  NH3;                //Port6 p2_2
int16_t  P7;
int16_t  P8;
int32_t  MeasurementTime;     //Time of measurement
int16_t  repeatCounter;      // 
int16_t  wdtCounter;

GPRS gprs;
NB            nbAccess(false);                // Set op true om te debuggen
NBClient      nbClient;                 // Used for the TCP socket connection
BearSSLClient sslClient(nbClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient    mqttClient(sslClient);
NBModem       modem;
NBScanner     nbScanner;

// Variabels for MQTT
String username;
String broker;

Adafruit_ADS1015 ads1115_48;
Adafruit_ADS1015 ads1115_49;

/* Declare the function that are needed in the sketch */
unsigned long getTime();
void onMessageReceived(int);
static void printBootUpMessage(Stream &stream);
void handleBootUpCommands();
void publishMessage();
void connectMQTT();
bool initGPS();
void onConfigReset(void);
// void publishMessage(DataRecord *record);
void onMessageReceived(int);
void blinkLed(int);
void connectNB();
void publishRegistration();
void sendRegistrationRequest();
void sendErrorToEndpoint();
void setupModem();
void writeToLogFile(String message);
void writeToDataFile(unsigned long time, int16_t H, int16_t T, int16_t CO2, int16_t NH3, int16_t P1, int16_t P5, int16_t P7, int16_t P8);

// void setGain(Adafruit_ADS1115 &sensor, uint8_t gain);
// double getMultiplier(int portNumber); // Returns the multiplier for this specific portnumber

void (*resetFunc)(void) = 0; // Functie voor harde reset. Wordt aangeroepen als buffer overloopt.

#line 149 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void setup();
#line 269 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void loop();
#line 511 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void publishSettings();
#line 735 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void getSensorData();
#line 797 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void onMessageReceived(int messageSize);
#line 893 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void blinkLed(int times);
#line 1084 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
char stringTochar(String s);
#line 1091 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
int freeMemory();
#line 149 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void setup()
{
  Serial.begin(115200);
  
  //Initialize SD card.
  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    sdPresent = false;
    // don't do anything more:
  }
  else
  {
    sdPresent = true;
    Serial.println("card initialized.");
  }

  lastResetCause = PM->RCAUSE.reg;
  blinkLed(5);

  ads1115_48.begin(0x48);
  ads1115_49.begin(0x49);

    // if coming from WDT reset!
  if (lastResetCause & PM_RCAUSE_WDT)
  {
    Serial.println("WDT reset executed!");
    writeToLogFile("WDT reset executed!");
  }
  else
  {
    Serial.println("Normal POR executed!");
    writeToLogFile("Normal POR executed!");
    setupModem();
  }

  // Disable the WDT tijdens het setup proces
  Serial.println("WDT is disabled");
  sodaq_wdt_disable();

  Serial1.begin(115200);
  params.read();




  // init params

  // init params
  // sodaq_wdt_disable();
  params.setConfigResetCallback(onConfigReset);
  params.read();
  handleBootUpCommands();
  // sodaq_wdt_enable(WDT_PERIOD_8X);

  /* Initialize the ECCX08 encrytion chip and reconstyruct the selfsigned certificate */
  if (!ECCX08.begin())
  {
    Serial.println("No ECCX08 present!");
    writeToLogFile("WDT reset executed!");
    while (1);
  }
  // Wait for the sensor to be ready

  ECCX08SelfSignedCert.beginReconstruction(0, 8);
  ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
  ECCX08SelfSignedCert.endReconstruction();
  ArduinoBearSSL.onGetTime(getTime); // Set a callback fur the current time. So the certificate can be validated
  sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

  // Compose the username
  // username += broker; username += "/"; username += deviceId; username += "/api-version=2018-06-30";

  // username += params.getMQTTBroker();
  if (params._isDeviceRegistered == true)
  {
    Serial.println("SETUP: Device is already registred");
    DeviceName = params._deviceName;
    MQTTBroker = params.getMQTTBroker();
  }

  username += MQTTBroker;
  username += "/";
  username += DeviceName;
  username += "/api-version=2018-06-30";

  Serial.print("Username  :");
  Serial.println(username);

  // Set the client id used for MQTT as the device id
  mqttClient.setId(DeviceName);
  mqttClient.setUsernamePassword(username, "");
  mqttClient.onMessage(onMessageReceived); // Call back if an mesaage is received from the IOT-HUB

  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(115200);
  Serial.begin(115200);

  sodaq_wdt_disable();
  Serial.println("WDT Period 8X");
  sodaq_wdt_enable(WDT_PERIOD_8X); // WDT is gezet op 8 seconden.
  Serial.println("WDT Reset");
  sodaq_wdt_reset();

  ads1115_48.setGain(GAIN_TWO);
  ads1115_49.setGain(GAIN_TWO);

  repeatCounter = 0;

  modem.begin();
  connectNB();
}

//=================================================================================================
// Function:    loop
// Return:      -
// Description: Main loop function within Arduino environment
//=================================================================================================
void loop()
{

  if (sdPresent==true){
    Serial.println("Writing to SD file");
  } else {
    Serial.println("No SD present.");
  }

  Serial.println("Entering loop");
  writeToLogFile("Entering loop");

  sodaq_wdt_reset(); // Reset de WDT
  // Make a connection

  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY)
  {
    Serial.println("No connection with T-Mobile");
    writeToLogFile("No connection with T-Mobile");
    connectNB();
  }

  // Controleer of de MQTT Client beschikbaar is. Als er een timeout is dan moet er iets worden gedaan? TODO
  if (!mqttClient.connected())
  {
    Serial.println("No connection with Azure");
    writeToLogFile("No connection with Azure");
    connectMQTT();
  }

  blinkLed(2);
  Serial.println("Device is registred. Starting measurements now.");

  Serial.println("Reading ports");
  
  getSensorData();

  // If the current measurementPointer is greater od equal then the buffer then send the data toi Azure.
  // Serial.print("Measurement pointer :");
  // Serial.println(measurementPointer);
  // Serial.print("Default number of measurements");
  // Serial.println(params._defaultNumberOfMeasurements);

  Serial.println("Checking signal strength");
  writeToLogFile("No connection with T-Mobile");

  String signalStrength = "Signal strength :" + String(nbScanner.getSignalStrength().toInt()) + " db";
  Serial.println(signalStrength);

  writeToLogFile(signalStrength);
  if (nbScanner.getSignalStrength().toInt() != NO_SIGNAL_STRENGTH && nbScanner.getSignalStrength().toInt() > MIN_SIGNAL_STRENGTH  )
  {
    Serial.println("Send message to IOT-HUB enough signal strength");
    writeToLogFile("Send message to IOT-HUB enough signal strength");
    mqttClient.poll();
    sodaq_wdt_safe_delay(200);
    publishMessage();
  }
  else
  {
    Serial.println("Not enough signal to send to Azure");
    writeToLogFile("Not enough signal to send to Azure");  
  }

  Serial.println("Entering waiting loop...");
  sodaq_wdt_safe_delay(params._defaultMeasurementInterval);
  repeatCounter = repeatCounter + 1;
}

//=================================================================================================
// Function:    getTime
// Return:
// Description: Get time from NarrowBand module
//=================================================================================================
unsigned long getTime()
{

  struct tm t = {0};
  time_t epoch;
  int YEAR, MONTH, DAY, HOUR, MINUTE, SECONDS, TZ;
  char timeBuffer[50];

  MODEM.send("AT+CCLK?");
  MODEM.waitForResponse(2000, &response);
  Serial.print("Time: ");
  Serial.println(response);

  int x = response.indexOf(String('"'))+1;                  //Get rit of AT stuff
  int y = response.lastIndexOf(String('"'));
  response.substring(x,y).toCharArray(timeBuffer,50);       //Write string to char buffer

  //const char *timeData = "22/03/02,14:17:40+22"; 
  sscanf(timeBuffer, "%d/%d/%d,%d:%d:%d+%d", &YEAR, &MONTH, &DAY, &HOUR, &MINUTE, &SECONDS, &TZ);

  t.tm_year = YEAR + 100;
  t.tm_mon = MONTH - 1;
  t.tm_mday = DAY;
  t.tm_hour = HOUR;
  t.tm_min = MINUTE;
  t.tm_sec = SECONDS;
  epoch = mktime(&t);     // -19800 TZ correction To get UTC

  return ((unsigned long)epoch);
}

//=================================================================================================
// Function:    printBootUpMessage
// Return:      -
// Description: Prints a boot-up message that includes project name, version and Cpu reset cause.
//=================================================================================================
static void printBootUpMessage(Stream &stream)
{
  stream.println("** " PROJECT_NAME " - " VERSION " **");

  stream.println();

  stream.print(" -> ");
  // printCpuResetCause(stream);

  stream.println();
}

//=================================================================================================
// Function:    onConfigReset
// Return:
// Description: [CALLBACK] The config has been reset, fill with default data
//=================================================================================================
void onConfigReset(void)
{

  /* CHANGE _params._isDeviceRegistered = false; wheen in production !!!! */

#ifdef REGISTERED
  params._isDeviceRegistered = true;
#endif

#ifdef DEVICE_NAME
  strcpy(params._deviceName, DEVICE_NAME);
#endif

#ifdef PIN_NUMBER
  strcpy(params._pinnumber, PIN_NUMBER);
#endif

#ifdef APN_A
  strcpy(params._apn, APN_A);
#endif

#ifdef DEBUG
  params._isDebugOn = true;
#endif

#ifdef MQTT_BROKER
  strcpy(params._endPointData, MQTT_BROKER);
#endif

#ifdef DEFAULT_MEASUREMENT_INTERVAL
  params._defaultMeasurementInterval = DEFAULT_MEASUREMENT_INTERVAL;
#endif

#ifdef DEFAULT_NUMBER_OF_MEASUREMENTS
  params._defaultNumberOfMeasurements = DEFAULT_NUMBER_OF_MEASUREMENTS;
#endif

#ifdef DEFAULT_REPEATS
  params._defaultRepeats = DEFAULT_REPEATS;
#endif

#ifdef USE_LED
  params._isLedEnabled = USE_LED;
#endif

#ifdef USEADC1
  params._useADC1 = true;
#endif

#ifdef USEADC2
  params._useADC2 = true;
#endif

#ifdef GAIN1
  params._gain_1 = GAIN1;
#endif

#ifdef GAIN2
  params._gain_2 = GAIN2;
#endif

#ifdef PORT1
  strcpy(params._p1_1, PORT1);
#endif

#ifdef PORT2
  strcpy(params._p1_2, PORT2);
#endif

#ifdef PORT3
  strcpy(params._p1_3, PORT3);
#endif

#ifdef PORT4
  strcpy(params._p1_4, PORT4);
#endif

#ifdef PORT5
  strcpy(params._p2_1, PORT5);
#endif

#ifdef PORT6
  strcpy(params._p2_2, PORT6);
#endif

#ifdef PORT7
  strcpy(params._p2_3, PORT7);
#endif

#ifdef PORT8
  strcpy(params._p2_4, PORT8);
#endif
}

//=================================================================================================
// Function:    handleBootUpCommands
// Return:      -
// Description: Shows and handles the boot up commands.
//=================================================================================================
void handleBootUpCommands()
{
  do
  {
    showBootMenu(CONSOLE_STREAM);
  } while (!params.checkConfig(CONSOLE_STREAM));

  params.showConfig(&CONSOLE_STREAM);
  params.commit();
}

//=================================================================================================
// Function:    publishSettings
// Return:      -
// Description: Send all settings to Azure cloud
//=================================================================================================
void publishSettings()
{
  unsigned long timeStamp = getTime();
  String endPoint;
  String deviceId;
  String imeiCode = "";
  String ICCID = "";

  deviceId = params.getDeviceName();
  imeiCode = modem.getIMEI();
  ICCID = modem.getICCID();

  strcpy(sendBuffer, "{");
  strcat(sendBuffer, "\"Type\":");
  strcat(sendBuffer, "\"Settings\"");
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"DeviceName\":");
  strcat(sendBuffer, "\"");
  strcat(sendBuffer, deviceId.c_str());
  strcat(sendBuffer,"\"");
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"IMeiCode\":");
  strcat(sendBuffer, "\"");
  strcat(sendBuffer, imeiCode.c_str()); 
  strcat(sendBuffer,"\"");
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"ICCID\":");
  strcat(sendBuffer, "\"");
  strcat(sendBuffer, ICCID.c_str());
  strcat(sendBuffer,"\"");
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"Timestamp\":");
  strcat(sendBuffer, String(getTime()).c_str() );
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"Buffer\":");
  strcat(sendBuffer, String(params._defaultNumberOfMeasurements).c_str() );
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"Interval\":");
  strcat(sendBuffer, String(params._defaultMeasurementInterval).c_str() );
  strcat(sendBuffer,"," );
  strcat(sendBuffer, "\"LastResetCause\":");
  strcat(sendBuffer, String(lastResetCause).c_str() );
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"Repeats\":");
  strcat(sendBuffer, String(params._defaultRepeats).c_str() );
  strcat(sendBuffer, "}");
  
  Serial.println('============================================================='); 
  Serial.println('New Json');
  Serial.println(sendBuffer);

  sodaq_wdt_reset(); // Reset de WDT

  Serial.println("Just before sending data");
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(sendBuffer);
  mqttClient.endMessage();
  Serial.println("End sending data");
}

//=================================================================================================
// Function:    sendErrorToEndpoint
// Return:      -
// Description: Send error to MQTT client
//=================================================================================================
void sendErrorToEndpoint()
{
  Serial.println("WDT save delay in sendError");
  sodaq_wdt_safe_delay(8000);
  char _errorMessage[128 + 1];
  unsigned long timeStamp = getTime();
  String endPoint;

  strcpy(_errorMessage, Error.c_str());
  StaticJsonDocument<128> doc;
  doc["type"] = "Error";
  doc["timestamp"] = getTime();
  doc["device"] = DeviceName;
  doc["message"] = _errorMessage;

  serializeJson(doc, Serial);
  Serial.println();

  endPoint = "devices/";
  endPoint += DeviceName;
  endPoint += "/messages/events/";

  mqttClient.beginMessage(endPoint);
  serializeJson(doc, mqttClient);
  mqttClient.endMessage();

  // Reset the error
  Error = "";
}


//=================================================================================================
// Function:    publishMessage
// Return:      -
// Description: Send data message to MQTT azure cloud
//=================================================================================================
void publishMessage()
{

  int signalStrength;
  signalStrength = nbScanner.getSignalStrength().toInt();

  unsigned long timeStamp = getTime();
  String deviceId;
  deviceId = params.getDeviceName();

  Serial.println("*** Entering publish message ***");

  Serial.print("Name of device:");
  Serial.println(params.getDeviceName());

  strcpy(sendBuffer, "{");
  strcat(sendBuffer, "\"Type\":");
  strcat(sendBuffer, "\"Data\"");
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"DeviceName\":");
  strcat(sendBuffer, "\"");
  strcat(sendBuffer, deviceId.c_str());
  strcat(sendBuffer,"\"");
  strcat(sendBuffer,"," );
  strcat(sendBuffer, "\"Timestamp\":");
  strcat(sendBuffer, String(getTime()).c_str() );
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"P1\":");
  strcat(sendBuffer, String((P1)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"CO2\":");
  strcat(sendBuffer, String((CO2)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"H\":");
  strcat(sendBuffer, String((H)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"T\":");
  strcat(sendBuffer, String((T)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"P5\":");
  strcat(sendBuffer, String((P5)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"NH3\":");
  strcat(sendBuffer, String((NH3)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"P7\":");
  strcat(sendBuffer, String((P7)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"P8\":");
  strcat(sendBuffer, String((P8)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"RC\":");
  strcat(sendBuffer, String((lastResetCause)).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"SS\":");
  strcat(sendBuffer, String((signalStrength)).c_str());
  strcat(sendBuffer, "}");

  

  Serial.println("Format send to MQTT");
  Serial.println(sendBuffer);

  sodaq_wdt_reset(); // Reset de WDT

  Serial.println("sending data...");
  mqttClient.beginMessage("devices/" + DeviceName + "/messages/events/");
  mqttClient.print(sendBuffer);
  mqttClient.endMessage();
  Serial.println("End sending data");
}

//=================================================================================================
// Function:    connectMQTT
// Return:      -
// Description: Connect to the MQTT Azure server
//=================================================================================================
void connectMQTT()
{

  String endPoint;
  char _mqttEndpoint[128 + 1];
  endPoint = "devices/";
  endPoint += DeviceName;
  endPoint += "/messages/devicebound/#";

 // Serial.println("Disable WDT");
//  sodaq_wdt_disable();

  strcpy(_mqttEndpoint, MQTTBroker.c_str());
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(_mqttEndpoint);
  Serial.println(" ");

  while (!mqttClient.connect(_mqttEndpoint, 8883))
  {
    // failed, retry
    Serial.print(".");
    Serial.println(mqttClient.connectError());
    sodaq_wdt_safe_delay(5000);
    if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
      connectNB();
    }
  }

  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  Serial.print("Device is listening to endPoint :");
  Serial.println(endPoint);
  mqttClient.subscribe(endPoint);
//  Serial.println("Enble WDT");
//  sodaq_wdt_enable(WDT_PERIOD_8X);
}

//=================================================================================================
// Function:    getSensorData
// Return:      -
// Description: Read out ADC values
//=================================================================================================
void getSensorData()
{
  Serial.println("Reading ADC converters....");
  writeToLogFile("Reading ADC converters....");
  int16_t value;

  P1 = ads1115_48.readADC_SingleEnded(3);
  Serial.println("Reading P1");
  sodaq_wdt_safe_delay(200);

  CO2 = ads1115_48.readADC_SingleEnded(2);
  Serial.println("Reading CO2");
  sodaq_wdt_safe_delay(200);

  H = ads1115_48.readADC_SingleEnded(1);
  Serial.println("Reading H....");
  sodaq_wdt_safe_delay(200);

  T = ads1115_48.readADC_SingleEnded(0);
  Serial.println("Reading Temp....");
  sodaq_wdt_safe_delay(200);

  P5 = ads1115_49.readADC_SingleEnded(3);
  Serial.println("Reading P5....");
  sodaq_wdt_safe_delay(200);

  NH3 = ads1115_49.readADC_SingleEnded(2);
  Serial.println("Reading NH3....");
  sodaq_wdt_safe_delay(200);

  P7 = ads1115_49.readADC_SingleEnded(1);
  Serial.println("Reading P7....");
  sodaq_wdt_safe_delay(200);

  P8 = ads1115_49.readADC_SingleEnded(0);
  Serial.println("Reading p8....");
  sodaq_wdt_safe_delay(200);

  MeasurementTime = getTime();

/* Send values to display */
  Serial.println("***************** Actual values read from port ***************************** ");
  Serial.print("CO2 :"); Serial.println(CO2);
  Serial.print("H   :"); Serial.println(H);
  Serial.print("T   :"); Serial.println(T);
  Serial.print("NH3 :"); Serial.println(NH3);
  Serial.println("**************************************************************************** ");

  writeToDataFile(MeasurementTime, H, T, CO2, NH3, P1, P5, P7, P8);

  if (params._isLedEnabled)
  {
    blinkLed(1);
  }
  //record->time = getTime();
}

//=================================================================================================
// Function:    onMessageReceived
// Return:      -
// Description: Set variables from Azure.
//=================================================================================================
void onMessageReceived(int messageSize)
{

  bool needsReboot = false;

  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic");
  writeToLogFile("Received a message with topic");

  char str[messageSize + 1];
  int ii = 0;
  for (ii = 0; ii < messageSize; ii++)
  {
    str[ii] = (char)mqttClient.read();
  }
  str[ii] = 0; // Terminate de string met een 0
  Serial.print(str);
  Serial.println();
  writeToLogFile(str);

  // Copy the string in a json document
  StaticJsonDocument<512> doc;
  deserializeJson(doc, str);

  const char *_type = doc["Type"];
  if (_type != nullptr)
  {
    Serial.print("Type of request");
    Serial.println(_type);
    if (strcmp(_type, "Settings") == 0)
    {
      Serial.println("Reading settings out of JSON");
      serializeJson(doc, Serial);
      Serial.println();
      int led = doc["Led"] | -1;
      int buffer = doc["Buffer"] | -1;
      int interval = doc["Interval"] | -1;
      int repeats = doc["Repeats"] | -1;

      if ((interval >= 100 && interval <= 120000) && interval != -1)
      {
        Serial.print("Setting measurement interval to :");
        Serial.print(interval);
        Serial.println(" ms");
        params._defaultMeasurementInterval = interval;
      }

      if ((buffer > 0 && buffer <= 15) && buffer != -1)
      {
        Serial.print("Setting buffer to :");
        Serial.print(interval);
        Serial.println(" meassurements");
        params._defaultNumberOfMeasurements = buffer;
      }

      if ((repeats > 0 && repeats < 3600))
      {
        Serial.print("Setting repeats to :");
        Serial.println(repeats);
        params._defaultRepeats = repeats;
        needsReboot = true;
      }

      params.commit(true);
      publishSettings();
      if (needsReboot)
      {
        Serial.println("Rebooting device....");
        writeToLogFile("Rebooting device on request");
        resetFunc();
      }
    }
    else if (strcmp(_type, "Info") == 0)
    {
      Serial.println("Asking for info. Sending settings to Azure.");
      writeToLogFile("Sending info to Azure");
      publishSettings();
    }
    else
    {
      Error = "Type is not recognized, valid types are :register, settings, ... ";
    }
  }
  else
  {
    Error = "Missing type in JSON string";
    // Error. There is no type give. Set the error flag.
  }

  params.commit(true);
  if (Error.length() > 0)
  {
    sendErrorToEndpoint();
  }
}

void blinkLed(int times)
{
  for (int ii = 0; ii < times; ii++)
  {
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    sodaq_wdt_safe_delay(200);
    digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
    sodaq_wdt_safe_delay(200);
  }
}

//=================================================================================================
// Function:    writeToLogFile
// Return:      -
// Description: Write string to logfile on SD Card. 
//=================================================================================================
void writeToLogFile(String message)
{
  String logfileName = "rdfv.log";

  if (sdPresent) {
    Serial.print("Writing :");
    Serial.print(message);
    Serial.println(" to logile");

    File logFile = SD.open(logfileName, FILE_WRITE);
    if (logFile) {
      Serial.println("Write log");
      logFile.println(message);
      logFile.close();
    }
  }
}

//=================================================================================================
// Function:    writeToLogFile
// Return:      -
// Description: Write string to logfile on SD Card. 
//=================================================================================================
void writeToDataFile(unsigned long time, int16_t H, int16_t T, int16_t CO2, int16_t NH3, int16_t P1, int16_t P5, int16_t P7, int16_t P8)
{
  Serial.println("Write to datafile");
  String datafileName = "rdfv.dat";
  if (sdPresent) {
    File dataFile = SD.open(datafileName, FILE_WRITE);
    dataFile.print(time);
    dataFile.print(",");
    dataFile.print(H);
    dataFile.print(",");
    dataFile.print(T);
    dataFile.print(",");
    dataFile.print(CO2);
    dataFile.print(",");
    dataFile.print(NH3);
    dataFile.print(",");
    dataFile.print(P1);
    dataFile.print(",");
    dataFile.print(P5);
    dataFile.print(",");
    dataFile.print(P7);
    dataFile.print(",");
    dataFile.println(P8);
    dataFile.close();
  }
}
//=================================================================================================
// Function:    connectNB
// Return:      -
// Description: Check if the NarrowBand connection is active, if not, try to reconnect!
//=================================================================================================
void connectNB()
{
  Serial.println("Attempting to connect to the cellular network");
  writeToLogFile("Attempting to connect to the cellular network");

  Serial.println("Disable WDT");
  sodaq_wdt_disable();

  //Connect only to T-Mobile if the signal strength is OK
  while ( (nbScanner.getSignalStrength().toInt() < MIN_SIGNAL_STRENGTH) || (nbScanner.getSignalStrength().toInt() == NO_SIGNAL_STRENGTH) )
    {
        Serial.println("No signal!");
        writeToLogFile("No signal in connectNB method");
        delay(500);
    }

  while ((nbAccess.begin(params._pinnumber, params._apn, true) != NB_READY) ||
         (gprs.attachGPRS() != GPRS_READY))
  {
    // failed, retry
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Enble WDT");
  sodaq_wdt_enable(WDT_PERIOD_8X);

  Serial.println("You're connected to the cellular network");
  writeToLogFile("You're connected to the cellular network");
  Serial.println(); 
}

//=================================================================================================
// Function:    setupModem
// Return:      -
// Description: set the Modem
//=================================================================================================
void setupModem()
{
  Serial.println("Waiting for modem to get ready...");
  writeToLogFile("Waiting for modem to get ready...");

  MODEM.begin();
  while (!MODEM.noop())
    ;

  /* Disconnect from any networks. */
  Serial.print("Disconnecting from network: ");
  writeToLogFile("Disconnecting from network: ");

  MODEM.sendf("AT+COPS=2");
  MODEM.waitForResponse(2000);
  Serial.println("done.");

  // Set Radio Access Technology (RAT). For this demo it will be set on Cat M1 (LTE-M)
  Serial.println("Set Radio Access Technology to LTE-M");
  writeToLogFile("Set Radio Access Technology to LTE-M");

  MODEM.sendf("AT+URAT=7");
  MODEM.waitForResponse(100, &response);
  Serial.println("done.");

  // Save changes to the modem.
  Serial.print("Applying changes and saving configuration: ");
  writeToLogFile("Applying changes and saving configuration: ");

  MODEM.sendf("AT+CFUN=15");
  do
  {
    delay(100);
    Serial.print(".");
    MODEM.noop();
  } while (MODEM.waitForResponse(1000) != 1);
  Serial.println("done.");

  Serial.println("Modem ready, turn radio on in order to configure it...");
  writeToLogFile("Modem ready, turn radio on in order to configure it...");

  MODEM.sendf("AT+CFUN=1");
  do
  {
    delay(100);
    Serial.print(".");
    MODEM.noop();
  } while (MODEM.waitForResponse(1000) != 1);
  Serial.println("done.");

  // Wait for a good signal strength (between 0 and 98)
  Serial.println("Check attachment until CSQ RSSI indicator is less than 99...");
  writeToLogFile("Check attachment until CSQ RSSI indicator is less than 99...");

  int status = 99;
  while (status > 31)
  {
    MODEM.send("AT+CSQ");
    MODEM.waitForResponse(2000, &response);

    // Parse response: +CSQ: <signal_power>,<qual>
    int delimeterIndex = response.indexOf(",");
    if (delimeterIndex != -1)
    {
      String sub = response.substring(6, delimeterIndex);
      status = sub.toInt(); // Will return 0 if no valid number is found
    }
    delay(1000);
  }
  Serial.println("done.");

  // Set operator to T-Mobile
  // Serial.println("Set operator to T-Mobile...");
  // dataFile = SD.open(logFile, FILE_WRITE);			// toegevoegd Carltech
  // dataFile.println("Set operator to T-Mobile...");
  Serial.println("Set operator to KPN...");
  writeToLogFile("Set operator to KPN...");

  //MODEM.sendf("AT+COPS=1,2,\"20416\"");
  MODEM.sendf("AT+COPS=1,2,\"20408\"");
  MODEM.waitForResponse(2000, &response);
  Serial.println("done.");
}

char stringTochar(String s)
{
  char arr[12];
  s.toCharArray(arr, sizeof(arr));
  return atol(arr);
}

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

