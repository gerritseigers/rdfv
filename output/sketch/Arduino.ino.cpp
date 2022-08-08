#include <Arduino.h>
#line 1 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
/*

*/

#include <MKRNB.h>
#include <ArduinoECCX08.h>
#include <ArduinoBearSSL.h>
#include <ArduinoMqttClient.h>
#include <Adafruit_ADS1015.h>
#include <Modem.h>
#include <utility/ECCX08SelfSignedCert.h>
#include "iot_configs.h"
#include "Params.h"
#include "BootMenu.h"
#include "DataRecord.h"
#include "Sodaq_wdt.h"
#include <ArduinoJson.h>
#include <cstring>

#define PROJECT_NAME "Project Marien"
#define VERSION "1.0.0"
#define STARTUP_DELAY 20000 // 20 seconden om te booten
#define CONSOLE_STREAM SerialUSB

#define MAX_NUMBER_OF_MEASUREMENTS 10 // Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS
#define DEFAULT_MEASUREMENT_INTERVAL 5000
#define DEFAULT_NUMBER_OF_MEASUREMENTS 10
#define DEFAULT_REPEATS 1

#define DEBUG 1
#define REGISTERED 1
#define DEVICE_NAME "A04072204" // THIS CODE MUST CHANGED FOR EVERY ARDUIO !!!!!
#define MQTT_BROKER "euw-iothub-rdfv-pr.azure-devices.net"
#define USE_GPS 1
#define USE_LED 1
#define PIN_NUMBER "915684"
#define APN_A "iot.t-mobile.nl"

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
#define GAIN2 2

// Translate iot-configs.h defines into variabels
static const char *host = IOT_CONFIG_IOTHUB_FQDN;
static const char *device_id = IOT_CONFIG_DEVICE_ID;
static const char *apn = LTE_APN;

/* Define the constants for the IOTHub and T-Mobile */
const char PINNUMBER[] = "031591"; // PIN Number
// const char APN[]              = "iot.t-mobile.nl";                                               // APN T-Mobile

const int delayBetweenMessages = 500; // Wait time im ms
String response;
bool useGPS = false;           // Geeft aan of de GPS beschikbaar is op de sensor
static uint8_t lastResetCause; // Veld waarin staat wat de oorzaak is van de laatste reset.
String IMEI = "";
String DeviceName = ""; //
String MQTTBroker = "";
String Error = "";
char sendBuffer[500];
DataRecord measurements[MAX_NUMBER_OF_MEASUREMENTS];
int measurementPointer = 0; // Actueel aantal metingen dat gedaan is.

GPRS gprs;
NB nbAccess(false);                // Set op true om te debuggen
NBClient nbClient;                 // Used for the TCP socket connection
BearSSLClient sslClient(nbClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient mqttClient(sslClient);
NBModem modem;

// Variabels for MQTT
String username;
String broker;

Adafruit_ADS1115 ads1115_48(0x48);
Adafruit_ADS1115 ads1115_49(0x49);

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

void setGain(Adafruit_ADS1115 &sensor, uint8_t gain);
double getMultiplier(int portNumber); // Returns the multiplier for this specific portnumber

void (*resetFunc)(void) = 0; // Functie voor harde reset. Wordt aangeroepen als buffer overloopt.

#line 112 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void setup();
#line 203 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void loop();
#line 264 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void setGain(Adafruit_ADS1115 &device, uint8_t gain);
#line 412 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void publishSettings();
#line 564 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void publishMessage(DataRecord records[], int numberOfMessages);
#line 784 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void getSensorData(DataRecord *record);
#line 857 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void onMessageReceived(int messageSize);
#line 948 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void blinkLed(int times);
#line 1053 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
char stringTochar(String s);
#line 112 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void setup()
{
  lastResetCause = PM->RCAUSE.reg;
  blinkLed(5);

  setupModem();

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
    while (1)
      ;
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

  measurementPointer = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  Serial1.begin(115200);
  Serial.begin(115200);

  sodaq_wdt_disable();
  Serial.println("WDT Period 8X");
  sodaq_wdt_enable(WDT_PERIOD_8X); // WDT is gezet op 8 seconden.
  Serial.println("WDT Reset");
  sodaq_wdt_reset();

  if (params._useADC1 == true)
  {
    Serial.print("Setting gain for ADC1 :");
    Serial.println(params._gain_1);
    setGain(ads1115_48, params._gain_1);
  }

  if (params._useADC2 == true)
  {
    Serial.print("Setting gain for ADC2 :");
    Serial.println(params._gain_2);  
    setGain(ads1115_49, params._gain_2);
  }

  modem.begin();
  measurementPointer = 0;
}

void loop()
{
  Serial.println("WDT is reset");
  sodaq_wdt_reset(); // Reset de WDT

  // Make a connection
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY)
  {
    connectNB();
  }

  // Controleer of de MQTT Client beschikbaar is. Als er een timeout is dan moet er iets worden gedaan? TODO
  if (!mqttClient.connected())
  {
    connectMQTT();
  }

  blinkLed(2);
  Serial.println("Device is registred. Starting measurements now.");
  getSensorData(&measurements[measurementPointer++]);

  // If the current measurementPointer is greater od equal then the buffer then send the data toi Azure.
  Serial.print("Measurement pointer :");
  Serial.println(measurementPointer);
  Serial.print("Default number of measurements");
  Serial.println(params._defaultNumberOfMeasurements);


  if (measurementPointer >= params._defaultNumberOfMeasurements)
  {
    Serial.println("WDT save delay (8000) in loop just before publishMessage");
    mqttClient.poll();
    publishMessage(measurements, measurementPointer);
    measurementPointer = 0;
  }
  sodaq_wdt_safe_delay(params._defaultMeasurementInterval);

  // Reset the pointer
}

unsigned long getTime()
{
  // get the current time from the cellular module
  return nbAccess.getTime();
}

/**
 * Prints a boot-up message that includes project name, version and Cpu reset cause.
 */
static void printBootUpMessage(Stream &stream)
{
  stream.println("** " PROJECT_NAME " - " VERSION " **");

  stream.println();

  stream.print(" -> ");
  // printCpuResetCause(stream);

  stream.println();
}

void setGain(Adafruit_ADS1115 &device, uint8_t gain)
{
  Serial.print("Setting gain for ");
  Serial.print("DeviceName ");
  Serial.print("on ");
  Serial.print(gain);

  if (gain == 0)
  {
    device.setGain(GAIN_ONE);
  }
  else if (gain == 1)
  {
    device.setGain(GAIN_ONE);
  }
  else if (gain == 2)
  {
    device.setGain(GAIN_TWO);
  }
  else if (gain == 4)
  {
    device.setGain(GAIN_FOUR);
  }
  else if (gain == 8)
  {
    device.setGain(GAIN_EIGHT);
  }
  else if (gain == 16)
  {
    device.setGain(GAIN_SIXTEEN);
  }
  else
  {
    device.setGain(GAIN_ONE);
  }
}

/**
 * Callback from Config.reset(), used to override default values.
 */
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

/**
 * Shows and handles the boot up commands.
 */
void handleBootUpCommands()
{
  do
  {
    showBootMenu(CONSOLE_STREAM);
  } while (!params.checkConfig(CONSOLE_STREAM));

  params.showConfig(&CONSOLE_STREAM);
  params.commit();
}

void publishSettings()
{
  unsigned long timeStamp = getTime();
  String endPoint;
  String jsonMessage;
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

  strcat(sendBuffer, "}");
  jsonMessage += "{";
  jsonMessage += "\"Type\":";
  jsonMessage += "\"Settings\"";
  jsonMessage += ",";
  jsonMessage += "\"DeviceName\":";
  jsonMessage += "\"";
  jsonMessage += deviceId;
  jsonMessage += "\"";
  jsonMessage += ",";
  jsonMessage += "\"IMeiCode\":";
  jsonMessage += "\"";
  jsonMessage += imeiCode;
  jsonMessage += "\"";
  jsonMessage += ",";
  jsonMessage += "\"ICCID\":";
  jsonMessage += "\"";
  jsonMessage += ICCID;
  jsonMessage += "\"";
  jsonMessage += ",";
  jsonMessage += "\"Timestamp\":";
  jsonMessage += getTime();
  jsonMessage += ",";
  jsonMessage += "\"Buffer\":";
  jsonMessage += params._defaultNumberOfMeasurements;
  jsonMessage += ",";
  jsonMessage += "\"Interval\":";
  jsonMessage += params._defaultMeasurementInterval;
  jsonMessage += ",";
  jsonMessage += "\"Repeats\":";
  jsonMessage += params._defaultRepeats;
  jsonMessage += "}";

  StaticJsonDocument<512> doc;
  doc["Type"] = "Settings";
  // doc["Timestamp"] = getTime();
  doc["DeviceName"] = params.getDeviceName();

  doc["Buffer"] = params._defaultNumberOfMeasurements;
  doc["Interval"] = params._defaultMeasurementInterval;
  doc["Repeats"] = params._defaultRepeats;

  serializeJson(doc, Serial);
  Serial.println();

  long l = (unsigned long)measureJson(doc);
  Serial.println("Lenght of document");
  Serial.println(l);

  Serial.println("Just before sending data");
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(jsonMessage);
  mqttClient.endMessage();
  Serial.println("End sending data");
}

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

double getMultiplier(int portNumber)
{
  const float multiplier_1 = 0.125F;
  const float multiplier_2 = 0.06250F;
  const float multiplier_4 = 0.03125F;
  const float multiplier_8 = 0.015625;
  const float multiplier_16 = 0.0078125F;
  int gain = 0;

  if (portNumber >= 0 && portNumber <= 3)
  {
    gain = params._gain_1;
  }
  else if (portNumber >= 4 && portNumber <= 7)
  {
    gain = params._gain_2;
  }

  if (gain == 1)
  {
    return multiplier_1;
  }
  else if (gain == 2)
  {
    return multiplier_2;
  }
  else if (gain == 4)
  {
    return multiplier_4;
  }
  else if (gain == 8)
  {
    return multiplier_8;
  }
  else if (gain == 16)
  {
    return multiplier_16;
  }
  return 0.0F;
}

/*
  Verstuur de berichten die in het buffer zitten naar het MQTT endpoint in azure.
*/
void publishMessage(DataRecord records[], int numberOfMessages)
{
  // sodaq_wdt_disable();

  /* Defintions of the multipliers */

  /* Send all message in the array record to azure. */
  unsigned long timeStamp = getTime();
  String endPoint;
  String jsonString;
  String deviceId;

  String Status = "Succes";
  String Message = "";

  int32_t Total_P1 = 0;
  int32_t Total_P2 = 0;
  int32_t Total_P3 = 0;
  int32_t Total_P4 = 0;
  int32_t Total_P5 = 0;
  int32_t Total_P6 = 0;
  int32_t Total_P7 = 0;
  int32_t Total_P8 = 0;

  Serial.println("*** Entering publish message ***");
  Serial.print("Total number of messeages: ");
  Serial.println(numberOfMessages);

  for (int ii = 0; ii < numberOfMessages; ii++)
  {
    if (params._useADC1 == true)
    {
      Total_P1 += records[ii].P1;
      Total_P2 += records[ii].P2;
      Total_P3 += records[ii].P3;
      Total_P4 += records[ii].P4;
    }

    if (params._useADC2 == true)
    {
      Total_P5 += records[ii].P5;
      Total_P6 += records[ii].P6;
      Total_P7 += records[ii].P7;
      Total_P8 += records[ii].P8;
    }
  }

  Serial.println("** Overview ***");
  if (params._useADC1 == true)
  {
    Serial.print("Total of port 1:");
    Serial.println(Total_P1);

    Serial.print("Total of port 2:");
    Serial.println(Total_P2);

    Serial.print("Total of port 3:");
    Serial.println(Total_P3);

    Serial.print("Total of port 4:");
    Serial.println(Total_P4);
  }

  if (params._useADC2 == true)
  {
    Serial.print("Total of port 5:");
    Serial.println(Total_P5);

    Serial.print("Total of port 6:");
    Serial.println(Total_P6);

    Serial.print("Total of port 7:");
    Serial.println(Total_P7);

    Serial.print("Total of port 8:");
    Serial.println(Total_P8);
  }

  deviceId = params.getDeviceName();
  Serial.print("Name of device:");
  Serial.println(deviceId);

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

  if (strlen(params._p1_1) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p1_1).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P1 / numberOfMessages) * getMultiplier(0)).c_str());

  }

  if (strlen(params._p1_2) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p1_2).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P2 / numberOfMessages) * getMultiplier(1)).c_str());

  }

  if (strlen(params._p1_3) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p1_3).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P3 / numberOfMessages) * getMultiplier(2)).c_str());
  }

  if (strlen(params._p1_4) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p1_4).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P4 / numberOfMessages) * getMultiplier(3)).c_str());

  }

  if (strlen(params._p2_1) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p2_1).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P5 / numberOfMessages) * getMultiplier(4)).c_str());
  }

  if (strlen(params._p2_2) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p2_2).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P6 / numberOfMessages) * getMultiplier(5)).c_str());
  }

  if (strlen(params._p2_3) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p2_3).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P7 / numberOfMessages) * getMultiplier(6)).c_str());
  }

  if (strlen(params._p2_4) > 0)
  {
    strcat(sendBuffer,",");
    strcat(sendBuffer,"\"");
    strcat(sendBuffer, String(params._p2_4).c_str());
    strcat(sendBuffer,"\":");
    strcat(sendBuffer, String((Total_P8 / numberOfMessages) * getMultiplier(7)).c_str());
  }

  strcat(sendBuffer, "}");

  Serial.println("New Format");
  Serial.println(sendBuffer);

  Serial.println("Just before sending data");
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(sendBuffer);
  mqttClient.endMessage();
  Serial.println("End sending data");
}

void connectMQTT()
{

  String endPoint;
  char _mqttEndpoint[128 + 1];
  endPoint = "devices/";
  endPoint += DeviceName;
  endPoint += "/messages/devicebound/#";

  Serial.println("Disable WDT");
  sodaq_wdt_disable();

  strcpy(_mqttEndpoint, MQTTBroker.c_str());
  Serial.print("Attempting to MQTT broker: ");
  Serial.print(_mqttEndpoint);
  Serial.println(" ");

  while (!mqttClient.connect(_mqttEndpoint, 8883))
  {
    // failed, retry
    Serial.print(".");
    Serial.println(mqttClient.connectError());
    delay(5000);
  }

  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  Serial.print("Device is listening to endPoint :");
  Serial.println(endPoint);
  mqttClient.subscribe(endPoint);
  Serial.println("Enble WDT");
  sodaq_wdt_enable(WDT_PERIOD_8X);
}

/* Lees de datauit van de methaan sensor. Omdat nog niet duidelijk hoe dit wordt gedaan wordt er hier een random getal genomen tussen de 0 en 10000. */
void getSensorData(DataRecord *record)
{
  Serial.println("Reading ADC converters....");
  int16_t value;

  if (params._useADC1 == true)
  {
    value = ads1115_48.readADC_SingleEnded(3);
    Serial.print("Port 1 :");
    Serial.println(value);
    record->P1 = value;

    value = ads1115_48.readADC_SingleEnded(2);
    Serial.print("Port 2 :");
    Serial.println(value);
    record->P2 = value;

    value = ads1115_48.readADC_SingleEnded(1);
    Serial.print("Port 3 :");
    Serial.println(value);
    record->P3 = value;

    value = ads1115_48.readADC_SingleEnded(0);
    Serial.print("Port 4 :");
    Serial.println(value);
    record->P4 = value;
  }

  if (params._useADC2 == true)
  {
    value = ads1115_49.readADC_SingleEnded(3);
    Serial.print("Port 5 :");
    Serial.println(value);
    record->P5 = value;

    value = ads1115_49.readADC_SingleEnded(2);
    Serial.print("Port 6 :");
    Serial.println(value);
    record->P6 = value;

    value = ads1115_49.readADC_SingleEnded(1);
    Serial.print("Port 7 :");
    Serial.println(value);
    record->P7 = value;

    value = ads1115_49.readADC_SingleEnded(0);
    Serial.print("Port 8 :");
    Serial.println(value);
    record->P8 = value;
  }

  record->P9 = random(1000);
  record->P10 = random(1000);
  record->P11 = random(1000);
  record->P12 = random(1000);

  record->P13 = random(1000);
  record->P14 = random(1000);
  record->P15 = random(1000);
  record->P16 = random(1000);

  if (params._isLedEnabled)
  {
    blinkLed(1);
  }
  record->time = getTime();
}

/*
 * OnMessageReceived:
 *
 * Set variables from Azure.
 */
void onMessageReceived(int messageSize)
{

  bool needsReboot = false;

  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");

  char str[messageSize + 1];
  int ii = 0;
  for (ii = 0; ii < messageSize; ii++)
  {
    str[ii] = (char)mqttClient.read();
  }
  str[ii] = 0; // Terminate de string met een 0
  Serial.print(str);
  Serial.println();

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

      if ((interval >= 100 && interval < 120000) && interval != -1)
      {
        Serial.print("Setting measurement interval to :");
        Serial.print(interval);
        Serial.println(" ms");
        params._defaultMeasurementInterval = interval;
      }

      if ((buffer > 0 && buffer <= 10) && buffer != -1)
      {
        Serial.print("Setting buffer to :");
        Serial.print(interval);
        Serial.println(" meassurements");
        params._defaultNumberOfMeasurements = buffer;
      }

      if ((repeats > 0 && repeats < 10))
      {
        Serial.print("Setting repeats to :");
        Serial.println(repeats);
        params._defaultRepeats = repeats;
      }

      params.commit(true);
      publishSettings();
      if (needsReboot)
      {
        Serial.println("Rebooting device....");
        resetFunc();
      }
    }
    else if (strcmp(_type, "Info") == 0)
    {
      Serial.println("Asking for info. Sending settings to Azure.");
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

void connectNB()
{
  Serial.println("Attempting to connect to the cellular network");

  Serial.println("Disable WDT");
  sodaq_wdt_disable();

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
  Serial.println();

  
}

void setupModem()
{
  Serial.println("Waiting for modem to get ready...");
  MODEM.begin();
  while (!MODEM.noop())
    ;

  /* Disconnect from any networks. */
  Serial.print("Disconnecting from network: ");
  MODEM.sendf("AT+COPS=2");
  MODEM.waitForResponse(2000);
  Serial.println("done.");

  // Set Radio Access Technology (RAT). For this demo it will be set on Cat M1 (LTE-M)
  Serial.println("Set Radio Access Technology to LTE-M");
  MODEM.sendf("AT+URAT=7");
  MODEM.waitForResponse(100, &response);
  Serial.println("done.");

  // Save changes to the modem.
  Serial.print("Applying changes and saving configuration: ");
  MODEM.sendf("AT+CFUN=15");
  do
  {
    delay(100);
    Serial.print(".");
    MODEM.noop();
  } while (MODEM.waitForResponse(1000) != 1);
  Serial.println("done.");

  Serial.println("Modem ready, turn radio on in order to configure it...");
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
