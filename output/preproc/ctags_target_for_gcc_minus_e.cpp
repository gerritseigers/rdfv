# 1 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
/*



*/
# 5 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
# 6 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 7 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 8 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 9 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 10 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 11 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 12 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 13 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 14 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 15 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 16 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 17 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 18 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 19 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 39 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"

# 39 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
//#define PORT1 "T"



//#define PORT5 "G"

// #define PORT7 ""
// #define PORT8 ""







// Translate iot-configs.h defines into variabels
static const char *host = "euw-ihb-dmo-poc.azure-devices.net";
static const char *device_id = "MKRNB1500_2";
static const char *apn = "iot.t-mobile.nl";

/* Define the constants for the IOTHub and T-Mobile */
const char PINNUMBER[] = "031591"; // PIN Number
// const char APN[]              = "iot.t-mobile.nl";                                               // APN T-Mobile

const int delayBetweenMessages = 500; // Wait time im ms
String response;
bool useGPS = false; // Geeft aan of de GPS beschikbaar is op de sensor
static uint8_t lastResetCause; // Veld waarin staat wat de oorzaak is van de laatste reset.
String IMEI = "";
String DeviceName = ""; //
String MQTTBroker = "";
String Error = "";
char sendBuffer[500];
DataRecord measurements[10 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/];
int measurementPointer = 0; // Actueel aantal metingen dat gedaan is.

GPRS gprs;
NB nbAccess(false); // Set op true om te debuggen
NBClient nbClient; // Used for the TCP socket connection
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

void setup()
{
  lastResetCause = ((Pm *)0x40000400UL) /**< \brief (PM) APB Base Address */->RCAUSE.reg;
  blinkLed(5);

  setupModem();

  // Disable the WDT tijdens het setup proces
  SerialUSB.println("WDT is disabled");
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
    SerialUSB.println("No ECCX08 present!");
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
    SerialUSB.println("SETUP: Device is already registred");
    DeviceName = params._deviceName;
    MQTTBroker = params.getMQTTBroker();
  }

  username += MQTTBroker;
  username += "/";
  username += DeviceName;
  username += "/api-version=2018-06-30";

  SerialUSB.print("Username  :");
  SerialUSB.println(username);

  // Set the client id used for MQTT as the device id
  mqttClient.setId(DeviceName);
  mqttClient.setUsernamePassword(username, "");
  mqttClient.onMessage(onMessageReceived); // Call back if an mesaage is received from the IOT-HUB

  measurementPointer = 0;
  pinMode((6u), OUTPUT);
  Serial1.begin(115200);
  SerialUSB.begin(115200);

  sodaq_wdt_disable();
  SerialUSB.println("WDT Period 8X");
  sodaq_wdt_enable(WDT_PERIOD_8X); // WDT is gezet op 8 seconden.
  SerialUSB.println("WDT Reset");
  sodaq_wdt_reset();

  if (params._useADC1 == true)
  {
    SerialUSB.print("Setting gain for ADC1 :");
    SerialUSB.println(params._gain_1);
    setGain(ads1115_48, params._gain_1);
  }

  if (params._useADC2 == true)
  {
    SerialUSB.print("Setting gain for ADC2 :");
    SerialUSB.println(params._gain_2);
    setGain(ads1115_49, params._gain_2);
  }

  modem.begin();
  measurementPointer = 0;
}

void loop()
{
  SerialUSB.println("WDT is reset");
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
  SerialUSB.println("Device is registred. Starting measurements now.");
  getSensorData(&measurements[measurementPointer++]);

  // If the current measurementPointer is greater od equal then the buffer then send the data toi Azure.
  SerialUSB.print("Measurement pointer :");
  SerialUSB.println(measurementPointer);
  SerialUSB.print("Default number of measurements");
  SerialUSB.println(params._defaultNumberOfMeasurements);


  if (measurementPointer >= params._defaultNumberOfMeasurements)
  {
    SerialUSB.println("WDT save delay (8000) in loop just before publishMessage");
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
# 252 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
static void printBootUpMessage(Stream &stream)
{
  stream.println("** " "Project Marien" " - " "1.0.0" " **");

  stream.println();

  stream.print(" -> ");
  // printCpuResetCause(stream);

  stream.println();
}

void setGain(Adafruit_ADS1115 &device, uint8_t gain)
{
  SerialUSB.print("Setting gain for ");
  SerialUSB.print("DeviceName ");
  SerialUSB.print("on ");
  SerialUSB.print(gain);

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
# 304 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void onConfigReset(void)
{

  /* CHANGE _params._isDeviceRegistered = false; wheen in production !!!! */


  params._isDeviceRegistered = true;



  strcpy(params._deviceName, "A04072205" /* THIS CODE MUST CHANGED FOR EVERY ARDUIO !!!!!*/);



  strcpy(params._pinnumber, "915684");



  strcpy(params._apn, "iot.t-mobile.nl");



  params._isDebugOn = true;



  strcpy(params._endPointData, "euw-iothub-rdfv-pr.azure-devices.net");



  params._defaultMeasurementInterval = 1000;



  params._defaultNumberOfMeasurements = 5;



  params._defaultRepeats = 1;



  params._isLedEnabled = 1;



  params._useADC1 = true;



  params._useADC2 = true;



  params._gain_1 = 2 /* Range for 0 to 2048 mV*/;



  params._gain_2 = 2;







  strcpy(params._p1_2, "CO2");



  strcpy(params._p1_3, "H");



  strcpy(params._p1_4, "T");







  strcpy(params._p2_2, "NH3");
# 396 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
}

/**

 * Shows and handles the boot up commands.

 */
# 401 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void handleBootUpCommands()
{
  do
  {
    showBootMenu(SerialUSB);
  } while (!params.checkConfig(SerialUSB));

  params.showConfig(&SerialUSB);
  params.commit();
}

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

  SerialUSB.println('=============================================================');
  SerialUSB.println('New Json');
  SerialUSB.println(sendBuffer);

  SerialUSB.println("Just before sending data");
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(sendBuffer);
  mqttClient.endMessage();
  SerialUSB.println("End sending data");
}

void sendErrorToEndpoint()
{
  SerialUSB.println("WDT save delay in sendError");
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

  serializeJson(doc, SerialUSB);
  SerialUSB.println();

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
# 544 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
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

  SerialUSB.println("*** Entering publish message ***");
  SerialUSB.print("Total number of messeages: ");
  SerialUSB.println(numberOfMessages);

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

  SerialUSB.println("** Overview ***");
  if (params._useADC1 == true)
  {
    SerialUSB.print("Total of port 1:");
    SerialUSB.println(Total_P1);

    SerialUSB.print("Total of port 2:");
    SerialUSB.println(Total_P2);

    SerialUSB.print("Total of port 3:");
    SerialUSB.println(Total_P3);

    SerialUSB.print("Total of port 4:");
    SerialUSB.println(Total_P4);
  }

  if (params._useADC2 == true)
  {
    SerialUSB.print("Total of port 5:");
    SerialUSB.println(Total_P5);

    SerialUSB.print("Total of port 6:");
    SerialUSB.println(Total_P6);

    SerialUSB.print("Total of port 7:");
    SerialUSB.println(Total_P7);

    SerialUSB.print("Total of port 8:");
    SerialUSB.println(Total_P8);
  }

  deviceId = params.getDeviceName();
  SerialUSB.print("Name of device:");
  SerialUSB.println(deviceId);

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

  SerialUSB.println("New Format");
  SerialUSB.println(sendBuffer);

  SerialUSB.println("Just before sending data");
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(sendBuffer);
  mqttClient.endMessage();
  SerialUSB.println("End sending data");
}

void connectMQTT()
{

  String endPoint;
  char _mqttEndpoint[128 + 1];
  endPoint = "devices/";
  endPoint += DeviceName;
  endPoint += "/messages/devicebound/#";

  SerialUSB.println("Disable WDT");
  sodaq_wdt_disable();

  strcpy(_mqttEndpoint, MQTTBroker.c_str());
  SerialUSB.print("Attempting to MQTT broker: ");
  SerialUSB.print(_mqttEndpoint);
  SerialUSB.println(" ");

  while (!mqttClient.connect(_mqttEndpoint, 8883))
  {
    // failed, retry
    SerialUSB.print(".");
    SerialUSB.println(mqttClient.connectError());
    delay(5000);
  }

  SerialUSB.println();

  SerialUSB.println("You're connected to the MQTT broker");
  SerialUSB.println();

  // subscribe to a topic
  SerialUSB.print("Device is listening to endPoint :");
  SerialUSB.println(endPoint);
  mqttClient.subscribe(endPoint);
  SerialUSB.println("Enble WDT");
  sodaq_wdt_enable(WDT_PERIOD_8X);
}

/* Lees de datauit van de methaan sensor. Omdat nog niet duidelijk hoe dit wordt gedaan wordt er hier een random getal genomen tussen de 0 en 10000. */
void getSensorData(DataRecord *record)
{
  SerialUSB.println("Reading ADC converters....");
  int16_t value;

  if (params._useADC1 == true)
  {
    value = ads1115_48.readADC_SingleEnded(3);
    SerialUSB.print("Port 1 :");
    SerialUSB.println(value);
    record->P1 = value;

    value = ads1115_48.readADC_SingleEnded(2);
    SerialUSB.print("Port 2 :");
    SerialUSB.println(value);
    record->P2 = value;

    value = ads1115_48.readADC_SingleEnded(1);
    SerialUSB.print("Port 3 :");
    SerialUSB.println(value);
    record->P3 = value;

    value = ads1115_48.readADC_SingleEnded(0);
    SerialUSB.print("Port 4 :");
    SerialUSB.println(value);
    record->P4 = value;
  }

  if (params._useADC2 == true)
  {
    value = ads1115_49.readADC_SingleEnded(3);
    SerialUSB.print("Port 5 :");
    SerialUSB.println(value);
    record->P5 = value;

    value = ads1115_49.readADC_SingleEnded(2);
    SerialUSB.print("Port 6 :");
    SerialUSB.println(value);
    record->P6 = value;

    value = ads1115_49.readADC_SingleEnded(1);
    SerialUSB.print("Port 7 :");
    SerialUSB.println(value);
    record->P7 = value;

    value = ads1115_49.readADC_SingleEnded(0);
    SerialUSB.print("Port 8 :");
    SerialUSB.println(value);
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
# 839 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void onMessageReceived(int messageSize)
{

  bool needsReboot = false;

  // we received a message, print out the topic and contents
  SerialUSB.print("Received a message with topic '");

  char str[messageSize + 1];
  int ii = 0;
  for (ii = 0; ii < messageSize; ii++)
  {
    str[ii] = (char)mqttClient.read();
  }
  str[ii] = 0; // Terminate de string met een 0
  SerialUSB.print(str);
  SerialUSB.println();

  // Copy the string in a json document
  StaticJsonDocument<512> doc;
  deserializeJson(doc, str);

  const char *_type = doc["Type"];
  if (_type != nullptr)
  {
    SerialUSB.print("Type of request");
    SerialUSB.println(_type);
    if (strcmp(_type, "Settings") == 0)
    {
      SerialUSB.println("Reading settings out of JSON");
      serializeJson(doc, SerialUSB);
      SerialUSB.println();
      int led = doc["Led"] | -1;
      int buffer = doc["Buffer"] | -1;
      int interval = doc["Interval"] | -1;
      int repeats = doc["Repeats"] | -1;

      if ((interval >= 100 && interval < 120000) && interval != -1)
      {
        SerialUSB.print("Setting measurement interval to :");
        SerialUSB.print(interval);
        SerialUSB.println(" ms");
        params._defaultMeasurementInterval = interval;
      }

      if ((buffer > 0 && buffer <= 10) && buffer != -1)
      {
        SerialUSB.print("Setting buffer to :");
        SerialUSB.print(interval);
        SerialUSB.println(" meassurements");
        params._defaultNumberOfMeasurements = buffer;
      }

      if ((repeats > 0 && repeats < 10))
      {
        SerialUSB.print("Setting repeats to :");
        SerialUSB.println(repeats);
        params._defaultRepeats = repeats;
      }

      params.commit(true);
      publishSettings();
      if (needsReboot)
      {
        SerialUSB.println("Rebooting device....");
        resetFunc();
      }
    }
    else if (strcmp(_type, "Info") == 0)
    {
      SerialUSB.println("Asking for info. Sending settings to Azure.");
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
    digitalWrite((6u), HIGH); // turn the LED on (HIGH is the voltage level)
    sodaq_wdt_safe_delay(200);
    digitalWrite((6u), LOW); // turn the LED off by making the voltage LOW
    sodaq_wdt_safe_delay(200);
  }
}

void connectNB()
{
  SerialUSB.println("Attempting to connect to the cellular network");

  SerialUSB.println("Disable WDT");
  sodaq_wdt_disable();

  while ((nbAccess.begin(params._pinnumber, params._apn, true) != NB_READY) ||
         (gprs.attachGPRS() != GPRS_READY))
  {
    // failed, retry
    SerialUSB.print(".");
    delay(1000);
  }

  SerialUSB.println("Enble WDT");
  sodaq_wdt_enable(WDT_PERIOD_8X);

  SerialUSB.println("You're connected to the cellular network");
  SerialUSB.println();


}

void setupModem()
{
  SerialUSB.println("Waiting for modem to get ready...");
  MODEM.begin();
  while (!MODEM.noop())
    ;

  /* Disconnect from any networks. */
  SerialUSB.print("Disconnecting from network: ");
  MODEM.sendf("AT+COPS=2");
  MODEM.waitForResponse(2000);
  SerialUSB.println("done.");

  // Set Radio Access Technology (RAT). For this demo it will be set on Cat M1 (LTE-M)
  SerialUSB.println("Set Radio Access Technology to LTE-M");
  MODEM.sendf("AT+URAT=7");
  MODEM.waitForResponse(100, &response);
  SerialUSB.println("done.");

  // Save changes to the modem.
  SerialUSB.print("Applying changes and saving configuration: ");
  MODEM.sendf("AT+CFUN=15");
  do
  {
    delay(100);
    SerialUSB.print(".");
    MODEM.noop();
  } while (MODEM.waitForResponse(1000) != 1);
  SerialUSB.println("done.");

  SerialUSB.println("Modem ready, turn radio on in order to configure it...");
  MODEM.sendf("AT+CFUN=1");
  do
  {
    delay(100);
    SerialUSB.print(".");
    MODEM.noop();
  } while (MODEM.waitForResponse(1000) != 1);
  SerialUSB.println("done.");

  // Wait for a good signal strength (between 0 and 98)
  SerialUSB.println("Check attachment until CSQ RSSI indicator is less than 99...");
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
  SerialUSB.println("done.");

  // Set operator to T-Mobile
  // Serial.println("Set operator to T-Mobile...");
  // dataFile = SD.open(logFile, FILE_WRITE);			// toegevoegd Carltech
  // dataFile.println("Set operator to T-Mobile...");
  SerialUSB.println("Set operator to KPN...");
  //MODEM.sendf("AT+COPS=1,2,\"20416\"");
  MODEM.sendf("AT+COPS=1,2,\"20408\"");
  MODEM.waitForResponse(2000, &response);
  SerialUSB.println("done.");
}

char stringTochar(String s)
{
  char arr[12];
  s.toCharArray(arr, sizeof(arr));
  return atol(arr);
}
