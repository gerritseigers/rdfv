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
static uint8_t lastResetCause; // Veld waarin staat wat de oorzaak is van de laatste reset.
String IMEI = "";
String DeviceName;
String MQTTBroker = "";
String Error = "";
char sendBuffer[500];
// DataRecord measurements[MAX_NUMBER_OF_MEASUREMENTS];
// int measurementPointer = 0; // Actueel aantal metingen dat gedaan is.

//Variables for reading the ports
int16_t CO2; //Port2 p1_2
int16_t H; //Port3 p1_3 
int16_t T; //Port4 p1_4 
int16_t NH3; //Port6 p2_2
int32_t MeasurementTime; //Time of measurement
int16_t repeatCounter; // 

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

// void setGain(Adafruit_ADS1115 &sensor, uint8_t gain);
// double getMultiplier(int portNumber); // Returns the multiplier for this specific portnumber

void (*resetFunc)(void) = 0; // Functie voor harde reset. Wordt aangeroepen als buffer overloopt.

void setup()
{
  SerialUSB.begin(115200);

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

  pinMode((6u), OUTPUT);
  Serial1.begin(115200);
  SerialUSB.begin(115200);

  sodaq_wdt_disable();
  SerialUSB.println("WDT Period 8X");
  sodaq_wdt_enable(WDT_PERIOD_8X); // WDT is gezet op 8 seconden.
  SerialUSB.println("WDT Reset");
  sodaq_wdt_reset();

  ads1115_48.setGain(GAIN_TWO);
  ads1115_49.setGain(GAIN_TWO);

  repeatCounter = 0;

  modem.begin();
}

void loop()
{
  SerialUSB.println("WDT is reset");
  sodaq_wdt_reset(); // Reset de WDT
  SerialUSB.print("Counter :");
  SerialUSB.println(repeatCounter);

  if (repeatCounter > params._defaultRepeats)
  {
        SerialUSB.println("Rebooting device counter reached settings....");
        resetFunc();
  }

  // Make a connection
  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY)
  {
    SerialUSB.println("No connection with T-Mobile");
    connectNB();
  }

  // Controleer of de MQTT Client beschikbaar is. Als er een timeout is dan moet er iets worden gedaan? TODO
  if (!mqttClient.connected())
  {
    SerialUSB.println("No connection with Azure");
    connectMQTT();
  }

  blinkLed(2);
  SerialUSB.println("Device is registred. Starting measurements now.");


  SerialUSB.println("Reading ports");
  getSensorData();

  // If the current measurementPointer is greater od equal then the buffer then send the data toi Azure.
  // Serial.print("Measurement pointer :");
  // Serial.println(measurementPointer);
  // Serial.print("Default number of measurements");
  // Serial.println(params._defaultNumberOfMeasurements);

  SerialUSB.println("Send message to IOT-HUB");
  publishMessage();
  // mqttClient.poll();
  SerialUSB.println("Entering waiting loop...");
  sodaq_wdt_safe_delay(params._defaultMeasurementInterval);
  repeatCounter = repeatCounter + 1;
}

unsigned long getTime()
{
  // get the current time from the cellular module
  SerialUSB.println("Getting the time...");
  return nbAccess.getTime();
}

/**

 * Prints a boot-up message that includes project name, version and Cpu reset cause.

 */
# 259 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
static void printBootUpMessage(Stream &stream)
{
  stream.println("** " "Project Marien" " - " "1.0.0" " **");

  stream.println();

  stream.print(" -> ");
  // printCpuResetCause(stream);

  stream.println();
}

void onConfigReset(void)
{

  /* CHANGE _params._isDeviceRegistered = false; wheen in production !!!! */


  params._isDeviceRegistered = true;



  strcpy(params._deviceName, "A04072203" /* THIS CODE MUST CHANGED FOR EVERY ARDUIO !!!!!*/);



  strcpy(params._pinnumber, "915684");



  strcpy(params._apn, "iot.t-mobile.nl");



  params._isDebugOn = true;



  strcpy(params._endPointData, "euw-iothub-rdfv-pr.azure-devices.net");



  params._defaultMeasurementInterval = 8000;



  params._defaultNumberOfMeasurements = 1;



  params._defaultRepeats = 100000;



  params._isLedEnabled = 1;



  params._useADC1 = true;



  params._useADC2 = true;



  params._gain_1 = 2 /* Range for 0 to 2048 mV*/;



  params._gain_2 = 2 /* Range from 0 to 2048 mV*/;







  strcpy(params._p1_2, "CO2");



  strcpy(params._p1_3, "H");



  strcpy(params._p1_4, "T");







  strcpy(params._p2_2, "NH3");
# 363 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
}

/**

 * Shows and handles the boot up commands.

 */
# 368 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
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

  sodaq_wdt_reset(); // Reset de WDT

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
# 513 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
void publishMessage()
{
  // sodaq_wdt_disable();

  /* Defintions of the multipliers */

  /* Send all message in the array record to azure. */
  sodaq_wdt_disable();
  delay(500);
  sodaq_wdt_enable(WDT_PERIOD_8X);
  SerialUSB.println("EDT Set for 8 sec");

  unsigned long timeStamp = getTime();
  //String endPoint;
  //String jsonString;
  //String deviceId;

  //String Status = "Succes";
  //String Message = "";

  SerialUSB.println("*** Entering publish message ***");

  //deviceId = params.getDeviceName();
  SerialUSB.print("Name of device:");
  SerialUSB.println(params.getDeviceName());

  strcpy(sendBuffer, "{");
  strcat(sendBuffer, "\"Type\":");
  strcat(sendBuffer, "\"Data\"");
  strcat(sendBuffer, ",");
  strcat(sendBuffer, "\"DeviceName\":");
  strcat(sendBuffer, "\"");
  strcat(sendBuffer, "\"A04072203\":");
  strcat(sendBuffer,"\"");
  strcat(sendBuffer,"," );
  strcat(sendBuffer, "\"Timestamp\":");
  strcat(sendBuffer, String(getTime()).c_str() );
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"CO2\":");
  strcat(sendBuffer, String((CO2) * 0.06250F).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"H\":");
  strcat(sendBuffer, String((H) * 0.06250F).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"T\":");
  strcat(sendBuffer, String((T) * 0.06250F).c_str());
  strcat(sendBuffer,",");
  strcat(sendBuffer, "\"NH3\":");
  strcat(sendBuffer, String((NH3) * 0.06250F).c_str());
  strcat(sendBuffer, "}");

  SerialUSB.println("Format send to MQTT");
  SerialUSB.println(sendBuffer);

  sodaq_wdt_reset(); // Reset de WDT

  SerialUSB.println("sending data...");
  mqttClient.beginMessage("devices/" + DeviceName + "/messages/events/");
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

 // Serial.println("Disable WDT");
//  sodaq_wdt_disable();

  strcpy(_mqttEndpoint, MQTTBroker.c_str());
  SerialUSB.print("Attempting to MQTT broker: ");
  SerialUSB.print(_mqttEndpoint);
  SerialUSB.println(" ");

  while (!mqttClient.connect(_mqttEndpoint, 8883))
  {
    // failed, retry
    SerialUSB.print(".");
    SerialUSB.println(mqttClient.connectError());
    sodaq_wdt_safe_delay(5000);
  }

  SerialUSB.println();

  SerialUSB.println("You're connected to the MQTT broker");
  SerialUSB.println();

  // subscribe to a topic
  SerialUSB.print("Device is listening to endPoint :");
  SerialUSB.println(endPoint);
  mqttClient.subscribe(endPoint);
//  Serial.println("Enble WDT");
//  sodaq_wdt_enable(WDT_PERIOD_8X);
}

/* Lees de datauit van de methaan sensor. Omdat nog niet duidelijk hoe dit wordt gedaan wordt er hier een random getal genomen tussen de 0 en 10000. */
void getSensorData()
{
  SerialUSB.println("Reading ADC converters....");
  int16_t value;


  CO2 = ads1115_48.readADC_SingleEnded(2);
  SerialUSB.println("Reading CO2");
  sodaq_wdt_safe_delay(1000);
  H = ads1115_48.readADC_SingleEnded(1);
  SerialUSB.println("Reading H....");
  sodaq_wdt_safe_delay(1000);
  T = ads1115_48.readADC_SingleEnded(0);
  SerialUSB.println("Reading Temp....");
  sodaq_wdt_safe_delay(1000);
  NH3 = ads1115_49.readADC_SingleEnded(2);
  SerialUSB.println("Reading NH3....");
  sodaq_wdt_safe_delay(1000);
  MeasurementTime = getTime();

/* Send values to display */
  SerialUSB.println("***************** Actual values read from port ***************************** ");
  SerialUSB.print("CO2 :"); SerialUSB.println(CO2);
  SerialUSB.print("H   :"); SerialUSB.println(H);
  SerialUSB.print("T   :"); SerialUSB.println(T);
  SerialUSB.print("NH3 :"); SerialUSB.println(NH3);
  SerialUSB.println("**************************************************************************** ");



  if (params._isLedEnabled)
  {
    blinkLed(1);
  }
  //record->time = getTime();
}

/*

 * OnMessageReceived:

 *

 * Set variables from Azure.

 */
# 657 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
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

      if ((interval >= 100 && interval <= 120000) && interval != -1)
      {
        SerialUSB.print("Setting measurement interval to :");
        SerialUSB.print(interval);
        SerialUSB.println(" ms");
        params._defaultMeasurementInterval = interval;
      }

      if ((buffer > 0 && buffer <= 15) && buffer != -1)
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
