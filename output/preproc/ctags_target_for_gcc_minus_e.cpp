# 1 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
/*=================================================================================================

 * File:            17553_ArdLogger.ino

 * Author:          G. Seigers / J. de Pagter

 * Created date:    -

 * Description:     Arduino program to run the MKR1500 NB with GPRS to Azure cloud (MQTT) to read

 * some sensors for remote monitoring

=================================================================================================*/
# 9 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
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
# 20 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 21 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 22 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 23 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 24 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2
# 25 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino" 2


// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
# 57 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
//#define PORT1 "T"



//#define PORT5 "G"

// #define PORT7 ""
// #define PORT8 ""







//Used for SD card
const int chipSelect = 4;

/* Define the constants for the IOTHub and T-Mobile */
const char PINNUMBER[] = "031591"; // PIN Number
// const char APN[]              = "iot.t-mobile.nl";                                               // APN T-Mobile

String response;
static uint8_t lastResetCause; // Veld waarin staat wat de oorzaak is van de laatste reset.
String IMEI = "";
String DeviceName;
String MQTTBroker = "";
String Error = "";
char sendBuffer[500];
bool sdPresent = false;
int measurementPointer = 0;
// DataRecord measurements[MAX_NUMBER_OF_MEASUREMENTS];
// int measurementPointer = 0; // Actueel aantal metingen dat gedaan is.

//Variables for reading the ports
int16_t P1[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/];
int16_t CO2[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/]; //Port2 p1_2
int16_t H[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/]; //Port3 p1_3 
int16_t T[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/]; //Port4 p1_4
int16_t P5[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/];
int16_t NH3[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/]; //Port6 p2_2
int16_t P7[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/];
int16_t P8[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/];
int32_t MeasurementTime[15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/]; //Time of measurement

int16_t repeatCounter; // 
int16_t wdtCounter;

GPRS gprs;
NB nbAccess(false); // Set op true om te debuggen
NBClient nbClient; // Used for the TCP socket connection
BearSSLClient sslClient(nbClient); // Used for SSL/TLS connection, integrates with ECC508
MqttClient mqttClient(sslClient);
NBModem modem;
NBScanner nbScanner;
RTCZero rtc;

// Variabels for MQTT
String username;
String broker;

//Adafruit_ADS1015 ads1115_48;
//Adafruit_ADS1015 ads1115_49;

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
void setTime();
// String dataFileFromDate();
// String logFileFromDate();

// void setGain(Adafruit_ADS1115 &sensor, uint8_t gain);
// double getMultiplier(int portNumber); // Returns the multiplier for this specific portnumber

void (*resetFunc)(void) = 0; // Functie voor harde reset. Wordt aangeroepen als buffer overloopt.

void setup()
{
  SerialUSB.begin(115200);
  Serial1.begin(115200);

  //Initialize SD card.
  SerialUSB.println("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    SerialUSB.println("SD card failed, or not present");
    sdPresent = false;
  }
  else
  {
    sdPresent = true;
    SerialUSB.println("SD card initialized.");
  }

  lastResetCause = ((Pm *)0x40000400UL) /**< \brief (PM) APB Base Address */->RCAUSE.reg;
  blinkLed(5);

  //ads1115_48.begin(0x48);
  //ads1115_49.begin(0x49);
  analogReadResolution(12);

    // if coming from WDT reset!
  if (lastResetCause & (0x1ul << 5 /**< \brief (PM_RCAUSE) Watchdog Reset */))
  {
    SerialUSB.println("WDT reset executed!");
    writeToLogFile("WDT reset executed!");
  }
  else
  {
    SerialUSB.println("Normal POR executed!");
    writeToLogFile("Normal POR executed!");
    setupModem();
  }

  // Disable the WDT tijdens het setup proces
  SerialUSB.println("WDT is disabled");
  sodaq_wdt_disable();

  params.read();
  params.setConfigResetCallback(onConfigReset);
  params.read();

  /* Initialize the ECCX08 encrytion chip and reconstyruct the selfsigned certificate */
  if (!ECCX08.begin())
  {
    SerialUSB.println("No ECCX08 present!, reboot device");
    writeToLogFile("No ECCX08 present!, reboot device");
    resetFunc();
  }
  // Wait for the sensor to be ready

  ECCX08SelfSignedCert.beginReconstruction(0, 8);
  ECCX08SelfSignedCert.setCommonName(ECCX08.serialNumber());
  ECCX08SelfSignedCert.endReconstruction();
  ArduinoBearSSL.onGetTime(getTime); // Set a callback fur the current time. So the certificate can be validated
  sslClient.setEccSlot(0, ECCX08SelfSignedCert.bytes(), ECCX08SelfSignedCert.length());

  DeviceName = params._deviceName;
  MQTTBroker = params.getMQTTBroker();
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


  //ads1115_48.setGain(GAIN_TWO);
  //ads1115_49.setGain(GAIN_TWO);

  repeatCounter = 0;

  modem.begin();
  connectNB();
  setTime();
  SerialUSB.println("Set the WDT to 8 seconds");
  sodaq_wdt_enable(WDT_PERIOD_8X); // WDT is gezet op 8 seconden.
  sodaq_wdt_reset();

}

//=================================================================================================
// Function:    loop
// Return:      -
// Description: Main loop function within Arduino environment
//=================================================================================================
void loop()
{
 sodaq_wdt_enable(WDT_PERIOD_8X);
  sodaq_wdt_reset(); // Reset de WDT

  if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY)
  {
    SerialUSB.println("No connection with T-Mobile");
    writeToLogFile("No connection with T-Mobile");
    connectNB();
  }

  if (!mqttClient.connected() && 1 /*Moet de data worden verstuurd naar Azure.        */ == 1)
  {
    SerialUSB.println("No connection with Azure");
    writeToLogFile("No connection with Azure");
    connectMQTT();
  }

  blinkLed(1);

  if (repeatCounter == 0)
  {
    getSensorData();
    if (measurementPointer >= 1 /*Aantal metingen voordat er wordt geprobeerd om de data te versturen (10).*/)
    {
      SerialUSB.println("measurement pointers is greater or equal the treshold");
      writeToLogFile("measurement pointers is greater or equal the treshold");

      if (measurementPointer >= 15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/)
      {
          SerialUSB.println("Reboot arduino because buffer overflow");
          writeToLogFile("Reboot arduino because buffer overflow");
          resetFunc();
      }
      else
      {
        if (1 /*Moet de data worden verstuurd naar Azure.        */ == 1)
        {
          SerialUSB.println("trying to send data to the IOT-HUB");
          writeToLogFile("trying to send data to the IOT-HUB");

          int signalStrength = nbScanner.getSignalStrength().toInt();
          SerialUSB.print("Signal strength :");
          SerialUSB.print(signalStrength);
          SerialUSB.println(" dB");

          if (signalStrength != 99 /* What signal is detected when modem starts with no antenne or still initializing*/ && signalStrength > 15 /* get from modem the detected carrier (31 > 51 dBm)*/ )
          {

            blinkLed(5);
            SerialUSB.println("Send message to IOT-HUB enough signal strength");
            writeToLogFile("Send message to IOT-HUB enough signal strength");
            publishMessage();
            measurementPointer = 0;
          }
          else
          {
            SerialUSB.println("To weak signal to send. Trying again in next loop");
            writeToLogFile("To weak signal to send. Trying again in next loop");
            measurementPointer += 1;
          }
        }
        else
        {
          SerialUSB.println("Not sending to Azure because of configuration setting");
          writeToLogFile("Not sending to Azure because of configuration setting");
          measurementPointer = 0;
        }
      }
    }
    else
    {
      SerialUSB.print("Number of message in cache ");
      SerialUSB.print(measurementPointer);
      SerialUSB.print(" of ");
      SerialUSB.println(15 /* Maximum aantal metingen dat in het buffer mag staan. Deze moet altijd groter zijn dan de parameter DEFAULT_NUMBER_OF_MEASUREMENTS*/);

      measurementPointer += 1;
    }
  }
  repeatCounter += 1;
  if (repeatCounter >= 5 /*aantal seconden voordat de er een nieuwe meting wordt uitgevoerd (58).*/ )
  {
    repeatCounter = 0;
  }


  SerialUSB.print("Waiting for next measurement. Value of repeatcounter :");
  SerialUSB.println(repeatCounter);
  if (1 /*Moet de data worden verstuurd naar Azure.        */ == 1)
  {
    SerialUSB.println("Polling mqtt");
    mqttClient.poll();
  }
  delay(800);
}

//=================================================================================================
// Function:    getTime
// Return:
// Description: Get UTC time from Modem 
//=================================================================================================
unsigned long getTime()
{
  return (rtc.getEpoch());
}

//=================================================================================================
// Function:    printBootUpMessage
// Return:      -
// Description: Prints a boot-up message that includes project name, version and Cpu reset cause.
//=================================================================================================
static void printBootUpMessage(Stream &stream)
{
  stream.println("** " "Project Marien" " - " "1.1.0" " **");

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


  params._isDeviceRegistered = true;



  strcpy(params._deviceName, "A04072212" /* THIS CODE MUST CHANGED FOR EVERY ARDUIO !!!!!*/);



  strcpy(params._pinnumber, "915684");



  strcpy(params._apn, "iot.t-mobile.nl");



  params._isDebugOn = true;



  strcpy(params._endPointData, "euw-iothub-rdfv-pr.azure-devices.net");



  params._defaultMeasurementInterval = 5 /*aantal seconden voordat de er een nieuwe meting wordt uitgevoerd (58).*/;



  params._defaultNumberOfMeasurements = 1;



  params._isLedEnabled = 1;



  params._useADC1 = true;



  params._useADC2 = true;



  params._gain_1 = 2 /* Range for 0 to 2048 mV*/;



  params._gain_2 = 2 /* Range from 0 to 2048 mV*/;







  strcpy(params._p1_2, "CO2");



  strcpy(params._p1_3, "H");



  strcpy(params._p1_4, "T");







  strcpy(params._p2_2, "NH3");
# 466 "c:\\Projects\\rdfv\\Arduino\\Arduino.ino"
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
    showBootMenu(SerialUSB);
  } while (!params.checkConfig(SerialUSB));

  params.showConfig(&SerialUSB);
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

  SerialUSB.println('=============================================================');
  SerialUSB.println('New Json');
  SerialUSB.println(sendBuffer);

  SerialUSB.println("Just before sending data");
  mqttClient.beginMessage("devices/" + deviceId + "/messages/events/");
  mqttClient.print(sendBuffer);
  mqttClient.endMessage();
  SerialUSB.println("End sending data");
}

//=================================================================================================
// Function:    sendErrorToEndpoint
// Return:      -
// Description: Send error to MQTT client
//=================================================================================================
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

  SerialUSB.println("*** Entering publish message ***");

  SerialUSB.print("Name of device:");
  SerialUSB.println(params.getDeviceName());

  for (int ii=0;ii<measurementPointer;ii++)
  {
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
    strcat(sendBuffer, String(MeasurementTime[ii]).c_str() );
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"P1\":");
    strcat(sendBuffer, String((P1[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"CO2\":");
    strcat(sendBuffer, String((CO2[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"H\":");
    strcat(sendBuffer, String((H[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"T\":");
    strcat(sendBuffer, String((T[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"P5\":");
    strcat(sendBuffer, String((P5[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"NH3\":");
    strcat(sendBuffer, String((NH3[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"P7\":");
    strcat(sendBuffer, String((P7[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"P8\":");
    strcat(sendBuffer, String((P8[ii])).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"RC\":");
    strcat(sendBuffer, String((lastResetCause)).c_str());
    strcat(sendBuffer,",");
    strcat(sendBuffer, "\"SS\":");
    strcat(sendBuffer, String((signalStrength)).c_str());
    strcat(sendBuffer, "}");

    SerialUSB.println("Format send to MQTT");
    SerialUSB.println(sendBuffer);

    sodaq_wdt_reset(); // Reset de WDT

    SerialUSB.println("sending data...");
    mqttClient.beginMessage("devices/" + DeviceName + "/messages/events/");
    mqttClient.print(sendBuffer);
    mqttClient.endMessage();
    SerialUSB.println("End sending data");
    mqttClient.poll();
  }
}

//=================================================================================================
// Function:    connectMQTT
// Return:      -
// Description: Connect to the MQTT Azure server
//=================================================================================================
void connectMQTT()
{

  int retryCounter = 0;

  String endPoint;
  char _mqttEndpoint[128 + 1];
  endPoint = "devices/";
  endPoint += DeviceName;
  endPoint += "/messages/devicebound/#";

  SerialUSB.println("Reset WDT");
  sodaq_wdt_reset();

  strcpy(_mqttEndpoint, MQTTBroker.c_str());
  SerialUSB.print("Attempting to MQTT broker: ");
  SerialUSB.print(_mqttEndpoint);
  SerialUSB.println(" ");

  while (!mqttClient.connect(_mqttEndpoint, 8883))
  {
    // failed, retry
    retryCounter += 1;
    SerialUSB.print(".");
    SerialUSB.println(mqttClient.connectError());
    writeToLogFile("Connection error with IOT-HUB. ");

    sodaq_wdt_safe_delay(1000);
    if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
      writeToLogFile("Connection error with IOT-HUB make new connection with T-Mobile");
      connectNB();
    }

    if (retryCounter == 5)
    {
      SerialUSB.println("Retry counter for connecting to IOT-HUB is 5. Reboot");
      writeToLogFile("Retry counter for connecting to IOT-HUB is 5. Reboot");
      resetFunc();
    }
  }

  SerialUSB.println();

  SerialUSB.println("You're connected to the MQTT broker");
  writeToLogFile("You're connected to the MQTT broker");

  SerialUSB.println();

  // subscribe to a topic
  SerialUSB.print("Device is listening to endPoint :");
  SerialUSB.println(endPoint);
  mqttClient.subscribe(endPoint);
}

//=================================================================================================
// Function:    getSensorData
// Return:      -
// Description: Read out ADC values
//=================================================================================================
void getSensorData()
{



  long port_0=0;
  long port_1=0;
  long port_2=0;
  long port_3=0;
  long port_4=0;
  long port_5=0;
  long port_6=0;

  SerialUSB.println("Reading ADC converters (v.1.0)");
  writeToLogFile("Reading ADC converters....");

  for(int ii=0;ii<10;ii++)
  {
    SerialUSB.print("Measurement :");
    SerialUSB.print(ii);
    SerialUSB.print(" out of ");
    SerialUSB.println(10);

    port_0 += analogRead(A0);
    delay(200);
    port_1 += analogRead(A1);
    delay(200);
    port_2 += analogRead(A2);
    delay(200);
    port_3 += analogRead(A3);
    delay(200);
    // port_4 += analogRead(A4);
    // delay(200);
    // port_5 += analogRead(A5);
    // delay(200);
    // port_6 += analogRead(A6);
    // delay(200);
  }

  P1[measurementPointer] = int( (port_0 / 10) * (3300.0F / 4096.0F) );
  CO2[measurementPointer] = int( (port_1 / 10) * (3300.0F / 4096.0F) );
  H[measurementPointer] = int( (port_2 / 10) * (3300.0F / 4096.0F) );
  T[measurementPointer] = int( (port_3 / 10) * (3300.0F / 4096.0F) );
  P5[measurementPointer] = 0;
  NH3[measurementPointer] =0;
  P7[measurementPointer] = 0;
  P8[measurementPointer] = 0;
  MeasurementTime[measurementPointer] = getTime();
  SerialUSB.println("Reading ports done, write data to SD card");

  SerialUSB.println("Reading ports done, write data to SD card");

  SerialUSB.print("Time :");
  SerialUSB.println(MeasurementTime[measurementPointer]);

  SerialUSB.print("H :");
  SerialUSB.println(H[measurementPointer]);

  SerialUSB.print("T :");
  SerialUSB.println(T[measurementPointer]);

  SerialUSB.print("CO2 :");
  SerialUSB.println(CO2[measurementPointer]);

  // Serial.print("NH3 :");
  // Serial.println(NH3[measurementPointer]);

  SerialUSB.print("P1 :");
  SerialUSB.println(P1[measurementPointer]);

  // Serial.print("P5 :");
  // Serial.println(P5[measurementPointer]);

  // Serial.print("P7 :");
  // Serial.println(P7[measurementPointer]);

  // Serial.print("P8 :");
  // Serial.println(P8[measurementPointer]);


  writeToDataFile(MeasurementTime[measurementPointer], H[measurementPointer], T[measurementPointer], CO2[measurementPointer], NH3[measurementPointer], P1[measurementPointer], P5[measurementPointer], P7[measurementPointer], P8[measurementPointer]);

  if (params._isLedEnabled)
  {
    blinkLed(1);
  }
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
  SerialUSB.print("Received a message with topic");
  writeToLogFile("Received a message with topic");

  char str[messageSize + 1];
  int ii = 0;
  for (ii = 0; ii < messageSize; ii++)
  {
    str[ii] = (char)mqttClient.read();
  }
  str[ii] = 0; // Terminate de string met een 0
  SerialUSB.print(str);
  SerialUSB.println();
  writeToLogFile(str);

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

      if ((repeats > 0 && repeats < 3600))
      {
        SerialUSB.print("Setting repeats to :");
        SerialUSB.println(repeats);
        params._defaultRepeats = repeats;
        needsReboot = true;
      }

      params.commit(true);
      publishSettings();
      if (needsReboot)
      {
        SerialUSB.println("Rebooting device....");
        writeToLogFile("Rebooting device on request");
        resetFunc();
      }
    }
    else if (strcmp(_type, "Info") == 0)
    {
      SerialUSB.println("Asking for info. Sending settings to Azure.");
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

//=================================================================================================
// Function:    blinkLed
// Return:      -
// Description: Blinks the led  
//=================================================================================================
void blinkLed(int times)
{
  for (int ii = 0; ii < times; ii++)
  {
    digitalWrite((6u), HIGH); // turn the LED on (HIGH is the voltage level)
    delay(200);
    digitalWrite((6u), LOW); // turn the LED off by making the voltage LOW
    delay(200);
  }
}

//=================================================================================================
// Function:    writeToLogFile
// Return:      -
// Description: Write string to logfile on SD Card. 
//=================================================================================================
void writeToLogFile(String message)
{
  String logfileName = logFileFromDate();
  SerialUSB.print("Name of logfile: ");
  SerialUSB.println(logfileName);

  if (sdPresent) {
    SerialUSB.print("Writing :");
    SerialUSB.print(message);
    SerialUSB.println(" to logile");

    File logFile = SD.open(logfileName, (O_READ | O_WRITE | O_CREAT | O_APPEND));
    if (logFile) {
      SerialUSB.println("Write log");
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
  SerialUSB.println("Write to datafile");
  String datafileName = dataFileFromDate();
  if (sdPresent) {
    File dataFile = SD.open(datafileName, (O_READ | O_WRITE | O_CREAT | O_APPEND));
    dataFile.print(time);
    dataFile.print(",");
    dataFile.print(P1);
    dataFile.print(",");
    dataFile.print(CO2);
    dataFile.print(",");
    dataFile.print(H);
    dataFile.print(",");
    dataFile.print(T);
    dataFile.print(",");
    dataFile.print(P5);
    dataFile.print(",");
    dataFile.print(NH3);
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
  SerialUSB.println("Attempting to connect to the cellular network");
  writeToLogFile("Attempting to connect to the cellular network");

  //Connect only to T-Mobile if the signal strength is OK
  while ( (nbScanner.getSignalStrength().toInt() < 15 /* get from modem the detected carrier (31 > 51 dBm)*/) || (nbScanner.getSignalStrength().toInt() == 99 /* What signal is detected when modem starts with no antenne or still initializing*/) )
    {
        SerialUSB.println("No signal!");
        writeToLogFile("No signal in connectNB method");
        delay(500);
    }

  while ((nbAccess.begin(params._pinnumber, params._apn, true) != NB_READY) ||
         (gprs.attachGPRS() != GPRS_READY))
  {
    // failed, retry
    SerialUSB.print(".");
    delay(1000);
  }
  SerialUSB.println("You're connected to the cellular network");
  writeToLogFile("You're connected to the cellular network");
  SerialUSB.println();
}

//=================================================================================================
// Function:    setupModem
// Return:      -
// Description: set the Modem
//=================================================================================================
void setupModem()
{
  SerialUSB.println("Waiting for modem to get ready...");
  writeToLogFile("Waiting for modem to get ready...");

  MODEM.begin();
  while (!MODEM.noop())
    ;

  /* Disconnect from any networks. */
  SerialUSB.print("Disconnecting from network: ");
  writeToLogFile("Disconnecting from network: ");

  MODEM.sendf("AT+COPS=2");
  MODEM.waitForResponse(2000);
  SerialUSB.println("done.");

  // Set Radio Access Technology (RAT). For this demo it will be set on Cat M1 (LTE-M)
  SerialUSB.println("Set Radio Access Technology to LTE-M");
  writeToLogFile("Set Radio Access Technology to LTE-M");

  MODEM.sendf("AT+URAT=7");
  MODEM.waitForResponse(100, &response);
  SerialUSB.println("done.");

  // Save changes to the modem.
  SerialUSB.print("Applying changes and saving configuration: ");
  writeToLogFile("Applying changes and saving configuration: ");

  MODEM.sendf("AT+CFUN=15");
  do
  {
    delay(100);
    SerialUSB.print(".");
    MODEM.noop();
  } while (MODEM.waitForResponse(1000) != 1);
  SerialUSB.println("done.");

  SerialUSB.println("Modem ready, turn radio on in order to configure it...");
  writeToLogFile("Modem ready, turn radio on in order to configure it...");

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
  SerialUSB.println("done.");

  // Set operator to T-Mobile
  // Serial.println("Set operator to T-Mobile...");
  // dataFile = SD.open(logFile, FILE_WRITE);			// toegevoegd Carltech
  // dataFile.println("Set operator to T-Mobile...");
  SerialUSB.println("Set operator to KPN...");
  writeToLogFile("Set operator to KPN...");

  //MODEM.sendf("AT+COPS=1,2,\"20416\"");
  MODEM.sendf("AT+COPS=1,2,\"20408\"");
  MODEM.waitForResponse(2000, &response);
  SerialUSB.print("Operator response: ");
  SerialUSB.println(response);
  SerialUSB.println("done.");
}

char stringTochar(String s)
{
  char arr[12];
  s.toCharArray(arr, sizeof(arr));
  return atol(arr);
}

int freeMemory() {
  char top;

  return &top - reinterpret_cast<char*>(sbrk(0));





}

//=================================================================================================
// Function:    setTime()
// Return:      -
// Description: Get the time from the telecom provider en set this to the RTC.
//=================================================================================================
void setTime()
{
  struct tm t = {0};
  time_t epoch;
  int YEAR, MONTH, DAY, HOUR, MINUTE, SECONDS, TZ;
  char timeBuffer[50];

  SerialUSB.println("Trying to get the time from T-Mobile...");
  writeToLogFile("Trying to get the time from T-Mobile...");

  if (nbAccess.status() == NB_READY || gprs.status() == GPRS_READY )
  {
    rtc.begin();

    SerialUSB.println("Connection established with T-Mobile");
    writeToLogFile("Connection established with T-Mobile");

    MODEM.send("AT+CCLK?");
    MODEM.waitForResponse(2000, &response);
    SerialUSB.print("Time: ");
    SerialUSB.println(response);

    int x = response.indexOf(String('"'))+1; //Get rit of AT stuff
    int y = response.lastIndexOf(String('"'));
    response.substring(x,y).toCharArray(timeBuffer,50); //Write string to char buffer

    //const char *timeData = "22/03/02,14:17:40+22"; 
    sscanf(timeBuffer, "%d/%d/%d,%d:%d:%d+%d", &YEAR, &MONTH, &DAY, &HOUR, &MINUTE, &SECONDS, &TZ);

    t.tm_year = YEAR + 100;
    t.tm_mon = MONTH - 1;
    t.tm_mday = DAY;
    t.tm_hour = HOUR;
    t.tm_min = MINUTE;
    t.tm_sec = SECONDS;
    epoch = mktime(&t); // -19800 TZ correction To get UTC

    byte year, month, day, hour, minute, seconds;
    year = (byte) YEAR;
    month = (byte) MONTH;
    day = (byte) DAY;
    hour = (byte) HOUR;
    minute = (byte) MINUTE;
    seconds = (byte) SECONDS;


    rtc.setTime(hour, minute, seconds);
    rtc.setDate(day, month, year );

    SerialUSB.print("RTCTime set to :");
    SerialUSB.println(rtc.getEpoch());
  }
  else
  {
    SerialUSB.println("No connection established with T-Mobile, Time not set");
    writeToLogFile("No connection established with T-Mobile, Time not set");
  }
}

String dataFileFromDate() {
  String result = "";
  if (rtc.getDay() < 10) result += "0";
  result += String(rtc.getDay());
  if (rtc.getMonth() < 10) result += "0";
  result += String(rtc.getMonth());
  if (rtc.getYear() < 10) result += "0";
  result += String(rtc.getYear());
  result += ".CSV";
  return result;
}

String logFileFromDate() {
  String result = "";
  if (rtc.getDay() < 10) result += "0";
  result += String(rtc.getDay());
  if (rtc.getMonth() < 10) result += "0";
  result += String(rtc.getMonth());
  if (rtc.getYear() < 10) result += "0";
  result += String(rtc.getYear());
  result += ".LOG";
  return result;
}
