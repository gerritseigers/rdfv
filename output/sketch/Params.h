#line 1 "c:\\Projects\\rdfv\\Arduino\\Params.h"
#ifndef PARAMS_H_
#define PARAMS_H_

#include <stdint.h>
#include <Arduino.h>

typedef void (*VoidCallbackMethodPtr)(void);

struct ConfigParams
{
    uint16_t _header;

    uint8_t _useGPS;
    uint16_t _gpsFixTimeout;
    uint8_t _gpsMinSatelliteCount;

    bool _useADC1;
    bool _useADC2;
    bool _useADC3;
    bool _useADC4;

    uint8_t _gain_1;
    uint8_t _gain_2;
    uint8_t _gain_3;
    uint8_t _gain_4;

    char _p1_1[32]; // Asigns a name to port 1 and enable this port.
    char _p1_2[32]; // Asigns a name to port 2 and enable this port.
    char _p1_3[32]; // Asigns a name to port 3 and enable this port.
    char _p1_4[32]; // Asigns a name to port 4 and enable this port.

    char _p2_1[32]; // Asigns a name to port 1 and enable this port.
    char _p2_2[32]; // Asigns a name to port 2 and enable this port.
    char _p2_3[32]; // Asigns a name to port 3 and enable this port.
    char _p2_4[32]; // Asigns a name to port 4 and enable this port.

    char _p3_1[32]; // Asigns a name to port 1 and enable this port.
    char _p3_2[32]; // Asigns a name to port 2 and enable this port.
    char _p3_3[32]; // Asigns a name to port 3 and enable this port.
    char _p3_4[32]; // Asigns a name to port 4 and enable this port.

    char _p4_1[32]; // Asigns a name to port 1 and enable this port.
    char _p4_2[32]; // Asigns a name to port 2 and enable this port.
    char _p4_3[32]; // Asigns a name to port 3 and enable this port.
    char _p4_4[32]; // Asigns a name to port 4 and enable this port.

    uint16_t _defaultSendInterval;
    uint16_t _defaultMeasurementInterval;
    uint16_t _defaultNumberOfMeasurements; // Total number of measurements on port of ADC
    uint16_t _defaultBatchSize;            // Total number of batches before sending to T-Mobile
    uint16_t _defaultRepeats;

    char _apn[32 + 1];
    char _pinnumber[10];

    char _endPointData[128 + 1];
    char _endPointControl[128 + 1];
    char _azureIOTEndPoint[128 + 1]; // Assigned endpoint to this device.
    char _deviceName[128 + 1];       // Assigned devicename

    uint8_t _isDebugOn;
    uint8_t _isLedEnabled;
    uint8_t _isDeviceRegistered;

    uint16_t _crc16;

public:
    void read();
    void reset();
    void commit(bool forced = false);

    bool execCommand(const char *line);

    uint8_t getUseGPS() const { return _useGPS; }
    uint16_t getDefaultSendInterval() const { return _defaultSendInterval; }
    uint16_t getDefaultMeasurementInterval() const { return _defaultMeasurementInterval; }
    uint16_t getDefaultNumberOfMeasurements() const { return _defaultNumberOfMeasurements; }
    uint16_t getDefaultBatchSize() const { return _defaultBatchSize; }
    uint16_t getGPSFixTimeout() const { return _gpsFixTimeout; }
    uint16_t getGPSMinSateliteCount() const { return _gpsMinSatelliteCount; }

    const char *getDeviceName() const { return _deviceName; }
    const char *getMQTTBroker() const { return _endPointData; }
    const char *getPINNumber() const { return _pinnumber; }
    const char *getAzureIOTEndPoint() const { return _azureIOTEndPoint; }
    const char *getAPN() const { return _apn; }

    static void showConfig(Stream *stream);
    bool checkConfig(Stream &stream);
    void setConfigResetCallback(VoidCallbackMethodPtr callback);
};

extern ConfigParams params;

#endif
