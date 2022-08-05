#line 1 "c:\\Marien\\Sources\\Arduino\\Params.cpp"
#include "Params.h"
#include "Command.h"
#include "FlashStorage.h"

#define DEFAULT_HEADER 0xBEEF

ConfigParams params;

FlashStorage(flash, ConfigParams);
static bool needsCommit;
static VoidCallbackMethodPtr configResetCallback;

static uint16_t crc16ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0;
    while (len--)
    {
        crc ^= (*buf++ << 8);
        for (uint8_t i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void ConfigParams::read()
{
    flash.read(this);

    // check header and CRC
    uint16_t calcCRC16 = crc16ccitt((uint8_t *)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);
    if (_header != DEFAULT_HEADER || _crc16 != calcCRC16)
    {
        reset();
    }
}

void ConfigParams::reset()
{
    // _defaultSendInterval = 180;
    // _defaultMeasurementInterval = 60;
    // _defaultNumberOfMeasurements = 1;                 //Number of readings before taking the median (15) = 30 seconden
    // _defaultBatchSize = 10;                            //Total number of batches before sending to T-Mobile (10) = 30 sec. * 10 = 300 sec. 5 vijf minuten interval
    // _gpsFixTimeout = 120;
    // _gpsMinSatelliteCount = 4;

    if (configResetCallback)
    {
        configResetCallback();
    }

    needsCommit = true;
}

void ConfigParams::commit(bool forced)
{
    if (!forced && !needsCommit)
    {
        return;
    }

    _header = DEFAULT_HEADER;
    _crc16 = crc16ccitt((uint8_t *)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);

    flash.write(*this);

    needsCommit = false;
}

static const Command args[] = {
    {"Port assignments              ", 0, 0, Command::show_title, 0},
    {"Sensor settings               ", 0, 0, Command::show_title, 0},
    {"Interval (ms)                 ", "int=", Command::set_uint16, Command::show_uint16, &params._defaultMeasurementInterval},
    {"Cellular                      ", 0, 0, Command::show_title, 0},
    {"APN                           ", "apn=", Command::set_string, Command::show_string, params._apn, sizeof(params._apn)},
    {"PIN                           ", "pin=", Command::set_string, Command::show_string, params._pinnumber, sizeof(params._pinnumber)},
    {"Azure                         ", 0, 0, Command::show_title, 0},
    {"End Point Data                ", "apd=", Command::set_string, Command::show_string, params._endPointData, sizeof(params._endPointData)},
    {"Devicenaam                    ", "dev=", Command::set_string, Command::show_string, params._deviceName, sizeof(params._deviceName)},
    {"Misc                          ", 0, 0, Command::show_title, 0},
    {"Status LED (OFF=0 / ON=1)     ", "led=", Command::set_uint8, Command::show_uint8, &params._isLedEnabled},
    {"Debug (OFF=0 / ON=1)          ", "dbg=", Command::set_uint8, Command::show_uint8, &params._isDebugOn}};

void ConfigParams::showConfig(Stream *stream)
{
    stream->println();
    stream->println("Settings:");
    for (size_t i = 0; i < sizeof(args) / sizeof(args[0]); ++i)
    {
        const Command *a = &args[i];
        if (a->show_func)
        {
            a->show_func(a, stream);
        }
    }
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
bool ConfigParams::execCommand(const char *line)
{
    bool done = Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
    if (done)
    {
        needsCommit = true;
    }

    return done;
}

/*
 * Check if all required config parameters are filled in
 */
bool ConfigParams::checkConfig(Stream &stream)
{
    bool fail = false;

    return !fail;
}

void ConfigParams::setConfigResetCallback(VoidCallbackMethodPtr callback)
{
    configResetCallback = callback;
}
