/*
 * Technical Data from 'AirChip 3000 Communication Protocol Options' --> https://www.rotronic.com/pub/media/productattachments/files/e/-/e-m-ac3000-cp_25.pdf
 */

#ifndef airChip3000_h
#define airChip3000_h

#include <Arduino.h>
#include <SoftwareSerial.h>

// #define DEBUG

#define SENSOR_STATUS_OK 0
#define SENSOR_STATUS_NOK 1

#define MAX_MEASURE_INTERVAL 4000 //for minimal self heating effect and minimal hysteresis, keep interval >= 5000

class AirChip3000
{
public:
    AirChip3000(uint8_t TX_PIN = 8, uint8_t RX_PIN = 9) {
        AirChip3000_TX_PIN = TX_PIN;
        AirChip3000_RX_PIN = RX_PIN;
    }

    AirChip3000(SoftwareSerial *sSerial) {
        _sSerial = sSerial;
    }

    bool begin()
    {
#ifdef DEBUG
        Serial.println(F("[AirChip3000] init..."));
#endif
        _sSerial->begin(19200);
        _sSerial->setTimeout(500);

        //if (!testSensor())
        //    return false;
        if (!measure())
            return false;
        if (!readData())
            return false;
        if (!requestRecordingStatus())
            return false;

        fullyInited = true;

        return true;
    }

    uint8_t getSensorStatus() // Sensor status 0 is OK
    {
        testSensor();

        return sensorStatus;
    }

    uint8_t getSensorQuality() // Sensor quality 0 to 100 or 255 (good to bad or na)
    {
        testSensor();

        return sensorQuality;
    }

    float getTemperature(bool fahrenheit = false)
    {
        measure();

        if (fahrenheit)
            return (temperature * 1.8f) + 32.0f;

        return temperature;
    }

    float getHumidity()
    {
        measure();

        return relHumidity;
    }

    float getFactoryCorrectionA1()
    {
        measure();

        return factoryCorrectionA1;
    }

    float getUserCorrectionA2()
    {
        measure();

        return userCorrectionA2;
    }

    float getSensorTemperatureCorrection()
    {
        measure();

        return sensorTemperatureCorrection;
    }

    float getSensorDriftCorrection()
    {
        measure();

        return sensorDriftCorrection;
    }

    char *getDeviceType()
    {
        return deviceType;
    }

    char *getFirmwareVersion()
    {
        return firmwareVersion;
    }

    char *getDeviceSerialNumber()
    {
        return deviceSerialNumber;
    }

    char *getDeviceName()
    {
        return deviceName;
    }

    bool getIsRecording()
    {
        requestRecordingStatus();

        return isRecording;
    }

    bool stopRecording() {
        sprintf(cmd_buffer, STOP, addr);
        _sSerial->print(cmd_buffer);

        if (!waitForResponse())
            return false;

        return unpackRawData();
    }
    
    char *getRecordingMode() // 1 = Start-Stop mode (record until memory is full); 2 = Loop recording (dump oldest data when memory is full and keep recording)
    {
        requestRecordingStatus();

        return recordingMode;
    }

    uint16_t getLogInterval() // Log interval (in multiples of 5 seconds); 1 = 5s, 2 = 10s, etc.
    {
        requestRecordingStatus();

        return logInterval;
    }

    uint32_t getFirstDataDate()
    {
        requestRecordingStatus();

        return firstDataDate;
    }

    uint16_t getNDataRecords()
    {
        requestRecordingStatus();

        return nDataRecords;
    }

private:
    bool testSensor()
    {
        if (millis() - last_testSensor < MAX_MEASURE_INTERVAL && fullyInited)
            return true;

        sprintf(cmd_buffer, TST2, addr);
        _sSerial->print(cmd_buffer); // send test cmd
        delay(500);
        _sSerial->print(cmd_buffer); // send test cmd
        if (!waitForResponse((fullyInited ? 500 : 5000)))
        {
#ifdef DEBUG
            Serial.println(F("[AirChip3000] no response!"));
#endif
            return false;
        }

        unpackRawData();

        last_testSensor = millis();

        return true;
    }

    bool measure()
    {
        if (millis() - last_readData < MAX_MEASURE_INTERVAL && fullyInited)
            return true;

        // Send the RDD command
        sprintf(cmd_buffer, RDD, addr);
        _sSerial->print(cmd_buffer);

        if (!waitForResponse((fullyInited ? 500 : 5000)))
            return false;

        unpackRawData();

        last_readData = millis();

        return true;
    }

    bool readData()
    {
        if (millis() - last_readData < MAX_MEASURE_INTERVAL && fullyInited)
            return true;

        // Send the RDD command
        sprintf(cmd_buffer, RDD, addr); // "{F[addr]RDD}\r"
        _sSerial->print(cmd_buffer);

        if (!waitForResponse((fullyInited ? 500 : 5000)))
            return false;

        unpackRawData();

        last_readData = millis();

        return true;
    }

    bool requestRecordingStatus()
    {
        if (millis() - last_requestRecordingStatus < MAX_MEASURE_INTERVAL && fullyInited)
            return true;

        // Send the LGC1 command
        sprintf(cmd_buffer, LGC1, addr); // "{F[addr]LGC}\r"
        _sSerial->print(cmd_buffer);

        if (!waitForResponse())
            return false;

        unpackRawData();

        last_requestRecordingStatus = millis();

        return true;
    }

    bool waitForResponse(uint32_t time_out = 500) // The maximal response time of all devices described in this document is 500 ms.
    {
        if (!_sSerial->isListening())
            _sSerial->listen();

        // Wait for response with timeout
        uint32_t start_waiting = millis();
        while (!_sSerial->available() && millis() - start_waiting < time_out)
            ;

        if (_sSerial->available())
        {
            delay(9); // Allow time to receive 172 baud
            return true;
        }

        return false;
    }

    bool unpackRawData()
    {
        // Read the data into a buffer
        uint8_t nCount = _sSerial->readBytes(response_buffer, 160);
        response_buffer[nCount] = 0; // Set a string terminator
        _sSerial->flush();

        // Break down the response
        if (nCount > 0 && response_buffer[0] == '{')
        {
            if (strstr_P(response_buffer, PSTR("tst")) != NULL)
            {
                // Get responding address
                char _addr_str[2] = {response_buffer[2], response_buffer[3]};
                addr = strtoul((char *)_addr_str, NULL, 10);
                if (addr < 0 || addr > 64)
                {
#ifdef DEBUG
                    Serial.println(F("[AirChip3000] invalid address in response"));
#endif
                    addr = 99;
                }

                // Get first token (sensorStatus)
                char *token = strtok(response_buffer, ";");
                if (token == NULL)
                    return false;

                // TST 10 or TST 20 ?
                if (countOccurrences(response_buffer, ';') == 1)
                {
                    // TST 20
                    sensorStatus = strtoul(token, NULL, 10);

                    // Get next token (sensorQuality)
                    token = strtok(NULL, ";");
                    if (token == NULL)
                        return false;
                    sensorQuality = strtoul(token, NULL, 10);

#ifdef DEBUG
                    Serial.print(F("[AirChip3000] Sensor address is: "));
                    Serial.println(addr);

                    Serial.print(F("[AirChip3000] Sensor status is: "));
                    Serial.println(sensorStatus);

                    Serial.print(F("[AirChip3000] Sensor quality [0 to 100 or 255 (good to bad or na)] is: "));
                    Serial.println(sensorQuality);
#endif
                    return true;
                } else {
                    // TST 10
                    for (int i = 0; i < 19; i++)
                    {
                        if (token == NULL)
                            break;

                        switch (i)
                        {
                        case 0:
                            // counts
                            break;
                        case 1:
                            // raw humidity value [%RH] based on the count scaling
                            break;
                        case 2:
                            // factory correction A1%
                            factoryCorrectionA1 = atof(token); // Float
                            break;
                        case 3:
                            // user correction A2%
                            userCorrectionA2 = atof(token); // Float
                            break;
                        case 4:
                            // sensor temperature correction
                            sensorTemperatureCorrection = atof(token); // Float
                            break;
                        case 5:
                            // sensor drift correction
                            sensorDriftCorrection = atof(token); // Float
                            break;
                        case 6:
                            // humidity end value
                            relHumidity = atof(token); // Float
                            break;
                        case 7:
                            // Counts (x1000)
                            break;
                        case 8:
                            // resistance value [Ohm] as measured based on the count scaling
                            break;
                        case 9:
                            // numerical temperature end value
                            temperature = atof(token); // Float
                            break;
                        }

                        // Get next token
                        token = strtok(NULL, ";");
                    }

                    return true;
                }
            }
            else if (strstr_P(response_buffer, PSTR("rdd")) != NULL)
            {
                // Get first token
                char *token = strtok(response_buffer, ";");

                // loop through the string to extract all other tokens
                for (int i = 0; i < 19; i++)
                {
                    if (token == NULL)
                        break;

                    switch (i)
                    {
                    case 0:
                        probeType = token; // Byte
                        break;
                    case 1:
                        relHumidity = atof(token); // Float
                        break;
                    case 2:
                        rhUnit = token; // String
                        break;
                    case 3:
                        rhAlarm = token; // Bool (out-of-limits)
                        break;
                    case 4:
                        rhTrend = token; // Char (+,-,= or “ “)
                        break;
                    case 5:
                        temperature = atof(token); // Float
                        break;
                    case 6:
                        tempUnit = token; // String
                        break;
                    case 7:
                        tempAlarm = token; // Bool (out-of-limits)
                        break;
                    case 8:
                        tempTrend = token; // Char (+,-,= or “ “)
                        break;
                    case 9:
                        //  Dp; String; Calculated parameter type (nc: no calculation, Dp: dew point, Fp: frost point)
                        break;
                    case 10:
                        //  1234.56; Float; Calculated numerical value
                        break;
                    case 11:
                        //  °C; String; Calculated parameter engineering unit
                        break;
                    case 12:
                        //  0..1; Bool; Calculated parameter alarm (out-of-limits)
                        break;
                    case 13:
                        //  +; Char; Calculated parameter trend (+,-,= or “ “)
                        break;
                    case 14:
                        deviceType = token; //  1..255; Byte; Device type (HygroClip, Logger, HF, HM, ...)
                        break;
                    case 15:
                        firmwareVersion = token; //  V1.0; String; Firmware version
                        break;
                    case 16:
                        deviceSerialNumber = token; //  12345678; String; Device serial number
                        break;
                    case 17:
                        deviceName = token; //  Name; String; Device name
                        break;
                    case 18:
                        //  000...255; Byte; Alarm Byte: (Bit0=out-of-limits value, Bit5= sensor quality, Bit6 = humidity simulator, Bit7= temperature simulator)
                        break;
                    default:
                        break;
                    }

                    // Get next token
                    token = strtok(NULL, ";");
                }

                return true;
            }
            else if (strstr_P(response_buffer, PSTR("lgc")) != NULL)
            {
                if (strstr_P(response_buffer, PSTR("OK")) != NULL)
                {
                    // Response for start or stop recording data command
                    return true;
                }
                else
                {
                    // Response for requestRecordingStatus()
                    // Get first token
                    char *token = strtok(response_buffer, ";");

                    // loop through the string to extract all other tokens
                    for (int i = 0; i < 19; i++)
                    {
                        if (token == NULL)
                            break;

                        switch (i)
                        {
                        case 0:
                            // 0..3; Boolean; 0 = not recording data, 1= recording data;
                            // Loop recording only: 2 = recording data and memory full (see note below);
                            // Loop recording only: 3 = not recording data and memory full (see note below)
                            if (token == (char *)'0' || token == (char *)'3')
                                isRecording = false;
                            else
                                isRecording = true;
                            break;
                        case 1:
                            // 1..2; Byte; 1 = Start-Stop mode (record until memory is full)
                            // 2 = Loop recording (dump oldest data when memory is full and keep recording)
                            recordingMode = token;
                            break;
                        case 2:
                            // 1..65535; Word; Log interval (in multiples of 5 seconds)
                            // 1 = 5s, 2 = 10s, etc.
                            logInterval = strtoul(token, NULL, 10);
                            break;
                        case 3:
                            // 123456789; Long; Date and time of the first data sample referenced to Jan, 01, 2000 and expressed in
                            // increments of 5 seconds – see note 2 below
                            firstDataDate = strtoul(token, NULL, 10);
                            break;
                        case 4:
                            // 0...2000; Word; Number of data records (see note below)
                            nDataRecords = strtoul(token, NULL, 10);
                            break;
                        }

                        // Get next token
                        token = strtok(NULL, ";");
                    }

                    return true;
                }
            }
#ifdef DEBUG
            else
            {
                Serial.print(F("Response: "));
                Serial.println(response_buffer);
            }
#endif
        }
#ifdef DEBUG
        else
        {
            Serial.print(F("[AirChip3000] invalid response received"));
        }
#endif
        return false;
    }

    
    uint8_t countOccurrences(char *haystack, char needle)
    {
        uint8_t i;
        for (i = 0; haystack[i]; haystack[i] == needle ? i++ : *haystack++)
            ;
        return i;
    }

    uint8_t AirChip3000_TX_PIN = 8;
    uint8_t AirChip3000_RX_PIN = 9;

    SoftwareSerial *_sSerial; //= SoftwareSerial(AirChip3000_TX_PIN, AirChip3000_RX_PIN);

    uint8_t addr = 99; // RS485 Address of the device (00-64). Use 99 for unknown address (any address will respond, collisions may occur).

    bool fullyInited = false;
    uint32_t last_readData = 0;
    uint32_t last_testSensor = 0;
    uint32_t last_requestRecordingStatus = 0;

    // Data obtained from 'TST 10'
    float factoryCorrectionA1 = 0;
    float userCorrectionA2 = 0;
    float sensorTemperatureCorrection = 0;
    float sensorDriftCorrection = 0;

    // Data obtained from 'TST 20'
    uint8_t sensorStatus = 0; // What does the value say? 0/1
    uint8_t sensorQuality;

    // Data obtained from 'RDD'
    char *probeType;          // Byte (1=digital probe, 2=analog probe, 3=pressure probe)
    float relHumidity;        // Float
    char *rhUnit;             // String
    char *rhAlarm;            // Bool (out-of-limits)
    char *rhTrend;            // Char (+,-,= or “ “)
    float temperature;        // Float
    char *tempUnit;           // String
    char *tempAlarm;          // Bool (out-of-limits)
    char *tempTrend;          // Char (+,-,= or “ “)
    char *deviceType;         //  1..255; Byte; Device type (HygroClip, Logger, HF, HM, ...)
    char *firmwareVersion;    //  V1.0; String; Firmware version
    char *deviceSerialNumber; //  12345678; String; Device serial number
    char *deviceName;         //  Name; String; Device name

    // Data obtained from 'LGC'
    bool isRecording = false;
    char *recordingMode; // 1 = Start-Stop mode (record until memory is full); 2 = Loop recording (dump oldest data when memory is full and keep recording)
    uint16_t logInterval = 0;
    uint32_t firstDataDate = 0;
    uint16_t nDataRecords = 0;

    // Buffers
    char cmd_buffer[11];
    char response_buffer[160]; // holds response from AirChip3000

    // Commands
    char *RDD = (char *)"{F%02dRDD}\r"; // Returns the measured and calculated values as well as the information necessary to interpret the data (calculated parameter type, engineering units, status, serial number and name of the device, etc.)
    // char* REN = (char*)"{F%02dREN[deviceSerialNumber];[New Address]}\r"; //This command is used to change the RS-485 address (0-64) of the device
    // char* HCA = (char*)"{F%02dHCA}\r"; //This command is used to adjust the probe against a reference (humidity and temperature)
    char *LGC1 = (char *)"{F%02dLGC}\r"; // This command has two different formats and is used either to read (query) or to program the log function; Command format 1 (used to read the status of the data recording function)
    // char* LGC2 = (char*)"{F%02dLGC}\r"; //This command has two different formats and is used either to read (query) or to program the log function; Command format 2 (used to program the data recording function)
    // char* ERD = (char*)"{F%02dERD 0;2176;0003}\r"; //Read 3 Byte (one Block) from EPROM
    char *TST1 = (char *)"{F%02dTST 10;;}\r"; // This command is used to test the condition of the humidity sensor: Humidity data; Temperature data [{F00tst 21855; 67.79;  4.43;  0.00;  0.01;  0.00; 72.22;0039218836;107.93; 20.34;*]
    char *TST2 = (char *)"{F%02dTST 20;;}\r"; // This command is used to test the condition of the humidity sensor: RH sensor status; Sensor quality 0 to 100 or 255 (good to bad or na) [{F00tst 000;023;W]
    char *STOP = (char*)"{F%02dLGC 0;1;2;50746164;}\r"; // Stop recording data [{F00lgc OK1]
    // char *CHECK = (char*)"{F%02dLGC}\r";                // Check function status [{F00lgc 000;001;00002;0050746164;00000;B]

    /*
        addrCmd counts; raw humidity value [%RH] based on the count scaling; factory correction A1%; user correction A2%; sensor temperature correction; sensor drift correction; humidity end value; Counts (x1000); resistance value [Ohm] as measured based on the count scaling; numerical temperature end value
        {F00tst 21855;  67.79;                                               4.43;                   0.00;                0.01;                          0.00;                    72.22;              0039218836;     107.93;                                                        20.34;
    */
};
#endif