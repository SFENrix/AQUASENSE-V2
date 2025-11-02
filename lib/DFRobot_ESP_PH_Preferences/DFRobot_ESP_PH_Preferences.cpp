/*
 * file DFRobot_ESP_PH_Preferences.cpp
 * Modified version of DFRobot_ESP_PH_BY_GREENPONIK
 * 
 * MODIFIED FOR AQUASENSE PROJECT - Uses Preferences.h instead of EEPROM
 * Modified by: [Your Name]
 * Date: 2025
 * 
 * Original library:
 * @ https://github.com/GreenPonik/DFRobot_ESP_PH_BY_GREENPONIK
 * using Gravity: Analog pH Sensor / Meter Kit V2, SKU:SEN0161-V2
 * Based on DFRobot_PH @ https://github.com/DFRobot/DFRobot_PH
 */

#include "DFRobot_ESP_PH_Preferences.h"

// Constructor
DFRobot_ESP_PH::DFRobot_ESP_PH()
{
    this->_temperature = 25.0;
    this->_phValue = 7.0;
    this->_acidVoltage = 2032.44;    // buffer solution 4.0 at 25C
    this->_neutralVoltage = 1500.0;  // buffer solution 7.0 at 25C
    this->_voltage = 1500.0;
}

// Destructor
DFRobot_ESP_PH::~DFRobot_ESP_PH()
{
    _preferences.end();
}

// Initialize and load calibration from Preferences
void DFRobot_ESP_PH::begin()
{
    // Open Preferences with namespace "ph-sensor" in read-write mode
    _preferences.begin("ph-sensor", false);
    
    // Read calibration values from Preferences
    readCalibrationFromPreferences();
    
    Serial.println(F("DFRobot_ESP_PH Initialized (Preferences.h)"));
    Serial.print(F("Neutral Voltage: "));
    Serial.println(this->_neutralVoltage, 2);
    Serial.print(F("Acid Voltage: "));
    Serial.println(this->_acidVoltage, 2);
}

// Read calibration values from Preferences (NVS)
void DFRobot_ESP_PH::readCalibrationFromPreferences()
{
    // Read with default values if not found
    this->_neutralVoltage = _preferences.getFloat("neutralV", 1500.0);
    this->_acidVoltage = _preferences.getFloat("acidV", 2032.44);
    
    Serial.println(F("Calibration loaded from Preferences"));
}

// Write calibration values to Preferences (NVS)
void DFRobot_ESP_PH::writeCalibrationToPreferences()
{
    _preferences.putFloat("neutralV", this->_neutralVoltage);
    _preferences.putFloat("acidV", this->_acidVoltage);
    
    Serial.println(F("Calibration saved to Preferences"));
    Serial.print(F("Neutral Voltage: "));
    Serial.println(this->_neutralVoltage, 2);
    Serial.print(F("Acid Voltage: "));
    Serial.println(this->_acidVoltage, 2);
}

// Convert voltage to pH with temperature compensation
float DFRobot_ESP_PH::readPH(float voltage, float temperature)
{
    float slope = (7.0 - 4.0) / ((this->_neutralVoltage - 1500.0) / 3.0 - (this->_acidVoltage - 1500.0) / 3.0);
    float intercept = 7.0 - slope * (this->_neutralVoltage - 1500.0) / 3.0;
    this->_phValue = slope * (voltage - 1500.0) / 3.0 + intercept;
    
    return this->_phValue;
}

// Calibration with automatic Serial command reading
void DFRobot_ESP_PH::calibration(float voltage, float temperature)
{
    this->_voltage = voltage;
    this->_temperature = temperature;
    
    if (cmdSerialDataAvailable() > 0)
    {
        phCalibration(cmdParse());  // Enter calibration mode if Serial CMD received
    }
}

// Calibration with manual command
void DFRobot_ESP_PH::calibration(float voltage, float temperature, char* cmd)
{
    this->_voltage = voltage;
    this->_temperature = temperature;
    phCalibration(cmdParse(cmd));  // Enter calibration mode with provided command
}

// Check if Serial data is available
boolean DFRobot_ESP_PH::cmdSerialDataAvailable(void)
{
    char cmdReceivedChar;
    static unsigned long cmdReceivedTimeOut = millis();
    
    while (Serial.available() > 0)
    {
        if (millis() - cmdReceivedTimeOut > 500U)
        {
            this->_cmdReceivedBufferIndex = 0;
            memset(this->_cmdReceivedBuffer, 0, (ReceivedBufferLength));
        }
        cmdReceivedTimeOut = millis();
        cmdReceivedChar = Serial.read();
        
        if (cmdReceivedChar == '\n' || this->_cmdReceivedBufferIndex == ReceivedBufferLength - 1)
        {
            this->_cmdReceivedBufferIndex = 0;
            return true;
        }
        else
        {
            this->_cmdReceivedBuffer[this->_cmdReceivedBufferIndex] = cmdReceivedChar;
            this->_cmdReceivedBufferIndex++;
        }
    }
    return false;
}

// Parse command from buffer
byte DFRobot_ESP_PH::cmdParse(const char* cmd)
{
    byte modeIndex = 0;
    
    if (strstr(cmd, "ENTERPH") != NULL || strstr(cmd, "enterph") != NULL)
    {
        modeIndex = 1;
    }
    else if (strstr(cmd, "EXITPH") != NULL || strstr(cmd, "exitph") != NULL)
    {
        modeIndex = 3;
    }
    else if (strstr(cmd, "CALPH") != NULL || strstr(cmd, "calph") != NULL)
    {
        modeIndex = 2;
    }
    return modeIndex;
}

// Parse command from Serial buffer
byte DFRobot_ESP_PH::cmdParse(void)
{
    byte modeIndex = 0;
    
    if (strstr(this->_cmdReceivedBuffer, "ENTERPH") != NULL || 
        strstr(this->_cmdReceivedBuffer, "enterph") != NULL)
    {
        modeIndex = 1;
    }
    else if (strstr(this->_cmdReceivedBuffer, "EXITPH") != NULL || 
             strstr(this->_cmdReceivedBuffer, "exitph") != NULL)
    {
        modeIndex = 3;
    }
    else if (strstr(this->_cmdReceivedBuffer, "CALPH") != NULL || 
             strstr(this->_cmdReceivedBuffer, "calph") != NULL)
    {
        modeIndex = 2;
    }
    return modeIndex;
}

// Calibration process
void DFRobot_ESP_PH::phCalibration(byte mode)
{
    char* receivedBufferPtr;
    static boolean phCalibrationFinish = false;
    static boolean enterCalibrationFlag = false;
    
    switch (mode)
    {
        case 0:  // Normal mode
            if (enterCalibrationFlag)
            {
                Serial.println(F(">>>Command Error<<<"));
            }
            break;

        case 1:  // Enter calibration mode
            enterCalibrationFlag = true;
            phCalibrationFinish = false;
            Serial.println();
            Serial.println(F(">>>Enter PH Calibration Mode<<<"));
            Serial.println(F(">>>Please put the probe into the 4.0 or 7.0 standard buffer solution<<<"));
            Serial.println();
            break;

        case 2:  // Calibration
            if (enterCalibrationFlag)
            {
                if ((this->_voltage > 1322) && (this->_voltage < 1678))  // 7.0 buffer
                {
                    Serial.println();
                    Serial.print(F(">>>Buffer Solution: 7.0"));
                    this->_neutralVoltage = this->_voltage;
                    Serial.println(F("  PASS<<<"));
                    Serial.println();
                    phCalibrationFinish = true;
                }
                else if ((this->_voltage > 1854) && (this->_voltage < 2210))  // 4.0 buffer
                {
                    Serial.println();
                    Serial.print(F(">>>Buffer Solution: 4.0"));
                    this->_acidVoltage = this->_voltage;
                    Serial.println(F("  PASS<<<"));
                    Serial.println();
                    phCalibrationFinish = true;
                }
                else
                {
                    Serial.println();
                    Serial.print(F(">>>Buffer Solution Error. Try Again<<<"));
                    Serial.println();
                    phCalibrationFinish = false;
                }
            }
            break;

        case 3:  // Exit calibration mode
            if (enterCalibrationFlag)
            {
                Serial.println();
                if (phCalibrationFinish)
                {
                    // Save to Preferences instead of EEPROM
                    writeCalibrationToPreferences();
                    Serial.print(F(">>>Calibration Successful. Saved to Preferences<<<"));
                }
                else
                {
                    Serial.print(F(">>>Calibration Failed. Not Saved<<<"));
                }
                Serial.println();
                phCalibrationFinish = false;
                enterCalibrationFlag = false;
            }
            break;
    }
}
