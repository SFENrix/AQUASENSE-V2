/*
 * file DFRobot_ESP_PH_Preferences.h
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
 * Copyright [DFRobot](http://www.dfrobot.com), 2018
 * Copyright GNU Lesser General Public License
 *
 * ##################################################
 * ############# ESP32 PREFERENCES VERSION ##########
 * ##################################################
 *
 * version V2.0 - Preferences.h implementation
 * date 2025-01
 */

#ifndef _DFROBOT_ESP_PH_PREFERENCES_H_
#define _DFROBOT_ESP_PH_PREFERENCES_H_

#include "Arduino.h"
#include <Preferences.h>

#define ReceivedBufferLength 10 // length of the Serial CMD buffer

class DFRobot_ESP_PH
{
public:
    DFRobot_ESP_PH();
    ~DFRobot_ESP_PH();

    /**
     * @brief Initialization
     */
    void begin();

    /**
     * @brief Convert voltage to pH with temperature compensation
     * @param voltage : Analog voltage value (mV)
     * @param temperature : Ambient temperature (°C)
     * @return float : pH value
     */
    float readPH(float voltage, float temperature);

    /**
     * @brief Calibration process, key function
     * @param voltage : Analog voltage value (mV)
     * @param temperature : Ambient temperature (°C)
     */
    void calibration(float voltage, float temperature);

    /**
     * @brief Calibration process with manual command
     * @param voltage : Analog voltage value (mV)
     * @param temperature : Ambient temperature (°C)
     * @param cmd : Calibration command
     */
    void calibration(float voltage, float temperature, char *cmd);

private:
    Preferences _preferences;

    float _phValue;
    float _acidVoltage;
    float _neutralVoltage;
    float _voltage;
    float _temperature;

    char _cmdReceivedBuffer[ReceivedBufferLength]; // buffer for receiving Serial CMD
    byte _cmdReceivedBufferIndex;

private:
    boolean cmdSerialDataAvailable(void);
    void phCalibration(byte mode); // calibration process
    byte cmdParse(const char *cmd);
    byte cmdParse(void);

    /**
     * @brief Read calibration values from Preferences
     */
    void readCalibrationFromPreferences();

    /**
     * @brief Write calibration values to Preferences
     */
    void writeCalibrationToPreferences();
};

#endif
