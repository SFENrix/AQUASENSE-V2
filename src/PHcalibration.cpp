/*
 * AQUASENSE V2 - pH Sensor Calibration Tool
 *
 * This is a dedicated calibration program for the DFRobot pH sensor.
 *
 * INSTRUCTIONS:
 * 1. Upload this program to ESP32
 * 2. Open Serial Monitor (115200 baud)
 * 3. Follow the on-screen prompts
 */

#include <Arduino.h>
#include "DFRobot_ESP_PH_Preferences.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// ===== PIN DEFINITIONS =====
#define PIN_PH_SENSOR 4   // GPIO4 for pH sensor
#define PIN_TEMP_SENSOR 5 // GPIO5 for DS18B20

// ===== CONSTANTS =====
const char *LINE_SEP = "============================================";

// ===== SENSOR OBJECTS =====
DFRobot_ESP_PH phSensor;
OneWire oneWire(PIN_TEMP_SENSOR);
DallasTemperature tempSensor(&oneWire);

// ===== VARIABLES =====
float voltage = 0.0;
float phValue = 7.0;
float temperature = 25.0;

// ===== FUNCTION PROTOTYPES =====
void printWelcomeMessage();
void printCalibrationInstructions();
void readTemperature();
void readAndDisplayPH();

void setup()
{
    Serial.begin(115200);
    delay(1000);

    printWelcomeMessage();

    // Configure ADC for ESP32-S3
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.println("[ADC] Configured: 12-bit, 0-3300mV");

    // Initialize pH sensor (uses Preferences.h)
    phSensor.begin();
    Serial.println("[pH] Sensor initialized");

    // Initialize temperature sensor
    tempSensor.begin();
    int devices = tempSensor.getDeviceCount();
    Serial.print("[Temp] Found ");
    Serial.print(devices);
    Serial.println(" DS18B20 sensor(s)");

    // Print instructions
    Serial.println();
    Serial.println(LINE_SEP);
    printCalibrationInstructions();
    Serial.println(LINE_SEP);
    Serial.println();
}

void loop()
{
    static unsigned long lastRead = 0;

    if (millis() - lastRead >= 1000) // Update every second
    {
        lastRead = millis();

        // Read temperature
        readTemperature();

        // Read and display pH
        readAndDisplayPH();

        Serial.println(); // Blank line for readability
    }

    // Handle calibration commands
    phSensor.calibration(voltage, temperature);
}

void printWelcomeMessage()
{
    Serial.println("\n\n");
    Serial.println("+==========================================+");
    Serial.println("|   AQUASENSE V2 - pH CALIBRATION TOOL    |");
    Serial.println("|                                          |");
    Serial.println("|   DFRobot SEN0161-V2 pH Sensor          |");
    Serial.println("|   with Preferences.h Storage            |");
    Serial.println("+==========================================+");
    Serial.println();
}

void printCalibrationInstructions()
{
    Serial.println("CALIBRATION COMMANDS:");
    Serial.println("+------------------------------------------+");
    Serial.println("| enterph -> Enter calibration mode        |");
    Serial.println("| calph   -> Calibrate current buffer      |");
    Serial.println("| exitph  -> Save & exit calibration       |");
    Serial.println("+------------------------------------------+");
    Serial.println();
    Serial.println("CALIBRATION PROCEDURE:");
    Serial.println("1. Type 'enterph' and press Enter");
    Serial.println("2. Rinse probe with distilled water");
    Serial.println("3. Place probe in pH 7.0 buffer solution");
    Serial.println("4. Wait for reading to stabilize (30-60 sec)");
    Serial.println("5. Type 'calph' and press Enter");
    Serial.println("6. Rinse probe again");
    Serial.println("7. Place probe in pH 4.0 buffer solution");
    Serial.println("8. Wait for reading to stabilize");
    Serial.println("9. Type 'calph' and press Enter");
    Serial.println("10. Type 'exitph' to save calibration");
    Serial.println();
    Serial.println("BUFFER SOLUTIONS NEEDED:");
    Serial.println("  * pH 7.0 (neutral) - REQUIRED");
    Serial.println("  * pH 4.0 (acidic)  - REQUIRED");
    Serial.println();
    Serial.println("NOTES:");
    Serial.println("  * Temperature compensation is automatic");
    Serial.println("  * Calibration saved to NVS storage");
    Serial.println("  * Data persists after power cycle");
    Serial.println();
}

void readTemperature()
{
    tempSensor.requestTemperatures();
    temperature = tempSensor.getTempCByIndex(0);

    // Validate temperature reading
    if (temperature == DEVICE_DISCONNECTED_C || temperature < -50 || temperature > 100)
    {
        Serial.println("[ERROR] Invalid temperature! Using default 25C");
        temperature = 25.0;
    }
}

void readAndDisplayPH()
{
    // Read voltage from pH sensor
    voltage = analogReadMilliVolts(PIN_PH_SENSOR);

    // Calculate pH with temperature compensation
    phValue = phSensor.readPH(voltage, temperature);

    // Display with formatting
    Serial.println("+--------------------------------------+");
    Serial.print("| Voltage:     ");
    Serial.print(voltage, 2);
    Serial.println(" mV");

    Serial.print("| Temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" C");

    Serial.print("| pH Value:    ");
    Serial.print(phValue, 2);

    // Add indicator for pH range
    if (phValue < 7.0)
    {
        Serial.println("  (ACIDIC)");
    }
    else if (phValue > 7.0)
    {
        Serial.println("  (BASIC)");
    }
    else
    {
        Serial.println("  (NEUTRAL)");
    }

    Serial.println("+--------------------------------------+");

    // Show stability indicator
    static float lastPH = 7.0;
    float change = abs(phValue - lastPH);

    if (change < 0.02)
    {
        Serial.println(">>> STABLE - Ready for calibration <<<");
    }
    else if (change < 0.05)
    {
        Serial.println(">>> Stabilizing... <<<");
    }
    else
    {
        Serial.println(">>> Reading fluctuating... wait <<<");
    }

    lastPH = phValue;
}