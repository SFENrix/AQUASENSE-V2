/*
 * AQUASENSE V2 - IoT Aquarium Smart Monitoring System
 * Features: Temperature & pH monitoring, Automatic feeding, Peltier cooling with PID
 * Board: ESP32-S3-DEVKITC-1
 * Framework: Arduino
 * Version: 2.0 - Updated with non-blocking WiFi and centralized LCD
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_ESP_PH.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include <PID_v1.h>

// ============================================
// CONFIGURATION SECTION
// ============================================

// WiFi Credentials
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// MQTT Configuration
#define MQTT_BROKER "mqtt.example.com"
#define MQTT_PORT 1883
#define MQTT_USER "your_username" // Leave empty if no auth
#define MQTT_PASSWORD "your_password"
#define MQTT_CLIENT_ID "AquasenseV2"

// MQTT Topics
#define TOPIC_TEMP "aquasense/sensor/temperature"
#define TOPIC_PH "aquasense/sensor/ph"
#define TOPIC_STATUS "aquasense/status"
#define TOPIC_CMD_FEED "aquasense/command/feed"
#define TOPIC_CMD_SETTEMP "aquasense/command/settemp"

// NTP Time Server
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 25200 // UTC+7 for Indonesia (7*3600)
#define DAYLIGHT_OFFSET_SEC 0

// Pin Definitions
#define PIN_DS18B20 4
#define PIN_SERVO 5
#define PIN_LED_INDICATOR 6
#define PIN_LCD_SDA 8
#define PIN_LCD_SCL 9
#define PIN_PH_SENSOR 1
#define PIN_TEMP_UP_BTN 38
#define PIN_TEMP_DOWN_BTN 47
#define PIN_MANUAL_FEED_BTN 2
#define PIN_PELTIER_PWM 21

// Temperature Settings
#define TEMP_TARGET_MIN 25.0
#define TEMP_TARGET_MAX 30.0
#define TEMP_TARGET_DEFAULT 27.0
#define TEMP_HYSTERESIS 0.5
#define TEMP_STEP 1.0

// Peltier PWM Settings (0-255)
#define PELTIER_IDLE_PWM 77 // 30% of 255
#define PELTIER_MAX_PWM 255 // 100%
#define PELTIER_OFF_PWM 0

// PID Tuning Parameters (adjust these based on testing)
#define PID_KP 50.0
#define PID_KI 0.5
#define PID_KD 10.0

// Servo Settings
#define SERVO_FEED_ANGLE 30
#define SERVO_FEED_DURATION 500 // milliseconds
#define SERVO_REST_ANGLE 0

// Feeding Schedule (24-hour format)
#define FEED_TIME_1_HOUR 8
#define FEED_TIME_1_MIN 0
#define FEED_TIME_2_HOUR 20
#define FEED_TIME_2_MIN 0

// Sensor Reading Intervals (milliseconds)
#define TEMP_READ_INTERVAL 2000
#define PH_READ_INTERVAL 5000
#define MQTT_PUBLISH_INTERVAL 10000
#define LCD_UPDATE_INTERVAL 1000

// LCD Configuration
#define LCD_ADDRESS 0x27 // Common I2C address, change to 0x3F if needed
#define LCD_COLS 20
#define LCD_ROWS 4

// WiFi & MQTT Retry Settings
#define WIFI_RETRY_INTERVAL 30000
#define MQTT_RETRY_INTERVAL 5000
#define NTP_SYNC_INTERVAL 3600000    // Resync every hour
#define WIFI_RECONNECT_TIMEOUT 10000 // 10 seconds max

// LCD Temporary Message Duration
#define TEMP_MESSAGE_DURATION 3000 // 3 seconds

// Button Debounce
#define DEBOUNCE_DELAY 50

// ============================================
// GLOBAL OBJECTS
// ============================================

WiFiClient espClient;
PubSubClient mqttClient(espClient);

OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensor(&oneWire);
DFRobot_ESP_PH phSensor;

Servo feedServo;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// PID Control Variables
double currentTemp = 0.0;
double targetTemp = TEMP_TARGET_DEFAULT;
double peltierOutput = 0.0;
PID tempPID(&currentTemp, &peltierOutput, &targetTemp, PID_KP, PID_KI, PID_KD, REVERSE);

// ============================================
// GLOBAL VARIABLES
// ============================================

// Sensor Data
float temperature = 0.0;
float phValue = 0.0;
float voltage = 0.0;

// System State
bool wifiConnected = false;
bool mqttConnected = false;
bool timesynced = false;
bool feedingInProgress = false;

// WiFi Reconnection State Machine
enum WiFiReconnectState
{
  WIFI_IDLE,
  WIFI_DISCONNECTING,
  WIFI_CONNECTING
};

WiFiReconnectState wifiReconnectState = WIFI_IDLE;
unsigned long wifiReconnectStartTime = 0;

// LCD Display Mode Management
enum LCDDisplayMode
{
  LCD_NORMAL,       // Normal sensor display
  LCD_TEMP_MESSAGE, // Temporary message with timer
  LCD_STARTUP       // During startup phase
};

LCDDisplayMode lcdMode = LCD_STARTUP;
String tempMessage = "";
unsigned long tempMessageStartTime = 0;

// Button States
bool tempUpPressed = false;
bool tempDownPressed = false;
bool manualFeedPressed = false;
unsigned long lastDebounceTime = 0;

// Feeding Tracking
bool fed1Today = false;
bool fed2Today = false;
int lastFeedDay = -1;

// Timing Variables
unsigned long lastTempRead = 0;
unsigned long lastPhRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long lastNtpSync = 0;

// ============================================
// FUNCTION DECLARATIONS
// ============================================

// Setup Functions
void setupWiFi();
void setupMQTT();
void setupSensors();
void setupActuators();
void setupLCD();
void syncTimeWithNTP();

// Connection Management
void reconnectWiFi();
void reconnectMQTT();
void mqttCallback(char *topic, byte *payload, unsigned int length);

// Sensor Functions
void readTemperature();
void readPH();

// Control Functions
void handlePeltierControl();

// Feeding Functions
void checkScheduledFeeding();
void performFeeding(const char *source);

// Input Handling
void checkButtons();

// LCD Display Functions
void updateLCD();
void displayNormalStatus();
void setLCDTempMessage(const char *line1, const char *line2 = "",
                       const char *line3 = "", const char *line4 = "");
void setLCDNormalMode();

// MQTT Functions
void publishSensorData();

// ============================================
// SETUP
// ============================================

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=================================");
  Serial.println("AQUASENSE V2 - Starting...");
  Serial.println("=================================\n");

  // Initialize Pin Modes
  pinMode(PIN_LED_INDICATOR, OUTPUT);
  pinMode(PIN_PELTIER_PWM, OUTPUT);
  pinMode(PIN_TEMP_UP_BTN, INPUT_PULLUP);
  pinMode(PIN_TEMP_DOWN_BTN, INPUT_PULLUP);
  pinMode(PIN_MANUAL_FEED_BTN, INPUT_PULLUP);

  digitalWrite(PIN_LED_INDICATOR, LOW);
  analogWrite(PIN_PELTIER_PWM, 0);

  // Initialize LCD
  setupLCD();
  lcdMode = LCD_STARTUP; // Prevent updateLCD() from interfering during setup

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUASENSE V2");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize Sensors
  setupSensors();

  // Initialize Servo
  setupActuators();

  // Initialize PID
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, 255);
  tempPID.SetSampleTime(1000);

  // Connect to WiFi (non-blocking with short timeout)
  setupWiFi();

  // Setup MQTT
  setupMQTT();

  // Sync time if WiFi connected
  if (wifiConnected)
  {
    syncTimeWithNTP();
  }

  // Show ready message for 2 seconds
  setLCDTempMessage("AQUASENSE V2", "System Ready!", "Starting...");
  delay(2000);

  // Switch to normal display mode
  setLCDNormalMode();

  Serial.println("\n=================================");
  Serial.println("AQUASENSE V2 - Ready!");
  Serial.println("=================================\n");
}

// ============================================
// MAIN LOOP
// ============================================

void loop()
{
  unsigned long currentMillis = millis();

  // Handle WiFi Connection
  if (!wifiConnected)
  {
    if (currentMillis - lastWifiAttempt >= WIFI_RETRY_INTERVAL)
    {
      reconnectWiFi();
      lastWifiAttempt = currentMillis;
    }
  }
  else
  {
    // Check if still connected
    reconnectWiFi();
  }

  // Handle MQTT Connection
  if (wifiConnected && !mqttConnected)
  {
    if (currentMillis - lastMqttAttempt >= MQTT_RETRY_INTERVAL)
    {
      reconnectMQTT();
      lastMqttAttempt = currentMillis;
    }
  }

  // MQTT Loop
  if (mqttConnected)
  {
    mqttClient.loop();
  }

  // Periodic NTP Sync
  if (wifiConnected && timesynced)
  {
    if (currentMillis - lastNtpSync >= NTP_SYNC_INTERVAL)
    {
      syncTimeWithNTP();
      lastNtpSync = currentMillis;
    }
  }

  // Read Temperature Sensor
  if (currentMillis - lastTempRead >= TEMP_READ_INTERVAL)
  {
    readTemperature();
    lastTempRead = currentMillis;
  }

  // Read pH Sensor
  if (currentMillis - lastPhRead >= PH_READ_INTERVAL)
  {
    readPH();
    lastPhRead = currentMillis;
  }

  // Control Peltier based on PID
  handlePeltierControl();

  // Check for scheduled feeding
  if (timesynced && !feedingInProgress)
  {
    checkScheduledFeeding();
  }

  // Check physical buttons
  checkButtons();

  // Update LCD Display
  if (currentMillis - lastLcdUpdate >= LCD_UPDATE_INTERVAL)
  {
    updateLCD();
    lastLcdUpdate = currentMillis;
  }

  // Publish sensor data to MQTT
  if (mqttConnected && (currentMillis - lastMqttPublish >= MQTT_PUBLISH_INTERVAL))
  {
    publishSensorData();
    lastMqttPublish = currentMillis;
  }
}

// ============================================
// SETUP FUNCTIONS
// ============================================

void setupLCD()
{
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();
  Serial.println("[LCD] Initialized");
}

void setupSensors()
{
  // DS18B20 Temperature Sensor
  tempSensor.begin();
  tempSensor.setResolution(12); // Set to 12-bit resolution (0.0625°C precision)

  Serial.print("[DS18B20] Found ");
  Serial.print(tempSensor.getDeviceCount());
  Serial.println(" device(s)");
  Serial.println("[DS18B20] Resolution set to 12-bit (0.0625°C)");

  // pH Sensor
  phSensor.begin();
  Serial.println("[PH Sensor] Initialized");
}

void setupActuators()
{
  // Servo
  feedServo.attach(PIN_SERVO);
  feedServo.write(SERVO_REST_ANGLE);
  Serial.println("[Servo] Initialized at rest position");
}

void setupWiFi()
{
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Try for 2 seconds only (non-blocking approach)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 4)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiConnected = true;
    digitalWrite(PIN_LED_INDICATOR, HIGH);
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    wifiConnected = false;
    digitalWrite(PIN_LED_INDICATOR, LOW);
    Serial.println("\n[WiFi] Not connected yet, will retry in background");
    // loop() will keep trying via reconnectWiFi()
  }
}

void setupMQTT()
{
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  if (wifiConnected)
  {
    reconnectMQTT();
  }
}

void syncTimeWithNTP()
{
  Serial.println("[NTP] Syncing time...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  struct tm timeinfo;
  int attempts = 0;
  while (!getLocalTime(&timeinfo) && attempts < 10)
  {
    delay(500);
    attempts++;
  }

  if (attempts < 10)
  {
    timesynced = true;
    Serial.println("[NTP] Time synced successfully!");
    Serial.print("[NTP] Current time: ");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

    // Reset daily feeding flags if day changed
    if (lastFeedDay != timeinfo.tm_mday)
    {
      fed1Today = false;
      fed2Today = false;
      lastFeedDay = timeinfo.tm_mday;
      Serial.println("[Feeding] Daily schedule reset");
    }
  }
  else
  {
    timesynced = false;
    Serial.println("[NTP] Time sync failed!");
  }
}

// ============================================
// RECONNECTION FUNCTIONS
// ============================================

void reconnectWiFi()
{
  unsigned long currentMillis = millis();

  // Check if already connected
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!wifiConnected)
    {
      // Just became connected
      wifiConnected = true;
      digitalWrite(PIN_LED_INDICATOR, HIGH);
      Serial.println("[WiFi] Connection detected!");
    }
    wifiReconnectState = WIFI_IDLE;
    return;
  }

  // Not connected anymore
  if (wifiConnected)
  {
    wifiConnected = false;
    digitalWrite(PIN_LED_INDICATOR, LOW);
    Serial.println("[WiFi] Connection lost!");
  }

  // State machine for non-blocking reconnection
  switch (wifiReconnectState)
  {
  case WIFI_IDLE:
    // Start reconnection process
    Serial.println("[WiFi] Attempting to reconnect...");
    WiFi.disconnect();
    wifiReconnectState = WIFI_DISCONNECTING;
    wifiReconnectStartTime = currentMillis;
    break;

  case WIFI_DISCONNECTING:
    // Wait a bit for clean disconnect
    if (currentMillis - wifiReconnectStartTime >= 500)
    {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      wifiReconnectState = WIFI_CONNECTING;
      wifiReconnectStartTime = currentMillis;
      Serial.println("[WiFi] Connection initiated...");
    }
    break;

  case WIFI_CONNECTING:
    // Check connection status
    if (WiFi.status() == WL_CONNECTED)
    {
      wifiConnected = true;
      wifiReconnectState = WIFI_IDLE;
      digitalWrite(PIN_LED_INDICATOR, HIGH);
      Serial.println("[WiFi] Reconnected!");
      Serial.print("[WiFi] IP Address: ");
      Serial.println(WiFi.localIP());

      // Show brief reconnection message
      setLCDTempMessage("AQUASENSE V2", "WiFi Reconnected!", WiFi.localIP().toString().c_str());

      syncTimeWithNTP();
    }
    // Timeout if taking too long
    else if (currentMillis - wifiReconnectStartTime >= WIFI_RECONNECT_TIMEOUT)
    {
      wifiConnected = false;
      wifiReconnectState = WIFI_IDLE;
      digitalWrite(PIN_LED_INDICATOR, LOW);
      Serial.println("[WiFi] Reconnection timeout!");
    }
    break;
  }
}

void reconnectMQTT()
{
  if (!wifiConnected)
    return;

  Serial.print("[MQTT] Attempting connection...");

  bool connected = false;
  if (strlen(MQTT_USER) > 0)
  {
    connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
  }
  else
  {
    connected = mqttClient.connect(MQTT_CLIENT_ID);
  }

  if (connected)
  {
    mqttConnected = true;
    Serial.println(" Connected!");

    // Subscribe to command topics
    mqttClient.subscribe(TOPIC_CMD_FEED);
    mqttClient.subscribe(TOPIC_CMD_SETTEMP);

    Serial.println("[MQTT] Subscribed to command topics");

    // Publish online status
    mqttClient.publish(TOPIC_STATUS, "{\"status\":\"online\"}");

    // Show brief connection message
    setLCDTempMessage("AQUASENSE V2", "MQTT Connected!");
  }
  else
  {
    mqttConnected = false;
    Serial.print(" Failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ============================================
// MQTT CALLBACK
// ============================================

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("[MQTT] Message received on topic: ");
  Serial.println(topic);

  // Convert payload to string
  String message = "";
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.print("[MQTT] Payload: ");
  Serial.println(message);

  // Handle feed command
  if (strcmp(topic, TOPIC_CMD_FEED) == 0)
  {
    if (message == "1" || message == "true" || message.equalsIgnoreCase("feed"))
    {
      performFeeding("MQTT Command");
    }
  }

  // Handle set temperature command
  if (strcmp(topic, TOPIC_CMD_SETTEMP) == 0)
  {
    float newTemp = message.toFloat();
    if (newTemp >= TEMP_TARGET_MIN && newTemp <= TEMP_TARGET_MAX)
    {
      targetTemp = newTemp;
      Serial.print("[Temp] Target updated to: ");
      Serial.print(targetTemp);
      Serial.println("°C via MQTT");

      // Show confirmation on LCD
      char msg[20];
      sprintf(msg, "New Target: %.1fC", targetTemp);
      setLCDTempMessage("AQUASENSE V2", msg, "Via MQTT");
    }
    else
    {
      Serial.println("[Temp] Invalid temperature range!");
    }
  }
}

// ============================================
// SENSOR READING FUNCTIONS
// ============================================

void readTemperature()
{
  tempSensor.requestTemperatures();
  temperature = tempSensor.getTempCByIndex(0);
  currentTemp = temperature; // Update PID input

  if (temperature < -50 || temperature > 100)
  {
    Serial.println("[DS18B20] Error reading temperature!");
    temperature = 0.0;
  }
  else
  {
    Serial.print("[DS18B20] Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
  }
}

void readPH()
{
  voltage = analogRead(PIN_PH_SENSOR) / 4095.0 * 3300; // ESP32 ADC: 12-bit, 3.3V
  phValue = phSensor.readPH(voltage, temperature);

  Serial.print("[PH Sensor] pH: ");
  Serial.print(phValue, 2);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.println("mV");
}

// ============================================
// PELTIER CONTROL
// ============================================

void handlePeltierControl()
{
  // Calculate PID output
  tempPID.Compute();

  // Determine Peltier state based on temperature difference
  float tempDiff = targetTemp - currentTemp;

  if (abs(tempDiff) <= TEMP_HYSTERESIS)
  {
    // Temperature at target - maintain with idle power
    analogWrite(PIN_PELTIER_PWM, PELTIER_IDLE_PWM);
  }
  else if (tempDiff < 0)
  {
    // Current temp > target - need cooling
    analogWrite(PIN_PELTIER_PWM, (int)peltierOutput);
  }
  else
  {
    // Current temp < target - idle/low power (no heating)
    analogWrite(PIN_PELTIER_PWM, PELTIER_IDLE_PWM);
  }
}

// ============================================
// FEEDING FUNCTIONS
// ============================================

void checkScheduledFeeding()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return;
  }

  int currentHour = timeinfo.tm_hour;
  int currentMin = timeinfo.tm_min;

  // Check first feeding time (8:00 AM)
  if (!fed1Today && currentHour == FEED_TIME_1_HOUR && currentMin == FEED_TIME_1_MIN)
  {
    performFeeding("Schedule 08:00");
    fed1Today = true;
  }

  // Check second feeding time (8:00 PM)
  if (!fed2Today && currentHour == FEED_TIME_2_HOUR && currentMin == FEED_TIME_2_MIN)
  {
    performFeeding("Schedule 20:00");
    fed2Today = true;
  }

  // Reset flags at midnight
  if (currentHour == 0 && currentMin == 0)
  {
    fed1Today = false;
    fed2Today = false;
    Serial.println("[Feeding] Daily schedule reset at midnight");
  }
}

void performFeeding(const char *source)
{
  if (feedingInProgress)
  {
    Serial.println("[Feeding] Already in progress, skipping...");
    return;
  }

  feedingInProgress = true;

  Serial.print("[Feeding] Feeding triggered by: ");
  Serial.println(source);

  // Show feeding message on LCD
  setLCDTempMessage("AQUASENSE V2", "FEEDING NOW!", source);

  // Move servo to feed position
  feedServo.write(SERVO_FEED_ANGLE);
  delay(SERVO_FEED_DURATION);

  // Return servo to rest position
  feedServo.write(SERVO_REST_ANGLE);

  Serial.println("[Feeding] Feeding complete");

  // Publish feeding event to MQTT
  if (mqttConnected)
  {
    String feedMsg = "{\"event\":\"fed\",\"source\":\"";
    feedMsg += source;
    feedMsg += "\"}";
    mqttClient.publish("aquasense/event/feeding", feedMsg.c_str());
  }

  feedingInProgress = false;
  // LCD will auto-return to normal display after 3 seconds
}

// ============================================
// BUTTON HANDLING
// ============================================

void checkButtons()
{
  unsigned long currentMillis = millis();

  // Debouncing
  if ((currentMillis - lastDebounceTime) < DEBOUNCE_DELAY)
  {
    return;
  }

  // Temperature UP button
  if (digitalRead(PIN_TEMP_UP_BTN) == LOW && !tempUpPressed)
  {
    tempUpPressed = true;
    lastDebounceTime = currentMillis;

    if (targetTemp < TEMP_TARGET_MAX)
    {
      targetTemp += TEMP_STEP;
      Serial.print("[Button] Target temp increased to: ");
      Serial.print(targetTemp);
      Serial.println("°C");

      // Show confirmation on LCD
      char msg[20];
      sprintf(msg, "Target: %.1fC", targetTemp);
      setLCDTempMessage("AQUASENSE V2", "Temp Increased", msg);
    }
    else
    {
      setLCDTempMessage("AQUASENSE V2", "Max Temp Reached", "30.0C");
    }
  }
  else if (digitalRead(PIN_TEMP_UP_BTN) == HIGH)
  {
    tempUpPressed = false;
  }

  // Temperature DOWN button
  if (digitalRead(PIN_TEMP_DOWN_BTN) == LOW && !tempDownPressed)
  {
    tempDownPressed = true;
    lastDebounceTime = currentMillis;

    if (targetTemp > TEMP_TARGET_MIN)
    {
      targetTemp -= TEMP_STEP;
      Serial.print("[Button] Target temp decreased to: ");
      Serial.print(targetTemp);
      Serial.println("°C");

      // Show confirmation on LCD
      char msg[20];
      sprintf(msg, "Target: %.1fC", targetTemp);
      setLCDTempMessage("AQUASENSE V2", "Temp Decreased", msg);
    }
    else
    {
      setLCDTempMessage("AQUASENSE V2", "Min Temp Reached", "25.0C");
    }
  }
  else if (digitalRead(PIN_TEMP_DOWN_BTN) == HIGH)
  {
    tempDownPressed = false;
  }

  // Manual Feed button
  if (digitalRead(PIN_MANUAL_FEED_BTN) == LOW && !manualFeedPressed)
  {
    manualFeedPressed = true;
    lastDebounceTime = currentMillis;
    performFeeding("Manual Button");
  }
  else if (digitalRead(PIN_MANUAL_FEED_BTN) == HIGH)
  {
    manualFeedPressed = false;
  }
}

// ============================================
// LCD DISPLAY FUNCTIONS
// ============================================

void updateLCD()
{
  unsigned long currentMillis = millis();

  // Check if temporary message expired
  if (lcdMode == LCD_TEMP_MESSAGE)
  {
    if (currentMillis - tempMessageStartTime >= TEMP_MESSAGE_DURATION)
    {
      lcdMode = LCD_NORMAL; // Return to normal display
      lcd.clear();          // Clear old message
    }
  }

  switch (lcdMode)
  {
  case LCD_STARTUP:
    // Don't update during startup - manual control
    break;

  case LCD_TEMP_MESSAGE:
    // Show temporary message (already displayed by setLCDTempMessage)
    // Just wait for timeout
    break;

  case LCD_NORMAL:
    // Normal operation display
    displayNormalStatus();
    break;
  }
}

void displayNormalStatus()
{
  // Row 0: Title and connection status
  lcd.setCursor(0, 0);
  lcd.print("AQUASENSE V2 ");
  lcd.print(wifiConnected ? "W" : " ");
  lcd.print(mqttConnected ? "M" : " ");
  lcd.print(timesynced ? "T" : " ");

  // Row 1: Temperature
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.print(temperature, 1);
  lcd.print("C T:");
  lcd.print(targetTemp, 1);
  lcd.print("C ");

  // Row 2: pH Value
  lcd.setCursor(0, 2);
  lcd.print("pH: ");
  lcd.print(phValue, 2);
  lcd.print("              ");

  // Row 3: Time or status
  lcd.setCursor(0, 3);
  if (timesynced)
  {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
      char timeStr[20];
      strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
      lcd.print(timeStr);
      lcd.print("          ");
    }
  }
  else
  {
    lcd.print("Time: Not synced    ");
  }
}

void setLCDTempMessage(const char *line1, const char *line2,
                       const char *line3, const char *line4)
{
  lcdMode = LCD_TEMP_MESSAGE;
  tempMessageStartTime = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);

  if (strlen(line2) > 0)
  {
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }

  if (strlen(line3) > 0)
  {
    lcd.setCursor(0, 2);
    lcd.print(line3);
  }

  if (strlen(line4) > 0)
  {
    lcd.setCursor(0, 3);
    lcd.print(line4);
  }
}

void setLCDNormalMode()
{
  lcdMode = LCD_NORMAL;
  lcd.clear();
}

// ============================================
// MQTT PUBLISH
// ============================================

void publishSensorData()
{
  // Publish temperature
  String tempMsg = String(temperature, 2);
  mqttClient.publish(TOPIC_TEMP, tempMsg.c_str());

  // Publish pH
  String phMsg = String(phValue, 2);
  mqttClient.publish(TOPIC_PH, phMsg.c_str());

  // Publish combined status (optional)
  String statusMsg = "{";
  statusMsg += "\"temperature\":" + String(temperature, 2) + ",";
  statusMsg += "\"target_temp\":" + String(targetTemp, 2) + ",";
  statusMsg += "\"ph\":" + String(phValue, 2) + ",";
  statusMsg += "\"peltier_pwm\":" + String((int)peltierOutput);
  statusMsg += "}";
  mqttClient.publish(TOPIC_STATUS, statusMsg.c_str());

  Serial.println("[MQTT] Sensor data published");
}
