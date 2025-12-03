#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_ESP_PH.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include <PID_v1.h>

// ============================================
// CONFIGURATION SECTION
// ============================================

// WiFi Credentials
#define WIFI_SSID "CIEEE"
#define WIFI_PASSWORD "FENRIRGIMANK"

// MQTT Configuration
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""
#define MQTT_CLIENT_ID "AquasenseV2"

// ============================================
// MQTT TOPICS - BIDIRECTIONAL SYSTEM
// ============================================

// Sensor Topics (One-way: ESP → App)
#define TOPIC_TEMP "aquasense/sensor/temperature"
#define TOPIC_PH "aquasense/sensor/ph"
#define TOPIC_TEMP_TARGET "aquasense/sensor/target_temp"
#define TOPIC_STATUS "aquasense/status"

// Feed Topics (Bidirectional)
#define TOPIC_CMD_FEED_REQ "aquasense/command/feed/request"
#define TOPIC_CMD_FEED_RES "aquasense/command/feed/response"

// Peltier Topics (Bidirectional)
#define TOPIC_CMD_PELTIER_REQ "aquasense/command/peltier/request"
#define TOPIC_CMD_PELTIER_RES "aquasense/command/peltier/response"

// Temperature Adjustment Topics (Bidirectional)
#define TOPIC_CMD_TEMP_ADJUST_REQ "aquasense/command/temp_adjust/request"
#define TOPIC_CMD_TEMP_ADJUST_RES "aquasense/command/temp_adjust/response"

// Temperature Set Topics (Bidirectional)
#define TOPIC_CMD_SETTEMP_REQ "aquasense/command/settemp/request"
#define TOPIC_CMD_SETTEMP_RES "aquasense/command/settemp/response"

// ============================================
// NTP Time Server
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC 25200
#define DAYLIGHT_OFFSET_SEC 0

// ============================================
// PIN DEFINITIONS - CORRECTED FROM PIN_ASSIGNMENT
// ============================================
#define PIN_PH_SENSOR 4        // pH SEN0169 Sensor - GPIO 4
#define PIN_DS18B20 5          // Temperature Sensor (DS18B20) - GPIO 5
#define PIN_SERVO 6            // Servo Motor - GPIO 6
#define PIN_LCD_SCL 7          // LCD I2C SCL - GPIO 7
#define PIN_LCD_SDA 8          // LCD I2C SDA - GPIO 8
#define PIN_LED_INDICATOR 10   // LED WiFi Indicator - GPIO 10
#define PIN_TEMP_UP_BTN 11     // Peltier Temperature UP Button - GPIO 11
#define PIN_TEMP_DOWN_BTN 12   // Peltier Temperature DOWN Button - GPIO 12
#define PIN_MANUAL_FEED_BTN 13 // Servo Feeder Manual Button - GPIO 13
#define PIN_PELTIER_PWM 21     // Peltier MOSFET Control - GPIO 21

// EEPROM Configuration
#define EEPROM_SIZE 32

// ESP32-S3 ADC Configuration
#define ESP32_ADC_RESOLUTION 4096.0
#define ESP32_VOLTAGE_REF 3300

// Temperature Settings
#define TEMP_TARGET_MIN 25.0
#define TEMP_TARGET_MAX 30.0
#define TEMP_TARGET_DEFAULT 27.0
#define TEMP_HYSTERESIS 0.5
#define TEMP_STEP 1.0
#define TEMP_DECIMAL_PLACES 1

// Peltier PWM Settings
#define PELTIER_IDLE_PWM 77
#define PELTIER_MAX_PWM 255
#define PELTIER_OFF_PWM 0

// Peltier Protection Settings
#define PELTIER_INIT_DELAY 3000     // 3 seconds initialization delay
#define PELTIER_MIN_CYCLE_TIME 5000 // Minimum 5 seconds between state changes

// PID Tuning Parameters
#define PID_KP 50.0
#define PID_KI 0.5
#define PID_KD 10.0

// Servo Settings
#define SERVO_FEED_ANGLE 30
#define SERVO_FEED_DURATION 500
#define SERVO_REST_ANGLE 0

// Feeding Schedule
#define FEED_TIME_1_HOUR 8
#define FEED_TIME_1_MIN 0
#define FEED_TIME_2_HOUR 20
#define FEED_TIME_2_MIN 0

// Sensor Reading Intervals
#define TEMP_READ_INTERVAL 2000
#define PH_READ_INTERVAL 5000
#define MQTT_PUBLISH_INTERVAL 10000
#define LCD_UPDATE_INTERVAL 1000

// LCD Configuration
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// LCD Blink Settings for Feeding
#define FEED_BLINK_COUNT 2
#define FEED_BLINK_INTERVAL 500

// WiFi & MQTT Retry Settings
#define WIFI_RETRY_INTERVAL 30000
#define MQTT_RETRY_INTERVAL 5000
#define NTP_SYNC_INTERVAL 3600000
#define WIFI_RECONNECT_TIMEOUT 10000

// LCD Temporary Message Duration
#define TEMP_MESSAGE_DURATION 3000

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
bool peltierOn = false;

// Peltier Smart Startup
bool peltierInitialized = false;
unsigned long peltierStartTime = 0;
unsigned long lastPeltierChange = 0;

// Feeding Counter
int feedingCount = 0;
int feedingCountToday = 0;
bool feedingBlinkActive = false;
int feedingBlinkRemaining = 0;
unsigned long lastFeedingBlinkTime = 0;
bool feedingBlinkState = true;

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
  LCD_NORMAL,
  LCD_TEMP_MESSAGE,
  LCD_STARTUP
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

// LCD Buffer for previous display state
char lcdBuffer[4][21] = {"", "", "", ""};

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
void smartPeltierStartup();

// Connection Management
void reconnectWiFi();
void reconnectMQTT();
void mqttCallback(char *topic, byte *payload, unsigned int length);

// Temperature Control Functions
void adjustTargetTemp(int direction);
void setTargetTemp(float newTemp);
bool isValidTargetTemp(float temp);
void publishTargetTemp();

// Peltier Control Functions
void setPeltierState(bool state);
void publishPeltierConfirmation();

// Sensor Functions
void readTemperature();
void readPH();

// Control Functions
void handlePeltierControl();

// Feeding Functions
void checkScheduledFeeding();
void performFeeding(const char *source);
void publishFeedingConfirmation(const char *status);
void startFeedingBlink();
void updateFeedingBlink();
void resetDailyFeedingCount();

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

  // Print Pin Configuration
  Serial.println("PIN CONFIGURATION:");
  Serial.println("  pH Sensor       : GPIO " + String(PIN_PH_SENSOR));
  Serial.println("  Temperature     : GPIO " + String(PIN_DS18B20));
  Serial.println("  Servo           : GPIO " + String(PIN_SERVO));
  Serial.println("  LCD SCL         : GPIO " + String(PIN_LCD_SCL));
  Serial.println("  LCD SDA         : GPIO " + String(PIN_LCD_SDA));
  Serial.println("  LED Indicator   : GPIO " + String(PIN_LED_INDICATOR));
  Serial.println("  Temp UP Button  : GPIO " + String(PIN_TEMP_UP_BTN));
  Serial.println("  Temp DOWN Button: GPIO " + String(PIN_TEMP_DOWN_BTN));
  Serial.println("  Feed Button     : GPIO " + String(PIN_MANUAL_FEED_BTN));
  Serial.println("  Peltier PWM     : GPIO " + String(PIN_PELTIER_PWM));
  Serial.println();

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("[EEPROM] Initialized - Reading pH calibration data");

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
  lcdMode = LCD_STARTUP;

  lcd.clear();
  lcd.setCursor(4, 1);
  lcd.print("AQUASENSE V2");
  lcd.setCursor(2, 2);
  lcd.print("Initializing...");

  // Initialize Sensors
  setupSensors();

  // Initialize Servo
  setupActuators();

  // Initialize PID
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, 255);
  tempPID.SetSampleTime(1000);

  // Connect to WiFi
  setupWiFi();

  // Setup MQTT
  setupMQTT();

  // Sync time if WiFi connected
  if (wifiConnected)
  {
    syncTimeWithNTP();
  }

  // ✅✅✅ SMART PELTIER STARTUP ✅✅✅
  smartPeltierStartup();
  // ✅✅✅ END OF SMART STARTUP ✅✅✅

  // Show ready message
  lcd.clear();
  lcd.setCursor(4, 1);
  lcd.print("System Ready!");
  lcd.setCursor(2, 2);
  if (peltierOn)
  {
    lcd.print("Peltier: Cooling");
  }
  else
  {
    lcd.print("Peltier: Standby");
  }
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

  // Control Peltier based on temperature
  handlePeltierControl();

  // Check for scheduled feeding
  if (timesynced && !feedingInProgress)
  {
    checkScheduledFeeding();
  }

  // Update feeding blink animation
  updateFeedingBlink();

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
  Serial.println("[LCD] Initialized on GPIO " + String(PIN_LCD_SCL) + " (SCL) and GPIO " + String(PIN_LCD_SDA) + " (SDA)");
}

void setupSensors()
{
  // DS18B20 Temperature Sensor
  tempSensor.begin();
  tempSensor.setResolution(12);

  Serial.print("[DS18B20] Found ");
  Serial.print(tempSensor.getDeviceCount());
  Serial.println(" device(s) on GPIO " + String(PIN_DS18B20));
  Serial.println("[DS18B20] Resolution set to 12-bit (0.0625°C)");

  // pH Sensor
  phSensor.begin();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.println("[pH Sensor] Greenponik library initialized on GPIO " + String(PIN_PH_SENSOR));
  Serial.println("[pH Sensor] Calibration data loaded from EEPROM");
  Serial.println("[pH Sensor] ADC: 12-bit resolution, 3.3V reference");
}

void setupActuators()
{
  feedServo.attach(PIN_SERVO);
  feedServo.write(SERVO_REST_ANGLE);
  Serial.println("[Servo] Initialized at rest position on GPIO " + String(PIN_SERVO));
}

void setupWiFi()
{
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

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

    if (lastFeedDay != timeinfo.tm_mday)
    {
      resetDailyFeedingCount();
      lastFeedDay = timeinfo.tm_mday;
    }
  }
  else
  {
    timesynced = false;
    Serial.println("[NTP] Time sync failed!");
  }
}

// ============================================
// SMART PELTIER STARTUP
// ============================================

void smartPeltierStartup()
{
  Serial.println("\n[Peltier] Starting smart initialization sequence...");

  // Turn ON peltier for initialization delay
  peltierOn = true;
  analogWrite(PIN_PELTIER_PWM, PELTIER_IDLE_PWM);
  peltierStartTime = millis();
  lastPeltierChange = millis();

  Serial.println("[Peltier] Turned ON for 3-second initialization");
  Serial.print("[Peltier] Target temperature: ");
  Serial.print(targetTemp, 1);
  Serial.println("°C");

  // Show initialization message on LCD
  lcd.setCursor(2, 3);
  lcd.print("Peltier: Init...");

  // Wait for initialization delay while reading temperature
  Serial.println("[Peltier] Waiting for sensor stabilization...");

  unsigned long initStart = millis();
  while (millis() - initStart < PELTIER_INIT_DELAY)
  {
    // Read temperature during wait
    tempSensor.requestTemperatures();
    temperature = tempSensor.getTempCByIndex(0);
    currentTemp = temperature;

    // Update LCD with countdown
    int remainingSeconds = (PELTIER_INIT_DELAY - (millis() - initStart)) / 1000 + 1;
    lcd.setCursor(2, 3);
    lcd.print("Peltier: Init ");
    lcd.print(remainingSeconds);
    lcd.print("s  ");

    delay(100);
  }

  // Make intelligent decision
  Serial.println("\n[Peltier] Initialization complete - Making decision...");
  Serial.print("[Peltier] Current temperature: ");
  Serial.print(temperature, 2);
  Serial.println("°C");
  Serial.print("[Peltier] Target temperature: ");
  Serial.print(targetTemp, 1);
  Serial.println("°C");

  if (temperature > targetTemp)
  {
    // Temperature is above target - KEEP COOLING
    peltierOn = true;
    Serial.println("[Peltier] Decision: KEEP ON (cooling needed)");
    Serial.print("[Peltier] Temperature difference: +");
    Serial.print(temperature - targetTemp, 1);
    Serial.println("°C");
  }
  else
  {
    // Temperature is at or below target - TURN OFF
    peltierOn = false;
    analogWrite(PIN_PELTIER_PWM, 0);
    Serial.println("[Peltier] Decision: TURN OFF (target reached)");
    Serial.print("[Peltier] Temperature difference: ");
    Serial.print(temperature - targetTemp, 1);
    Serial.println("°C");
  }

  peltierInitialized = true;
  Serial.println("[Peltier] Smart startup complete - Entering normal operation\n");
}

// ============================================
// RECONNECTION FUNCTIONS
// ============================================

void reconnectWiFi()
{
  unsigned long currentMillis = millis();

  if (WiFi.status() == WL_CONNECTED)
  {
    if (!wifiConnected)
    {
      wifiConnected = true;
      digitalWrite(PIN_LED_INDICATOR, HIGH);
      Serial.println("[WiFi] Connection detected!");
    }
    wifiReconnectState = WIFI_IDLE;
    return;
  }

  if (wifiConnected)
  {
    wifiConnected = false;
    digitalWrite(PIN_LED_INDICATOR, LOW);
    Serial.println("[WiFi] Connection lost!");
  }

  switch (wifiReconnectState)
  {
  case WIFI_IDLE:
    Serial.println("[WiFi] Attempting to reconnect...");
    WiFi.disconnect();
    wifiReconnectState = WIFI_DISCONNECTING;
    wifiReconnectStartTime = currentMillis;
    break;

  case WIFI_DISCONNECTING:
    if (currentMillis - wifiReconnectStartTime >= 500)
    {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      wifiReconnectState = WIFI_CONNECTING;
      wifiReconnectStartTime = currentMillis;
      Serial.println("[WiFi] Connection initiated...");
    }
    break;

  case WIFI_CONNECTING:
    if (WiFi.status() == WL_CONNECTED)
    {
      wifiConnected = true;
      wifiReconnectState = WIFI_IDLE;
      digitalWrite(PIN_LED_INDICATOR, HIGH);
      Serial.println("[WiFi] Reconnected!");
      Serial.print("[WiFi] IP Address: ");
      Serial.println(WiFi.localIP());

      lcd.clear();
      lcd.setCursor(2, 1);
      lcd.print("WiFi Reconnected!");
      delay(2000);
      setLCDNormalMode();

      syncTimeWithNTP();
    }
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

    mqttClient.subscribe(TOPIC_CMD_FEED_REQ);
    mqttClient.subscribe(TOPIC_CMD_PELTIER_REQ);
    mqttClient.subscribe(TOPIC_CMD_TEMP_ADJUST_REQ);
    mqttClient.subscribe(TOPIC_CMD_SETTEMP_REQ);

    Serial.println("\n=== MQTT Connected ===");
    Serial.println("ESP is SUBSCRIBED to (receives from app):");
    Serial.println("  - " + String(TOPIC_CMD_FEED_REQ));
    Serial.println("  - " + String(TOPIC_CMD_PELTIER_REQ));
    Serial.println("  - " + String(TOPIC_CMD_TEMP_ADJUST_REQ));
    Serial.println("  - " + String(TOPIC_CMD_SETTEMP_REQ));

    mqttClient.publish(TOPIC_STATUS, "{\"status\":\"online\"}");
    publishTargetTemp();
    publishPeltierConfirmation();
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
  Serial.print("\n[MQTT RECEIVED] Topic: ");
  Serial.println(topic);
  Serial.print("[MQTT RECEIVED] Payload: ");

  String message = "";
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  message.trim();
  Serial.println(message);

  if (strcmp(topic, TOPIC_CMD_FEED_REQ) == 0)
  {
    Serial.println(">>> Feed command received from app!");
    performFeeding("MQTT App");
  }

  if (strcmp(topic, TOPIC_CMD_PELTIER_REQ) == 0)
  {
    Serial.println(">>> Peltier command received from app!");

    if (message.indexOf("\"on\"") > 0 || message.indexOf("true") > 0)
    {
      setPeltierState(true);
      Serial.println(">>> Peltier turned ON via MQTT");
    }
    else if (message.indexOf("\"off\"") > 0 || message.indexOf("false") > 0)
    {
      setPeltierState(false);
      Serial.println(">>> Peltier turned OFF via MQTT");
    }

    int tempIdx = message.indexOf("\"target\":");
    if (tempIdx > 0)
    {
      String tempStr = message.substring(tempIdx + 9);
      int endIdx = tempStr.indexOf(',');
      if (endIdx < 0)
        endIdx = tempStr.indexOf('}');
      tempStr = tempStr.substring(0, endIdx);
      tempStr.trim();

      float newTemp = tempStr.toFloat();
      if (isValidTargetTemp(newTemp))
      {
        setTargetTemp(newTemp);
      }
    }

    publishPeltierConfirmation();
  }

  if (strcmp(topic, TOPIC_CMD_TEMP_ADJUST_REQ) == 0)
  {
    Serial.println(">>> Temperature adjustment command received from app!");

    if (message.equalsIgnoreCase("up"))
    {
      adjustTargetTemp(1);
    }
    else if (message.equalsIgnoreCase("down"))
    {
      adjustTargetTemp(-1);
    }
    else
    {
      Serial.println("[Temp] Invalid adjustment command. Use 'up' or 'down'");
      String errorResponse = "{\"status\":\"error\",\"message\":\"Invalid command\"}";
      mqttClient.publish(TOPIC_CMD_TEMP_ADJUST_RES, errorResponse.c_str());
      return;
    }

    String response = "{\"status\":\"success\",\"targetTemp\":" +
                      String(targetTemp, TEMP_DECIMAL_PLACES) +
                      ",\"currentTemp\":" + String(temperature, 2) + "}";
    mqttClient.publish(TOPIC_CMD_TEMP_ADJUST_RES, response.c_str());
    Serial.println(">>> Temperature adjustment confirmation sent to app");
  }

  if (strcmp(topic, TOPIC_CMD_SETTEMP_REQ) == 0)
  {
    Serial.println(">>> Direct temperature set command received from app!");

    float newTemp = message.toFloat();

    if (isValidTargetTemp(newTemp))
    {
      setTargetTemp(newTemp);

      String response = "{\"status\":\"success\",\"targetTemp\":" +
                        String(targetTemp, TEMP_DECIMAL_PLACES) +
                        ",\"currentTemp\":" + String(temperature, 2) + "}";
      mqttClient.publish(TOPIC_CMD_SETTEMP_RES, response.c_str());
      Serial.println(">>> Temperature set confirmation sent to app");
    }
    else
    {
      Serial.print("[Temp] Invalid temperature: ");
      Serial.print(newTemp);
      Serial.println("°C");

      String errorResponse = "{\"status\":\"error\",\"message\":\"Temperature out of range\",\"min\":" +
                             String(TEMP_TARGET_MIN, 1) + ",\"max\":" +
                             String(TEMP_TARGET_MAX, 1) + "}";
      mqttClient.publish(TOPIC_CMD_SETTEMP_RES, errorResponse.c_str());
      Serial.println(">>> Temperature set error sent to app");
    }
  }
}

// ============================================
// TEMPERATURE CONTROL FUNCTIONS
// ============================================

void adjustTargetTemp(int direction)
{
  float newTemp = targetTemp + (direction * TEMP_STEP);

  if (isValidTargetTemp(newTemp))
  {
    setTargetTemp(newTemp);
  }
  else
  {
    if (direction > 0)
    {
      Serial.println("[Temp] Already at maximum temperature");
    }
    else
    {
      Serial.println("[Temp] Already at minimum temperature");
    }
  }
}

void setTargetTemp(float newTemp)
{
  float roundedTemp = round(newTemp * pow(10, TEMP_DECIMAL_PLACES)) / pow(10, TEMP_DECIMAL_PLACES);
  targetTemp = roundedTemp;

  Serial.print("[Temp] Target temperature set to: ");
  Serial.print(targetTemp, TEMP_DECIMAL_PLACES);
  Serial.println("°C");

  publishTargetTemp();
}

bool isValidTargetTemp(float temp)
{
  return (temp >= TEMP_TARGET_MIN && temp <= TEMP_TARGET_MAX);
}

void publishTargetTemp()
{
  if (mqttConnected)
  {
    char tempStr[10];
    dtostrf(targetTemp, 4, TEMP_DECIMAL_PLACES, tempStr);
    mqttClient.publish(TOPIC_TEMP_TARGET, tempStr, true);

    Serial.print("[MQTT] Published target temperature: ");
    Serial.print(tempStr);
    Serial.println("°C");
  }
}

// ============================================
// PELTIER CONTROL FUNCTIONS
// ============================================

void setPeltierState(bool state)
{
  if (!peltierInitialized)
  {
    Serial.println("[Peltier] Cannot change state - still initializing");
    return;
  }

  unsigned long currentMillis = millis();
  unsigned long timeSinceLastChange = currentMillis - lastPeltierChange;

  if (timeSinceLastChange < PELTIER_MIN_CYCLE_TIME)
  {
    Serial.println("[Peltier] State change blocked - minimum cycle time not met");
    Serial.print("[Peltier] Wait ");
    Serial.print((PELTIER_MIN_CYCLE_TIME - timeSinceLastChange) / 1000.0, 1);
    Serial.println(" more seconds");
    return;
  }

  peltierOn = state;
  lastPeltierChange = currentMillis;

  if (!peltierOn)
  {
    analogWrite(PIN_PELTIER_PWM, PELTIER_OFF_PWM);
    Serial.println("[Peltier] Manually turned OFF");
  }
  else
  {
    Serial.println("[Peltier] Manually turned ON - automatic control active");
  }
}

void publishPeltierConfirmation()
{
  if (mqttConnected)
  {
    String response = "{\"peltierOn\":" + String(peltierOn ? "true" : "false") +
                      ",\"currentTemp\":" + String(temperature, 2) +
                      ",\"targetTemp\":" + String(targetTemp, TEMP_DECIMAL_PLACES) +
                      ",\"pwm\":" + String((int)peltierOutput) + "}";

    mqttClient.publish(TOPIC_CMD_PELTIER_RES, response.c_str());
    Serial.println("[MQTT] Published peltier confirmation: " + response);
  }
}

// ============================================
// SENSOR READING FUNCTIONS
// ============================================

void readTemperature()
{
  tempSensor.requestTemperatures();
  temperature = tempSensor.getTempCByIndex(0);
  currentTemp = temperature;

  if (temperature < -50 || temperature > 100)
  {
    Serial.println("[DS18B20] Error reading temperature!");
    temperature = 0.0;
  }
  else
  {
    Serial.print("[DS18B20] Temperature: ");
    Serial.print(temperature, 2);
    Serial.print("°C | Target: ");
    Serial.print(targetTemp, TEMP_DECIMAL_PLACES);
    Serial.println("°C");
  }
}

void readPH()
{
  int rawADC = analogRead(PIN_PH_SENSOR);
  voltage = analogReadMilliVolts(PIN_PH_SENSOR);
  phValue = phSensor.readPH(voltage, temperature);

  Serial.print("[pH Sensor] Raw ADC: ");
  Serial.print(rawADC);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print("mV | pH: ");
  Serial.print(phValue, 2);
  Serial.print(" | Temp: ");
  Serial.print(temperature, 1);
  Serial.println("°C");
}

// ============================================
// PELTIER CONTROL WITH PROTECTION
// ============================================

void handlePeltierControl()
{
  if (!peltierInitialized)
  {
    return;
  }

  unsigned long currentMillis = millis();

  // Calculate temperature difference
  float tempDiff = temperature - targetTemp;

  // Determine desired state based on temperature
  bool shouldBeOn = false;

  if (peltierOn)
  {
    // Currently ON - check if should turn OFF
    if (temperature <= targetTemp)
    {
      shouldBeOn = false;
    }
    else
    {
      shouldBeOn = true;
    }
  }
  else
  {
    // Currently OFF - check if should turn ON
    if (temperature > (targetTemp + TEMP_HYSTERESIS))
    {
      shouldBeOn = true;
    }
    else
    {
      shouldBeOn = false;
    }
  }

  // Protection: Enforce minimum time between state changes
  if (shouldBeOn != peltierOn)
  {
    unsigned long timeSinceLastChange = currentMillis - lastPeltierChange;

    if (timeSinceLastChange >= PELTIER_MIN_CYCLE_TIME)
    {
      peltierOn = shouldBeOn;
      lastPeltierChange = currentMillis;

      if (peltierOn)
      {
        Serial.println("[Peltier] State Change: OFF → ON (cooling required)");
        Serial.print("[Peltier] Temperature: ");
        Serial.print(temperature, 2);
        Serial.print("°C > Target: ");
        Serial.print(targetTemp, 1);
        Serial.println("°C");
      }
      else
      {
        Serial.println("[Peltier] State Change: ON → OFF (target reached)");
        Serial.print("[Peltier] Temperature: ");
        Serial.print(temperature, 2);
        Serial.print("°C ≤ Target: ");
        Serial.print(targetTemp, 1);
        Serial.println("°C");
      }
    }
  }

  // Apply the current state
  if (peltierOn)
  {
    tempPID.Compute();

    if (abs(tempDiff) <= TEMP_HYSTERESIS)
    {
      analogWrite(PIN_PELTIER_PWM, PELTIER_IDLE_PWM);
    }
    else if (tempDiff > 0)
    {
      analogWrite(PIN_PELTIER_PWM, (int)peltierOutput);
    }
    else
    {
      analogWrite(PIN_PELTIER_PWM, PELTIER_IDLE_PWM);
    }
  }
  else
  {
    analogWrite(PIN_PELTIER_PWM, 0);
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

  if (!fed1Today && currentHour == FEED_TIME_1_HOUR && currentMin == FEED_TIME_1_MIN)
  {
    performFeeding("Schedule 08:00");
    fed1Today = true;
  }

  if (!fed2Today && currentHour == FEED_TIME_2_HOUR && currentMin == FEED_TIME_2_MIN)
  {
    performFeeding("Schedule 20:00");
    fed2Today = true;
  }

  if (currentHour == 0 && currentMin == 0)
  {
    resetDailyFeedingCount();
  }
}

void performFeeding(const char *source)
{
  if (feedingInProgress)
  {
    Serial.println("[Feeding] Already in progress, skipping...");
    publishFeedingConfirmation("busy");
    return;
  }

  feedingInProgress = true;

  Serial.print("[Feeding] Feeding triggered by: ");
  Serial.println(source);

  feedingCountToday++;
  startFeedingBlink();

  feedServo.write(SERVO_FEED_ANGLE);
  delay(SERVO_FEED_DURATION);
  feedServo.write(SERVO_REST_ANGLE);

  Serial.print("[Feeding] Feeding complete. Count today: ");
  Serial.println(feedingCountToday);

  publishFeedingConfirmation("completed");

  feedingInProgress = false;
}

void publishFeedingConfirmation(const char *status)
{
  if (mqttConnected)
  {
    String response = "{\"action\":\"feeding\",\"status\":\"" + String(status) +
                      "\",\"count\":" + String(feedingCountToday) + "}";
    mqttClient.publish(TOPIC_CMD_FEED_RES, response.c_str());

    Serial.println("[MQTT] Published feeding confirmation: " + response);
  }
}

void startFeedingBlink()
{
  feedingBlinkActive = true;
  feedingBlinkRemaining = FEED_BLINK_COUNT * 2;
  feedingBlinkState = true;
  lastFeedingBlinkTime = millis();
}

void updateFeedingBlink()
{
  if (!feedingBlinkActive)
    return;

  unsigned long currentMillis = millis();

  if (currentMillis - lastFeedingBlinkTime >= FEED_BLINK_INTERVAL)
  {
    lastFeedingBlinkTime = currentMillis;

    feedingBlinkState = !feedingBlinkState;
    feedingBlinkRemaining--;

    if (feedingBlinkRemaining <= 0)
    {
      feedingBlinkActive = false;
      feedingBlinkState = true;
    }
  }
}

void resetDailyFeedingCount()
{
  feedingCountToday = 0;
  fed1Today = false;
  fed2Today = false;
  Serial.println("[Feeding] Daily counters reset at midnight");
}

// ============================================
// BUTTON HANDLING
// ============================================

void checkButtons()
{
  unsigned long currentMillis = millis();

  if ((currentMillis - lastDebounceTime) < DEBOUNCE_DELAY)
  {
    return;
  }

  if (digitalRead(PIN_TEMP_UP_BTN) == LOW && !tempUpPressed)
  {
    tempUpPressed = true;
    lastDebounceTime = currentMillis;
    adjustTargetTemp(1);
  }
  else if (digitalRead(PIN_TEMP_UP_BTN) == HIGH)
  {
    tempUpPressed = false;
  }

  if (digitalRead(PIN_TEMP_DOWN_BTN) == LOW && !tempDownPressed)
  {
    tempDownPressed = true;
    lastDebounceTime = currentMillis;
    adjustTargetTemp(-1);
  }
  else if (digitalRead(PIN_TEMP_DOWN_BTN) == HIGH)
  {
    tempDownPressed = false;
  }

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
// LCD DISPLAY FUNCTIONS - FIXED
// ============================================

void updateLCD()
{
  unsigned long currentMillis = millis();

  if (lcdMode == LCD_TEMP_MESSAGE)
  {
    if (currentMillis - tempMessageStartTime >= TEMP_MESSAGE_DURATION)
    {
      lcdMode = LCD_NORMAL;
      lcd.clear();
      // Clear buffer to force full redraw
      for (int i = 0; i < 4; i++)
      {
        lcdBuffer[i][0] = '\0';
      }
    }
  }

  switch (lcdMode)
  {
  case LCD_STARTUP:
    break;

  case LCD_TEMP_MESSAGE:
    break;

  case LCD_NORMAL:
    displayNormalStatus();
    break;
  }
}

void displayNormalStatus()
{
  char line[21];

  // Row 0: Temperature + Time (HH:MM)
  // Build complete line first
  char tempStr[8];
  dtostrf(temperature, 4, 1, tempStr);

  if (timesynced)
  {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
      sprintf(line, "Temp:%sC    %02d:%02d", tempStr, timeinfo.tm_hour, timeinfo.tm_min);
    }
    else
    {
      sprintf(line, "Temp:%sC    --:--", tempStr);
    }
  }
  else
  {
    sprintf(line, "Temp:%sC    --:--", tempStr);
  }

  // Only update if changed
  if (strcmp(line, lcdBuffer[0]) != 0)
  {
    lcd.setCursor(0, 0);
    lcd.print(line);
    strcpy(lcdBuffer[0], line);
  }

  // Row 1: pH + WiFi/MQTT indicators
  char phStr[8];
  dtostrf(phValue, 4, 2, phStr);
  sprintf(line, "pH: %s         %c%c", phStr,
          wifiConnected ? 'W' : ' ',
          mqttConnected ? 'M' : ' ');

  if (strcmp(line, lcdBuffer[1]) != 0)
  {
    lcd.setCursor(0, 1);
    lcd.print(line);
    strcpy(lcdBuffer[1], line);
  }

  // Row 2: Target Temperature + Peltier/Time indicators
  char targetStr[8];
  dtostrf(targetTemp, 4, TEMP_DECIMAL_PLACES, targetStr);
  sprintf(line, "Target:%sC     %c%c", targetStr,
          peltierOn ? 'P' : ' ',
          timesynced ? 'T' : ' ');

  if (strcmp(line, lcdBuffer[2]) != 0)
  {
    lcd.setCursor(0, 2);
    lcd.print(line);
    strcpy(lcdBuffer[2], line);
  }

  // Row 3: Feeding Counter with Blink Animation
  if (feedingBlinkActive && !feedingBlinkState)
  {
    sprintf(line, "Feed:                   ");
  }
  else
  {
    sprintf(line, "Feed: %-14d", feedingCountToday);
  }

  if (strcmp(line, lcdBuffer[3]) != 0)
  {
    lcd.setCursor(0, 3);
    lcd.print(line);
    strcpy(lcdBuffer[3], line);
  }
}

void setLCDTempMessage(const char *line1, const char *line2,
                       const char *line3, const char *line4)
{
  lcdMode = LCD_TEMP_MESSAGE;
  tempMessageStartTime = millis();

  lcd.clear();

  // Clear buffer
  for (int i = 0; i < 4; i++)
  {
    lcdBuffer[i][0] = '\0';
  }

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
  // Clear buffer to force full redraw
  for (int i = 0; i < 4; i++)
  {
    lcdBuffer[i][0] = '\0';
  }
}

// ============================================
// MQTT PUBLISH
// ============================================

void publishSensorData()
{
  String tempMsg = String(temperature, 2);
  mqttClient.publish(TOPIC_TEMP, tempMsg.c_str());

  String phMsg = String(phValue, 2);
  mqttClient.publish(TOPIC_PH, phMsg.c_str());

  publishTargetTemp();

  String statusMsg = "{";
  statusMsg += "\"temperature\":" + String(temperature, 2) + ",";
  statusMsg += "\"target_temp\":" + String(targetTemp, TEMP_DECIMAL_PLACES) + ",";
  statusMsg += "\"ph\":" + String(phValue, 2) + ",";
  statusMsg += "\"peltier_on\":" + String(peltierOn ? "true" : "false") + ",";
  statusMsg += "\"peltier_pwm\":" + String((int)peltierOutput) + ",";
  statusMsg += "\"feeding_count\":" + String(feedingCountToday);
  statusMsg += "}";
  mqttClient.publish(TOPIC_STATUS, statusMsg.c_str());

  Serial.println("[MQTT] Sensor data published");
}