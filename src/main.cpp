#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_ESP_PH.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>

// ============================================
// CONFIGURATION
// ============================================

#define WIFI_SSID "CIEEE"
#define WIFI_PASSWORD "FENRIRGIMANK"
#define MQTT_BROKER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_USER ""
#define MQTT_PASSWORD ""

// Pin Definitions
#define PIN_PH_SENSOR 4
#define PIN_DS18B20 5
#define PIN_SERVO 8
#define PIN_LCD_SCL 7
#define PIN_LCD_SDA 6
#define PIN_LED_INDICATOR 10
#define PIN_TEMP_UP_BTN 11
#define PIN_TEMP_DOWN_BTN 12
#define PIN_MANUAL_FEED_BTN 13
#define PIN_PELTIER_PWM 21

// MQTT Topics
#define TOPIC_TEMP "aquasense/sensor/temperature"
#define TOPIC_PH "aquasense/sensor/ph"
#define TOPIC_TEMP_TARGET "aquasense/sensor/target_temp"
#define TOPIC_STATUS "aquasense/status"
#define TOPIC_CMD_FEED_REQ "aquasense/command/feed/request"
#define TOPIC_CMD_FEED_RES "aquasense/command/feed/response"
#define TOPIC_CMD_PELTIER_REQ "aquasense/command/peltier/request"
#define TOPIC_CMD_PELTIER_RES "aquasense/command/peltier/response"
#define TOPIC_CMD_TEMP_ADJUST_REQ "aquasense/command/temp_adjust/request"
#define TOPIC_CMD_TEMP_ADJUST_RES "aquasense/command/temp_adjust/response"
#define TOPIC_CMD_SETTEMP_REQ "aquasense/command/settemp/request"
#define TOPIC_CMD_SETTEMP_RES "aquasense/command/settemp/response"

// Temperature Settings
#define TEMP_TARGET_MIN 25.0
#define TEMP_TARGET_MAX 30.0
#define TEMP_TARGET_DEFAULT 27.0
#define TEMP_HYSTERESIS 0.5
#define TEMP_STEP 1.0

// Sensor Validation Ranges
#define TEMP_MIN_VALID -55.0
#define TEMP_MAX_VALID 125.0
#define TEMP_ERROR_VALUE -127.0
#define PH_MIN_VALID 0.0
#define PH_MAX_VALID 14.0

// Buffer Sizes (INCREASED FOR SAFETY)
#define LCD_BUFFER_SIZE 64
#define MQTT_BUFFER_SIZE 256
#define TEMP_STR_SIZE 16

// Peltier PWM Configuration
#define PELTIER_PWM_CHANNEL 4
#define PELTIER_PWM_FREQ 25000
#define PELTIER_PWM_RESOLUTION 8
#define PELTIER_ON_PWM 255
#define PELTIER_OFF_PWM 0
#define PELTIER_MIN_CYCLE_TIME 5000

// Servo Configuration
#define SERVO_FEED_ANGLE 30
#define SERVO_FEED_DURATION 500
#define SERVO_REST_ANGLE 0
#define SERVO_DETACH_DELAY 2000

// Feeding Schedule
#define FEED_TIME_1_HOUR 8
#define FEED_TIME_1_MIN 0
#define FEED_TIME_2_HOUR 20
#define FEED_TIME_2_MIN 0

// LCD Configuration
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Startup Optimization
#define WIFI_CONNECT_TIMEOUT 4
#define STARTUP_SCREEN_DELAY 1000
#define READY_SCREEN_DELAY 1000

// MQTT Reconnection Settings
#define MQTT_CONNECT_TIMEOUT 5000
#define MQTT_MAX_RETRY_COUNT 10

// ============================================
// STATE DEFINITIONS
// ============================================

// WiFi States
#define WIFI_STATE_CONNECTED 0
#define WIFI_STATE_DISCONNECTED 1
#define WIFI_STATE_RECONNECTING 2

// MQTT States
#define MQTT_STATE_CONNECTED 0
#define MQTT_STATE_DISCONNECTED 1
#define MQTT_STATE_CONNECTING 2
#define MQTT_STATE_WAITING_RETRY 3

// LCD Display Modes
#define LCD_MODE_NORMAL 0
#define LCD_MODE_TEMP_MESSAGE 1
#define LCD_MODE_STARTUP 2

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

// ============================================
// GLOBAL VARIABLES
// ============================================

// Sensor Data
float temperature = 0.0;
float phValue = 0.0;
float voltage = 0.0;
float targetTemp = TEMP_TARGET_DEFAULT;

// Last Valid Readings (for fallback)
float lastValidTemp = 25.0;
float lastValidPH = 7.0;

// System Status
bool wifiConnected = false;
bool mqttConnected = false;
bool timesynced = false;
bool feedingInProgress = false;
bool peltierOn = false;
bool peltierInitialized = false;

// WiFi State
uint8_t wifiState = WIFI_STATE_DISCONNECTED;
unsigned long wifiReconnectStartTime = 0;

// MQTT State
uint8_t mqttState = MQTT_STATE_DISCONNECTED;
unsigned long mqttReconnectStartTime = 0;
unsigned long mqttLastRetryTime = 0;
int mqttRetryCount = 0;

// Servo Control
bool servoMoving = false;
bool servoAttached = true;
unsigned long servoMoveStartTime = 0;
unsigned long servoDetachTime = 0;

// Deferred Actions
volatile bool pendingFeedAction = false;
volatile bool pendingTempAdjustUp = false;
volatile bool pendingTempAdjustDown = false;
volatile bool pendingTempSet = false;
float pendingTempValue = 0.0;

// Timing Variables
unsigned long peltierStartTime = 0;
unsigned long lastPeltierChange = 0;
unsigned long lastTempRead = 0;
unsigned long lastPhRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastNtpSync = 0;
unsigned long lastDebounceTime = 0;

// Feeding Management
int feedingCountToday = 0;
bool feedingBlinkActive = false;
int feedingBlinkRemaining = 0;
unsigned long lastFeedingBlinkTime = 0;
bool feedingBlinkState = true;
bool fed1Today = false;
bool fed2Today = false;
int lastFeedDay = -1;

// LCD Display
uint8_t lcdMode = LCD_MODE_STARTUP;
unsigned long tempMessageStartTime = 0;
char lcdBuffer[4][21];

// Button States
bool tempUpPressed = false;
bool tempDownPressed = false;
bool manualFeedPressed = false;

// MQTT Client ID
char mqttClientId[32];

// ============================================
// SENSOR VALIDATION FUNCTIONS
// ============================================

float validateTemperature(float temp)
{
  // Check for DS18B20 error value
  if (temp == TEMP_ERROR_VALUE)
  {
    Serial.println("⚠️ [TEMP] DS18B20 error value detected");
    return lastValidTemp;
  }

  // Check valid range
  if (temp < TEMP_MIN_VALID || temp > TEMP_MAX_VALID)
  {
    Serial.printf("⚠️ [TEMP] Out of range: %.2f°C\n", temp);
    return lastValidTemp;
  }

  // Valid reading - store and return
  lastValidTemp = temp;
  return temp;
}

float validatePH(float ph)
{
  // Check valid range
  if (ph < PH_MIN_VALID || ph > PH_MAX_VALID)
  {
    Serial.printf("⚠️ [PH] Out of range: %.2f\n", ph);
    return lastValidPH;
  }

  // Check for suspiciously low values
  if (ph < 1.0)
  {
    Serial.printf("⚠️ [PH] Suspiciously low: %.2f\n", ph);
    return lastValidPH;
  }

  // Valid reading - store and return
  lastValidPH = ph;
  return ph;
}

// ============================================
// HELPER FUNCTIONS
// ============================================

unsigned long getMQTTRetryInterval(int retryCount)
{
  if (retryCount == 0)
    return 5000;
  if (retryCount == 1)
    return 10000;
  if (retryCount == 2)
    return 20000;
  return 30000;
}

// Safe string formatting with bounds checking
void safeFloatToStr(char *buffer, size_t bufferSize, float value, int width, int precision)
{
  if (bufferSize < 2)
    return;

  // Format with snprintf for safety
  int written = snprintf(buffer, bufferSize, "%*.*f", width, precision, value);

  // Check for truncation
  if (written >= (int)bufferSize)
  {
    Serial.println("⚠️ [BUFFER] String truncated in safeFloatToStr");
  }
}

// ============================================
// FUNCTION DECLARATIONS
// ============================================

void setupWiFi();
void setupMQTT();
void setupSensors();
void setupActuators();
void setupLCD();
void syncTimeWithNTPBackground();
void smartPeltierStartup();
void reconnectWiFi();
void checkMQTTConnection();
void reconnectMQTT();
const char *getMQTTStateDescription(int state);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void processDeferredActions();
void adjustTargetTemp(int direction);
void setTargetTemp(float newTemp);
bool isValidTargetTemp(float temp);
void publishTargetTemp();
void setPeltierState(bool state);
void publishPeltierConfirmation();
void readTemperature();
void readPH();
void handlePeltierControl();
void checkScheduledFeeding();
void performFeeding(const char *source);
void publishFeedingConfirmation(const char *status);
void updateServoPosition();
void startFeedingBlink();
void updateFeedingBlink();
void resetDailyFeedingCount();
void checkButtons();
void updateLCD();
void displayNormalStatus();
void setLCDNormalMode();
void publishSensorData();

// ============================================
// SETUP
// ============================================

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  AQUASENSE V2 - FIXED VERSION");
  Serial.println("========================================\n");

  // Generate unique MQTT client ID
  uint64_t chipid = ESP.getEfuseMac();
  snprintf(mqttClientId, sizeof(mqttClientId), "AquaSense-%04X%08X",
           (uint16_t)(chipid >> 32), (uint32_t)chipid);
  Serial.print("[MQTT] Client ID: ");
  Serial.println(mqttClientId);

  EEPROM.begin(64);

  // GPIO Configuration
  pinMode(PIN_LED_INDICATOR, OUTPUT);
  pinMode(PIN_TEMP_UP_BTN, INPUT_PULLUP);
  pinMode(PIN_TEMP_DOWN_BTN, INPUT_PULLUP);
  pinMode(PIN_MANUAL_FEED_BTN, INPUT_PULLUP);
  digitalWrite(PIN_LED_INDICATOR, LOW);

  // Initialize subsystems
  setupLCD();
  lcdMode = LCD_MODE_STARTUP;
  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("AQUASENSE V2");
  lcd.setCursor(4, 2);
  lcd.print("Starting...");
  delay(STARTUP_SCREEN_DELAY);

  setupSensors();
  setupActuators();

  // Configure Peltier PWM
  ledcSetup(PELTIER_PWM_CHANNEL, PELTIER_PWM_FREQ, PELTIER_PWM_RESOLUTION);
  ledcAttachPin(PIN_PELTIER_PWM, PELTIER_PWM_CHANNEL);
  ledcWrite(PELTIER_PWM_CHANNEL, 0);
  Serial.print("[PELTIER] Configured on LEDC Channel ");
  Serial.println(PELTIER_PWM_CHANNEL);

  // Quick WiFi attempt
  setupWiFi();

  if (wifiConnected)
  {
    setupMQTT();
    lastNtpSync = 0;
  }

  // Fast peltier startup
  smartPeltierStartup();

  // Startup complete
  lcd.clear();
  lcd.setCursor(4, 1);
  lcd.print("System Ready!");
  lcd.setCursor(0, 2);
  lcd.print("Heap:");
  lcd.print(ESP.getFreeHeap() / 1024);
  lcd.print("KB");
  delay(READY_SCREEN_DELAY);
  setLCDNormalMode();
}

// ============================================
// MAIN LOOP
// ============================================

void loop()
{
  unsigned long currentMillis = millis();

  // WiFi management
  reconnectWiFi();

  // MQTT management
  checkMQTTConnection();

  // Always call mqttClient.loop()
  if (mqttClient.connected())
  {
    mqttClient.loop();
    processDeferredActions();
  }

  // Background NTP sync
  if (wifiConnected && !timesynced)
  {
    if (lastNtpSync == 0 || currentMillis - lastNtpSync >= 5000)
    {
      syncTimeWithNTPBackground();
      lastNtpSync = currentMillis;
    }
  }
  else if (wifiConnected && timesynced)
  {
    if (currentMillis - lastNtpSync >= 3600000)
    {
      syncTimeWithNTPBackground();
      lastNtpSync = currentMillis;
    }
  }

  // Sensor readings with validation
  if (currentMillis - lastTempRead >= 2000)
  {
    readTemperature();
    lastTempRead = currentMillis;
  }

  if (currentMillis - lastPhRead >= 5000)
  {
    readPH();
    lastPhRead = currentMillis;
  }

  // Temperature control
  handlePeltierControl();

  // Feeding management
  if (timesynced && !feedingInProgress)
  {
    checkScheduledFeeding();
  }

  updateFeedingBlink();
  updateServoPosition();
  checkButtons();

  // Display update
  if (currentMillis - lastLcdUpdate >= 1000)
  {
    updateLCD();
    lastLcdUpdate = currentMillis;
  }

  // MQTT publishing
  if (mqttConnected && (currentMillis - lastMqttPublish >= 10000))
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

  // Initialize LCD buffers
  for (int i = 0; i < 4; i++)
  {
    memset(lcdBuffer[i], 0, sizeof(lcdBuffer[i]));
  }

  Serial.println("[LCD] Initialized");
}

void setupSensors()
{
  tempSensor.begin();
  int deviceCount = tempSensor.getDeviceCount();
  Serial.printf("[DS18B20] Found %d device(s)\n", deviceCount);

  if (deviceCount == 0)
  {
    Serial.println("[DS18B20] ⚠️ WARNING: No sensor detected!");
    Serial.println("[DS18B20] Check: 4.7kΩ pull-up resistor, power, wiring");
  }

  tempSensor.setResolution(12);
  phSensor.begin();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("[Sensors] Initialized (DS18B20 + pH)");
}

void setupActuators()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  feedServo.setPeriodHertz(50);
  feedServo.attach(PIN_SERVO, 500, 2400);
  feedServo.write(SERVO_REST_ANGLE);
  delay(300);

  Serial.println("[Servo] Initialized on GPIO8");
  Serial.printf("[Servo] Feed angle: %d°, Duration: %dms\n",
                SERVO_FEED_ANGLE, SERVO_FEED_DURATION);
}

void setupWiFi()
{
  Serial.print("[WiFi] Quick connect to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < WIFI_CONNECT_TIMEOUT)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  digitalWrite(PIN_LED_INDICATOR, wifiConnected ? HIGH : LOW);

  if (wifiConnected)
  {
    wifiState = WIFI_STATE_CONNECTED;
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    wifiState = WIFI_STATE_DISCONNECTED;
    Serial.println("\n[WiFi] Not connected, will retry in background");
    lastWifiAttempt = millis();
  }
}

void setupMQTT()
{
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  mqttClient.setKeepAlive(15);

  Serial.println("[MQTT] Configured with 15s keepalive");

  if (wifiConnected)
  {
    mqttState = MQTT_STATE_DISCONNECTED;
  }
}

void syncTimeWithNTPBackground()
{
  static bool syncInProgress = false;

  if (syncInProgress)
    return;

  syncInProgress = true;
  configTime(25200, 0, "pool.ntp.org");
  struct tm timeinfo;

  if (getLocalTime(&timeinfo))
  {
    timesynced = true;
    Serial.println("[NTP] Background sync complete!");
    Serial.printf("[NTP] Current time: %02d:%02d:%02d\n",
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
  else
  {
    Serial.println("[NTP] Sync attempt (will retry)");
  }

  syncInProgress = false;
}

void smartPeltierStartup()
{
  Serial.println("[Peltier] Quick initialization...");

  tempSensor.requestTemperatures();
  float rawTemp = tempSensor.getTempCByIndex(0);
  temperature = validateTemperature(rawTemp);

  peltierOn = (temperature > targetTemp);
  ledcWrite(PELTIER_PWM_CHANNEL, peltierOn ? PELTIER_ON_PWM : PELTIER_OFF_PWM);
  lastPeltierChange = millis();
  peltierInitialized = true;

  Serial.print("[Peltier] Initialized - State: ");
  Serial.println(peltierOn ? "ON (Cooling)" : "OFF");
  Serial.printf("[Peltier] Current: %.2f°C, Target: %.1f°C\n",
                temperature, targetTemp);
}

// ============================================
// WIFI RECONNECTION
// ============================================

void reconnectWiFi()
{
  wl_status_t status = WiFi.status();
  unsigned long currentMillis = millis();

  switch (wifiState)
  {
  case WIFI_STATE_CONNECTED:
    if (status != WL_CONNECTED)
    {
      wifiConnected = false;
      mqttConnected = false;
      digitalWrite(PIN_LED_INDICATOR, LOW);
      Serial.println("[WiFi] ❌ Connection lost!");
      wifiState = WIFI_STATE_DISCONNECTED;
      lastWifiAttempt = currentMillis;
      mqttState = MQTT_STATE_DISCONNECTED;
    }
    break;

  case WIFI_STATE_DISCONNECTED:
    if (currentMillis - lastWifiAttempt >= 30000)
    {
      Serial.println("[WiFi] Starting reconnection attempt...");
      WiFi.disconnect();
      delay(100);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      wifiReconnectStartTime = currentMillis;
      wifiState = WIFI_STATE_RECONNECTING;
    }
    break;

  case WIFI_STATE_RECONNECTING:
    if (status == WL_CONNECTED)
    {
      wifiConnected = true;
      digitalWrite(PIN_LED_INDICATOR, HIGH);
      Serial.println("[WiFi] ✅ Reconnected!");
      Serial.print("[WiFi] IP: ");
      Serial.println(WiFi.localIP());
      wifiState = WIFI_STATE_CONNECTED;
      mqttState = MQTT_STATE_DISCONNECTED;

      if (!timesynced)
      {
        lastNtpSync = 0;
      }
    }
    else if (currentMillis - wifiReconnectStartTime > 10000)
    {
      Serial.println("[WiFi] ❌ Reconnection failed, will retry in 30s");
      wifiState = WIFI_STATE_DISCONNECTED;
      lastWifiAttempt = currentMillis;
    }
    break;
  }
}

// ============================================
// MQTT RECONNECTION
// ============================================

const char *getMQTTStateDescription(int state)
{
  switch (state)
  {
  case -4:
    return "TIMEOUT";
  case -3:
    return "CONNECTION_LOST";
  case -2:
    return "CONNECT_FAILED";
  case -1:
    return "DISCONNECTED";
  case 0:
    return "CONNECTED";
  case 1:
    return "BAD_PROTOCOL";
  case 2:
    return "BAD_CLIENT_ID";
  case 3:
    return "UNAVAILABLE";
  case 4:
    return "BAD_CREDENTIALS";
  case 5:
    return "UNAUTHORIZED";
  default:
    return "UNKNOWN";
  }
}

void reconnectMQTT()
{
  if (!wifiConnected)
  {
    mqttState = MQTT_STATE_DISCONNECTED;
    mqttConnected = false;
    return;
  }

  Serial.printf("[MQTT] Connection attempt %d/%d...",
                mqttRetryCount + 1, MQTT_MAX_RETRY_COUNT);

  bool connected = (strlen(MQTT_USER) > 0)
                       ? mqttClient.connect(mqttClientId, MQTT_USER, MQTT_PASSWORD)
                       : mqttClient.connect(mqttClientId);

  if (connected)
  {
    mqttConnected = true;
    mqttState = MQTT_STATE_CONNECTED;
    mqttRetryCount = 0;

    Serial.println(" ✅ Connected!");

    mqttClient.subscribe(TOPIC_CMD_FEED_REQ);
    mqttClient.subscribe(TOPIC_CMD_PELTIER_REQ);
    mqttClient.subscribe(TOPIC_CMD_TEMP_ADJUST_REQ);
    mqttClient.subscribe(TOPIC_CMD_SETTEMP_REQ);

    mqttClient.publish(TOPIC_STATUS, "{\"status\":\"online\"}");
    publishTargetTemp();

    Serial.println("[MQTT] All subscriptions renewed");
  }
  else
  {
    mqttConnected = false;
    mqttRetryCount++;

    Serial.printf(" ❌ Failed: %s (rc=%d)\n",
                  getMQTTStateDescription(mqttClient.state()),
                  mqttClient.state());

    if (mqttRetryCount >= MQTT_MAX_RETRY_COUNT)
    {
      Serial.println("[MQTT] Max retries reached, resetting counter");
      mqttRetryCount = 0;
    }
    else
    {
      unsigned long nextRetry = getMQTTRetryInterval(mqttRetryCount - 1);
      Serial.printf("[MQTT] Will retry in %lu seconds\n", nextRetry / 1000);
    }
  }
}

void checkMQTTConnection()
{
  unsigned long currentMillis = millis();

  switch (mqttState)
  {
  case MQTT_STATE_CONNECTED:
    if (!mqttClient.connected())
    {
      mqttConnected = false;
      mqttState = MQTT_STATE_DISCONNECTED;
      mqttLastRetryTime = currentMillis;
      mqttRetryCount = 0;

      Serial.println("\n[MQTT] ❌ Connection lost!");
      Serial.println("[MQTT] Will attempt reconnection...");
    }
    break;

  case MQTT_STATE_DISCONNECTED:
    if (!wifiConnected)
      break;

    Serial.println("[MQTT] Starting reconnection...");
    mqttState = MQTT_STATE_CONNECTING;
    mqttReconnectStartTime = currentMillis;
    reconnectMQTT();
    break;

  case MQTT_STATE_CONNECTING:
    if (mqttConnected)
    {
      mqttState = MQTT_STATE_CONNECTED;
    }
    else if (currentMillis - mqttReconnectStartTime > MQTT_CONNECT_TIMEOUT)
    {
      mqttState = MQTT_STATE_WAITING_RETRY;
      mqttLastRetryTime = currentMillis;
      Serial.println("[MQTT] Connection attempt timed out");
    }
    break;

  case MQTT_STATE_WAITING_RETRY:
    if (!wifiConnected)
    {
      mqttState = MQTT_STATE_DISCONNECTED;
      break;
    }

    unsigned long retryDelay = getMQTTRetryInterval(mqttRetryCount);

    if (currentMillis - mqttLastRetryTime >= retryDelay)
    {
      mqttState = MQTT_STATE_CONNECTING;
      mqttReconnectStartTime = currentMillis;
      reconnectMQTT();
    }
    break;
  }
}

// ============================================
// MQTT CALLBACK
// ============================================

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  char message[256];
  if (length >= sizeof(message))
    length = sizeof(message) - 1;
  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.printf("\n[MQTT] %s: %s\n", topic, message);

  if (strcmp(topic, TOPIC_CMD_FEED_REQ) == 0)
  {
    if (strcasecmp(message, "NOW") == 0)
    {
      Serial.println("[MQTT] Feed command received");
      pendingFeedAction = true;
    }
    return;
  }

  if (strcmp(topic, TOPIC_CMD_TEMP_ADJUST_REQ) == 0)
  {
    if (strcasecmp(message, "up") == 0)
    {
      pendingTempAdjustUp = true;
    }
    else if (strcasecmp(message, "down") == 0)
    {
      pendingTempAdjustDown = true;
    }
    return;
  }

  if (strcmp(topic, TOPIC_CMD_SETTEMP_REQ) == 0)
  {
    float newTemp = atof(message);
    if (isValidTargetTemp(newTemp))
    {
      pendingTempSet = true;
      pendingTempValue = newTemp;
    }
    return;
  }
}

// ============================================
// DEFERRED ACTIONS
// ============================================

void processDeferredActions()
{
  if (pendingFeedAction)
  {
    Serial.println("[ACTION] Processing feed action...");
    pendingFeedAction = false;
    performFeeding("MQTT App");
  }

  if (pendingTempAdjustUp)
  {
    pendingTempAdjustUp = false;
    adjustTargetTemp(1);
    if (mqttConnected)
    {
      char response[128];
      snprintf(response, sizeof(response),
               "{\"status\":\"success\",\"targetTemp\":%.1f,\"currentTemp\":%.2f}",
               targetTemp, temperature);
      mqttClient.publish(TOPIC_CMD_TEMP_ADJUST_RES, response);
    }
  }

  if (pendingTempAdjustDown)
  {
    pendingTempAdjustDown = false;
    adjustTargetTemp(-1);
    if (mqttConnected)
    {
      char response[128];
      snprintf(response, sizeof(response),
               "{\"status\":\"success\",\"targetTemp\":%.1f,\"currentTemp\":%.2f}",
               targetTemp, temperature);
      mqttClient.publish(TOPIC_CMD_TEMP_ADJUST_RES, response);
    }
  }

  if (pendingTempSet)
  {
    pendingTempSet = false;
    if (isValidTargetTemp(pendingTempValue))
    {
      setTargetTemp(pendingTempValue);
      if (mqttConnected)
      {
        char response[128];
        snprintf(response, sizeof(response),
                 "{\"status\":\"success\",\"targetTemp\":%.1f,\"currentTemp\":%.2f}",
                 targetTemp, temperature);
        mqttClient.publish(TOPIC_CMD_SETTEMP_RES, response);
      }
    }
  }
}

// ============================================
// TEMPERATURE CONTROL
// ============================================

void adjustTargetTemp(int direction)
{
  float newTemp = targetTemp + (direction * TEMP_STEP);
  if (isValidTargetTemp(newTemp))
  {
    setTargetTemp(newTemp);
  }
}

void setTargetTemp(float newTemp)
{
  targetTemp = round(newTemp * 10.0) / 10.0;
  Serial.printf("[TEMP] Target set to: %.1f°C\n", targetTemp);
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
    char tempStr[TEMP_STR_SIZE];
    snprintf(tempStr, sizeof(tempStr), "%.1f", targetTemp);
    mqttClient.publish(TOPIC_TEMP_TARGET, tempStr, true);
  }
}

// ============================================
// PELTIER CONTROL
// ============================================

void setPeltierState(bool state)
{
  if (!peltierInitialized)
    return;
  if (millis() - lastPeltierChange < PELTIER_MIN_CYCLE_TIME)
    return;

  peltierOn = state;
  lastPeltierChange = millis();
  ledcWrite(PELTIER_PWM_CHANNEL, peltierOn ? PELTIER_ON_PWM : PELTIER_OFF_PWM);

  Serial.printf("[PELTIER] State changed: %s\n", peltierOn ? "ON" : "OFF");
}

void publishPeltierConfirmation()
{
  if (mqttConnected)
  {
    char response[128];
    snprintf(response, sizeof(response),
             "{\"peltierOn\":%s,\"currentTemp\":%.2f,\"targetTemp\":%.1f,\"pwm\":%d}",
             peltierOn ? "true" : "false", temperature, targetTemp,
             peltierOn ? PELTIER_ON_PWM : PELTIER_OFF_PWM);
    mqttClient.publish(TOPIC_CMD_PELTIER_RES, response);
  }
}

// ============================================
// SENSORS (WITH VALIDATION)
// ============================================

void readTemperature()
{
  tempSensor.requestTemperatures();
  float rawTemp = tempSensor.getTempCByIndex(0);

  // Validate and store
  temperature = validateTemperature(rawTemp);
}

void readPH()
{
  voltage = analogReadMilliVolts(PIN_PH_SENSOR);
  float rawPH = phSensor.readPH(voltage, temperature);

  // Validate and store
  phValue = validatePH(rawPH);
}

// ============================================
// TEMPERATURE CONTROL
// ============================================

void handlePeltierControl()
{
  if (!peltierInitialized)
    return;
  if (millis() - lastPeltierChange < PELTIER_MIN_CYCLE_TIME)
  {
    ledcWrite(PELTIER_PWM_CHANNEL, peltierOn ? PELTIER_ON_PWM : PELTIER_OFF_PWM);
    return;
  }

  bool shouldBeOn = peltierOn
                        ? (temperature > targetTemp)
                        : (temperature > (targetTemp + TEMP_HYSTERESIS));

  if (shouldBeOn != peltierOn)
  {
    peltierOn = shouldBeOn;
    lastPeltierChange = millis();
    ledcWrite(PELTIER_PWM_CHANNEL, peltierOn ? PELTIER_ON_PWM : PELTIER_OFF_PWM);

    Serial.printf("[PELTIER] Auto control: %s (Temp: %.2f°C, Target: %.1f°C)\n",
                  peltierOn ? "ON" : "OFF", temperature, targetTemp);
  }
}

// ============================================
// FEEDING
// ============================================

void checkScheduledFeeding()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
    return;

  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;

  if (!fed1Today && h == FEED_TIME_1_HOUR && m == FEED_TIME_1_MIN)
  {
    performFeeding("Schedule 08:00");
    fed1Today = true;
  }

  if (!fed2Today && h == FEED_TIME_2_HOUR && m == FEED_TIME_2_MIN)
  {
    performFeeding("Schedule 20:00");
    fed2Today = true;
  }

  if (h == 0 && m == 0)
  {
    resetDailyFeedingCount();
  }
}

void performFeeding(const char *source)
{
  Serial.println("\n========================================");
  Serial.println("🐟 FEEDING SEQUENCE START");
  Serial.println("========================================");

  if (feedingInProgress)
  {
    Serial.println("[FEED] Already in progress - ignoring");
    publishFeedingConfirmation("busy");
    return;
  }

  Serial.printf("[FEED] Triggered by: %s\n", source);

  feedingInProgress = true;
  servoMoving = true;
  servoMoveStartTime = millis();
  feedingCountToday++;
  startFeedingBlink();

  if (!servoAttached)
  {
    Serial.println("[SERVO] Re-attaching servo...");
    feedServo.attach(PIN_SERVO, 500, 2400);
    servoAttached = true;
    delay(100);
  }

  Serial.printf("[SERVO] Moving to feed position (%d°)...\n", SERVO_FEED_ANGLE);
  feedServo.write(SERVO_FEED_ANGLE);

  Serial.println("========================================\n");
}

void updateServoPosition()
{
  if (!servoMoving)
    return;

  if (millis() - servoMoveStartTime >= SERVO_FEED_DURATION)
  {
    Serial.println("\n========================================");
    Serial.println("🐟 FEEDING SEQUENCE COMPLETE");
    Serial.println("========================================");

    Serial.printf("[SERVO] Returning to rest (%d°)...\n", SERVO_REST_ANGLE);

    feedServo.write(SERVO_REST_ANGLE);
    delay(500);

    servoMoving = false;
    feedingInProgress = false;
    servoDetachTime = millis();

    Serial.println("[SERVO] Returned to rest position");
    Serial.printf("[FEED] Complete! Today's count: %d\n", feedingCountToday);
    Serial.println("========================================\n");

    publishFeedingConfirmation("completed");
  }
}

void publishFeedingConfirmation(const char *status)
{
  if (mqttConnected)
  {
    char response[128];
    snprintf(response, sizeof(response),
             "{\"action\":\"feeding\",\"status\":\"%s\",\"count\":%d}",
             status, feedingCountToday);
    mqttClient.publish(TOPIC_CMD_FEED_RES, response);
  }
}

void startFeedingBlink()
{
  feedingBlinkActive = true;
  feedingBlinkRemaining = 4;
  feedingBlinkState = true;
  lastFeedingBlinkTime = millis();
}

void updateFeedingBlink()
{
  if (!feedingBlinkActive)
    return;

  if (millis() - lastFeedingBlinkTime >= 500)
  {
    lastFeedingBlinkTime = millis();
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
  Serial.println("[FEED] Daily counter reset");
}

// ============================================
// BUTTONS
// ============================================

void checkButtons()
{
  if (millis() - lastDebounceTime < 50)
    return;

  if (digitalRead(PIN_TEMP_UP_BTN) == LOW && !tempUpPressed)
  {
    tempUpPressed = true;
    lastDebounceTime = millis();
    adjustTargetTemp(1);
    Serial.println("[BTN] Temp UP pressed");
  }
  else if (digitalRead(PIN_TEMP_UP_BTN) == HIGH)
  {
    tempUpPressed = false;
  }

  if (digitalRead(PIN_TEMP_DOWN_BTN) == LOW && !tempDownPressed)
  {
    tempDownPressed = true;
    lastDebounceTime = millis();
    adjustTargetTemp(-1);
    Serial.println("[BTN] Temp DOWN pressed");
  }
  else if (digitalRead(PIN_TEMP_DOWN_BTN) == HIGH)
  {
    tempDownPressed = false;
  }

  if (digitalRead(PIN_MANUAL_FEED_BTN) == LOW && !manualFeedPressed)
  {
    manualFeedPressed = true;
    lastDebounceTime = millis();
    Serial.println("[BTN] Manual FEED pressed");
    performFeeding("Manual Button");
  }
  else if (digitalRead(PIN_MANUAL_FEED_BTN) == HIGH)
  {
    manualFeedPressed = false;
  }
}

// ============================================
// LCD DISPLAY (FIXED WITH SAFE BUFFERS)
// ============================================

void updateLCD()
{
  if (lcdMode != LCD_MODE_NORMAL)
    return;
  displayNormalStatus();
}

void displayNormalStatus()
{
  // Use static buffers to keep them off the stack
  static char line[LCD_BUFFER_SIZE];
  static char tempStr[TEMP_STR_SIZE];
  static char phStr[TEMP_STR_SIZE];
  static char targetStr[TEMP_STR_SIZE];

  // Clear line buffer
  memset(line, 0, LCD_BUFFER_SIZE);

  // Safe float to string conversion
  safeFloatToStr(tempStr, sizeof(tempStr), temperature, 4, 1);
  safeFloatToStr(phStr, sizeof(phStr), phValue, 4, 2);
  safeFloatToStr(targetStr, sizeof(targetStr), targetTemp, 4, 1);

  struct tm timeinfo;
  bool hasTime = timesynced && getLocalTime(&timeinfo);

  // Line 0: Temperature and Time (SAFE)
  if (hasTime)
  {
    snprintf(line, LCD_BUFFER_SIZE, "Temp:%sC    %02d:%02d",
             tempStr, timeinfo.tm_hour, timeinfo.tm_min);
  }
  else
  {
    snprintf(line, LCD_BUFFER_SIZE, "Temp:%sC    --:--", tempStr);
  }

  if (strcmp(line, lcdBuffer[0]) != 0)
  {
    lcd.setCursor(0, 0);
    lcd.print(line);
    strncpy(lcdBuffer[0], line, sizeof(lcdBuffer[0]) - 1);
    lcdBuffer[0][sizeof(lcdBuffer[0]) - 1] = '\0';
  }

  // Line 1: pH and Status Indicators (SAFE)
  char mqttIndicator = ' ';
  if (mqttState == MQTT_STATE_CONNECTED)
    mqttIndicator = 'M';
  else if (mqttState == MQTT_STATE_CONNECTING)
    mqttIndicator = 'm';
  else if (mqttState == MQTT_STATE_WAITING_RETRY)
    mqttIndicator = '.';

  snprintf(line, LCD_BUFFER_SIZE, "pH: %s         %c%c%c",
           phStr,
           wifiConnected ? 'W' : ' ',
           mqttIndicator,
           timesynced ? 'T' : ' ');

  if (strcmp(line, lcdBuffer[1]) != 0)
  {
    lcd.setCursor(0, 1);
    lcd.print(line);
    strncpy(lcdBuffer[1], line, sizeof(lcdBuffer[1]) - 1);
    lcdBuffer[1][sizeof(lcdBuffer[1]) - 1] = '\0';
  }

  // Line 2: Target Temperature and Peltier Status (SAFE)
  snprintf(line, LCD_BUFFER_SIZE, "Target:%sC     %c ",
           targetStr, peltierOn ? 'P' : ' ');

  if (strcmp(line, lcdBuffer[2]) != 0)
  {
    lcd.setCursor(0, 2);
    lcd.print(line);
    strncpy(lcdBuffer[2], line, sizeof(lcdBuffer[2]) - 1);
    lcdBuffer[2][sizeof(lcdBuffer[2]) - 1] = '\0';
  }

  // Line 3: Feeding Count (SAFE, with blink effect)
  if (feedingBlinkActive && !feedingBlinkState)
  {
    snprintf(line, LCD_BUFFER_SIZE, "Feed:               ");
  }
  else
  {
    snprintf(line, LCD_BUFFER_SIZE, "Feed: %-14d", feedingCountToday);
  }

  if (strcmp(line, lcdBuffer[3]) != 0)
  {
    lcd.setCursor(0, 3);
    lcd.print(line);
    strncpy(lcdBuffer[3], line, sizeof(lcdBuffer[3]) - 1);
    lcdBuffer[3][sizeof(lcdBuffer[3]) - 1] = '\0';
  }
}

void setLCDNormalMode()
{
  lcdMode = LCD_MODE_NORMAL;
  lcd.clear();
  for (int i = 0; i < 4; i++)
  {
    memset(lcdBuffer[i], 0, sizeof(lcdBuffer[i]));
  }
}

// ============================================
// MQTT PUBLISH (SAFE)
// ============================================

void publishSensorData()
{
  char tempMsg[TEMP_STR_SIZE];
  char phMsg[TEMP_STR_SIZE];
  char statusMsg[MQTT_BUFFER_SIZE];

  // Safe formatting
  snprintf(tempMsg, sizeof(tempMsg), "%.2f", temperature);
  mqttClient.publish(TOPIC_TEMP, tempMsg);

  snprintf(phMsg, sizeof(phMsg), "%.2f", phValue);
  mqttClient.publish(TOPIC_PH, phMsg);

  publishTargetTemp();

  snprintf(statusMsg, sizeof(statusMsg),
           "{\"temperature\":%.2f,\"target_temp\":%.1f,\"ph\":%.2f,\"peltier_on\":%s,\"feeding_count\":%d}",
           temperature, targetTemp, phValue,
           peltierOn ? "true" : "false",
           feedingCountToday);
  mqttClient.publish(TOPIC_STATUS, statusMsg);
}