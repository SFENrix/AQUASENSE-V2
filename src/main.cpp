#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_ESP_PH.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include <ArduinoJson.h>

// WIFI CONFIGURATION

#define WIFI_SSID "CIEEE"
#define WIFI_PASSWORD "FENRIRGIMANK"

// THINGSBOARD MQTT CONFIG

#define TB_ENABLED true // Set to false to disable ThingsBoard (for basic pack)
#define TB_SERVER "thingsboard.cloud"
#define TB_PORT 1883
#define TB_TOKEN "er0x5rqcdq1x81mzumls" // GET FROM THINGSBOARD CLOUD WEB
#define TB_TELEMETRY_TOPIC "v1/devices/me/telemetry"
#define TB_ATTRIBUTES_TOPIC "v1/devices/me/attributes"
#define TB_RPC_SUBSCRIBE_TOPIC "v1/devices/me/rpc/request/+"
#define TB_PUBLISH_INTERVAL 10000

// EMQX MQTT CONFIGURATION

#define EMQX_ENABLED true // setting to false will disable EMQX broker (for Flutter app)
#define EMQX_SERVER "broker.emqx.io"
#define EMQX_PORT 1883
#define EMQX_USER ""
#define EMQX_PASSWORD ""
#define EMQX_PUBLISH_INTERVAL 5000

// EMQX Topics (for Flutter App)

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
#define TOPIC_EVENT_FEEDING "aquasense/event/feeding"

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

// Peltier Control Settings
#define TEMP_TARGET_MIN 25.0
#define TEMP_TARGET_MAX 30.0
#define TEMP_TARGET_DEFAULT 27.0
#define TEMP_HYSTERESIS 0.5
#define TEMP_STEP 1.0

// Sensor Validation
#define TEMP_MIN_VALID -55.0
#define TEMP_MAX_VALID 125.0
#define TEMP_ERROR_VALUE -127.0
#define PH_MIN_VALID 0.0
#define PH_MAX_VALID 14.0

// Buffer Sizes
#define LCD_BUFFER_SIZE 64
#define MQTT_BUFFER_SIZE 512
#define TEMP_STR_SIZE 16

// Peltier Configuration
#define PELTIER_PWM_CHANNEL 4
#define PELTIER_PWM_FREQ 25000
#define PELTIER_PWM_RESOLUTION 8
#define PELTIER_ON_PWM 255
#define PELTIER_OFF_PWM 0
#define PELTIER_MIN_CYCLE_TIME 5000

// Servo Configuration
#define SERVO_FEED_ANGLE 30
#define SERVO_FEED_DURATION 75
#define SERVO_REST_ANGLE 0

// Feeding Schedule
#define FEED_TIME_1_HOUR 8
#define FEED_TIME_1_MIN 0
#define FEED_TIME_2_HOUR 20
#define FEED_TIME_2_MIN 0

// LCD Configuration
#define LCD_ADDRESS 0x27
#define LCD_COLS 20
#define LCD_ROWS 4

// Startup Settings
#define WIFI_CONNECT_TIMEOUT 4
#define STARTUP_SCREEN_DELAY 1000
#define READY_SCREEN_DELAY 2000

// MQTT Settings
#define MQTT_CONNECT_TIMEOUT 5000
#define MQTT_MAX_RETRY_COUNT 10

// WiFi States
#define WIFI_STATE_CONNECTED 0
#define WIFI_STATE_DISCONNECTED 1
#define WIFI_STATE_RECONNECTING 2

// MQTT States
#define MQTT_STATE_CONNECTED 0
#define MQTT_STATE_DISCONNECTED 1
#define MQTT_STATE_CONNECTING 2
#define MQTT_STATE_WAITING_RETRY 3

// LCD Modes
#define LCD_MODE_NORMAL 0
#define LCD_MODE_TEMP_MESSAGE 1
#define LCD_MODE_STARTUP 2

// =====GLOBAL OBJECTS=====

// WiFi Clients (separate for each broker)
WiFiClient espClientTB;   // ThingsBoard WiFi client
WiFiClient espClientEMQX; // EMQX WiFi client

// MQTT Clients
PubSubClient mqttClientTB(espClientTB);     // ThingsBoard MQTT
PubSubClient mqttClientEMQX(espClientEMQX); // EMQX MQTT

// Sensors and Actuators
OneWire oneWire(PIN_DS18B20);
DallasTemperature tempSensor(&oneWire);
DFRobot_ESP_PH phSensor;
Servo feedServo;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// =====GLOBAL VARIABLES=====

// Sensor Data
float temperature = 0.0;
float phValue = 0.0;
float voltage = 0.0;
float targetTemp = TEMP_TARGET_DEFAULT;
float lastValidTemp = 25.0;
float lastValidPH = 7.0;

// System Status
bool wifiConnected = false;
bool mqttConnectedTB = false;
bool mqttConnectedEMQX = false;
bool timesynced = false;
bool feedingInProgress = false;
bool peltierOn = false;
bool peltierInitialized = false;

// WiFi State
uint8_t wifiState = WIFI_STATE_DISCONNECTED;
unsigned long wifiReconnectStartTime = 0;

// MQTT States (separate for each broker)
uint8_t mqttStateTB = MQTT_STATE_DISCONNECTED;
uint8_t mqttStateEMQX = MQTT_STATE_DISCONNECTED;
unsigned long mqttReconnectStartTimeTB = 0;
unsigned long mqttReconnectStartTimeEMQX = 0;
unsigned long mqttLastRetryTimeTB = 0;
unsigned long mqttLastRetryTimeEMQX = 0;
int mqttRetryCountTB = 0;
int mqttRetryCountEMQX = 0;

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

// Timing
unsigned long peltierStartTime = 0;
unsigned long lastPeltierChange = 0;
unsigned long lastTempRead = 0;
unsigned long lastPhRead = 0;
unsigned long lastMqttPublishTB = 0;
unsigned long lastMqttPublishEMQX = 0;
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
static bool resetDoneToday = false;

// LCD Display
uint8_t lcdMode = LCD_MODE_STARTUP;
unsigned long tempMessageStartTime = 0;
char lcdBuffer[4][21];

// Button States
bool tempUpPressed = false;
bool tempDownPressed = false;
bool manualFeedPressed = false;

// MQTT Client IDs
char mqttClientIdTB[32];
char mqttClientIdEMQX[32];

// =====FUNCTION DECLARATIONS=====

// Sensor Validation
float validateTemperature(float temp);
float validatePH(float ph);

// ThingsBoard Functions
unsigned long getTimestampMillis();
void publishTelemetryToThingsBoard(const char *key, float value);
void publishSensorTelemetryToThingsBoard();
void publishFeedingEventToThingsBoard(const char *source);
void publishAttributesToThingsBoard();

// EMQX Functions
void publishSensorDataToEMQX();
void publishFeedingEventToEMQX(const char *status);
void publishTargetTempToEMQX();
void publishPeltierStatusToEMQX();

// MQTT Management
void setupMQTT();
void checkMQTTConnectionTB();
void checkMQTTConnectionEMQX();
void reconnectMQTTTB();
void reconnectMQTTEMQX();
const char *getMQTTStateDescription(int state);
void mqttCallbackTB(char *topic, byte *payload, unsigned int length);
void mqttCallbackEMQX(char *topic, byte *payload, unsigned int length);

// Setup Functions
void setupWiFi();
void setupSensors();
void setupActuators();
void setupLCD();
void syncTimeWithNTPBackground();
void smartPeltierStartup();

// WiFi Management
void reconnectWiFi();

// Core Functions
void processDeferredActions();
void adjustTargetTemp(int direction);
void setTargetTemp(float newTemp);
bool isValidTargetTemp(float temp);
void setPeltierState(bool state);
void readTemperature();
void readPH();
void handlePeltierControl();
void checkScheduledFeeding();
void performFeeding(const char *source);
bool isTimePassedToday(int targetHour, int targetMin, const struct tm &now);
void updateServoPosition();
void startFeedingBlink();
void updateFeedingBlink();
void resetDailyFeedingCount();
void checkButtons();
void updateLCD();
void displayNormalStatus();
void setLCDNormalMode();

// =====SENSOR VALIDATION=====

float validateTemperature(float temp)
{
  if (temp == TEMP_ERROR_VALUE || temp < TEMP_MIN_VALID || temp > TEMP_MAX_VALID)
  {
    Serial.printf("⚠️ [TEMP] Invalid: %.2f°C, using last valid: %.2f°C\n", temp, lastValidTemp);
    return lastValidTemp;
  }
  lastValidTemp = temp;
  return temp;
}

float validatePH(float ph)
{
  if (ph < PH_MIN_VALID || ph > PH_MAX_VALID || ph < 1.0)
  {
    Serial.printf("⚠️ [PH] Invalid: %.2f, using last valid: %.2f\n", ph, lastValidPH);
    return lastValidPH;
  }
  lastValidPH = ph;
  return ph;
}

// =====THINGSBOARD FUNCTIONS=====

unsigned long getTimestampMillis()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
    return 0;
  time_t now = mktime(&timeinfo);
  return (unsigned long)now * 1000UL;
}

void publishTelemetryToThingsBoard(const char *key, float value)
{
  if (!TB_ENABLED || !mqttConnectedTB)
    return;

  StaticJsonDocument<128> doc;
  doc[key] = value;
  char jsonBuffer[128];
  serializeJson(doc, jsonBuffer);

  mqttClientTB.publish(TB_TELEMETRY_TOPIC, jsonBuffer);
  Serial.printf("[TB] %s = %.2f\n", key, value);
}

void publishSensorTelemetryToThingsBoard()
{
  if (!TB_ENABLED || !mqttConnectedTB)
    return;

  StaticJsonDocument<256> doc;
  doc["temperature"] = round(temperature * 10.0) / 10.0;
  doc["pH"] = round(phValue * 100.0) / 100.0;

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);

  if (mqttClientTB.publish(TB_TELEMETRY_TOPIC, jsonBuffer))
  {
    Serial.printf("[TB] Telemetry: %s\n", jsonBuffer);
  }
}

void publishFeedingEventToThingsBoard(const char *source)
{
  if (!TB_ENABLED || !mqttConnectedTB)
    return;

  StaticJsonDocument<64> doc;
  doc["feed"] = 1;

  char jsonBuffer[64];
  serializeJson(doc, jsonBuffer);

  if (mqttClientTB.publish(TB_TELEMETRY_TOPIC, jsonBuffer))
  {
    Serial.printf("[TB] Feed event sent: %s\n", source);
  }
}

void publishAttributesToThingsBoard()
{
  if (!TB_ENABLED || !mqttConnectedTB)
    return;

  StaticJsonDocument<256> doc;
  doc["targetTemp"] = targetTemp;
  doc["peltierOn"] = peltierOn;
  doc["feedingCountToday"] = feedingCountToday;
  doc["firmware"] = "AquaSense_V2_Dual";

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);

  mqttClientTB.publish(TB_ATTRIBUTES_TOPIC, jsonBuffer);
}

// =====EMQX FUNCTIONS (for Flutter App)=====

void publishSensorDataToEMQX()
{
  if (!EMQX_ENABLED || !mqttConnectedEMQX)
    return;

  char tempMsg[16];
  char phMsg[16];

  snprintf(tempMsg, sizeof(tempMsg), "%.1f", temperature);
  snprintf(phMsg, sizeof(phMsg), "%.2f", phValue);

  mqttClientEMQX.publish(TOPIC_TEMP, tempMsg);
  mqttClientEMQX.publish(TOPIC_PH, phMsg);

  Serial.printf("[EMQX] Temp=%.1f pH=%.2f\n", temperature, phValue);
}

void publishFeedingEventToEMQX(const char *status)
{
  if (!EMQX_ENABLED || !mqttConnectedEMQX)
    return;

  mqttClientEMQX.publish(TOPIC_EVENT_FEEDING, status);
  mqttClientEMQX.publish(TOPIC_CMD_FEED_RES, status);
  Serial.printf("[EMQX] Feed status: %s\n", status);
}

void publishTargetTempToEMQX()
{
  if (!EMQX_ENABLED || !mqttConnectedEMQX)
    return;

  char msg[16];
  snprintf(msg, sizeof(msg), "%.1f", targetTemp);
  mqttClientEMQX.publish(TOPIC_TEMP_TARGET, msg);
  Serial.printf("[EMQX] Target temp: %.1f\n", targetTemp);
}

void publishPeltierStatusToEMQX()
{
  if (!EMQX_ENABLED || !mqttConnectedEMQX)
    return;

  mqttClientEMQX.publish(TOPIC_CMD_PELTIER_RES, peltierOn ? "ON" : "OFF");
}

// =====SETUP=====

void setup()
{
  Serial.begin(115200);
  delay(100);

  Serial.println("\n========================================");
  Serial.println("  AQUASENSE V2 - DUAL MQTT EDITION");
  Serial.println("========================================");
  Serial.println("  ThingsBoard + EMQX for Flutter App");
  Serial.println("========================================\n");

  uint64_t chipid = ESP.getEfuseMac();
  snprintf(mqttClientIdTB, sizeof(mqttClientIdTB), "AquaTB-%04X%08X",
           (uint16_t)(chipid >> 32), (uint32_t)chipid);
  snprintf(mqttClientIdEMQX, sizeof(mqttClientIdEMQX), "AquaEMQX-%04X%08X",
           (uint16_t)(chipid >> 32), (uint32_t)chipid);

  Serial.printf("[MQTT] TB Client ID: %s\n", mqttClientIdTB);
  Serial.printf("[MQTT] EMQX Client ID: %s\n", mqttClientIdEMQX);

  EEPROM.begin(64);

  pinMode(PIN_LED_INDICATOR, OUTPUT);
  pinMode(PIN_TEMP_UP_BTN, INPUT_PULLUP);
  pinMode(PIN_TEMP_DOWN_BTN, INPUT_PULLUP);
  pinMode(PIN_MANUAL_FEED_BTN, INPUT_PULLUP);
  digitalWrite(PIN_LED_INDICATOR, LOW);

  setupLCD();
  lcdMode = LCD_MODE_STARTUP;
  lcd.clear();
  lcd.setCursor(4, 1);
  lcd.print("AQUASENSE V2");
  lcd.setCursor(7, 2);
  lcd.print("by R2J");
  delay(STARTUP_SCREEN_DELAY);

  setupSensors();
  setupActuators();

  ledcSetup(PELTIER_PWM_CHANNEL, PELTIER_PWM_FREQ, PELTIER_PWM_RESOLUTION);
  ledcAttachPin(PIN_PELTIER_PWM, PELTIER_PWM_CHANNEL);
  ledcWrite(PELTIER_PWM_CHANNEL, 0);

  setupWiFi();

  if (wifiConnected)
  {
    setupMQTT();
    lastNtpSync = 0;
  }

  smartPeltierStartup();

  lcd.clear();
  lcd.setCursor(4, 1);
  lcd.print("System Ready");
  lcd.setCursor(3, 2);
  lcd.print("Free Heap: ");
  lcd.print(ESP.getFreeHeap() / 1024);
  lcd.print("KB");
  delay(READY_SCREEN_DELAY);
  setLCDNormalMode();

  Serial.println("\n[SYSTEM] Initialization complete!");
  Serial.printf("[MEMORY] Free heap: %d bytes\n", ESP.getFreeHeap());
}

// =====MAIN LOOP======

void loop()
{
  unsigned long currentMillis = millis();

  reconnectWiFi();
  checkMQTTConnectionTB();
  checkMQTTConnectionEMQX();

  if (mqttClientTB.connected())
  {
    mqttClientTB.loop();
  }
  if (mqttClientEMQX.connected())
  {
    mqttClientEMQX.loop();
    processDeferredActions();
  }

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

  handlePeltierControl();

  if (timesynced && !feedingInProgress)
  {
    checkScheduledFeeding();
  }

  updateFeedingBlink();
  updateServoPosition();
  checkButtons();

  if (currentMillis - lastLcdUpdate >= 1000)
  {
    updateLCD();
    lastLcdUpdate = currentMillis;
  }

  if (TB_ENABLED && mqttConnectedTB && (currentMillis - lastMqttPublishTB >= TB_PUBLISH_INTERVAL))
  {
    publishSensorTelemetryToThingsBoard();
    publishAttributesToThingsBoard();
    lastMqttPublishTB = currentMillis;
  }

  if (EMQX_ENABLED && mqttConnectedEMQX && (currentMillis - lastMqttPublishEMQX >= EMQX_PUBLISH_INTERVAL))
  {
    publishSensorDataToEMQX();
    lastMqttPublishEMQX = currentMillis;
  }
}

// =====SETUP FUNCTIONS=====

void setupLCD()
{
  Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
  lcd.init();
  lcd.backlight();
  for (int i = 0; i < 4; i++)
  {
    memset(lcdBuffer[i], 0, sizeof(lcdBuffer[i]));
  }
  Serial.println("[LCD] Initialized");
}

void setupSensors()
{
  tempSensor.begin();
  tempSensor.setResolution(12);
  phSensor.begin();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  Serial.println("[Sensors] Initialized");
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
  Serial.println("[Servo] Initialized");
}

void setupWiFi()
{
  Serial.print("[WiFi] Connecting to ");
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
    Serial.println("\n[WiFi] ✅ Connected!");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    wifiState = WIFI_STATE_DISCONNECTED;
    Serial.println("\n[WiFi] Not connected, will retry");
    lastWifiAttempt = millis();
  }
}

void setupMQTT()
{
  Serial.println("\n========================================");
  Serial.println("  DUAL MQTT BROKER CONFIGURATION");
  Serial.println("========================================");

  if (TB_ENABLED)
  {
    Serial.println("[TB] Configuring ThingsBoard...");
    Serial.printf("  Server: %s:%d\n", TB_SERVER, TB_PORT);
    Serial.printf("  Token: %s\n", strlen(TB_TOKEN) > 10 ? "***CONFIGURED***" : "❌ NOT SET");
    mqttClientTB.setServer(TB_SERVER, TB_PORT);
    mqttClientTB.setCallback(mqttCallbackTB);
    mqttClientTB.setBufferSize(1024);
    mqttClientTB.setKeepAlive(15);
  }
  else
  {
    Serial.println("[TB] ⚠️ ThingsBoard DISABLED");
  }

  if (EMQX_ENABLED)
  {
    Serial.println("[EMQX] Configuring EMQX for Flutter App...");
    Serial.printf("  Server: %s:%d\n", EMQX_SERVER, EMQX_PORT);
    mqttClientEMQX.setServer(EMQX_SERVER, EMQX_PORT);
    mqttClientEMQX.setCallback(mqttCallbackEMQX);
    mqttClientEMQX.setBufferSize(512);
    mqttClientEMQX.setKeepAlive(15);
  }
  else
  {
    Serial.println("[EMQX] ⚠️ EMQX DISABLED");
  }

  Serial.println("========================================\n");
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
    Serial.printf("[NTP] ✅ Synced: %02d:%02d:%02d\n",
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }

  syncInProgress = false;
}

void smartPeltierStartup()
{
  tempSensor.requestTemperatures();
  float rawTemp = tempSensor.getTempCByIndex(0);
  temperature = validateTemperature(rawTemp);

  peltierOn = (temperature > targetTemp);
  ledcWrite(PELTIER_PWM_CHANNEL, peltierOn ? PELTIER_ON_PWM : PELTIER_OFF_PWM);
  lastPeltierChange = millis();
  peltierInitialized = true;

  Serial.printf("[Peltier] Init: %s (%.2f°C)\n", peltierOn ? "ON" : "OFF", temperature);
}

// =====WIFI RECONNECTION=====

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
      mqttConnectedTB = false;
      mqttConnectedEMQX = false;
      digitalWrite(PIN_LED_INDICATOR, LOW);
      Serial.println("[WiFi] ❌ Connection lost!");
      wifiState = WIFI_STATE_DISCONNECTED;
      lastWifiAttempt = currentMillis;
      mqttStateTB = MQTT_STATE_DISCONNECTED;
      mqttStateEMQX = MQTT_STATE_DISCONNECTED;
    }
    break;

  case WIFI_STATE_DISCONNECTED:
    if (currentMillis - lastWifiAttempt >= 30000)
    {
      Serial.println("[WiFi] Reconnecting...");
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
      mqttStateTB = MQTT_STATE_DISCONNECTED;
      mqttStateEMQX = MQTT_STATE_DISCONNECTED;
      if (!timesynced)
        lastNtpSync = 0;
    }
    else if (currentMillis - wifiReconnectStartTime > 10000)
    {
      Serial.println("[WiFi] ❌ Reconnection failed");
      wifiState = WIFI_STATE_DISCONNECTED;
      lastWifiAttempt = currentMillis;
    }
    break;
  }
}

// =====MQTT MANAGEMENT - THINGSBOARD=====

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

void reconnectMQTTTB()
{
  if (!TB_ENABLED || !wifiConnected)
    return;

  Serial.printf("[TB] Attempt %d/%d...", mqttRetryCountTB + 1, MQTT_MAX_RETRY_COUNT);

  bool connected = mqttClientTB.connect(mqttClientIdTB, TB_TOKEN, NULL);

  if (connected)
  {
    mqttConnectedTB = true;
    mqttStateTB = MQTT_STATE_CONNECTED;
    mqttRetryCountTB = 0;
    Serial.println("Connected!");

    mqttClientTB.subscribe(TB_RPC_SUBSCRIBE_TOPIC);
    publishAttributesToThingsBoard();
    publishSensorTelemetryToThingsBoard();
  }
  else
  {
    mqttConnectedTB = false;
    mqttRetryCountTB++;
    Serial.printf(" Failed: %s\n", getMQTTStateDescription(mqttClientTB.state()));

    if (mqttRetryCountTB >= MQTT_MAX_RETRY_COUNT)
    {
      mqttRetryCountTB = 0;
    }
  }
}

void checkMQTTConnectionTB()
{
  if (!TB_ENABLED)
    return;

  unsigned long currentMillis = millis();

  switch (mqttStateTB)
  {
  case MQTT_STATE_CONNECTED:
    if (!mqttClientTB.connected())
    {
      mqttConnectedTB = false;
      mqttStateTB = MQTT_STATE_DISCONNECTED;
      mqttLastRetryTimeTB = currentMillis;
      mqttRetryCountTB = 0;
      Serial.println("[TB] Connection lost!");
    }
    break;

  case MQTT_STATE_DISCONNECTED:
    if (!wifiConnected)
      break;
    mqttStateTB = MQTT_STATE_CONNECTING;
    mqttReconnectStartTimeTB = currentMillis;
    reconnectMQTTTB();
    break;

  case MQTT_STATE_CONNECTING:
    if (mqttConnectedTB)
    {
      mqttStateTB = MQTT_STATE_CONNECTED;
    }
    else if (currentMillis - mqttReconnectStartTimeTB > MQTT_CONNECT_TIMEOUT)
    {
      mqttStateTB = MQTT_STATE_WAITING_RETRY;
      mqttLastRetryTimeTB = currentMillis;
    }
    break;

  case MQTT_STATE_WAITING_RETRY:
    if (!wifiConnected)
    {
      mqttStateTB = MQTT_STATE_DISCONNECTED;
      break;
    }

    unsigned long retryDelay = (mqttRetryCountTB == 0) ? 5000 : 30000;

    if (currentMillis - mqttLastRetryTimeTB >= retryDelay)
    {
      mqttStateTB = MQTT_STATE_CONNECTING;
      mqttReconnectStartTimeTB = currentMillis;
      reconnectMQTTTB();
    }
    break;
  }
}

// =====MQTT MANAGEMENT - EMQX=====

void reconnectMQTTEMQX()
{
  if (!EMQX_ENABLED || !wifiConnected)
    return;

  Serial.printf("[EMQX] Attempt %d/%d...", mqttRetryCountEMQX + 1, MQTT_MAX_RETRY_COUNT);

  bool connected = mqttClientEMQX.connect(mqttClientIdEMQX, EMQX_USER, EMQX_PASSWORD);

  if (connected)
  {
    mqttConnectedEMQX = true;
    mqttStateEMQX = MQTT_STATE_CONNECTED;
    mqttRetryCountEMQX = 0;
    Serial.println("Connected!");

    mqttClientEMQX.subscribe(TOPIC_CMD_FEED_REQ);
    mqttClientEMQX.subscribe(TOPIC_CMD_TEMP_ADJUST_REQ);
    mqttClientEMQX.subscribe(TOPIC_CMD_SETTEMP_REQ);
    mqttClientEMQX.subscribe(TOPIC_CMD_PELTIER_REQ);

    publishSensorDataToEMQX();
    publishTargetTempToEMQX();
    publishPeltierStatusToEMQX();
  }
  else
  {
    mqttConnectedEMQX = false;
    mqttRetryCountEMQX++;
    Serial.printf("Failed: %s\n", getMQTTStateDescription(mqttClientEMQX.state()));

    if (mqttRetryCountEMQX >= MQTT_MAX_RETRY_COUNT)
    {
      mqttRetryCountEMQX = 0;
    }
  }
}

void checkMQTTConnectionEMQX()
{
  if (!EMQX_ENABLED)
    return;

  unsigned long currentMillis = millis();

  switch (mqttStateEMQX)
  {
  case MQTT_STATE_CONNECTED:
    if (!mqttClientEMQX.connected())
    {
      mqttConnectedEMQX = false;
      mqttStateEMQX = MQTT_STATE_DISCONNECTED;
      mqttLastRetryTimeEMQX = currentMillis;
      mqttRetryCountEMQX = 0;
      Serial.println("[EMQX] Connection lost!");
    }
    break;

  case MQTT_STATE_DISCONNECTED:
    if (!wifiConnected)
      break;
    mqttStateEMQX = MQTT_STATE_CONNECTING;
    mqttReconnectStartTimeEMQX = currentMillis;
    reconnectMQTTEMQX();
    break;

  case MQTT_STATE_CONNECTING:
    if (mqttConnectedEMQX)
    {
      mqttStateEMQX = MQTT_STATE_CONNECTED;
    }
    else if (currentMillis - mqttReconnectStartTimeEMQX > MQTT_CONNECT_TIMEOUT)
    {
      mqttStateEMQX = MQTT_STATE_WAITING_RETRY;
      mqttLastRetryTimeEMQX = currentMillis;
    }
    break;

  case MQTT_STATE_WAITING_RETRY:
    if (!wifiConnected)
    {
      mqttStateEMQX = MQTT_STATE_DISCONNECTED;
      break;
    }

    unsigned long retryDelay = (mqttRetryCountEMQX == 0) ? 5000 : 30000;

    if (currentMillis - mqttLastRetryTimeEMQX >= retryDelay)
    {
      mqttStateEMQX = MQTT_STATE_CONNECTING;
      mqttReconnectStartTimeEMQX = currentMillis;
      reconnectMQTTEMQX();
    }
    break;
  }
}

// =====MQTT CALLBACKS=====

void mqttCallbackTB(char *topic, byte *payload, unsigned int length)
{
  char message[256];
  if (length >= sizeof(message))
    length = sizeof(message) - 1;
  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.printf("[TB] RPC: %s\n", message);

  if (strstr(topic, "rpc/request"))
  {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, message) == DeserializationError::Ok)
    {
      const char *method = doc["method"];

      if (strcmp(method, "feedNow") == 0)
      {
        pendingFeedAction = true;
      }
      else if (strcmp(method, "setTargetTemp") == 0)
      {
        float temp = doc["params"]["value"];
        if (isValidTargetTemp(temp))
        {
          pendingTempSet = true;
          pendingTempValue = temp;
        }
      }
    }
  }
}

void mqttCallbackEMQX(char *topic, byte *payload, unsigned int length)
{
  char message[256];
  if (length >= sizeof(message))
    length = sizeof(message) - 1;
  memcpy(message, payload, length);
  message[length] = '\0';

  Serial.printf("[EMQX] %s: %s\n", topic, message);

  if (strcmp(topic, TOPIC_CMD_FEED_REQ) == 0)
  {
    if (strcasecmp(message, "NOW") == 0)
    {
      pendingFeedAction = true;
    }
  }
  else if (strcmp(topic, TOPIC_CMD_TEMP_ADJUST_REQ) == 0)
  {
    if (strcasecmp(message, "up") == 0)
    {
      pendingTempAdjustUp = true;
    }
    else if (strcasecmp(message, "down") == 0)
    {
      pendingTempAdjustDown = true;
    }
  }
  else if (strcmp(topic, TOPIC_CMD_SETTEMP_REQ) == 0)
  {
    float newTemp = atof(message);
    if (isValidTargetTemp(newTemp))
    {
      pendingTempSet = true;
      pendingTempValue = newTemp;
    }
  }
}

// =====DEFERRED ACTIONS=====

void processDeferredActions()
{
  if (pendingFeedAction)
  {
    pendingFeedAction = false;
    performFeeding("MQTT Command");
  }

  if (pendingTempAdjustUp)
  {
    pendingTempAdjustUp = false;
    adjustTargetTemp(1);
  }

  if (pendingTempAdjustDown)
  {
    pendingTempAdjustDown = false;
    adjustTargetTemp(-1);
  }

  if (pendingTempSet)
  {
    pendingTempSet = false;
    if (isValidTargetTemp(pendingTempValue))
    {
      setTargetTemp(pendingTempValue);
    }
  }
}

// =====TEMPERATURE CONTROL=====

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
  Serial.printf("[TEMP] Target: %.1f°C\n", targetTemp);

  publishTargetTempToEMQX();
  publishAttributesToThingsBoard();
}

bool isValidTargetTemp(float temp)
{
  return (temp >= TEMP_TARGET_MIN && temp <= TEMP_TARGET_MAX);
}

void setPeltierState(bool state)
{
  if (!peltierInitialized)
    return;
  if (millis() - lastPeltierChange < PELTIER_MIN_CYCLE_TIME)
    return;

  peltierOn = state;
  lastPeltierChange = millis();
  ledcWrite(PELTIER_PWM_CHANNEL, peltierOn ? PELTIER_ON_PWM : PELTIER_OFF_PWM);

  Serial.printf("[PELTIER] %s\n", peltierOn ? "ON" : "OFF");

  publishPeltierStatusToEMQX();
  publishAttributesToThingsBoard();
}

// =====SENSORS=====

void readTemperature()
{
  tempSensor.requestTemperatures();
  float rawTemp = tempSensor.getTempCByIndex(0);
  temperature = validateTemperature(rawTemp);
}

void readPH()
{
  voltage = analogReadMilliVolts(PIN_PH_SENSOR);
  float rawPH = phSensor.readPH(voltage, temperature);
  phValue = validatePH(rawPH);
}

void handlePeltierControl()
{
  if (!peltierInitialized)
    return;
  if (millis() - lastPeltierChange < PELTIER_MIN_CYCLE_TIME)
    return;

  bool shouldBeOn = peltierOn
                        ? (temperature > targetTemp)
                        : (temperature > (targetTemp + TEMP_HYSTERESIS));

  if (shouldBeOn != peltierOn)
  {
    setPeltierState(shouldBeOn);
  }
}

// =====FEEDING=====

bool isTimePassedToday(int targetHour, int targetMin, const struct tm &now)
{
  if (now.tm_hour > targetHour)
    return true;

  if (now.tm_hour == targetHour && now.tm_min >= targetMin)
    return true;

  return false;
}

void checkScheduledFeeding()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
    return;

  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;

  if (!fed1Today && isTimePassedToday(FEED_TIME_1_HOUR, FEED_TIME_1_MIN, timeinfo))
  {
    performFeeding("Schedule 08:00");
    fed1Today = true;
  }

  if (!fed2Today && isTimePassedToday(FEED_TIME_2_HOUR, FEED_TIME_2_MIN, timeinfo))
  {
    performFeeding("Schedule 20:00");
    fed2Today = true;
  }

  if (h == 0 && m == 0 && !resetDoneToday)
  {
    resetDailyFeedingCount();
    resetDoneToday = true;
  }

  if (h != 0 || m != 0)
  {
    resetDoneToday = false;
  }
}

void performFeeding(const char *source)
{
  if (feedingInProgress)
  {
    publishFeedingEventToEMQX("busy");
    return;
  }

  Serial.printf("\n[FEED] 🐟 Feeding: %s\n", source);

  feedingInProgress = true;
  servoMoving = true;
  servoMoveStartTime = millis();
  feedingCountToday++;
  startFeedingBlink();

  if (!servoAttached)
  {
    feedServo.attach(PIN_SERVO, 500, 2400);
    servoAttached = true;
    delay(100);
  }

  feedServo.write(SERVO_FEED_ANGLE);

  publishFeedingEventToThingsBoard(source);
  publishFeedingEventToEMQX("started");
}

void updateServoPosition()
{
  if (!servoMoving)
    return;

  if (millis() - servoMoveStartTime >= SERVO_FEED_DURATION)
  {
    feedServo.write(SERVO_REST_ANGLE);
    delay(500);

    servoMoving = false;
    feedingInProgress = false;
    servoDetachTime = millis();

    Serial.printf("[FEED] Complete! Count today: %d\n", feedingCountToday);

    publishFeedingEventToEMQX("completed");
    publishAttributesToThingsBoard();
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
  Serial.println("[FEED] Daily reset");
  publishAttributesToThingsBoard();
}

// =====BUTTONS=====

void checkButtons()
{
  if (millis() - lastDebounceTime < 50)
    return;

  if (digitalRead(PIN_TEMP_UP_BTN) == LOW && !tempUpPressed)
  {
    tempUpPressed = true;
    lastDebounceTime = millis();
    adjustTargetTemp(1);
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
  }
  else if (digitalRead(PIN_TEMP_DOWN_BTN) == HIGH)
  {
    tempDownPressed = false;
  }

  if (digitalRead(PIN_MANUAL_FEED_BTN) == LOW && !manualFeedPressed)
  {
    manualFeedPressed = true;
    lastDebounceTime = millis();
    performFeeding("Manual Button");
  }
  else if (digitalRead(PIN_MANUAL_FEED_BTN) == HIGH)
  {
    manualFeedPressed = false;
  }
}

// =====LCD DISPLAY=====

void updateLCD()
{
  if (lcdMode != LCD_MODE_NORMAL)
    return;
  displayNormalStatus();
}

void displayNormalStatus()
{
  static char line[LCD_BUFFER_SIZE];
  char tempStr[TEMP_STR_SIZE];
  char phStr[TEMP_STR_SIZE];
  char targetStr[TEMP_STR_SIZE];

  snprintf(tempStr, sizeof(tempStr), "%.1f", temperature);
  snprintf(phStr, sizeof(phStr), "%.2f", phValue);
  snprintf(targetStr, sizeof(targetStr), "%.1f", targetTemp);

  struct tm timeinfo;
  bool hasTime = timesynced && getLocalTime(&timeinfo);

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
  }

  char mqttIndicatorTB = ' ';
  char mqttIndicatorEMQX = ' ';

  if (mqttStateTB == MQTT_STATE_CONNECTED)
    mqttIndicatorTB = 'B'; // B for ThingsBoard
  else if (mqttStateTB == MQTT_STATE_CONNECTING)
    mqttIndicatorTB = 'b';

  if (mqttStateEMQX == MQTT_STATE_CONNECTED)
    mqttIndicatorEMQX = 'E'; // E for EMQX
  else if (mqttStateEMQX == MQTT_STATE_CONNECTING)
    mqttIndicatorEMQX = 'e';

  snprintf(line, LCD_BUFFER_SIZE, "pH: %s        %c%c%c%c",
           phStr,
           wifiConnected ? 'W' : ' ', // W for WiFi
           mqttIndicatorEMQX,
           mqttIndicatorTB,
           timesynced ? 'T' : ' '); // T for Time synced

  if (strcmp(line, lcdBuffer[1]) != 0)
  {
    lcd.setCursor(0, 1);
    lcd.print(line);
    strncpy(lcdBuffer[1], line, sizeof(lcdBuffer[1]) - 1);
  }

  snprintf(line, LCD_BUFFER_SIZE, "Target:%sC     %c ",
           targetStr, peltierOn ? 'P' : ' '); // P for Peltier ON

  if (strcmp(line, lcdBuffer[2]) != 0)
  {
    lcd.setCursor(0, 2);
    lcd.print(line);
    strncpy(lcdBuffer[2], line, sizeof(lcdBuffer[2]) - 1);
  }

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