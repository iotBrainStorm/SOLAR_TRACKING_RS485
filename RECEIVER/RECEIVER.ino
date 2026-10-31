#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiManager.h>        // To save wifi password, instead of hard coding
#include <ESPAsyncWebServer.h>  // For web server
#include <HTTPClient.h>         // Node-Red
#include <Wire.h>               // I2C communication
#include <SPI.h>
#include <SPIFFS.h>          // File system
#include <HardwareSerial.h>  // Serial communication
#include "time.h"            // Time management
#include <U8g2lib.h>         // OLED display
#include <Preferences.h>     // To store users' settings
#include <Adafruit_AHT10.h>  // For temperature, humidity
#include <Dusk2Dawn.h>       // For Sunset & Sunrise
#include <esp_task_wdt.h>    // Hardware watchdog timer

// -- RS485 Setup
#define RXD2 16
#define TXD2 17
#define RS485_EN 4
#define LED_PIN 2
#define BUZZER 32
HardwareSerial rs485(1);
unsigned long lastModbusPoll = 0;
unsigned long lastValidModbus = 0;
unsigned long lastModbusReset = 0;
uint8_t modbusFailCount = 0;
const uint8_t MODBUS_FAIL_LIMIT = 10;
bool sensorDataStale = false;

// -- NTC Setup
unsigned long lastNTCReadTime = 0;

// -- Global variables for settings (No heap String members -> cross-core safe)
Preferences preferences;
struct DeviceSettings {
  uint8_t tempPrecision;      // 0,1,2
  uint8_t humidityPrecision;  // 0,1,2
  uint16_t ahtInterval;       // >=1

  float ntcResistance;  // 100..65535 (fits 16-bit Modbus register)
  float betaConstant;
  float ntcOffset;
  uint16_t ntcInterval;

  uint8_t luxPercentageMode;  // 0 auto, 1 manual
  uint32_t maxLuxValue;
  uint32_t minLuxValue;
  uint16_t luxInterval;
  uint32_t luxUpdateThreshold;
  uint32_t luxSaveInterval;

  uint8_t enableNodeRed;  // 0 or 1
  char nodeRedIP[64];
  uint16_t nodeRedPort;
  uint16_t nodeRedInterval;

  uint8_t enableRS485;      // 0 = disabled, 1 = enabled
  uint8_t modbusDeviceID;   // 1-247
  uint32_t modbusBaudRate;  // 9600, 19200, 38400, 57600, 115200
  uint16_t modbusInterval;  // seconds (share interval)

  long gmtOffset;       // seconds
  uint8_t clockFormat;  // 12 or 24

  float latitude;
  float longitude;
  float tzOffset;  // timezone offset in hours (e.g. 5.5 for +5:30)

  char placeStart[12];
  char placeEnd[12];
};

DeviceSettings settings;
DeviceSettings stagedSettings;
portMUX_TYPE settingsMux = portMUX_INITIALIZER_UNLOCKED;

// -- Deferred Action Flags (Safe execution outside async_tcp thread)
volatile bool pendingSaveAndSync = false;
volatile bool pendingLuxReset = false;
volatile bool pendingFactoryReset = false;
uint8_t prevEnableRS485 = 0;
uint8_t prevSlaveDeviceID = 1;
uint32_t prevSlaveBaudRate = 115200;

// -- Feedback LED Setup
bool ledState = false;
unsigned long ledStartTime = 0;
unsigned long ledDuration = 50;
const unsigned long ledDurationRX = 60;   // receive blink
const unsigned long ledDurationTX = 120;  // transmit blink

// -- Buzzer Feedback
bool buzzerState = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 100;

// -- LCD Setup
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
unsigned long lastLCDUpdate = 0;
unsigned long lastLCDInitRefresh = 0;

// -- Temperature and Humidity sensor setup
Adafruit_AHT10 aht;
bool ahtConnected = false;
uint8_t ahtFailCount = 0;
const uint8_t AHT_FAIL_LIMIT = 5;
unsigned long lastAHTReadTime = 0;
unsigned long lastAHTReinitTime = 0;

// -- Sunset and Sunrise setup
int sunriseMinutes = -1;
int sunsetMinutes = -1;
char sunriseStr[6] = "--:--";
char sunsetStr[6] = "--:--";
bool ntpSynced = false;
int lastCalcDay = -1;

// -- Light sensor (LUX) setup
bool luxConnected = false;
unsigned long lastLuxReadTime = 0;
unsigned long lastLuxSaveTime = 0;
float luxFiltered = 0;  // EMA filtered value
bool luxInitialized = false;
float previousLux = 0;
long luxDiff = 0;

// -- Sensor Values Storage
float ntcTemp = 0.0f;
float ahtTemp = 0.0f;
float humidity = 0.0f;
uint32_t luxValue = 0;
uint8_t sunlightPercentage = 0;

// -- Serial Output
unsigned long lastSerialPrint = 0;

// -- Time Management
const char *ntpServer = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const int daylightOffset_sec = 0;
unsigned long lastNtpCheck = 0;

// -- Web Server
AsyncWebServer server(80);
bool webServerStarted = false;

// -- Node-Red
bool nodeRedConnected = false;
int httpResponseCode = 0;

// -- Welcome Message
const char MSG_WELCOME[] PROGMEM = "ESP";
const char MSG_SUBTITLE[] PROGMEM = "SOLAR - TEM";
const char MSG_DEVELOPER[] PROGMEM = "developed by M.Maity";

// Forward declarations
void calculateSunriseSunset();
void sendSettingsToSlave(uint8_t targetDeviceID, uint32_t targetBaudRate);

//////////////////////   SAFE HELPERS   //////////////////////

float safeJsonFloat(float v) {
  return (isnan(v) || isinf(v)) ? 0.0f : v;
}

void sanitizeSafeStr(char *str, size_t maxLen) {
  if (!str || maxLen == 0) return;
  str[maxLen - 1] = '\0';
  for (size_t i = 0; str[i] != '\0'; i++) {
    if (str[i] == '"' || str[i] == '\\' || (uint8_t)str[i] < 32) {
      str[i] = '\0';
      break;
    }
  }
}

//////////////////////   BLINK FEEDBACK LED   //////////////////////

void triggerLED(unsigned long duration) {
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  ledStartTime = millis();
  ledDuration = duration;
}

void updateLED() {
  // Fast blink pattern when transmitter is not responding
  if (sensorDataStale) {
    static unsigned long lastStaleToggle = 0;
    if (millis() - lastStaleToggle >= 100) {  // 100ms on/off = rapid blink
      lastStaleToggle = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
    return;
  }

  // Normal single-blink for TX/RX feedback
  if (ledState && millis() - ledStartTime >= ledDuration) {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  }
}

//////////////////////   BLINK FEEDBACK BUZZER   //////////////////////

void triggerBuzzer(unsigned long duration) {
  digitalWrite(BUZZER, HIGH);
  buzzerState = true;
  buzzerStartTime = millis();
  buzzerDuration = duration;
}

void updateBuzzer() {
  if (buzzerState && millis() - buzzerStartTime >= buzzerDuration) {
    digitalWrite(BUZZER, LOW);
    buzzerState = false;
  }
}

//////////////////////   DEFAULT SETTINGS   //////////////////////

void setDefaultSettings(DeviceSettings &cfg) {
  cfg.tempPrecision = 1;
  cfg.humidityPrecision = 0;
  cfg.ahtInterval = 1;

  cfg.ntcResistance = 10000.0f;
  cfg.betaConstant = 3435.0f;
  cfg.ntcOffset = 0.0f;
  cfg.ntcInterval = 1;

  cfg.luxPercentageMode = 0;
  cfg.maxLuxValue = 100;
  cfg.minLuxValue = 1;
  cfg.luxInterval = 1;
  cfg.luxUpdateThreshold = 200;
  cfg.luxSaveInterval = 300;

  cfg.enableNodeRed = 0;
  cfg.nodeRedIP[0] = '\0';
  cfg.nodeRedPort = 1880;
  cfg.nodeRedInterval = 10;

  cfg.enableRS485 = 0;
  cfg.modbusDeviceID = 1;
  cfg.modbusBaudRate = 115200;
  cfg.modbusInterval = 2;

  cfg.gmtOffset = 19800;
  cfg.clockFormat = 24;

  cfg.latitude = 22.5726f;
  cfg.longitude = 88.3639f;
  cfg.tzOffset = 5.5f;

  cfg.placeStart[0] = '\0';
  cfg.placeEnd[0] = '\0';
}

//////////////////////   SANITIZE SETTINGS   //////////////////////

void sanitizeSettings(DeviceSettings &cfg) {
  if (cfg.tempPrecision > 2) cfg.tempPrecision = 1;
  if (cfg.humidityPrecision > 2) cfg.humidityPrecision = 0;
  if (cfg.ahtInterval < 1) cfg.ahtInterval = 1;
  if (cfg.ntcInterval < 1) cfg.ntcInterval = 1;

  if (isnan(cfg.ntcResistance) || cfg.ntcResistance < 100.0f || cfg.ntcResistance > 65535.0f)
    cfg.ntcResistance = 10000.0f;
  if (isnan(cfg.betaConstant) || cfg.betaConstant <= 0.0f || cfg.betaConstant > 65535.0f)
    cfg.betaConstant = 3435.0f;
  if (isnan(cfg.ntcOffset) || cfg.ntcOffset < -50.0f || cfg.ntcOffset > 50.0f)
    cfg.ntcOffset = 0.0f;

  if (cfg.luxPercentageMode > 1) cfg.luxPercentageMode = 0;
  if (cfg.luxInterval < 1) cfg.luxInterval = 1;
  if (cfg.luxUpdateThreshold < 10) cfg.luxUpdateThreshold = 10;
  if (cfg.luxSaveInterval < 300) cfg.luxSaveInterval = 300;
  if (cfg.luxPercentageMode == 1 && cfg.maxLuxValue <= cfg.minLuxValue) {
    cfg.maxLuxValue = cfg.minLuxValue + 100;
  }

  if (cfg.nodeRedPort == 0) cfg.nodeRedPort = 1880;
  if (cfg.nodeRedInterval < 1) cfg.nodeRedInterval = 1;

  if (cfg.modbusDeviceID < 1 || cfg.modbusDeviceID > 247) cfg.modbusDeviceID = 1;
  if (cfg.modbusBaudRate == 0) cfg.modbusBaudRate = 115200;
  if (cfg.modbusInterval < 1) cfg.modbusInterval = 1;

  if (cfg.clockFormat != 12 && cfg.clockFormat != 24) cfg.clockFormat = 24;
  if (isnan(cfg.latitude) || cfg.latitude < -90.0f || cfg.latitude > 90.0f) cfg.latitude = 22.5726f;
  if (isnan(cfg.longitude) || cfg.longitude < -180.0f || cfg.longitude > 180.0f) cfg.longitude = 88.3639f;
  if (isnan(cfg.tzOffset) || cfg.tzOffset < -12.0f || cfg.tzOffset > 14.0f) cfg.tzOffset = 5.5f;

  sanitizeSafeStr(cfg.nodeRedIP, sizeof(cfg.nodeRedIP));
  sanitizeSafeStr(cfg.placeStart, sizeof(cfg.placeStart));
  sanitizeSafeStr(cfg.placeEnd, sizeof(cfg.placeEnd));
}

//////////////////////   LOAD SETTINGS   //////////////////////

void loadSettings() {
  Serial.println("\n========== LOADING SETTINGS ==========");

  DeviceSettings loaded;
  setDefaultSettings(loaded);

  preferences.begin("device", true);  // read-only

  loaded.tempPrecision = preferences.getUChar("tPrec", 1);
  loaded.humidityPrecision = preferences.getUChar("hPrec", 0);
  loaded.ahtInterval = preferences.getUInt("ahtInt", 1);

  loaded.ntcResistance = preferences.getFloat("ntcR", 10000.0f);
  loaded.betaConstant = preferences.getFloat("beta", 3435.0f);
  loaded.ntcOffset = preferences.getFloat("ntcOff", 0.0f);
  loaded.ntcInterval = preferences.getUInt("ntcInt", 1);

  loaded.luxPercentageMode = preferences.getUChar("luxMode", 0);
  loaded.maxLuxValue = preferences.getULong("luxMax", 100);
  loaded.minLuxValue = preferences.getULong("luxMin", 1);
  loaded.luxInterval = preferences.getUInt("luxInt", 1);
  loaded.luxUpdateThreshold = preferences.getULong("luxThr", 200);
  loaded.luxSaveInterval = preferences.getULong("luxSave", 300);

  loaded.enableNodeRed = preferences.getUChar("nrEn", 0);
  if (preferences.isKey("nrIP")) {
    preferences.getString("nrIP", loaded.nodeRedIP, sizeof(loaded.nodeRedIP));
  }
  loaded.nodeRedPort = preferences.getUInt("nrPort", 1880);
  loaded.nodeRedInterval = preferences.getUInt("nrInt", 10);

  loaded.enableRS485 = preferences.getUChar("rsEn", 0);
  loaded.modbusDeviceID = preferences.getUChar("rsID", 1);
  loaded.modbusBaudRate = preferences.getULong("rsBaud", 115200);
  loaded.modbusInterval = preferences.getUInt("rsInt", 2);

  loaded.gmtOffset = preferences.getLong("gmt", 19800);
  loaded.clockFormat = preferences.getUChar("clkFmt", 24);

  loaded.latitude = preferences.getFloat("lat", 22.5726f);
  loaded.longitude = preferences.getFloat("lon", 88.3639f);
  loaded.tzOffset = preferences.getFloat("tzOff", 5.5f);

  if (preferences.isKey("plStart")) {
    preferences.getString("plStart", loaded.placeStart, sizeof(loaded.placeStart));
  }
  if (preferences.isKey("plEnd")) {
    preferences.getString("plEnd", loaded.placeEnd, sizeof(loaded.placeEnd));
  }

  preferences.end();

  sanitizeSettings(loaded);

  portENTER_CRITICAL(&settingsMux);
  settings = loaded;
  stagedSettings = loaded;
  portEXIT_CRITICAL(&settingsMux);

  prevEnableRS485 = settings.enableRS485;
  prevSlaveDeviceID = settings.modbusDeviceID;
  prevSlaveBaudRate = settings.modbusBaudRate;

  Serial.println("=======================================\n");
}

//////////////////////   SAVE SETTINGS   //////////////////////

void saveSettings() {
  portENTER_CRITICAL(&settingsMux);
  sanitizeSettings(settings);
  DeviceSettings snap = settings;
  portEXIT_CRITICAL(&settingsMux);

  preferences.begin("device", false);  // write mode

  preferences.putUChar("tPrec", snap.tempPrecision);
  preferences.putUChar("hPrec", snap.humidityPrecision);
  preferences.putUInt("ahtInt", snap.ahtInterval);

  preferences.putFloat("ntcR", snap.ntcResistance);
  preferences.putFloat("beta", snap.betaConstant);
  preferences.putFloat("ntcOff", snap.ntcOffset);
  preferences.putUInt("ntcInt", snap.ntcInterval);

  preferences.putUChar("luxMode", snap.luxPercentageMode);
  preferences.putULong("luxMax", snap.maxLuxValue);
  preferences.putULong("luxMin", snap.minLuxValue);
  preferences.putUInt("luxInt", snap.luxInterval);
  preferences.putULong("luxThr", snap.luxUpdateThreshold);
  preferences.putULong("luxSave", snap.luxSaveInterval);

  preferences.putUChar("nrEn", snap.enableNodeRed);
  preferences.putString("nrIP", snap.nodeRedIP);
  preferences.putUInt("nrPort", snap.nodeRedPort);
  preferences.putUInt("nrInt", snap.nodeRedInterval);

  preferences.putUChar("rsEn", snap.enableRS485);
  preferences.putUChar("rsID", snap.modbusDeviceID);
  preferences.putULong("rsBaud", snap.modbusBaudRate);
  preferences.putUInt("rsInt", snap.modbusInterval);

  preferences.putLong("gmt", snap.gmtOffset);
  preferences.putUChar("clkFmt", snap.clockFormat);

  preferences.putFloat("lat", snap.latitude);
  preferences.putFloat("lon", snap.longitude);
  preferences.putFloat("tzOff", snap.tzOffset);

  preferences.putString("plStart", snap.placeStart);
  preferences.putString("plEnd", snap.placeEnd);

  preferences.end();
  Serial.println("Settings Saved to NVS");
}

// Lightweight NVS helper so Auto-LUX calibration doesn't rewrite all 28 NVS keys
void saveLuxBounds() {
  portENTER_CRITICAL(&settingsMux);
  uint32_t maxL = settings.maxLuxValue;
  uint32_t minL = settings.minLuxValue;
  portEXIT_CRITICAL(&settingsMux);

  preferences.begin("device", false);
  preferences.putULong("luxMax", maxL);
  preferences.putULong("luxMin", minL);
  preferences.end();
}

//////////////////////   I2C RECOVERY & AHT RE-INIT   //////////////////////

void recoverI2CBus() {
  Serial.println("[I2C] Recovering I2C bus...");
  Wire.end();
  delay(5);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);

  // Clock SCL up to 16 times to release any slave holding SDA low
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }

  // Generate I2C STOP condition (SDA LOW -> SCL HIGH -> SDA HIGH)
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);

  Wire.begin();
  Wire.setTimeOut(100);  // 100ms timeout prevents I2C lockups

  // Re-sync OLED I2C interface without clearing screen content
  u8g2.initInterface();
  Serial.println("[I2C] Bus recovery complete");
}

void reinitAHT10() {
  Serial.println("[AHT10] Attempting sensor re-initialization...");
  recoverI2CBus();
  delay(20);
  if (aht.begin()) {
    ahtConnected = true;
    ahtFailCount = 0;
    Serial.println("[AHT10] Re-initialization successful!");
  } else {
    ahtConnected = false;
    Serial.println("[AHT10] Re-initialization failed.");
  }
}

//////////////////////   WIFI SETUP   //////////////////////

bool connectToSavedWiFi() {
  Serial.println("\n==============================");
  Serial.println("WiFi Connection Started");
  Serial.println("==============================");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Connecting WiFi...");
  u8g2.sendBuffer();

  WiFiManager wm;
  bool success = false;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin();

  const int MAX_ATTEMPTS = 5;
  for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
    char attemptStr[16];
    snprintf(attemptStr, sizeof(attemptStr), "Attempt: %d/5", attempts + 1);

    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();
    Serial.printf("[INFO] %s\n", attemptStr);

    // Poll every 250ms up to 2s per attempt so association is caught right through the end
    for (int step = 0; step < 8; step++) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[SUCCESS] Connected to Saved WiFi");
        Serial.printf("SSID       : %s\n", WiFi.SSID().c_str());
        Serial.printf("IP Address : %s\n", WiFi.localIP().toString().c_str());
        Serial.println("==============================\n");

        u8g2.clearBuffer();
        u8g2.drawStr(0, 12, "WiFi Connected!");
        u8g2.drawStr(0, 24, "SSID:");
        u8g2.drawStr(0, 36, WiFi.SSID().c_str());
        u8g2.drawStr(0, 48, "IP:");
        u8g2.drawStr(0, 60, WiFi.localIP().toString().c_str());
        u8g2.sendBuffer();

        delay(2000);
        return true;
      }
      delay(250);
    }
  }

  // Failed to connect -> launch Config Portal
  Serial.println("\n[WARNING] No saved WiFi found!");
  Serial.println("[INFO] Starting Config Portal...");

  u8g2.clearBuffer();
  u8g2.drawStr(0, 12, "No Saved WiFi!");
  u8g2.drawStr(0, 24, "Starting AP...");
  u8g2.drawStr(0, 36, "AP IP:");
  u8g2.drawStr(0, 48, "192.168.4.1");
  u8g2.drawStr(0, 60, "Connect & Setup");
  u8g2.sendBuffer();

  delay(1500);

  wm.setConfigPortalTimeout(60);
  success = wm.autoConnect("Solar Weather");

  if (success) {
    WiFi.setAutoReconnect(true);
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "WiFi Connected!");
    u8g2.drawStr(0, 24, "SSID:");
    u8g2.drawStr(0, 36, WiFi.SSID().c_str());
    u8g2.drawStr(0, 48, "IP:");
    u8g2.drawStr(0, 60, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer();

    delay(2000);
    return true;
  } else {
    Serial.println("\n[ERROR] Config Portal Timeout!");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Time over!");
    u8g2.sendBuffer();

    delay(1000);
    WiFi.mode(WIFI_STA);  // Return cleanly to STA mode for background reconnects
    return false;
  }
}

//////////////////////   TIME SETUP (BOOT & SILENT BACKGROUND)   //////////////////////

// Silent background NTP trigger - never blocks loop() or touches OLED
void syncNTPInBackground() {
  if (WiFi.status() == WL_CONNECTED) {
    configTime(settings.gmtOffset, daylightOffset_sec, ntpServer, ntpServer2);
    Serial.println("[NTP] Background SNTP sync configured/triggered.");
  }
}

// Uses year 2024 as fallback so >= 2025 checks only pass after genuine NTP sync
void applyFallbackTimeIfUnsynced() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10) || timeinfo.tm_year < (2024 - 1900)) {
    struct tm tm = { 0 };
    tm.tm_year = 2024 - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 12;
    tm.tm_min = 0;
    tm.tm_sec = 0;
    tm.tm_isdst = -1;

    time_t t = mktime(&tm);
    struct timeval now = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&now, nullptr);
    Serial.println("[TIME] Fallback time applied (01/01/2024 12:00:00).");
  }
}

// Used only once during setup() to display boot status on OLED
void configDateTimeBoot() {
  Serial.println("\n==============================");
  Serial.println("Date & Time Configuration");
  Serial.println("==============================");

  if (WiFi.status() != WL_CONNECTED) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 24, "No WiFi!");
    u8g2.drawStr(0, 36, "Time not synced!");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();

    delay(1500);
    applyFallbackTimeIfUnsynced();
    return;
  }

  configTime(settings.gmtOffset, daylightOffset_sec, ntpServer, ntpServer2);

  int attempts = 0;
  const int MAX_ATTEMPTS = 5;
  struct tm timeinfo;

  while (attempts < MAX_ATTEMPTS) {
    char attemptStr[16];
    snprintf(attemptStr, sizeof(attemptStr), "Attempt: %d/5", attempts + 1);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Syncing Time...");
    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();

    if (getLocalTime(&timeinfo, 1000) && timeinfo.tm_year >= (2025 - 1900)) {
      ntpSynced = true;
      Serial.println("[SUCCESS] Time synced from NTP server.");
      break;
    }
    attempts++;
  }

  if (ntpSynced && getLocalTime(&timeinfo, 10)) {
    char timeStr[24];
    char dateStr[24];
    char gmtStr[30];

    strftime(timeStr, sizeof(timeStr), "Time: %H:%M:%S", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "Date: %d.%m.%Y", &timeinfo);
    strftime(gmtStr, sizeof(gmtStr), "GMT: %z %Z", &timeinfo);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_14_tr);
    u8g2.drawStr(0, 16, timeStr);
    u8g2.drawStr(0, 32, dateStr);
    u8g2.drawStr(0, 62, gmtStr);
    u8g2.sendBuffer();
    delay(1500);
  } else {
    applyFallbackTimeIfUnsynced();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Time not synced!");
    u8g2.drawStr(0, 24, "Syncing in bg...");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();
    delay(1500);
  }
}

//////////////////////   SERVER SETUP   //////////////////////

void setupWebServer() {
  if (webServerStarted)
    return;  // Already started — never re-add handlers to avoid memory leaks!

  if (!SPIFFS.exists("/index.html.gz") || !SPIFFS.exists("/config.html.gz") || !SPIFFS.exists("/dashboard.svg.gz") || !SPIFFS.exists("/settings.svg.gz")) {
    Serial.println("[ERROR] Required .gz files not found in SPIFFS!");
    return;
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html.gz", "text/html");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/config.html.gz", "text/html");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/dashboard.svg", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/dashboard.svg.gz", "image/svg+xml");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  server.on("/settings.svg", HTTP_GET, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/settings.svg.gz", "image/svg+xml");
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  // Zero-heap stack-allocated JSON response
  server.on("/sensor.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    portENTER_CRITICAL(&settingsMux);
    uint8_t tP = settings.tempPrecision;
    uint8_t hP = settings.humidityPrecision;
    float lat = settings.latitude;
    float lon = settings.longitude;
    float tz = settings.tzOffset;
    float sNtc = safeJsonFloat(ntcTemp);
    float sAht = safeJsonFloat(ahtTemp);
    float sHum = safeJsonFloat(humidity);
    uint32_t sLux = luxValue;
    uint8_t sSun = sunlightPercentage;
    bool sStale = sensorDataStale;
    int srMin = sunriseMinutes;
    int ssMin = sunsetMinutes;
    char srBuf[6], ssBuf[6];
    memcpy(srBuf, sunriseStr, sizeof(srBuf));
    memcpy(ssBuf, sunsetStr, sizeof(ssBuf));
    portEXIT_CRITICAL(&settingsMux);

    char json[384];
    snprintf(json, sizeof(json),
             "{\"ntcTemp\":%.*f,\"ahtTemp\":%.*f,\"humidity\":%.*f,"
             "\"lux\":%lu,\"sunlight\":%u,\"tempPrecision\":%u,\"humPrecision\":%u,"
             "\"sunrise\":\"%s\",\"sunset\":\"%s\",\"sunriseMin\":%d,\"sunsetMin\":%d,"
             "\"latitude\":%.4f,\"longitude\":%.4f,\"tzOffset\":%.2f,\"sensorStale\":%s}",
             tP, sNtc, tP, sAht, hP, sHum,
             (unsigned long)sLux, sSun, tP, hP,
             srBuf, ssBuf, srMin, ssMin,
             safeJsonFloat(lat), safeJsonFloat(lon), safeJsonFloat(tz),
             sStale ? "true" : "false");

    request->send(200, "application/json", json);
  });

  // Thread-safe staged save (never mutates active settings directly on Core 0)
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    DeviceSettings next;
    portENTER_CRITICAL(&settingsMux);
    next = pendingSaveAndSync ? stagedSettings : settings;
    portEXIT_CRITICAL(&settingsMux);

    // -------- TEMPERATURE --------
    if (request->hasParam("tempPrecision", true))
      next.tempPrecision = request->getParam("tempPrecision", true)->value().toInt();
    if (request->hasParam("humidityPrecision", true))
      next.humidityPrecision = request->getParam("humidityPrecision", true)->value().toInt();
    if (request->hasParam("ahtInterval", true))
      next.ahtInterval = request->getParam("ahtInterval", true)->value().toInt();

    // -------- NTC --------
    if (request->hasParam("ntcResistance", true))
      next.ntcResistance = request->getParam("ntcResistance", true)->value().toFloat();
    if (request->hasParam("betaConstant", true))
      next.betaConstant = request->getParam("betaConstant", true)->value().toFloat();
    if (request->hasParam("ntcOffset", true))
      next.ntcOffset = request->getParam("ntcOffset", true)->value().toFloat();
    if (request->hasParam("ntcInterval", true))
      next.ntcInterval = request->getParam("ntcInterval", true)->value().toInt();

    // -------- LUX --------
    uint8_t previousLuxMode = next.luxPercentageMode;
    if (request->hasParam("luxMode", true))
      next.luxPercentageMode = request->getParam("luxMode", true)->value().toInt();

    if (previousLuxMode == 1 && next.luxPercentageMode == 0) {
      next.maxLuxValue = 0;
      next.minLuxValue = 0;
    } else {
      if (request->hasParam("maxLuxValue", true))
        next.maxLuxValue = request->getParam("maxLuxValue", true)->value().toInt();
      if (request->hasParam("minLuxValue", true))
        next.minLuxValue = request->getParam("minLuxValue", true)->value().toInt();
    }
    if (request->hasParam("luxInterval", true))
      next.luxInterval = request->getParam("luxInterval", true)->value().toInt();
    if (request->hasParam("luxUpdateThreshold", true))
      next.luxUpdateThreshold = request->getParam("luxUpdateThreshold", true)->value().toInt();
    if (request->hasParam("luxSaveInterval", true))
      next.luxSaveInterval = request->getParam("luxSaveInterval", true)->value().toInt();

    // -------- NODE RED --------
    next.enableNodeRed = request->hasParam("enableNodeRed", true) ? 1 : 0;
    if (request->hasParam("nodeRedIP", true)) {
      strlcpy(next.nodeRedIP, request->getParam("nodeRedIP", true)->value().c_str(), sizeof(next.nodeRedIP));
    }
    if (request->hasParam("nodeRedPort", true))
      next.nodeRedPort = request->getParam("nodeRedPort", true)->value().toInt();
    if (request->hasParam("nodeRedInterval", true))
      next.nodeRedInterval = request->getParam("nodeRedInterval", true)->value().toInt();

    // -------- RS485 --------
    next.enableRS485 = request->hasParam("enableRS485", true) ? 1 : 0;
    if (request->hasParam("modbusDeviceID", true))
      next.modbusDeviceID = request->getParam("modbusDeviceID", true)->value().toInt();
    if (request->hasParam("modbusBaudRate", true))
      next.modbusBaudRate = request->getParam("modbusBaudRate", true)->value().toInt();
    if (request->hasParam("modbusInterval", true))
      next.modbusInterval = request->getParam("modbusInterval", true)->value().toInt();

    // -------- CLOCK & LOCATION --------
    if (request->hasParam("clockFormat", true))
      next.clockFormat = request->getParam("clockFormat", true)->value().toInt();
    if (request->hasParam("latitude", true))
      next.latitude = request->getParam("latitude", true)->value().toFloat();
    if (request->hasParam("longitude", true))
      next.longitude = request->getParam("longitude", true)->value().toFloat();

    // Parse tzOffset first, then let explicit gmtOffset take precedence so custom offsets survive
    if (request->hasParam("tzOffset", true)) {
      next.tzOffset = request->getParam("tzOffset", true)->value().toFloat();
    }
    if (request->hasParam("gmtOffset", true) && request->getParam("gmtOffset", true)->value().length() > 0) {
      next.gmtOffset = request->getParam("gmtOffset", true)->value().toInt();
      next.tzOffset = next.gmtOffset / 3600.0f;
    } else {
      next.gmtOffset = (long)round(next.tzOffset * 3600.0f);
    }

    // -------- PLACEMENT DATE RANGE --------
    if (request->hasParam("placeStart", true)) {
      strlcpy(next.placeStart, request->getParam("placeStart", true)->value().c_str(), sizeof(next.placeStart));
    }
    if (request->hasParam("placeEnd", true)) {
      strlcpy(next.placeEnd, request->getParam("placeEnd", true)->value().c_str(), sizeof(next.placeEnd));
    }

    sanitizeSettings(next);

    portENTER_CRITICAL(&settingsMux);
    stagedSettings = next;
    pendingSaveAndSync = true;  // Handled safely in loop() on Core 1
    portEXIT_CRITICAL(&settingsMux);

    request->send(200, "text/plain", "Settings Saved");
  });

  server.on("/settings.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    DeviceSettings snap;
    portENTER_CRITICAL(&settingsMux);
    snap = settings;
    portEXIT_CRITICAL(&settingsMux);

    char json[768];
    snprintf(json, sizeof(json),
             "{\"tempPrecision\":%u,\"humidityPrecision\":%u,\"ahtInterval\":%u,"
             "\"ntcResistance\":%lu,\"betaConstant\":%lu,\"ntcOffset\":%.2f,\"ntcInterval\":%u,"
             "\"luxMode\":%u,\"maxLuxValue\":%lu,\"minLuxValue\":%lu,\"luxInterval\":%u,"
             "\"luxUpdateThreshold\":%lu,\"luxSaveInterval\":%lu,"
             "\"enableNodeRed\":%u,\"nodeRedIP\":\"%s\",\"nodeRedPort\":%u,\"nodeRedInterval\":%u,"
             "\"enableRS485\":%u,\"modbusDeviceID\":%u,\"modbusBaudRate\":%lu,\"modbusInterval\":%u,"
             "\"gmtOffset\":%ld,\"clockFormat\":%u,"
             "\"latitude\":%.6f,\"longitude\":%.6f,\"tzOffset\":%.2f,"
             "\"placeStart\":\"%s\",\"placeEnd\":\"%s\"}",
             snap.tempPrecision, snap.humidityPrecision, snap.ahtInterval,
             (unsigned long)round(snap.ntcResistance), (unsigned long)round(snap.betaConstant),
             safeJsonFloat(snap.ntcOffset), snap.ntcInterval,
             snap.luxPercentageMode, (unsigned long)snap.maxLuxValue, (unsigned long)snap.minLuxValue, snap.luxInterval,
             (unsigned long)snap.luxUpdateThreshold, (unsigned long)snap.luxSaveInterval,
             snap.enableNodeRed, snap.nodeRedIP, snap.nodeRedPort, snap.nodeRedInterval,
             snap.enableRS485, snap.modbusDeviceID, (unsigned long)snap.modbusBaudRate, snap.modbusInterval,
             snap.gmtOffset, snap.clockFormat,
             safeJsonFloat(snap.latitude), safeJsonFloat(snap.longitude), safeJsonFloat(snap.tzOffset),
             snap.placeStart, snap.placeEnd);

    request->send(200, "application/json", json);
  });

  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    pendingFactoryReset = true;  // Handled safely in loop()
    request->send(200, "text/plain", "Factory Reset Successful");
  });

  server.on("/reset-lux", HTTP_POST, [](AsyncWebServerRequest *request) {
    pendingLuxReset = true;  // Handled safely in loop()
    request->send(200, "text/plain", "LUX Settings Reset Successfully!");
  });

  server.begin();
  webServerStarted = true;
}

//////////////////////   WIFI & SILENT NTP MONITOR   //////////////////////

void checkWiFiAndStartServer() {
  static unsigned long lastCheck = 0;
  static bool wasConnected = (WiFi.status() == WL_CONNECTED);
  static unsigned long lastReconnectAttempt = 0;

  if (millis() - lastCheck < 3000)
    return;  // Check every 3s
  lastCheck = millis();

  bool isConnected = (WiFi.status() == WL_CONNECTED);

  // WiFi Just Connected (or Reconnected)
  if (isConnected && !wasConnected) {
    Serial.println("\n[WiFi] Connected!");
    Serial.printf("[WiFi] SSID: %s | IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    if (!webServerStarted) {
      Serial.println("[WiFi] Starting WebServer...");
      setupWebServer();
    }

    syncNTPInBackground();
  }

  // WiFi Lost
  if (!isConnected && wasConnected) {
    Serial.println("\n[WiFi] Disconnected! WebServer remains bound for reconnect.");
  }

  // Attempt non-blocking Reconnect every 15s
  if (!isConnected) {
    if (millis() - lastReconnectAttempt > 15000UL) {
      lastReconnectAttempt = millis();
      Serial.println("[WiFi] Attempting non-blocking reconnection...");
      WiFi.reconnect();
    }
  }

  // Confirm genuine background NTP sync (year >= 2025 ignores 2024 fallback time)
  if (isConnected && !ntpSynced && (millis() - lastNtpCheck > 5000UL)) {
    lastNtpCheck = millis();
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5) && timeinfo.tm_year >= (2025 - 1900)) {
      ntpSynced = true;
      Serial.println("[NTP] Background time sync confirmed!");
      calculateSunriseSunset();
    }
  }

  wasConnected = isConnected;
}

//////////////////////   PROCESS DEFERRED WEB ACTIONS   //////////////////////

void processPendingWebActions() {
  if (pendingSaveAndSync) {
    portENTER_CRITICAL(&settingsMux);
    settings = stagedSettings;
    pendingSaveAndSync = false;
    portEXIT_CRITICAL(&settingsMux);

    saveSettings();

    if (settings.enableRS485) {
      if (!prevEnableRS485) {
        // Freshly enabled RS485: reset timers so stale/watchdog don't falsely fire
        lastValidModbus = millis();
        lastModbusReset = millis();
        lastModbusPoll = 0;
        modbusFailCount = 0;
        sensorDataStale = false;
      }
      sendSettingsToSlave(prevSlaveDeviceID, prevSlaveBaudRate);
    } else {
      rs485.updateBaudRate(settings.modbusBaudRate);
      prevSlaveDeviceID = settings.modbusDeviceID;
      prevSlaveBaudRate = settings.modbusBaudRate;
      sensorDataStale = false;
      modbusFailCount = 0;
    }
    prevEnableRS485 = settings.enableRS485;

    syncNTPInBackground();
    calculateSunriseSunset();
    triggerLED(100);
    triggerBuzzer(100);
    Serial.println("[SYSTEM] Settings Saved & Synced Successfully");
  }

  if (pendingLuxReset) {
    pendingLuxReset = false;

    portENTER_CRITICAL(&settingsMux);
    settings.luxPercentageMode = 0;  // Auto
    settings.maxLuxValue = 0;        // 0 triggers auto-init on next sample
    settings.minLuxValue = 0;
    settings.luxInterval = 1;
    settings.luxUpdateThreshold = 200;
    settings.luxSaveInterval = 300;
    portEXIT_CRITICAL(&settingsMux);

    luxInitialized = false;  // Re-seed EMA filter cleanly on next sample

    saveSettings();
    if (settings.enableRS485) {
      sendSettingsToSlave(prevSlaveDeviceID, prevSlaveBaudRate);
    }
    triggerLED(100);
    triggerBuzzer(100);
    Serial.println("[LUX] Settings Reset & Saved to NVS");
  }

  if (pendingFactoryReset) {
    pendingFactoryReset = false;
    Serial.println("\n===== EXECUTING FACTORY RESET =====");
    preferences.begin("device", false);
    preferences.clear();
    preferences.end();

    DeviceSettings defaults;
    setDefaultSettings(defaults);
    portENTER_CRITICAL(&settingsMux);
    settings = defaults;
    stagedSettings = defaults;
    portEXIT_CRITICAL(&settingsMux);

    saveSettings();
    if (prevEnableRS485) {
      sendSettingsToSlave(prevSlaveDeviceID, prevSlaveBaudRate);
    }
    triggerLED(100);
    triggerBuzzer(100);

    // Wait 500ms while feeding WDT + updating buzzer/LED so HTTP finishes before reboot
    unsigned long t = millis();
    while (millis() - t < 500) {
      esp_task_wdt_reset();
      updateLED();
      updateBuzzer();
      delay(10);
    }
    ESP.restart();
  }
}

//////////////////////   WELCOME MESSAGE   //////////////////////

void welcomeMsg() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(0, 22, MSG_WELCOME);
  u8g2.setFont(u8g2_font_ncenR12_tr);
  u8g2.drawStr(0, 40, MSG_SUBTITLE);
  u8g2.setFont(u8g2_font_t0_11_tr);
  u8g2.drawStr(2, 60, MSG_DEVELOPER);
  u8g2.sendBuffer();
}

void drawCenteredStr(int y, const char *str, const uint8_t *font) {
  u8g2.setFont(font);
  int16_t strWidth = u8g2.getStrWidth(str);
  int16_t x = (128 - strWidth) / 2;
  u8g2.drawStr(x, y, str);
}

//////////////////////   SUNRISE & SUNSET CALCULATION   //////////////////////

void calculateSunriseSunset() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10)) {
    Serial.println("[SUN] Cannot calculate - time not available");
    return;
  }

  int tzHours = (int)settings.tzOffset;
  int tzRemainder = (int)round((settings.tzOffset - tzHours) * 60.0f);

  Dusk2Dawn location(settings.latitude, settings.longitude, tzHours);

  int year = timeinfo.tm_year + 1900;
  int month = timeinfo.tm_mon + 1;
  int day = timeinfo.tm_mday;

  int sr = location.sunrise(year, month, day, false);
  int ss = location.sunset(year, month, day, false);

  int newSrMin = -1;
  int newSsMin = -1;
  char newSrStr[6] = "--:--";
  char newSsStr[6] = "--:--";

  if (sr >= 0 && ss >= 0) {
    newSrMin = (sr + tzRemainder + 1440) % 1440;
    newSsMin = (ss + tzRemainder + 1440) % 1440;
    snprintf(newSrStr, sizeof(newSrStr), "%02d:%02d", newSrMin / 60, newSrMin % 60);
    snprintf(newSsStr, sizeof(newSsStr), "%02d:%02d", newSsMin / 60, newSsMin % 60);
  }

  portENTER_CRITICAL(&settingsMux);
  sunriseMinutes = newSrMin;
  sunsetMinutes = newSsMin;
  memcpy(sunriseStr, newSrStr, sizeof(sunriseStr));
  memcpy(sunsetStr, newSsStr, sizeof(sunsetStr));
  portEXIT_CRITICAL(&settingsMux);

  lastCalcDay = day;
}

//////////////////////   CALCULATE PRECISION   //////////////////////

float applyPrecision(float value, uint8_t precision) {
  if (isnan(value) || isinf(value)) return 0.0f;
  if (precision == 0)
    return round(value);
  if (precision == 1)
    return round(value * 10.0f) / 10.0f;
  if (precision == 2)
    return round(value * 100.0f) / 100.0f;
  return value;
}

void handleNTC() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastNTCReadTime >= settings.ntcInterval * 1000UL) {
    lastNTCReadTime = currentMillis;
    ntcTemp = applyPrecision(ntcTemp, settings.tempPrecision);
  }
}

//////////////////////   AHT10 WITH I2C FALLBACK   //////////////////////

void handleAHT() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastAHTReadTime < settings.ahtInterval * 1000UL)
    return;
  lastAHTReadTime = currentMillis;

  if (ahtConnected) {
    sensors_event_t humidityEvent, tempEvent;
    if (aht.getEvent(&humidityEvent, &tempEvent) && !isnan(tempEvent.temperature) && !isnan(humidityEvent.relative_humidity)) {
      ahtFailCount = 0;
      ahtTemp = applyPrecision(tempEvent.temperature, settings.tempPrecision);
      humidity = applyPrecision(humidityEvent.relative_humidity, settings.humidityPrecision);
    } else {
      ahtFailCount++;
      Serial.printf("[AHT10] Read failed (%u/%u)\n", ahtFailCount, AHT_FAIL_LIMIT);
      if (ahtFailCount >= AHT_FAIL_LIMIT) {
        reinitAHT10();
      }
    }
  } else {
    // Periodically retry initializing disconnected AHT10 every 30s
    if (currentMillis - lastAHTReinitTime >= 30000UL) {
      lastAHTReinitTime = currentMillis;
      reinitAHT10();
    }
  }
}

//////////////////////   CALCULATE LUX   //////////////////////

void handleLUX() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastLuxReadTime < settings.luxInterval * 1000UL)
    return;
  lastLuxReadTime = currentMillis;

  if (!luxInitialized) {
    luxFiltered = (float)luxValue;
    previousLux = luxFiltered;
    luxInitialized = true;
  } else {
    luxFiltered = (luxFiltered * 0.6f) + (luxValue * 0.4f);
  }

  luxDiff = (long)(luxFiltered - previousLux);
  previousLux = luxFiltered;

  // ====================== AUTO MODE =========================
  if (settings.luxPercentageMode == 0) {
    uint32_t threshold = settings.luxUpdateThreshold;
    uint32_t saveInterval = settings.luxSaveInterval * 1000UL;
    bool updated = false;

    portENTER_CRITICAL(&settingsMux);
    // Initialize min/max on first valid reading or after reset
    if ((settings.maxLuxValue <= 1 && settings.minLuxValue <= 1) && luxFiltered > 0.0f) {
      settings.maxLuxValue = (uint32_t)luxFiltered + 10;
      settings.minLuxValue = (uint32_t)luxFiltered;
      updated = true;
    }

    // Update MAX
    if (luxFiltered > (float)(settings.maxLuxValue + threshold)) {
      settings.maxLuxValue = (uint32_t)luxFiltered;
      updated = true;
    }

    // Update MIN (allows night readings near 0 lux without getting trapped at threshold=200)
    if (luxFiltered < (float)settings.minLuxValue && (((float)settings.minLuxValue - luxFiltered >= (float)threshold) || luxFiltered <= 5.0f)) {
      settings.minLuxValue = (uint32_t)luxFiltered;
      updated = true;
    }
    portEXIT_CRITICAL(&settingsMux);

    // NVS Save Protection (writes only luxMin & luxMax)
    if (updated && (currentMillis - lastLuxSaveTime > saveInterval)) {
      saveLuxBounds();
      lastLuxSaveTime = currentMillis;
      Serial.println("[LUX] AUTO: Min/Max updated & saved");
    }
  }

  // ================== PERCENTAGE CALC =======================
  uint32_t minLux = settings.minLuxValue;
  uint32_t maxLux = settings.maxLuxValue;

  if (maxLux > minLux) {
    float percent = ((luxFiltered - (float)minLux) * 100.0f) / (float)(maxLux - minLux);
    percent = constrain(percent, 0.0f, 100.0f);
    sunlightPercentage = (uint8_t)(percent + 0.5f);
  } else {
    sunlightPercentage = 0;
  }
}

//////////////////////   RS485 CONTROL   //////////////////////

void setTransmitMode() {
  digitalWrite(RS485_EN, HIGH);
}
void setReceiveMode() {
  digitalWrite(RS485_EN, LOW);
}

void resetRS485UART() {
  Serial.println("[RS485] Resetting UART...");
  rs485.flush();
  while (rs485.available())
    rs485.read();
  setReceiveMode();
  delay(5);
  rs485.end();
  delay(10);
  rs485.begin(settings.modbusBaudRate, SERIAL_8N1, RXD2, TXD2);
  rs485.setTimeout(2);
  setReceiveMode();
  modbusFailCount = 0;
  Serial.println("[RS485] UART reset complete");
}

//////////////////////   NODE RED SHARE   //////////////////////

void sendDataToNodeRed() {
  static unsigned long lastUpdate = 0;

  // Let any active 100ms buzzer/LED pulse finish cleanly before a potentially blocking HTTP call
  if (buzzerState || (ledState && !sensorDataStale))
    return;

  if (millis() - lastUpdate < (settings.nodeRedInterval * 1000UL))
    return;
  lastUpdate = millis();

  if (!settings.enableNodeRed || strlen(settings.nodeRedIP) < 7) {
    nodeRedConnected = false;
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    nodeRedConnected = false;
    return;
  }

  char url[112];
  snprintf(url, sizeof(url), "http://%s:%u/solar-data", settings.nodeRedIP, settings.nodeRedPort);

  WiFiClient client;
  client.setTimeout(1500);  // Short 1.5s LAN timeout prevents long loop() stalls

  HTTPClient http;
  http.setConnectTimeout(1500);
  http.setTimeout(1500);
  http.setReuse(false);

  if (!http.begin(client, url)) {
    nodeRedConnected = false;
    return;
  }

  http.addHeader("Content-Type", "application/json");

  char jsonPayload[200];
  snprintf(jsonPayload, sizeof(jsonPayload),
           "{\"ntcTemp\":%.*f,"
           "\"ahtTemp\":%.*f,"
           "\"humidity\":%.*f,"
           "\"lux\":%.1f,"
           "\"sunlight\":%.1f}",
           settings.tempPrecision, safeJsonFloat(ntcTemp),
           settings.tempPrecision, safeJsonFloat(ahtTemp),
           settings.humidityPrecision, safeJsonFloat(humidity),
           (float)luxValue,
           (float)sunlightPercentage);

  httpResponseCode = http.POST((uint8_t *)jsonPayload, strlen(jsonPayload));
  nodeRedConnected = (httpResponseCode == 200);

  http.end();
}

//////////////////////   SERIAL OUTPUT   //////////////////////

void serialOutput() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialPrint < 2000)
    return;
  lastSerialPrint = currentMillis;

  Serial.println("\n========================================");
  Serial.println("           Sensor Data Report");
  Serial.println("========================================");
  Serial.printf("NTC Temperature  : %.2f °C\n", ntcTemp);
  Serial.printf("AHT Temperature  : %.2f °C\n", ahtTemp);
  Serial.printf("Humidity         : %.2f %%\n", humidity);
  Serial.println("----------------------------------------");
  Serial.printf("Light Intensity  : %lu lux\n", (unsigned long)luxValue);
  Serial.printf("Filtered Lux     : %.2f lux\n", luxFiltered);
  Serial.printf("Sunlight Level   : %u %%\n", sunlightPercentage);
  Serial.printf("Sensor Stale     : %s\n", sensorDataStale ? "YES" : "No");
  Serial.println("----------------------------------------");
  Serial.printf("Sunrise          : %s\n", sunriseStr);
  Serial.printf("Sunset           : %s\n", sunsetStr);
  Serial.println("========================================\n");
}

//////////////////////   DISPLAY LCD (NO GHOST PIXELS)   //////////////////////

void displayLCD() {
  unsigned long nowMs = millis();

  // Throttle OLED redraws to 250ms (4 FPS) to prevent I2C bus saturation & SH1106 page corruption
  if (nowMs - lastLCDUpdate < 250)
    return;
  lastLCDUpdate = nowMs;

  // Periodically refresh SH1106 controller registers every 60s to clear any EMI state glitches
  if (nowMs - lastLCDInitRefresh > 60000UL) {
    lastLCDInitRefresh = nowMs;
    u8g2.initInterface();
  }

  u8g2.clearBuffer();
  u8g2.setFontMode(0);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_6x12_tf);

  // Draw divider lines
  u8g2.drawHLine(0, 0, 128);
  u8g2.drawVLine(50, 0, 40);
  u8g2.drawHLine(0, 40, 128);

  // ================= RIGHT COLUMN (Clipped to x=52..127, y=1..39) =================
  u8g2.setClipWindow(52, 1, 127, 39);

  struct tm timeinfo;
  bool hasTime = getLocalTime(&timeinfo, 5);
  char timeStr[16] = "--:-- -----";
  if (hasTime) {
    if (settings.clockFormat == 12) {
      strftime(timeStr, sizeof(timeStr), "%I:%M %b%e", &timeinfo);
    } else {
      strftime(timeStr, sizeof(timeStr), "%H:%M %b%e", &timeinfo);
    }
  }
  u8g2.drawStr(53, 12, timeStr);

  char sunRiseSetStr[16];
  snprintf(sunRiseSetStr, sizeof(sunRiseSetStr), "%s %s", sunriseStr, sunsetStr);
  u8g2.drawStr(53, 24, sunRiseSetStr);

  char noonRemStr[16] = "--:-- --h--m";
  if (sunriseMinutes >= 0 && sunsetMinutes > sunriseMinutes) {
    int noonMin = (sunriseMinutes + sunsetMinutes) / 2;
    int noonH = (noonMin / 60) % 24;
    int noonM = noonMin % 60;

    int remMin = 0;
    if (hasTime) {
      int nowMin = timeinfo.tm_hour * 60 + timeinfo.tm_min;
      if (nowMin < sunriseMinutes) {
        remMin = sunsetMinutes - sunriseMinutes;
      } else if (nowMin < sunsetMinutes) {
        remMin = sunsetMinutes - nowMin;
      } else {
        remMin = 0;
      }
    }
    remMin = constrain(remMin, 0, 1440);
    int remH = remMin / 60;
    int remM = remMin % 60;

    snprintf(noonRemStr, sizeof(noonRemStr), "%02d:%02d %dh%02dm", noonH, noonM, remH, remM);
  }
  u8g2.drawStr(53, 36, noonRemStr);

  // ================= LEFT COLUMN (Clipped to x=0..49, y=1..39) =================
  u8g2.setClipWindow(0, 1, 49, 39);

  char ntcStr[16];
  snprintf(ntcStr, sizeof(ntcStr), "S:%.*f", settings.tempPrecision, safeJsonFloat(ntcTemp));
  u8g2.drawStr(0, 12, ntcStr);

  char ahtStr[16];
  snprintf(ahtStr, sizeof(ahtStr), "R:%.*f", settings.tempPrecision, safeJsonFloat(ahtTemp));
  u8g2.drawStr(0, 24, ahtStr);

  char humStr[16];
  snprintf(humStr, sizeof(humStr), "H:%.*f", settings.humidityPrecision, safeJsonFloat(humidity));
  u8g2.drawStr(0, 36, humStr);

  // Restore full screen clip window for bottom section
  u8g2.setMaxClipWindow();

  // ================= ROW 4: LUX, DIFF & WIFI BARS =================
  char luxStr[16];
  snprintf(luxStr, sizeof(luxStr), "LX: %lu", (unsigned long)luxValue);
  u8g2.drawStr(0, 52, luxStr);

  char diffStr[12];
  long clampedDiff = constrain(luxDiff, -99999L, 99999L);
  if (clampedDiff > 0)
    snprintf(diffStr, sizeof(diffStr), "+%ld", clampedDiff);
  else
    snprintf(diffStr, sizeof(diffStr), "%ld", clampedDiff);
  u8g2.drawStr(74, 52, diffStr);

  if (WiFi.status() == WL_CONNECTED) {
    int rssi = WiFi.RSSI();
    int wifiSignal = constrain(2 * (rssi + 100), 0, 100);
    int activeBars = (wifiSignal * 8 + 99) / 100;

    const uint8_t baseX = 120;
    const uint8_t baseY = 52;

    for (int i = 0; i < activeBars; i++) {
      uint8_t barHeight = i + 1;
      u8g2.drawVLine(baseX + i, baseY - barHeight + 1, barHeight);
    }

    if (settings.enableNodeRed && nodeRedConnected && httpResponseCode == 200) {
      u8g2.drawBox(baseX, baseY - 7, 2, 2);
    }
  }

  // ================= ROW 5: SUNLIGHT % PROGRESS BAR (y <= 63) =================
  char sunStr[16];
  snprintf(sunStr, sizeof(sunStr), "SUN: %u%%", sunlightPercentage);
  u8g2.drawStr(0, 63, sunStr);

  int barWidth = map(constrain(sunlightPercentage, 0, 100), 0, 100, 0, 60);
  u8g2.drawFrame(64, 55, 64, 8);
  if (barWidth > 0) {
    u8g2.drawBox(66, 57, barWidth, 4);
  }

  u8g2.sendBuffer();
}

//////////////////////   RS485 MODBUS MASTER   //////////////////////

uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void sendModbusRequest(uint8_t id, uint16_t reg, uint16_t count) {
  while (rs485.available())
    rs485.read();
  delayMicroseconds(200);

  uint8_t frame[8];
  frame[0] = id;
  frame[1] = 0x03;
  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;
  frame[4] = count >> 8;
  frame[5] = count & 0xFF;

  uint16_t crc = modbusCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = crc >> 8;

  setTransmitMode();
  delayMicroseconds(150);
  triggerLED(ledDurationTX);
  rs485.write(frame, 8);
  rs485.flush();
  delayMicroseconds(500);
  setReceiveMode();
  delayMicroseconds(150);
  while (rs485.available())
    rs485.read();  // Discard TX->RX transceiver switching glitch
}

void sendSettingsToSlave(uint8_t targetDeviceID, uint32_t targetBaudRate) {
  uint32_t safeBaud = (targetBaudRate > 0) ? targetBaudRate : 115200;

  // Ensure Master transmits at the Slave's current baud rate first
  if (safeBaud != settings.modbusBaudRate) {
    rs485.updateBaudRate(safeBaud);
    delay(10);
  }

  uint8_t frame[32];
  frame[0] = targetDeviceID;  // Address Slave by its active ID
  frame[1] = 0x10;            // Write Multiple Registers
  frame[2] = 0x00;
  frame[3] = 0x00;
  frame[4] = 0x00;
  frame[5] = 0x09;  // 9 registers
  frame[6] = 18;    // 18 bytes

  int idx = 7;
  uint16_t val;

  val = (uint16_t)constrain(round(settings.ntcResistance), 100.0f, 65535.0f);
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = (uint16_t)constrain(round(settings.betaConstant), 1.0f, 65535.0f);
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = (uint16_t)((int16_t)round(settings.ntcOffset * 100.0f));
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.ntcInterval;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.luxInterval;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.modbusDeviceID;  // New Slave ID inside payload
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = (uint16_t)(settings.modbusBaudRate >> 16);
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = (uint16_t)(settings.modbusBaudRate & 0xFFFF);
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.modbusInterval;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  uint16_t crc = modbusCRC(frame, idx);
  frame[idx++] = crc & 0xFF;
  frame[idx++] = crc >> 8;

  while (rs485.available())
    rs485.read();

  setTransmitMode();
  delayMicroseconds(150);
  triggerLED(ledDurationTX);
  rs485.write(frame, idx);
  rs485.flush();
  delayMicroseconds(500);
  setReceiveMode();
  delayMicroseconds(150);
  while (rs485.available())
    rs485.read();

  Serial.println("[MODBUS] Settings sent to slave");

  uint8_t buffer[32];
  int rIdx = 0;
  unsigned long startTime = millis();
  unsigned long lastByteTime = 0;
  const unsigned long RESPONSE_TIMEOUT = 800;
  const unsigned long FRAME_TIMEOUT = max(10UL, 40000UL / safeBaud + 3);

  while (millis() - startTime < RESPONSE_TIMEOUT) {
    esp_task_wdt_reset();
    updateLED();
    updateBuzzer();
    if (rs485.available()) {
      if (rIdx < (int)sizeof(buffer)) {
        buffer[rIdx++] = rs485.read();
        lastByteTime = millis();
      } else {
        break;
      }
    } else if (rIdx > 0 && millis() - lastByteTime >= FRAME_TIMEOUT) {
      break;
    }
  }

  // Switch Master UART to the newly configured baud rate
  if (settings.modbusBaudRate != safeBaud) {
    delay(20);
    rs485.updateBaudRate(settings.modbusBaudRate);
    delay(10);
  }
  prevSlaveDeviceID = settings.modbusDeviceID;
  prevSlaveBaudRate = settings.modbusBaudRate;

  // Strip any leading noise bytes before [targetDeviceID][0x10]
  int startOffset = -1;
  for (int i = 0; i <= rIdx - 8; i++) {
    if (buffer[i] == targetDeviceID && buffer[i + 1] == 0x10) {
      startOffset = i;
      break;
    }
  }

  if (startOffset < 0) {
    Serial.println("[MODBUS] No valid confirmation from slave");
    return;
  }

  uint8_t *pkt = &buffer[startOffset];
  uint16_t crcR = pkt[6] | (pkt[7] << 8);
  uint16_t crcC = modbusCRC(pkt, 6);

  if (crcR == crcC) {
    Serial.println("[MODBUS] Slave confirmed settings saved successfully!");
    triggerLED(1000);
    lastValidModbus = millis();
    sensorDataStale = false;
  } else {
    Serial.printf("[MODBUS] Settings ACK CRC error (got: %04X, calc: %04X)\n", crcR, crcC);
  }
}

void readModbusResponse() {
  uint8_t buffer[40];
  int index = 0;
  unsigned long startTime = millis();
  unsigned long lastByteTime = 0;
  uint32_t activeBaud = (settings.modbusBaudRate > 0) ? settings.modbusBaudRate : 115200;
  const unsigned long RESPONSE_TIMEOUT = 450;
  const unsigned long FRAME_TIMEOUT = max(8UL, 40000UL / activeBaud + 2);

  while (millis() - startTime < RESPONSE_TIMEOUT) {
    if (rs485.available()) {
      if (index < (int)sizeof(buffer)) {
        buffer[index++] = rs485.read();
        lastByteTime = millis();
      } else {
        break;
      }
    } else if (index > 0 && millis() - lastByteTime >= FRAME_TIMEOUT) {
      break;
    }
    updateLED();
    updateBuzzer();
  }

  if (index == 0) {
    Serial.println("[MODBUS] No response");
    modbusFailCount++;
    return;
  }

  // Locate valid frame header [modbusDeviceID][0x03] to ignore any leading TX/RX noise bytes
  int offset = -1;
  for (int i = 0; i <= index - 5; i++) {
    if (buffer[i] == settings.modbusDeviceID && buffer[i + 1] == 0x03) {
      offset = i;
      break;
    }
  }

  if (offset < 0) {
    Serial.println("[MODBUS] Invalid header or frame too short");
    modbusFailCount++;
    return;
  }

  uint8_t *frame = &buffer[offset];
  int frameLen = index - offset;
  uint8_t byteCount = frame[2];
  int expectedLen = 3 + byteCount + 2;

  if (frameLen < expectedLen) {
    Serial.println("[MODBUS] Incomplete frame");
    modbusFailCount++;
    return;
  }

  uint16_t crcReceived = frame[expectedLen - 2] | (frame[expectedLen - 1] << 8);
  uint16_t crcCalc = modbusCRC(frame, expectedLen - 2);

  if (crcReceived != crcCalc) {
    Serial.printf("[MODBUS] CRC error (got: %04X, calc: %04X)\n", crcReceived, crcCalc);
    modbusFailCount++;
    return;
  }

  triggerLED(ledDurationRX);
  modbusFailCount = 0;
  lastValidModbus = millis();
  sensorDataStale = false;

  if (byteCount >= 8) {
    int16_t rawTemp = (int16_t)((frame[3] << 8) | frame[4]);
    float parsedTemp = rawTemp / 100.0f;
    uint16_t luxHi = (frame[5] << 8) | frame[6];
    uint16_t luxLo = (frame[7] << 8) | frame[8];
    uint32_t parsedLux = ((uint32_t)luxHi << 16) | luxLo;

    portENTER_CRITICAL(&settingsMux);
    ntcTemp = parsedTemp;
    luxValue = parsedLux;
    portEXIT_CRITICAL(&settingsMux);
  }
  if (byteCount >= 10) {
    luxConnected = ((frame[9] << 8) | frame[10]) > 0;
  }
}

void handleModbus() {
  if (!settings.enableRS485) {
    sensorDataStale = false;
    modbusFailCount = 0;
    return;
  }

  uint32_t staleTimeout = max((uint32_t)15000, (uint32_t)(settings.modbusInterval * 3000UL));
  uint32_t watchdogTimeout = max((uint32_t)30000, (uint32_t)(settings.modbusInterval * 6000UL));

  if (lastValidModbus > 0 && (millis() - lastValidModbus > staleTimeout) && !sensorDataStale) {
    sensorDataStale = true;
    Serial.println("[RS485] Sensor data is STALE - transmitter not responding");
  }

  // Separate lastModbusReset from lastValidModbus so UART resets never mask stale detection!
  if ((modbusFailCount >= MODBUS_FAIL_LIMIT) || (lastValidModbus > 0 && (millis() - lastValidModbus > watchdogTimeout) && (millis() - lastModbusReset > watchdogTimeout))) {
    Serial.println("[RS485] Bus recovery triggered -> resetting UART");
    resetRS485UART();
    lastModbusReset = millis();
    lastModbusPoll = millis();
    return;
  }

  if (millis() - lastModbusPoll >= settings.modbusInterval * 1000UL) {
    lastModbusPoll = millis();
    setReceiveMode();
    while (rs485.available())
      rs485.read();

    sendModbusRequest(settings.modbusDeviceID, 0, 5);
    readModbusResponse();
  }
}

//////////////////////   SETUP   //////////////////////

void setup() {
  Serial.begin(115200);
  pinMode(RS485_EN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(100);

  Wire.begin();
  Wire.setTimeOut(100);  // 100ms I2C timeout prevents bus hang
  u8g2.begin();

  welcomeMsg();
  delay(2000);

  // --- Storage Init ---
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_14_tr);
  u8g2.drawStr(0, 18, "Settings Init:");
  u8g2.sendBuffer();
  loadSettings();
  u8g2.clearBuffer();
  u8g2.drawStr(0, 18, "Settings Init: OK");
  u8g2.sendBuffer();
  delay(800);

  // --- SPIFFS ---
  u8g2.clearBuffer();
  if (!SPIFFS.begin(true)) {
    u8g2.drawStr(0, 18, "SPIFFS: ERROR");
    Serial.println("[ERROR] SPIFFS Mount Failed!");
  } else {
    u8g2.drawStr(0, 18, "SPIFFS: OK");
    Serial.println("[SUCCESS] SPIFFS Mounted Successfully.");
  }
  u8g2.sendBuffer();
  delay(800);

  // --- RS485 modbus receiving mode ---
  setReceiveMode();
  rs485.begin(settings.modbusBaudRate, SERIAL_8N1, RXD2, TXD2);
  rs485.setTimeout(2);
  lastValidModbus = millis();
  lastModbusReset = millis();

  // --- AHT Sensor Init ---
  u8g2.clearBuffer();
  if (!aht.begin()) {
    ahtConnected = false;
    u8g2.drawStr(0, 18, "AHT10: ERROR");
    Serial.println("[ERROR] AHT10 not detected!");
  } else {
    ahtConnected = true;
    u8g2.drawStr(0, 18, "AHT10: OK");
    Serial.println("[SUCCESS] AHT10 detected successfully.");
  }
  u8g2.sendBuffer();
  delay(800);

  // --- WiFi Setup ---
  connectToSavedWiFi();
  delay(500);

  // --- Server Setup ---
  setupWebServer();
  if (WiFi.status() == WL_CONNECTED) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    if (webServerStarted) {
      u8g2.drawStr(0, 12, "Server Started!");
      u8g2.drawStr(0, 24, "IP Address:");
      u8g2.drawStr(0, 36, WiFi.localIP().toString().c_str());
    } else {
      u8g2.drawStr(0, 12, "Server Skipped!");
      u8g2.drawStr(0, 24, "Missing .gz files");
    }
    u8g2.sendBuffer();
    delay(1500);
  }

  // --- Time Setup ---
  configDateTimeBoot();

  // --- Sunrise & Sunset Calculation ---
  calculateSunriseSunset();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Sunrise & Sunset");
  char sunBuf[28];
  snprintf(sunBuf, sizeof(sunBuf), "Sunrise : %s", sunriseStr);
  u8g2.drawStr(0, 32, sunBuf);
  snprintf(sunBuf, sizeof(sunBuf), "Sunset  : %s", sunsetStr);
  u8g2.drawStr(0, 48, sunBuf);
  u8g2.sendBuffer();
  delay(1500);

  // --- Ready ---
  u8g2.clearBuffer();
  drawCenteredStr(35, "System Ready!", u8g2_font_t0_14_tr);
  u8g2.sendBuffer();
  delay(1000);
  u8g2.clearBuffer();

  // --- Enable Hardware Watchdog (12s timeout + Core 0 idle supervision) ---
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 12000,
    .idle_core_mask = (1 << 0),
    .trigger_panic = true
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK) {
    esp_task_wdt_init(&wdt_config);
  }
  esp_task_wdt_add(NULL);
}

//////////////////////   LOOP   //////////////////////

void loop() {
  esp_task_wdt_reset();

  processPendingWebActions();
  handleModbus();
  handleNTC();
  handleAHT();
  handleLUX();

  // Recalculate sunrise/sunset when date changes (non-blocking 5ms check)
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5) && timeinfo.tm_mday != lastCalcDay) {
    if (timeinfo.tm_year >= (2025 - 1900)) {
      ntpSynced = true;
    }
    calculateSunriseSunset();
  }

  serialOutput();
  displayLCD();
  checkWiFiAndStartServer();
  sendDataToNodeRed();

  updateLED();
  updateBuzzer();
}