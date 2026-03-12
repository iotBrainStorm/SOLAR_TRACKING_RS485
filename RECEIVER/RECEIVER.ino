#include <Arduino.h>
#include <math.h>  // To calculate NTC
#include <WiFi.h>
#include <WiFiManager.h>        // To save wifi password, instead of hard coding
#include <ESPAsyncWebServer.h>  // For web server
#include <HTTPClient.h>         // Node-Red
#include <Wire.h>               // I2C communication
#include <SPI.h>
#include <EEPROM.h>          // Settings storage
#include <SPIFFS.h>          // File system
#include <HardwareSerial.h>  // Serial communication
#include "time.h"            // Time management
#include <U8g2lib.h>         // OLED display
#include <ArduinoJson.h>     // make json format
#include <Preferences.h>     // To store users' settings
#include <Adafruit_AHT10.h>  // For temperature, humidity
#include <BH1750.h>          // For LUX measurement of sunlight

// -- RS485 Setup
#define RXD2 16
#define TXD2 17
#define RS485_EN 4
#define LED_PIN 2
#define STX 0x02
#define ETX 0x03
HardwareSerial rs485(1);
unsigned long lastModbusPoll = 0;


// -- NTC Setup
#define FIXED_RESISTOR 10000.0  // 10k fixed resistor
#define R0 10000.0              // NTC resistance at 25°C
#define BETA 3950.0             // 3950 (common value)
#define T0 298.15               // 25°C in Kelvin
#define OFFSET 17.29            // Adjust later (+ or -)
#define ADC_RESOLUTION 4095.0   // 12 bits (1111 1111 1111)
#define VREF 3.3                // Maximum sensing voltage of ESP32
unsigned long lastNTCReadTime = 0;
float ntcSampleSum = 0;
uint8_t ntcSampleCount = 0;

// -- Global variables for settings
Preferences preferences;
struct DeviceSettings {
  uint8_t tempPrecision;      // 0,1,2
  uint8_t humidityPrecision;  // 0,1,2
  uint16_t ahtInterval;       // >=1

  float ntcResistance;
  float betaConstant;
  float ntcOffset;
  uint16_t ntcInterval;

  uint8_t luxPercentageMode;  // 0 auto, 1 manual
  uint32_t maxLuxValue;
  uint32_t minLuxValue;
  uint16_t luxInterval;

  uint8_t enableNodeRed;  // 0 or 1
  String nodeRedIP;
  uint16_t nodeRedPort;
  uint16_t nodeRedInterval;

  uint8_t enableRS485;      // 0 = disabled, 1 = enabled
  uint8_t modbusDeviceID;   // 1–247
  uint32_t modbusBaudRate;  // 9600, 19200, 38400 etc
  uint16_t modbusInterval;  // seconds (share interval)

  long gmtOffset;       // seconds
  uint8_t clockFormat;  // 12 or 24
};
DeviceSettings settings;

// -- feedback LED Setup
bool ledState = false;
unsigned long ledStartTime = 0;
unsigned long ledDuration = 50;
const unsigned long ledDurationRX = 60;   // receive blink
const unsigned long ledDurationTX = 120;  // transmit blink

// -- LCD Setup
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
bool colonState = true;
unsigned long lastBlink = 0;

// -- Temperature and Humidity sensor setup
Adafruit_AHT10 aht;
unsigned long lastAHTReadTime = 0;

// -- Light sensor (LUX) setup
BH1750 lightMeter;
bool luxConnected = false;
unsigned long lastLuxReadTime = 0;
unsigned long lastLuxSaveTime = 0;
float luxFiltered = 0;  // EMA filtered value
bool luxInitialized = false;
float previousLux = 0;
long luxDiff = 0;

// -- Sensor Values Storage
float ntcTemp = 0.0;
float ahtTemp = 0.0;
float humidity = 0.0;
uint32_t luxValue = 0;
uint8_t sunlightPercentage = 0;

// -- Serial Output
unsigned long lastSerialPrint = 0;

// -- Time Management
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

// -- Web Server
AsyncWebServer server(80);
bool webServerStarted = false;

// -- Node-Red
bool nodeRedConnected = false;
String lastNodeRedResponse = "";
int httpResponseCode;

// -- Welcome Message
const char MSG_WELCOME[] PROGMEM = "ESP";
const char MSG_SUBTITLE[] PROGMEM = "SOLAR - TEM";
const char MSG_DEVELOPER[] PROGMEM = "developed by M.Maity";


//////////////////////   BLINK FEEDBACK LED   //////////////////////

void triggerLED(unsigned long duration) {
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  ledStartTime = millis();
  ledDuration = duration;
}

void updateLED() {
  if (ledState && millis() - ledStartTime >= ledDuration) {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  }
}

//////////////////////   DEFAULT SETTINGS   //////////////////////

void setDefaultSettings() {
  settings.tempPrecision = 1;
  settings.humidityPrecision = 0;
  settings.ahtInterval = 1;

  settings.ntcResistance = 10000.0;
  settings.betaConstant = 3435.0;
  settings.ntcOffset = 0.0;
  settings.ntcInterval = 1;

  settings.luxPercentageMode = 0;
  settings.maxLuxValue = 100;
  settings.minLuxValue = 1;
  settings.luxInterval = 1;

  settings.enableNodeRed = 0;
  settings.nodeRedIP = "";
  settings.nodeRedPort = 1880;
  settings.nodeRedInterval = 10;

  settings.enableRS485 = 0;
  settings.modbusDeviceID = 1;
  settings.modbusBaudRate = 115200;
  settings.modbusInterval = 2;


  settings.gmtOffset = 19800;
  settings.clockFormat = 24;
}

//////////////////////   LOAD SETTINGS   //////////////////////

void loadSettings() {

  Serial.println("\n========== LOADING SETTINGS ==========");

  preferences.begin("device", true);  // read-only

  settings.tempPrecision = preferences.getUChar("tPrec", 1);
  settings.humidityPrecision = preferences.getUChar("hPrec", 0);
  settings.ahtInterval = preferences.getUInt("ahtInt", 1);

  settings.ntcResistance = preferences.getFloat("ntcR", 10000.0);
  settings.betaConstant = preferences.getFloat("beta", 3435.0);
  settings.ntcOffset = preferences.getFloat("ntcOff", 0.0);
  settings.ntcInterval = preferences.getUInt("ntcInt", 1);

  settings.luxPercentageMode = preferences.getUChar("luxMode", 0);
  settings.maxLuxValue = preferences.getULong("luxMax", 100);
  settings.minLuxValue = preferences.getULong("luxMin", 1);
  settings.luxInterval = preferences.getUInt("luxInt", 1);

  settings.enableNodeRed = preferences.getUChar("nrEn", 0);
  settings.nodeRedIP = preferences.getString("nrIP", "");
  settings.nodeRedPort = preferences.getUInt("nrPort", 1880);
  settings.nodeRedInterval = preferences.getUInt("nrInt", 10);

  settings.enableRS485 = preferences.getUChar("rsEn", 0);
  settings.modbusDeviceID = preferences.getUChar("rsID", 1);
  settings.modbusBaudRate = preferences.getULong("rsBaud", 115200);
  settings.modbusInterval = preferences.getUInt("rsInt", 2);

  settings.gmtOffset = preferences.getLong("gmt", 19800);
  settings.clockFormat = preferences.getUChar("clkFmt", 24);

  preferences.end();

  // -------- PRINT LOADED SETTINGS --------
  Serial.println("Temperature Precision : " + String(settings.tempPrecision));
  Serial.println("Humidity Precision    : " + String(settings.humidityPrecision));
  Serial.println("AHT Interval (sec)    : " + String(settings.ahtInterval));

  Serial.println("NTC Resistance        : " + String(settings.ntcResistance));
  Serial.println("Beta Constant         : " + String(settings.betaConstant));
  Serial.println("NTC Offset            : " + String(settings.ntcOffset));
  Serial.println("NTC Interval (sec)    : " + String(settings.ntcInterval));

  Serial.println("Lux Mode (0=A,1=M)    : " + String(settings.luxPercentageMode));
  Serial.println("Max Lux Value         : " + String(settings.maxLuxValue));
  Serial.println("Min Lux Value         : " + String(settings.minLuxValue));
  Serial.println("Lux Interval (sec)    : " + String(settings.luxInterval));

  Serial.println("Node-RED Enabled      : " + String(settings.enableNodeRed));
  Serial.println("Node-RED IP           : " + settings.nodeRedIP);
  Serial.println("Node-RED Port         : " + String(settings.nodeRedPort));
  Serial.println("Node-RED Interval     : " + String(settings.nodeRedInterval));

  Serial.println("RS485 Enabled         : " + String(settings.enableRS485));
  Serial.println("Modbus Device ID      : " + String(settings.modbusDeviceID));
  Serial.println("Modbus Baud Rate      : " + String(settings.modbusBaudRate));
  Serial.println("Modbus Interval (sec) : " + String(settings.modbusInterval));

  Serial.println("GMT Offset (sec)      : " + String(settings.gmtOffset));
  Serial.println("Clock Format          : " + String(settings.clockFormat));

  Serial.println("=======================================\n");
}

//////////////////////   SAVE SETTINGS   //////////////////////

void saveSettings() {
  preferences.begin("device", false);  // write mode

  preferences.putUChar("tPrec", settings.tempPrecision);
  preferences.putUChar("hPrec", settings.humidityPrecision);
  preferences.putUInt("ahtInt", settings.ahtInterval);

  preferences.putFloat("ntcR", settings.ntcResistance);
  preferences.putFloat("beta", settings.betaConstant);
  preferences.putFloat("ntcOff", settings.ntcOffset);
  preferences.putUInt("ntcInt", settings.ntcInterval);

  preferences.putUChar("luxMode", settings.luxPercentageMode);
  preferences.putULong("luxMax", settings.maxLuxValue);
  preferences.putULong("luxMin", settings.minLuxValue);
  preferences.putUInt("luxInt", settings.luxInterval);

  preferences.putUChar("nrEn", settings.enableNodeRed);
  preferences.putString("nrIP", settings.nodeRedIP);
  preferences.putUInt("nrPort", settings.nodeRedPort);
  preferences.putUInt("nrInt", settings.nodeRedInterval);

  preferences.putUChar("rsEn", settings.enableRS485);
  preferences.putUChar("rsID", settings.modbusDeviceID);
  preferences.putULong("rsBaud", settings.modbusBaudRate);
  preferences.putUInt("rsInt", settings.modbusInterval);

  preferences.putLong("gmt", settings.gmtOffset);
  preferences.putUChar("clkFmt", settings.clockFormat);

  preferences.end();
  Serial.println("Settings Saved to NVS");
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
  WiFi.begin();

  int attempts = 0;
  const int MAX_ATTEMPTS = 5;

  while (attempts < MAX_ATTEMPTS) {

    char attemptStr[16];
    snprintf(attemptStr, sizeof(attemptStr), "Attempt: %d/5", attempts + 1);

    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();

    Serial.printf("[INFO] %s\n", attemptStr);

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

    delay(2000);
    attempts++;
  }

  // Failed to connect
  Serial.println("\n[WARNING] No saved WiFi found!");
  Serial.println("[INFO] Starting Config Portal...");
  Serial.println("AP SSID    : Solar Weather");
  Serial.println("AP IP      : 192.168.4.1");
  Serial.println("Timeout    : 60 seconds");
  Serial.println("------------------------------");

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

    Serial.println("\n[SUCCESS] WiFi Connected via Config Portal");
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
  } else {

    Serial.println("\n[ERROR] Config Portal Timeout!");
    Serial.println("Device not connected to WiFi.");
    Serial.println("==============================\n");

    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Time over!");
    u8g2.sendBuffer();

    delay(1000);
    return false;  // Better logic than returning true
  }
}

//////////////////////   TIME SETUP   //////////////////////

void configDateTime() {

  Serial.println("\n==============================");
  Serial.println("Date & Time Configuration");
  Serial.println("==============================");

  if (WiFi.status() != WL_CONNECTED) {

    Serial.println("[WARNING] WiFi not connected!");
    Serial.println("[INFO] Running in offline mode.");
    Serial.println("[INFO] Setting default time: 01/01/2025 12:00:00");

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 24, "No WiFi!");
    u8g2.drawStr(0, 36, "Time not synced!");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();

    unsigned long start = millis();
    while (millis() - start < 2000)
      yield();

    struct tm tm;
    tm.tm_year = 2025 - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 12;
    tm.tm_min = 0;
    tm.tm_sec = 0;

    time_t t = mktime(&tm);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, nullptr);

    Serial.println("[SUCCESS] Default time applied.");
    Serial.println("==============================\n");
    return;
  }

  Serial.println("[INFO] WiFi connected.");
  Serial.println("[INFO] Starting NTP sync...");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Syncing Time...");
  u8g2.sendBuffer();

  unsigned long start = millis();
  while (millis() - start < 1000)
    yield();

  int attempts = 0;
  const int MAX_ATTEMPTS = 5;
  struct tm timeinfo;

  while (attempts < MAX_ATTEMPTS) {

    Serial.printf("[INFO] NTP Attempt %d/%d\n", attempts + 1, MAX_ATTEMPTS);

    char attemptStr[16];
    snprintf(attemptStr, sizeof(attemptStr), "Attempt: %d/5", attempts + 1);

    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Syncing Time...");
    u8g2.drawStr(0, 24, attemptStr);
    u8g2.sendBuffer();

    configTime(settings.gmtOffset, daylightOffset_sec, ntpServer);

    start = millis();
    while (millis() - start < 1000)
      yield();

    if (getLocalTime(&timeinfo)) {
      Serial.println("[SUCCESS] Time synced from NTP server.");
      break;
    }

    attempts++;
  }

  if (getLocalTime(&timeinfo)) {

    char timeStr[16];
    char dateStr[18];
    char gmtStr[30];

    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "%d.%m.%Y", &timeinfo);
    strftime(gmtStr, sizeof(gmtStr), "%z %Z", &timeinfo);

    Serial.println("-------- Current Time --------");
    Serial.printf("Time : %s\n", timeStr);
    Serial.printf("Date : %s\n", dateStr);
    Serial.printf("Zone : %s\n", gmtStr);
    Serial.println("------------------------------");
    Serial.println("==============================\n");

    strftime(timeStr, sizeof(timeStr), "Time: %H:%M:%S", &timeinfo);
    strftime(dateStr, sizeof(dateStr), "Date: %d.%m.%Y", &timeinfo);
    strftime(gmtStr, sizeof(gmtStr), "GMT: %z %Z", &timeinfo);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_14_tr);
    u8g2.drawStr(0, 16, timeStr);
    u8g2.drawStr(0, 32, dateStr);
    u8g2.drawStr(0, 62, gmtStr);
    u8g2.sendBuffer();

    start = millis();
    while (millis() - start < 2000)
      yield();
  } else {

    Serial.println("[ERROR] NTP sync failed!");
    Serial.println("[INFO] Applying default offline time.");
    Serial.println("[INFO] Default: 01/01/2025 12:00:00");

    struct tm tm;
    tm.tm_year = 2025 - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 12;
    tm.tm_min = 0;
    tm.tm_sec = 0;

    time_t t = mktime(&tm);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, nullptr);

    Serial.println("[SUCCESS] Default time applied.");
    Serial.println("==============================\n");

    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Time not synced!");
    u8g2.drawStr(0, 24, "Check internet!");
    u8g2.drawStr(0, 48, "Starting offline...");
    u8g2.sendBuffer();

    start = millis();
    while (millis() - start < 2000)
      yield();
  }
}

//////////////////////   SERVER SETUP   //////////////////////

void setupWebServer() {
  if (webServerStarted)
    return;  // Already started

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    // Serial.println("Root Requested");
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/config.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/config.html", "text/html");
  });

  server.on("/dashboard.svg", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/dashboard.svg", "image/svg+xml");
  });

  server.on("/settings.svg", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/settings.svg", "image/svg+xml");
  });

  server.on("/sensor.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<256> doc;
    doc["ntcTemp"] = ntcTemp;
    doc["ahtTemp"] = ahtTemp;
    doc["humidity"] = humidity;
    doc["lux"] = luxValue;
    doc["sunlight"] = sunlightPercentage;

    String response;
    serializeJson(doc, response);

    request->send(200, "application/json", response);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest * request) {
    // -------- TEMPERATURE --------
    if (request->hasParam("tempPrecision", true))
      settings.tempPrecision = request->getParam("tempPrecision", true)->value().toInt();

    if (request->hasParam("humidityPrecision", true))
      settings.humidityPrecision = request->getParam("humidityPrecision", true)->value().toInt();

    if (request->hasParam("ahtInterval", true))
      settings.ahtInterval = request->getParam("ahtInterval", true)->value().toInt();


    // -------- NTC --------
    if (request->hasParam("ntcResistance", true))
      settings.ntcResistance = request->getParam("ntcResistance", true)->value().toFloat();

    if (request->hasParam("betaConstant", true))
      settings.betaConstant = request->getParam("betaConstant", true)->value().toFloat();

    if (request->hasParam("ntcOffset", true))
      settings.ntcOffset = request->getParam("ntcOffset", true)->value().toFloat();

    if (request->hasParam("ntcInterval", true))
      settings.ntcInterval = request->getParam("ntcInterval", true)->value().toInt();


    // -------- LUX --------
    uint8_t previousLuxMode = settings.luxPercentageMode;
    if (request->hasParam("luxMode", true))
      settings.luxPercentageMode = request->getParam("luxMode", true)->value().toInt();

    if (previousLuxMode == 1 && settings.luxPercentageMode == 0) {
      settings.maxLuxValue = 0;
      settings.minLuxValue = 0;
      Serial.println("[LUX] AUTO mode selected → Min/Max reset");
    } else {
      if (request->hasParam("maxLuxValue", true))
        settings.maxLuxValue = request->getParam("maxLuxValue", true)->value().toInt();
      if (request->hasParam("minLuxValue", true))
        settings.minLuxValue = request->getParam("minLuxValue", true)->value().toInt();
    }
    if (request->hasParam("luxInterval", true))
      settings.luxInterval = request->getParam("luxInterval", true)->value().toInt();

    // -------- NODE RED --------
    settings.enableNodeRed = request->hasParam("enableNodeRed", true) ? 1 : 0;

    if (request->hasParam("nodeRedIP", true))
      settings.nodeRedIP = request->getParam("nodeRedIP", true)->value();

    if (request->hasParam("nodeRedPort", true))
      settings.nodeRedPort = request->getParam("nodeRedPort", true)->value().toInt();

    if (request->hasParam("nodeRedInterval", true))
      settings.nodeRedInterval = request->getParam("nodeRedInterval", true)->value().toInt();

    // -------- RS485 --------
    settings.enableRS485 =
      request->hasParam("enableRS485", true) ? 1 : 0;

    settings.modbusDeviceID =
      request->getParam("modbusDeviceID", true)->value().toInt();

    settings.modbusBaudRate =
      request->getParam("modbusBaudRate", true)->value().toInt();

    settings.modbusInterval =
      request->getParam("modbusInterval", true)->value().toInt();

    // -------- CLOCK --------
    if (request->hasParam("gmtOffset", true))
      settings.gmtOffset = request->getParam("gmtOffset", true)->value().toInt();

    if (request->hasParam("clockFormat", true))
      settings.clockFormat = request->getParam("clockFormat", true)->value().toInt();


    // -------- SAVE TO NVS --------
    saveSettings();
    sendSettingsToSlave();
    // triggerLED(60);

    Serial.println("Settings Saved Successfully");

    request->send(200, "text/plain", "Settings Saved");
  });

  server.on("/settings.json", HTTP_GET, [](AsyncWebServerRequest * request) {
    StaticJsonDocument<768> doc;

    doc["tempPrecision"] = settings.tempPrecision;
    doc["humidityPrecision"] = settings.humidityPrecision;
    doc["ahtInterval"] = settings.ahtInterval;

    doc["ntcResistance"] = settings.ntcResistance;
    doc["betaConstant"] = settings.betaConstant;
    doc["ntcOffset"] = settings.ntcOffset;
    doc["ntcInterval"] = settings.ntcInterval;

    doc["luxMode"] = settings.luxPercentageMode;
    doc["maxLuxValue"] = settings.maxLuxValue;
    doc["minLuxValue"] = settings.minLuxValue;
    doc["luxInterval"] = settings.luxInterval;

    doc["enableNodeRed"] = settings.enableNodeRed;
    doc["nodeRedIP"] = settings.nodeRedIP;
    doc["nodeRedPort"] = settings.nodeRedPort;
    doc["nodeRedInterval"] = settings.nodeRedInterval;

    doc["enableRS485"] = settings.enableRS485;
    doc["modbusDeviceID"] = settings.modbusDeviceID;
    doc["modbusBaudRate"] = settings.modbusBaudRate;
    doc["modbusInterval"] = settings.modbusInterval;

    doc["gmtOffset"] = settings.gmtOffset;
    doc["clockFormat"] = settings.clockFormat;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest * request) {
    Serial.println("\n===== FACTORY RESET REQUEST RECEIVED =====");

    // 1️⃣ Clear old preferences
    preferences.begin("device", false);
    preferences.clear();
    preferences.end();

    Serial.println("NVS Cleared");

    // 2️⃣ Load default values into RAM
    setDefaultSettings();

    // 3️⃣ Save default values back to NVS
    saveSettings();

    Serial.println("Default Settings Applied & Saved");

    // 4️⃣ Send response to browser
    request->send(200, "text/plain", "Factory Reset Successful");

    // 5️⃣ Optional: restart device after short delay
    delay(1000);
    ESP.restart();
  });

  server.begin();
  webServerStarted = true;
}

//////////////////////   SERVER RECONNECT   //////////////////////

void checkWiFiAndStartServer() {
  static unsigned long lastCheck = 0;
  static bool wasConnected = false;
  static unsigned long lastReconnectAttempt = 0;

  if (millis() - lastCheck < 3000)
    return;  // Check every 3s
  lastCheck = millis();

  bool isConnected = (WiFi.status() == WL_CONNECTED);

  // ===============================
  // 🔵 WiFi Just Connected
  // ===============================
  if (isConnected && !wasConnected) {

    Serial.println("WiFi connected!");

    // Start Web Server
    if (!webServerStarted) {
      Serial.println("Starting WebServer...");
      setupWebServer();  // must include server.begin()
      webServerStarted = true;
    }

    // ===============================
    // 🔴 WiFi Lost
    // ===============================
    if (!isConnected && wasConnected) {
      Serial.println("WiFi disconnected!");
      webServerStarted = false;  // allow restart after reconnection
    }

    // ===============================
    // 🟡 Attempt Reconnect
    // ===============================
    if (!isConnected) {

      if (millis() - lastReconnectAttempt > 20000) {  // every 20 sec
        lastReconnectAttempt = millis();

        Serial.println("Attempting WiFi reconnection...");

        WiFi.disconnect();
        delay(200);
        WiFi.reconnect();
      }
    }
  }
  wasConnected = isConnected;
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

//////////////////////   CENTRE TEXT   //////////////////////

void drawCenteredStr(int y, const char *str, const uint8_t *font) {
  u8g2.setFont(font);
  int16_t strWidth = u8g2.getStrWidth(str);
  int16_t x = (128 - strWidth) / 2;
  u8g2.drawStr(x, y, str);
}

//////////////////////   CALCULATE PRECISION   //////////////////////

float applyPrecision(float value, uint8_t precision) {
  if (precision == 0) return round(value);
  if (precision == 1) return round(value * 10.0) / 10.0;
  if (precision == 2) return round(value * 100.0) / 100.0;
  return value;
}

void handleNTC() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastNTCReadTime >= settings.ntcInterval * 1000UL) {
    lastNTCReadTime = currentMillis;
    ntcTemp = applyPrecision(ntcTemp, settings.tempPrecision);
  }
}

//////////////////////   AHT10   //////////////////////

void handleAHT() {
  unsigned long currentMillis = millis();
  // Check interval (seconds → milliseconds)
  if (currentMillis - lastAHTReadTime >= settings.ahtInterval * 1000UL) {
    lastAHTReadTime = currentMillis;
    sensors_event_t humidityEvent, tempEvent;
    if (aht.getEvent(&humidityEvent, &tempEvent)) {
      // Raw values
      float rawTemp = tempEvent.temperature;
      float rawHumidity = humidityEvent.relative_humidity;
      // Apply precision from settings
      ahtTemp = applyPrecision(rawTemp, settings.tempPrecision);
      humidity = applyPrecision(rawHumidity, settings.humidityPrecision);
    } else {
      ahtTemp = 0.0;  // Error indicator
      humidity = 0;
    }
  }
}

//////////////////////   CALCULATE LUX   //////////////////////

void handleLUX() {
  unsigned long currentMillis = millis();

  // ---- Interval Control ----
  if (currentMillis - lastLuxReadTime < settings.luxInterval * 1000UL) return;
  lastLuxReadTime = currentMillis;

  // ---- EMA Filter ----
  luxFiltered = (luxFiltered * 0.6f) + (luxValue * 0.4f);

  luxDiff = (long)(luxFiltered - previousLux);
  previousLux = luxFiltered;

  // ==========================================================
  // ====================== AUTO MODE =========================
  // ==========================================================
  if (settings.luxPercentageMode == 0) {

    const uint32_t threshold = 200;        // Min change to update
    const uint32_t saveInterval = 300000;  // 5 min EEPROM safety

    bool updated = false;

    // Initialize min/max on first reading
    if (settings.maxLuxValue == 0 && settings.minLuxValue == 0) {
      settings.maxLuxValue = luxFiltered;
      settings.minLuxValue = luxFiltered;
      updated = true;
    }

    // Update MAX
    if (luxFiltered > settings.maxLuxValue + threshold) {
      settings.maxLuxValue = (uint32_t)luxFiltered;
      updated = true;
    }

    // Update MIN
    if (luxFiltered + threshold < settings.minLuxValue) {
      settings.minLuxValue = (uint32_t)luxFiltered;
      updated = true;
    }

    // EEPROM Save Protection
    if (updated && (currentMillis - lastLuxSaveTime > saveInterval)) {
      saveSettings();
      lastLuxSaveTime = currentMillis;
      Serial.println("[LUX] AUTO: Min/Max updated & saved");
    }
  }

  // ==========================================================
  // ===================== MANUAL MODE ========================
  // ==========================================================
  else if (settings.luxPercentageMode == 1) {

    // Do nothing with min/max
    // Just use stored calibration values
    // No EEPROM write

    Serial.println("[LUX] MANUAL mode active");
  }

  // ==========================================================
  // ================== PERCENTAGE CALC =======================
  // ==========================================================

  uint32_t minLux = settings.minLuxValue;
  uint32_t maxLux = settings.maxLuxValue;

  if (maxLux > minLux) {

    float percent = ((luxFiltered - minLux) * 100.0f) / (float)(maxLux - minLux);

    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    sunlightPercentage = (uint8_t)percent;

  } else {
    sunlightPercentage = 0;  // safety
  }

  // // Debug
  // Serial.print("[LUX] Raw: ");
  // Serial.print(luxValue);
  // Serial.print(" | Filtered: ");
  // Serial.print(luxFiltered);
  // Serial.print(" | Sunlight: ");
  // Serial.println(sunlightPercentage);
}

//////////////////////   RS485 CONTROL   //////////////////////
void setTransmitMode() {
  digitalWrite(RS485_EN, HIGH);
}
void setReceiveMode() {
  digitalWrite(RS485_EN, LOW);
}

//////////////////////   NODE RED SHARE   //////////////////////

void sendDataToNodeRed() {

  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate < (settings.nodeRedInterval * 1000UL))
    return;
  lastUpdate = millis();

  Serial.println(F("\n[Node-RED] Send attempt started"));

  // Check enabled
  if (!settings.enableNodeRed) {
    Serial.println(F("[Node-RED] Disabled in settings"));
    return;
  }

  // Check WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[Node-RED] WiFi not connected"));
    return;
  }

  // Build URL
  String url = "http://" + settings.nodeRedIP + ":" + String(settings.nodeRedPort) + "/solar-data";

  Serial.print(F("[Node-RED] URL: "));
  Serial.println(url);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(5000);
  http.addHeader("Content-Type", "application/json");

  char jsonPayload[256];
  float safeLux = luxValue;
  float safeSun = sunlightPercentage;
  if (isnan(safeLux))
    safeLux = 0.0;
  if (isnan(safeSun))
    safeSun = 0.0;
  snprintf(jsonPayload, sizeof(jsonPayload),
           "{\"ntcTemp\":%.*f,"
           "\"ahtTemp\":%.*f,"
           "\"humidity\":%.*f,"
           "\"lux\":%.1f,"
           "\"sunlight\":%.1f}",
           settings.tempPrecision, ntcTemp,
           settings.tempPrecision, ahtTemp,
           settings.humidityPrecision, humidity,
           safeLux,
           safeSun);
  Serial.print("[Node-RED] Payload: ");
  Serial.println(jsonPayload);
  httpResponseCode = http.POST((uint8_t *)jsonPayload, strlen(jsonPayload));

  Serial.print(F("[Node-RED] HTTP Response code: "));
  Serial.println(httpResponseCode);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.print(F("[Node-RED] Response body: "));
    Serial.println(response);

    nodeRedConnected = true;
    lastNodeRedResponse = String(httpResponseCode);
  } else {
    Serial.print(F("[Node-RED] Error: "));
    Serial.println(http.errorToString(httpResponseCode));

    nodeRedConnected = false;
    lastNodeRedResponse = http.errorToString(httpResponseCode);
  }

  http.end();
  Serial.println(F("[Node-RED] Send attempt finished"));
}

//////////////////////   SERIAL OUTPUT   //////////////////////

void serialOutput() {

  unsigned long currentMillis = millis();

  // Run every 1 second (1000 ms)
  if (currentMillis - lastSerialPrint < 1000) return;
  lastSerialPrint = currentMillis;

  Serial.println("\n========================================");
  Serial.println("           Sensor Data Report");
  Serial.println("========================================");

  Serial.printf("NTC Temperature  : %8.2f °C\n", ntcTemp);
  Serial.printf("AHT Temperature  : %8.2f °C\n", ahtTemp);
  Serial.printf("Humidity         : %8.2f %%\n", humidity);

  Serial.println("----------------------------------------");

  Serial.printf("Light Intensity  : %8lu lux\n", luxValue);
  Serial.printf("Filtered Lux     : %8.2f lux\n", luxFiltered);
  Serial.printf("Sunlight Level   : %8u %%\n", sunlightPercentage);

  Serial.println("----------------------------------------");

  float tempDiff = fabs(ntcTemp - ahtTemp);
  Serial.printf("Temp Difference  : %8.2f °C\n", tempDiff);

  Serial.println("========================================\n");
}

//////////////////////   DISPLAY LCD   //////////////////////

void displayLCD() {

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  // ================= TIME =================
  struct tm timeinfo;
  bool hasTime = getLocalTime(&timeinfo);
  char timeStr[6] = "--:--";
  if (hasTime) {
    if (settings.clockFormat == 12) {
      strftime(timeStr, sizeof(timeStr), "%I:%M", &timeinfo);
    } else {
      strftime(timeStr, sizeof(timeStr), "%H:%M", &timeinfo);
    }
  }
  u8g2.drawStr(77, 12, timeStr);

  // ================= WIFI RSSI =================
  int rssi = WiFi.isConnected() ? WiFi.RSSI() : -99;  // e.g., -40 strong, -90 weak
  char wifiStr[15];
  if (WiFi.isConnected()) {
    snprintf(wifiStr, sizeof(wifiStr), "W:%ddBm", rssi);
  } else {
    snprintf(wifiStr, sizeof(wifiStr), "W:Disc");
  }
  u8g2.drawStr(77, 24, wifiStr);

  // ================= SENSOR VALUES =================

  char ntcStr[20];
  snprintf(ntcStr, sizeof(ntcStr), "SLR: %.2fC", ntcTemp);
  char fmtTemp[10];
  sprintf(fmtTemp, "SLR: %%.%dfC", settings.tempPrecision);
  snprintf(ntcStr, sizeof(ntcStr), fmtTemp, ntcTemp);
  u8g2.drawStr(0, 12, ntcStr);

  char ahtStr[20];
  sprintf(fmtTemp, "SRR: %%.%dfC", settings.tempPrecision);
  snprintf(ahtStr, sizeof(ahtStr), fmtTemp, ahtTemp);
  u8g2.drawStr(0, 24, ahtStr);

  char humStr[20];
  char fmtHum[10];
  sprintf(fmtHum, "HUM: %%.%df%%%%", settings.humidityPrecision);
  snprintf(humStr, sizeof(humStr), fmtHum, humidity);
  u8g2.drawStr(0, 36, humStr);

  // Divider line
  u8g2.drawHLine(0, 40, 128);

  // Lux
  char luxStr[20];
  snprintf(luxStr, sizeof(luxStr), "LUX: %lu", luxValue);
  u8g2.drawStr(0, 52, luxStr);

  // Difference string
  char diffStr[12];
  if (luxDiff > 0)
    snprintf(diffStr, sizeof(diffStr), "+%ld", luxDiff);
  else
    snprintf(diffStr, sizeof(diffStr), "%ld", luxDiff);
  u8g2.drawStr(83, 52, diffStr);  // Adjust X if needed

  // Sunlight % PROGRESS BAR
  char sunStr[20];
  snprintf(sunStr, sizeof(sunStr), "SUN: %u%%", sunlightPercentage);
  u8g2.drawStr(0, 64, sunStr);
  int barWidth = map(sunlightPercentage, 0, 100, 0, 60);
  barWidth = constrain(barWidth, 0, 60);  // Prevent overflow
  u8g2.drawFrame(64, 56, 64, 8);          // Frame
  u8g2.drawBox(66, 58, barWidth, 4);      // Fill

  // ================= NODE-RED STATUS =================
  char nrStr[25];
  if (lastNodeRedResponse.length() > 0) {
    String responseCodeString = String(httpResponseCode);
    snprintf(nrStr, sizeof(nrStr), "NR: %s", responseCodeString.c_str());
  } else {
    snprintf(nrStr, sizeof(nrStr), "NR: --");
  }
  u8g2.drawStr(77, 36, nrStr);

  u8g2.sendBuffer();
}

//////////////////////   RS485 SETUP   //////////////////////

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

  while (rs485.available()) rs485.read();
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
  triggerLED(ledDurationTX);
  rs485.write(frame, 8);
  rs485.flush();
  delay(15);
  setReceiveMode();

  Serial.println("[MODBUS] Request sent");
}

void sendSettingsToSlave() {

  uint8_t frame[32];

  frame[0] = settings.modbusDeviceID;
  frame[1] = 0x10;
  frame[2] = 0x00;
  frame[3] = 0x03;
  frame[4] = 0x00;
  frame[5] = 0x06;
  frame[6] = 12;

  int idx = 7;

  uint16_t val;

  val = settings.ntcResistance;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.betaConstant;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.ntcOffset * 100;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.ntcInterval;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.luxInterval;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  val = settings.modbusInterval;
  frame[idx++] = val >> 8;
  frame[idx++] = val & 0xFF;

  uint16_t crc = modbusCRC(frame, idx);

  frame[idx++] = crc & 0xFF;
  frame[idx++] = crc >> 8;

  setTransmitMode();
  rs485.write(frame, idx);
  rs485.flush();
  setReceiveMode();

  Serial.println("[MODBUS] Settings sent to slave");
}

void sendOK() {
  const char *msg = "OK\n";

  setTransmitMode();

  triggerLED(ledDurationTX);

  rs485.write((uint8_t *)msg, strlen(msg));
  rs485.flush();

  delayMicroseconds(200);

  setReceiveMode();
}

void readModbusResponse() {
  static uint8_t buffer[32];
  static int index = 0;
  static unsigned long lastByteTime = 0;

  while (rs485.available()) {
    buffer[index++] = rs485.read();
    lastByteTime = millis();

    if (index >= sizeof(buffer)) index = 0;
  }

  // wait for frame end (5ms silence)
  if (index > 0 && millis() - lastByteTime > 20) {
    int len = index;
    index = 0;

    if (len < 5) {
      Serial.println("[MODBUS] Frame too short");
      return;
    }

    uint16_t crcReceived = buffer[len - 2] | (buffer[len - 1] << 8);
    uint16_t crcCalc = modbusCRC(buffer, len - 2);

    if (crcReceived != crcCalc) {
      Serial.println("[MODBUS] CRC error");
      return;
    }

    Serial.println("[MODBUS] Valid response");
    triggerLED(ledDurationRX);

    uint8_t byteCount = buffer[2];

    if (byteCount >= 6) {
      ntcTemp = ((buffer[3] << 8) | buffer[4]) / 100.0;
      luxValue = (buffer[5] << 8) | buffer[6];
    }
    if (byteCount >= 8) {
      luxConnected = ((buffer[7] << 8) | buffer[8]) > 0;
    }
  }
}

void handleModbus() {
  if (settings.enableRS485) {

    if (millis() - lastModbusPoll >= settings.modbusInterval * 1000UL) {

      lastModbusPoll = millis();

      sendModbusRequest(settings.modbusDeviceID, 0, 4);
    }

    readModbusResponse();
  }
}
//////////////////////   SETUP   //////////////////////

void setup() {
  Serial.begin(115200);
  pinMode(RS485_EN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  u8g2.begin();

  // --- Welcome ---
  welcomeMsg();
  delay(2000);

  // --- Storage Init ---
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_14_tr);
  u8g2.drawStr(0, 18, "Settings Init:");
  u8g2.sendBuffer();
  Serial.println("\n==============================");
  Serial.println("Settings Initialization");
  Serial.println("==============================");
  Serial.println("[INFO] Initializing EEPROM...");
  delay(1000);
  EEPROM.begin(512);
  loadSettings();
  Serial.println("[SUCCESS] EEPROM Initialized.");
  Serial.printf("[INFO] EEPROM Size: %d bytes\n", 512);
  Serial.println("==============================\n");
  u8g2.clearBuffer();  // Recommended for clean update
  u8g2.drawStr(0, 18, "Settings Init: OK");
  u8g2.sendBuffer();
  delay(1000);

  // --- SPIFFS ---
  u8g2.clearBuffer();
  Serial.println("\n==============================");
  Serial.println("SPIFFS Initialization");
  Serial.println("==============================");
  if (!SPIFFS.begin(true)) {
    u8g2.drawStr(0, 18, "SPIFFS: ERROR");
    Serial.println("[ERROR] SPIFFS Mount Failed!");
    Serial.println("[INFO] Filesystem not available.");
    Serial.println("==============================\n");
  } else {
    u8g2.drawStr(0, 18, "SPIFFS: OK");
    Serial.println("[SUCCESS] SPIFFS Mounted Successfully.");
    Serial.println("[INFO] Listing Files:");
    Serial.println("------------------------------");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    int fileCount = 0;
    while (file) {
      Serial.printf("File %02d : %s  |  Size: %d bytes\n",
                    fileCount + 1,
                    file.name(),
                    file.size());
      fileCount++;
      file = root.openNextFile();
    }
    Serial.println("------------------------------");
    Serial.printf("Total Files: %d\n", fileCount);
    Serial.println("==============================\n");
    root.close();
  }
  u8g2.sendBuffer();
  delay(1000);

  // --- RS485 modbus receiving mode ---
  setReceiveMode();
  rs485.begin(settings.modbusBaudRate, SERIAL_8N1, RXD2, TXD2);
  rs485.setTimeout(2);

  // --- AHT Sensor Init ---
  u8g2.clearBuffer();
  Serial.println("\n==============================");
  Serial.println("AHT10 Sensor Initialization");
  Serial.println("==============================");
  Serial.println("[INFO] Checking AHT10 sensor...");
  Wire.begin();  // SDA, SCL default for ESP32
  if (!aht.begin()) {
    u8g2.drawStr(0, 18, "AHT10: ERROR");
    Serial.println("[ERROR] AHT10 not detected!");
    Serial.println("[INFO] Check wiring (SDA/SCL) and power supply.");
    Serial.println("==============================\n");
  } else {
    u8g2.drawStr(0, 18, "AHT10: OK");
    Serial.println("[SUCCESS] AHT10 detected successfully.");
    Serial.println("[INFO] Sensor ready for reading");
    Serial.println("==============================\n");
  }
  u8g2.sendBuffer();
  delay(1000);

  // --- WiFi Setup ---
  connectToSavedWiFi();
  delay(1000);

  // --- Server Setup ---
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n==============================");
    Serial.println("Web Server Initialization");
    Serial.println("==============================");
    Serial.println("[INFO] WiFi Connected.");
    Serial.println("[INFO] Starting Web Server...");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(0, 12, "Starting Server...");
    u8g2.sendBuffer();
    delay(300);

    setupWebServer();

    Serial.println("[SUCCESS] Web Server Started!");
    Serial.printf("SSID       : %s\n", WiFi.SSID().c_str());
    Serial.printf("Server IP  : %s\n", WiFi.localIP().toString().c_str());
    Serial.println("==============================\n");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Server Started!");
    u8g2.drawStr(0, 24, "IP Address:");
    u8g2.drawStr(0, 36, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer();
    delay(2000);
  } else {
    Serial.println("\n==============================");
    Serial.println("Web Server Initialization");
    Serial.println("==============================");
    Serial.println("[ERROR] WiFi not connected!");
    Serial.println("[INFO] Server will auto-start once WiFi reconnects.");
    Serial.println("==============================\n");
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "WiFi Failed!");
    u8g2.drawStr(0, 24, "Will auto-start");
    u8g2.drawStr(0, 36, "when connected");
    u8g2.sendBuffer();
    delay(2000);
  }

  // --- Time Setup ---
  configDateTime();
  delay(1000);

  // --- Ready ---
  u8g2.clearBuffer();
  drawCenteredStr(35, "System Ready!", u8g2_font_t0_14_tr);
  u8g2.sendBuffer();
  delay(1500);
  u8g2.clearBuffer();
}

void loop() {
  handleModbus();
  handleNTC();
  handleAHT();
  handleLUX();

  serialOutput();
  displayLCD();
  checkWiFiAndStartServer();
  sendDataToNodeRed();

  updateLED();
}
