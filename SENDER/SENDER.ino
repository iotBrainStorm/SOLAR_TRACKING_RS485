#include <Arduino.h>
#include <math.h>            // To calculate NTC
#include <Wire.h>            // I2C communication
#include <SPI.h>             // For I2C communication
#include <EEPROM.h>          // Settings storage
#include <SPIFFS.h>          // File system
#include <HardwareSerial.h>  // Serial communication
#include <ArduinoJson.h>     // make json format
#include <Preferences.h>     // To store users' settings
#include <BH1750.h>          // For LUX measurement of sunlight


#define RXD2 20
#define TXD2 21
#define RS485_EN 4
#define LED_PIN 2

#define STX 0x02
#define ETX 0x03

// -- Modbus setup
HardwareSerial rs485(1);
unsigned long lastModbusSend = 0;
String rs485Buffer = "";
bool receivingPacket = false;
uint16_t holdingRegisters[10];

// -- NTC Setup
#define NTC_PIN 0               // ADC pin
#define FIXED_RESISTOR 10000.0  // 10k fixed resistor
#define R0 10000.0              // NTC resistance at 25°C
#define BETA 3435.0             // 3950 (common value)
#define T0 298.15               // 25°C in Kelvin
#define OFFSET 0.0              // Adjust later (+ or -)
#define ADC_RESOLUTION 4095.0   // 12 bits (1111 1111 1111)
#define VREF 3.3                // Maximum sensing voltage of ESP32
unsigned long lastNTCReadTime = 0;
float ntcSampleSum = 0;
uint8_t ntcSampleCount = 0;

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
uint32_t luxValue = 0;

// -- Serial Output
unsigned long lastSerialPrint = 0;

// -- Feedback LED controll
bool ledState = false;
unsigned long ledStartTime = 0;
const unsigned long ledDuration = 100;


//--------------------------------
//Global variables for user's settings
//--------------------------------
Preferences preferences;
struct DeviceSettings {
  float ntcResistance;   // 10K in general
  float betaConstant;    // 3435 in general
  float ntcOffset;       // To fix error
  uint16_t ntcInterval;  // Reading interval for NTC sensor

  uint16_t luxInterval;  // Reading interval for BH1750 sensor

  uint8_t enableRS485;      // 0 = disabled, 1 = enabled
  uint8_t modbusDeviceID;   // 1–247
  uint32_t modbusBaudRate;  // 9600, 19200, 38400 etc
  uint16_t modbusInterval;  // seconds (share interval)
};
DeviceSettings settings;

//--------------------------------
//Default user's settings
//--------------------------------
void setDefaultSettings() {
  settings.ntcResistance = 10000.0;
  settings.betaConstant = 3435.0;
  settings.ntcOffset = 0.0;
  settings.ntcInterval = 1;

  settings.luxInterval = 1;

  settings.enableRS485 = 1;
  settings.modbusDeviceID = 1;
  settings.modbusBaudRate = 115200;
  settings.modbusInterval = 5;
}

//--------------------------------
//Loading user's settings
//--------------------------------
void loadSettings() {

  Serial.println("\n========== LOADING SETTINGS ==========");

  preferences.begin("device", true);  // read-only

  settings.ntcResistance = preferences.getFloat("ntcR", 10000.0);
  settings.betaConstant = preferences.getFloat("beta", 3435.0);
  settings.ntcOffset = preferences.getFloat("ntcOff", 0.0);
  settings.ntcInterval = preferences.getUInt("ntcInt", 1);

  settings.luxInterval = preferences.getUInt("luxInt", 1);

  settings.enableRS485 = preferences.getUChar("rsEn", 1);
  settings.modbusDeviceID = preferences.getUChar("rsID", 1);
  settings.modbusBaudRate = preferences.getULong("rsBaud", 115200);
  settings.modbusInterval = preferences.getUInt("rsInt", 5);

  preferences.end();

  // -------- PRINT LOADED SETTINGS --------
  Serial.println("NTC Resistance        : " + String(settings.ntcResistance));
  Serial.println("Beta Constant         : " + String(settings.betaConstant));
  Serial.println("NTC Offset            : " + String(settings.ntcOffset));
  Serial.println("NTC Interval (sec)    : " + String(settings.ntcInterval));

  Serial.println("Lux Interval (sec)    : " + String(settings.luxInterval));

  Serial.println("RS485 Enabled         : " + String(settings.enableRS485));
  Serial.println("Modbus Device ID      : " + String(settings.modbusDeviceID));
  Serial.println("Modbus Baud Rate      : " + String(settings.modbusBaudRate));
  Serial.println("Modbus Interval (sec) : " + String(settings.modbusInterval));

  Serial.println("=======================================\n");
}

//--------------------------------
//Save user's settings
//--------------------------------
void saveSettings() {
  preferences.begin("device", false);  // write mode

  preferences.putFloat("ntcR", settings.ntcResistance);
  preferences.putFloat("beta", settings.betaConstant);
  preferences.putFloat("ntcOff", settings.ntcOffset);
  preferences.putUInt("ntcInt", settings.ntcInterval);

  preferences.putUInt("luxInt", settings.luxInterval);

  preferences.putUChar("rsEn", settings.enableRS485);
  preferences.putUChar("rsID", settings.modbusDeviceID);
  preferences.putULong("rsBaud", settings.modbusBaudRate);
  preferences.putUInt("rsInt", settings.modbusInterval);

  preferences.end();
  Serial.println("Settings Saved to NVS");
}


void triggerLED() {
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  ledStartTime = millis();
}
void updateLED() {
  if (ledState && millis() - ledStartTime >= ledDuration) {
    digitalWrite(LED_PIN, LOW);
    ledState = false;
  }
}

// void handleRS485() {

//   if (!settings.enableRS485) return;

//   unsigned long currentMillis = millis();

//   if (currentMillis - lastModbusSend < settings.modbusInterval * 1000UL)
//     return;

//   lastModbusSend = currentMillis;

//   // -------- SEND SENSOR DATA --------
//   StaticJsonDocument<256> doc;
//   doc["id"] = settings.modbusDeviceID;
//   doc["ntc"] = ntcTemp;
//   doc["lux"] = luxValue;

//   String output;
//   serializeJson(doc, output);

//   sendPacket(output);

//   delayMicroseconds(200);  // allow bus settle

//   // -------- WAIT FOR RESPONSE --------
//   unsigned long startWait = millis();

//   while (millis() - startWait < 200) {

//     while (rs485.available()) {

//       byte incoming = rs485.read();

//       if (incoming == STX) {
//         rs485Buffer = "";
//         receivingPacket = true;
//       }

//       else if (incoming == ETX && receivingPacket) {

//         receivingPacket = false;

//         StaticJsonDocument<256> doc;

//         DeserializationError err = deserializeJson(doc, rs485Buffer);

//         if (!err) {

//           if (doc.containsKey("interval"))
//             settings.modbusInterval = doc["interval"];

//           if (doc.containsKey("ntcOffset"))
//             settings.ntcOffset = doc["ntcOffset"];

//           saveSettings();

//           triggerLED();  // feedback LED
//         }

//         rs485Buffer = "";
//       }

//       else if (receivingPacket) {
//         rs485Buffer += (char)incoming;
//       }
//     }
//   }
// }

void printHex(uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// ------------ RS485 Control ------------
void setTransmitMode() {
  digitalWrite(RS485_EN, HIGH);
}
void setReceiveMode() {
  digitalWrite(RS485_EN, LOW);
}

uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];

    for (int i = 0; i < 8; i++) {

      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else
        crc >>= 1;
    }
  }
  return crc;
}

void updateRegisters() {
  holdingRegisters[0] = ntcTemp * 100;
  holdingRegisters[1] = luxValue;
  holdingRegisters[2] = luxConnected;
  holdingRegisters[3] = settings.modbusInterval;
}

void handleModbus() {

  if (!settings.enableRS485) return;

  static uint8_t buffer[64];
  static uint8_t index = 0;

  while (rs485.available()) {

    buffer[index++] = rs485.read();

    if (index >= 8) {

      Serial.println("\n----- MODBUS REQUEST RECEIVED -----");
      Serial.print("Raw Packet: ");
      printHex(buffer, index);

      uint16_t receivedCRC =
        buffer[index - 2] | (buffer[index - 1] << 8);

      uint16_t calcCRC = modbusCRC(buffer, index - 2);

      if (receivedCRC != calcCRC) {
        Serial.println("❌ CRC ERROR - Packet ignored");
        index = 0;
        return;
      }

      Serial.println("✔ CRC OK");

      uint8_t deviceID = buffer[0];
      uint8_t function = buffer[1];

      Serial.print("Device ID : ");
      Serial.println(deviceID);

      Serial.print("Function  : ");
      Serial.println(function, HEX);

      if (deviceID != settings.modbusDeviceID) {
        Serial.println("⚠ Not my device ID - Ignored");
        index = 0;
        return;
      }

      if (function == 0x03) {

        uint16_t startReg =
          (buffer[2] << 8) | buffer[3];

        uint16_t regCount =
          (buffer[4] << 8) | buffer[5];

        Serial.print("Start Register : ");
        Serial.println(startReg);

        Serial.print("Register Count : ");
        Serial.println(regCount);

        updateRegisters();

        uint8_t response[64];

        response[0] = deviceID;
        response[1] = 0x03;
        response[2] = regCount * 2;

        Serial.println("\nSending Register Values:");

        for (int i = 0; i < regCount; i++) {

          uint16_t value =
            holdingRegisters[startReg + i];

          Serial.print("Reg ");
          Serial.print(startReg + i);
          Serial.print(" = ");
          Serial.println(value);

          response[3 + i * 2] = value >> 8;
          response[4 + i * 2] = value & 0xFF;
        }

        uint16_t crc =
          modbusCRC(response, 3 + regCount * 2);

        response[3 + regCount * 2] = crc & 0xFF;
        response[4 + regCount * 2] = crc >> 8;

        Serial.println("\n----- MODBUS RESPONSE -----");
        Serial.print("Response Packet: ");
        printHex(response, 5 + regCount * 2);

        setTransmitMode();

        rs485.write(response, 5 + regCount * 2);
        rs485.flush();

        setReceiveMode();

        Serial.println("✔ Response Sent Successfully");

        triggerLED();
      }

      index = 0;
    }
  }
}

// // ------------ Send Packet ------------
// void sendPacket(String payload) {

//   setTransmitMode();

//   rs485.write(STX);
//   rs485.print(payload);
//   rs485.write(ETX);

//   rs485.flush();

//   setReceiveMode();
// }

// // ------------ Send Sensor JSON ------------
// void sendSensorData() {
//   StaticJsonDocument<200> doc;
//   doc["ntc"] = ntcTemp;
//   doc["lux"] = luxValue;

//   String output;
//   serializeJson(doc, output);

//   sendPacket(output);
// }

// // ------------ Receive Settings ------------
// void receivePacket() {
//   static String buffer = "";
//   static bool receiving = false;

//   while (rs485.available()) {
//     byte incoming = rs485.read();

//     if (incoming == STX) {
//       buffer = "";
//       receiving = true;
//     }
//     else if (incoming == ETX && receiving) {
//       receiving = false;

//       StaticJsonDocument<200> doc;
//       if (!deserializeJson(doc, buffer)) {
//         sendInterval = doc["interval"] * 1000UL;
//         triggerLED();
//       }
//     }
//     else if (receiving) {
//       buffer += (char)incoming;
//     }
//   }
// }

//--------------------------------
// Read averaged voltage
//--------------------------------
float readNTCVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogReadMilliVolts(NTC_PIN);
  }
  float avgMilliVolt = sum / 50.0;
  return avgMilliVolt / 1000.0;  // convert to volts
}

//--------------------------------
// Calculate temperature
//--------------------------------
void calculateNTCFromADC() {
  float voltage = readNTCVoltage();

  if (voltage <= 0.001) {
    ntcTemp = 0.0;
    return;
  }

  float rNTC = FIXED_RESISTOR * (VREF - voltage) / voltage;
  float tempK = 1.0 / ((1.0 / T0) + (1.0 / settings.betaConstant) * log(rNTC / settings.ntcResistance));
  ntcTemp = tempK - 273.15;
  ntcTemp += settings.ntcOffset;
}

//--------------------------------
// Handle NTC according to user
//--------------------------------
void handleNTC() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastNTCReadTime >= settings.ntcInterval * 1000UL) {
    lastNTCReadTime = currentMillis;
    calculateNTCFromADC();
  }
}

//--------------------------------
//LUX value calculation
//--------------------------------
void handleLUX() {
  unsigned long currentMillis = millis();

  // ---- Interval Control ----
  if (currentMillis - lastLuxReadTime < settings.luxInterval * 1000UL)
    return;
  lastLuxReadTime = currentMillis;

  // ---- Read Sensor ----
  if (luxConnected) {
    luxValue = lightMeter.readLightLevel();
  } else {
    luxValue = 1;  // fallback value if sensor missing
  }
}

//--------------------------------
//Serial output control
//--------------------------------
void serialOutput() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialPrint < 1000)
    return;
  lastSerialPrint = currentMillis;

  Serial.println("\n========================================");
  Serial.println("           Sensor Data Report");
  Serial.println("========================================");
  Serial.printf("NTC Temperature  : %8.2f °C\n", ntcTemp);
  Serial.printf("Light Intensity  : %8lu lux\n", luxValue);
  Serial.println("========================================\n");
}

//--------------------------------
//Setup function
//--------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(RS485_EN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(NTC_PIN, INPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // Full 0–3.3V range

  digitalWrite(LED_PIN, LOW);

  // --- Storage Init ---
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
  delay(1000);

  // --- LUX Sensor Init ---
  Serial.println("\n==============================");
  Serial.println("BH1750 Initialization");
  Serial.println("==============================");
  Serial.println("[INFO] Starting BH1750...");

  Wire.begin();  // SDA, SCL default for ESP32
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    luxConnected = true;
    Serial.println("[SUCCESS] BH1750 detected.");
    Serial.println("[INFO] Mode: Continuous High Resolution");
    Serial.println("==============================\n");
  } else {
    luxConnected = false;
    Serial.println("[ERROR] BH1750 not detected!");
    Serial.println("[INFO] Check SDA/SCL wiring.");
    Serial.println("==============================\n");
  }
  delay(1000);


  setReceiveMode();
  rs485.begin(settings.modbusBaudRate, SERIAL_8N1, RXD2, TXD2);
}


//--------------------------------
//Loop function
//--------------------------------
void loop() {
  handleNTC();
  handleLUX();
  handleModbus();
  serialOutput();
  updateLED();
}