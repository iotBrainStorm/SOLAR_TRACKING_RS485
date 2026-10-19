#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <BH1750.h>
#include <esp_task_wdt.h>
#include <Ticker.h>

#define RXD2 20
#define TXD2 21
#define RS485_EN 4
#define LED_PIN 2

// -- Safe Sensor Status LED Pins (ESP32-C3)
#define NTC_LED_PIN 6
#define LUX_LED_PIN 7
const uint32_t SENSOR_LED_BLINK_MS = 40;  // 40ms crisp, non-blocking flash

Ticker ntcLedTicker;
Ticker luxLedTicker;

// -- Modbus setup
HardwareSerial rs485(1);
uint16_t holdingRegisters[10];

// -------- RS485 Protection --------
unsigned long lastByteTime = 0;
unsigned long lastValidPacket = 0;
const uint32_t BUS_WATCHDOG = 30000;  // 30s recovery
uint8_t crcErrorCount = 0;
const uint8_t CRC_LIMIT = 5;

// -- NTC Setup
#define NTC_PIN 0
#define FIXED_RESISTOR 10000.0f
#define T0 298.15f
#define VREF 3.3f
unsigned long lastNTCReadTime = 0;

// -- Light sensor (LUX) setup
BH1750 lightMeter;
bool luxConnected = false;
unsigned long lastLuxReadTime = 0;
float luxFiltered = 0;
bool luxInitialized = false;
uint8_t luxFailCount = 0;
const uint8_t LUX_FAIL_LIMIT = 5;

// -- Sensor Values Storage
float ntcTemp = 0.0;
uint32_t luxValue = 0;

// -- Serial Output
unsigned long lastSerialPrint = 0;

// -- Feedback LED control (Modbus)
bool ledState = false;
unsigned long ledStartTime = 0;
unsigned long ledDuration = 100;
const unsigned long ledDurationRX = 60;
const unsigned long ledDurationTX = 120;

//--------------------------------
// Global variables for user's settings
//--------------------------------
Preferences preferences;
struct DeviceSettings {
  float ntcResistance;
  float betaConstant;
  float ntcOffset;
  uint16_t ntcInterval;
  uint16_t luxInterval;
  uint8_t enableRS485;
  uint8_t modbusDeviceID;
  uint32_t modbusBaudRate;
  uint16_t modbusInterval;
};
DeviceSettings settings;

//--------------------------------
// Non-Blocking Sensor LED Callbacks (Ticker)
//--------------------------------
void turnOffNtcLED() {
  digitalWrite(NTC_LED_PIN, LOW);
}

void turnOffLuxLED() {
  digitalWrite(LUX_LED_PIN, LOW);
}

void blinkNtcLED() {
  digitalWrite(NTC_LED_PIN, HIGH);
  ntcLedTicker.once_ms(SENSOR_LED_BLINK_MS, turnOffNtcLED);
}

void blinkLuxLED() {
  digitalWrite(LUX_LED_PIN, HIGH);
  luxLedTicker.once_ms(SENSOR_LED_BLINK_MS, turnOffLuxLED);
}

void sanitizeSettings() {
  if (settings.ntcResistance <= 0) settings.ntcResistance = 10000.0f;
  if (settings.betaConstant <= 0) settings.betaConstant = 3435.0f;
  if (settings.ntcInterval < 1) settings.ntcInterval = 1;
  if (settings.luxInterval < 1) settings.luxInterval = 1;
  if (settings.modbusDeviceID < 1 || settings.modbusDeviceID > 247) settings.modbusDeviceID = 1;
  if (settings.modbusBaudRate == 0) settings.modbusBaudRate = 115200;
  if (settings.modbusInterval < 1) settings.modbusInterval = 1;
  settings.enableRS485 = 1;
}

void setDefaultSettings() {
  settings.ntcResistance = 10000.0f;
  settings.betaConstant = 3435.0f;
  settings.ntcOffset = 0.0f;
  settings.ntcInterval = 1;
  settings.luxInterval = 1;
  settings.enableRS485 = 1;
  settings.modbusDeviceID = 1;
  settings.modbusBaudRate = 115200;
  settings.modbusInterval = 2;
}

void loadSettings() {
  Serial.println("\n========== LOADING SETTINGS ==========");
  preferences.begin("device", true);

  settings.ntcResistance = preferences.getFloat("ntcR", 10000.0f);
  settings.betaConstant = preferences.getFloat("beta", 3435.0f);
  settings.ntcOffset = preferences.getFloat("ntcOff", 0.0f);
  settings.ntcInterval = preferences.getUInt("ntcInt", 1);
  settings.luxInterval = preferences.getUInt("luxInt", 1);
  settings.enableRS485 = 1;
  settings.modbusDeviceID = preferences.getUChar("rsID", 1);
  settings.modbusBaudRate = preferences.getULong("rsBaud", 115200);
  settings.modbusInterval = preferences.getUInt("rsInt", 2);

  preferences.end();
  sanitizeSettings();
  Serial.println("=======================================\n");
}

void saveSettings() {
  sanitizeSettings();
  preferences.begin("device", false);

  preferences.putFloat("ntcR", settings.ntcResistance);
  preferences.putFloat("beta", settings.betaConstant);
  preferences.putFloat("ntcOff", settings.ntcOffset);
  preferences.putUInt("ntcInt", settings.ntcInterval);
  preferences.putUInt("luxInt", settings.luxInterval);
  preferences.putUChar("rsEn", 1);
  preferences.putUChar("rsID", settings.modbusDeviceID);
  preferences.putULong("rsBaud", settings.modbusBaudRate);
  preferences.putUInt("rsInt", settings.modbusInterval);

  preferences.end();
  Serial.println("Settings Saved to NVS");
}

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

// ------------ RS485 Control ------------
void setTransmitMode() {
  digitalWrite(RS485_EN, HIGH);
}
void setReceiveMode() {
  digitalWrite(RS485_EN, LOW);
}

void resetRS485UART() {
  Serial.println("[RS485] Resetting UART peripheral...");
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
  Serial.println("[RS485] UART reset complete");
}

uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void updateRegisters() {
  int16_t scaledTemp = (int16_t)round(ntcTemp * 100.0f);
  holdingRegisters[0] = (uint16_t)scaledTemp;
  holdingRegisters[1] = (luxValue >> 16) & 0xFFFF;  // lux high word
  holdingRegisters[2] = luxValue & 0xFFFF;          // lux low word
  holdingRegisters[3] = luxConnected ? 1 : 0;
  holdingRegisters[4] = settings.modbusInterval;
}

void handleModbus() {
  if (!settings.enableRS485)
    return;

  static uint8_t buffer[64];
  static uint8_t index = 0;

  while (rs485.available()) {
    if (index < sizeof(buffer)) {
      buffer[index++] = rs485.read();
    } else {
      index = 0;
    }
    lastByteTime = millis();
  }

  uint32_t busTimeout = max(5UL, 40000UL / settings.modbusBaudRate + 2);

  if (index > 0 && millis() - lastByteTime > busTimeout) {
    // Locate valid start of frame [modbusDeviceID][0x03 or 0x10] to strip any leading noise bytes
    int startIdx = -1;
    for (int i = 0; i <= (int)index - 8; i++) {
      if (buffer[i] == settings.modbusDeviceID && (buffer[i + 1] == 0x03 || buffer[i + 1] == 0x10)) {
        startIdx = i;
        break;
      }
    }

    if (startIdx < 0) {
      index = 0;
      return;
    }

    uint8_t *pkt = &buffer[startIdx];
    uint8_t pktLen = index - startIdx;
    uint8_t function = pkt[1];

    uint8_t expectedLen = 8;
    if (function == 0x10) {
      expectedLen = 9 + pkt[6];  // 9 + 18 = 27 bytes
    }

    if (pktLen < expectedLen) {
      index = 0;
      return;
    }

    uint16_t receivedCRC = pkt[expectedLen - 2] | (pkt[expectedLen - 1] << 8);
    uint16_t calcCRC = modbusCRC(pkt, expectedLen - 2);

    if (receivedCRC != calcCRC) {
      Serial.println("[MODBUS] CRC ERROR");
      crcErrorCount++;
      if (crcErrorCount >= CRC_LIMIT) {
        resetRS485UART();
        crcErrorCount = 0;
      }
      index = 0;
      return;
    }

    triggerLED(ledDurationRX);
    crcErrorCount = 0;
    lastValidPacket = millis();

    uint8_t deviceID = pkt[0];

    // ---- Function 0x03: Read Holding Registers ----
    if (function == 0x03) {
      uint16_t startReg = (pkt[2] << 8) | pkt[3];
      uint16_t regCount = (pkt[4] << 8) | pkt[5];

      if (regCount == 0 || regCount > 10 || (startReg + regCount) > 10) {
        index = 0;
        return;
      }

      updateRegisters();

      uint8_t response[32];
      response[0] = deviceID;
      response[1] = 0x03;
      response[2] = regCount * 2;

      for (uint16_t i = 0; i < regCount; i++) {
        uint16_t value = holdingRegisters[startReg + i];
        response[3 + i * 2] = value >> 8;
        response[4 + i * 2] = value & 0xFF;
      }

      uint16_t crc = modbusCRC(response, 3 + regCount * 2);
      response[3 + regCount * 2] = crc & 0xFF;
      response[4 + regCount * 2] = crc >> 8;

      while (rs485.available())
        rs485.read();
      delayMicroseconds(200);

      setTransmitMode();
      triggerLED(ledDurationTX);
      delayMicroseconds(200);
      rs485.write(response, 5 + regCount * 2);
      rs485.flush();
      delayMicroseconds(500);
      setReceiveMode();
    }

    // ---- Function 0x10: Write Multiple Registers (Settings from Master) ----
    else if (function == 0x10) {
      uint16_t regCount = (pkt[4] << 8) | pkt[5];
      uint8_t byteCount = pkt[6];

      if (regCount != 9 || byteCount != 18) {
        index = 0;
        return;
      }

      uint32_t oldBaudRate = settings.modbusBaudRate;

      int di = 7;
      settings.ntcResistance = (float)((pkt[di] << 8) | pkt[di + 1]);
      di += 2;
      settings.betaConstant = (float)((pkt[di] << 8) | pkt[di + 1]);
      di += 2;
      settings.ntcOffset = (int16_t)((pkt[di] << 8) | pkt[di + 1]) / 100.0f;
      di += 2;
      settings.ntcInterval = (pkt[di] << 8) | pkt[di + 1];
      di += 2;
      settings.luxInterval = (pkt[di] << 8) | pkt[di + 1];
      di += 2;
      settings.modbusDeviceID = (pkt[di] << 8) | pkt[di + 1];
      di += 2;
      uint16_t baudHi = (pkt[di] << 8) | pkt[di + 1];
      di += 2;
      uint16_t baudLo = (pkt[di] << 8) | pkt[di + 1];
      di += 2;
      settings.modbusBaudRate = ((uint32_t)baudHi << 16) | baudLo;
      settings.modbusInterval = (pkt[di] << 8) | pkt[di + 1];
      settings.enableRS485 = 1;

      saveSettings();

      // Echo confirmation using the deviceID that received the request
      uint8_t response[8];
      response[0] = deviceID;
      response[1] = 0x10;
      response[2] = pkt[2];
      response[3] = pkt[3];
      response[4] = pkt[4];
      response[5] = pkt[5];

      uint16_t crc = modbusCRC(response, 6);
      response[6] = crc & 0xFF;
      response[7] = crc >> 8;

      while (rs485.available())
        rs485.read();
      delayMicroseconds(200);

      setTransmitMode();
      triggerLED(1000);
      delayMicroseconds(200);
      rs485.write(response, 8);
      rs485.flush();
      delayMicroseconds(500);
      setReceiveMode();

      // If baud rate changed, switch UART after sending confirmation
      if (settings.modbusBaudRate != oldBaudRate) {
        delay(10);
        rs485.updateBaudRate(settings.modbusBaudRate);
        Serial.printf("[RS485] Switched baud rate to %lu\n", settings.modbusBaudRate);
      }
    }

    index = 0;
  }
}

//--------------------------------
// Read averaged voltage & calculate NTC safely
//--------------------------------
float readNTCVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogReadMilliVolts(NTC_PIN);
  }
  return (sum / 50.0f) / 1000.0f;
}

void calculateNTCFromADC() {
  float voltage = readNTCVoltage();

  // Guard against open/shorted thermistor or invalid parameters (prevents NaN / log(0) crash)
  if (voltage <= 0.05f || voltage >= (VREF - 0.05f) || settings.ntcResistance <= 0.0f || settings.betaConstant <= 0.0f) {
    ntcTemp = 0.0f;
    return;
  }

  float rNTC = FIXED_RESISTOR * (VREF - voltage) / voltage;
  if (rNTC <= 0.0f) {
    ntcTemp = 0.0f;
    return;
  }

  float tempK = 1.0f / ((1.0f / T0) + (1.0f / settings.betaConstant) * logf(rNTC / settings.ntcResistance));
  float calcC = (tempK - 273.15f) + settings.ntcOffset;

  if (!isnan(calcC) && !isinf(calcC)) {
    ntcTemp = constrain(calcC, -50.0f, 200.0f);
    blinkNtcLED();  // Blink NTC LED only on a valid temperature read
  }
}

void handleNTC() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastNTCReadTime >= settings.ntcInterval * 1000UL) {
    lastNTCReadTime = currentMillis;
    calculateNTCFromADC();
  }
}

//--------------------------------
// I2C Bus Recovery & BH1750 Fallback
//--------------------------------
void recoverI2CBus() {
  Serial.println("[I2C] Attempting bus recovery...");
  Wire.end();
  delay(5);

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }

  // Generate STOP condition
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);

  Wire.begin();
  Wire.setTimeOut(100);  // Fast 100ms timeout so Modbus polling is never blocked
  Serial.println("[I2C] Bus recovery done");
}

void reinitBH1750() {
  Serial.println("[BH1750] Re-initializing sensor...");
  recoverI2CBus();
  delay(20);

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    lightMeter.setMTreg(31);
    luxConnected = true;
    luxFailCount = 0;
    Serial.println("[BH1750] Re-init successful");
  } else {
    luxConnected = false;
    Serial.println("[BH1750] Re-init FAILED");
  }
}

void handleLUX() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastLuxReadTime < settings.luxInterval * 1000UL)
    return;
  lastLuxReadTime = currentMillis;

  if (luxConnected) {
    float rawLux = lightMeter.readLightLevel();

    if (rawLux < 0 || isnan(rawLux) || isinf(rawLux)) {
      luxFailCount++;
      Serial.printf("[BH1750] Read failed (%u/%u)\n", luxFailCount, LUX_FAIL_LIMIT);
      if (luxFailCount >= LUX_FAIL_LIMIT) {
        reinitBH1750();
      }
      return;
    }

    luxFailCount = 0;

    if (!luxInitialized) {
      luxFiltered = rawLux;
      luxInitialized = true;
    } else {
      luxFiltered = 0.2f * rawLux + 0.8f * luxFiltered;
    }

    luxValue = (uint32_t)(luxFiltered + 0.5f);
    blinkLuxLED();  // Blink LUX LED only on a valid BH1750 read
  } else {
    luxValue = 1;

    static unsigned long lastReInitAttempt = 0;
    if (currentMillis - lastReInitAttempt > 30000UL) {
      lastReInitAttempt = currentMillis;
      reinitBH1750();
    }
  }
}

void serialOutput() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialPrint < 2000)
    return;
  lastSerialPrint = currentMillis;

  Serial.println("\n========================================");
  Serial.println("           Sensor Data Report");
  Serial.println("========================================");
  Serial.printf("NTC Temperature  : %.2f °C\n", ntcTemp);
  Serial.printf("Light Intensity  : %lu lux\n", luxValue);
  Serial.println("========================================\n");
}

void setup() {
  Serial.begin(115200);

  pinMode(RS485_EN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(NTC_LED_PIN, OUTPUT);
  pinMode(LUX_LED_PIN, OUTPUT);
  pinMode(NTC_PIN, INPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(NTC_LED_PIN, LOW);
  digitalWrite(LUX_LED_PIN, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  delay(1000);

  loadSettings();

  Wire.begin();
  Wire.setTimeOut(100);  // 100ms I2C timeout prevents blocking Modbus
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    luxConnected = true;
    lightMeter.setMTreg(31);
    Serial.println("[SUCCESS] BH1750 detected.");
  } else {
    luxConnected = false;
    Serial.println("[ERROR] BH1750 not detected!");
  }

  setReceiveMode();
  rs485.begin(settings.modbusBaudRate, SERIAL_8N1, RXD2, TXD2);
  rs485.setTimeout(2);
  lastValidPacket = millis();

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 10000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.println("[WDT] Hardware watchdog enabled (10s)");
}

void loop() {
  esp_task_wdt_reset();

  handleNTC();
  handleLUX();
  handleModbus();
  serialOutput();
  updateLED();

  if (settings.enableRS485 && millis() - lastValidPacket > BUS_WATCHDOG) {
    Serial.println("[RS485] No communication for 30s -> recovering bus");
    resetRS485UART();
    lastValidPacket = millis();
  }
}