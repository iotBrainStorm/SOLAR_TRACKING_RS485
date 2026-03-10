#include <Arduino.h>
#include <math.h>
#include <BH1750.h>
#include <ArduinoJson.h>

#define RXD2 20
#define TXD2 21
#define RS485_EN 4
#define LED_PIN 2

#define STX 0x02
#define ETX 0x03

// HardwareSerial rs485(1);

// -- NTC Setup
#define NTC_PIN 0              // ADC pin
#define FIXED_RESISTOR 10000.0  // 10k fixed resistor
#define R0 10000.0               // NTC resistance at 25°C
#define BETA 3435.0             // 3950 (common value)
#define T0 298.15               // 25°C in Kelvin
#define OFFSET 0.0            // Adjust later (+ or -)
#define ADC_RESOLUTION 4095.0   // 12 bits (1111 1111 1111)
#define VREF 3.3               // Maximum sensing voltage of ESP32
unsigned long lastNTCReadTime = 0;
float ntcSampleSum = 0;
uint8_t ntcSampleCount = 0;

// -- Light sensor (LUX) setup
BH1750 lightMeter;
unsigned long lastLuxReadTime = 0;
unsigned long lastLuxSaveTime = 0;
float luxFiltered = 0;  // EMA filtered value
bool luxInitialized = false;
float previousLux = 0;
long luxDiff = 0;

// -- Sensor Values Storage
float ntcTemp = 0.0;
uint32_t luxValue = 0;
uint8_t sunlightPercentage = 0;

// -- Serial Output
unsigned long lastSerialPrint = 0;

// unsigned long sendInterval = 5000;
// unsigned long lastSendTime = 0;

// // ------------ Led Control ------------

// bool ledState = false;
// unsigned long ledStartTime = 0;
// const unsigned long ledDuration = 50;

// void triggerLED() {
//   digitalWrite(LED_PIN, HIGH);
//   ledState = true;
//   ledStartTime = millis();
// }
// void updateLED() {
//   if (ledState && millis() - ledStartTime >= ledDuration) {
//     digitalWrite(LED_PIN, LOW);
//     ledState = false;
//   }
// }

// // ------------ RS485 Control ------------
// void setTransmitMode() { digitalWrite(RS485_EN, HIGH); }
// void setReceiveMode()  { digitalWrite(RS485_EN, LOW); }

// // ------------ Fake Sensor Read ------------
// void readSensors() {
//   ntcTemp = random(250, 350) / 10.0;
//   luxValue = random(100, 1000);
// }

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
float readNTCVoltage()
{
  uint32_t sum = 0;

  for (int i = 0; i < 20; i++)
  {
    sum += analogReadMilliVolts(NTC_PIN);
  }

  float avgMilliVolt = sum / 20.0;

  return avgMilliVolt / 1000.0;   // convert to volts
}

//--------------------------------
// Calculate temperature
//--------------------------------
void calculateNTCFromADC()
{
  float voltage = readNTCVoltage();

  if (voltage <= 0.001)
  {
    ntcTemp = -100;
    return;
  }

  float rNTC = FIXED_RESISTOR * (VREF - voltage) / voltage;

  float tempK = 1.0 / ((1.0 / T0) + (1.0 / BETA) * log(rNTC / R0));

  ntcTemp = tempK - 273.15;

  ntcTemp += OFFSET;
}

//--------------------------------
// Handle NTC every 1 second
//--------------------------------
void handleNTC()
{
  unsigned long currentMillis = millis();

  if (currentMillis - lastNTCReadTime >= 1000)
  {
    lastNTCReadTime = currentMillis;

    calculateNTCFromADC();

  }
}

//////////////////////   CALCULATE LUX   //////////////////////

void handleLUX() {
  unsigned long currentMillis = millis();

  // ---- Interval Control ----
  if (currentMillis - lastLuxReadTime < 1000)
    return;
  lastLuxReadTime = currentMillis;

  // ---- Read Sensor ----
  luxValue = lightMeter.readLightLevel();

}

//////////////////////   SERIAL OUTPUT   //////////////////////

void serialOutput() {

  unsigned long currentMillis = millis();

  // Run every 1 second (1000 ms)
  if (currentMillis - lastSerialPrint < 1000)
    return;
  lastSerialPrint = currentMillis;

  Serial.println("\n========================================");
  Serial.println("           Sensor Data Report");
  Serial.println("========================================");

  Serial.printf("NTC Temperature  : %8.2f °C\n", ntcTemp);
  // Serial.printf("AHT Temperature  : %8.2f °C\n", ahtTemp);
  // Serial.printf("Humidity         : %8.2f %%\n", humidity);

  Serial.println("----------------------------------------");

  Serial.printf("Light Intensity  : %8lu lux\n", luxValue);
  // Serial.printf("Filtered Lux     : %8.2f lux\n", luxFiltered);
  // Serial.printf("Sunlight Level   : %8u %%\n", sunlightPercentage);

  Serial.println("----------------------------------------");

  // float tempDiff = fabs(ntcTemp - ahtTemp);
  // Serial.printf("Temp Difference  : %8.2f °C\n", tempDiff);

  Serial.println("========================================\n");
}

void setup() {
  Serial.begin(115200);

  pinMode(RS485_EN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(NTC_PIN, INPUT);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // Full 0–3.3V range

  digitalWrite(LED_PIN, LOW);

    // --- LUX Sensor Init ---
  // u8g2.clearBuffer();
  Serial.println("\n==============================");
  Serial.println("BH1750 Initialization");
  Serial.println("==============================");
  Serial.println("[INFO] Starting BH1750...");

  Wire.begin();  // SDA, SCL default for ESP32
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    // u8g2.drawStr(0, 18, "BH1750: OK");
    Serial.println("[SUCCESS] BH1750 detected.");
    Serial.println("[INFO] Mode: Continuous High Resolution");
    Serial.println("==============================\n");
  } else {
    // u8g2.drawStr(0, 18, "BH1750: ERROR");
    Serial.println("[ERROR] BH1750 not detected!");
    Serial.println("[INFO] Check SDA/SCL wiring.");
    Serial.println("==============================\n");
  }
  // u8g2.sendBuffer();
  delay(1000);


  // setReceiveMode();

  // rs485.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop() {

handleNTC();
handleLUX();
serialOutput();

  // unsigned long currentMillis = millis();

  // if (currentMillis - lastSendTime >= sendInterval) {
  //   lastSendTime = currentMillis;
  //   readSensors();
  //   sendSensorData();
  // }

  // receivePacket();
  // updateLED();
}