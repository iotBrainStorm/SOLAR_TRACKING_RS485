<p align="center">
  <img src="https://img.shields.io/badge/Platform-ESP32-blue?style=for-the-badge&logo=espressif&logoColor=white" />
  <img src="https://img.shields.io/badge/Protocol-RS485%20Modbus-orange?style=for-the-badge" />
  <img src="https://img.shields.io/badge/IDE-Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white" />
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" />
</p>

<h1 align="center">🌞 Solar Panel Smart Monitoring System</h1>

<p align="center">
  <b>Dual ESP32 | RS485 Modbus Communication | Web Dashboard | Node-RED Integration</b><br/>
  <i>Industrial-grade solar monitoring with real-time sensor data, OLED display, and full web configuration</i>
</p>

---

## 📸 Project Preview

<p align="center">
  <img src="display.png" alt="OLED Display Output" width="280" style="margin:10px; border-radius:12px; box-shadow: 0 4px 8px rgba(0,0,0,0.3);"/>
  <img src="dashboard.png" alt="Web Dashboard" width="280" style="margin:10px; border-radius:12px;"/>
  <img src="sunmap.png" width="280" style="margin:10px; border-radius:12px;"/>
  <img src="config.png" alt="Configuration Page" width="280" style="margin:10px; border-radius:12px;"/>
</p>

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SOLAR MONITORING SYSTEM                      │
│                                                                     │
│   ┌─────────────────────┐    RS485 Modbus    ┌───────────────────┐  │
│   │   🟢 SENDER (Slave) │◄═════════════════► │ 🔵 RECEIVER      │  │
│   │      ESP32 #1       │    Half-Duplex     │    (Master)       │  │
│   │                     │    115200 baud     │    ESP32 #2       │  │
│   │  • NTC Thermistor   │                    │                   │  │
│   │  • BH1750 LUX       │                    │  • AHT10 Temp/Hum │  │
│   │  • LED Feedback     │                    │  • OLED Display   │  │
│   │                     │                    │  • WiFi + WebUI   │  │
│   │  Reads sensors and  │                    │  • Node-RED Link  │  │
│   │  responds to master │                    │  • LED Feedback   │  │
│   └─────────────────────┘                    └────────┬──────────┘  │
│                                                       │             │
│                                              ┌────────▼──────────┐  │
│                                              │   Web Browser     │  │
│                                              │Dashboard + Config │  │
│                                              └───────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## ⚡ Features

### 📡 Communication

- **RS485 Modbus RTU** half-duplex communication between two ESP32 modules
- **Function 0x03** — Master polls sensor data from slave at configurable intervals
- **Function 0x10** — Master pushes configuration settings to slave with save confirmation
- Automatic **garbage byte stripping** and CRC validation
- LED feedback synchronized on both modules (TX/RX blink)

### 🌡️ Sensor Monitoring

| Parameter             | Sensor             | Module   | Description                      |
| --------------------- | ------------------ | -------- | -------------------------------- |
| Panel Temperature     | NTC 10K Thermistor | Sender   | Direct panel surface temperature |
| Ambient Temperature   | AHT10              | Receiver | Environmental temperature        |
| Humidity              | AHT10              | Receiver | Environmental humidity           |
| Light Intensity (LUX) | BH1750             | Sender   | Sunlight measurement             |
| Sunlight %            | Calculated         | Receiver | Percentage based on min/max LUX  |
| WiFi Strength         | Internal RSSI      | Receiver | Network stability monitoring     |

### 🖥️ Display & Interface

- **128×64 OLED** live display with smart UI layout
- **WiFi Signal Bars** + status icons
- **Sunlight Progress Bar** visualization
- **LUX Trend** indicator (+/-)
- **12H / 24H** clock format support

### 🌐 Web Interface

- **Real-time Dashboard** — all sensor values, WiFi strength, Node-RED status
- **Interactive Sun Map** — real-time sun arc with:
  - NOAA astronomical formula for accurate sun elevation & azimuth angles
  - 45° optimal zone with binary-search crossing times
  - Mini compass SVG showing real sun position (N/E/S/W)
  - Azimuth direction label (e.g., `SE 117°`)
  - Sunrise, Solar Noon, Sunset times
  - Sun stats: Elevation, Sunlight %, LUX, Remaining time
- **Configuration Portal** — adjust all settings from any browser
- **GPS Auto-detect** — one-tap location fetch from device GPS (mobile-friendly)
- **Timezone Dropdown** — 40+ timezone options with country names
- **Save Configuration** — saves to both master & slave NVS simultaneously
- **Factory Reset** — resets both modules to default settings
- **Responsive Design** — optimized for desktop, tablet, and mobile screens
- Fully **async** non-blocking web server

### 🔗 Node-RED Integration

- Configurable HTTP POST data sharing
- Adjustable share interval
- Timeout protection & failure handling
- Enable/Disable toggle from web config

---

## 🔄 How It Works

### Modbus Data Polling (Every `modbusInterval` seconds)

```
  RECEIVER (Master)                              SENDER (Slave)
  ─────────────────                              ────────────────

  1. TX LED blinks
     Sends Modbus request ──────────────────────►
     (Function 0x03, 4 registers)                2. RX LED blinks
                                                    Receives request
                                                    Reads sensors
                                                    Builds response

                          ◄────────────────────── 3. TX LED blinks
                                                    Sends: ntcTemp,
  4. RX LED blinks                                  luxValue,
     Receives response                              luxConnected,
     Updates local values                           modbusInterval

  5. Waits modbusInterval...
     Then repeats from step 1
```

### Settings Sync (On "Save" or "Reset" button click)

```
  RECEIVER (Master)                              SENDER (Slave)
  ─────────────────                              ────────────────

  1. User clicks Save/Reset
     on Web Config Portal

  2. Saves settings to own NVS

  3. TX LED blinks
     Sends settings via ────────────────────────►
     Function 0x10                               4. Receives settings
     (9 registers)                                  Saves to NVS

                          ◄──────────────────────  5. Sends confirmation
  6. Receives confirmation                          (echo Function 0x10)
     LED glows 1 second ✔                          LED glows 1 second ✔
     Serial: "Slave confirmed
     settings saved!"
```

---

## 🔌 Pin Configuration

### RECEIVER (Master) — ESP32

| Component            | Function           | GPIO Pin    |
| -------------------- | ------------------ | ----------- |
| RS485 Module RO (RX) | UART Receive       | **GPIO 16** |
| RS485 Module DI (TX) | UART Transmit      | **GPIO 17** |
| RS485 Module DE/RE   | TX/RX Direction    | **GPIO 4**  |
| OLED Display SDA     | I2C Data           | **GPIO 21** |
| OLED Display SCL     | I2C Clock          | **GPIO 22** |
| AHT10 SDA            | I2C Data (shared)  | **GPIO 21** |
| AHT10 SCL            | I2C Clock (shared) | **GPIO 22** |
| Status LED           | TX/RX Feedback     | **GPIO 2**  |

### SENDER (Slave) — ESP32

| Component            | Function        | GPIO Pin    |
| -------------------- | --------------- | ----------- |
| RS485 Module RO (RX) | UART Receive    | **GPIO 20** |
| RS485 Module DI (TX) | UART Transmit   | **GPIO 21** |
| RS485 Module DE/RE   | TX/RX Direction | **GPIO 4**  |
| BH1750 SDA           | I2C Data        | **GPIO 21** |
| BH1750 SCL           | I2C Clock       | **GPIO 22** |
| NTC Thermistor       | ADC Input       | **GPIO 0**  |
| Status LED           | TX/RX Feedback  | **GPIO 2**  |

> ⚠️ **Note:** On the Sender, GPIO 21 is shared between RS485 TX and I2C SDA — ensure your board variant supports this or remap accordingly.

---

## 🔧 Circuit Diagram

```
                    ┌──────────────┐               ┌──────────────┐
                    │  ESP32 #2    │               │  ESP32 #1    │
                    │  (RECEIVER)  │               │  (SENDER)    │
                    │              │               │              │
                    │  GPIO16 (RX)─┤               ├─GPIO20 (RX)  │
                    │  GPIO17 (TX)─┤               ├─GPIO21 (TX)  │
                    │  GPIO4  (EN)─┤               ├─GPIO4  (EN)  │
                    │              │               │              │
                    │  GPIO21(SDA)─┤               ├─GPIO21(SDA)  │
                    │  GPIO22(SCL)─┤               ├─GPIO22(SCL)  │
                    │              │               │              │
                    │  GPIO2 (LED)─┤               ├─GPIO2 (LED)  │
                    │              │               ├─GPIO0 (NTC)  │
                    └──────┬───────┘               └──────┬───────┘
                           │                              │
                    ┌──────▼───────┐               ┌──────▼───────┐
                    │ MAX485/      │               │ MAX485/      │
                    │ RS485 Module │               │ RS485 Module │
                    │              │               │              │
                    │    A ────────┼───────────────┼──── A        │
                    │    B ────────┼───────────────┼──── B        │
                    │   GND ───────┼───────────────┼─── GND       │
                    └──────────────┘               └──────────────┘

        ┌─────────┐  ┌─────────┐                   ┌─────────┐  ┌───────────┐
        │  OLED   │  │ AHT10   │                   │ BH1750  │  │  NTC 10K  │
        │ SH1106  │  │Temp/Hum │                   │  LUX    │  │Thermistor │
        │ 128x64  │  │         │                   │         │  │    +      │
        │         │  │         │                   │         │  │  10K Res  │
        │ SDA─21  │  │ SDA─21  │                   │ SDA─21  │  │  GPIO0    │
        │ SCL─22  │  │ SCL─22  │                   │ SCL─22  │  │           │
        └─────────┘  └─────────┘                   └─────────┘  └───────────┘

        ◄──────── RECEIVER Side ────────►          ◄─────── SENDER Side ──────►
```

### RS485 Module Wiring (MAX485 / TTL-to-RS485)

```
  ESP32                MAX485 Module
  ─────                ─────────────
  GPIO TX  ──────────► DI  (Data In)
  GPIO RX  ◄────────── RO  (Receive Out)
  GPIO EN  ──────────► DE + RE (tied together)
  3.3V     ──────────► VCC
  GND      ──────────► GND

  Module A ◄──────────► Module A  (twisted pair)
  Module B ◄──────────► Module B  (twisted pair)
  GND      ◄──────────► GND      (common ground)
```

### NTC Thermistor Voltage Divider (Sender)

```
  3.3V ─────┬───── 10K Fixed Resistor ─────┬───── GND
            │                              │
            └─── (Junction) ── GPIO 0      │
                                           │
                              NTC 10K ─────┘
```

---

## ⚙️ Configurable Settings (via Web Portal)

| Setting           | Range       | Default | Description                                 |
| ----------------- | ----------- | ------- | ------------------------------------------- |
| NTC Resistance    | Ohms        | 10000   | NTC nominal resistance                      |
| Beta Constant     | —           | 3435    | NTC beta coefficient                        |
| NTC Offset        | °C          | 0.0     | Temperature calibration                     |
| NTC Interval      | 1+ sec      | 1       | NTC reading frequency                       |
| LUX Interval      | 1+ sec      | 1       | Light reading frequency                     |
| RS485 Enable      | On/Off      | On      | Enable/Disable Modbus polling (master only) |
| Device ID         | 1–247       | 1       | Modbus slave address                        |
| Baud Rate         | 9600–115200 | 115200  | RS485 communication speed                   |
| Modbus Interval   | 1+ sec      | 2       | Polling frequency                           |
| Node-RED Enable   | On/Off      | Off     | Toggle data sharing                         |
| Node-RED IP       | IP Address  | —       | Target server                               |
| Node-RED Port     | 1–65535     | 1880    | Target port                                 |
| Node-RED Interval | 1+ sec      | 10      | Data push frequency                         |
| GMT Offset        | Seconds     | 19800   | Timezone (IST default)                      |
| Clock Format      | 12/24       | 24      | Display time format                         |
| Latitude          | Decimal °   | 0.0     | Location latitude (GPS auto-detect support) |
| Longitude         | Decimal °   | 0.0     | Location longitude                          |
| Timezone Offset   | Hours       | 5.5     | UTC offset (dropdown with country names)    |

---

## 📂 Project Structure

```
SOLAR_TRACKING_RS485/
│
├── RECEIVER/
│   ├── RECEIVER.ino          # Master ESP32 — WiFi, Web, Display, Modbus Master
│   └── data/
│       ├── index.html        # Web Dashboard
│       └── config.html       # Configuration Portal
│
├── SENDER/
│   └── SENDER.ino            # Slave ESP32 — Sensors, Modbus Slave
│
├── display.png               # OLED screenshot
├── dashboard.png             # Web dashboard screenshot
├── config.png                # Config page screenshot
└── README.md
```

---

## 🛠️ Hardware Required

| #   | Component                           | Qty | Used By           |
| --- | ----------------------------------- | --- | ----------------- |
| 1   | ESP32 Development Board             | 2   | Sender + Receiver |
| 2   | MAX485 / TTL-to-RS485 Module        | 2   | Both modules      |
| 3   | SH1106 128×64 OLED (I2C)            | 1   | Receiver          |
| 4   | AHT10 Temperature & Humidity Sensor | 1   | Receiver          |
| 5   | BH1750 Light Intensity Sensor       | 1   | Sender            |
| 6   | NTC 10K Thermistor                  | 1   | Sender            |
| 7   | 10K Resistor (for voltage divider)  | 1   | Sender            |
| 8   | LEDs (or use onboard LED)           | 2   | Both (GPIO 2)     |
| 9   | Twisted Pair Cable (for RS485 A/B)  | 1   | Between modules   |
| 10  | 5V / 3.3V Power Supply              | 2   | Both modules      |

---

## 📦 Libraries Required

| Library             | Purpose                  |
| ------------------- | ------------------------ |
| `WiFi.h`            | WiFi connectivity        |
| `WiFiManager`       | Auto WiFi config portal  |
| `ESPAsyncWebServer` | Non-blocking web server  |
| `ArduinoJson`       | JSON serialization       |
| `U8g2lib`           | OLED display driver      |
| `Adafruit_AHT10`    | Temperature & humidity   |
| `BH1750`            | Light intensity sensor   |
| `Preferences`       | NVS settings storage     |
| `HardwareSerial`    | RS485 UART communication |
| `Dusk2Dawn`         | Sunrise/sunset times     |

---

## 🌞 Sun Map — How It Works

The dashboard features an interactive Sun Map that uses the **NOAA Solar Position Algorithm** to calculate real-time sun elevation and azimuth angles based on your configured latitude, longitude, and timezone.

### Key Calculations

- **Solar Declination** — Earth's axial tilt relative to the sun
- **Equation of Time** — correction for Earth's elliptical orbit
- **Hour Angle** — sun's angular position relative to solar noon
- **Elevation Angle** — sun's height above the horizon (0° to ~90°)
- **Azimuth Angle** — compass bearing of the sun (0°=N, 90°=E, 180°=S, 270°=W)
- **45° Crossing Times** — found via binary search for optimal solar panel angle

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/iotBrainStorm/SOLAR_TRACKING_RS485.git
```

### 2. Flash the SENDER

- Open `SENDER/SENDER.ino` in Arduino IDE
- Select your ESP32 board & COM port
- Upload

### 3. Flash the RECEIVER

- Open `RECEIVER/RECEIVER.ino` in Arduino IDE
- **Upload SPIFFS data** first: `Tools → ESP32 Sketch Data Upload`
- Then upload the sketch

### 4. Connect to WiFi

- On first boot, the Receiver creates an AP: **"Solar Weather"**
- Connect and configure your WiFi credentials at `192.168.4.1`

### 5. Access Dashboard

- Open the Receiver's IP address in any browser
- Dashboard: `http://<IP>/`
- Config: `http://<IP>/config.html`

---

## 🔐 Reliability & Safety

- ✅ CRC-16 validation on every Modbus frame
- ✅ Automatic garbage byte detection and stripping
- ✅ Bus watchdog recovery (30s timeout)
- ✅ HTTP timeout protection for Node-RED
- ✅ WiFi auto-reconnect with fallback AP
- ✅ NVS persistent storage (survives power cycles)
- ✅ Sender RS485 always enabled (cannot be bricked via config)

---

## 📈 Future Improvements

- 🐶 Hardware Watchdog Timer
- 🛠️ OTA Firmware Update
- 🌫️ Dust/Rain Monitoring Sensor
- ☁️ Cloud Backup (MQTT / Firebase)
- 💤 Deep Sleep Power Mode
- 💾 Data Logging to SD Card
- 📊 Historical charts on dashboard

---

## 🎥 Demo

📺 **Watch Full Working Demo:** [Demo 1](https://youtu.be/VbW-eC95sSU), [Demo 2](https://youtu.be/SrXycDGpsag), [Demo 3](https://youtu.be/nSapfp4AQMc)

---

## 👨‍💻 Developed By

**Mrinal Maity**  
_ESP32 Solar Monitoring System with RS485 Modbus_  
Made with dedication and engineering passion ❤️

---

<p align="center">
  ⭐ <b>Star this repo</b> if you found it useful! &nbsp; | &nbsp; 🍴 <b>Fork</b> to customize &nbsp; | &nbsp; 📢 <b>Share</b> with fellow makers
</p>

---

## 💎 Extra Professional Touch (Optional Additions)

<ul>
	<li>🏷️ GitHub badges (ESP32, Arduino, License)</li>
	<li>🎞️ Animated GIF demo</li>
	<li>🗂️ Block diagram</li>
	<li>📊 Feature comparison table</li>
	<li>📝 Version changelog</li>
	<li>📄 License section</li>
</ul>
