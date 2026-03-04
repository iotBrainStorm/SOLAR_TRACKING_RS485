🌞 Solar Panel Smart Monitoring System (ESP32 Based)

Real-time Solar Monitoring + Web Dashboard + Configurable Node-RED Integration
Built with ESP32, Async Web Server, and Advanced Sensor Monitoring

📸 <b>Project Preview</b>

<p align="center">
	<img src="display.png" alt="OLED Display Output" width="300" style="margin:10px;"/>
	<img src="dashboard.png" alt="Web Dashboard" width="300" style="margin:10px;"/>
	<img src="config.png" alt="Configuration Page" width="300" style="margin:10px;"/>
</p>

🌐 Web Dashboard (Real-Time)

⚙ Configuration Page

---

## 🚀 Features

<ul>
	<li>🌞 <b>Real-Time Solar Panel Monitoring</b></li>
	<li>🌡️ Temperature Monitoring (NTC + AHT)</li>
	<li>💡 Ambient Light (LUX) Monitoring</li>
	<li>📶 WiFi Signal Strength (% Based)</li>
	<li>🔗 Node-RED Data Sharing (Configurable Interval)</li>
	<li>🛠️ Web-Based Configuration Page</li>
	<li>⚡ Fully Async Web Server (Non-Blocking)</li>
	<li>🖥️ OLED Live Display with Smart UI</li>
	<li>📡 WiFi Signal Bars + Status Icons</li>
	<li>🌅 Sunlight Progress Bar</li>
	<li>📈 LUX Increasing/Decreasing Indicator</li>
	<li>🕒 12H / 24H Time Format Support</li>
	<li>💾 EEPROM Settings Storage</li>
	<li>🏭 Industrial-Ready Architecture</li>
</ul>

📊 Parameters Monitored
Parameter Sensor Used Purpose
Panel Temperature NTC Thermistor Monitor panel heating
Ambient Temperature AHT Sensor Weather condition
Humidity AHT Sensor Environmental condition
Sunlight Intensity BH1750 LUX measurement
WiFi Strength Internal RSSI Network stability
Node-RED Status HTTP Check Data sharing health
🛠 Hardware Used

---

## 🛠 Hardware Used

<ul>
	<li>🟦 <b>ESP32 Development Board</b></li>
	<li>🖥️ 128x64 OLED Display (I2C)</li>
	<li>🌡️ NTC Thermistor</li>
	<li>🌬️ AHT Temperature & Humidity Sensor</li>
	<li>💡 BH1750 LUX Sensor</li>
	<li>🔋 Voltage Divider Circuit (for NTC)</li>
	<li>⚡ Stable 5V/3.3V Power Supply</li>
</ul>

---

## 🔌 Pin Configuration (Example)

| Device     | ESP32 Pin |
| ---------- | --------- |
| OLED SDA   | GPIO 21   |
| OLED SCL   | GPIO 22   |
| BH1750 SDA | GPIO 21   |
| BH1750 SCL | GPIO 22   |
| AHT SDA    | GPIO 21   |
| AHT SCL    | GPIO 22   |
| NTC Analog | GPIO 34   |

> _Modify according to your wiring_

---

## 🌐 Web Interface

### 🖥 Dashboard

<ul>
	<li>📊 Real-time sensor values</li>
	<li>📶 WiFi strength indicator</li>
	<li>🌅 Sunlight progress visualization</li>
	<li>💓 Node-RED heartbeat status</li>
	<li>📈 LUX trend indicator (+/-)</li>
</ul>

### ⚙ Configuration Page

<ul>
	<li>📡 WiFi Settings</li>
	<li>⏱️ Reading Intervals (Temperature / NTC / LUX)</li>
	<li>🔗 Node-RED Enable / Disable</li>
	<li>🔄 Node-RED Data Share Interval</li>
	<li>🕒 Time Format Selection (12H / 24H)</li>
	<li>💾 EEPROM Save</li>
</ul>

---

## 🔄 Node-RED Integration

<ul>
	<li>⏲️ Configurable Data Share Interval</li>
	<li>📤 HTTP POST based data sending</li>
	<li>⏱️ Timeout protection</li>
	<li>📡 Network failure safe handling</li>
</ul>

🖼 Circuit Diagram

📌 (Add your circuit diagram image here)

Example:

![Circuit Diagram](circuit.png)

If you want, I can generate a clean professional circuit diagram layout for you.

🎥 YouTube Demo

📺 Watch Full Working Demo Here:

👉 [Watch the YouTube Demo](https://youtu.be/oVlurzfXVxU)

🧠 System Architecture

---

## 🧠 System Architecture

<ul>
	<li>⚡ Non-blocking Async Web Server</li>
	<li>⏲️ <code>millis()</code> based timing system</li>
	<li>💾 EEPROM persistent settings</li>
	<li>🧩 Modular sensor handling</li>
	<li>🔄 WiFi auto reconnect logic</li>
	<li>📈 Scalable design</li>
</ul>

📂 Project Structure
/data
index.html
config.html
dashboard assets
/src
main.ino
README.md
display.png
dashboard.png
config.png
🔐 Stability & Safety

---

## 🔐 Stability & Safety

<ul>
	<li>⏱️ HTTP timeout protection</li>
	<li>🔄 WiFi reconnect logic</li>
	<li>🧠 Memory-efficient design</li>
	<li>⏳ Long runtime tested</li>
</ul>

📈 Future Improvements

---

## 📈 Future Improvements

<ul>
	<li>🐶 Watchdog protection</li>
	<li>🛠️ OTA Firmware Update</li>
	<li>🌫️ Dust Monitoring Sensor</li>
	<li>☁️ Cloud Backup Integration</li>
	<li>💤 Deep Sleep Power Mode</li>
	<li>💾 Data Logging to SD Card</li>
</ul>

👨‍💻 Developed By

**Mrinal Maity**  
_ESP32 Solar Monitoring System_  
Made with dedication and engineering passion ❤️

⭐ Support

If you like this project:

<ul>
	<li>⭐ <b>Star the repository</b></li>
	<li>🍴 <b>Fork it</b></li>
	<li>📢 <b>Share it</b></li>
</ul>

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
