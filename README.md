# 🌞 Solar Panel Smart Monitoring System (ESP32 Based)

# 

# Real-time Solar Monitoring + Web Dashboard + Configurable Node-RED Integration

# Built with ESP32, Async Web Server, and Advanced Sensor Monitoring

# 

# 📸 <b>Project Preview</b>

# 

# <p align="center">

# &nbsp;	<img src="display.png" alt="OLED Display Output" width="300" style="margin:10px;"/>

# &nbsp;	<img src="dashboard.png" alt="Web Dashboard" width="300" style="margin:10px;"/>

# &nbsp;	<img src="config.png" alt="Configuration Page" width="300" style="margin:10px;"/>

# </p>

# 

# 🌐 Web Dashboard (Real-Time)

# 

# ⚙ Configuration Page

# 

# ---

# 

# \## 🚀 Features

# 

# <ul>

# &nbsp;	<li>🌞 <b>Real-Time Solar Panel Monitoring</b></li>

# &nbsp;	<li>🌡️ Temperature Monitoring (NTC + AHT)</li>

# &nbsp;	<li>💡 Ambient Light (LUX) Monitoring</li>

# &nbsp;	<li>📶 WiFi Signal Strength (% Based)</li>

# &nbsp;	<li>🔗 Node-RED Data Sharing (Configurable Interval)</li>

# &nbsp;	<li>🛠️ Web-Based Configuration Page</li>

# &nbsp;	<li>⚡ Fully Async Web Server (Non-Blocking)</li>

# &nbsp;	<li>🖥️ OLED Live Display with Smart UI</li>

# &nbsp;	<li>📡 WiFi Signal Bars + Status Icons</li>

# &nbsp;	<li>🌅 Sunlight Progress Bar</li>

# &nbsp;	<li>📈 LUX Increasing/Decreasing Indicator</li>

# &nbsp;	<li>🕒 12H / 24H Time Format Support</li>

# &nbsp;	<li>💾 EEPROM Settings Storage</li>

# &nbsp;	<li>🏭 Industrial-Ready Architecture</li>

# </ul>

# 

# 📊 Parameters Monitored

# Parameter Sensor Used Purpose

# Panel Temperature NTC Thermistor Monitor panel heating

# Ambient Temperature AHT Sensor Weather condition

# Humidity AHT Sensor Environmental condition

# Sunlight Intensity BH1750 LUX measurement

# WiFi Strength Internal RSSI Network stability

# Node-RED Status HTTP Check Data sharing health

# 🛠 Hardware Used

# 

# ---

# 

# \## 🛠 Hardware Used

# 

# <ul>

# &nbsp;	<li>🟦 <b>ESP32 Development Board</b></li>

# &nbsp;	<li>🖥️ 128x64 OLED Display (I2C)</li>

# &nbsp;	<li>🌡️ NTC Thermistor</li>

# &nbsp;	<li>🌬️ AHT Temperature \& Humidity Sensor</li>

# &nbsp;	<li>💡 BH1750 LUX Sensor</li>

# &nbsp;	<li>🔋 Voltage Divider Circuit (for NTC)</li>

# &nbsp;	<li>⚡ Stable 5V/3.3V Power Supply</li>

# </ul>

# 

# ---

# 

# \## 🔌 Pin Configuration (Example)

# 

# | Device     | ESP32 Pin |

# | ---------- | --------- |

# | OLED SDA   | GPIO 21   |

# | OLED SCL   | GPIO 22   |

# | BH1750 SDA | GPIO 21   |

# | BH1750 SCL | GPIO 22   |

# | AHT SDA    | GPIO 21   |

# | AHT SCL    | GPIO 22   |

# | NTC Analog | GPIO 34   |

# 

# > \_Modify according to your wiring\_

# 

# ---

# 

# \## 🌐 Web Interface

# 

# \### 🖥 Dashboard

# 

# <ul>

# &nbsp;	<li>📊 Real-time sensor values</li>

# &nbsp;	<li>📶 WiFi strength indicator</li>

# &nbsp;	<li>🌅 Sunlight progress visualization</li>

# &nbsp;	<li>💓 Node-RED heartbeat status</li>

# &nbsp;	<li>📈 LUX trend indicator (+/-)</li>

# </ul>

# 

# \### ⚙ Configuration Page

# 

# <ul>

# &nbsp;	<li>📡 WiFi Settings</li>

# &nbsp;	<li>⏱️ Reading Intervals (Temperature / NTC / LUX)</li>

# &nbsp;	<li>🔗 Node-RED Enable / Disable</li>

# &nbsp;	<li>🔄 Node-RED Data Share Interval</li>

# &nbsp;	<li>🕒 Time Format Selection (12H / 24H)</li>

# &nbsp;	<li>💾 EEPROM Save</li>

# </ul>

# 

# ---

# 

# \## 🔄 Node-RED Integration

# 

# <ul>

# &nbsp;	<li>⏲️ Configurable Data Share Interval</li>

# &nbsp;	<li>📤 HTTP POST based data sending</li>

# &nbsp;	<li>⏱️ Timeout protection</li>

# &nbsp;	<li>📡 Network failure safe handling</li>

# </ul>

# 

# 🖼 Circuit Diagram

# 

# 📌 (Add your circuit diagram image here)

# 

# Example:

# 

# !\[Circuit Diagram](circuit.png)

# 

# If you want, I can generate a clean professional circuit diagram layout for you.

# 

# 🎥 YouTube Demo

# 

# 📺 Watch Full Working Demo Here:

# 

# 👉 \[Watch the YouTube Demo](https://youtu.be/oVlurzfXVxU)

# 

# 🧠 System Architecture

# 

# ---

# 

# \## 🧠 System Architecture

# 

# <ul>

# &nbsp;	<li>⚡ Non-blocking Async Web Server</li>

# &nbsp;	<li>⏲️ <code>millis()</code> based timing system</li>

# &nbsp;	<li>💾 EEPROM persistent settings</li>

# &nbsp;	<li>🧩 Modular sensor handling</li>

# &nbsp;	<li>🔄 WiFi auto reconnect logic</li>

# &nbsp;	<li>📈 Scalable design</li>

# </ul>

# 

# 📂 Project Structure

# /data

# index.html

# config.html

# dashboard assets

# /src

# main.ino

# README.md

# display.png

# dashboard.png

# config.png

# 🔐 Stability \& Safety

# 

# ---

# 

# \## 🔐 Stability \& Safety

# 

# <ul>

# &nbsp;	<li>⏱️ HTTP timeout protection</li>

# &nbsp;	<li>🔄 WiFi reconnect logic</li>

# &nbsp;	<li>🧠 Memory-efficient design</li>

# &nbsp;	<li>⏳ Long runtime tested</li>

# </ul>

# 

# 📈 Future Improvements

# 

# ---

# 

# \## 📈 Future Improvements

# 

# <ul>

# &nbsp;	<li>🐶 Watchdog protection</li>

# &nbsp;	<li>🛠️ OTA Firmware Update</li>

# &nbsp;	<li>🌫️ Dust Monitoring Sensor</li>

# &nbsp;	<li>☁️ Cloud Backup Integration</li>

# &nbsp;	<li>💤 Deep Sleep Power Mode</li>

# &nbsp;	<li>💾 Data Logging to SD Card</li>

# </ul>

# 

# 👨‍💻 Developed By

# 

# \*\*Mrinal Maity\*\*  

# \_ESP32 Solar Monitoring System\_  

# Made with dedication and engineering passion ❤️

# 

# ⭐ Support

# 

# If you like this project:

# 

# <ul>

# &nbsp;	<li>⭐ <b>Star the repository</b></li>

# &nbsp;	<li>🍴 <b>Fork it</b></li>

# &nbsp;	<li>📢 <b>Share it</b></li>

# </ul>

# 

# ---

# 

# \## 💎 Extra Professional Touch (Optional Additions)

# 

# <ul>

# &nbsp;	<li>🏷️ GitHub badges (ESP32, Arduino, License)</li>

# &nbsp;	<li>🎞️ Animated GIF demo</li>

# &nbsp;	<li>🗂️ Block diagram</li>

# &nbsp;	<li>📊 Feature comparison table</li>

# &nbsp;	<li>📝 Version changelog</li>

# &nbsp;	<li>📄 License section</li>

# </ul>



