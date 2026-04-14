# Air Quality Monitoring System (AQMS) - PIC32CM Firmware

This repository contains the firmware for an Air Quality Monitoring System (AQMS) built using the PIC32CM LS00 microcontroller and an ESP-01S (ESP8266) WiFi module.

## 🌡️ Features
- **Sensor Monitoring**: Reads real-time data for:
  - O3 (Ozone)
  - CO (Carbon Monoxide)
  - NH3 (Ammonia)
  - Dust / PM2.5
  - CO2 (Carbon Dioxide)
  - Temperature & Humidity (DHT11)
- **Local Display**: Updates a TFT display with current sensor readings.
- **Serial Monitoring**: Outputs data to UART/Serial for debugging (Putty/Arduino Monitor).
- **IoT Integration**: Transmits sensor data via MQTT (using ESP-01S) to a backend dashboard in JSON format.
- **Relay Control**: Supports manual and automatic relay triggering based on air quality thresholds.

## 🔌 Hardware Setup

### Components
- **Microcontroller**: PIC32CM LS00
- **WiFi Module**: ESP-01S (ESP8266)
- **Sensors**: MQ series, Dust sensor, CO2 sensor, DHT11
- **Display**: ST7735 / Similar TFT Display

### Wiring (ESP-01S to PIC32CM)
| ESP-01S Pin | PIC32CM Pin | Description |
|---|---|---|
| **VCC** | 3.3V | Power (Do not use 5V) |
| **GND** | GND | Ground |
| **TX** | PB17 (SERCOM5 RX) | Data from ESP to PIC |
| **RX** | PB16 (SERCOM5 TX) | Data from PIC to ESP |
| **CH_PD (EN)**| 3.3V | Must be pulled HIGH |
| **GPIO0** | 3.3V | Must be pulled HIGH for run mode |

## 💻 Software Configuration

### 1. WiFi & MQTT Settings
Edit `src/wifi_mqtt.h` to configure your environment:
```c
#define WIFI_SSID       "Your_SSID"
#define WIFI_PASSWORD   "Your_Password"
#define MQTT_BROKER_IP  "broker.hivemq.com"
#define MQTT_TOPIC      "aqms/PIC32CM_AQI_01/data"
```

### 2. Building the Project
- Open the project in **MPLAB X IDE**.
- Build and program the PIC32CM device.
- The firmware uses a unity-build pattern (including `.c` files in `main.c`) for simpler configuration.

## 📊 Data Pipeline
1. **Sensors** → Analog/Digital Read on PIC32.
2. **PIC32** → Formats data into a JSON string.
3. **ESP-01S** → Transmits JSON over TCP/MQTT.
4. **Broker** → HiveMQ Public Broker.
5. **Backend** → Node.js server parses and stores data.
6. **Frontend** → React dashboard visualizes air quality metrics in real-time.

---
**Author**: Monish
**Project**: AQMS (Air Quality Monitoring System)
