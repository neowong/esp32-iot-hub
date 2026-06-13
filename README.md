# ESP32 Universal IoT Hub

ESP32 通用固件 + MQTT 集中管理系统。同一份 `.bin` 适配所有板子，传感器模块化配置，支持 OTA 无线升级。

## Architecture

```
ESP32#1 ──┐                    ┌── MQTT Broker ──┐
ESP32#2 ──┤── publish ────→    │  (Mosquitto)    │── subscribe → Hub (Python)
ESP32#3 ──┘  esp32/MAC/data    └─────────────────┘              SQLite + Dashboard
```

## Features

| 功能 | 说明 |
|------|------|
| 📡 LD2410C 雷达 | 人体存在 / 运动+微动能量 / 距离 / 光照 |
| 🌐 本地网页 | 实时仪表盘 + 配置页 + OTA 上传 |
| 📟 MQTT | 每 1s 推送传感器数据，topic: `esp32/MAC/data` |
| 🖥️ 集中面板 | 多设备管理，历史曲线 (1h/6h/24h)，自定义名称 |
| ⬆️ OTA 升级 | 浏览器上传 `.bin`，自动重启 |
| 🔧 自动配网 | 新设备开 ESP32-XXXXXX 热点，密码 12345678 |
| ⚙️ NVS 配置 | WiFi / MQTT / 设备名 / 传感器开关，重启不丢 |

## Quick Start

### ESP32 Firmware

```bash
# Build
source ~/esp-idf/export.sh
idf.py build

# First flash (serial)
idf.py -p /dev/ttyUSB0 flash

# After first flash: all updates via OTA
# Open http://[ESP32-IP]/ota → upload build/esp32-demo.bin
```

### Hub Server (Docker)

```bash
cd server
# Copy dashboard to app/
cp ../web/dashboard.html app/

# Deploy
docker compose up -d

# Services:
# - Mosquitto MQTT :1883
# - Hub Dashboard :8080
```

## Configuration

All stored in NVS, configurable via web:
- WiFi SSID / Password
- MQTT Broker address (default: 192.168.31.251)
- Device name (default: ESP32-MAC)
- Radar enabled/disabled, RX/TX pins, threshold

## API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api` | GET | Sensor data + version |
| `/cfg` | GET | Config page (prefilled) |
| `/save` | GET | Save config (query params) |
| `/ota` | GET | OTA upload page |
| `/update` | POST | Receive firmware binary |
| `/mqtt` | GET | MQTT broker config |

## Project Structure

```
esp32/
├── main/
│   ├── main.c              # Universal firmware (~400 lines)
│   ├── CMakeLists.txt
│   └── idf_component.yml   # MQTT dependency
├── server/
│   ├── compose.yml         # Docker deployment
│   ├── mosquitto.conf
│   └── app/
│       ├── Dockerfile
│       ├── hub.py           # MQTT subscriber + API
│       └── dashboard.html   # Web dashboard
├── web/
│   └── dashboard.html      # Standalone panel (CORS)
├── partitions.csv          # OTA dual-partition
└── sdkconfig.defaults
```
