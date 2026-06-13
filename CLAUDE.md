# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32 demo project — embedded development targeting ESP32 microcontrollers.

## Build & Flash

```bash
# Build the project
idf.py build

# Flash to device
idf.py -p /dev/ttyUSB0 flash

# Build, flash, and monitor
idf.py -p /dev/ttyUSB0 flash monitor

# Monitor serial output only
idf.py -p /dev/ttyUSB0 monitor

# Clean build
idf.py fullclean
idf.py build
```

## Configuration

```bash
# Open menuconfig
idf.py menuconfig

# Set target chip (e.g., esp32, esp32s3, esp32c3)
idf.py set-target esp32
```

## Project Structure

```
esp32/
├── CMakeLists.txt          # Top-level CMake project file
├── main/
│   ├── CMakeLists.txt      # Component CMake file
│   └── *.c / *.h           # Application source code
├── components/             # Custom components (optional)
├── sdkconfig               # ESP-IDF project configuration
└── partitions.csv          # Partition table (if used)
```

## Key Conventions

- `main/` contains the application entry point and primary logic
- Custom shared components go in `components/`
- Configuration is managed via `idf.py menuconfig` / `sdkconfig`
- Partition tables go in `partitions.csv` at the project root
