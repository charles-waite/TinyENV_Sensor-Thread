# TinyENV Sensor – Matter over Thread (ESP32‑C6)

A headless, battery‑powered temperature & humidity sensor built on **Seeed XIAO ESP32‑C6**, using **Matter over Thread**. Works with **Home Assistant** and **Apple Home**.

This project is intentionally minimal: no screen, no buttons beyond BOOT for decommissioning, and no OTA (for now). The goal is a stable, low‑power environmental sensor using a modern Matter/Thread stack.

---

## Hardware

- **MCU**: Seeed XIAO ESP32‑C6
- **Sensor**: Sensirion SHT41 (I²C)
- **Power**: Supports Battery or USB
- **Battery sense**: A0 via 1:2 divider (220,000Ω, offset calibrated in firmware to 1.0123)

Xiao ESP32-C6 board-specific pin definitions:

```cpp
#define SDA_PIN 22
#define SCL_PIN 23
#define VBAT_ADC_PIN A0
```

---

## Software Stack

- **ESP‑IDF**: v5.5.x
- **Arduino Core**: arduino‑esp32 3.3.x (as ESP‑IDF component)
- **Matter**: esp‑matter + Arduino Matter wrappers
- **Transport**: Thread (IEEE 802.15.4)
- **Sensor driver**: Adafruit SHT4x - https://github.com/adafruit/Adafruit_SHT4X

---

## Project Structure (important bits)

```text
TinyENV_Sensor-Thread/
├── CMakeLists.txt              # Top-level CMake (ESP-IDF project entry)
├── sdkconfig
├── sdkconfig.defaults
├── sdkconfig.old
├── dependencies.lock
├── partitions.csv
├── README.md
├── components/
│   ├── Adafruit_SHT4X/         # SHT4x temperature/humidity sensor driver
│   ├── Adafruit_BusIO/         # Adafruit BusIO dependency
│   ├── Adafruit_Sensor/        # Adafruit unified sensor base
│   └── MatterEndpoints/
│       ├── include/
│       │   └── MatterEndpoints/
│       │       └── MatterTemperatureSensorBattery.h
│       ├── MatterTemperatureSensorBattery.cpp
│       └── CMakeLists.txt
└── main/
    ├── TinyENV_Thread.cpp      # Main application entry point
    ├── CMakeLists.txt
    └── idf_component.yml
```

### Custom Matter Endpoint

A custom endpoint (`MatterTemperatureSensorBattery`) extends the standard Matter temperature sensor to also expose **battery voltage and percentage**. This is a local shim, not a published library.

---

## What Changed From the Original Example

- Replaced example **Color Light** with:
  - Temperature Sensor cluster
  - Humidity Sensor cluster
  - Battery reporting
- Switched transport to **Matter over Thread** (no Wi‑Fi required at runtime)
- Explicit USB CDC configuration so `Serial.print()` works reliably
- Explicit I²C pin configuration for XIAO ESP32‑C6
- Headless commissioning (QR + manual code via serial)
- Added power‑saving features (CPU scaling + light sleep)

---

## One‑Time ESP‑IDF Setup

Clone ESP‑IDF and install tools (once per machine):

```bash
git clone -b v5.5 https://github.com/espressif/esp-idf.git ~/esp-idf
cd ~/esp-idf
./install.sh
```

Add a shell alias (recommended):

```bash
alias espidf-init='. $HOME/esp-idf/export.sh'
```

---

## Build & Flash Instructions

From the project root:

```bash
espidf-init
idf.py set-target esp32c6
idf.py fullclean
idf.py build
idf.py -p /dev/cu.usbmodemXXXX erase-flash flash monitor
```

> ⚠️ Use `/dev/cu.*`, **not** `/dev/tty.*` on macOS.

---

## Commissioning

On first boot (or after erase/decommission), the device prints:

- Manual pairing code
- QR code URL

Use either **Home Assistant** or **Apple Home** to commission. The same firmware works in both ecosystems.

BOOT button held for **>5 seconds** will decommission the node.

---

## Runtime Behavior

- Sensor polling interval: **120 seconds**.
- Updates temperature, humidity, and battery over Matter over Thread every 2 minutes.
- Designed to run unattended on battery.
- Serial output tied to DEBUG_SERIAL bool.

Example serial output:

```text
Updated: 72.8 F, 55 %RH, VBAT: 4.12V (93%)
```

---


## Status

✅ Builds cleanly on ESP‑IDF 5.5.x.
✅ Works with Home Assistant.
✅ Works with Apple Home.
✅ Matter over Thread confirmed working.

This is a stable baseline. Future work will focus on power optimization and optional sensor expansion.

## To-Do

- Build current monitor jig.
- Reduce power consumption to <10mA. (currently nearly 70mA with 1/7 build)
- - Verify low power/sleep states are implemented.
- - Test for and implement sleep modes.
- Tweak polling
