# ATOM — Flight Computer

V2.0
Replacing the thermal cam with an IMU for tilt-based fin stabilization

> Model rocketry flight computer with thermal tracking, active fin stabilization, and telemetry.

---

## Overview

ATOM is an ESP32-based flight computer designed for model rockets. The thermal tracking module uses an MLX90640 infrared camera to detect and lock onto a heat source, then drives four servo-controlled fins to actively stabilize the rocket's trajectory. For version 2, the IMU module uses an MPU9250 to sense pitch and roll, then drives four servo-controlled fins.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32 |
| IMU | MPU9250 (accelerometer + gyro) |
| Thermal Camera | MLX90640 (32×24, up to 16 Hz) |
| Fins | 4× Servo motors (N/E/S/W) |
| Power | External supply for servos |

**Wiring:**

| MPU9250 | ESP32-S3 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |


| MLX90640 | ESP32 |
|---|---|
| VIN | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

| Servo | GPIO |
|---|---|
| North | 25 |
| East | 26 |
| South | 27 |
| West | 32 |

---

## Repository Structure

```
ATOM-Flight-Computer/
├── ATOM/               # Main thermal tracking firmware
├── ESP1/               # ESP1 module code
├── ESP2/               # ESP2 module code
├── Fins/               # Fin servo control
├── Integrate_test/     # Integration tests
├── MLX90640_Test/      # Thermal camera tests
└── NorthFinDebug/      # North fin debug sketch
```

---

## How It Works

1. The MLX90640 captures a 32×24 thermal frame at 16 Hz.
2. The hottest pixel above the temperature threshold (`30°C`) is identified as the target.
3. Smoothed X/Y coordinates are computed to reduce jitter.
4. Servo positions for all four fins are calculated from the tracking error relative to frame center.
5. If the target stops moving for more than 1.5 seconds, fins return to center.

**Key parameters:**

| Parameter | Value |
|---|---|
| Refresh rate | 16 Hz |
| Temp threshold | 30.0 °C |
| Smoothing factor | 0.05 |
| Tracking gain | 2.5 |
| Target timeout | 1500 ms |

---

## Dependencies

- [Adafruit MLX90640 Library](https://github.com/adafruit/Adafruit_MLX90640)
- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)
- Arduino Wire library

---

## Getting Started

1. Install dependencies via the Arduino Library Manager.
2. Open the sketch in `ATOM/`.
3. Select **ESP32** as your board.
4. Flash and open Serial Monitor at **115200 baud**.

---
##  Contact

**Tisha Patel**
- **GitHub:** [@tishap27](https://github.com/tishap27)
- **Email:** [tishaapatel08@gmail.com](mailto:tishaapatel08@gmail.com)
