# ESP32 Firmware

This crate is the ESP32 steering controller firmware. It is built separately 
from the PC workspace using the ESP-IDF toolchain.

## Status: Phase 4 (not yet implemented)

The first real-world test focuses on GPS guidance only (PC side). 
The ESP32 firmware for motor control will be implemented after the 
guidance system is validated in the field.

## Planned functionality

- Read wheel angle sensor (WAS) via ADC
- Read BNO055 IMU via I2C  
- Drive IBT-2 H-bridge via PWM
- Communicate with PC via UDP over WiFi
- Local PID safety loop (watchdog - stop motor if PC comms lost)

## Build requirements

- Rust with ESP32 target: `espup install`
- ESP-IDF toolchain
- `cargo build --target xtensa-esp32-espidf`

## Hardware connections

```
ESP32 Pin       Connection
─────────       ──────────
GPIO 34 (ADC)   WAS signal (via voltage divider 5V->3.3V)
GPIO 21 (SDA)   BNO055 I2C SDA
GPIO 22 (SCL)   BNO055 I2C SCL
GPIO 25 (PWM)   IBT-2 RPWM
GPIO 26 (PWM)   IBT-2 LPWM
GPIO 27         IBT-2 R_EN + L_EN
```
