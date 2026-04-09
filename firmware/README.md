# ESP32 Firmware

**This directory is the original placeholder.** The firmware has been split into
two separate crates for the two-ESP32 architecture:

- **`firmware-sensor/`** — ESP32 #1 (Sensor Module): WAS pot ADC, BNO055 IMU, GPS NMEA passthrough
- **`firmware-motor/`** — ESP32 #2 (Motor Controller): IBT-2 H-bridge PWM, steer command watchdog

Both are standalone ESP-IDF Rust projects built outside the workspace with:
```bash
cd firmware-sensor   # or firmware-motor
cargo build --target xtensa-esp32-espidf
```

## Serial protocol

All communication is text-based NMEA-style over USB serial for easy debugging
with any serial monitor.

### Sensor module → PC (USB-A)
| Sentence | Fields | Rate |
|----------|--------|------|
| `$GPGGA,...` | GPS position (passthrough from LC29H) | 1Hz |
| `$GPVTG,...` | GPS velocity (passthrough from LC29H) | 1Hz |
| `$FINNWAS,<raw_adc>,<voltage_mv>*XX` | Wheel angle sensor | 20Hz |
| `$FINNIMU,<roll>,<pitch>,<heading>,<cal_sys>,<cal_gyro>,<cal_accel>,<cal_mag>*XX` | BNO055 orientation + calibration | 20Hz |
| `$FINNHB,<uptime_ms>*XX` | Heartbeat | 0.5Hz |

### PC → Motor controller (USB-B)
| Sentence | Fields | Rate |
|----------|--------|------|
| `$FINNSTEER,<pwm_value>*XX` | Steer command, -255 to 255 | ~20Hz from PID |

### Motor controller → PC (USB-B)
| Sentence | Fields | Rate |
|----------|--------|------|
| `$FINNMTR,<current_pwm>,<enabled>,<uptime_ms>*XX` | Motor status | 5Hz |

### Checksum
Standard NMEA XOR checksum — XOR of all bytes between `$` and `*`, rendered as
two hex digits. Same algorithm used by GPS NMEA sentences.

## Prerequisite: ESP Rust toolchain

```bash
# Install espup (ESP32 Rust toolchain manager)
cargo install espup
espup install

# Source the environment (adds to PATH)
# Windows: restart terminal after espup install
# Linux/Mac: source ~/export-esp.sh
```
