# FINN Guidance — Installation Guide

This guide covers everything needed to go from bare hardware to a working FINN
Guidance system: wiring the ESP32 modules, installing the PC software, flashing
the firmware, and verifying that everything is talking.

---

## 1. Hardware overview

The system uses two ESP32 DevKit modules connected to a field laptop via USB.
There is no WiFi, no buck converter, and no external power supply beyond the
laptop USB ports and a 12V battery for the steering motor.

### Components

| Component | Purpose |
|-----------|---------|
| ESP32 #1 (Sensor Module) | Reads wheel angle sensor, BNO055 IMU, GPS passthrough |
| ESP32 #2 (Motor Controller) | Drives IBT-2 H-bridge for steering motor |
| Quectel LC29H DA (on ArduSimple board) | GPS receiver, NMEA output at 1Hz |
| BNO055 breakout | 9DOF IMU for roll/pitch/heading |
| 10kΩ potentiometer | Wheel angle sensor (WAS) |
| IBT-2 H-bridge module | Motor driver for steering motor |
| 12V battery | Steering motor power (direct to IBT-2) |
| Field laptop (Dell Latitude 7390 2-in-1) | Runs PC guidance software |

---

## 2. Wiring

### 2.1 ESP32 #1 — Sensor Module

Connect via USB-A cable to laptop.

| ESP32 GPIO | Connect to | Notes |
|------------|-----------|-------|
| GPIO 34 | WAS pot wiper (middle pin) | ADC input, 0–3.3V |
| GPIO 33 | WAS pot pin 1 (high side) | Set HIGH at boot = 3.3V reference |
| GND | WAS pot pin 3 (low side) | Completes pot circuit |
| GPIO 21 | BNO055 SDA | I2C data, 3.3V logic |
| GPIO 22 | BNO055 SCL | I2C clock, 3.3V logic |
| GPIO 16 | ArduSimple TX (3.3V header) | GPS NMEA into ESP32 |
| GPIO 17 | ArduSimple RX (3.3V header) | Config commands to GPS |
| VIN (5V) | ArduSimple 5V | Powers GPS module |
| GND | Common ground | Shared across all devices |

**WAS pot wiring detail:**

```
GPIO 33 (3.3V HIGH) ──► Pot pin 1
                         Pot wiper ──► GPIO 34 (ADC)
                         Pot pin 3 ──► GND
```

Use shielded two-core microphone cable (XLR cable) for the pot connection
between the ESP32 and the steering column:

- Core 1 → 3.3V (GPIO 33)
- Core 2 → Wiper signal (GPIO 34)
- Shield → GND

Terminate the shield at the ESP32 end only to avoid ground loops.

**BNO055 wiring:**

| BNO055 pin | Connect to |
|------------|-----------|
| VIN | ESP32 3V3 pin |
| GND | ESP32 GND |
| SDA | ESP32 GPIO 21 |
| SCL | ESP32 GPIO 22 |

No external pull-up resistors needed for short cable runs — the ESP32 internal
pull-ups are sufficient.

**GPS wiring — IMPORTANT:**

Use the **3.3V serial header** on the ArduSimple board, NOT the 5V header.
The ESP32 GPIO pins are 3.3V logic and connecting to a 5V serial output will
damage them. The ArduSimple board has clearly labelled 3.3V and 5V headers.

### 2.2 ESP32 #2 — Motor Controller

Connect via USB-B cable to laptop.

| ESP32 GPIO | Connect to | Notes |
|------------|-----------|-------|
| GPIO 25 | IBT-2 RPWM | PWM for steer right |
| GPIO 26 | IBT-2 LPWM | PWM for steer left |
| GPIO 27 | IBT-2 R_EN | Right enable, HIGH to run |
| GPIO 14 | IBT-2 L_EN | Left enable, HIGH to run |
| VIN (5V) | IBT-2 VCC | Powers IBT-2 logic |
| GND | IBT-2 GND | Shared ground |

**IBT-2 motor supply:**

| IBT-2 terminal | Connect to |
|----------------|-----------|
| B+ | 12V battery positive |
| B- | 12V battery negative / chassis ground |
| M+ | Steering motor terminal 1 |
| M- | Steering motor terminal 2 |

The motor supply (12V) is completely separate from the logic supply (5V from
ESP32 VIN). They share a common ground.

### 2.3 Power distribution

```
Laptop USB-A ──► ESP32 #1 (sensor)
                   ├── 3V3 pin  → BNO055 VIN
                   ├── GPIO 33  → WAS pot high side (3.3V)
                   └── VIN (5V) → ArduSimple GPS board

Laptop USB-B ──► ESP32 #2 (motor controller)
                   └── VIN (5V) → IBT-2 logic VCC

12V Battery  ──► IBT-2 B+/B- (motor supply, high current)
```

No buck converter is needed. Both ESP32s are USB-powered from the laptop.
The 5V on the VIN pins back-feeds to the GPS and IBT-2 logic. This has been
bench-tested and confirmed working.

---

## 3. PC software installation

### 3.1 Prerequisites

Install the Rust toolchain on the field laptop:

```bash
# Install rustup (Rust toolchain manager)
# Download from https://rustup.rs or run:
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Verify installation
rustup --version
cargo --version
```

### 3.2 Clone and build

```bash
git clone <your-repo-url> finn-guidance
cd finn-guidance

# Build the PC guidance application (release mode for performance)
cargo build --release
```

The first build downloads and compiles all dependencies. Subsequent builds are
fast (a few seconds for code-only changes).

### 3.3 Run the PC application

```bash
cargo run --release
```

The application will auto-detect the GPS receiver on whichever COM port it
appears on. No manual port configuration is needed.

---

## 4. ESP32 firmware installation

The ESP32 firmware uses Arduino/PlatformIO (C++). Each ESP32 module has its own
PlatformIO project directory.

### 4.1 Install PlatformIO

This is a one-time setup on whichever machine you use for flashing (can be the
field laptop or a separate development PC).

```bash
# Install PlatformIO CLI
pip install platformio

# Verify installation
pio --version
```

PlatformIO will automatically download the ESP32 Arduino framework (~200MB) on
the first build. No additional toolchain installation is needed.

### 4.2 Flash the Sensor Module (ESP32 #1)

Plug in **only** ESP32 #1 via USB so PlatformIO can auto-detect the correct port.

```bash
cd finn-guidance/firmware-sensor-pio

# Build and flash
pio run --target upload

# Open a serial monitor to verify output (Ctrl+C to exit)
pio device monitor
```

You should see output like:

```
FINN Sensor Module starting...
GPIO 33 set HIGH — WAS pot powered (3.3V)
ADC1 CH6 (GPIO 34) configured — WAS input
I2C0 configured — SDA=21, SCL=22, 400kHz
BNO055 initialised in NDOF mode
UART2 configured — RX=16, TX=17, 115200 baud (GPS)
All peripherals initialised. Entering main loop.
$FINNWAS,2048,1650*4A
$FINNIMU,1.2,-0.5,182.3,3,3,2,1*3B
$GPGGA,123456.00,3402.1234,S,13845.6789,E,1,12,0.8,234.5,M,...*XX
$FINNHB,2000*1C
```

If the BNO055 is not connected, you will see a warning but the firmware
continues running and sends WAS and GPS data:

```
BNO055: no response on I2C
BNO055 init failed — IMU data will be zeros. Check wiring.
```

### 4.3 Flash the Motor Controller (ESP32 #2)

Unplug ESP32 #1. Plug in **only** ESP32 #2.

```bash
cd finn-guidance/firmware-motor-pio

# Build and flash
pio run --target upload

# Open a serial monitor to verify output (Ctrl+C to exit)
pio device monitor
```

You should see status messages at 5Hz:

```
FINN Motor Controller starting...
IBT-2 enable lines LOW (motor disabled)
PWM configured — RPWM=GPIO25, LPWM=GPIO26, 20kHz 8-bit
All peripherals initialised. Entering main loop.
$FINNMTR,0,0,200*XX
$FINNMTR,0,0,400*XX
```

The motor stays disabled (PWM=0, enabled=0) until it receives a valid steer
command.

### 4.4 Specifying a COM port

If auto-detection fails (e.g. multiple serial devices connected), specify the
port explicitly:

```bash
# Windows
pio run --target upload --upload-port COM3
pio device monitor --port COM3

# Linux
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0
```

To find available ports on Windows, check Device Manager under
"Ports (COM & LPT)", or run `mode` in a terminal.

### 4.5 First build downloads the framework

The first `pio run` for each firmware project downloads the ESP32 Arduino
framework (~200MB) and compiles it. This takes a few minutes. Subsequent builds
only recompile your code and take seconds.

### 4.6 Modifying firmware

The firmware source files are:

- `firmware-sensor-pio/src/main.cpp` — sensor module
- `firmware-motor-pio/src/main.cpp` — motor controller

After editing, rebuild and flash with `pio run --target upload` from the
relevant project directory.

---

## 5. Serial protocol reference

All communication between the ESP32 modules and the PC is text-based NMEA-style.
You can debug any connection with a standard serial monitor (PuTTY, the Arduino
serial monitor, or `pio device monitor`).

### 5.1 Sensor Module → PC (USB-A)

| Sentence | Fields | Rate |
|----------|--------|------|
| `$GPGGA,...*XX` | GPS position (passthrough from LC29H) | 1Hz |
| `$GPVTG,...*XX` | GPS velocity (passthrough from LC29H) | 1Hz |
| `$FINNWAS,<raw_adc>,<voltage_mv>*XX` | Wheel angle sensor | 20Hz |
| `$FINNIMU,<roll>,<pitch>,<heading>,<cal_sys>,<cal_gyro>,<cal_accel>,<cal_mag>*XX` | BNO055 IMU | 20Hz |
| `$FINNHB,<uptime_ms>*XX` | Heartbeat | Every 2s |

**WAS fields:** `raw_adc` is 0–4095 (12-bit), `voltage_mv` is 0–3300.

**IMU fields:** angles in degrees, calibration values 0–3 (3 = fully calibrated).

### 5.2 PC → Motor Controller (USB-B)

| Sentence | Fields | Rate |
|----------|--------|------|
| `$FINNSTEER,<pwm_value>*XX` | Steer command | ~20Hz from PID |

`pwm_value` ranges from -255 (full left) to +255 (full right). 0 = stop.

### 5.3 Motor Controller → PC (USB-B)

| Sentence | Fields | Rate |
|----------|--------|------|
| `$FINNMTR,<current_pwm>,<enabled>,<uptime_ms>*XX` | Motor status | 5Hz |

`enabled` is 1 (motor active) or 0 (motor disabled by watchdog or startup).

### 5.4 Checksum

All `$FINN*` sentences use standard NMEA XOR checksum: XOR every byte between
`$` and `*`, rendered as two uppercase hex digits. Same algorithm used by GPS
sentences.

### 5.5 Safety: motor watchdog

The motor controller stops the motor (PWM=0, enable lines LOW) if no valid
`$FINNSTEER` command is received within 500ms. This means:

- If the PC application crashes, the motor stops
- If the USB cable is unplugged, the motor stops
- If the serial connection hangs, the motor stops

The motor re-enables automatically when valid commands resume.

---

## 6. Testing and verification

### 6.1 Bench test — sensor module

1. Flash `firmware-sensor-pio` and open `pio device monitor`
2. Verify `$FINNWAS` values change when you turn the pot
3. Verify `$FINNIMU` heading changes when you rotate the BNO055
4. Verify `$GPGGA` sentences appear (GPS needs sky view or will show no-fix)
5. Verify `$FINNHB` appears every 2 seconds

### 6.2 Bench test — motor controller

1. Flash `firmware-motor-pio` and open `pio device monitor`
2. Verify `$FINNMTR,0,0,...` status messages (motor disabled)
3. In a second terminal, open the same COM port and send:
   `$FINNSTEER,100*2B` (calculate correct checksum)
4. Motor should spin in one direction
5. Send `$FINNSTEER,-100*54`
6. Motor should reverse
7. Stop sending commands — motor should stop within 500ms (watchdog)

### 6.3 Bench test — full system

1. Plug both ESP32s into the laptop
2. Run the PC guidance application: `cargo run --release`
3. Verify GPS fix appears in the status bar
4. Verify WAS and IMU readings appear (once PC-side parser is implemented)

---

## 7. Troubleshooting

**`pio run` fails with "No device found":**
Check that the ESP32 is plugged in. Try a different USB cable — some cables are
charge-only with no data lines. Check Device Manager for new COM ports when you
plug in.

**`pio run` fails to download the ESP32 framework:**
Check your internet connection. The first build downloads ~200MB. Retry if it
times out.

**BNO055 not responding (`no response on I2C`):**
Check SDA/SCL wiring (GPIO 21/22). Ensure the BNO055 board is powered from the
ESP32 3V3 pin (not 5V). Check that the AD0 pin on the BNO055 is not pulled high
(default address 0x28 assumes AD0 low).

**GPS shows no fix (`$GPGGA` with empty position fields):**
The GPS needs clear sky view. It will not get a fix indoors. For bench testing
the passthrough, the empty-position sentences confirm the wiring is correct.

**WAS reads 0 or 4095 constantly:**
Check that GPIO 33 is connected to the pot high side and GND to the low side.
Verify with a multimeter that there is ~3.3V across the pot terminals. If the
readings are noisy, check the shield termination on the mic cable.

**Motor doesn't respond to steer commands:**
Verify the checksum in your `$FINNSTEER` command is correct. Check that
`$FINNMTR` shows `enabled=1` after sending a command. If it stays at 0, the
command parsing is failing — check for correct `$` prefix and `*XX` suffix.

**Motor runs but immediately stops:**
The 500ms watchdog requires continuous commands. For manual testing, send
commands in a loop or increase `WATCHDOG_TIMEOUT_MS` temporarily in the firmware.

**Multiple serial devices causing auto-detect issues:**
Use `--upload-port COMx` (Windows) or `--upload-port /dev/ttyUSBx` (Linux) to
specify the port explicitly. See section 4.4.

**ESP32 not detected on any COM port:**
Try a different USB cable — some cables are charge-only with no data lines.
Check Device Manager for new COM ports when you plug in. Some ESP32 DevKit
boards use a CP2102 or CH340 USB-serial chip that may need a driver installed.
