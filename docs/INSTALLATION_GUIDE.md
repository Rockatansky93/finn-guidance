# FINN Guidance - Installation Guide

Reviewed: 30 May 2026

This guide describes the maintained tractor installation: LC29H BA connected
directly to the field laptop, plus one motor ESP32 for steering.

The older two-ESP32 sensor-module path is not the maintained installation. It
is retained in history only.

## 1. Hardware Overview

| Component | Purpose |
| --- | --- |
| Field laptop / tablet | Runs the Rust guidance app. |
| Quectel LC29H BA on ArduSimple board | Tractor GNSS, heading, velocity, roll, and pitch. |
| ESP32 DevKit running `firmware-motor-pio` | Reads wheel angle sensor and drives the steering motor. |
| Wheel angle sensor | Reports actual steering angle to the motor ESP32. |
| IBT-2 / BTS7960 H-bridge | Drives the steering motor. |
| Steering motor | Turns the steering wheel or steering column mechanism. |

The laptop has two USB serial devices:

- LC29H BA direct USB serial for GPS / attitude data.
- Motor ESP32 USB serial for FINN steering commands and telemetry.

## 2. Maintained Data Flow

```text
LC29H BA -> PC app
  GGA: position and fix metadata, 1 Hz on current NR11 firmware
  PQTMINS: heading, velocity, roll, and pitch, 10 Hz

PC app -> Motor ESP32
  FINNSTEER: target wheel angle, motor limits, engage state

Motor ESP32 -> PC app
  FINNMTR / FINNACK: wheel angle, PWM, config acknowledgements, heartbeat
```

There is no separate sensor ESP32 in the maintained tractor stack. Wheel angle
sensor readings come from the motor ESP32.

## 3. Motor ESP32 Wiring

Connect the motor ESP32 to the laptop by USB.

| ESP32 GPIO | Connect to | Notes |
| --- | --- | --- |
| GPIO 34 | WAS wiper | ADC input, 0-3.3 V only. |
| GPIO 33 | WAS high side | 3.3 V reference controlled by firmware. |
| GND | WAS low side | Common sensor ground. |
| GPIO 25 | IBT-2 RPWM | PWM for one motor direction. |
| GPIO 26 | IBT-2 LPWM | PWM for the other motor direction. |
| GPIO 27 | IBT-2 R_EN | Enable line. |
| GPIO 14 | IBT-2 L_EN | Enable line. |
| VIN / 5V | IBT-2 VCC | Logic supply only. |
| GND | IBT-2 GND | Shared logic ground. |

IBT-2 motor supply:

| IBT-2 terminal | Connect to |
| --- | --- |
| B+ | Motor supply positive. |
| B- | Motor supply negative / chassis ground. |
| M+ | Steering motor terminal 1. |
| M- | Steering motor terminal 2. |

Keep motor power wiring sized for the steering motor current. Keep low-voltage
signal wiring physically separate from motor power wiring where practical.

## 4. GPS Connection

Connect the LC29H BA / ArduSimple board directly to the field laptop by USB.

The guidance app auto-detects the GPS serial port unless a specific port is
configured. Current LC29H BA NR11 firmware is expected to keep GGA position at
1 Hz while providing 10 Hz attitude/velocity through PQTMINS. That split is
normal for this stack.

Do not route the BA through an ESP32 sensor module for the current tractor
installation.

## 5. PC Software

Install Rust from https://rustup.rs, then build the app:

```bash
git clone https://github.com/Rockatansky93/finn-guidance.git
cd finn-guidance
cargo build --release -p finn-guidance-pc
```

Run it:

```bash
cargo run --release -p finn-guidance-pc
```

For Linux serial permissions and desktop notes, see
[`LINUX_INSTALLATION_GUIDE.md`](LINUX_INSTALLATION_GUIDE.md).

## 6. Flash Motor Firmware

Install PlatformIO, then flash the maintained motor firmware:

```bash
cd finn-guidance/firmware-motor-pio
pio run --target upload
```

Open the serial monitor if you need to confirm startup:

```bash
pio device monitor
```

Expected behavior:

- The motor ESP32 sends heartbeat / motor telemetry.
- The PC app finds the motor port.
- The motor watchdog holds PWM at zero until valid steering commands arrive.

## 7. Bench Checks Before Field Use

Before engaging auto-steer in the field:

1. Confirm the app can see GPS fixes from the LC29H BA.
2. Confirm the app can see the motor ESP32.
3. Calibrate wheel angle sensor centre, left lock, and right lock.
4. Confirm actual wheel angle moves in the expected direction.
5. Confirm PWM goes to zero when the app is closed or steering is disengaged.
6. Confirm motor direction with the wheels off the ground or with a safe test
   setup before applying force in the paddock.

## 8. Roll Calibration

Manual roll calibration is present in the app and still needs field validation
on the current tractor build.

Procedure:

1. Park on flat, level ground.
2. Open Setup -> ROLL CORRECTION.
3. Wait for the live roll reading to settle.
4. Press Capture Level.
5. On a known cross-slope, check whether `roll_corr_m` moves cross-track error
   toward zero.
6. If the correction pushes the line the wrong way, toggle roll direction once
   and test again.

Keep telemetry enabled for this validation run if practical.

## 9. Troubleshooting

### GPS opens but position appears to update at 1 Hz

This is expected on the current LC29H BA NR11 firmware. GGA position is 1 Hz;
PQTMINS supplies 10 Hz heading, velocity, roll, and pitch. The app interpolates
between position fixes.

### Log says PAIR050 was not acknowledged

That usually means the LC29H BA firmware does not accept the requested GGA rate
change. It is not a reason to stop field work if GGA and PQTMINS are both
arriving.

### Auto-steer will not engage

Check:

- AB line loaded.
- Motor ESP32 detected.
- Wheel angle sensor calibrated.
- GPS fix available.
- Current speed is above the steering speed gate.

### WAS data lost

Check the motor ESP32 USB cable, the WAS wiring, and the motor firmware serial
output. In the maintained tractor stack, WAS data comes from the motor ESP32,
not a separate sensor ESP32.

### Wheels turn the wrong way

Disengage immediately. Use the motor direction control in Setup, then repeat a
slow bench test before field use.
