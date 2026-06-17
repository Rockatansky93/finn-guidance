# FINN Guidance - Current Hardware Architecture

Reviewed: 30 May 2026

This is the maintained tractor hardware direction for FINN Guidance.

## Maintained Tractor Stack

```text
LC29H BA rover over USB serial
        |
        v
Rust PC/tablet guidance app
        |
        | FINNSTEER commands over USB serial
        v
Motor ESP32 running firmware-motor-pio
        |
        | PWM + direction
        v
IBT-2 / BTS7960 motor driver
        |
        v
Steering motor

Wheel angle sensor -> Motor ESP32 ADC -> FINNMTR telemetry -> PC app
```

The PC app owns the outer guidance loop, AB-line logic, pass selection,
coverage display, SQLite storage, and operator interface. The motor ESP32 owns
the inner steering loop, wheel angle sensor reads, H-bridge drive, steering
watchdog, and motor-side safety stop.

## GNSS And Attitude

The tractor steering stack is BA-critical. Use the Quectel LC29H BA module for
tractor guidance and auto-steer.

Current LC29H BA NR11 behavior:

- GGA provides position and fix metadata at 1 Hz.
- PQTMINS provides heading, velocity, roll, and pitch at 10 Hz.
- The interpolator uses the 10 Hz attitude/velocity stream to keep the field
  view and steering loop smooth between 1 Hz position fixes.
- PAIR050 rate-setting may not be acknowledged by this firmware. That is not a
  steering failure; 10 Hz position output is not expected on this module.

## Module Roles

| Module | Use in FINN | Notes |
| --- | --- | --- |
| LC29H BA | Tractor guidance and auto-steer | Critical for the tractor because it supplies the BA attitude/velocity path used by the current guidance stack. |
| LC29H DA | Implement-side experiments and simpler rover roles | Fine for `finn-pilot` implement fix work where shaft-speed and implement state provide coverage truth. Do not treat it as the maintained tractor steering receiver. |
| LC29H BS | Base station / NTRIP service | Belongs to `finn-base`. It has different output and configuration behavior from the BA and DA rover modules. |

## Retired Tractor Path

The older separate sensor ESP32 path is not the maintained tractor
architecture. The following are retained only for history, comparison, or
possible salvage:

- `firmware-sensor-pio`
- BNO055 IMU wiring notes
- FINNWAS / FINNIMU sensor-module protocol notes
- DA-only tractor GPS assumptions

Current tractor builds should use:

- LC29H BA directly connected to the field laptop by USB serial.
- Motor ESP32 directly connected to the field laptop by USB serial.
- Wheel angle sensor connected to the motor ESP32 ADC.
- Motor ESP32 telemetry via FINNMTR / FINNACK.

## Field Validation State

Manual roll calibration exists in code, but the field validation is still
pending:

- Capture the level roll offset once per tractor.
- Check the correction sign on a known cross-slope.
- Confirm `roll_corr_m` moves cross-track error toward zero.

Until that validation is complete, do not make larger hardware or steering
behavior changes on the field build.
