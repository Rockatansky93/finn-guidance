# FINN Guidance

Open-source GPS guidance and auto-steer software for agricultural equipment,
built around a Rust PC/tablet application and ESP32 steering hardware.

The long-term goal is an AgOpenGPS-style system with live guidance, AB lines,
coverage recording, and closed-loop auto-steer.

## Current Architecture

```text
PC / tablet running Rust app
  - reads LC29H BA GGA/PQTMINS from USB serial
  - parses FINN motor telemetry from USB serial
  - displays field view and guidance in egui
  - stores jobs, fields, AB lines, config, and coverage in SQLite
  - computes steering target at a fixed rate
  - sends FINNSTEER commands to motor ESP32

Motor ESP32 running PlatformIO/Arduino firmware
  - reads wheel angle sensor
  - drives IBT-2 H-bridge
  - closes the inner steering loop
  - enforces motor watchdog safety
```

The maintained tractor hardware direction is LC29H BA direct to the laptop plus
one motor ESP32. The older separate sensor ESP32 path is retained only as
reference material.

## Project Structure

```text
finn-guidance/
  Cargo.toml                  Rust workspace root
  common/                     shared Rust types, coordinate math, serial protocol
  pc/                         Rust PC guidance application
    src/gps/                  GPS serial reader and NMEA parser
    src/guidance/             AB line, pass selection, steering logic
    src/gui/                  egui application and field view
    src/comms/                serial comms with motor ESP32
    src/coverage/             SQLite coverage/job/field storage
    src/position/             position tracking and interpolation
    src/telemetry/            steering telemetry logs
  firmware-motor-pio/         maintained ESP32 motor controller firmware
  firmware-sensor-pio/        archived older ESP32 sensor firmware reference
  docs/                       installation, tuning, context, and design notes
  scripts/upload_field_run.py summarize and register field-run logs in Core
```

## Status

The PC application builds and runs on Windows and Linux. The current hardware
direction uses a direct GPS serial connection plus a motor ESP32. The older
separate sensor ESP32 path is documented in the decision log and retained only
where still useful for reference.

Main implemented features:

- GPS serial auto-detect and NMEA parsing
- AB line guidance and pass selection
- egui field view with trails, pass lines, coverage, and lightbar
- SQLite storage for jobs, fields, AB lines, config, and coverage points
- motor ESP32 serial protocol and watchdog-aware steering commands
- closed-loop steering work in progress

See [docs/ACTIVE_CONTEXT.md](docs/ACTIVE_CONTEXT.md) for the latest field-test
state and next work. The main Phase 1 architecture docs are:

- [Current hardware architecture](docs/HARDWARE_ARCHITECTURE.md)
- [Coverage ownership](docs/COVERAGE_OWNERSHIP.md)
- [Release channels](docs/RELEASE_CHANNELS.md)
- [Implementation plan](docs/IMPLEMENTATION_PLAN.md)

## FINN Core Upload

After a run, the field laptop can register the latest steering telemetry and
coverage database summary with FINN Core:

```bash
python scripts/upload_field_run.py --core-url http://<core-pc>:8000 --field-name "North paddock"
```

The command keeps full files on the laptop and stores local file URIs plus
summaries in Core, where analysis tasks and the field-run dashboard can use
them.

## Hardware

### PC / Tablet

- Linux or Windows field laptop/tablet
- Rust PC application
- USB serial connection to GPS
- USB serial connection to motor ESP32

### GPS Receiver

- Quectel LC29H BA on ArduSimple board
- Direct USB serial connection to the laptop
- GGA position/fix metadata at 1 Hz on current NR11 firmware
- PQTMINS heading, velocity, roll, and pitch at 10 Hz
- RTK corrections via NTRIP for centimetre-level accuracy

The LC29H DA path is fine for future implement-side `finn-pilot` work, but the
tractor auto-steer stack should be treated as BA-critical.

### Motor Controller

- ESP32 DevKit running `firmware-motor-pio`
- IBT-2 / BTS7960 H-bridge motor driver
- 24V 500rpm brushed DC motor on steering column
- wheel angle sensor connected to ESP32 ADC

## Installation

For Linux setup, serial permissions, and field laptop notes, see
[docs/LINUX_INSTALLATION_GUIDE.md](docs/LINUX_INSTALLATION_GUIDE.md).

For full hardware wiring, firmware flashing, and bench testing, see
[docs/INSTALLATION_GUIDE.md](docs/INSTALLATION_GUIDE.md).

## Building The PC App

```bash
cd finn-guidance
cargo build --release -p finn-guidance-pc
cargo run --release -p finn-guidance-pc
```

## Flashing ESP32 Firmware

The maintained ESP32 firmware uses PlatformIO/Arduino.

```bash
cd firmware-motor-pio
pio run --target upload
```

The older `firmware-sensor-pio` project is retained for reference to the
previous separate sensor-module design.

## License

GPLv3, matching AgOpenGPS.
