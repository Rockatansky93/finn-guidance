# FINN Guidance

Open-source GPS guidance and auto-steer system for agricultural equipment, 
built in Rust. Part of the FINN Initiative (Farm Integrated Neural Network).

## Architecture

```
┌─────────────────────────────┐        UDP         ┌──────────────────────┐
│  PC / Tablet  (pc crate)    │◄──────────────────►│  ESP32 (firmware)    │
│                              │                    │                      │
│  GPS receiver (USB serial)   │  Steer cmd ──────► │  PWM ──► IBT-2 ──►  │
│  NMEA/UBX parsing            │                    │            24V Motor │
│  AB line guidance            │  ◄── Sensor data   │  ADC ◄── WAS        │
│  Cross-track error           │                    │  I2C ◄── BNO055     │
│  GUI display (egui)          │                    │                      │
│  PID controller              │                    │                      │
│  Coverage logging (CSV)      │                    │                      │
│  Metadata database (SQLite)  │                    │                      │
└─────────────────────────────┘                    └──────────────────────┘
```

## Project Structure

```
finn-guidance/
├── Cargo.toml              # Workspace root
├── common/                 # Shared types & protocol (used by both PC and firmware)
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs
│       ├── types.rs        # GpsFix, ImuData, WasReading, VehicleState, GuidanceLine
│       ├── protocol.rs     # UDP message types, ports, constants
│       └── coords.rs       # Coordinate math (haversine, bearing, cross-track)
├── pc/                     # PC guidance application
│   ├── Cargo.toml
│   └── src/
│       ├── main.rs         # Entry point, thread orchestration
│       ├── gps/
│       │   ├── mod.rs
│       │   ├── reader.rs   # Serial port GPS reader thread
│       │   └── parser.rs   # NMEA sentence parsing -> GpsFix (epoch-based)
│       ├── guidance/
│       │   ├── mod.rs
│       │   └── ab_line.rs  # AB line guidance + cross-track error
│       ├── gui/
│       │   ├── mod.rs
│       │   ├── app.rs      # egui guidance display application
│       │   ├── field_view.rs    # 2D field canvas with layers
│       │   └── field_projection.rs  # Lat/lon → local metres → screen pixels
│       ├── comms/
│       │   ├── mod.rs
│       │   └── udp.rs      # UDP comms with ESP32 (Phase 4)
│       ├── coverage/
│       │   ├── mod.rs
│       │   ├── logger.rs   # GPS coverage recording (CSV, filtered)
│       │   └── db.rs       # SQLite database (jobs, segments, AB lines)
│       └── position/
│           ├── mod.rs
│           └── tracker.rs  # Position history and odometer
├── coverage_logs/          # CSV coverage files (gitignored)
├── firmware/               # ESP32 steering controller (Phase 4)
│   ├── Cargo.toml
│   ├── README.md
│   └── src/
│       └── main.rs
└── docs/
    └── IMPLEMENTATION_PLAN.md
```

## Current Status

**Phase 1 nearly complete** — GPS reader, NMEA parsing, field view canvas, 
AB line guidance, and coverage logging all working. First field test completed
(walk test with GPS). Coverage logging rewritten with epoch-based deduplication
and configurable distance/time filtering. SQLite database added for job metadata
and AB line persistence.

See [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md) for full details.

## Hardware

### GPS Receiver
- Quectel lc29h (ArduSimple board)
- Connected to PC via USB (serial)
- NMEA output: GGA (position) + VTG (speed/heading)
- RTK corrections via NTRIP for centimetre accuracy

### Steering Controller (Phase 4)
- ESP32 DevKit
- IBT-2 H-bridge motor driver (BTS7960)
- 24V 500rpm brushed DC motor on steering column
- Wheel angle sensor (WAS) - 0-5V via voltage divider to ESP32 ADC
- BNO055 9-DOF IMU for roll/tilt compensation

## Building

### PC Application
```bash
cd finn-guidance
cargo build --release
cargo run --release
```

### ESP32 Firmware (Phase 4)
```bash
# Install ESP Rust toolchain first:
# cargo install espup && espup install
cd firmware
cargo build --target xtensa-esp32-espidf
```

## License

GPLv3 (matching AgOpenGPS)
