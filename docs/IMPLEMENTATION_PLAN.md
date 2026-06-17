# FINN Guidance - Implementation Plan

## Session Continuity

> **New session?** Read these files first, in this order:
> 1. `docs/ACTIVE_CONTEXT.md` — current state, what to work on next
> 2. `docs/DECISIONS.md` — settled architectural decisions (don't revisit)
> 3. This file — for task tracking and session history
>
> Use `codesnip:edit_snippet` for all code changes. Never rewrite entire files.

> **Historical note:** this file contains the original phased build plan and
> older hardware assumptions. For the maintained tractor hardware direction,
> read `docs/HARDWARE_ARCHITECTURE.md` first: LC29H BA direct to the laptop plus
> the motor ESP32 inner loop.

## Overview

Build a Rust-based GPS guidance system for agricultural equipment in phases,
starting with a guidance-only display (no steering) and progressing to full 
auto-steer. Each phase produces a working, testable system.

---

## Phase 1: GPS Reader & Position Display
**Goal:** Read GPS data from the Quectel LC29H and display position on screen.  
**First real-world test:** Walk/drive around the paddock with a laptop and GPS receiver.

### Tasks
1. **Install Rust toolchain**
   - Install rustup: https://rustup.rs
   - `rustup default stable`
   - Verify: `cargo --version`

2. **Get GPS serial working**
   - Connect Quectel lc29h via USB
   - Identify COM port (Device Manager on Windows)
   - Update `GpsConfig` default port in `pc/src/gps/reader.rs`
   - Write a simple test binary that prints raw NMEA sentences to console

3. **NMEA parsing**
   - Parse GGA sentences (position, fix quality, satellites, HDOP)
   - Parse VTG sentences (speed, heading)
   - Build `GpsFix` structs from parsed data
   - Epoch-based fix emission: only one fix per GGA sentence (not per NMEA sentence)
   - Unit tests with sample NMEA strings

4. **Basic GUI window**
   - Display GPS status bar (fix quality, satellite count, HDOP)
   - Display current lat/lon/altitude
   - Display speed and heading
   - Colour-coded fix quality indicator (green=RTK, yellow=float, red=no fix)

5. **Position trail**
   - Record position history
   - Draw breadcrumb trail on a 2D canvas
   - Auto-scale view to fit trail

6. **Field view canvas**
   - 2D canvas with heading-up and north-up modes
   - Zoom in/out with scroll wheel
   - Grid overlay with adaptive spacing
   - Scale bar indicator
   - Vehicle position triangle with fix-quality colour ring
   - Trail with age-based colour fading

7. **Coverage logging**
   - Engage/disengage toggle for recording GPS positions
   - CSV output: segment, timestamp, lat, lon, alt, speed, heading, fix_quality, sats, hdop
   - Deduplication: only log on new GPS epoch (one point per GGA sentence)
   - Configurable filtering: minimum distance (m) and/or minimum time interval (ms)
   - In-memory point storage for canvas rendering
   - Load previous CSV files for review
   - SQLite database for job metadata: jobs, segments, saved AB lines, config

### Done when
- Can walk around paddock with laptop + GPS and see live position updating
- Fix quality indicator works correctly
- Position trail draws a recognisable path
- Coverage logger writes reasonable-sized CSV files (~1 point/second at 1Hz GPS)
- Coverage database persists job metadata and AB lines

### Estimated effort: 2-3 sessions

---

## Phase 2: AB Line Guidance & Auto-Pass Selection
**Goal:** Set A/B points, see cross-track error in real-time, and have the system
automatically select the correct pass line when turning at the headland.  
**Test:** Drive tractor up and down a paddock, observe guidance accuracy and auto-pass switching.

### Tasks
1. **AB line creation** ✅
2. **Cross-track error calculation** ✅
3. **Pass management** ✅ (manual next/prev)
4. **Auto-pass selection** ✅ (distance-based, Decision #005)
5. **Lightbar indicator** ✅
6. **AB line persistence** ✅ (field→line model, Decision #010)

### Done when ✅ (all criteria met)

### Estimated effort: 3-4 sessions

---

## Phase 3: GUI Pages, Coverage Display & Config
**Goal:** Split the interface into purpose-built pages for working vs setup,
render real-time coverage on the field view, and manage data volume efficiently.

### Tasks
1. **GUI page system** ✅
2. **Coverage rendering on field view** ✅
3. **Coverage data management** ✅
4. **Job management** ✅
5. **Configuration persistence** ✅

### Done when ✅ (all criteria met)

### Estimated effort: 4-5 sessions

---

## Phase 4: ESP32 Sensor & Motor Firmware
**Goal:** ESP32 firmware that reads sensors and controls the steering motor,
communicating with the PC over USB serial using text-based NMEA-style protocol.

**Platform:** Arduino/PlatformIO (C++). See Decision #015 for rationale — Rust
ESP-IDF toolchain was abandoned after multiple blocking toolchain issues.

### Tasks
1. **ESP32 development environment** ✅
   - ~~Install ESP Rust toolchain (`espup install`)~~ (abandoned — Decision #015)
   - Installed PlatformIO CLI (`pip install platformio`)
   - Arduino framework for ESP32 DevKit
   - Build and flash verified working for both modules

2. **Sensor module firmware (ESP32 #1)** ✅
   - `firmware-sensor-pio/` — PlatformIO Arduino project
   - WAS: ADC read on GPIO 34, powered by GPIO 33 (3.3V HIGH), 20Hz
   - BNO055: Adafruit library on I2C (GPIO 21/22), NDOF mode, 20Hz
   - GPS: NMEA passthrough from ArduSimple via UART2 (GPIO 16 RX / 17 TX)
   - Heartbeat every 2 seconds
   - Output: `$FINNWAS`, `$FINNIMU`, `$FINNHB`, raw `$GPGGA`/`$GPVTG` passthrough
   - Flashed and running ✅

3. **Motor controller firmware (ESP32 #2)** ✅
   - `firmware-motor-pio/` — PlatformIO Arduino project
   - IBT-2: PWM on GPIO 25 (RPWM) / 26 (LPWM), 20kHz 8-bit via LEDC
   - Enable lines: GPIO 27 (R_EN) / 14 (L_EN), LOW on startup
   - Serial command parsing: `$FINNSTEER,<pwm>*<checksum>`
   - Watchdog: motor stops if no valid command for 500ms
   - Status output at 5Hz: `$FINNMTR,<pwm>,<enabled>,<uptime>*<checksum>`
   - Flashed and running ✅

4. **PC-side FINN sentence parser** ✅
   - Created `finn_parser.rs` to parse $FINNWAS, $FINNIMU, $FINNHB, $FINNMTR
     with NMEA checksum validation. Unit tested.
   - Route sensor data to GUI (WAS readout, IMU heading/calibration status)
   - Route motor status to GUI (PWM, enabled state)
   - Separate serial port for motor controller (second USB, auto-detected)

5. **WAS calibration** ✅
   - Three-point calibration wizard (centre, left-lock, right-lock)
   - Stored in SQLite config table (was_centre, was_left_lock, was_right_lock)
   - Piecewise linear mapping: raw ADC → ±45° normalised angle
   - "Recalibrate" button clears all three values
   - Field-tested: L:1617 C:1832 R:2031 (angle sign correct)

6. **Bench testing** ✅
   - Sensor ESP32: WAS pot sweep, BNO055 movement, GPS passthrough
   - Motor ESP32: PWM response, watchdog timeout, status messages
   - Full system: dual auto-detect, sensor data pipeline to GUI

### Done when ✅ (all criteria met)
- ESP32 sensor module reads WAS, IMU, and GPS, sends data to PC over USB ✅
- ESP32 motor controller drives motor in response to PC commands ✅
- Motor stops automatically if communication lost ✅
- PC application parses FINN sentences and displays sensor data ✅
- WAS calibration routine stores and applies calibration values ✅
- All sensors bench-tested and verified ✅
- Field-tested in tractor cab on Dell 7390 ✅

### Estimated effort: 4-5 sessions (firmware done in 1, PC integration remaining)

---

## Phase 5: Closed-Loop Auto-Steer
**Goal:** Full auto-steer with closed-loop steering controller.

### Tasks
1. **Steering controller on PC** ✅
   - Two-loop architecture in `guidance/steering.rs` (Decision #020)
   - Outer loop: XTE → desired steering angle (Kp, degrees per metre)
   - Inner loop: desired angle vs WAS actual angle → motor PWM (Kp_angle, PWM per degree)
   - WAS feedback closes the inner loop — wheels return to straight automatically
   - Configurable: Kp (outer), Kp_angle (inner), max_pwm, deadband, min speed
   - GUI sliders for all parameters in Setup page AUTO-STEER section

2. **Steer integration** ✅
   - Controller runs in GUI update loop (~60fps), sends PWM every frame
   - Motor direction inversion applied after controller output
   - WAS angle from `was_calibrated_angle()` fed into inner loop
   - Working page: ⊕ AUTO-STEER / ⊗ STEER OFF button on toolbar
   - Working page overlay: live PWM, target angle (T:), actual angle (A:)

3. **Safety systems** ✅
   - GPS fix timeout: auto-disengage after 2 seconds without fix
   - WAS data tiered response: warning after 2s, disengage after 5s
   - Speed gate: no steering below 0.5 m/s
   - Max PWM clamp (configurable, default 180)
   - Engage preconditions: AB line + motor + WAS calibrated
   - ESP32 watchdog: motor stops within 500ms if serial commands stop
   - Manual disengage always available

4. **Field testing and tuning** — IN PROGRESS
   - [x] First field test: motor responds, safety systems work
   - [x] Identified open-loop bug (no WAS feedback) — fixed with two-loop architecture
   - [x] Identified WAS timeout too aggressive — fixed with tiered timeout
   - [ ] Second field test with two-loop controller
   - [ ] Tune Kp, Kp_angle, deadband for standalone GPS
   - [ ] Document tuning procedure (STEERING_TUNING_GUIDE.md created)

5. **Future improvements** 🔲
   - Add derivative term (Kd) to outer loop if oscillation persists after tuning
   - Add heading error feedforward for better curve tracking
   - IMU roll compensation (adjust for hillside)
   - Speed-dependent gain adjustment
   - Physical disengage switch input on ESP32
   - Real-time plots: steer command vs actual angle vs XTE
   - Save tuning profiles per vehicle

6. **Auto-turn at headland** 🔲
   - Detect end of run (approaching field boundary or user-defined headland line)
   - Generate turn path to the next pass line
   - Configurable skip factor determines which pass to target: skip factor 2 means
     work 1→3→5→7 then fill 2→4→6→8 (reduces turn tightness for wide implements)
   - Note: auto-pass selection (Phase 2) is distance-based and skip-agnostic, so
     skip factor is only relevant here for path generation, not line selection
   - Turn path types: U-turn (tight), bulb turn (wider), or figure-of-7
   - Minimum turning radius constraint from vehicle geometry
   - Speed reduction approaching the turn zone
   - Smooth handoff: straight-line PID → turn path following → straight-line PID
   - Disengage auto-turn if operator touches steering or hits disengage

5. **Tuning interface**
   - Live P, I, D sliders in GUI
   - Real-time plots: steer command vs actual angle vs cross-track error
   - Save tuning profiles per vehicle

### Done when
- Tractor steers itself along AB line with acceptable accuracy ✅ (controller implemented)
- Safety systems prevent dangerous behaviour ✅
- Can tune controller gains without reflashing firmware ✅
- Two-loop controller field-tested and tuned 🔲

### Estimated effort: 4-6 sessions (controller done, field tuning remaining)

---

## Phase 6: FINN Integration
**Goal:** Connect guidance system to FINN network as a node.

### Tasks
1. **FINN node registration**
   - Guidance PC registers as a FINN worker node
   - Exposes capabilities: GPS position, guidance state, field data
   - ESP32 optionally registers as a sensor node (Tier 0/1)

2. **Voice interface integration**
   - FINN interface node can trigger guidance actions via voice
   - "Set A point" / "Set B point" / "Next pass" via speech
   - Voice readback of guidance status

3. **Data sharing**
   - Push coverage data to FINN hub
   - Share field maps across nodes
   - Historical field data for season planning

### Estimated effort: 3-4 sessions

---

## Current Status

**Phase 1: COMPLETE**
- [x] Project structure created
- [x] Common types defined (GpsFix, ImuData, WasReading, GuidanceLine)
- [x] Protocol types defined (ESP<->PC messages, ports)
- [x] Coordinate math implemented (haversine, bearing, cross-track)
- [x] GPS reader (serial port thread, sends fixes via crossbeam channel)
- [x] NMEA parsing (GGA for position, VTG for speed/heading)
- [x] Epoch-based fix emission (one fix per GGA, not per NMEA sentence)
- [x] AB line guidance calculator (cross-track error, pass offset)
- [x] GUI with status bar, guidance readout, and controls
- [x] Field view canvas (heading-up/north-up, zoom, grid, scale bar)
- [x] Vehicle triangle with fix-quality colour ring
- [x] Position trail with age-based colour fading
- [x] Coverage logger with engage/disengage toggle
- [x] Coverage CSV output with deduplication (epoch-based)
- [x] Configurable log filtering (distance and/or time based)
- [x] Coverage SQLite database (jobs, segments, AB lines, config)
- [x] Load previous coverage CSV files
- [x] First field test completed (walk test, GPS fix confirmed, coverage recorded)
- [x] Second field test completed (ute drive, 42 sats, HDOP 0.4, trail + AB line working)

**Phase 2: COMPLETE**
- [x] AB line set A/B from GPS position
- [x] Cross-track error calculation with pass offset
- [x] Large cm readout with colour coding
- [x] Pass management (next/prev, implement width)
- [x] Parallel pass lines drawn on field view
- [x] Pass line normal vector corrected to match cross-track sign convention
- [x] Auto-pass selection (distance-based, snap at 60% of pass spacing)
- [x] Auto-pass toggle button and on-screen notification
- [x] Active pass line rendered in blue, distinct from red reference lines
- [x] Third field test (ute drive, 46-50 sats, coverage strips + auto-pass verified)
- [x] Lightbar indicator (31 segments, green→yellow→red, configurable sensitivity)
- [x] Implement width adjustable in UI (0.5m steps, synced to guidance + coverage)
- [x] Overlap adjustable in UI (5cm steps, pass_spacing = width − overlap)
- [x] Nudge (±5cm/±1cm fine, amber indicator, align-grid-to-here)
- [x] AB line persistence (field→line model, save/load/delete, JSON export/import)
- [x] Last-loaded AB line auto-restored on startup
- [x] Fourth field test — AB line save/load and nudge confirmed working

**Phase 3: COMPLETE**
- [x] GUI page system (working page + setup page, shared GPS status bar)
- [x] Coverage rendering on field view (strips with fix quality colours, viewport culling)
- [x] Coverage data management (zoom-dependent render thinning, 100k memory cap, clear button)
- [x] Job management UI (JOB HISTORY list with delete)
- [x] Configuration persistence (implement width, overlap, lightbar sensitivity, last AB line)

**Phase 4: COMPLETE**
- [x] PlatformIO development environment installed
- [x] Sensor module firmware written and flashed (firmware-sensor-pio/)
- [x] Motor controller firmware written and flashed (firmware-motor-pio/)
- [x] Serial protocol implemented (FINNWAS, FINNIMU, FINNHB, FINNSTEER, FINNMTR)
- [x] Watchdog safety (500ms timeout, motor stops automatically)
- [x] PC-side FINN sentence parser (finn_parser.rs — parses $FINN* with checksums)
- [x] PC-side sensor data pipeline (crossbeam channel, GUI SENSORS section)
- [x] PC-side motor command sender (second serial port, MotorHandle, MOTOR TEST UI)
- [x] Dual-ESP32 auto-detect (sensor vs motor by sentence type, Decision #016)
- [x] Bench testing (WAS pot sweep, BNO055 movement, GPS passthrough, motor status)
- [x] WAS calibration wizard (three-point, PC-side piecewise linear, Decision #018)
- [x] Motor direction toggle with SQLite persistence
- [x] Field test on tractor (Dell 7390, coverage rendering, WAS calibration confirmed)

**Phase 5: IN PROGRESS** (controller implemented, field tuning remaining)
- [x] Steering controller implemented (guidance/steering.rs)
- [x] Two-loop architecture: outer (XTE→angle) + inner (angle error→PWM)
- [x] WAS feedback integrated into inner control loop
- [x] Safety systems: GPS timeout, tiered WAS timeout, speed gate, max PWM
- [x] Auto-steer engage/disengage button on working page toolbar
- [x] Working page overlay: live PWM, target angle, actual angle
- [x] Setup page: Kp (°/m), Kp_angle (PWM/°), max PWM, deadband sliders
- [x] Kp and Kp_angle persisted to SQLite config
- [x] First field test: motor responds, safety works, identified open-loop bug
- [x] Fixed: two-loop controller with WAS feedback (wheels return to straight)
- [x] Fixed: WAS timeout tiered (warn 2s, disengage 5s instead of hard 1s)
- [x] Steering tuning guide written (docs/STEERING_TUNING_GUIDE.md)
- [ ] Second field test with two-loop controller
- [ ] Tune Kp, Kp_angle, deadband for standalone GPS

### Next up: Field test two-loop controller
- Follow AUTO-STEER field test checklist in ACTIVE_CONTEXT.md
- Verify motor direction convention (critical first step)
- Start with Kp=30, Kp_angle=4, max_pwm=180, deadband=3cm
- Tune using live sliders in Setup page
- Refer to STEERING_TUNING_GUIDE.md for tuning procedure

---

## Session Log

### Session 1 (Feb 2026)
- Created project structure (workspace: common, pc, firmware crates)
- Defined core types: GpsFix, ImuData, WasReading, GuidanceLine, CrossTrackError
- Implemented coordinate math: haversine, bearing, cross-track distance with tests
- Set up UDP protocol types and constants
- GPS reader skeleton with serial port and NMEA parsing
- AB line guidance calculator with cross-track error
- Basic GUI skeleton (status bar, controls, guidance readout)
- Position tracker with odometer

### Session 2 (Mar 2026)
- Built full field view canvas with FieldProjection coordinate system
- Heading-up and north-up view modes with smooth rotation
- Adaptive grid overlay with scale bar
- Vehicle position triangle with heading indicator
- Position trail with age-based fading
- AB line and parallel pass rendering on canvas
- Coverage logger with engage/disengage toggle
- First GPS field test: walked around paddock, confirmed live position display

### Session 3 (Mar 2026)
- Identified critical logging bug: 27x point duplication per GPS epoch
- Fixed NMEA parser: epoch-based emission, only emit fix on GGA sentences
- Rewrote coverage logger with three-gate filtering
- Added SQLite coverage database (rusqlite with bundled feature)
- Reorganised Phase 1-3 in implementation plan

### Session 4 (Mar 2026) — Ute road test + planning
- Second field test: drove ute along road with laptop + GPS
- Confirmed 42 satellites, HDOP 0.4 (excellent signal quality)
- Reviewed real-world usability and identified key improvements needed
- Updated implementation plan with auto-pass and GUI priorities

### Session 5 (27 Mar 2026) — Auto-pass, coverage rendering, field test + fixes
- Implemented auto-pass selection (initially heading-based, then replaced)
- Implemented coverage strip rendering on field view canvas
- Third field test: ute drive with coverage + auto-pass
- Replaced heading-based auto-pass with distance-based approach (Decision #005)

### Session 6 (27 Mar 2026) — GUI page split, lightbar, implement width/overlap
- Implemented GUI page split (Decision #004)
- Implemented lightbar indicator (31 segments)
- Implemented adjustable implement width and overlap (Decision #006)

### Session 7 (27 Mar 2026) — GPS 5Hz, auto-detect, distance-based logging
- Implemented GPS module configuration on startup (Decision #007)
- Implemented auto-detection of GPS serial port
- Changed coverage logger to distance-based filtering
- Discovered LC29H DA limited to 1Hz PVT
- Implemented position interpolation (Decision #008)
- Changed lightbar sensitivity to 20 cm/segment

### Session 8 (31 Mar 2026) — Nudge, align grid to here
- Implemented nudge feature (Decision #009)
- Implemented "Align Grid to Here" (Decision #011)

### Session 9 (31 Mar 2026) — AB line persistence, field→line model
- Implemented AB line persistence (Decision #010)
- Two-level field→line model with JSON export/import

### Session 10 (1 Apr 2026) — Bugfixes, config persistence, coverage management, Phase 3 completion
- Fixed compile errors from egui 0.29 API changes (Decision #012)
- Field tested — AB line save/load and nudge confirmed working
- Implemented configuration persistence (Decision #013)
- Implemented coverage data management (Decision #014)
- Implemented job management UI
- **Phase 3 complete.**

### Session 11 (10 Apr 2026) — ESP32 firmware: Rust abandoned, Arduino/PlatformIO adopted
- Attempted Rust/esp-idf-hal firmware build for ESP32 sensor and motor modules
- Encountered cascading toolchain issues (Decision #015):
  - Cargo workspace conflict → fixed with `workspace.exclude`
  - Missing Xtensa target → fixed with `rustup override set esp`
  - No prebuilt `core` → fixed with `build-std` in `.cargo/config.toml`
  - Windows path too long → fixed with `CARGO_TARGET_DIR=/c/espbuild`
  - `time_t` size mismatch (`i64` vs `i32`) between esp-idf-sys and toolchain
  - `i8`/`u8` pointer mismatch in esp-idf-svc TLS bindings
  - Version resolution failures (esp-idf-hal 0.44 vs 0.46 conflict)
  - 465MB esp-clang download repeatedly failing on unstable internet (final straw)
- Used `cargo generate esp-rs/esp-idf-template` to identify correct version
  combinations — confirmed the ecosystem requires git-master crates with
  ESP-IDF v5.5.3, `espidf_time64` rustflag, and `[patch.crates-io]` overrides
- **Decision: abandon Rust for ESP32 firmware, switch to Arduino/PlatformIO (C++)**
- Created `firmware-sensor-pio/` — PlatformIO Arduino project for ESP32 #1:
  - WAS ADC (GPIO 34), GPIO 33 HIGH for pot power
  - BNO055 via Adafruit library on I2C (GPIO 21/22)
  - GPS NMEA passthrough via HardwareSerial UART2 (GPIO 16/17)
  - 20Hz sensor output, 2s heartbeat, NMEA checksum on all FINN sentences
- Created `firmware-motor-pio/` — PlatformIO Arduino project for ESP32 #2:
  - IBT-2 PWM via LEDC (GPIO 25/26, 20kHz 8-bit)
  - Enable lines (GPIO 27/14), LOW on startup
  - `$FINNSTEER` command parsing with checksum verification
  - 500ms watchdog, 5Hz status output
- **Both ESP32 modules flashed successfully** — total time ~10 minutes vs hours
  of failed Rust toolchain debugging
- Updated root `Cargo.toml` workspace exclude to include PlatformIO directories
- Serial protocol identical to original Rust design — PC-side code unaffected
- Old Rust firmware crates (`firmware-sensor/`, `firmware-motor/`) archived in repo

### Session 12 (10 Apr 2026) — Sensor bench testing
- Bench tested sensor ESP32 with all three sensors connected
- WAS pot: clean linear sweep 0–4095 across rotation, no noise
- BNO055: roll/pitch/heading responding to movement, cal status updating
- GPS passthrough: full NMEA constellation data flowing (3D fix, HDOP 0.86)
- All three data streams interleaved correctly on single USB serial

### Session 13 (10 Apr 2026) — PC-side FINN parser, motor test UI, dual auto-detect
- Created `finn_parser.rs` — parses $FINNWAS, $FINNIMU, $FINNHB, $FINNMTR with
  NMEA checksum validation. Unit tests against real serial output (all passing).
- Rewrote `protocol.rs` — replaced old UDP-based types with actual serial protocol.
  New `FinnMessage` enum, `nmea_checksum()`, `format_steer_command()`.
- Updated `types.rs` — ImuData now has cal fields, WasReading has voltage_mv,
  added EspHeartbeat and MotorStatus structs.
- Wired FINN channel (crossbeam bounded:128) from serial reader to GUI.
- Added SENSORS section to Setup page — live WAS/IMU/heartbeat display.
- Built motor serial reader (`comms/serial.rs`) — auto-detects motor ESP32 on
  separate COM port, MotorHandle with Arc<Mutex> for thread-safe steer commands.
- Added MOTOR TEST section to Setup page — preset PWM buttons (-100 to +100),
  ±10 fine adjust, emergency STOP, motor status and WAS feedback display.
- Fixed dual-ESP32 auto-detect issue (Decision #016): sensor reader now
  distinguishes sensor ESP32 ($FINNWAS/$FINNIMU) from motor ESP32 ($FINNMTR),
  skips motor port. Reads 40 lines to catch FINN sentences when GPS NMEA arrives
  first. Skips GPS module config (PAIR commands) for ESP32 connections.
- Fixed port coordination: sensor reader reports claimed port via channel so
  motor reader excludes it during auto-detect.
- Replaced `comms/udp.rs` with `comms/serial.rs`.
- **Full bench test successful**: sensor data + motor status flowing through
  complete pipeline to GUI. Motor test panel connected and responsive.
- **Hardware being installed on tractor for first full prototype field test.**

### Session 14 (13 Apr 2026) — Coverage SQLite migration
- Migrated coverage point storage from CSV to SQLite (Decision #017)
- Distance filter reduced from 1.0m to 0.25m for gap-free rendering
- `coverage_points` table with batch inserts (50-point write buffer)
- Removed all CSV file handling from logger.rs
- Export function preserved for CSV output from database

### Session 15 (15 Apr 2026) — WAS calibration, coverage fix, motor direction, field test
- WAS three-point calibration wizard implemented (Set Centre / Left Lock / Right Lock)
- WAS calibration values stored in SQLite config, loaded on startup
- `was_calibrated_angle()` piecewise linear mapping: raw ADC → ±45°
- Motor direction toggle (`motor_invert` config key) with Setup page UI
- `apply_motor_direction()` helper ready for steering controller
- Coverage render thinning bridging fix (Decision #019): quads span i→i+step
- Field-tested in tractor cab on Dell 7390 — coverage rendering solid
- IBT-2 motor controller did not respond — suspected hardware failure
- Decisions #018, #019 documented

### Session 16 (15 Apr 2026) — Auto-steer controller, two-loop architecture, field test
- Created `guidance/steering.rs` — initial P-only controller (pwm = -kp × xte)
- Auto-steer button on working page toolbar, Kp slider in Setup page
- Safety: GPS timeout (2s disengage), WAS timeout (1s disengage), speed gate
- Engage preconditions: AB line + motor connected + WAS calibrated
- Working page overlay: green "AUTO-STEER PWM N" indicator
- **First field test**: motor responds to auto-steer, safety systems functional
- **Bug identified**: P-only controller had no WAS feedback — motor steered toward
  line but didn't straighten wheels on arrival, causing overshoot
- **Bug identified**: WAS 1-second timeout too aggressive — brief serial hiccups
  triggered disengage during normal operation
- **Fix**: rewrote controller with two-loop architecture (Decision #020 revised):
  - Outer loop: XTE → desired steering angle (Kp = 30 °/m)
  - Inner loop: desired angle vs WAS actual angle → PWM (Kp_angle = 4 PWM/°)
  - Wheels now return to straight when line is reached (inner loop drives to 0°)
- **Fix**: WAS timeout tiered — warn at 2s (amber indicator), disengage at 5s
- Setup page: two sliders (Kp outer, Kp_angle inner), both persisted to SQLite
- Working page overlay: shows target angle (T:) and actual angle (A:) for field tuning
- Steering tuning guide written (docs/STEERING_TUNING_GUIDE.md)
- Decision #020 documented
- **Phase 4 marked COMPLETE. Phase 5 in progress — awaiting second field test.**

---

## Hardware Shopping List

Already have:
- [x] 24V 500rpm brushed DC motor
- [x] Wheel angle sensor (WAS) — 10kΩ potentiometer
- [x] H-bridge motor driver (IBT-2)
- [x] GPS receiver (Quectel LC29H DA on ArduSimple board)
- [x] 2× ESP32 DevKit modules (flashed and running)
- [x] BNO055 IMU breakout board

Still needed:
- [ ] Mounting hardware for motor + GPS antenna
- [ ] RTK base station OR NTRIP subscription for corrections
- [ ] Dell Latitude 7390 2-in-1 field laptops (on order)

---

## Key Dependencies

### PC application (Rust)

| Crate           | Version | Purpose                          |
|-----------------|---------|----------------------------------|
| nmea            | 0.7     | NMEA sentence parsing            |
| serialport      | 4.0     | Serial port for GPS + ESP32s     |
| eframe/egui     | 0.29    | GUI framework                    |
| rusqlite        | 0.31    | Coverage database (bundled SQLite)|
| serde           | 1.0     | Serialisation for JSON export    |
| crossbeam       | 0.5     | Thread-safe channels             |
| chrono          | 0.4     | Timestamps                       |

### ESP32 firmware (Arduino/PlatformIO)

| Library                    | Purpose                          |
|----------------------------|----------------------------------|
| espressif32 (platform)     | ESP32 Arduino framework          |
| Adafruit BNO055            | IMU driver (sensor module)       |
| Adafruit Unified Sensor    | Sensor abstraction (BNO055 dep)  |
