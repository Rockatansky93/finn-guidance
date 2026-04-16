# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 18 — 16 April 2026 (Heading error, max steer angle, data rate, min_pwm fix)

## What we're working on
**Phase 5 — Two-loop auto-steer controller. Four field-test issues fixed.**

Second and third field tests revealed four issues:

**1. Diagonal overshoot (fixed):** Pure XTE control had no heading awareness.
Added heading error feedforward (Kh) to the outer loop.

**2. Full-lock runaway (fixed):** max_steer_angle reduced 25°→15° (adjustable
5°–30° via slider, persisted).

**3. System lag (fixed):** Sensor ESP32 and steer commands both reduced 20Hz→10Hz.
Total serial volume halved.

**4. Bang-bang motor oscillation (fixed):** The min_pwm deadzone compensation was
BOOSTING any non-zero output to ±80 PWM, creating a three-level controller
(0, +80, -80). For straight-line tracking where the inner loop needs small
corrections back and forth, this meant the motor was slamming between ±80 on
every frame — no proportional range in between.

The Trimble EZ-Steer is direct-drive (single shaft, no gears) on the steering
column. The hydraulic steering resistance stalls the motor below ~100 PWM.
This means the boost IS needed (the motor genuinely can't move without it),
but the old approach had no mechanism to prevent hunting.

Fix: kept the boost (non-zero output → ±min_pwm) but added an **inner loop
angle deadband** (2° default). When the angle error is within 2° of the target,
the motor outputs zero — the wheels are "close enough." The motor only fires
when the angle error exceeds 2°, makes its correction, and stops when back
within 2°. This prevents the +100/-100 bang-bang cycle.

Also: min_pwm raised from 80→100 (based on field observation that 80 sometimes
stalls), Kp_angle raised from 4→10 (so realistic angle errors produce proportional
PWM above the 100 threshold), and the firmware brake phase was removed (unnecessary
for direct-drive — steering resistance is the brake).

**Not yet done:**
- **Re-flash BOTH ESP32s** — sensor (10Hz) and motor (brake phase)
- **Fourth field test** — verify all four fixes together
- Tune Kp/Kh/Kp_angle/max_steer_angle/min_pwm/deadband in the field
- Add derivative term (Kd) to outer loop if oscillation persists after tuning
- Wire up CSV export button in job history UI

## Current state of the code
All features below are IMPLEMENTED and working:

- **GPS reading**: auto-detect COM port, PAIR module config (1Hz LC29H DA), NMEA
  parsing (GGA + VTG, epoch-based), crossbeam channel to GUI thread
- **Position interpolation**: dead-reckoning between 1Hz fixes at ~30fps for smooth
  display. Real fixes for coverage/auto-pass, interpolated for GUI.
- **Field view canvas**: heading-up/north-up, zoom, adaptive grid, scale bar,
  vehicle triangle with fix-quality ring, age-faded trail
- **AB line guidance**: set A/B, cross-track error, parallel pass lines, manual
  next/prev, distance-based auto-pass (60% snap threshold, 0.5 m/s speed gate)
- **Lightbar**: 31 segments, green→yellow→red ramp, configurable sensitivity
  (default 20 cm/seg for standalone GPS, reduce for RTK). Adjustable in UI.
- **Implement controls**: width (±0.5m), overlap (±5cm), nudge (±5cm/±1cm fine),
  align-grid-to-here. All persisted except nudge (session-specific by design).
- **AB line persistence**: two-level field→line model, save/load/delete, JSON
  export/import for cross-PC transfer, last-loaded line auto-restored on startup
- **Coverage**: engage/disengage, distance-based logging (0.25m), SQLite storage
  with batch inserts (50-point write buffer), in-memory render cache for field view,
  colour-coded by fix quality, viewport culling, zoom-dependent render thinning
  (step 1–4 with bridging quads for gap-free display), CSV export from DB available.
  Clear button for task transitions. **Field-tested, rendering confirmed solid.**
- **Job management**: JOB HISTORY list in Setup page (last 10, with delete)
- **GUI**: working page (full-screen + overlaid lightbar/XTE/notifications/auto-steer
  indicator) and setup page (scrollable panel with AB LINE, IMPLEMENT, NUDGE,
  GUIDANCE, COVERAGE, POSITION, SENSORS, WAS CALIBRATION, MOTOR DIRECTION,
  AUTO-STEER, MOTOR TEST, VIEW, LIGHTBAR sections). GPS status bar shared across
  both pages. **Framerate capped at ~30fps** via `request_repaint_after(33ms)` to
  keep CPU manageable on field laptops.
- **Configuration persistence**: implement width, overlap, lightbar sensitivity,
  last AB line ID, WAS calibration (centre/left/right lock ADC), motor invert,
  steer Kp, steer Kp_angle — all via SQLite config table, saved on change, loaded
  on startup
- **ESP32 firmware (Arduino/PlatformIO)**: both sensor and motor modules flashed and
  running. Identical serial protocol to the original Rust design.
- **FINN sentence parser**: PC-side parser for `$FINNWAS`, `$FINNIMU`, `$FINNHB`,
  `$FINNMTR` with NMEA-style checksum validation. Unit tested against real serial output.
- **Dual serial port**: sensor ESP32 (COM3) and motor ESP32 (COM6) auto-detected
  and opened independently. Sensor reader handles GPS + FINN, motor reader handles
  motor status + steer commands.
- **Motor test UI**: Setup page MOTOR TEST section with preset PWM buttons, fine
  adjust, emergency stop, live motor status and WAS feedback.
- **WAS calibration**: three-point wizard (centre/left-lock/right-lock) storing raw
  ADC values in SQLite config. PC-side `was_calibrated_angle()` maps ADC to ±45°
  via piecewise linear interpolation. Calibrated angle shown in SENSORS section.
  Latest field values: centre=1832, left lock=1617, right lock=2031. Angle sign
  is correct (left < centre < right → negative angle for left steering).
- **Motor direction**: `motor_invert` toggle in MOTOR DIRECTION section, persisted
  to SQLite. `apply_motor_direction()` applied after steering controller output.
- **Auto-steer controller**: `SteeringController` in `guidance/steering.rs`.
  Two-loop architecture:
  - Outer loop: XTE + heading error → desired steering angle
    (Kp = 30 °/m default, Kh = 0.5 °/° default)
  - Inner loop: desired angle vs WAS actual angle → PWM (Kp_angle = 4 PWM/° default)
  WAS feedback closes the inner loop — wheels return to straight when on-line
  AND heading is aligned. Heading error feedforward prevents the tractor from
  approaching the line at an angle and overshooting through it.
  Safety: GPS fix timeout (2s disengage), WAS timeout tiered (2s warning, 5s
  disengage), speed gate (0.5 m/s), max PWM clamp (180 default), motor deadzone
  compensation (min_pwm 80 — any non-zero output boosted to at least this value).
  Engage button on working page toolbar (requires AB line + motor + WAS calibrated).
  Kp, Kh and Kp_angle adjustable via sliders in Setup page, persisted to SQLite.
  Working page overlay shows live PWM, target angle (T:), actual angle (A:),
  heading error (H:).
  **Steer commands throttled to ~10Hz** (100ms interval) to avoid flooding serial.
  See `docs/STEERING_TUNING_GUIDE.md` for tuning procedure.

## Key decisions (see DECISIONS.md for full detail)
- #001–#004: Auto-pass priority, skip factor, display thinning, GUI page split
- #005: Distance-based auto-pass (replaced heading-based)
- #006: Overlap via pass_spacing() method
- #007: GPS 5Hz config + distance-based logging + COM auto-detect
- #008: Position interpolation for smooth GUI (LC29H DA limited to 1Hz)
- #009: Nudge as independent sub-pass lateral shift
- #010: Field-grouped AB line persistence model
- #011: Align Grid to Here (snap pass grid without changing AB geometry)
- #012: egui 0.29 API migration fixes
- #013: Persist width/overlap/lightbar/last-AB-line, exclude nudge
- #014: Coverage data management (render thinning, memory cap, clear button)
- #015: Arduino/PlatformIO for ESP32 firmware (replaced Rust/esp-idf-hal)
- #016: Dual-ESP32 auto-detect (distinguish sensor vs motor by sentence type)
- #017: Coverage data to SQLite, replacing CSV as primary store (0.25m filter)
- #018: WAS calibration as PC-side three-point mapping, not ESP32-side
- #019: Coverage render thinning bridging fix (quads span i→i+step, not i→i+1)
- #020: Auto-steer as P-control in GUI loop with safety auto-disengage
- #021: Heading error feedforward in outer loop (fixes diagonal overshoot)
- #022: Max steer angle cap (15°) and sensor rate reduction (20Hz→10Hz)

## Phase 4 hardware inventory (as of 15 April 2026)
- **ESP32 #1 (sensor node)**: reads WAS (ADC GPIO 34), BNO055 (I2C GPIO 21/22),
  forwards GPS NMEA via UART2 passthrough (GPIO 16/17). USB A to laptop. FLASHED.
- **ESP32 #2 (controller node)**: drives IBT-2 via PWM (GPIO 25/26) and enable
  lines (GPIO 27/14). USB B to laptop. FLASHED.
- **BNO055**: 3.3V I2C, connects direct to sensor ESP32, no level shifting needed.
- **WAS**: 10kΩ potentiometer. Wiper → GPIO 34 direct (no voltage divider needed —
  pot powered from ESP32 3.3V rail, output 0–3.3V). **Calibrated:** centre=1832,
  left lock=1617, right lock=2031. Angle sign correct.
- **IBT-2**: RPWM/LPWM from ESP32 PWM pins. R_EN/L_EN on GPIO 27/14 for hard stop.
  Logic powered from ESP32 #2 VIN (5V). Motor supply 12V direct from battery.
  **Motor confirmed working — responds to MOTOR TEST buttons in tractor.**
- **GPS**: Quectel LC29H DA on ArduSimple board, UART to sensor ESP32 GPIO 16/17.
  Powered from ESP32 #1 VIN (5V).
- **No buck converter**: ESP32s powered via USB from laptop. IBT-2 logic and GPS
  draw 5V from the VIN pins of their respective ESP32s (bench-tested OK).

## Phase 4 ESP32 pinout map

### ESP32 #1 — Sensor Module (USB-A to laptop)

| GPIO | Function           | Direction | Notes                                          |
|------|--------------------|-----------|-------------------------------------------------|
| 34   | WAS pot wiper (ADC)| Input     | ADC1_CH6. Input-only pin, no pull needed. 0–3.3V from pot |
| 33   | WAS pot VCC (3.3V) | Output    | Set HIGH at boot. Powers high side of 10kΩ pot (0.33mA draw) |
| 21   | BNO055 SDA (I2C)   | I/O       | 3.3V I2C, internal pull-ups OK for short runs   |
| 22   | BNO055 SCL (I2C)   | I/O       | 3.3V I2C, internal pull-ups OK for short runs   |
| 16   | GPS UART2 RX       | Input     | Receives NMEA from ArduSimple 3.3V serial header |
| 17   | GPS UART2 TX       | Output    | Sends config commands to LC29H DA               |
| GND  | Common ground      | —         | Shared with pot low side, BNO055, GPS            |
| VIN  | 5V out to GPS      | Power     | USB-powered. GPS module draws 5V from this pin   |

**WAS pot wiring:** GPIO 33 (HIGH=3.3V) → pot pin 1 → wiper → GPIO 34 (ADC) → pot pin 3 → GND

**Free pins (available for future use):** 4, 5, 12, 13, 14, 15, 18, 19, 23, 25, 26, 27, 32, 35

### ESP32 #2 — Motor Controller (USB-B to laptop)

| GPIO | Function           | Direction | Notes                                          |
|------|--------------------|-----------|-------------------------------------------------|
| 25   | IBT-2 RPWM         | Output    | PWM channel A (steer right). 20kHz recommended  |
| 26   | IBT-2 LPWM         | Output    | PWM channel B (steer left). 20kHz recommended   |
| 27   | IBT-2 R_EN         | Output    | Right enable. HIGH to enable, LOW for hard stop  |
| 14   | IBT-2 L_EN         | Output    | Left enable. HIGH to enable, LOW for hard stop   |
| GND  | Common ground      | —         | Shared with IBT-2 logic GND                      |
| VIN  | 5V out to IBT-2    | Power     | USB-powered. IBT-2 logic VCC draws 5V from this pin |

**Free pins (available for Trimble motor encoder):** 4, 5, 12, 13, 16, 17, 18, 19, 21, 22, 23, 32, 33, 34, 35, 36, 39

### Power distribution

```
Laptop USB-A ──► ESP32 #1 (sensor)
                   ├── 3.3V rail → BNO055, pot ref (GPIO 33)
                   └── VIN (5V)  → GPS module (ArduSimple board)

Laptop USB-B ──► ESP32 #2 (controller)
                   └── VIN (5V)  → IBT-2 logic VCC

12V Battery ───► IBT-2 motor supply (direct, high current)
```

**No buck converter needed.** Both ESP32s are USB-powered from the field laptop.
The 5V VIN pins back-feed 5V to the GPS and IBT-2 logic (bench-tested OK).

## What's blocked
- **RTK**: no base station or NTRIP subscription yet. Running standalone GPS
  (HDOP 0.4 with 42–50 sats — usable for guidance display, not centimetre-accurate)

## Auto-steer field test checklist (THIRD TEST — heading error fix)
**Pre-flight (before engaging auto-steer):**
- [ ] Motor responds to MOTOR TEST preset buttons
- [ ] Motor direction verified: +50 PWM steers RIGHT (toggle motor_invert if not)
- [ ] WAS calibration values loaded (L:1617 C:1832 R:2031)
- [ ] WAS angle reads ~0° when wheels are straight (check SENSORS section)
- [ ] AB line set and loaded
- [ ] Start with Kp = 30 °/m, Kh = 0.5 °/°, Kp_angle = 4 PWM/°, max_pwm = 180, deadband = 3cm

**First engagement:**
- [ ] Drive onto AB line at working speed (~5 km/h)
- [ ] Tap ⊕ AUTO-STEER on working page
- [ ] Green overlay should show: `AUTO-STEER  PWM N  T:-X° A:-Y° H:-Z°`
- [ ] Observe: does the motor steer toward the line? Do T:, A:, H: values make sense?
- [ ] **Key test — heading alignment**: drive 3–5m to the side of the AB line, then
  engage. Watch H: as the tractor approaches. H: should start large (approach angle),
  then decrease as the tractor aligns with the line. The tractor should NOT punch
  through the line diagonally — it should curve in and run parallel.
- [ ] **Key test — steady state**: once on the line, H: should be near 0°, T: near 0°,
  A: near 0°. The tractor should track straight.
- [ ] If motor steers AWAY from line: immediately tap ⊗ STEER OFF, toggle motor_invert
- [ ] If still overshooting/punching through: increase Kh (try 0.8, then 1.0, then 1.2)
- [ ] If too slow to get onto line: increase Kp (try 40, then 50)
- [ ] If oscillating/weaving once on line: reduce Kp (try 20, then 15) or increase Kh
- [ ] If wheels are slow to reach target: increase inner Kp_angle (try 5, then 6)
- [ ] If wheels jerk/buzz: decrease inner Kp_angle (try 3, then 2)

**Safety verification:**
- [ ] Tap ⊗ STEER OFF → motor stops immediately
- [ ] Close app → motor stops within 500ms (watchdog)
- [ ] Unplug motor USB → motor stops, auto-steer disengages
- [ ] Stop tractor (speed < 0.5 m/s) → PWM goes to zero (speed gate)
- [ ] WAS amber warning appears briefly during normal driving (not full disengage)

## Next session should
1. Read this file and DECISIONS.md for context
2. **Re-flash BOTH ESP32s**:
   - Sensor: `cd firmware-sensor-pio && pio run --target upload` (10Hz WAS+IMU)
   - Motor: `cd firmware-motor-pio && pio run --target upload` (direction brake phase)
3. **Fourth field test** — verify all four fixes:
   - Heading error (no diagonal overshoot — watch H: decreasing on approach)
   - Max steer angle (no full-lock runaway — gentle arc back)
   - Data rate (responsive UI, no lag)
   - Min PWM threshold (smooth proportional corrections, no bang-bang slamming)
4. Tune Kp/Kh/Kp_angle/max_steer_angle/min_pwm/deadband.
   New defaults: Kp=30, Kh=0.5, Kp_angle=10, max_angle=15°, min_pwm=80.
   Key relationship: Kp_angle × angle_error must exceed min_pwm for the motor
   to move. At Kp_angle=10, 8° error → 80 PWM (motor starts). Refer to
   STEERING_TUNING_GUIDE.md
5. If oscillation persists after tuning, add derivative term (Kd) to outer loop
6. Wire up CSV export button in job history UI

## File map (quick reference)
```
pc/src/main.rs              — entry point, thread setup, GPS + motor auto-detect
pc/src/gps/reader.rs         — serial port reader, auto-detect, module config, FINN routing
pc/src/gps/parser.rs         — NMEA parsing (GGA + VTG, epoch-based)
pc/src/gps/finn_parser.rs    — FINN sentence parser ($FINNWAS, $FINNIMU, $FINNHB, $FINNMTR)
pc/src/gps/mod.rs            — GPS + serial module declarations
pc/src/comms/serial.rs       — Motor ESP32 serial (auto-detect, MotorHandle, steer commands)
pc/src/comms/mod.rs          — Comms module declarations
pc/src/guidance/ab_line.rs   — AB line guidance, cross-track error, auto-pass, overlap, nudge
pc/src/guidance/steering.rs  — Auto-steer controller (two-loop, WAS feedback, safety)
pc/src/gui/app.rs            — egui application, page split, lightbar, config persistence,
                               SENSORS, WAS CALIBRATION, MOTOR DIRECTION, AUTO-STEER,
                               MOTOR TEST sections. Auto-steer button on working page.
pc/src/gui/field_view.rs     — 2D canvas rendering (grid, coverage strips, lines, trail, vehicle)
pc/src/gui/field_projection.rs — lat/lon → local metres → screen pixels
pc/src/coverage/logger.rs    — coverage logger: filter, buffer, flush to SQLite, render cache
pc/src/coverage/db.rs        — SQLite database (jobs, segments, coverage points, AB lines, fields, config)
pc/src/position/tracker.rs   — position history and odometer
pc/src/position/interpolator.rs — dead-reckoning between 1Hz GPS fixes for smooth GUI
common/src/types.rs          — GpsFix, ImuData (with cal), WasReading (with mV),
                               EspHeartbeat, MotorStatus, CrossTrackError, GuidanceLine
common/src/coords.rs         — haversine, bearing, cross-track distance
common/src/protocol.rs       — FinnMessage enum, nmea_checksum(), format_steer_command()
firmware-sensor-pio/          — ESP32 #1 Arduino/PlatformIO project (ACTIVE)
firmware-motor-pio/           — ESP32 #2 Arduino/PlatformIO project (ACTIVE)
firmware-sensor/              — ESP32 #1 Rust crate (ARCHIVED — replaced by PlatformIO)
firmware-motor/               — ESP32 #2 Rust crate (ARCHIVED — replaced by PlatformIO)
docs/IMPLEMENTATION_PLAN.md  — full phase plan, task tracking, session log
docs/DECISIONS.md            — architectural decision log (#001–#020)
docs/INSTALLATION_GUIDE.md   — hardware wiring, PC setup, ESP32 flashing, troubleshooting
docs/STEERING_TUNING_GUIDE.md — auto-steer setup, tuning procedure, troubleshooting
docs/ACTIVE_CONTEXT.md       — this file
```

## Important conventions
- Use `codesnip:edit_snippet` for code changes, never rewrite entire files
- GPS receiver is a Quectel LC29H on ArduSimple board, connected via USB serial
- GUI framework is egui 0.29 (eframe), rendering via Painter API
- All coordinate math in `common/src/coords.rs`, types in `common/src/types.rs`
- Coverage CSV format (for export): `segment,timestamp_ms,lat,lon,alt,speed,heading,fix_quality,sats,hdop`
- Coverage primary store is SQLite `coverage_points` table (batch inserts, 0.25m distance filter)
- Implement width defaults to 12.0m in `main.rs`, persisted in SQLite config table
- Overlap defaults to 0cm, persisted. Pass spacing = width − overlap.
- Lightbar sensitivity defaults to 20 cm/segment (3.0m full scale), persisted
- Last-loaded AB line auto-restored on startup via `last_ab_line_id` config key
- egui 0.29 uses `id_salt` (not `id_source`), `show_tooltip_text` takes 4 args
- ESP32 firmware uses Arduino/PlatformIO (C++), built with `pio run --target upload`
- Old `pc/src/comms/udp.rs` is dead code — can be deleted (replaced by `serial.rs`)
- Old `coverage_logs/` CSV directory no longer written to (coverage now in SQLite)
- Auto-steer sign convention: positive XTE = right of line → negative desired angle (steer left)
- Two-loop controller: outer (XTE+heading→angle, Kp °/m + Kh °/°), inner (angle error→PWM, Kp_angle PWM/°)
- Heading error: positive = pointed right of line bearing → negative desired angle (steer left to align)
- `apply_motor_direction()` is applied AFTER the steering controller, not inside it
- WAS timeout is tiered: warn at 2s (amber), disengage at 5s (not hard 1s)
- GUI framerate capped at ~30fps (FRAME_INTERVAL = 33ms) — do NOT use uncapped `request_repaint()`
- Motor serial writes throttled to ~10Hz (STEER_SEND_INTERVAL = 100ms) — safety disengages bypass throttle
- Motor stall: min_pwm = 100 (direct-drive EZ-Steer stalls below this against steering resistance). Non-zero output boosted to ±min_pwm. Inner loop angle deadband (2°) prevents bang-bang hunting.
- Trail capped at 5,000 points (was 50K) — ~83 minutes at 1Hz, plenty for a working session
