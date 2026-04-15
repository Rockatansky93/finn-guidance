# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 16 — 15 April 2026 (Auto-steer controller, steering.rs, working page button)

## What we're working on
**Phase 5 — Auto-steer controller implemented. Ready for field test.**

Steering controller module created (`steering.rs`) with proportional control.
Auto-steer button added to the working page toolbar. Kp/max PWM/deadband sliders
in Setup page AUTO-STEER section. Full safety system: GPS fix timeout, WAS data
timeout, minimum speed gate. Motor direction inversion applied after controller
output. WAS calibration updated with new field values (L:1617 C:1832 R:2031) —
angle sign is now correct (left lock ADC < centre < right lock ADC, so negative
angle = left steering as expected).

**Completed this session:**
- New `pc/src/guidance/steering.rs` — `SteeringController` struct with proportional
  control: `pwm = -kp × xte`, clamped to `±max_pwm`, with deadband and speed gate
- Steering controller wired into `app.rs` update loop: computes PWM from
  interpolated XTE every frame (~60fps), sends via `motor_handle.send_steer()`
- `apply_motor_direction()` applied after controller output (removes `#[allow(dead_code)]`)
- Safety: `notify_gps_fix()` called on each real fix, `notify_was_reading()` on
  each WAS message. Auto-disengage if GPS fix age > 2s or WAS data age > 1s.
- Working page: `⊕ AUTO-STEER` / `⊗ STEER OFF` button in bottom toolbar, greyed
  out until AB line loaded + motor connected + WAS calibrated
- Working page overlay: green "AUTO-STEER PWM N" indicator (top-left, below
  lightbar) when engaged; amber status messages for engage/disengage/safety events
- Setup page: AUTO-STEER section between MOTOR DIRECTION and MOTOR TEST, with:
  - Kp slider (20–300 PWM/m, 5-step, persisted to SQLite `steer_kp`)
  - Max PWM slider (50–255, 5-step)
  - Deadband slider (0–20 cm, 1-step)
  - Live status showing engaged/disengaged + last PWM + disengage reason
- WAS calibration values confirmed correct: L:1617 C:1832 R:2031 (left < centre
  < right, so `was_calibrated_angle()` produces correct sign without modification)
- Decision #020 documented

**Previous session (Session 15) completed:**
- WAS three-point calibration wizard in Setup page
- Motor direction toggle with SQLite persistence
- Coverage rendering bridging fix (gap-free at all zoom levels)
- Field-tested in tractor cab on Dell 7390

**Not yet done:**
- **Field test auto-steer** — first test with conservative Kp (start at 100)
- Determine motor direction convention (positive PWM = which steering direction)
  and set motor_invert accordingly — critical before first auto-steer test
- Tune Kp, max_pwm, deadband in the field
- Add derivative term (Kd) if P-only control oscillates
- Add heading error feedforward if P-only tracks poorly on curves
- Wire up CSV export button in job history UI
- Touch target improvements (lower priority, UI is useable)

## Current state of the code
All features below are IMPLEMENTED and working:

- **GPS reading**: auto-detect COM port, PAIR module config (1Hz LC29H DA), NMEA
  parsing (GGA + VTG, epoch-based), crossbeam channel to GUI thread
- **Position interpolation**: dead-reckoning between 1Hz fixes at ~60fps for smooth
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
  both pages.
- **Configuration persistence**: implement width, overlap, lightbar sensitivity,
  last AB line ID, WAS calibration (centre/left/right lock ADC), motor invert,
  steer Kp — all via SQLite config table, saved on change, loaded on startup
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
  Proportional control: `pwm = -kp × xte`, clamped to `±max_pwm`, with deadband
  and minimum speed gate. Safety auto-disengage on GPS fix timeout (2s) or WAS
  data timeout (1s). Engage button on working page toolbar (requires AB line +
  motor + WAS calibrated). Kp/max_pwm/deadband adjustable in Setup page AUTO-STEER
  section. Kp persisted to SQLite. Default tuning: Kp=100, max_pwm=180,
  deadband=3cm, min_speed=0.5m/s.

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

## Auto-steer field test checklist (FIRST TEST)
**Pre-flight (before engaging auto-steer):**
- [ ] Motor responds to MOTOR TEST preset buttons (verified last session)
- [ ] Determine motor direction: hit +50 PWM, observe wheel direction
- [ ] If +50 PWM steers LEFT, toggle motor_invert in MOTOR DIRECTION section
- [ ] Confirm: +50 PWM now steers RIGHT (matches controller convention)
- [ ] WAS calibration values loaded (L:1617 C:1832 R:2031)
- [ ] AB line set and loaded
- [ ] Start with Kp = 100 (conservative), max_pwm = 180, deadband = 3cm

**First engagement:**
- [ ] Drive onto AB line at walking speed (~5 km/h)
- [ ] Tap ⊕ AUTO-STEER on working page
- [ ] Green "AUTO-STEER PWM N" overlay should appear
- [ ] Observe: does the motor respond? Does it steer toward the line?
- [ ] If motor steers AWAY from line: immediately tap ⊗ STEER OFF, toggle motor_invert
- [ ] If oscillating: reduce Kp (try 50)
- [ ] If too sluggish: increase Kp (try 150–200)

**Safety verification:**
- [ ] Tap ⊗ STEER OFF → motor stops immediately
- [ ] Close app → motor stops within 500ms (watchdog)
- [ ] Unplug motor USB → motor stops, auto-steer disengages
- [ ] Stop tractor (speed < 0.5 m/s) → PWM goes to zero (speed gate)

## Next session should
1. Read this file and DECISIONS.md for context
2. **Field test auto-steer** following the checklist above
3. **Determine motor direction** — the FIRST thing to verify before engaging
4. Tune Kp/max_pwm/deadband based on field behaviour
5. If P-only oscillates, add derivative term (Kd) to `steering.rs`
6. Wire up CSV export button in job history UI
7. Consider adding heading error as feedforward term for better curve tracking

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
pc/src/guidance/steering.rs  — Auto-steer controller (P-control, safety, engage/disengage)
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
- Auto-steer sign convention: positive XTE = right of line → negative PWM (steer left)
- `apply_motor_direction()` is applied AFTER the steering controller, not inside it
