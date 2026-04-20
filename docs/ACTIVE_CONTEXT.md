# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 20 — 18 April 2026 (Waveform-aware inner loop: rate damping, smooth taper, sub-stall pulsing)

## What we're working on
**Phase 5 continued — Waveform-aware inner loop rewrite (Decision #025).**

Insight from real driving: straight-line tractor steering is not straight — the
wheel constantly oscillates left and right. The controller's job is to minimise
the *amplitude* of that oscillation, not snap to a fixed angle. The previous inner
loop's hard deadbands clipped the control signal at the zero-crossing (where fine
control matters most) and the min_pwm stall floor created bang-bang switching.

Three changes implemented in `guidance/steering.rs`, all in the inner loop.
Outer loop (pure pursuit from #023) is unchanged:

**1. XTE rate damping (dXTE/dt) — Decision #025, Change 1.**
New `kd_xte` field (default 0.5). Each `compute()` call tracks how fast XTE is
changing. When converging on the line, the damping term reduces the desired angle
(prevents overshoot). When diverging, it adds urgency. This is the core mechanism
that makes the oscillation amplitude shrink over successive cycles. Rate gated to
reject stale/double samples (dt must be 1ms–1s). State reset on engage/disengage.

**2. Smooth taper replaces hard XTE deadband — Decision #025, Change 2.**
Instead of snapping `desired_angle` to 0 when XTE < 3cm, the desired angle now
tapers linearly to zero via `taper = |XTE| / deadband_m`. The waveform passes
through the zero-crossing smoothly — no more signal clipping in the zone where
fine control matters most.

**3. Sub-stall pulsing replaces hard angle deadband — Decision #025, Change 3.**
The old `angle_error < 2° → PWM = 0` deadband is removed. Instead, when desired
PWM is below `min_pwm`, it's accumulated over cycles. When the accumulator
reaches min_pwm, one pulse is fired. This gives time-averaged torque below the
stall floor — small periodic corrections instead of dead-silence → sudden kick.
The motor only goes truly silent when the entire control chain (pursuit + damping
+ taper) outputs essentially nothing (`|desired_pwm| < 1`).

**Not yet done:**
- **Field test 6** — verify waveform-aware inner loop behaviour.
- Tune `kd_xte` from default 0.5 based on field observations.
- Remove or relabel the now-inert angle deadband slider.
- Update `docs/STEERING_TUNING_GUIDE.md` to describe the new three-slider tuning
  procedure (Approach Aggression, Online Aggression, XTE Damping).

## Current state of the code
All features below are IMPLEMENTED. Changes in Session 20 are marked **S20**.
Previous session changes marked **S19**.

- **GPS reading**: auto-detect COM port, PAIR module config (1Hz LC29H DA), NMEA
  parsing (GGA + VTG, epoch-based), crossbeam channel to GUI thread
- **Position interpolation**: dead-reckoning between 1Hz fixes at ~30fps for smooth
  display. Real fixes for coverage/auto-pass, interpolated for GUI. **S19:**
  `interpolate(override_heading)` takes optional fused heading; when supplied,
  both projects position along it and overwrites `fix.heading` on the returned
  synthetic fix. **S19:** `HeadingFilter` in `position/heading_filter.rs` fuses
  BNO055 IMU yaw (10 Hz) with GPS COG (1 Hz) via complementary filter.
- **Field view canvas**: heading-up/north-up (now driven by fused heading),
  zoom, adaptive grid, scale bar, vehicle triangle with fix-quality ring,
  age-faded trail
- **AB line guidance**: set A/B, cross-track error (using fused heading for the
  heading-error term), parallel pass lines, manual next/prev, distance-based
  auto-pass (60% snap threshold, 0.5 m/s speed gate)
- **Lightbar**: 31 segments, green→yellow→red ramp, configurable sensitivity
- **Implement controls**: width (±0.5m), overlap (±5cm), nudge (±5cm/±1cm fine),
  align-grid-to-here. All persisted except nudge (session-specific by design).
- **AB line persistence**: two-level field→line model, save/load/delete, JSON
  export/import for cross-PC transfer, last-loaded line auto-restored on startup
- **Coverage**: engage/disengage, distance-based logging (0.25m), SQLite storage,
  in-memory render cache, colour-coded by fix quality, viewport culling,
  zoom-dependent render thinning with bridging quads. Clear button. Field-tested.
- **Job management**: JOB HISTORY list in Setup page (last 10, with delete)
- **GUI**: working page (full-screen + overlaid lightbar/XTE/notifications/auto-steer
  indicator) and setup page. **S19:** auto-steer overlay now shows lookahead
  distance (`L:N.Nm`) alongside PWM/target/actual/heading. **S19:** SENSORS
  panel shows fused heading with colour-coded filter status. **S20:** new
  "XTE damping (Kd)" slider in AUTO-STEER section. GPS status bar
  shared across both pages. Framerate capped at ~30fps.
- **Configuration persistence**: implement width, overlap, lightbar sensitivity,
  last AB line ID, WAS calibration, motor invert, **S19:** `steer_lookahead_base`,
  `steer_lookahead_speed_factor`, `steer_wheelbase`, **S20:** `steer_kd_xte`,
  plus retained `steer_max_angle`, `steer_kp_angle`. Old `steer_kp` / `steer_kh`
  keys ignored (not removed).
- **ESP32 firmware**: both sensor and motor modules flashed. Unchanged this session.
- **FINN sentence parser**: PC-side parser for `$FINNWAS`, `$FINNIMU`, `$FINNHB`,
  `$FINNMTR` with NMEA-style checksum validation.
- **Dual serial port**: sensor ESP32 (COM3) and motor ESP32 (COM6) auto-detected.
- **Motor test UI**: Setup page MOTOR TEST section with preset PWM buttons,
  fine adjust, emergency stop, live motor status and WAS feedback.
- **WAS calibration**: three-point wizard (centre/left-lock/right-lock).
  Current values: centre=1832, left=1617, right=2031.
- **Motor direction**: `motor_invert` toggle persisted to SQLite.
- **Auto-steer controller**: `SteeringController` in `guidance/steering.rs`.
  **S19 ARCHITECTURE:**
  - Outer loop: **pure pursuit** — lookahead point on the AB line, bicycle-model
    curvature. Single geometric quantity captures XTE + heading. Parameters:
    `lookahead_base` (m), `lookahead_speed_factor` (s), `wheelbase_m` (m),
    `max_steer_angle` (°). Removed: `kp`, `kh`.
  - **S20 — Waveform-aware inner loop rewrite:**
    - XTE rate damping (`kd_xte`, default 0.5): dXTE/dt reduces corrections
      when converging, increases when diverging. Core amplitude reduction.
    - Smooth taper replaces hard XTE deadband: desired angle scales linearly
      from 0 at XTE=0 to full at XTE=deadband_m. No zero-crossing clipping.
    - Sub-stall pulsing: accumulates sub-min_pwm effort, fires periodic pulses.
      Replaces hard angle deadband. Motor only silent when entire chain ≈ 0.
    - `angle_deadband_deg` retained in struct/UI but no longer used by compute().
  Safety: GPS fix timeout (2s disengage), WAS timeout tiered, speed gate,
  max PWM clamp, motor deadzone compensation. Engage button on working page.
  Three tuning sliders (Approach Aggression, Online Aggression, XTE Damping)
  plus inner loop Kp in Setup page, all persisted to SQLite.
  Working page overlay: `PWM / T° / A° / H° / L:m` (L = live lookahead).
  Steer commands throttled to ~10Hz (100ms).

## Key decisions (see DECISIONS.md for full detail)
- #001–#019: See DECISIONS.md
- #020: Auto-steer as P-control in GUI loop with safety auto-disengage
- #021: Heading error feedforward in outer loop (fixes diagonal overshoot)
- #022: Max steer angle cap (15°) and sensor rate reduction (20Hz→10Hz)
- #023: Pure pursuit outer loop (replaces XTE+heading PD controller)
- #024: Fused heading filter (IMU + GPS complementary filter)
- **#025: Waveform-aware inner loop (rate damping, smooth taper, sub-stall pulsing)**

## Phase 4 hardware inventory (as of 15 April 2026)
- **ESP32 #1 (sensor node)**: reads WAS (ADC GPIO 34), BNO055 (I2C GPIO 21/22),
  forwards GPS NMEA via UART2 passthrough (GPIO 16/17). USB A to laptop. FLASHED.
- **ESP32 #2 (controller node)**: drives IBT-2 via PWM (GPIO 25/26) and enable
  lines (GPIO 27/14). USB B to laptop. FLASHED.
- **BNO055**: 3.3V I2C, connects direct to sensor ESP32. **Now actually used** —
  data flows through HeadingFilter into the steering controller.
- **WAS**: 10kΩ potentiometer. Calibrated: centre=1832, left=1617, right=2031.
- **IBT-2**: RPWM/LPWM from ESP32 PWM pins. Motor confirmed working in cab.
- **GPS**: Quectel LC29H DA on ArduSimple board, UART to sensor ESP32 GPIO 16/17.
- **No buck converter**: ESP32s powered via USB from laptop.

## What's blocked
- **RTK**: no base station or NTRIP subscription yet. Running standalone GPS
  (HDOP 0.4 with 42–50 sats — usable for guidance display, not centimetre-accurate)

## Auto-steer field test checklist (SIXTH TEST — waveform-aware inner loop)
**Pre-flight (before engaging auto-steer):**
- [ ] Motor responds to MOTOR TEST preset buttons
- [ ] Motor direction verified: +50 PWM steers RIGHT (toggle motor_invert if not)
- [ ] WAS calibration values loaded (L:1617 C:1832 R:2031)
- [ ] AB line set and loaded
- [ ] BNO055 calibrated — drive figure-eights until SENSORS shows
      `Cal: S2+ G3 A3 M3` and "Fused heading (IMU+GPS)" is GREEN
- [ ] Sliders at defaults: Approach Aggression=7, Online Aggression=5, XTE Damping=0.5

**First engagement:**
- [ ] Drive onto AB line at working speed (~5 km/h)
- [ ] Tap ⊕ AUTO-STEER on working page
- [ ] Green overlay should show: `AUTO-STEER PWM N T:X° A:Y° H:Z° L:M.Mm`
- [ ] **Key test — smooth zero-crossing**: watch PWM as the tractor crosses the
      line (XTE goes through zero). With the old controller, PWM went to zero in
      the deadband. Now it should smoothly taper, possibly with small sub-stall
      pulses. No dead silence → sudden kick pattern.
- [ ] **Key test — converging damping**: approach the line from 1m off. As XTE
      decreases, the correction should visibly ease off (desired angle shrinking
      faster than XTE alone would predict). No overshoot through the line.
- [ ] **Key test — no hunting**: once on the line, T: A: and H: should hover
      near zero with small oscillations. The oscillation amplitude should be
      smaller than with the previous hard-deadband controller.
- [ ] **Key test — sub-stall pulses**: when near the line (small XTE), watch
      for periodic small motor movements (pulses at min_pwm). These should be
      gentle nudges, not the old bang-bang kicks.
- [ ] If motor steers AWAY from line: immediately ⊗ STEER OFF, toggle motor_invert.
- [ ] If oscillation amplitude is too large: increase XTE Damping (Kd) toward 1.0.
- [ ] If approach to line feels sluggish: decrease XTE Damping toward 0.2.

**Safety verification:** (unchanged from previous tests)
- [ ] ⊗ STEER OFF → motor stops immediately
- [ ] Close app → motor stops within 500ms (watchdog)
- [ ] Unplug motor USB → motor stops, auto-steer disengages
- [ ] Stop tractor (speed < 0.5 m/s) → PWM goes to zero
- [ ] WAS amber warning appears briefly during normal driving (not full disengage)

## Next session should
1. Read this file and DECISIONS.md #025 for context
2. **Field test 6** — verify waveform-aware inner loop as described above
3. Tune XTE Damping (kd_xte) from field observations
4. If behaviour is good, update STEERING_TUNING_GUIDE.md to reflect the three-slider
   approach (Approach Aggression, Online Aggression, XTE Damping)
5. Remove or relabel the now-inert angle deadband slider
6. If a specific repeatable failure mode appears, capture it in DECISIONS.md
   before patching — want to understand the structural cause first

## File map (quick reference)
```
pc/src/main.rs              — entry point, thread setup, GPS + motor auto-detect
pc/src/gps/reader.rs         — serial port reader, auto-detect, module config
pc/src/gps/parser.rs         — NMEA parsing (GGA + VTG, epoch-based)
pc/src/gps/finn_parser.rs    — FINN sentence parser ($FINNWAS, $FINNIMU, $FINNHB, $FINNMTR)
pc/src/comms/serial.rs       — Motor ESP32 serial
pc/src/guidance/ab_line.rs   — AB line guidance, cross-track error, auto-pass
pc/src/guidance/steering.rs  — Auto-steer: pure pursuit outer + WAS-feedback inner (NEW)
pc/src/gui/app.rs            — egui app, page split, lightbar, config persistence
pc/src/gui/field_view.rs     — 2D canvas rendering
pc/src/gui/field_projection.rs — lat/lon → local metres → screen pixels
pc/src/coverage/logger.rs    — coverage logger
pc/src/coverage/db.rs        — SQLite database
pc/src/position/tracker.rs   — position history and odometer
pc/src/position/interpolator.rs — dead-reckoning between 1Hz fixes (NOW: accepts override_heading)
pc/src/position/heading_filter.rs — NEW: fuses BNO055 IMU yaw with GPS COG
common/src/types.rs          — GpsFix, ImuData, WasReading, CrossTrackError, etc.
common/src/coords.rs         — haversine, bearing, cross-track distance
common/src/protocol.rs       — FinnMessage enum, nmea_checksum(), format_steer_command()
firmware-sensor-pio/         — ESP32 #1 Arduino/PlatformIO project (ACTIVE)
firmware-motor-pio/          — ESP32 #2 Arduino/PlatformIO project (ACTIVE)
docs/IMPLEMENTATION_PLAN.md  — full phase plan, task tracking, session log
docs/DECISIONS.md            — architectural decision log (#001–#024)
docs/INSTALLATION_GUIDE.md   — hardware wiring, PC setup, ESP32 flashing
docs/STEERING_TUNING_GUIDE.md — auto-steer setup (STALE — describes old Kp/Kh)
docs/ACTIVE_CONTEXT.md       — this file
```

## Important conventions
- Use `codesnip:edit_snippet` for code changes, never rewrite entire files
- GPS receiver is a Quectel LC29H on ArduSimple board, connected via USB serial
- GUI framework is egui 0.29 (eframe), rendering via Painter API
- All coordinate math in `common/src/coords.rs`, types in `common/src/types.rs`
- Implement width defaults to 12.0m, persisted in SQLite config table
- Overlap defaults to 0cm, persisted. Pass spacing = width − overlap.
- Lightbar sensitivity defaults to 20 cm/segment, persisted
- Last-loaded AB line auto-restored on startup via `last_ab_line_id` config key
- ESP32 firmware uses Arduino/PlatformIO (C++), built with `pio run --target upload`
- Auto-steer sign convention: positive XTE = right of line → negative desired
  angle (steer left). Positive heading error = pointed right of line bearing.
- **Pure pursuit lookahead** = `lookahead_base + lookahead_speed_factor × speed`,
  clamped to [2, 15] m. Exposed as inverted 1–10 sliders (higher = more aggressive).
- **Fused heading** from `HeadingFilter` overrides `fix.heading` via the
  interpolator's `override_heading` parameter. IMU trusted if cal_sys ≥ 2;
  GPS COG trusted if speed ≥ 0.8 m/s; complementary filter alpha = 0.98.
- `apply_motor_direction()` is applied AFTER the steering controller, not inside
- WAS timeout is tiered: warn at 2s (amber), disengage at 5s
- GUI framerate capped at ~30fps (FRAME_INTERVAL = 33ms)
- Motor serial writes throttled to ~10Hz (STEER_SEND_INTERVAL = 100ms)
- Motor stall: min_pwm = 100 (EZ-Steer direct-drive). Inner loop angle deadband
  (2°) prevents bang-bang hunting.
- Trail capped at 5,000 points

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
