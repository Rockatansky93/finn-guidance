# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 21 — 21 April 2026 (Decision #026: hardware simplification + inner loop on ESP32)

## What we're working on
**Decision #026 — Hardware simplification: LC29H BA direct-connect + inner loop on ESP32.**

Major architecture change driven by two findings:
1. Field test 6 showed sluggish steering — root cause is the 10Hz PC-side inner loop
   (150–200ms round-trip latency per correction cycle).
2. LC29H BA GPS modules arrived — 10Hz fix rate with onboard IMU fusion, replacing
   both the 1Hz DA module and the problematic BNO055 IMU.

The change eliminates the sensor ESP32 entirely, connects the LC29H BA directly to
the laptop, moves the WAS pot and inner loop to the motor ESP32 (running at 50–100Hz),
and simplifies the PC software to outer-loop-only steering.

**Current state: PLANNING COMPLETE, IMPLEMENTATION NOT STARTED.**

## Architecture — before and after

### Before (current):
```
Laptop USB-A → Sensor ESP32 → GPS (UART passthrough) + WAS (ADC) + BNO055 (I2C)
Laptop USB-B → Motor ESP32  → IBT-2 H-bridge
Inner loop: PC-side at 10Hz (steering.rs)
GPS: LC29H DA at 1Hz, heading via BNO055 complementary filter
```

### After (Decision #026):
```
Laptop USB   → ArduSimple LC29H BA (direct serial, 10Hz GPS + onboard IMU fusion)
Laptop USB   → Motor ESP32 → IBT-2 + WAS (pot moved here) + inner loop at 50-100Hz
Inner loop: ESP32-side at 50-100Hz
GPS: LC29H BA at 10Hz, heading via onboard DR fusion
```

## Implementation plan (from Decision #026)

### Phase A — Hardware + firmware (do first):
- [ ] Wire WAS pot to motor ESP32 (move signal from sensor ESP32 GPIO 34)
- [ ] Write and flash new motor ESP32 firmware:
  - ADC reading for WAS
  - Inner loop P controller with sub-stall pulsing
  - NVS storage for WAS calibration + PID params
  - `$FINNCFG` parser (WAS cal, PID params, motor invert)
  - `$FINNACK` responses
  - Extended `$FINNMTR` (PWM, WAS raw, actual angle, enabled)
  - `$FINNSTEER` now receives desired angle × 100 (not PWM)
- [ ] Connect LC29H BA ArduSimple board directly to laptop USB
- [ ] Verify 10Hz NMEA output (GGA + VTG)
- [ ] Verify DR calibration process ($PQTMDRCAL CalState → 2)
- [ ] Remove sensor ESP32, BNO055, and old wiring from cab

### Phase B — PC software refactor:
- [ ] Update `gps/reader.rs` — direct LC29H BA connection, `$PAIR050,100` for 10Hz
- [ ] Add `$PQTMDRCAL` parser for DR calibration status
- [ ] Strip inner loop from `guidance/steering.rs` — return desired angle (f64)
- [ ] Update `comms/serial.rs` — new `$FINNSTEER` (angle), extended `$FINNMTR`
- [ ] Add `$FINNCFG` send methods for WAS cal and PID params
- [ ] Delete `position/heading_filter.rs`
- [ ] Remove BNO055 UI, add DR calibration status display
- [ ] Simplify `gps/finn_parser.rs` — remove $FINNWAS, $FINNIMU, $FINNHB
- [ ] Simplify auto-detect to two-device model (GPS vs motor ESP32)
- [ ] Update config persistence (inner loop params → ESP32, not local SQLite)

### Phase C — Field test 7:
- [ ] Verify GPS at 10Hz — position, heading, DR status
- [ ] Verify WAS on motor ESP32 (compare ADC to known calibration)
- [ ] Verify inner loop: MOTOR TEST with fixed desired angles
- [ ] Full auto-steer: engage on AB line, assess responsiveness
- [ ] Key metric: does 100Hz inner loop eliminate the sluggishness from test 6?
- [ ] Tune inner loop PID if needed (Setup sliders → $FINNCFG → ESP32)

## Current state of the code
All features below are IMPLEMENTED unless marked otherwise.
Changes in Session 21 are marked **S21**. Previous session changes marked **S20**.

- **GPS reading**: auto-detect COM port, PAIR module config (1Hz LC29H DA), NMEA
  parsing (GGA + VTG, epoch-based), crossbeam channel to GUI thread.
  **S21:** TO BE REFACTORED — will connect to LC29H BA directly at 10Hz.
- **Position interpolation**: dead-reckoning between 1Hz fixes at ~30fps.
  **S21:** Will become less critical at 10Hz (100ms gaps vs 1000ms).
- **Heading filter**: BNO055 + GPS complementary filter.
  **S21:** TO BE DELETED — replaced by LC29H BA onboard fusion.
- **Auto-steer controller**: pure pursuit outer + WAS-feedback inner loop.
  **S21:** Inner loop TO BE MOVED to motor ESP32 firmware. PC becomes outer-loop only.
- **Motor ESP32 firmware**: receives $FINNSTEER PWM, drives IBT-2.
  **S21:** TO BE REWRITTEN — gains WAS ADC, inner loop, NVS, $FINNCFG.
- **Sensor ESP32 firmware**: reads WAS, BNO055, forwards GPS.
  **S21:** TO BE ARCHIVED — entire board removed.
- All other features (field view, AB lines, coverage, lightbar, implement controls,
  job management, GUI) are UNCHANGED and unaffected by #026.

## Key decisions (see DECISIONS.md for full detail)
- #001–#025: See DECISIONS.md
- **#026: Hardware simplification — LC29H BA direct-connect + inner loop on ESP32**

## Hardware inventory (as of 21 April 2026)

### Current (to be modified):
- **ESP32 #1 (sensor)**: reads WAS, BNO055, forwards GPS. **TO BE REMOVED.**
- **ESP32 #2 (motor)**: drives IBT-2. **TO BE UPGRADED with WAS + inner loop.**
- **BNO055**: 3.3V I2C IMU. **TO BE REMOVED.**
- **WAS**: 10kΩ pot. Calibrated: centre=1832, left=1617, right=2031.
  **Wire to be moved to motor ESP32.**
- **IBT-2**: H-bridge motor driver. Unchanged.
- **GPS (DA)**: Quectel LC29H DA on ArduSimple board. **TO BE REPLACED by BA.**

### New hardware:
- **GPS (BA)**: Quectel LC29H BA on ArduSimple board. 10Hz, onboard IMU, DR fusion.
  Connects directly to laptop USB. DR calibration: drive >3 m/s with turns, ~3 min.

### Target pinout — Motor ESP32 (after #026):

| GPIO | Function           | Direction | Notes                                          |
|------|--------------------|-----------|-------------------------------------------------|
| 25   | IBT-2 RPWM         | Output    | PWM channel A (steer right). 20kHz              |
| 26   | IBT-2 LPWM         | Output    | PWM channel B (steer left). 20kHz               |
| 27   | IBT-2 R_EN         | Output    | Right enable. HIGH to enable                     |
| 14   | IBT-2 L_EN         | Output    | Left enable. HIGH to enable                      |
| 34   | WAS pot wiper (ADC)| Input     | ADC1_CH6. Input-only, 0–3.3V from pot (NEW)     |
| 33   | WAS pot VCC (3.3V) | Output    | Set HIGH at boot. Powers pot high side (NEW)     |
| GND  | Common ground      | —         | Shared with IBT-2, pot low side                  |

### Target power distribution:

```
Laptop USB ──► ArduSimple LC29H BA board (USB-powered, 10Hz GPS + DR)
Laptop USB ──► Motor ESP32
                 ├── VIN (5V) → IBT-2 logic VCC
                 └── 3.3V (GPIO 33) → WAS pot reference
12V Battery ──► IBT-2 motor supply (direct, high current)
```

## What's blocked
- **RTK**: no base station or NTRIP subscription yet

## Next session should
1. Read this file and DECISIONS.md #026
2. **Start Phase A** — write the new motor ESP32 firmware:
   - WAS ADC reading (port code from sensor ESP32)
   - Inner loop P controller at 50-100Hz with sub-stall pulsing
   - NVS storage for calibration + PID params (Arduino `Preferences.h`)
   - $FINNCFG parser and $FINNACK response
   - Extended $FINNMTR status sentence
   - $FINNSTEER now receives desired angle × 100
3. Bench-test firmware before wiring in cab
4. If firmware works on bench, proceed to wiring change and LC29H BA connection
5. Then tackle Phase B (PC refactor) and Phase C (field test 7)

## File map (quick reference)
```
pc/src/main.rs                — entry point, thread setup, GPS + motor auto-detect
pc/src/gps/reader.rs           — serial port reader, auto-detect, module config
pc/src/gps/parser.rs           — NMEA parsing (GGA + VTG, epoch-based)
pc/src/gps/finn_parser.rs      — FINN sentence parser (TO BE SIMPLIFIED)
pc/src/comms/serial.rs         — Motor ESP32 serial (TO BE UPDATED)
pc/src/guidance/ab_line.rs     — AB line guidance, cross-track error, auto-pass
pc/src/guidance/steering.rs    — Auto-steer (TO BE SIMPLIFIED: outer loop only)
pc/src/gui/app.rs              — egui app, page split, lightbar, config persistence
pc/src/gui/field_view.rs       — 2D canvas rendering
pc/src/gui/field_projection.rs — lat/lon → local metres → screen pixels
pc/src/coverage/logger.rs      — coverage logger
pc/src/coverage/db.rs          — SQLite database
pc/src/position/tracker.rs     — position history and odometer
pc/src/position/interpolator.rs — dead-reckoning between fixes (LESS CRITICAL at 10Hz)
pc/src/position/heading_filter.rs — IMU+GPS fusion (TO BE DELETED)
common/src/types.rs            — GpsFix, ImuData, WasReading, CrossTrackError, etc.
common/src/coords.rs           — haversine, bearing, cross-track distance
common/src/protocol.rs         — FinnMessage enum, nmea_checksum(), format_steer_command()
firmware-sensor-pio/           — ESP32 #1 PlatformIO project (TO BE ARCHIVED)
firmware-motor-pio/            — ESP32 #2 PlatformIO project (TO BE REWRITTEN)
docs/IMPLEMENTATION_PLAN.md    — full phase plan, task tracking, session log
docs/DECISIONS.md              — architectural decision log (#001–#026)
docs/INSTALLATION_GUIDE.md     — hardware wiring, PC setup, ESP32 flashing
docs/STEERING_TUNING_GUIDE.md  — auto-steer setup (STALE — needs update after #026)
docs/ACTIVE_CONTEXT.md         — this file
```

## Important conventions
- Use `codesnip:edit_snippet` for code changes, never rewrite entire files
- GPS receiver is a Quectel LC29H BA on ArduSimple board (CHANGED from DA)
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
- **Inner loop runs on motor ESP32** at 50-100Hz (CHANGED from PC-side 10Hz).
  PC sends desired angle via `$FINNSTEER,<angle×100>`.
- `apply_motor_direction()` is applied on the ESP32 (CHANGED from PC-side)
- WAS calibration stored in ESP32 NVS (CHANGED from PC SQLite)
- WAS timeout: handled by ESP32 locally (ADC read is synchronous, no timeout needed)
- GPS fix timeout: PC-side, sends desired_angle=0 on timeout → ESP32 centres wheels
- GUI framerate capped at ~30fps (FRAME_INTERVAL = 33ms)
- Motor steer commands sent at ~10Hz from PC (desired angle, not PWM)
- Trail capped at 5,000 points
