# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 11 — 9 April 2026 (Phase 4 firmware scaffolding)

## What we're working on
**Phase 4 — ESP32 firmware.** Two firmware crates created:
- `firmware-sensor/` — ESP32 #1: WAS pot ADC (GPIO 34), BNO055 I2C (21/22),
  GPS NMEA passthrough via UART2 (16/17), GPIO 33 HIGH as pot 3.3V reference
- `firmware-motor/` — ESP32 #2: IBT-2 PWM (GPIO 25/26), enable lines (27/14),
  500ms watchdog, steer command parsing

Serial protocol is text-based NMEA-style (not UDP — both ESP32s on USB serial):
- Sensor → PC: `$FINNWAS`, `$FINNIMU`, `$FINNHB`, plus raw GPS passthrough
- PC → Motor: `$FINNSTEER,<pwm>*XX`
- Motor → PC: `$FINNMTR,<pwm>,<enabled>,<uptime>*XX`

**Not yet done:**
- ESP Rust toolchain not yet installed (`espup install`)
- PC-side parser for FINN sentences not yet written (currently only parses NMEA)
- WAS calibration routine not yet implemented
- PID controller (Phase 5) — depends on sensor data flowing first

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
- **Coverage**: engage/disengage, distance-based CSV logging (1m), in-memory points
  for quad-strip rendering (colour-coded by fix quality), viewport culling,
  zoom-dependent render thinning (step 1–4), 100k memory cap with downsample,
  clear button for task transitions
- **Job management**: JOB HISTORY list in Setup page (last 10, with delete)
- **GUI**: working page (full-screen + overlaid lightbar/XTE/notifications) and
  setup page (scrollable panel with AB LINE, IMPLEMENT, NUDGE, GUIDANCE, COVERAGE,
  POSITION, VIEW, LIGHTBAR sections). GPS status bar shared across both pages.
- **Configuration persistence**: implement width, overlap, lightbar sensitivity,
  last AB line ID — all via SQLite config table, saved on change, loaded on startup

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

## Phase 4 hardware inventory (as of 9 April 2026)
- **ESP32 #1 (sensor node)**: reads WAS (ADC GPIO 34), BNO055 (I2C GPIO 21/22),
  forwards GPS NMEA via UART2 passthrough (GPIO 16/17). USB A to laptop.
- **ESP32 #2 (controller node)**: drives IBT-2 via PWM (GPIO 25/26) and enable
  lines (GPIO 27/14). USB B to laptop.
- **BNO055**: 3.3V I2C, connects direct to sensor ESP32, no level shifting needed.
- **WAS**: RQH100030 replaced with a 10kΩ potentiometer. Wiper → GPIO 34 direct
  (no voltage divider needed — pot powered from ESP32 3.3V rail, output 0–3.3V).
  Calibration (centre, left lock, right lock ADC counts) to be stored in SQLite
  config table. A "recalibrate WAS" button needed in Setup page.
- **IBT-2**: RPWM/LPWM from ESP32 PWM pins. R_EN/L_EN on GPIO 27/14 for hard stop.
  Logic powered from ESP32 #2 VIN (5V). Motor supply 12V direct from battery.
- **GPS**: Quectel LC29H DA on ArduSimple board, UART to sensor ESP32 GPIO 16/17.
  Powered from ESP32 #1 VIN (5V). Check ArduSimple header voltage (3.3V serial
  header, not 5V) before wiring UART lines.
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

**Trimble encoder notes (future):** If the encoder is quadrature (A/B channels), use two interrupt-capable GPIOs (e.g. 16/17 or 18/19). If it's a simple pulse output, one GPIO with interrupt is enough. Need to identify the encoder type and connector pinout from the Trimble motor datasheet.

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

## Field test checklist (tractor cab — 9 April 2026)
This is the first build-and-run on the Dell 7390 2-in-1. Key things to verify:

**Build / startup:**
- [ ] `cargo build --release` completes clean on the Dell
- [ ] GPS auto-detect finds the LC29H on the correct COM port
- [ ] Module config commands send without errors (check console output)
- [ ] App opens, GPS status bar shows sats/HDOP

**Touch targets:**
- [ ] ENGAGE button easy to hit with fingers while bouncing
- [ ] ⚙ Setup and ◄ Working View buttons work with touch
- [ ] +/- buttons for implement width, overlap, lightbar all usable
- [ ] Save Line / Load dialogs operable without keyboard

**Guidance display:**
- [ ] Lightbar readable in sunlight / tractor cab lighting conditions
- [ ] XTE readout visible from driving position
- [ ] Vehicle triangle moves smoothly (interpolation working)
- [ ] Auto-pass notification visible when triggered

**AB line workflow:**
- [ ] Set A → drive to B → Set B → line appears correct
- [ ] Save line to a field, load it back
- [ ] Auto-pass snaps correctly at headland turns
- [ ] Nudge ±5cm step feels right for inter-row alignment

**Coverage:**
- [ ] Coverage strips painting correctly on field view
- [ ] No slowdown or lag after extended working period

**Notes to record after the test:**
- Any touch targets that are too small or awkward
- Lightbar sensitivity — does 20 cm/seg feel right? Too sensitive / not sensitive enough?
- Any crashes or errors (note which operation triggered them)
- Screen readability — any glare/contrast issues
- Any features missing that would be needed for a real day's work

## Next session should
1. Read this file and DECISIONS.md for context
2. **Install ESP Rust toolchain** — `cargo install espup && espup install`, verify
   with a blink test on one ESP32 before attempting the full firmware
3. **Build and flash firmware-sensor** — test GPS passthrough first (easiest to
   verify), then WAS ADC reads, then BNO055 I2C
4. **Build and flash firmware-motor** — test with manual `$FINNSTEER` commands from
   a serial terminal, verify watchdog stops motor on disconnect
5. **PC-side FINN sentence parser** — extend `pc/src/gps/parser.rs` (or new module)
   to parse `$FINNWAS`, `$FINNIMU`, `$FINNHB`, `$FINNMTR` alongside existing NMEA
6. **WAS calibration** — add calibration routine (centre/left-lock/right-lock) and
   store in SQLite config table, add "Recalibrate WAS" button to Setup page

## File map (quick reference)
```
pc/src/main.rs              — entry point, thread setup, GPS auto-detect config
pc/src/gps/reader.rs         — serial port GPS reader, auto-detect, module config
pc/src/gps/parser.rs         — NMEA parsing (GGA + VTG, epoch-based)
pc/src/guidance/ab_line.rs   — AB line guidance, cross-track error, auto-pass, overlap, nudge
pc/src/gui/app.rs            — egui application, page split, lightbar, config persistence
pc/src/gui/field_view.rs     — 2D canvas rendering (grid, coverage strips, lines, trail, vehicle)
pc/src/gui/field_projection.rs — lat/lon → local metres → screen pixels
pc/src/coverage/logger.rs    — coverage CSV recording, 3-gate filtering, memory cap, clear
pc/src/coverage/db.rs        — SQLite database (jobs, segments, AB lines, fields, config)
pc/src/position/tracker.rs   — position history and odometer
pc/src/position/interpolator.rs — dead-reckoning between 1Hz GPS fixes for smooth GUI
common/src/types.rs          — GpsFix, CrossTrackError, GuidanceLine, FixQuality
common/src/coords.rs         — haversine, bearing, cross-track distance
common/src/protocol.rs       — serial protocol types (FINN sentences, was UDP, now serial)
firmware-sensor/src/main.rs  — ESP32 #1: WAS ADC + BNO055 I2C + GPS NMEA passthrough
firmware-motor/src/main.rs   — ESP32 #2: IBT-2 PWM + steer command parsing + watchdog
docs/IMPLEMENTATION_PLAN.md  — full phase plan, task tracking, session log
docs/DECISIONS.md            — architectural decision log (#001–#014)
docs/INSTALLATION_GUIDE.md   — hardware wiring, PC setup, ESP32 flashing, troubleshooting
docs/ACTIVE_CONTEXT.md       — this file
```

## Important conventions
- Use `codesnip:edit_snippet` for code changes, never rewrite entire files
- GPS receiver is a Quectel LC29H on ArduSimple board, connected via USB serial
- GUI framework is egui 0.29 (eframe), rendering via Painter API
- All coordinate math in `common/src/coords.rs`, types in `common/src/types.rs`
- Coverage CSV format: `segment,timestamp_ms,lat,lon,alt,speed,heading,fix_quality,sats,hdop`
- Implement width defaults to 12.0m in `main.rs`, persisted in SQLite config table
- Overlap defaults to 0cm, persisted. Pass spacing = width − overlap.
- Lightbar sensitivity defaults to 20 cm/segment (3.0m full scale), persisted
- Last-loaded AB line auto-restored on startup via `last_ab_line_id` config key
- egui 0.29 uses `id_salt` (not `id_source`), `show_tooltip_text` takes 4 args
