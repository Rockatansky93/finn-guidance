# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 22 — 24 April 2026 (root cause of 3s freeze identified as bounded-channel
backpressure; drop-on-full fix applied to both serial reader threads)

## What we're working on
**Phase D — Investigating and fixing intermittent 3-second PC-side stalls
during auto-steer.** Phase A, B, and C of Decision #026 are complete and
field-validated. Test 7 (initial, without implement) showed the new
architecture works as designed — auto-steer was responsive and clean,
resolving the sluggishness from test 6.

**The intermittent freeze diagnosis is done.** An audit of `gps/reader.rs`
and `main.rs` identified the root cause: all four channels between the
serial reader threads and the two consumers (GUI + steer thread) are
bounded (GPS: 64, FINN: 128) and both reader threads were using blocking
`send()`. When the GUI had a slow frame, the GUI-facing channel filled,
the reader thread blocked on the send, and — critically — the steer-thread
channel stopped being fed through the same blocked reader thread.
The steer thread starved of fresh fixes; no `$FINNSTEER` was emitted; the
motor ESP32 correctly held its last commanded angle (~5°); the tractor
physically drifted ~1m during the 3s freeze. When the GUI recovered,
the channels drained and everything caught up.

The fix is applied but not yet field-tested. See Phase D status below.

## Current status of the freeze

- **Root cause**: confirmed via code audit — blocking `send()` on bounded
  channels shared across two consumers, where the slow consumer (GUI) could
  backpressure the reader thread and starve the fast consumer (steer thread)
  through the same blocked thread.
- **Fix applied**: `gps/reader.rs` and `comms/serial.rs` both now use
  `try_send` and drop messages when a channel is full. Drop counts are
  logged as WARN every ~5s so stalls remain visible but don't cascade.
  Guidance gets the latest-value semantics it actually wants (a stale 3s
  fix is worse than no fix — better to skip a fix than to chase a backlog).
- **Fix verified**: not yet. Next field test will confirm.
- **Decision #027** to be written up after field test 8 verifies the fix.

## Phase D — Investigation & fix

### Phase D.0 — Audit (DONE this session)
- [x] Audit `$PAIR062` sentence filtering — confirmed GLL, GSA, GSV, RMC are
      disabled at startup. GPS volume is not the root cause. (Minor finding:
      the disable commands are fire-and-forget with no ACK check — worth
      hardening later but not blocking.)
- [x] Audit `main.rs` channel setup — found all four channels bounded, both
      reader threads doing blocking sends → root cause of the freeze.
- [x] Audit `comms/serial.rs` — confirmed motor reader has the same bug
      pattern (blocking sends on bounded finn_tx / finn_tx_steer). Fixed.

### Phase D.1 — Instrumentation (SKILL-UP, STILL WANTED)
Even though the root cause appears resolved, the in-app diagnostic logging
is still valuable for future issues and for moving toward autonomy. Keep
this phase on the roadmap.

- [ ] Add high-resolution timestamped logging to `gps/reader.rs`:
      - Per-read loop iteration timing
      - Bytes read / sentences parsed per iteration
      - Drop counts (partial — 5s rolling warn already added this session,
        but could be complemented with finer-grained event timing)
- [ ] Add latency logging to the GUI thread:
      - Per-frame delta time (wall clock)
      - Log a WARN line if frame delta > 100ms
      - Log the top 3 longest frames each minute
- [ ] Add latency logging to `guidance/steering.rs` / `steer_thread.rs`:
      - Time from GpsFix received → desired angle computed → sent to motor
      - Log WARN if pipeline latency > 200ms
- [ ] Add a 1Hz heartbeat log summarising: GPS fix rate, GUI fps, steer
      command rate, channel depths, dropped counts
- [ ] Route all diagnostic logs to a rotating file in the app directory
      (so they can be grepped after field runs, not read in-cab)
- [ ] Bench-run for 10+ minutes with GPS connected — confirm instrumentation
      doesn't add jitter, confirm baseline values

### Phase D.2 — Reduce GPS sentence volume (RESOLVED)
- [x] Sentence filtering already present and correct. No change needed.
      (Optional future hardening: check ACK on each `$PAIR062` disable.)

### Phase D.3 — Windows USB-serial hardening (STILL WORTH DOING)
- [ ] Identify the USB-serial chipset on the ArduSimple LC29H BA board
- [ ] Device Manager: disable "Allow the computer to turn off this device
      to save power" for the USB-serial port AND the parent USB root hub
- [ ] Check FTDI/CH340/CP2102 driver latency timer setting
- [ ] Document the required Windows settings in INSTALLATION_GUIDE.md

### Phase D.4 — Field test 8 (verification of the fix)
Previously scoped as a diagnostic run. Now a **verification** run: we
expect the freeze to be gone. If it still occurs, logs from D.1 (once
implemented) will tell us where.

- [ ] Deploy current build to the tractor
- [ ] Re-run the test 7 scenario — AB line, implement attached, similar
      field and conditions
- [ ] Engage auto-steer and drive for at least 10 minutes to allow a
      comparable stall opportunity
- [ ] Check the app logs for:
      - Any "GPS fix drops" WARN — if present, a consumer stalled
        momentarily (now survivable, but indicates where to look next)
      - Any "Motor msg drops" WARN — ditto for the motor side
- [ ] If the freeze DOES reproduce despite the fix, escalate to D.1
      instrumentation and field test 9

### Phase D.5 — Write up Decision #027 (after D.4 verifies)
- [ ] Document the root cause, the fix, and the trade-off (drop vs block)
      in DECISIONS.md as #027
- [ ] Update STEERING_TUNING_GUIDE.md to mention drop-warn log interpretation

## What's done (post-#026)

Decision #026 is fully implemented and field-validated for single-axis
behaviour:

- **Motor ESP32 firmware** (Arduino/PlatformIO): rewritten with WAS ADC read,
  inner loop P controller at 50-100Hz with sub-stall pulsing, NVS storage
  for WAS calibration and PID params via `Preferences.h`, `$FINNCFG` parser,
  `$FINNACK` responses, extended `$FINNMTR` (PWM + WAS raw + actual angle +
  enabled flag), and `$FINNSTEER` now accepts desired angle × 100 not PWM.
- **WAS pot rewired** to motor ESP32 (GPIO 34 ADC, GPIO 33 3.3V ref).
- **Sensor ESP32 physically removed** from the cab. BNO055 removed.
- **LC29H BA ArduSimple board** connected directly to laptop USB. 10Hz NMEA
  confirmed. DR calibration verified (CalState → 2 via `$PQTMDRCAL`).
- **PC software refactored**:
    - `gps/reader.rs` connects to LC29H BA directly, `$PAIR050,100` for 10Hz,
      sentence filtering via `$PAIR062`, `$PQTMINS` enabled for DR heading
    - `$PQTMDRCAL` parser added
    - `guidance/steering.rs` inner loop stripped — returns desired angle
    - `comms/serial.rs` updated — `$FINNSTEER` sends angle × 100, extended
      `$FINNMTR` parser, `$FINNCFG` send methods added
    - `position/heading_filter.rs` deleted (BA provides fused heading)
    - BNO055 UI removed, DR calibration status display added
    - `gps/finn_parser.rs` simplified
    - Auto-detect simplified to two-device model (GPS vs motor ESP32)
    - Inner loop config persistence moved to ESP32 NVS
- **Session 22 changes (Decision #027, pending verification)**:
    - `gps/reader.rs`: `send()` → `try_send()` for both GPS channels;
      drop counters with 5s rolling WARN log
    - `comms/serial.rs`: `send()` → `try_send()` for both FINN channels;
      drop counters with 5s rolling WARN log
- **Field test 7** (initial, without implement): auto-steer responsive and
  clean, sluggishness from test 6 resolved. Architecture validated.
- **Field test 7** (with air seeder hitched): intermittent 3s freezes
  observed. Tom has since assessed the implement is not the cause —
  issue was likely present before and missed. Root cause identified this
  session and fix applied.

## Current state of the code (by module)

- **GPS reading** (`gps/reader.rs`): auto-detect, PAIR config for LC29H BA
  10Hz, NMEA parsing (GGA + VTG epoch-based), `$PQTMDRCAL` parsing,
  try_send to GUI + steer thread with drop logging.
- **Position interpolation** (`position/interpolator.rs`): dead-reckoning
  at ~30fps between 10Hz fixes (100ms gaps).
- **Heading filter**: DELETED (replaced by LC29H BA onboard DR fusion).
- **Auto-steer controller** (`guidance/steering.rs`): pure pursuit outer
  loop only. Returns desired angle sent as `$FINNSTEER,<angle×100>`.
- **Steer thread** (`guidance/steer_thread.rs`): dedicated 10Hz fixed loop,
  consumes `gps_rx_steer` and `finn_rx_steer`, computes desired angle,
  writes `$FINNSTEER` via MotorHandle. No longer starved by GUI hiccups
  as of Session 22.
- **Motor ESP32 firmware** (`firmware-motor-pio/`): WAS ADC + inner loop +
  NVS + `$FINNCFG`/`$FINNACK`/extended `$FINNMTR`. Field-validated.
- **Motor serial reader** (`comms/serial.rs`): try_send to GUI + steer
  thread with drop logging (Session 22).
- **Sensor ESP32 firmware**: archived (no longer in use).
- All other features (field view, AB lines, coverage, lightbar, implement
  controls, job management, GUI) unchanged and unaffected.

## Key decisions (see DECISIONS.md for full detail)
- #001–#025: See DECISIONS.md
- **#026: Hardware simplification — LC29H BA direct-connect + inner loop on
  ESP32.** Implemented and field-validated in test 7.
- **#027 (pending write-up)**: Drop-on-full channel sends in both serial
  reader threads to prevent cascading stalls. Fix applied in Session 22;
  write-up follows field verification.

## Hardware inventory (as of 24 April 2026)

### Current:
- **Motor ESP32**: drives IBT-2, reads WAS, runs inner loop at 50-100Hz,
  stores config in NVS. Firmware in `firmware-motor-pio/`.
- **IBT-2**: H-bridge motor driver. Unchanged.
- **WAS**: 10kΩ pot. Calibration stored in ESP32 NVS.
  Calibrated values: centre=1832, left=1617, right=2031.
- **GPS**: Quectel LC29H BA on ArduSimple board. 10Hz, onboard IMU, DR
  fusion. Connects directly to laptop USB.
- **Antenna**: roof-mounted, ~3.6m high (well above implement).
- **Laptop**: Dell Latitude 7390 2-in-1. Powered via 300W inverter from
  tractor.
- **Tractor**: 1980s, no electric solenoids / no EMI from hydraulic control.

### Removed in #026:
- ESP32 #1 (sensor board)
- BNO055 IMU
- LC29H DA module

## What's blocked
- **RTK**: no base station or NTRIP subscription yet
- **Implement-level testing**: resume once #027 fix is field-verified

## Next session should
1. Read this file and DECISIONS.md #026
2. **Priority: field test 8** to verify the #027 fix resolves the 3s
   freeze. Watch the app log for "GPS fix drops" / "Motor msg drops" WARN
   lines — these now replace the freeze symptom and tell us which
   consumer was slow.
3. If verified: write up Decision #027 in DECISIONS.md and close Phase D.
4. If NOT verified: proceed with Phase D.1 instrumentation to get finer
   visibility into where the remaining latency lives.
5. Phase D.3 (Windows USB-serial power hardening) is still worth doing
   regardless — can be done alongside the field test prep.

## File map (quick reference)
```
pc/src/main.rs                 — entry point, thread setup
                                 (channels: gps_tx 64, finn_tx 128, all bounded)
pc/src/gps/reader.rs           — LC29H BA serial reader, try_send + drop warn
pc/src/gps/parser.rs           — NMEA parsing (GGA + VTG)
pc/src/gps/finn_parser.rs      — FINN sentence parser
pc/src/comms/serial.rs         — Motor ESP32 serial, try_send + drop warn
pc/src/guidance/ab_line.rs     — AB line guidance, XTE, auto-pass
pc/src/guidance/steering.rs    — Outer-loop-only pure pursuit
pc/src/guidance/steer_thread.rs — 10Hz fixed-rate steering loop
pc/src/gui/app.rs              — egui app, page split, lightbar, config
pc/src/gui/field_view.rs       — 2D canvas rendering
pc/src/gui/field_projection.rs — lat/lon → local metres → screen pixels
pc/src/coverage/logger.rs      — coverage logger
pc/src/coverage/db.rs          — SQLite database
pc/src/position/tracker.rs     — position history and odometer
pc/src/position/interpolator.rs — dead-reckoning between fixes
common/src/types.rs            — GpsFix, WasReading, CrossTrackError
common/src/coords.rs           — haversine, bearing, cross-track distance
common/src/protocol.rs         — FinnMessage, nmea_checksum, format_*
firmware-motor-pio/            — ESP32 motor controller (Arduino/PlatformIO)
docs/IMPLEMENTATION_PLAN.md    — full phase plan, task tracking
docs/DECISIONS.md              — decision log (#001–#026; #027 pending)
docs/INSTALLATION_GUIDE.md     — hardware wiring, PC setup, ESP32 flashing
docs/STEERING_TUNING_GUIDE.md  — auto-steer setup (needs update post-#026)
docs/ACTIVE_CONTEXT.md         — this file
```

## Important conventions
- Use `codesnip:edit_snippet` for code changes where possible; if it
  misbehaves, fall back to `filesystem:write_file` with a full re-write
  (keep changes surgical).
- GPS receiver is Quectel LC29H BA on ArduSimple board (10Hz, DR fusion)
- GUI framework is egui 0.29 (eframe), rendering via Painter API
- All coordinate math in `common/src/coords.rs`, types in
  `common/src/types.rs`
- Implement width defaults to 12.0m, persisted in SQLite config table
- Overlap defaults to 0cm, persisted. Pass spacing = width − overlap.
- Lightbar sensitivity defaults to 20 cm/segment, persisted
- Last-loaded AB line auto-restored on startup via `last_ab_line_id`
- ESP32 firmware uses Arduino/PlatformIO (C++), built with
  `pio run --target upload`
- Auto-steer sign convention: positive XTE = right of line → negative
  desired angle (steer left). Positive heading error = pointed right of
  line bearing.
- **Pure pursuit lookahead** = `lookahead_base + lookahead_speed_factor ×
  speed`, clamped to [2, 15] m. Exposed as inverted 1–10 sliders.
- **Inner loop runs on motor ESP32** at 50-100Hz. PC sends desired angle
  via `$FINNSTEER,<angle×100>`.
- `apply_motor_direction()` is applied on the ESP32
- WAS calibration stored in ESP32 NVS
- WAS timeout handled by ESP32 locally
- GPS fix timeout handled PC-side: desired_angle=0 on timeout → ESP32
  centres wheels
- GUI framerate capped at ~30fps (FRAME_INTERVAL = 33ms)
- Motor steer commands sent at ~10Hz from PC
- Trail capped at 5,000 points
- **Channel discipline (Session 22 / #027)**: both serial reader threads
  (GPS, motor) use `try_send` with drop-on-full and periodic WARN logs of
  drop counts. Guidance prefers latest-value semantics over backlog.
  Never re-introduce blocking `send()` on reader → consumer channels
  without explicit consideration of the shared-thread starvation risk.
