# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 30 — 30 May 2026 (AB-line sign-convention consolidation +
manual roll calibration; Decision #029)

## What we're working on
**Seeding is underway.** Development changes must be stable and not break
what's working.

**Session 30 — AB-line sign cleanup + manual roll calibration (30 May 2026):**

Two pieces of work this session, both aimed at the gap between "functioning"
and "working well" while seeding continues. All changes are additive or
behaviour-preserving on the drive path; the only intentional behaviour change
is the return-pass Snap fix (a bug fix).

### Part 1 — AB-line sign-convention consolidation (`guidance/ab_line.rs`)

The AB-line code re-derived the forward/return direction flip in several
places, which made every sign-related change risky. Consolidated to two
primitives that are now the single source of truth:
- `signed_line_offset(fix)` — direction-independent signed offset from the
  current target track (pass + nudge), in the fixed A→B frame. Zero = on track.
- `travel_sign(fix)` — +1.0 forward (A→B), -1.0 return (B→A).

`calculate_error` now returns `travel_sign × signed_line_offset` — **bit-for-bit
identical output to before**, just factored through the primitives, so the
drive path (steering, lightbar, telemetry) is unchanged.

**Bug fixed:** `align_grid_to_position` (the "⊚ Snap" button) used the
sign-flipped effective XTE on return passes, which *doubled* the error instead
of zeroing it — broken on roughly half of all passes. It now absorbs
`signed_line_offset` into the nudge with no heading logic, so it zeroes the
displayed XTE on both forward and return passes. This is the one intentional
drive-path behaviour change this session.

**Also:**
- New `snap_to_nearest_pass(fix)` — renumbers to the nearest whole pass and
  clears nudge (coarse, pass-level). The Setup-page "⊚ Snap Passes to Here"
  button was *documented* as doing this but was actually wired to
  `align_grid_to_position`; it now calls `snap_to_nearest_pass`. The two Snap
  operations are now genuinely distinct (exact sub-pass nudge vs coarse
  renumber).
- Nudge readout on the working page was inverted relative to the buttons
  (pressing ► showed "L"). Readout now derives a cab-relative value so it
  agrees with the cab-relative buttons.
- `set_point_a` now zeroes nudge (was inconsistent with `load_ab_line`).
- Minimum-line-length / sign-comment fixes: the "positive = right" comments in
  `coords.rs` and `types.rs` were inverted — the code produces **positive =
  LEFT**, matching `steering.rs`. Comments corrected; code unchanged. A lying
  comment at the primitive layer is a real hazard in a sign-sensitive codebase.
- 18 unit tests in `ab_line.rs` (was 0), including the return-pass align case
  and the direction-independence invariants that the consolidation rests on.

### Part 2 — Manual roll calibration (Decision #029)

Roll compensation (#028) assumed the module reads 0° level and that the roll
sign was known. Neither is true across installs. Added a manual calibration so
each tractor captures its own mounting bias and sign:
- **`roll_offset_deg`** (config key `roll_offset_deg`): mounting-bias zero.
  Parser uses `effective_roll = smoothed_roll - roll_offset_deg`. Captured via
  a **◎ Capture Level** button (park level, press) in the Setup-page ROLL
  CORRECTION section.
- **`roll_invert`** (config key `roll_invert`): Normal/Inverted toggle that
  flips the correction sign. Verify on a slope.
- **`roll_corr_m`** (new `GpsFix` field + telemetry `IterRecord` field): the
  signed lateral correction actually applied to each fix (positive = shifted
  left of travel). Now visible in `.jsonl` logs and the GUI instead of being
  silently baked into lat/lon.

With offset 0 and invert false the math reduces exactly to the prior #028
correction — safe to ship before any capture. Two new shared atomics
(`SharedRollOffset` = `Arc<AtomicI32>` centidegrees, `SharedRollInvert` =
`Arc<AtomicBool>`) follow the existing heading-offset/antenna-height pattern.
5 new parser tests cover the offset/invert/threshold arithmetic.

**Field procedure (do once per tractor):**
1. Park on flat, level ground → ROLL CORRECTION → confirm live roll is stable
   → **Capture Level**. Effective roll should drop to ~0°.
2. On a known cross-slope, watch the live correction and XTE. If the correction
   pushes XTE *away* from zero, press **Direction: Inverted** once. `roll_corr_m`
   in the logs confirms the sign afterward. Resolves the sign question for that
   machine permanently.

### Files changed this session
- `common/src/types.rs` — `roll_corr_m` on `GpsFix`; corrected `distance_m`
  sign comment on `CrossTrackError`
- `common/src/coords.rs` — corrected `cross_track_distance_local` sign
  doc/comment (positive = LEFT), renamed a test
- `pc/src/guidance/ab_line.rs` — primitives, align fix, `snap_to_nearest_pass`,
  `set_point_a` nudge reset, 18 tests
- `pc/src/gps/parser.rs` — `roll_offset_deg` + `roll_invert`, effective-roll
  correction, `roll_corr_m`, 5 tests
- `pc/src/gps/reader.rs` — `SharedRollOffset` / `SharedRollInvert` + polling
- `pc/src/main.rs` — create + thread the two roll atomics
- `pc/src/telemetry/logger.rs` — `roll_corr_m` in `IterRecord`
- `pc/src/guidance/steer_thread.rs` — populate `roll_corr_m`; removed dead
  `current_implement_w` / `current_overlap` captures
- `pc/src/gui/app.rs` — Snap-button rewire, nudge-readout fix, lightbar
  comment fix, roll-calibration controls (Capture Level + Direction toggle),
  persistence, effective-roll/applied-correction readouts

### Build + test status
- `cargo build` clean (no errors). 24 warnings, all pre-existing dead-code
  except none from this work; the 4 `steer_thread.rs` unused-variable warnings
  were fixed. `travel_sign` shows an unused warning by design (public primitive
  for the future GUI consolidation).
- `cargo test -p finn-guidance-pc`: all green except the known pre-existing
  `was_centre_learner::tests::learns_drift_over_time` failure (unrelated,
  not touched this session).

### Pending from this session
- [ ] Field: do the Capture Level + slope sign-check on the tractor; confirm
      `roll_corr_m` moves XTE toward zero on a cross-slope.
- [ ] Optional later cleanup: route the GUI nudge buttons/label through
      `travel_sign()` so the primitive is actually consumed (clears its
      unused-code warning and removes the last inline `is_return` flip).

---

**Session 29 — Hot fix during seeding (1 May 2026):**

Tom hit "no GPS fix in GUI" while seeding. Diagnosed and fixed live
from Adelaide via Windows-MCP + Filesystem-MCP on the tractor laptop
while Tom drove. Root cause was a deadlock in the GPS module config
routine: the rate-set ack-wait loop in `ensure_module_config()` had
per-read timeouts but no wall-clock cap, and the LC29H BA streams
~11 NMEA sentences/second continuously (1 Hz GGA + 10 Hz PQTMINS).
Every `port.read()` returned data within the timeout, so the loop's
fallback `_ => break` arm never fired. The loop ran forever, the GPS
reader thread never reached its main NMEA processing loop, no fixes
ever reached the GUI or steer thread.

Symptom in the stdout log was distinctive: the line
`Trying 10Hz (100ms): $PAIR050,100*22` was the last GPS reader output
before the motor reader took over and the GPS thread went silent
forever. Looked like a clean startup followed by a dead GPS — actually
a hung config thread.

### Change made this session

**Change 1: Bound the PAIR050 ack-wait loop with a wall-clock deadline**
`pc/src/gps/reader.rs` `ensure_module_config()` rate-set ack loop now
uses a 500ms `Instant`-based deadline in addition to the per-read
timeout. Comment added explaining the NMEA-flood deadlock so the next
debugger doesn't repeat the diagnosis. Per-read timeout dropped from
300ms to 100ms (no longer needed as the safety net). After the loop
exits, control flows normally through SAVEPAR → main read loop. First
fix arrived 1.2 seconds after launch on the rebuild. Verified by
watching `app_stdout.log` show:
```
No ack for 10Hz (may still be applied)   ← bounded loop exited
Persisting runtime config to NVS
GPS module configuration complete
Got fix: lat=-33.261023 lon=138.596789 sats=42 quality=Gps
```

### Important hardware-firmware finding (worth documenting)

The **LC29H BA NR11A02S firmware caps GGA/VTG output at 1 Hz** regardless
of what `$PAIR050` is set to. The `PAIR001,050,1` response ("unsupported")
is what the module actually returns to `$PAIR050,100`, but our buffered-
ack reader misses that response in the NMEA flood and falls through to
the "may still be applied" log path. Then SAVEPAR persists the
still-1 Hz state to NVS.

**This is fine** — it matches the existing architecture. `gps/parser.rs`
deliberately uses GGA only for position and PQTMINS for heading, velocity,
and attitude. The 10 Hz update rate is delivered via PQTMINS; GGA at 1 Hz
is sufficient for position and the interpolator dead-reckons between
fixes. Coverage points landed at exactly 1.00 Hz in the field DB
(deltas 997-1003ms across 50 consecutive points), confirming the fix
rate.

FT8 Finding 5 ("GPS effective rate is ~1-2Hz, not 10Hz") was therefore
not a bug — it was the actual hardware behaviour. The interpolator was
already handling it correctly. This finding can be marked resolved.

**TODO**: change the misleading log line in `ensure_module_config()`
from "No ack for 10Hz (may still be applied)" to something honest like
"PAIR050 not acknowledged by LC29H BA NR11; 10Hz position not supported
on this firmware — PQTMINS provides 10Hz attitude/velocity, GGA stays
at 1Hz by design". Save the next debugger an hour.

### Field observation — auto-steer working in real seeding

While observing the rebuilt app, watched Tom seed for ~80 minutes of
active auto-steer use. Observations from `app_stdout.log`,
`coverage.db`, and one screenshot:

- **Sky conditions excellent**: 41-45 satellites, HDOP 0.30-0.44 across
  the session. By late afternoon HDOP was 0.3 with 45 sats.
- **Field speed**: averaged 8.7 km/h on auto-steer engages — considerably
  faster than FT8/FT9 (3.3-4.5 km/h). The Session 28 tune
  (`max_steer_angle = 1.0`) is correctly tracking the line at this speed.
- **Engage durations growing**: engages of 50s → 146s → 246s → … →
  408s. The 6m48s engage at 04:23 covered the full length of a pass
  with no safety disengages, no warnings, no channel drops. Tom
  manually steered the headland turns; auto-steer held the straight
  passes.
- **9.39m pass spacing** (9.64m implement, 0.25m overlap). 8.55 ha
  covered by end of session. Pass count reached 31.
- **Active AB line**: "Line1" at -33.25919, 138.59045 → -33.25918,
  138.59715. 623m run, bearing 89.9° (east-west).
- **Snap function used in earnest**: GUI showed toast
  `Line snapped -105cm (nudge now 278cm)`. Validates Session 28 Change
  1 (nudge cap raised from ±2m to ±implement_width). A 278cm nudge
  would have been impossible under the old ±2m cap.
- **Process health**: 19% avg CPU, 105 MB RAM, 11 threads, ~25 min
  uptime when checked. Battery 100% on inverter.
- **Telemetry was off** (`telemetry_enabled = false` in SQLite config).
  This made fix-rate verification require the coverage DB rather than
  JSONL traces. Worth turning on for future sessions.

### What I observed about the architecture in operation

For anyone reading this who wasn't there: the code architecture from
Decisions #026 (LC29H BA direct, ESP32 inner loop) and #027 (try_send
with drop-on-full) holds up well in active use. Specifically:

- Zero channel drops across the entire 80-minute session. The #027 fix
  is doing its job invisibly.
- The 1 Hz GGA / 10 Hz PQTMINS split is invisible to the operator. The
  interpolator dead-reckons between GGA fixes using PQTMINS
  velocity/heading at 30 fps and the GUI shows smooth motion.
- The GUI runs at the capped 30 fps without choking the reader threads
  thanks to the bounded channels + try_send. No backpressure freezes
  observed (none expected post-#027, but worth noting).
- The Session 28 lightbar three-dot style was visible saturating at
  the right end during the screenshot when XTE was -283 cm — the
  segments correctly dragged out of range with the readout, no glitches.

### Files changed this session
- `pc/src/gps/reader.rs` — bounded ack-wait loop (the hot fix)

### Build + deploy
- Killed the running app (PID 6480)
- `cargo build --release -p finn-guidance-pc` — 9.5s, 22 warnings
  (pre-existing), no errors
- Relaunched via `Start-Process -RedirectStandardOutput`. Verified PID
  908 came up clean and started receiving fixes within 1.2 seconds.

### Pending
- [ ] Tidy the misleading "may still be applied" log line (TODO above)
- [x] Mark FT8 Finding 5 as resolved ("by-design hardware behaviour,
      not a bug") — resolved; captured in Decision #030
- [ ] Consider whether to remove the PAIR050 send entirely on LC29H BA
      NR11 — it's a no-op that costs ~700ms of startup time
- [x] Decision write-up for the ack-wait deadlock — done as **#030**
      (Session 30). NOTE: originally slated as "#028" here, but #028 was
      already taken by the DR-remediation entry, so it became #030.

---

**Session 28 — Usability improvements (28 April 2026):**

Auto-steer field test was successful — holding a line within ~30cm,
performing comparably to the Trimble EZ-Steer system it replaces.
Motor direction, WAS calibration, and pure pursuit gain issues from
Session 27 have been resolved in the field. Focus shifted to usability
improvements for in-cab operation during seeding.

### Changes made this session

**Change 1: Nudge cap raised from ±2m to ±implement_width_m**
The 2m hard cap on nudge distance was too restrictive for using the Snap
function when more than 2m off the line. All three clamp points in
`guidance/ab_line.rs` (`nudge_left`, `nudge_right`,
`align_grid_to_position`) now use `implement_width_m` as the cap (12m
default). This allows full pass-width shifts via nudge or Snap.

**Change 2: Lightbar changed to three-dot sliding style**
Replaced the "segments light from centre outward" lightbar with a
three-dot style matching commercial guidance systems. Three adjacent
segments represent the AB line position and slide across the bar. A
fixed dim white centre tick marks the tractor position. Steer towards
the dots. Colour ramp (green→yellow→red) still indicates distance from
centre. Same 31-segment layout, same sensitivity setting.

**Change 3: Status bar shows hectares instead of point count**
The `● REC` indicator in the GPS status bar now shows `● REC X.XX ha`
instead of `● REC <point_count>`. More useful at a glance during seeding.

**Change 4: Working page buttons doubled in size**
All buttons on the working page bottom bar doubled for touchscreen use
in a bouncing tractor cab:
- Row 1 (Set A/B, nudge, snap): buttons 120×60 / 64×60 / 130×60,
  text 24pt
- Row 2 (engage, auto-steer, auto-pass, setup): buttons 220×70 /
  130×70 / 140×70, text 24-28pt
- Pass indicator text: 28pt
Setup page buttons unchanged (not used while moving).

### Field status update from Tom

- Auto-steer is working and holding within ~30cm, matching Trimble
- Motor direction / WAS calibration / pure pursuit gain issues from
  FT9 findings have been resolved
- The triangle icon inversion bug on E/W headings is **cosmetic only**
  — confirmed not connected to steering calculations
- Remaining work is usability-focused, not algorithmic

**Previous session context (Session 27, for reference):**
- Snap button reworked ("Snap Passes to Here" / "⊚ Snap")
- Max steer angle slider range changed to 1–30° with 0.5° steps
- Session 25 cab observations (see git history for full list)

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
- **Fix verified**: YES — field tests 8 and 9 both show zero drops.
- **Decision #027** to be written up (fix is verified).

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

### Phase D.1 — Telemetry Logging (DONE — Session 23)
Full telemetry logging implemented and field-tested. New `pc/src/telemetry/`
module writes newline-delimited JSON (`.jsonl`) files to `logs/` directory.

- [x] `TelemetryLogger` creates a new log file per auto-steer engage,
      named `steer_YYYYMMDD_HHMMSS.jsonl`
- [x] **Header record**: tuning snapshot at engage time (lookahead, wheelbase,
      max_steer_angle, kd_xte, deadband, implement_width, overlap)
- [x] **10Hz iteration records**: full control loop state per tick —
      timestamp, loop duration, fix age, lat/lon/speed/heading, fix quality/
      sats/hdop, pass number, XTE, heading error, desired angle, actual angle,
      PWM, lookahead distance
- [x] **1Hz summary records**: mean/max XTE, mean angle error, PWM sign
      change count (oscillation indicator), loop timing stats, per-channel
      drop counts (gps_gui, gps_steer, mtr_gui, mtr_steer)
- [x] **Event records**: engage, disengage (manual + safety), pass changes
- [x] **Shared atomic drop counters** (`SharedDropCounters` in
      `telemetry/mod.rs`): `Arc<AtomicU64>` counters incremented by reader
      threads via `fetch_add`, read-and-reset by steer thread via `swap_all()`
      each second for telemetry summaries. Local counters preserved for
      tracing WARN output. Wired through `main.rs` to GPS reader, motor
      reader, and steer thread.
- [x] `log_iteration()` returns `bool` indicating whether a 1Hz summary
      was emitted, so the steer thread resets its drop accumulator cleanly
- [x] BufWriter (8KB) with 1Hz flush — no per-iteration filesystem hit
- [x] No new dependencies (serde_json already present, atomics from stdlib)
- [x] Two field test logs captured and analysed (see findings below)

### Phase D.2 — Reduce GPS sentence volume (RESOLVED)
- [x] Sentence filtering already present and correct. No change needed.
      (Optional future hardening: check ACK on each `$PAIR062` disable.)

### Phase D.3 — Windows USB-serial hardening (STILL WORTH DOING)
- [ ] Identify the USB-serial chipset on the ArduSimple LC29H BA board
- [ ] Device Manager: disable "Allow the computer to turn off this device
      to save power" for the USB-serial port AND the parent USB root hub
- [ ] Check FTDI/CH340/CP2102 driver latency timer setting
- [ ] Document the required Windows settings in INSTALLATION_GUIDE.md

### Phase D.4 — Field test 8 (DONE — 25 April 2026)
Field test 8 ran with telemetry logging active. Two runs captured:

**Run 1: `steer_20260425_123509.jsonl`** — Pass 0, 3.8 min, 3.3 km/h
- Tuning: lookahead_base=5.1, speed_factor=2.7, wheelbase=2.8, max_steer=7.0
- Mean |XTE|=0.49m, max=0.84m, **100% right of line** (never crossed zero)
- Angle error: mean 1.89°, saturation 2%
- 142 PWM sign changes, 36% PWM=0
- Zero channel drops — #027 fix verified

**Run 2: `steer_20260425_124018.jsonl`** — Pass 3, 2.6 min, mixed speed
  (3.3 → 4.5 km/h)
- Tuning: lookahead_base=6.3, speed_factor=3.0, wheelbase=3.2, max_steer=7.0
- Mean |XTE|=0.61m, max=1.32m, **96% right of line**
- Angle error: mean 2.30°, saturation frequent at speed
- 106 PWM sign changes, 31% PWM=0
- Zero channel drops
- One 3.2s loop stall at t=113s (fix_age=3298ms, loop_us=3198425) —
  upstream origin (USB serial or GPS module pause), not channel backpressure

### Phase D.4 — Telemetry Analysis Findings

**Finding 1: Systematic right-side XTE bias (~0.5m)**
Both runs show persistent positive XTE regardless of tuning or speed.
Run 1 is 100% right, run 2 is 96% right. The mean offset is +0.49m and
+0.60m respectively. This is structural, not tuning-dependent. Likely
causes (in priority order):
  1. GPS antenna mounted off-centre from the implement's reference line
  2. WAS zero calibration has a small offset
  3. AB line set from a position that doesn't match desired track
**Action**: Try nudge of -0.5m to confirm it's an offset. If centred,
measure actual antenna-to-centreline distance and apply as a permanent
antenna offset parameter.

**Finding 2: max_steer_angle too low (7.0°)**
At speeds above ~4 km/h, the controller saturates at the ±7° limit
constantly. The sawtooth pattern is: XTE drifts to 1.0-1.3m → desired
pegs at -7° → slow correction → overshoots near zero → drifts out again.
Cycle is ~25-30s. The Trimble EZ-Steer motor can physically achieve
much more.
**Action**: Increase max_steer_angle to 12-15° for next field test.

**Finding 3: Inner loop angle tracking lag (2-3° mean error)**
The actual steer angle consistently lags the desired by 2-3°. At critical
moments (desired=-7°), actual is typically -4° to -5° — delivering only
60-70% of requested correction. 9% of iterations have >5° angle error.
**Action**: Increase ESP32 Kp_angle from 10 to 15+, or tune sub-stall
pulse accumulation rate to reduce the dead zone.

**Finding 4: PWM dead zone causing idle corrections**
31-36% of iterations have PWM=0, but 7% of those have angle errors >2°.
The motor should be correcting but isn't — the outer loop PWM command
falls below min_pwm so nothing happens. The PWM chart shows bang-bang
behaviour: 0→±100→0 cycling rather than smooth proportional output.
**Action**: Lower min_pwm threshold or add faster sub-stall pulsing.
Consider adding a D term to the inner loop to dampen oscillation.

**Finding 5: GPS effective rate is ~1-2Hz, not 10Hz**
Mean fix age is 534-548ms across both runs. Fix ages routinely exceed
500ms, with regular 1-1.1s gaps. The interpolator compensates but the
control loop is working with staler data than expected.
**Action**: Investigate LC29H BA actual output rate. The DR mode may be
throttling real fix output. Check `$PAIR050` config.

**Finding 6: #027 fix verified — zero drops**
Both runs show zero channel drops across all four counters. The
try_send + drop-on-full fix is working as designed. The one remaining
3.2s stall (t=113s in run 2) originates upstream of the reader thread.

**Finding 7: Speed-dependent performance**
Run 1 (3.3 km/h): max XTE 0.84m, 2% saturation
Run 2 (4.5 km/h): max XTE 1.32m, frequent saturation
The controller works reasonably at walking pace but degrades at field
speed. Raising max_steer_angle is the primary fix.

### Phase D.5 — Write up Decision #027 (DONE — Session 30)
Field test 8 verified the fix — zero drops, no backpressure freezes.
- [x] Document the root cause, the fix, and the trade-off (drop vs block)
      in DECISIONS.md as #027 — done Session 30
- [ ] Update STEERING_TUNING_GUIDE.md to mention drop-warn log interpretation
- [ ] Document telemetry log format and analysis workflow

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
- **Session 22 changes (Decision #027, verified in field test 8)**:
    - `gps/reader.rs`: `send()` → `try_send()` for both GPS channels;
      drop counters with 5s rolling WARN log + shared atomic counters
    - `comms/serial.rs`: `send()` → `try_send()` for both FINN channels;
      drop counters with 5s rolling WARN log + shared atomic counters
- **Session 23 changes (Phase D.1 telemetry)**:
    - New module: `pc/src/telemetry/mod.rs` — `SharedDropCounters` with
      four `Arc<AtomicU64>` fields, `swap_all()` method
    - New module: `pc/src/telemetry/logger.rs` — `TelemetryLogger`,
      `IterRecord`, `SummaryRecord`, `EventRecord`, `HeaderRecord`,
      `TuningSnapshot`, `DropCounts`, `fix_quality_to_u8()`
    - `main.rs`: added `mod telemetry`, creates `SharedDropCounters`,
      clones to GPS reader, motor reader, and steer thread
    - `gps/reader.rs`: accepts `SharedDropCounters`, increments atomics
      on drops alongside local tracing counters
    - `comms/serial.rs`: accepts `SharedDropCounters`, same pattern
    - `guidance/steer_thread.rs`: accepts `SharedDropCounters`, creates/
      drops `TelemetryLogger` on engage/disengage, logs every iteration
      when engaged, swaps drop counters each second, logs pass changes
      as events
- **Session 24 changes (Heading offset calibration)**:
    - `common/src/types.rs`: added `diag_vtg_heading` and `diag_ins_heading`
      fields to `GpsFix` for diagnostic heading source comparison
    - `gps/parser.rs`: added `heading_offset_deg`, `last_vtg_heading`,
      `last_ins_heading` fields to `NmeaState`; VTG heading always stored
      for diagnostics even when INS is preferred; heading offset applied
      to all emitted fixes via `normalise_heading()` helper; diagnostic
      heading values included in emitted fixes
    - `gps/reader.rs`: added `SharedHeadingOffset` type (`Arc<AtomicI32>`,
      centidegrees); added `heading_offset_deg` to `GpsConfig`; reader
      polls shared atomic each sentence and updates parser's offset;
      `run_gps_reader()` accepts `SharedHeadingOffset` parameter
    - `main.rs`: creates `SharedHeadingOffset`, passes to GPS reader and
      GUI app constructor
    - `gui/app.rs`: added `heading_offset_shared` and `heading_offset_deg`
      fields; loads persisted value from SQLite `heading_offset_deg` key;
      pushes to shared atomic on load; new `apply_heading_offset()` method
      persists to SQLite and updates shared atomic; SENSORS section now
      shows VTG/INS/corrected heading comparison with INS−VTG delta;
      new HEADING OFFSET section with ±0.5° coarse and ±0.1° fine buttons
      plus reset, amber when non-zero, ±15° hard cap; status bar shows
      offset indicator when active
- **Field test 7** (initial, without implement): auto-steer responsive and
  clean, sluggishness from test 6 resolved. Architecture validated.
- **Field test 7** (with air seeder hitched): intermittent 3s freezes
  observed. Tom has since assessed the implement is not the cause —
  issue was likely present before and missed. Root cause identified in
  Session 22 and fix applied.
- **Field test 8** (25 April 2026, with telemetry): two runs logged.
  #027 fix verified (zero drops). Systematic right-side bias and inner
  loop lag identified from telemetry analysis. See Phase D.4 findings.
- **Field test 9** (28 April 2026, Session 27): three runs. Run 1 motor-
  engaged but aborted after 7s (runaway — motor driving wrong direction).
  Runs 2 & 3: motor disengaged, lightbar-only hand-steering. Proved
  outer loop algorithm is sound. Key learnings: max_steer_angle of 1° is
  correct for line-holding (previous 12-15° recommendation was wrong),
  motor direction or WAS cal needs verification, pure pursuit gain too
  aggressive, triangle icon inverts on E/W headings (possible heading
  sign bug).

## Current state of the code (by module)

- **GPS reading** (`gps/reader.rs`): auto-detect, PAIR config for LC29H BA
  10Hz, NMEA parsing (GGA + VTG epoch-based), `$PQTMDRCAL` parsing,
  try_send to GUI + steer thread with drop logging + shared atomic counters.
- **Position interpolation** (`position/interpolator.rs`): dead-reckoning
  at ~30fps between 10Hz fixes (100ms gaps).
- **Heading filter**: DELETED (replaced by LC29H BA onboard DR fusion).
- **Auto-steer controller** (`guidance/steering.rs`): pure pursuit outer
  loop only. Returns desired angle sent as `$FINNSTEER,<angle×100>`.
- **Steer thread** (`guidance/steer_thread.rs`): dedicated 10Hz fixed loop,
  consumes `gps_rx_steer` and `finn_rx_steer`, computes desired angle,
  writes `$FINNSTEER` via MotorHandle. Creates/drops `TelemetryLogger` on
  engage/disengage. Swaps shared drop counters each second for summaries.
- **Telemetry** (`telemetry/mod.rs`, `telemetry/logger.rs`): NEW in Session
  23. Writes `.jsonl` files to `logs/` with 10Hz iteration records, 1Hz
  summaries, and discrete events. `SharedDropCounters` provides atomic
  counters shared between reader threads and steer thread.
- **Motor ESP32 firmware** (`firmware-motor-pio/`): WAS ADC + inner loop +
  NVS + `$FINNCFG`/`$FINNACK`/extended `$FINNMTR`. Field-validated.
- **Motor serial reader** (`comms/serial.rs`): try_send to GUI + steer
  thread with drop logging + shared atomic counters (Session 22/23).
- **Sensor ESP32 firmware**: archived (no longer in use).
- All other features (field view, AB lines, coverage, lightbar, implement
  controls, job management, GUI) unchanged and unaffected.

## Key decisions (see DECISIONS.md for full detail)
- #001–#025: See DECISIONS.md
- **#026: Hardware simplification** — LC29H BA direct-connect + inner loop on
  ESP32. Implemented and field-validated in test 7.
- **#027: Drop-on-full channel sends** in both serial reader threads to prevent
  cascading stalls (slow GUI consumer starving the steer thread through the
  shared reader thread). Verified zero drops in FT8 + FT9. Written up.
- **#028: LC29H BA DR remediation** — PQTMINS/PQTMIMU enabled with corrected
  NR11 two-wheel field parsing; roll/pitch/heading sourced from PQTMINS,
  position still from GGA. Written up.
- **#029: Manual roll calibration** — per-machine mounting-bias capture
  (`roll_offset_deg`, "Capture Level"), direction invert (`roll_invert`), and
  applied-correction telemetry (`roll_corr_m`). Effective roll = smoothed roll
  − captured bias. No-op until calibrated. Written up.
- **#030: Bound GPS config ack-wait loops with a wall-clock deadline** — a
  per-read timeout never fires against a continuously-streaming device, so the
  config loop hung and starved the GPS thread. Also documents the NR11 1 Hz
  GGA cap as by-design (resolves FT8 Finding 5). Written up.

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
- **Hardware switch inputs**: momentary switch for auto-steer engage and
  seed engage switch for coverage logging both require ESP32 GPIO wiring
  and firmware + protocol additions (see "Next session" items 2 & 3)

## FINN Core integration plan (designed Session 23)

The telemetry `.jsonl` files are designed as the handoff artifact for
FINN Core integration. The 1Hz summary records are compact enough for a
worker node's context window (~600 records for a 10-minute run). The
planned integration path:

1. **Near term**: After field runs, POST `.jsonl` file to FINN hub as a
   task: "analyse steering log, recommend tuning changes." Worker processes
   1Hz summaries, identifies bias/oscillation/saturation patterns, outputs
   recommendations (config changes or code patches).
2. **Code change loop**: If analysis reveals a code change is needed,
   architect reads the analysis + relevant source → writes patch → office-pc
   node compiles (`cargo build`) → pushes to GitHub → tractor laptop pulls.
3. **Long term**: Accumulate diagnosis cycles as training data for
   fine-tuning local models on FINN Guidance debugging. Phase 3.1 of FINN
   Core roadmap (Learning System) already supports this pattern.
4. **Log format considerations**: Include run_id linking to coverage DB
   for cross-referencing field conditions. Header captures tuning snapshot
   so analysis knows what config was active. JSON format is directly
   parseable by worker nodes without regex.

## Next session should
1. Read this file and the recent decisions in DECISIONS.md (#026–#030 are
   the live architecture; #029 roll calibration and the #029 AB-line sign
   convention are the newest).
2. **Field validation owed from Session 30 (do when next in the cab):**
   - Roll calibration: park level → Capture Level; then on a cross-slope
     confirm the live correction / `roll_corr_m` moves XTE toward zero
     (flip Direction if not). See Decision #029.
   - Consider turning telemetry back on by default
     (`telemetry_enabled = true`) so future field-data analysis is
     possible without requiring a manual toggle.
3. **Hardware input: Momentary switch for auto-steer engage/disengage**
   (hardware-dependent, plan when components available)
   - Wire a momentary pushbutton to an ESP32 GPIO input (with pull-up)
   - ESP32 firmware: detect press, send a new FINN sentence e.g.
     `$FINN,STEER_BTN,1*` on press
   - PC side: parse the button event in `comms/serial.rs`, forward to
     steer thread as `SteerCommand::Toggle` (engage if off, disengage
     if on)
   - This replaces the touchscreen AUTO-STEER button for in-cab use —
     physical button is safer and faster than a touchscreen target
4. **Hardware input: Seed engage switch triggers coverage logging**
   (hardware-dependent, plan when components available)
   - Wire the air seeder’s seed-engage switch output (likely 12V) to an
     ESP32 GPIO via a voltage divider or optocoupler
   - ESP32 firmware: monitor the input state, send e.g.
     `$FINN,SEED,1*` / `$FINN,SEED,0*` on state changes
   - PC side: parse seed state in `comms/serial.rs`, auto-engage/
     disengage coverage logging in response — no manual button press
     needed
   - Coverage logging then tracks actual seeding, not just driving
5. **Triangle icon inversion bug** — cosmetic only, low priority.
   Fix when convenient: check `field_view.rs` and `field_projection.rs`
   for heading-dependent sign errors around 90°/270°.
6. **Add startup config push** — when motor port connects, send all
   stored config (WAS, PID, WASF, INVERT) to ESP32 to guarantee sync.
   Still worth doing even though the FT9 runaway is resolved.
7. **Small code TODO carried from Session 29:** tidy the misleading
   "No ack for 10Hz (may still be applied)" log line in `gps/reader.rs`
   to name the NR11 1Hz cap explicitly (see Decision #030). Optionally
   drop the no-op `$PAIR050` send entirely (~700ms startup saving).
8. Phase D.3 (Windows USB-serial power hardening) — still worth doing
9. Optional: route the GUI nudge buttons/label through `ab_line.rs`'s
   `travel_sign()` primitive so it's actually consumed (removes the last
   inline `is_return` flip and clears its unused-code warning).

**Write-ups now COMPLETE (were pending across Sessions 22–29):** Decision
#027 (drop-on-full channels), #028 (DR remediation), #030 (ack-wait
deadlock). FT8 Finding 5 resolved as by-design. The decision log is now
gap-free #001–#030.

**IMPORTANT CORRECTION for future sessions**: The field test 8 finding
that recommended increasing max_steer_angle to 12-15° was wrong.
For broadacre line-holding at field speeds, real steering corrections
are <0.5°. A max_steer_angle of 1-2° is appropriate. Do not raise it
above ~3° for line-holding. Higher values may be needed only for initial
line acquisition from large offsets.

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
pc/src/guidance/steer_thread.rs — 10Hz fixed-rate steering loop + telemetry
pc/src/telemetry/mod.rs        — SharedDropCounters, module root
pc/src/telemetry/logger.rs     — TelemetryLogger, record types, accumulator
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
docs/DECISIONS.md              — decision log (#001–#030, complete)
docs/INSTALLATION_GUIDE.md     — hardware wiring, PC setup, ESP32 flashing
docs/STEERING_TUNING_GUIDE.md  — auto-steer setup (needs update post-#026)
docs/ACTIVE_CONTEXT.md         — this file
```

## Important conventions
- Code edits are made via the Filesystem MCP (`edit_file` for surgical
  changes, `write_file` for full rewrites). Keep changes surgical and
  prefer `edit_file` with exact-match `oldText`. (Earlier docs referenced
  `codesnip:edit_snippet`; that tool is no longer the path.)
- GPS receiver is Quectel LC29H BA on ArduSimple board (1Hz GGA position,
  10Hz PQTMINS attitude/velocity/heading — see #030)
- GUI framework is egui 0.29 (eframe), rendering via Painter API
- All coordinate math in `common/src/coords.rs`, types in
  `common/src/types.rs`
- Implement width defaults to 12.0m, persisted in SQLite config table
- Overlap defaults to 0cm, persisted. Pass spacing = width − overlap.
- Lightbar sensitivity defaults to 20 cm/segment, persisted
- Last-loaded AB line auto-restored on startup via `last_ab_line_id`
- ESP32 firmware uses Arduino/PlatformIO (C++), built with
  `pio run --target upload`
- **Sign convention (corrected Session 30 — see `ab_line.rs` module docs):**
  `cross_track_distance_local` and `CrossTrackError::distance_m` are
  **positive = vehicle LEFT of line, negative = RIGHT**, matching
  `steering.rs`. (Older comments in `coords.rs`/`types.rs` said "positive =
  right" — those were wrong and have been corrected; the code was always
  positive = left.) The controller consumes `distance_m` as
  `travel_sign × signed_line_offset`; positive heading error = pointed right
  of the line bearing.
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
