# FINN Guidance — Decision Log

> **Purpose**: Record architectural and design decisions with rationale so future
> sessions don't revisit settled questions. Append-only — don't edit old entries,
> add a new one if a decision is revised.

---

## #001 — Auto-pass selection is priority over lightbar and persistence
**Date:** 26 March 2026  
**Context:** After the ute road test, reviewed what's needed for the system to be
usable in real fieldwork. Manual pass selection (clicking next/prev) requires the
operator to interact with the laptop at every headland turn, which is impractical.  
**Decision:** Auto-pass selection (detecting headland turns and snapping to the
correct next pass) is the #1 development priority. Lightbar and AB line persistence
are important but secondary — the system is fundamentally unusable in the field
without auto-pass.  
**Alternatives considered:** Lightbar first (rejected: easier to build but doesn't
solve the core usability gap), AB persistence first (rejected: useful but you still
need to manually change passes every turn).

---

## #002 — Pass sequence model with skip factor from day one
**Date:** 26 March 2026  
**Context:** In real farming operations, tractors commonly skip rows to reduce turn
tightness (e.g., work passes 1, 3, 5, 7 then fill in 2, 4, 6, 8). This is the
standard pattern for wide implements. Eventually the system should support auto-turn
at the headland, where the tractor automatically does the U-turn.  
**Decision:** The pass sequence model in `AbLineGuide` will include a configurable
`skip_factor` (default 1 = adjacent rows) from the initial implementation. Auto-pass
selection will snap to the next pass *in sequence*, not just the nearest line.
A `completed_passes` set will track which passes have been worked so the system
knows when to switch from "skipping" to "filling in."  
**Rationale:** Retrofitting a skip pattern onto a simple integer pass counter would
require reworking the auto-pass logic and the coverage display. Building it in from
the start is minimal extra effort and ensures forward-compatibility with auto-turn
(Phase 5+), where the skip factor directly determines turn radius and path generation.  
**Alternatives considered:** Simple increment-by-1 (rejected: would need rework when
auto-turn is added), full path planner from the start (rejected: over-engineering for
the current phase).

---

## #003 — Separate display thinning from CSV recording fidelity
**Date:** 26 March 2026  
**Context:** Coverage logging at 10Hz RTK produces ~36,000 points per hour. The CSV
needs high fidelity for accurate area calculations and auditing, but the canvas can't
efficiently render tens of thousands of filled strips every frame. The session 3 bug
(27x duplication) highlighted how quickly data volume can get out of control.  
**Decision:** Coverage data management will operate at two levels:
1. **CSV recording**: full fidelity after passing the 3-gate filter (epoch dedup,
   time filter, distance filter). This is the permanent record.
2. **In-memory display**: thinned for rendering performance. Recent points (e.g.,
   last 5 minutes) at full resolution, older points downsampled (e.g., 1:4). Memory
   capped at ~100k points with oldest-first eviction.
The canvas rendering layer will also do viewport culling (only draw visible points).  
**Rationale:** Trying to use the same data structure for both recording and display
forces a compromise — either the CSV loses fidelity or the canvas bogs down. Keeping
them separate lets each optimise for its purpose.  
**Alternatives considered:** Single point store for both (rejected: conflicting
requirements), spatial database like R-tree (rejected: over-engineering for current
point volumes, reconsider if needed in Phase 3).

---

## #004 — GUI split into working page and setup page
**Date:** 26 March 2026  
**Context:** The current single-page layout shows everything at once: lat/lon to 7
decimal places, view controls, coverage stats, guidance readout, and all control
buttons. In a tractor cab, the operator needs to glance at the screen and immediately
see cross-track error and whether they're recording. Small buttons are hard to hit on
a bouncing tractor.  
**Decision:** Split the GUI into two pages:
- **Working page**: full-screen field view with oversized lightbar and cross-track
  readout overlaid. Large engage button. HDOP + sat count always visible in a
  compact status strip. Minimal clutter — designed for glancing while driving.
- **Setup page**: AB line management (set/save/load), implement width adjustment,
  coverage job controls, log filter configuration, lat/lon display, view settings.
Both pages share the GPS status strip (fix quality, sats, HDOP) since these are
safety-critical indicators.  
**Alternatives considered:** Collapsible panels (rejected: still too much on screen
when expanded), single page with bigger fonts (rejected: doesn't solve the clutter
problem, just makes clutter bigger).

---

## #005 — Distance-based auto-pass, not heading-based (revises #002)
**Date:** 27 March 2026  
**Context:** Initial auto-pass implementation used heading reversal detection (90°
threshold relative to AB line bearing) to detect headland turns. Field testing
revealed two problems: (1) the heading approach is fragile — driving around an
obstacle, taking a curved path, or any lateral drift that crosses the 90° threshold
triggers a false pass change; (2) it requires separate debounce logic (distance from
last turn point, minimum speed) to avoid mid-turn bouncing, adding complexity.  
**Decision:** Replace heading-based turn detection with continuous distance monitoring.
The system now checks the cross-track error from the current pass line on every GPS
fix. If the error exceeds `snap_threshold × implement_width` (default 0.6 = 60%),
the system snaps to the nearest pass line. No heading analysis, no turn detection,
no debounce position tracking.  
**Rationale:** What the operator actually cares about is "am I on a different line
now?" — that's fundamentally a distance question. The distance approach handles every
scenario naturally: headland U-turns, driving around obstacles, skipping rows,
diagonal approaches, even the operator physically driving the tractor to a different
part of the field. The 0.6 threshold adds hysteresis so it doesn't flicker when
riding right on the boundary between two passes.  
**Impact on #002 (skip factor):** The skip factor concept from Decision #002 is no
longer needed for auto-pass selection — the distance approach is inherently
skip-agnostic. If the operator skips to pass 5, the system will follow as soon as
they cross the 60% threshold toward pass 5. Skip factor remains relevant only for
future auto-turn path generation (Phase 5+).  
**Alternatives considered:** Heading reversal (original approach, rejected: fragile
in real-world driving), hybrid heading+distance (rejected: complexity without
benefit — distance alone handles all cases).

---

## #006 — Overlap via pass_spacing(), not reduced implement width
**Date:** 27 March 2026  
**Context:** Farmers need overlap between passes to ensure no gaps in coverage. The
question is whether overlap should reduce the implement width value or be a separate
parameter that affects only pass line spacing.  
**Decision:** Overlap is a separate `overlap_m` field on `AbLineGuide`. A new
`pass_spacing()` method returns `implement_width_m - overlap_m` (floored at 0.1m).
All pass-related logic (line spacing, pass offset, auto-pass snap threshold,
`find_nearest_pass`, field view rendering) uses `pass_spacing()`. Coverage strip
rendering continues to use the full `implement_width_m` — this is correct because
the strips represent the actual swath the implement covers, and overlapping strips
visually show where double-coverage occurs.  
**UI:** Overlap adjustable in 5cm steps (0 to 90% of width). Displayed in cm. Pass
spacing shown as a computed readout below the controls.  
**Rationale:** Keeping overlap separate from implement width is cleaner conceptually
(the implement doesn't get narrower when you overlap) and avoids confusing the
coverage system. It also means changing overlap doesn't require re-entering the
implement width.  
**Alternatives considered:** Reducing implement_width_m directly (rejected: would
shrink coverage strips incorrectly and conflate two different concepts), having
a separate "effective width" field (rejected: adds a redundant field when a method
on the existing struct achieves the same thing).

---

## #007 — 5Hz GPS for GUI, distance-based logging for CSV, auto-detect COM port
**Date:** 27 March 2026  
**Context:** The LC29H was running at 1Hz, which made the GUI feel sluggish — the
position, lightbar, and cross-track readout only updated once per second. The coverage
logger was set to log every fix, which at higher rates would flood the CSV. The COM
port was hardcoded to COM3, making the system non-portable to other PCs.  
**Decision:** Three changes implemented together:
1. **5Hz GPS output**: the reader sends `$PAIR050,200` on every boot to ensure the
   module outputs at 5Hz (200ms interval). This is an "ensure config" pattern —
   idempotent, safe to send repeatedly, and the module's NVM retains the setting
   across power cycles anyway.
2. **Distance-based coverage logging**: default `LogFilter` changed from every-fix
   to `min_distance_m: 1.0`. The CSV only records when the machine has moved ≥1m,
   so it stays quiet when stationary and logs at a natural density when working
   (~2,800 points/hr at 10km/h). This cleanly decouples the GUI update rate from
   the recording rate.
3. **Auto-detect COM port**: `GpsConfig.port_name` defaults to `"auto"`. On startup,
   `auto_detect_gps_port()` scans all serial ports (USB first), opens each briefly,
   and checks for NMEA sentence prefixes (`$G`, `$PAIR`, `$PQTM`). First match
   wins. Falls back to error if no GPS found.
**Rationale:** 5Hz (not 10Hz) was chosen because the Gemini research doc confirmed
5Hz is the sweet spot for fix stability, and for a tractor at 10-15km/h it provides
more than enough temporal resolution. Distance-based logging is more natural than
time-based for agricultural coverage — it automatically adapts to speed and doesn't
waste space when the machine is idling. Auto-detect removes a deployment friction
point that would bite every time the system moves to a different PC.  
**Alternatives considered:** 10Hz (rejected: marginal benefit for tractor speeds,
and the research doc warns of RTK stability issues at 10Hz), time-based logging
(rejected: still logs when stationary, and logging density varies with speed),
manual COM port config via config file (rejected: adds a config file dependency
when auto-detect is simple and reliable).

---

## #008 — Position interpolation for smooth GUI, not hardware upgrade
**Date:** 27 March 2026  
**Context:** The LC29H DA variant firmware only accepts 1Hz PVT output. All attempts
to set higher rates (10Hz, 5Hz, 2Hz) were rejected with PAIR001 error code 2
("invalid parameter"). The module correctly accepted 1Hz. The EA variant supports
10Hz but upgrading the module is cost-prohibitive at this stage. With 1Hz updates,
the GUI vehicle triangle, lightbar, and XTE readout jump visibly once per second.  
**Decision:** Add a `PositionInterpolator` that dead-reckons between 1Hz fixes using
the last known speed and heading. The interpolator produces a synthetic `GpsFix`
every GUI frame (~60fps) by extrapolating: `position += speed × dt × heading`.
The interpolated position is used for display and guidance; real fixes are used for
coverage logging and auto-pass detection.
**Separation of concerns:**
- **Interpolated (`display_fix`)**: field view vehicle position, guidance error
  (XTE + lightbar), heading-up rotation. Updated every frame.
- **Real (`current_fix`)**: coverage CSV logging (distance filter needs truth),
  auto-pass detection (avoid jitter triggering snaps), Set A/B points, status bar
  (sats, HDOP, fix quality are hardware truth).
- **Trail**: real fixes only (1Hz breadcrumb path is fine).
**Safety bounds:**
- Speed gate: no interpolation below 0.3 m/s (~1 km/h) — heading is unreliable
  at standstill, so the interpolator just holds the last position.
- Time cap: no extrapolation beyond 2 seconds — if GPS drops out, the position
  freezes rather than drifting indefinitely.
**Accuracy:** At tractor speeds (3-4 m/s), one second of dead reckoning accumulates
~10-20cm of error before the next real fix snaps back to truth. Well within visual
tolerance for the GUI.  
**Also in this session:** Lightbar sensitivity changed from 2 cm/segment (30cm full
scale) to 20 cm/segment (3.0m full scale) — the original setting was far too acute
for standalone GPS, causing the lightbar to be permanently maxed out. Will reduce to
5-10 cm/segment when RTK corrections are added.  
**Alternatives considered:** Upgrading to LC29H EA (rejected: cost-prohibitive now,
but remains the long-term plan), increasing baud rate to 460800 (irrelevant: the
limitation is firmware PVT rate not serial bandwidth), accepting 1Hz GUI (rejected:
feels sluggish and unprofessional).

---

## #009 — Nudge as a sub-pass lateral shift, independent of pass number
**Date:** 31 March 2026  
**Context:** Inter-row sowing requires the seeder tines to track exactly between the
previous year's furrows. Even when the pass spacing is correct, the operator needs a
way to fine-tune the lateral position of the entire line system by small amounts
(typically 5–20 cm) without changing the pass number or implement width. Overlap
correction also benefits from a quick sub-pass adjustment.  
**Decision:** Add a `nudge_m: f64` field to `AbLineGuide` (default 0.0, hard cap
±2.0 m). Nudge is applied additively with `pass_offset_m` in `calculate_error()`.
`find_nearest_pass()` subtracts nudge from the raw cross-track distance before
rounding, so auto-pass continues to snap to the correct pass in the nudged system.
Field view pass lines are drawn with nudge added to each pass offset, so all lines
visually shift together. Three public methods: `nudge_right(amount_m)`,
`nudge_left(amount_m)`, `nudge_reset()`.  
**UI:** Setup page — NUDGE section between IMPLEMENT and GUIDANCE. Displays current
offset in cm with direction label (→R / ←L), amber colour when non-zero. Two rows of
buttons: ±5 cm (standard, for inter-row alignment) and ±1 cm (fine, for trim). Reset
button. Working page — amber "Nudge N cm →R/←L" indicator in the bottom bar, only
visible when nudge ≠ 0 (zero-clutter default).  
**Sign convention:** Positive = shift right, consistent with positive pass offset
and the existing XTE sign convention (positive XTE = right of line).  
**Step sizes:** 5 cm standard (matches typical inter-row sowing adjustments of
5–20 cm), 1 cm fine (for precise trim). Hard cap ±200 cm prevents accidental
large shifts.  
**What nudge does NOT affect:** coverage strip positions (strips follow the actual
GPS track, not the guidance lines), pass number, implement width, overlap.  
**Alternatives considered:** Fractional pass offset (rejected: conflates fine trim
with the pass counting system, complicates auto-pass), adjusting overlap to achieve
shift (rejected: overlap changes pass spacing for all passes, nudge affects position
without changing spacing).

---

## #010 — AB line persistence uses field-grouped model, not a flat list
**Date:** 31 March 2026  
**Context:** A farm operation typically sets up to four AB lines per paddock
(e.g. N/S and E/W working directions, plus diagonals). A flat list of saved lines
would quickly grow to dozens of entries with no way to tell which lines belong
together or to which paddock, making it easy to load the wrong line at the start
of a run.  
**Decision:** AB line persistence uses a two-level model: fields (paddocks) contain
lines. A `fields` table was added to the SQLite schema. `ab_lines` has an optional
`field_id` foreign key (NULL = unassigned). Lines are still functional without a
field — they appear in an "Unassigned" section in the load list.
**UI:**
- Setup page AB LINE section gains a "💾 Save Line…" button (greyed out until A+B
  are set). Clicking opens an inline dialog with a name field (auto-filled with a
  timestamp) and a field selector dropdown.
- Below the save button is a "SAVED LINES" subheading with a "+ Field" button.
  The load list uses egui `CollapsingHeader` — each field is a collapsible row
  showing the line count. Lines within each field have "Load" and "🗑" (delete)
  buttons.
- "⬆ Export JSON" and "⬇ Import JSON" buttons write/read `data/finn_ab_lines.json`
  for cross-PC transfer. Import is idempotent (skips duplicates matched by name +
  A-point coordinates within the same field).
**Database location:** `data/coverage.db` relative to the binary — same as before,
just documented explicitly. The file is self-contained and portable; copying it
between PCs is a valid alternative to JSON export/import.
**What load does:** sets the AB line on `AbLineGuide`, resets pass number to 0,
resets nudge to 0. The operator then engages auto-pass as usual.
**Alternatives considered:** Flat list (rejected: unmanageable once a property has
20+ paddocks each with 4 lines), tagging/labelling system (rejected: more complex
UI for no practical benefit over simple field grouping), separate config file for
lines (rejected: redundant with the SQLite database already in place).

---

## #011 — Align Grid to Here snaps pass grid without changing AB geometry
**Date:** 31 March 2026  
**Context:** When changing implements or starting work from a fence line, the pass
grid may not align with the operator's current position. Rather than re-setting A+B
(which would change the line geometry), the operator needs a way to shift only the
pass numbering so that their current GPS position falls exactly on a whole pass line.  
**Decision:** "⊕ Align Grid to Here" button in the AB LINE section of the setup page.
Calculates the nearest whole pass number for the current GPS position, sets
`pass_number` to that value, and resets nudge to zero. The AB line A/B coordinates
are unchanged — only the pass offset shifts. Greyed out until a line is loaded and
a GPS fix is available. Shows confirmation with the new pass number.  
**Rationale:** Preserving line geometry means the alignment of all other passes
(relative to each other and to previous coverage) is maintained. Only the "which pass
am I on?" question is answered differently.  
**Alternatives considered:** Re-set A or B to current position (rejected: changes
line bearing and invalidates all previous pass geometry), manual pass number entry
(rejected: requires mental arithmetic to figure out the right number).

---

## #012 — egui 0.29 API migration fixes
**Date:** 1 April 2026  
**Context:** After Session 9, the codebase failed to compile due to breaking changes
in egui 0.29.1 and cosmetic issues introduced by smart-quoted text (likely from
copy-pasting through a tool that applied typographic curly quotes).  
**Changes:**
1. **Curly quotes in `format!()` macros**: Three instances of `format!("Saved "{}"",
   name)` and `format!("Loaded "{}"", name)` used Unicode curly quotes (`\u{201C}`
   / `\u{201D}`). Rust's `format!` macro interprets the `{` inside `"` as a format
   placeholder, causing parse errors. Fixed by replacing with escaped ASCII quotes:
   `format!("Saved \"{}\"", name)`.
2. **`show_tooltip_text` signature change**: egui 0.29 added a `LayerId` parameter
   as the second argument (4 args total, was 3). Fixed by inserting
   `egui::LayerId::new(egui::Order::Tooltip, egui::Id::new("align_tooltip_layer"))`.
3. **Deprecated `id_source` → `id_salt` renames**: `ComboBox::from_id_source`,
   `ScrollArea::id_source`, and `CollapsingHeader::id_source` were all renamed to
   `from_id_salt` / `id_salt` in egui 0.29. Updated all three call sites.
4. **Unused `PathBuf` import**: removed from `db.rs` — only `Path` was needed.
**Lesson:** When egui is updated, check the changelog for renamed/resignatured APIs
before building. Smart-quote substitution in code is a known hazard when pasting
through rich-text editors or AI tools — always verify string literals compile.

---

## #013 — Persist implement width and overlap, but not nudge
**Date:** 1 April 2026  
**Context:** Restarting the application resets implement width (to the 12m default in
main.rs) and overlap (to 0). These are equipment-specific values that rarely change
during a day's work and are tedious to re-enter. Nudge, on the other hand, is a
per-session fine adjustment that gets cleared by "Align Grid to Here" anyway.  
**Decision:** Persist `implement_width_m` and `overlap_m` to the SQLite config table
(`config` key-value store already in `db.rs`). Values are saved immediately on each
UI button click (no separate "save settings" step). On startup, `GuidanceApp::new()`
reads them back, falling through to the hardcoded defaults if missing or unparseable.
Nudge is deliberately excluded — it resets to zero on every launch, same as on every
"Align Grid to Here" press.  
**Config keys:** `implement_width_m` (stored as `"{:.1}"` e.g. `"12.0"`),
`overlap_m` (stored as `"{:.2}"` e.g. `"0.10"`), `lightbar_cm_per_seg` (stored as
`"{:.0}"` e.g. `"20"`), `last_ab_line_id` (stored as line ID integer, e.g. `"3"`).
Lightbar sensitivity adjustable in a new LIGHTBAR section in the Setup page (1–50
cm/seg, ±1 steps). Last AB line auto-loaded on startup by matching the stored ID
against saved_ab_lines — if the line has been deleted, it's silently skipped.  
**Alternatives considered:** TOML config file (rejected: the SQLite config table
already exists and is simpler — no extra dependency, no file path management),
persisting nudge too (rejected: nudge is session-specific and would cause confusion
if it survived restarts, especially combined with align-grid).

---

## #014 — Coverage data management: render thinning, memory cap, and clear button
**Date:** 1 April 2026  
**Context:** Coverage points accumulate in memory for the duration of a session.
A 10-hour day at 1m distance-based logging produces ~28,000 points (~2MB) — not a
memory problem in itself, but the rendering loop builds a convex polygon per point
per frame at 60fps. When zoomed out viewing the whole field, viewport culling can't
help and all points are drawn. Additionally, coverage data from a completed job
(e.g. seeding) is irrelevant clutter when starting the next task (spraying, harvest).
The critical use case for coverage display is seeing where you stopped after a
breakdown — once the job is done, the data should go.  
**Decision:** Three-part approach:
1. **Zoom-dependent render thinning** (`field_view.rs`): `draw_coverage()` calculates
   a `step` variable based on visible width. At close zoom (≤100m visible), every
   point is drawn (step=1). At wide zoom (≥500m), every 4th point (step=4). Linear
   ramp between thresholds. At wide zoom, individual 1m strips are sub-pixel and
   overlap into a solid mass — skipping is invisible but cuts draw calls by 75%.
2. **Memory cap with oldest-half downsample** (`logger.rs`): After each point push,
   if `coverage_points.len() > max_display_points` (default 100,000), the oldest
   half of the vec is downsampled by keeping every 2nd point. The newer half is kept
   at full resolution. This preserves spatial coverage while halving memory. The CSV
   on disk retains full fidelity. The cap triggers rarely — 100k points is ~35 hours
   of continuous 1m-spaced logging.
3. **Clear Coverage button** (`app.rs`): "🗑 Clear Coverage" button in the Setup page
   COVERAGE section. Greyed out while engaged (must disengage first). Clears all
   in-memory points, resets counters, closes the current CSV file (next engage starts
   a fresh one). CSV files on disk are never deleted. Status message confirms the
   action. Use when switching between tasks on the same field or moving to a new field.
**Rationale:** The render thinning is the highest-impact change — it's the actual
performance bottleneck. The memory cap is a safety net for very long sessions. The
clear button is the user-facing off-ramp that makes coverage data ephemeral per task,
matching the real workflow where coverage is only needed while actively working a job.
**Alternatives considered:** Spatial indexing / R-tree (rejected: over-engineering for
current volumes), time-based recency windows (rejected: adds complexity for minimal
benefit over the simple oldest-half downsample), automatic clear on job end (rejected:
the operator should decide when to clear — a breakdown recovery needs the data to
persist across disengage/re-engage cycles).

---

## #015 — Arduino/PlatformIO for ESP32 firmware, not Rust/esp-idf-hal
**Date:** 10 April 2026  
**Context:** Phase 4 required flashing two ESP32 DevKit modules with firmware for
sensor reading (WAS + BNO055 + GPS passthrough) and motor control (IBT-2 H-bridge).
The initial approach used Rust with `esp-idf-hal` / `esp-idf-svc` crates, targeting
the `xtensa-esp32-espidf` target. This required:
- Espressif's custom Rust toolchain fork (`espup install`)
- A `.cargo/config.toml` with `build-std = ["std", "panic_abort"]` (no prebuilt
  standard library for Xtensa)
- `CARGO_TARGET_DIR` redirection to a short path (ESP-IDF build scripts fail with
  long Windows paths, even with directory junctions)
- Precise version alignment between `esp-idf-sys`, `esp-idf-hal`, `esp-idf-svc`,
  `embuild`, and the ESP-IDF SDK version

Multiple blocking issues were encountered:
1. **Workspace conflict**: `firmware-sensor/` auto-discovered by root workspace.
   Fixed with `workspace.exclude`.
2. **Missing Xtensa target**: needed `rustup override set esp` to use the espup
   toolchain instead of stable.
3. **No prebuilt `core`**: required `build-std` in `.cargo/config.toml`.
4. **Path too long**: ESP-IDF build output exceeded Windows path limits. Required
   `CARGO_TARGET_DIR=/c/espbuild`.
5. **`time_t` size mismatch** (`i64` vs `i32`): `esp-idf-sys 0.34` incompatible
   with the installed toolchain. Required `--cfg espidf_time64` rustflag.
6. **`i8`/`u8` pointer mismatch**: `esp-idf-svc 0.49` TLS bindings generated
   incorrect pointer types against ESP-IDF v5.1.
7. **Version resolution failures**: `esp-idf-svc` on git master required
   `esp-idf-hal 0.46`, conflicting with pinned `0.44`.
8. **465MB clang download**: the `esp-clang` toolchain download repeatedly failed
   due to unstable internet, with no resume support. This was the final straw.

The fundamental problem: `esp-idf-sys` compiles the entire ESP-IDF C framework from
source and generates Rust bindings on top. You pay all the complexity of C *plus* a
thick layer of binding issues. The Rust safety guarantees provide minimal value for
this firmware — the code is simple, single-threaded, and the failure modes are
electrical (wrong pin, wrong voltage) not logical.

**Decision:** Replace the Rust firmware crates with Arduino/PlatformIO (C++) projects.
New directories: `firmware-sensor-pio/` and `firmware-motor-pio/`. The serial
protocol is identical — the PC-side Rust code needs no changes.

**Result:** Both ESP32 modules flashed successfully in under 5 minutes each. The
PlatformIO ESP32 Arduino framework download was ~200MB (vs 465MB+ for Rust ESP-IDF
clang alone) and completed without issues. Build times are seconds, not 10-20 minutes.

**What stayed the same:**
- Pin assignments (GPIO 34 WAS, 33 power, 21/22 I2C, 16/17 GPS, 25/26 PWM, 27/14 EN)
- Serial protocol ($FINNWAS, $FINNIMU, $FINNHB, $FINNSTEER, $FINNMTR)
- NMEA checksum algorithm
- Watchdog timeout (500ms)
- PWM frequency (20kHz) and resolution (8-bit)
- BNO055 NDOF mode with calibration readout

**What changed:**
- BNO055 uses Adafruit library instead of raw I2C register access (same result,
  less code, well-tested library)
- LEDC PWM uses Arduino `ledcSetup()`/`ledcWrite()` instead of `esp-idf-hal` wrapper
- GPS UART uses Arduino `HardwareSerial` instead of `esp-idf-hal::uart`

**Old Rust crates** (`firmware-sensor/`, `firmware-motor/`) remain in the repo but
are archived — no longer built or maintained.

**Alternatives considered:** Persevering with Rust (rejected: multiple hours spent
on toolchain issues with no firmware actually running — the complexity tax is not
justified for simple embedded I/O), ESP-IDF native C without Arduino (rejected:
Arduino framework provides the same ESP-IDF underneath with a simpler API and
better library ecosystem, no practical downside for this use case).

---

## #016 — Dual-ESP32 auto-detect: distinguish sensor vs motor by sentence type
**Date:** 10 April 2026  
**Context:** The sensor ESP32 and motor ESP32 are both connected via USB serial
(separate COM ports). Both send `$FINN`-prefixed sentences, so the original
auto-detect (which just looked for `$FINN`) could not distinguish them. On first
run, the sensor reader grabbed COM6 (motor) because USB ports were enumerated in
an unexpected order. This caused two problems: (1) the sensor reader tried to send
PAIR GPS config commands to the motor ESP32, which blocked for 90 seconds waiting
for an ack that would never come; (2) the motor reader then couldn't find its port
because the sensor reader had already claimed it.

**Decision:** The sensor reader's auto-detect reads up to 40 lines from each port
and classifies by sentence type:
- `$FINNWAS`, `$FINNIMU`, `$FINNHB` → sensor ESP32 (claim this port)
- `$FINNMTR` → motor ESP32 (skip this port, let the motor reader find it)
- `$GNGGA`, `$GNVTG` etc. → raw GPS module (claim, but keep reading to check
  for FINN sentences — the sensor ESP32 also passes through GPS NMEA)

The motor reader runs in a separate thread and auto-detects independently, excluding
the port already claimed by the sensor reader (communicated via a one-shot channel).

**Why 40 lines:** The sensor ESP32 sends WAS+IMU at 20Hz and GPS at 1Hz. In the
worst case, the auto-detect's first line is a GPS sentence. At 20Hz FINN output,
a `$FINNWAS` or `$FINNIMU` will appear within 2-3 more lines. 40 lines gives
comfortable margin even with GPS GSV satellite blocks (which can be 10+ lines
per epoch).

**ESP32-aware config skip:** When `is_esp32` is true, the reader skips
`ensure_module_config()` entirely. PAIR commands sent to the ESP32's USB serial
would be consumed by the ESP32's main loop (which doesn't understand them) rather
than reaching the LC29H behind UART2. The GPS module retains its 1Hz default
config, which is correct for the DA variant.

**Alternatives considered:** Separate config for port names (rejected: defeats the
purpose of auto-detect and breaks portability between PCs), USB VID/PID matching
(rejected: both ESP32s use identical CH340 USB-serial chips — VID/PID is the same),
fixed port assignment (rejected: COM port numbers change when USB ports are
rearranged or the system moves to a different PC).

---

## #017 — Coverage data to SQLite, replacing CSV as primary store
**Date:** 13 April 2026  
**Context:** First field test in the tractor cab revealed that coverage strips
rendered as dashes rather than a continuous painted swath. The cause: the 1m
distance-based filter produced points spaced too far apart — the quad-strip renderer
in `draw_coverage()` draws a rectangle between consecutive points, so 1m spacing
left 1m gaps between each strip. Additionally, the CSV-based architecture had
structural limitations: no efficient partial reload, no spatial querying, and the
in-memory vec with 100k downsample cap degraded rendering fidelity over long runs.

**Decision:** Move all coverage point storage to SQLite, replacing CSV as the
primary data store. Distance filter reduced from 1.0m to 0.25m for gap-free
coverage rendering.

**Schema changes (`db.rs`):**
- New `coverage_points` table: `(id, job_id, segment, timestamp_ms, lat, lon, alt,
  speed, heading, fix_quality, sats, hdop)` with index on `(job_id, segment)`.
- `jobs` table: `csv_filename` column replaced with `name` (migration handles
  existing databases).
- New methods: `insert_coverage_batch()` (transactional batch insert),
  `load_coverage_points()`, `count_coverage_points()`, `clear_coverage_points()`,
  `export_coverage_csv()` (generates CSV string from DB for export).

**Logger changes (`logger.rs`):**
- Removed all CSV file handling (`File`, `PathBuf`, `fs::write`).
- Removed in-memory vec downsample logic and 100k cap.
- Added 50-point write buffer that flushes to SQLite in a single transaction.
- Distance filter default changed: 1.0m → 0.25m.
- All mutation methods (`toggle_engage`, `log_fix`, `clear_coverage`, `end_job`)
  now take `db: Option<&CoverageDb>` parameter.
- Added `load_job_coverage()` for reloading previous jobs into the render cache.
- Render cache grows as points arrive (no downsample needed — SQLite is source
  of truth and can be reloaded).

**Data volume at 0.25m:**
- At 10km/h (~2.78m/s) and 1Hz GPS: 1 point per fix (machine moves ~2.78m per
  fix, well above 0.25m threshold). So in practice, point rate is still 1Hz —
  the tighter filter just ensures we log at low speeds too.
- At 5km/h: still 1 point per fix (~1.39m movement).
- A 10-hour day: ~36,000 points. SQLite handles millions trivially.

**CSV export preserved:** `export_coverage_csv()` on `CoverageDb` generates the
same CSV format as before. Can be triggered from UI (not yet wired up — future
job history enhancement).

**What's unchanged:** `field_view.rs` renderer still takes `&[CoveragePoint]` —
it doesn't know or care whether the source is CSV or SQLite. Zoom-dependent
render thinning still applies. `main.rs` unchanged.

**Status:** Code complete, NOT YET TESTED. Requires `cargo build` verification
and field test to confirm gap-free coverage rendering at 0.25m.

**Alternatives considered:** Reducing CSV distance filter without changing storage
(rejected: CSV is still append-only with no efficient reload or query — just kicks
the can on the structural issues), spatial database / R-tree (rejected:
over-engineering for sequential coverage data — the access pattern is always
"load all points for job X", not spatial range queries).

---

## #018 — WAS calibration as PC-side three-point mapping, not ESP32-side
**Date:** 15 April 2026
**Context:** The wheel angle sensor (10kΩ pot replacing the RQH100030) outputs a
raw ADC value (0–4095) via the sensor ESP32. To use this for PID steering control,
the raw value needs to be mapped to a meaningful steering angle. The question was
whether to perform calibration on the ESP32 (sending calibrated angles over serial)
or on the PC (sending raw ADC and calibrating in software).

**Decision:** Calibration is performed entirely on the PC side. The ESP32 sends raw
ADC values in `$FINNWAS` sentences. The PC stores three calibration points in the
SQLite config table (`was_centre`, `was_left_lock`, `was_right_lock` as raw ADC
counts) and computes the steering angle via `was_calibrated_angle()` — a piecewise
linear mapping from [left_lock, centre, right_lock] to [-45°, 0°, +45°].

**UI:** WAS CALIBRATION section in Setup page with a three-step wizard:
1. Turn wheels straight → press "Set Centre"
2. Turn to full left lock → press "Set Left Lock"
3. Turn to full right lock → press "Set Right Lock"
Each press saves the current `latest_was.raw_value` to SQLite immediately. A live
ADC readout provides feedback during calibration. Status shows "Calibrated" (green)
with L/C/R values, or "Not calibrated" (amber). "Recalibrate" button clears all
three values. The SENSORS section shows the calibrated angle alongside raw values
when calibration is complete.

**Field test result:** centre=1800, left lock=1950, right lock=1650. The pot's ADC
range is ~300 counts across the full steering range. **Known issue:** left lock ADC
(1950) is higher than centre (1800), which means the piecewise mapping produces a
positive angle for left steering — the sign is inverted. Fix deferred to next session.

**Why ±45° default:** We don't know the actual lock-to-lock angle of this tractor's
steering. 45° is a reasonable placeholder — the PID controller only needs a
proportional signal (raw ADC → normalised position), not true degrees. If the actual
angle matters later, the MAX_ANGLE constant can be updated.

**Motor direction:** A separate `motor_invert` boolean config key was added alongside
WAS calibration. When true, `apply_motor_direction()` flips the PWM sign so that
positive always means "steer right" from the PID's perspective. Toggled via a
"MOTOR DIRECTION" section in Setup page. Not yet field-verified (IBT-2 suspected
hardware failure).

**Alternatives considered:** ESP32-side calibration (rejected: would require
reflashing to recalibrate, and the ESP32 doesn't have persistent storage for
calibration values without adding EEPROM/NVS code), sending calibrated angles
over serial (rejected: raw values are more flexible — the PC can recalibrate
without touching firmware), single-point calibration with assumed linearity
(rejected: pot response may not be perfectly linear across the range —
three points handles asymmetric steering geometry).

---

## #019 — Coverage render thinning bridging fix (revises #014)
**Date:** 15 April 2026
**Context:** After the SQLite migration (#017), coverage rendering still showed
dashed strips in the field. The 0.25m distance filter and SQLite storage were
working correctly — the problem was in the rendering layer.

**Root cause:** The zoom-dependent render thinning from Decision #014 used a `step`
variable (1–4) to skip points when zoomed out. The renderer drew a quad from
point[i] to point[i+1], then advanced the loop counter by `step`. When step > 1,
this meant the quad only covered the distance from i to i+1, but the loop then
jumped to i+step — leaving the gap from i+1 to i+step undrawn. The result was a
regular dashed pattern: one quad drawn, (step-1) quads missing, repeat.

**Decision:** Change the quad bridging target from `i+1` to `i+step`. Each rendered
quad now spans directly from point[i] to point[i+step], covering the full distance
to the next rendered point. At step=1 (close zoom), behaviour is identical to before.
At step=4 (zoomed out), each quad is ~4x longer but there are 4x fewer of them —
the coverage band remains continuous with no visible gaps.

**Segment boundary handling:** If i+step crosses into a different segment (or past
the end of the array), the renderer falls back to i+1 within the same segment. If
neither i+step nor i+1 are in the same segment, the point is drawn as a small
isolated square (existing fallback behaviour).

**Field test result:** Coverage now renders as solid continuous strips at all zoom
levels. Confirmed in tractor cab on Dell 7390.

**Alternatives considered:** Disabling render thinning entirely (rejected: would
cause performance issues when zoomed out with thousands of points), drawing all
intermediate quads even when thinning (rejected: defeats the purpose of thinning
for performance).

---

## #020 — Auto-steer as P-control in GUI loop with safety auto-disengage
**Date:** 15 April 2026
**Context:** The motor controller hardware is confirmed working (IBT-2 replaced,
motor responds to MOTOR TEST buttons in the tractor cab). WAS calibration values
updated (L:1617, C:1832, R:2031 — angle sign now correct). The next step is
closing the loop: XTE → motor PWM → physical steering correction.

**Decision:** Implement a `SteeringController` in `pc/src/guidance/steering.rs`
using proportional control as the initial strategy. The controller runs in the
GUI update loop (~60fps), computing `pwm = -kp × xte` from the interpolated
cross-track error and sending the result via `motor_handle.send_steer()`.

**Control architecture:**
- Input: `CrossTrackError.distance_m` from `AbLineGuide::calculate_error()`,
  computed from the interpolated GPS position (same source as lightbar/XTE display).
- Output: PWM value (-255 to +255), sent every frame to keep the ESP32 watchdog fed.
- Sign convention: positive XTE = vehicle right of line → negative PWM = steer left
  to correct. `apply_motor_direction()` (motor_invert toggle) is applied by the
  caller after the controller returns, keeping the controller sign-agnostic.
- Deadband: XTE < 3cm → zero output (prevents motor hunting when on-line).
- Speed gate: speed < 0.5 m/s → zero output (prevents GPS-drift steering at standstill).
- Clamp: output clamped to ±max_pwm (default 180, configurable 50–255).

**Safety system:**
- GPS fix timeout: if no real GPS fix received for >2 seconds, auto-disengage and
  send PWM 0. Prevents runaway steering if GPS cable is disconnected or module fails.
- WAS data timeout: if no `$FINNWAS` received for >1 second, auto-disengage.
  Prevents steering without position feedback from the wheel angle sensor.
- ESP32 watchdog (firmware-side): motor ESP32 kills motor if no `$FINNSTEER` received
  within 500ms. This is the ultimate safety net — if the PC app crashes, freezes, or
  loses USB connection, the motor stops within half a second.
- Manual disengage: ⊗ STEER OFF button on working page immediately sends PWM 0 and
  disengages. The button is always clickable when engaged (no precondition checks).
- Engage preconditions: AB line loaded + motor ESP32 connected + WAS calibrated (all
  three centre/left/right values set). Button greyed out if any precondition is unmet.

**UI placement:**
- Working page toolbar: ⊕ AUTO-STEER / ⊗ STEER OFF button, sized for touch, next
  to the coverage ENGAGE button. Blue when available, red when engaged.
- Working page overlay: green "AUTO-STEER PWM N" pill (top-left below lightbar)
  showing live output when engaged. Amber status messages for state changes.
- Setup page: AUTO-STEER section with Kp slider (20–300, persisted to SQLite),
  max PWM slider (50–255), deadband slider (0–20cm). Live status display.

**Why P-only to start:** The system is running standalone GPS (no RTK), so position
accuracy is ±1–2m. A proportional controller is the simplest thing that could
possibly work, and its behaviour is easy to reason about in the field. Adding
derivative (Kd) or integral (Ki) terms before understanding the basic system
response would risk masking fundamental issues (wrong motor direction, WAS sign
error, GPS latency). P-only will either work acceptably or reveal clearly what
additional terms are needed (oscillation → add Kd, steady-state offset → add Ki).

**Default tuning rationale:**
- Kp = 100: at 1m off-line, outputs 100 PWM (~39% of 255). Conservative enough
  that the motor won't slam the steering, aggressive enough to see a response.
- max_pwm = 180: leaves headroom below 255. The Trimble EZ-Steer motor doesn't
  need full power for normal corrections.
- deadband = 3cm: larger than GPS noise at standstill, small enough that the
  controller engages before the operator notices drift. Will likely need to increase
  to 10–20cm for standalone GPS due to position wander.

**Why run in the GUI loop (not a separate thread):** The GUI loop already has
access to the interpolated position, the guidance error, the motor handle, and
all configuration state. Running the controller in a separate thread would require
shared state synchronisation (Arc<Mutex>) for all of these, adding complexity for
no benefit — the GUI loop runs at 60fps which is more than fast enough for tractor
steering dynamics. The ESP32 watchdog timeout (500ms) is 30× longer than the
frame interval (~16ms), so there's ample margin even if frames are occasionally
slow.

**Alternatives considered:** PID from the start (rejected: unnecessary complexity
before basic P behaviour is validated), separate control thread at fixed rate
(rejected: adds synchronisation overhead without improving control quality — 60fps
is already faster than the GPS update rate), Stanley controller or pure pursuit
(rejected: these are path-following algorithms that require heading + look-ahead
distance — overkill for straight AB line following where cross-track error alone
is sufficient; reconsider for curved guidance lines in future phases).

---

## #021 — Heading error feedforward in outer loop (fixes diagonal overshoot)
**Date:** 16 April 2026
**Context:** Second field test with the two-loop controller (#020) revealed a
fundamental flaw: when the tractor approached the AB line at an angle, the outer
loop drove XTE toward zero and commanded `desired_angle = 0` ("straighten up").
But "straight wheels" only means "driving parallel to the line" if the tractor's
heading already matches the line bearing. If the tractor approached at e.g. 15°,
straightening the wheels meant driving a straight line AT 15° to the AB line.
The tractor punched through the line and kept going — overshooting so far that
auto-pass snapped to the next AB line (12m away), causing the tractor to drive
perpendicular to all the AB lines.

Traditional guidance systems with only XTE control produce a weaving/wave pattern.
Ours was worse because the 12m implement width meant the perpendicular overshoot
exceeded the auto-pass 60% snap threshold (7.2m).

**Root cause:** Pure XTE control has no heading awareness. It knows *how far* off
the line but not *which direction the tractor is pointed relative to it*.

**Decision:** Add heading error as a feedforward term in the outer loop. The formula
changed from:
```
desired_angle = -Kp × XTE
```
to:
```
desired_angle = -Kp × XTE - Kh × heading_error
```

Where `heading_error` is the difference between the tractor's GPS heading (from VTG)
and the AB line bearing, normalised to ±180°. This was already computed in
`AbLineGuide::calculate_error()` and returned in `CrossTrackError.heading_error` —
it just wasn't used by the controller.

**Sign convention:** Positive heading_error = pointed right of line bearing. The `-Kh`
sign ensures this produces a negative desired angle (steer left to correct), matching
the XTE term's sign logic.

**New parameter — Kh (heading error gain):**
- Default: 0.5 °/° (10° off bearing → 5° of desired steering correction)
- Range: 0.0 (disabled, pure XTE) to 2.0 (very heading-aggressive)
- Persisted: SQLite config key `steer_kh`, saved on slider change
- UI: Slider in Setup page AUTO-STEER section, between Kp and Kp_angle
- Warning: amber label when Kh < 0.1 ("tractor will overshoot line")

**Deadband update:** Previously, the deadband only checked XTE (`xte < deadband_m →
zero output`). This meant the controller stopped correcting when on-line but still
pointed diagonally. Now requires BOTH `xte < deadband_m` AND `heading_error < 2°`.
The 2° heading threshold prevents hunting when well-aligned.

**Display updates:**
- Working page overlay: `AUTO-STEER  PWM N  T:-X° A:-Y° H:-Z°` (H: = heading error)
- Setup page engaged status: `Target: X°  Actual: Y°  Hdg err: Z°`
- `last_heading_error` field added to `SteeringController` for display

**Why this works:** The heading term naturally damps the approach. As the tractor
turns toward the line, XTE decreases AND heading aligns simultaneously. When both
are small, `desired_angle → 0` and the wheels straighten — but now "straight" means
"pointed along the line", not just "wheels centred". This is standard in all
commercial guidance systems (AgOpenGPS, Trimble, Raven, etc.).

**Tuning guidance:**
- Start at Kh = 0.5 (conservative)
- If still overshooting: increase to 0.8, then 1.0, then 1.2
- If sluggish getting onto line: decrease to 0.3
- Kh interacts with Kp: high Kh with high Kp may oscillate; reduce Kp first
- At Kh = 0: behaviour reverts to pure XTE control (the old broken behaviour)

**Alternatives considered:** Stanley controller (rejected: would require a complete
rewrite of the outer loop architecture; the heading feedforward achieves the same
core benefit — heading-aware line tracking — with a single additive term),
derivative term on XTE (rejected: Kd would damp the rate-of-change of XTE, which
helps with oscillation but doesn't address the fundamental problem of not knowing
which way the tractor is pointed), look-ahead point tracking (rejected:
over-engineering for straight AB lines — reconsider for curved guidance).

---

## #022 — Max steer angle cap (15°) and sensor rate reduction (20Hz→10Hz)
**Date:** 16 April 2026
**Context:** Third field test with heading error fix (#021) revealed two additional
problems:

1. **Full-lock runaway**: when the tractor was far off-line, the outer loop
commanded increasingly aggressive steering angles up to the previous cap of 25°.
25° is close to the physical steering lock limit. At full lock the tractor turns
in a tight circle and can't recover back to the line — it just circles. The Kp
gain of 30 °/m means you only need to be 0.83m off-line to hit 25° (30 × 0.83 = 25).
With standalone GPS wander of 1-2m, this happens routinely.

2. **System lag**: the sensor ESP32 was sending WAS + IMU at 20Hz (41 messages/second
total), and the PC was sending steer commands at 20Hz (50ms interval). On the Dell
7390 field laptops, this was too much serial I/O — causing visible lag in both the
steering response and the UI. The lag compounded because slow frames caused message
queue buildup, and draining the queue made the next frame slower.

**Decisions:**

**Max steer angle reduced to 15° (adjustable 5°–30°):** At 15° with Kp=30, the
controller commands max steering at 0.5m off-line. Beyond that distance, the
desired angle plateaus at 15° and the tractor sweeps back in a gentle arc. This is
a much more farmable behaviour than cranking to full lock. The previous 25° default
was too close to physical limits where the steering geometry becomes nonlinear and
the tractor enters tight circles. The slider (5°–30°, persisted to SQLite as
`steer_max_angle`) lets the operator find the sweet spot for their tractor's
turning radius.

**Sensor rate: 20Hz → 10Hz:** The WAS potentiometer doesn't change fast enough to
benefit from 20Hz sampling — at tractor speeds the steering angle changes at maybe
1-2°/second. 10Hz captures this with ample margin. Halving the sensor rate:
- Cuts serial message volume from ~41 msg/s to ~21 msg/s
- Halves the parsing load on the PC's serial reader thread
- Reduces the `finn_rx` channel queue depth (fewer messages per GUI frame)

**Steer command rate: 20Hz → 10Hz:** Sending steer commands faster than the WAS
update rate is pointless — the inner loop can't react to wheel position changes
it hasn't measured yet. 10Hz steer commands with 10Hz WAS readings means every
command uses the latest available wheel angle. The ESP32 watchdog (500ms) still
has 5× safety margin.

**GUI optimisation:** Moved `steering.notify_was_reading()` from inside the
per-message loop to once-per-frame after the loop. When multiple WAS messages
arrive between frames (during burst drain), only one timestamp update is needed.

**Firmware change required:** The sensor ESP32 must be re-flashed with the updated
`SENSOR_INTERVAL_MS` (50→100). Run `pio run --target upload` in
`firmware-sensor-pio/`.

**Alternatives considered:** Reducing to 5Hz (rejected: might miss fast WAS changes
during aggressive corrections — 10Hz is conservative enough), adaptive rate based
on steering activity (rejected: complexity without clear benefit — fixed 10Hz is
simple and adequate), moving serial I/O to a separate thread (rejected: the serial
read is already on its own thread; the lag was from message volume overwhelming the
GUI frame budget, which is fixed by sending less).

---

## #023 — Pure pursuit outer loop (replaces XTE+heading PD controller)
**Date:** 16 April 2026
**Context:** Fourth field test with the two-loop PD controller (#020, #021, #022)
revealed that the tractor was hunting the line rather than tracking parallel to it.
As XTE approached zero, the motor was increasing its turn rather than decreasing —
the classic oscillation pattern of a position-error controller regulating through
two integrators (wheel angle → heading → lateral position) with insufficient
damping.

**Root cause:** The `-Kp·XTE - Kh·heading_error` formulation weighted two competing
terms against each other. At the defaults (Kp=30, Kh=0.5), 1 m of XTE had the same
control authority as 60° of heading error — but realistic driving produces XTEs
of 0.1–0.5 m and heading errors of 3–10°. The XTE term dominated by ~36×, so the
heading term couldn't provide meaningful damping as XTE crossed zero. Result:
tractor arrives at the line with substantial heading error, XTE term winds down,
heading term is too weak to counter-steer, tractor sails through, develops
negative XTE, hunt cycle begins.

**Structural problem:** Regulating lateral position through two integrators with
pure P gain on position error is unconditionally oscillatory without strong rate
damping. Commercial guidance systems (Trimble AgGPS, John Deere StarFire, AgLeader,
AgOpenGPS) don't use this topology — they use pure pursuit or Stanley control,
which reformulate the problem from "error → correction" to "geometry → trajectory."

**Decision:** Replace the PD outer loop with pure pursuit. The inner loop (WAS
feedback → PWM with min_pwm floor and angle deadband) is unchanged.

Pure pursuit selects a lookahead point on the AB line some distance `L` ahead of
the tractor's projection onto the line, then commands the wheel angle that would
curve the bicycle-model vehicle through that point:

```
L = lookahead_base + lookahead_speed_factor × speed
alpha = atan2(-xte, L) - heading_error    [closed form, line-frame geometry]
desired_angle = atan2(2 · wheelbase · sin(alpha), L)
desired_angle = clamp(desired_angle, ±max_steer_angle)
```

**Why this topology works:** XTE and heading error are captured in a single
geometric quantity (`alpha`) — they can't fight each other because they're two
views of the same thing (the bearing to the lookahead point). The controller is
inherently damped: as the tractor approaches the line, both XTE and heading
naturally go to zero together, and the lookahead-based curvature command smoothly
tapers rather than snapping at XTE=0. No Kp/Kh balance to get wrong.

**Parameters replaced:**
- Removed: `kp` (°/m), `kh` (°/°)
- Added: `lookahead_base` (m, default 3.0), `lookahead_speed_factor` (s, default
  1.0), `wheelbase_m` (m, default 2.8 — tractor-specific, not a tuning knob)
- Kept: `max_steer_angle`, `kp_angle`, `max_pwm`, `min_pwm`, `angle_deadband_deg`,
  `deadband_m` — all inner-loop parameters work identically

**Operator-facing UI:** The two pure-pursuit lookahead parameters are exposed
as sliders labelled "Approach Aggression" and "Online Aggression", each with a
1–10 range where higher = more aggressive (shorter lookahead). This inverts the
underlying math (smaller lookahead = crisper/more aggressive) to give intuitive
tuning. The slider labels show the internal value in physical units so the
operator can learn the mapping:
- Approach Aggression → `lookahead_speed_factor`: slider 1 = 3.0 s time-horizon
  (very gentle), slider 7 = 1.2 s (balanced default), slider 10 = 0.3 s (aggressive)
- Online Aggression → `lookahead_base`: slider 1 = 7.5 m (very smooth), slider 5
  = 5.1 m (balanced default), slider 10 = 2.1 m (crisp)

The live computed lookahead (`base + speed × factor`) is displayed on both the
working-page overlay (`L:N.Nm`) and the setup page AUTO-STEER section, so the
operator can see the lookahead expand with speed and understand how the
controller is framing the problem at any moment.

**Config key migration:** Old `steer_kp` and `steer_kh` keys are ignored (left
in SQLite config table as harmless dead data). New keys: `steer_lookahead_base`,
`steer_lookahead_speed_factor`, `steer_wheelbase`. `steer_max_angle` and
`steer_kp_angle` are retained. `steer_kp_angle` default bumped 4.0 → 10.0 to
match the new controller's expected angle-error magnitudes (pure pursuit
produces smaller commanded angles for the same XTE than the old Kp=30 did).

**Alternatives considered:**
- Stanley controller (rejected: Stanley's `atan(k·XTE/speed)` term divides by
  speed and misbehaves near standstill; pure pursuit is well-defined at any
  speed with its base lookahead floor)
- Retuning the PD controller (rejected: structural oscillation, not a tuning
  issue — would recur under any parameter choice)
- Reinforcement learning (rejected for now: policy is trivial — "drive along
  the line" — so the right fix is better sensing and geometry, not a learned
  policy. Reconsider in Phase 7 if vehicle-specific refinements need learning)

---

## #024 — Fused heading filter (IMU + GPS complementary filter)
**Date:** 16 April 2026
**Context:** Investigation of the hunting behaviour in #023 revealed a deeper
problem with the heading signal itself. The steering controller was consuming
`fix.heading` from GPS VTG course-over-ground, which has three compounding
defects:

1. **Rate-limited to 1 Hz** — the LC29H DA firmware caps at 1 Hz (Decision #008).
   Between fixes, heading is frozen. At 30 fps the outer loop computes the same
   heading error 30 times in a row.
2. **Noisy at low speed** — GPS COG is derived from successive position deltas.
   At 5 km/h (1.4 m/s) the tractor moves ~1.4 m per fix, while standalone GPS
   position noise is ±30–50 cm per fix. The bearing between two noisy points
   separated by ~1 m can wobble by ±10–20°.
3. **Stale after turns** — because the interpolator dead-reckons position forward
   using `fix.heading`, turning the tractor leaves the projected heading behind
   for up to 1 s.

Meanwhile, a BNO055 IMU was physically installed, its data was being parsed into
`latest_imu` at 10 Hz, and its heading was being displayed in the SENSORS panel —
but it was never fed into any guidance calculation. The best heading source on
the tractor was serving as a dashboard gauge.

**Decision:** Add a `HeadingFilter` (new module `pc/src/position/heading_filter.rs`)
that fuses BNO055 IMU yaw with GPS COG using a complementary filter. The fused
heading is passed as an `override_heading` parameter to `PositionInterpolator`
and thereby overrides `fix.heading` for all downstream guidance calculations
(AB line error, pure-pursuit alpha, field view rotation).

**Filter structure:**
```
On each IMU sample:
  dt = now - last_imu_time
  imu_yaw_rate = wrap_diff(imu_heading - prev_imu_heading) / dt
  predicted = wrap(fused + imu_yaw_rate · dt)
  fused ← predicted

On each GPS fix (speed ≥ 0.8 m/s):
  diff = wrap_diff(gps_cog - fused)
  fused ← wrap(fused + (1 - alpha) · diff)   [alpha = 0.98]
```

The `alpha = 0.98` constant means ~2% pull toward GPS COG per fix. With GPS at
1 Hz, this means the filter's long-term heading anchor is GPS, but it runs at
IMU rate (10 Hz) between fixes — giving clean, high-frequency heading response
with no long-term drift.

**Gating:**
- **IMU trusted only if `cal_sys ≥ 2`** (BNO055 system calibration metric).
  Below this, the magnetometer reference isn't trustworthy. Filter falls back
  to GPS COG only (no IMU prediction contribution).
- **GPS COG trusted only if `speed ≥ 0.8 m/s`** (~2.9 km/h). At lower speed,
  COG is noise. Filter holds on IMU prediction only at standstill / creep.
- **Neither source available → fused heading is `None`** and the interpolator
  falls back to the old behaviour (using the stale `fix.heading` from the last
  real GPS sample).

**UI:** A new line in the SENSORS panel shows the fused heading with a colour
code:
- Green "`Fused heading: N.N° (IMU+GPS)`" — both sources active
- Amber "`Fused heading: N.N° (GPS only — IMU not calibrated)`" — filter is
  operating but degraded. Operator is prompted to run the BNO055 calibration
  dance (figure-eight motions until cal_sys = 2+)

**Wrap-around handling:** All heading math uses `wrap_diff()` (returns signed
shortest-path difference in -180..+180) and `normalise_360()` helpers. Avoids
the classic bug where 5° and 355° appear 350° apart instead of 10°.

**Interpolator change:** `PositionInterpolator::interpolate()` gained an
`override_heading: Option<f64>` parameter. When supplied, it replaces `fix.heading`
for both the dead-reckon projection direction AND in the returned synthetic fix's
`heading` field. This means position tracking during turns is now correct (no
1-second lag) and all downstream consumers see the fused heading transparently.

**Why complementary filter, not Kalman:** A Kalman filter would require process
and measurement noise covariances that we'd need to measure or guess. The
complementary filter achieves ~95% of the benefit with zero tuning, using only
the IMU calibration gate and the speed gate as state transitions. We can upgrade
later if telemetry reveals a need.

**Alternatives considered:**
- Use IMU heading directly when calibrated, ignore GPS (rejected: IMU heading
  drifts with magnetometer interference — GPS provides long-term anchoring)
- Kalman filter with proper covariances (rejected: premature optimisation;
  complementary filter is good enough and has fewer knobs to get wrong)
- Integrate raw gyro rate from the BNO055 separately (rejected: the firmware
  sends BNO055's fused heading, not raw gyro; differentiating the fused heading
  gives us yaw rate for free without firmware changes)
- Run the filter on the ESP32 (rejected: calibration gating, GPS COG delivery,
  and the fused output all live on the PC anyway — keeping filter logic PC-side
  is simpler and doesn't require re-flashing to tune)

---

## #025 — Waveform-aware inner loop (replaces hard deadbands with smooth damping)
**Date:** 18 April 2026
**Context:** Epiphany from real driving observation: straight-line tractor driving
is not actually straight — the steering wheel constantly oscillates left and right.
A human driver tracks a line by minimising the *amplitude* of this oscillation, not
by snapping to a fixed angle. The existing inner loop had three mechanisms that
worked against this natural oscillation model:

1. **Hard XTE deadband** (3cm + heading < 2° → desired_angle = 0): this clipped
   the control signal at the zero-crossing — the motor went silent in exactly the
   zone where fine corrections matter most. The tractor coasted through, overshot,
   and only got a correction once it was 3cm+ past the line.

2. **Hard angle deadband** (angle_error < 2° → PWM = 0): created a dead zone where
   the motor stopped and the tractor drifted. Combined with the min_pwm stall floor
   (100 PWM), this produced a bang-bang pattern: 0→0→0→suddenly 100→0→0→0→-100.

3. **No rate information**: the controller had no knowledge of whether XTE was
   growing or shrinking. A tractor at 3cm XTE and converging rapidly got the same
   correction as one at 3cm and diverging. No mechanism to damp the oscillation
   amplitude over successive cycles.

**Decision:** Three changes to the inner loop in `guidance/steering.rs`. The outer
loop (pure pursuit geometry from #023) is unchanged.

**Change 1 — XTE rate damping (dXTE/dt):**
New field `kd_xte` (default 0.5, persisted to SQLite as `steer_kd_xte`). Each
`compute()` call calculates the rate of change of cross-track error from the
previous sample. The damping term is added to the desired angle:
```
desired_angle += kd_xte × dXTE/dt
```
When converging (dXTE/dt has opposite sign to XTE), this reduces the desired angle
magnitude — softer approach, less overshoot. When diverging (same sign), it
increases correction urgency. This is the core amplitude-reduction mechanism that
makes the oscillation waveform converge rather than sustain.

Rate calculation is gated: dt must be between 1ms and 1s to reject double-calls
and stale samples. On first call after engage, rate is zero (no history). State
(`prev_xte_m`, `prev_compute_time`) is reset on both engage and disengage.

**Change 2 — Smooth taper replaces hard XTE deadband:**
Instead of snapping `desired_angle` to zero when XTE < `deadband_m`, the desired
angle is now scaled by `taper = |XTE| / deadband_m` within the deadband zone
(still gated by heading_error < 2°). At the deadband boundary the scale is 1.0
(full correction); at XTE = 0 the scale is 0.0 (natural zero). The waveform
passes through the zero-crossing smoothly instead of being clipped.

**Change 3 — Sub-stall pulsing replaces hard angle deadband:**
The hard angle deadband (`angle_error < 2° → PWM = 0`) is removed from the
control loop. Instead, the inner loop now operates in three zones:

- **Above stall floor** (`|desired_pwm| ≥ min_pwm`): direct drive with stall
  compensation, same as before. Pulse accumulator is reset.
- **Sub-stall zone** (`1 ≤ |desired_pwm| < min_pwm`): desired effort is
  accumulated in a `pulse_accumulator` each cycle. When the accumulator
  reaches ±min_pwm, one pulse at min_pwm is fired and the accumulator is
  decremented. This gives time-averaged torque below the stall floor —
  the motor makes small periodic corrections instead of either doing nothing
  or slamming at full stall torque.
- **Negligible zone** (`|desired_pwm| < 1`): accumulator is zeroed, motor is
  silent. This replaces the hard deadband — the motor only goes truly silent
  when the entire control chain (pursuit + damping + taper) outputs
  essentially nothing.

The `angle_deadband_deg` field is retained in the struct and UI slider for
backward compatibility but is no longer read by `compute()`.

**UI changes:**
- New "XTE damping (Kd)" slider in Setup page AUTO-STEER section, range
  0.0–2.0, step 0.1. Persisted to SQLite as `steer_kd_xte`.
- Angle deadband slider remains but is now effectively inert (cosmetic, can
  be removed in a future cleanup).

**New tests added:**
- `test_converging_xte_reduces_correction`: verifies that when XTE is
  shrinking, the desired angle magnitude is reduced vs steady-state.
- `test_diverging_xte_increases_correction`: verifies the opposite.
- `test_sub_stall_pulsing`: verifies that repeated sub-stall calls
  eventually fire a pulse at min_pwm.

**Field test tuning guidance:**
- Start at kd_xte = 0.5 (default). If still overshooting the line, increase
  toward 1.0. If corrections feel sluggish on approach, decrease toward 0.2.
- Sub-stall pulsing should be immediately noticeable as small periodic
  motor twitches when near the line, replacing the old dead-silence → sudden
  kick pattern.
- The combination of smooth taper + rate damping should produce visibly
  smoother line tracking compared to the old hard-deadband controller.

**Alternatives considered:**
- Full PID on XTE (rejected: integral windup is dangerous for a mobile plant —
  accumulated error during a headland turn would produce a large kick on
  re-entry. Rate damping achieves the needed dynamic response without windup)
- Kalman filter on XTE (rejected: over-engineering for this stage; simple
  finite-difference dXTE/dt is adequate given 30fps compute rate)
- Variable-rate PWM output instead of pulse accumulation (rejected: the ESP32
  motor controller firmware uses fixed 20kHz PWM; we can't change the PWM
  frequency per-command. Pulse accumulation achieves the same average-torque
  effect within the existing firmware)

---

## #026 — Hardware simplification: LC29H BA direct-connect + inner loop on ESP32 (revises #008, #015, #016, #018, #024, #025)
**Date:** 21 April 2026
**Context:** Field test 6 with the waveform-aware inner loop (#025) showed sluggish
steering corrections — the tractor responded slowly to XTE errors despite the smooth
taper and sub-stall pulsing working as designed. Analysis identified a structural
bottleneck: the inner loop runs on the PC at 10Hz command rate (limited by the WAS
sample rate from the sensor ESP32 and the 100ms steer-send throttle). Each correction
cycle therefore takes at minimum 100ms, with real-world round-trip latency of
150–200ms (WAS → USB → PC parse → compute → USB → motor ESP32). At 5 km/h the
tractor travels 20–28cm per correction cycle, and multiple cycles are needed to
converge — corrections always feel like they're arriving late.

Concurrently, LC29H BA GPS modules arrived on new ArduSimple boards. The BA variant
has two key advantages over the DA: (1) 10Hz fix rate (vs 1Hz on the DA), and
(2) onboard IMU with internal GPS+IMU dead-reckoning fusion. The BNO055 IMU mounted
on the sensor ESP32 has never achieved reliable calibration in the field — the tractor
doesn't produce the rapid angular movements needed for magnetometer calibration,
so `cal_sys` rarely exceeds 1. The HeadingFilter (#024) falls back to GPS-only
heading most of the time, negating its purpose.

Study of the AgOpenGPS codebase confirmed that commercial/open-source guidance
systems run the inner loop (WAS → PWM) on the microcontroller at 100Hz+, with the
PC sending a desired steer angle (not raw PWM) at a slower rate. This architecture
gives the inner loop 10× faster reaction time than FINN's current PC-side approach.

**Decision:** Two simultaneous hardware/architecture changes:

### Change 1 — LC29H BA replaces DA, connects directly to laptop USB (no sensor ESP32)

The ArduSimple LC29H BA board connects directly to the laptop via USB serial. It
outputs standard NMEA (GGA, VTG) at 10Hz plus Quectel proprietary DR sentences
(`$PQTMDRCAL` for calibration status, `$PQTMVEHMOT` for fused DR heading). The
BA's onboard IMU handles heading fusion internally — no external IMU needed.

**Calibration:** The BA self-calibrates by driving at >3 m/s with 3–4 turns for
approximately 3 minutes. Calibration state is reported in `$PQTMDRCAL` with
`CalState` field (0 = uncalibrated, 1 = calibrating, 2 = calibrated). This is
dramatically easier than the BNO055 (no figure-eights, no magnetic interference
issues, no per-session recalibration).

**Mounting:** The BA module must be rigidly fixed to the vehicle frame (no relative
movement). No orientation restrictions — the module auto-detects its mounting angle.

**What this eliminates:**
- Sensor ESP32 (entire board removed from cab)
- BNO055 IMU board and I2C wiring
- `firmware-sensor-pio/` firmware (archived, no longer maintained)
- `position/heading_filter.rs` (HeadingFilter — replaced by BA's internal fusion)
- FINNIMU parser in `gps/finn_parser.rs`
- BNO055 calibration UI in setup page
- GPS UART passthrough complexity (ESP32 was just forwarding NMEA from the LC29H)

### Change 2 — Inner loop moves to motor ESP32 with local WAS

The motor ESP32 gains the WAS potentiometer input (wire moved from sensor ESP32
GPIO 34 to motor ESP32 — plenty of free ADC pins: 32, 33, 34, 35, 36, 39). It
runs a local inner loop at 50–100Hz:

```
loop() at ~100Hz:
  1. Read WAS via ADC → convert to angle using stored calibration
  2. Check for new $FINNSTEER,<desired_angle_x100> from PC (arrives at 10Hz)
  3. angle_error = desired_angle - actual_angle
  4. pwm = kp_angle × angle_error
  5. Apply min_pwm stall boost / max_pwm clamp / sub-stall pulsing
  6. Drive IBT-2
  7. Report: $FINNMTR,<pwm>,<was_raw>,<actual_angle_x100>,<enabled>
  8. Watchdog: no $FINNSTEER for 500ms → stop motor
```

**Protocol change:** `$FINNSTEER,<value>` changes meaning from raw PWM (-255..255)
to desired angle × 100 (e.g. `$FINNSTEER,-523` = -5.23°). The motor ESP32 does the
angle-to-PWM conversion locally. `$FINNMTR` gains WAS and actual-angle fields so
the PC can display them without needing a separate sensor serial port.

**WAS calibration moves to ESP32 NVS:** The three-point calibration values
(centre, left-lock, right-lock ADC counts) are stored in ESP32 NVS (non-volatile
storage) so they survive power cycles without re-flashing. A new config sentence
`$FINNCFG,WAS,<centre>,<left>,<right>*XX` allows the PC to push calibration values
to the ESP32 during the calibration wizard. The ESP32 acknowledges with
`$FINNACK,WAS,OK*XX`. Additional config sentences for inner loop tuning:
`$FINNCFG,PID,<kp_angle_x100>,<min_pwm>,<max_pwm>*XX`.

**Inner loop parameters that move to ESP32:**
- `kp_angle` (PWM per degree of angle error)
- `min_pwm` (motor stall floor)
- `max_pwm` (output clamp)
- Sub-stall pulse accumulator
- Motor direction invert flag

**What stays on the PC (outer loop only):**
- Pure pursuit geometry (lookahead, wheelbase, max_steer_angle)
- XTE rate damping (kd_xte) — operates on XTE not angle, so stays in outer loop
- Smooth taper near the line — operates on desired angle before sending
- Speed gate (below min_speed → send desired_angle = 0)
- GPS fix timeout safety (disengage → send desired_angle = 0 continuously)
- The PC no longer needs WAS data for control, but still receives it via
  `$FINNMTR` for display and diagnostics

### Combined new architecture

```
Laptop USB ──► ArduSimple LC29H BA (direct serial)
                 └── 10Hz GGA + VTG + $PQTMDRCAL + $PQTMVEHMOT
                 └── DR provides fused heading, position through GNSS gaps

Laptop USB ──► Motor ESP32 (sole microcontroller)
                 ├── WAS pot ADC input (wire moved from old sensor ESP32)
                 ├── Inner loop: desired angle vs WAS → PWM at 50-100Hz
                 ├── IBT-2 H-bridge motor drive
                 ├── Status: $FINNMTR with PWM, WAS, angle, enabled
                 └── Config: $FINNCFG for WAS cal + PID params via NVS
```

**USB device count:** 2 (down from 3). One COM port for GPS, one for motor ESP32.

### Impact on PC-side code

**`gps/reader.rs`:** Simplified — connects directly to the ArduSimple COM port
(auto-detect looks for `$GNGGA`/`$GNVTG` without FINN sentences). Sends
`$PAIR050,100` to configure 10Hz output (100ms interval). No more `is_esp32`
flag or PAIR-skip logic. Adds parsing for `$PQTMDRCAL` (DR calibration state).

**`gps/parser.rs`:** Unchanged — GGA and VTG parsing is identical regardless of
the module variant. 10Hz data rate means the parser runs 10× more often.

**`gps/finn_parser.rs`:** Simplified — remove `$FINNWAS` and `$FINNIMU` parsers
(no longer received from GPS port). `$FINNMTR` parser updated to include WAS and
angle fields. `$FINNHB` removed (sensor ESP32 gone).

**`position/interpolator.rs`:** Retained but much less critical. At 10Hz GPS, the
interpolator bridges 100ms gaps (was 1000ms). Dead-reckoning error in 100ms at
5 km/h is ~14cm vs ~140cm at 1Hz. Could be simplified or eventually removed.

**`position/heading_filter.rs`:** **Deleted.** The BA's internal fusion replaces
this entirely. GPS VTG heading at 10Hz is already usable without external IMU
fusion. The `$PQTMVEHMOT` sentence provides an even better DR-fused heading.

**`guidance/steering.rs`:** Major simplification. The outer loop (pure pursuit
+ XTE rate damping + smooth taper) stays. The inner loop (kp_angle, min_pwm,
max_pwm, sub-stall pulsing, angle deadband) is **removed entirely**. The
`compute()` function now returns a desired angle (f64) instead of a PWM (i16).
The caller sends `$FINNSTEER,<angle×100>` to the motor ESP32.

**`comms/serial.rs`:** Motor serial handler updated:
- `send_steer()` sends desired angle instead of PWM
- Parses extended `$FINNMTR` for WAS/angle display data
- New `send_config()` methods for WAS calibration and PID params

**`gui/app.rs`:** Setup page changes:
- WAS CALIBRATION wizard now sends `$FINNCFG,WAS,...` to ESP32 instead of
  storing locally. The ESP32 saves to NVS.
- Inner loop sliders (Kp_angle, min_pwm, max_pwm) send `$FINNCFG,PID,...`
  to ESP32 on change.
- BNO055 calibration UI removed. Replaced with DR calibration status from
  `$PQTMDRCAL` (uncal/calibrating/calibrated indicator).
- SENSORS panel simplified: no IMU section, just GPS + WAS (from $FINNMTR).

**Auto-detect (`main.rs`):** Simplified from three-device to two-device:
- Port with `$GNGGA`/`$GNVTG` = GPS module (claim for GPS reader)
- Port with `$FINNMTR` = motor ESP32 (claim for motor handler)
- No more sensor/motor disambiguation logic from #016

### Motor ESP32 firmware changes

The `firmware-motor-pio/src/main.cpp` grows from ~150 lines to ~350 lines:
- Add ADC reading for WAS pot (same code as current sensor ESP32)
- Add NVS storage for calibration values and PID parameters
- Add `$FINNCFG` parser for receiving config from PC
- Add inner loop P controller with sub-stall pulsing
- Extend `$FINNMTR` status to include WAS raw, calibrated angle
- Add `$FINNACK` response for config commands
- Inner loop runs at hardware timer rate (50–100Hz), decoupled from serial I/O

### Wiring changes in cab

Only one physical change: move the WAS pot signal wire from the sensor ESP32's
GPIO 34 to a free ADC pin on the motor ESP32 (suggest GPIO 34 for consistency,
or GPIO 36/39 which are input-only with clean ADC). The pot's 3.3V reference
wire moves from sensor ESP32 GPIO 33 to motor ESP32 GPIO 33 (or any free output
pin). GND is common. The sensor ESP32, BNO055 board, and their associated wiring
are removed from the cab entirely.

### What this revises

- **#008 (1Hz interpolation):** LC29H BA outputs 10Hz natively — the workaround
  of interpolating between 1Hz fixes is largely obviated. Interpolator kept for
  smoothing but gap is 100ms not 1000ms.
- **#015 (dual ESP32 firmware):** Sensor ESP32 firmware archived. Only motor
  ESP32 firmware is active.
- **#016 (dual-ESP32 auto-detect):** Simplified to two-device detect (GPS vs
  motor ESP32). No more sensor/motor sentence-type disambiguation.
- **#018 (WAS calibration PC-side):** Calibration values move to ESP32 NVS.
  PC-side wizard UX unchanged but sends config via serial instead of storing
  locally.
- **#024 (BNO055 heading filter):** Entire HeadingFilter module deleted. BA's
  onboard IMU fusion replaces external BNO055 + complementary filter.
- **#025 (waveform inner loop):** Sub-stall pulsing and inner loop move to
  ESP32 at 50–100Hz. The PC-side steering controller becomes outer-loop only.

### Implementation phases

**Phase A — Hardware + firmware (do first):**
1. Wire WAS pot to motor ESP32 (one wire move)
2. Flash new motor ESP32 firmware with inner loop + NVS + WAS ADC
3. Connect LC29H BA ArduSimple board directly to laptop USB
4. Verify GPS sentences at 10Hz, confirm DR calibration process
5. Remove sensor ESP32, BNO055, and old wiring from cab

**Phase B — PC software refactor:**
1. Update GPS reader for direct LC29H BA connection (remove ESP32 passthrough)
2. Add `$PQTMDRCAL` parser for DR calibration status display
3. Strip inner loop from `steering.rs` — return desired angle, not PWM
4. Update motor serial handler for new `$FINNSTEER` (angle) and `$FINNMTR` format
5. Update `$FINNCFG` send for WAS cal and PID params
6. Delete `heading_filter.rs`, remove BNO055 UI
7. Simplify auto-detect to two-device model
8. Update config persistence (inner loop params sent to ESP32, not stored locally)

**Phase C — Field test 7:**
1. Verify GPS at 10Hz — position updates, heading, DR status
2. Verify WAS reading on motor ESP32 (compare ADC values to known calibration)
3. Verify inner loop: send fixed desired angles from MOTOR TEST, watch WAS track
4. Full auto-steer test: engage on AB line, assess responsiveness vs field test 6
5. Key metric: does the 100Hz inner loop eliminate the sluggishness?
6. Tune inner loop PID on ESP32 if needed (via Setup page sliders → $FINNCFG)

### Risk assessment

**Low risk:**
- WAS ADC reading on motor ESP32 — identical hardware, proven code from sensor ESP32
- LC29H BA direct serial — same NMEA protocol, just faster fix rate
- NVS storage on ESP32 — well-documented ESP32 feature, Arduino `Preferences.h`

**Medium risk:**
- Inner loop timing on ESP32 — need to verify that ADC read + PID + PWM output
  fits within the loop period at 100Hz. The current motor firmware loop is trivially
  fast (just serial parse + PWM write), so adding an ADC read and a few multiplies
  should be fine, but needs bench verification.
- DR calibration in field conditions — the BA requires driving at >3 m/s (10.8 km/h)
  for calibration. This is working speed for broadacre but may require a deliberate
  drive-around before engaging auto-steer on first startup.

**Low risk of regression:**
- Outer loop (pure pursuit) is unchanged — same code, same parameters
- Safety systems (GPS timeout, watchdog, manual disengage) are unchanged
- Coverage, AB lines, field management — completely unaffected

**Alternatives considered:**
- Keep sensor ESP32 and relay WAS to motor ESP32 via direct serial link between the
  two ESP32s (rejected: adds inter-ESP32 wiring complexity and doesn't reduce USB
  device count — the whole point is simplification)
- Keep sensor ESP32 and relay WAS via the PC (rejected: adds latency that defeats
  the purpose of moving the inner loop to the ESP32)
- Keep BNO055 and add it to motor ESP32's I2C (rejected: the BNO055 calibration
  problem is fundamental — it needs rapid angular movement the tractor doesn't
  produce. The BA's onboard IMU solves this at the hardware level)
- Run inner loop on PC at higher rate via a dedicated thread (rejected: still
  limited by USB serial round-trip latency — even at 100Hz PC-side compute, the
  command has to traverse USB twice. On-ESP32 eliminates the serial hop entirely)
- Keep DA module and just move inner loop to ESP32 (rejected: misses the 10Hz GPS
  upgrade and heading fusion improvement. The BA boards are already purchased and
  available — no cost barrier)

---

## #027 — Drop-on-full channel sends in the serial reader threads

**Date:** 22 April 2026 (Session 22; verified field tests 8 & 9)
**Status:** Accepted
**Context:** Field test 7 with the air seeder hitched showed intermittent ~3-second
freezes — the GUI locked and the tractor physically drifted ~1 m before recovering.
A code audit of `main.rs` and the two reader threads found the cause. All four
channels between the serial readers and their consumers are bounded (GPS: 64,
FINN: 128), and both reader threads (`gps/reader.rs`, `comms/serial.rs`) used
blocking `send()`. Each reader feeds *two* consumers from a single thread: the GUI
and the steer thread. When the GUI had a slow frame its channel filled, the reader
blocked on that `send()`, and — because the same thread also feeds the steer-thread
channel — the steer thread stopped receiving fixes too. The steer thread starved,
emitted no `$FINNSTEER`, the motor ESP32 correctly held its last commanded angle
(~5°), and the tractor drifted until the GUI recovered and the channels drained.
The slow consumer was backpressuring the fast one through the shared reader thread.

**Decision:** Both reader threads use `try_send` and drop the message when a
channel is full, instead of blocking. Drop counts are tracked two ways: a local
rolling counter logged as WARN every ~5 s (human-visible in field logs), and shared
atomic counters (`SharedDropCounters`, `Arc<AtomicU64>`) that the steer thread
reads-and-resets each second for the telemetry summary records. Guidance gets the
latest-value semantics a real-time control loop actually wants — a stale 3-second
fix is worse than a skipped one, so dropping and computing off the next fix is
correct.

**Trade-off (drop vs block):** dropping can lose a fix under load, but at 1 Hz GGA
/ 10 Hz PQTMINS into a 64-deep channel, drops only occur once a consumer has
stalled across many fixes — exactly when the freshest fix matters and the backlog
is worthless. The drop counters make any sustained dropping visible, so a genuine
throughput problem can't hide behind the silent discard.

**Verified:** field tests 8 (25 April) and 9 (28 April) both logged zero channel
drops across all four counters with no backpressure freezes. The single residual
3.2 s stall in FT8 originated upstream of the reader thread (USB serial / GPS
module pause), not from channel backpressure.

**Alternatives considered:** larger channel buffers (rejected: only delays the
freeze and adds latency — accumulating stale fixes is the wrong thing for real-time
guidance); a separate reader thread per consumer (rejected: doubles serial parsing
and still wouldn't help when a consumer stalls — the bottleneck is consumer speed,
not reader throughput); unbounded channels (rejected: a stalled consumer would grow
memory without bound — trades a freeze for a leak).

**Hardens:** the threading model established in #026. Never re-introduce blocking
`send()` on a reader → consumer channel without explicitly considering the
shared-thread starvation risk.

---

## #028 — LC29H BA DR remediation: PQTMINS/PQTMIMU with corrected NR11 parsing

**Date:** 30 April 2026
**Status:** Accepted
**Context:** Field testing in hilly country showed roll compensation stuck at
zero, so the tractor drifted laterally on slope. The detailed remediation plan
is in `docs/DR_REMEDIATION_STRATEGY.md`. Probe work identified the module as
LC29H BA NR11 two-wheel firmware:

```text
$PQTMVERNO,LC29HBANR11A02S_CSA2,2023/05/09,20:49:45
```

The module accepts `PQTMCFGEINSMSG` only with numeric `Type` fields, not the
literal `W` previously used by `reader.rs`. It streams `$PQTMINS` and `$PQTMIMU`
once enabled and saved, but roll/pitch remain zero until the module has had a
proper DR calibration drive.

**Decision:** Use `$PQTMINS` from the LC29H BA NR11 firmware as the attitude and
DR-fused heading source. Continue to source position from GGA. Enable:

```text
$PQTMCFGEINSMSG,1,1,1,0,10
```

This turns on PQTMINS and PQTMIMU at 10Hz while leaving PQTMGPS off. Also assert
`$PAIR6010,2,1` at startup so `$PQTMDRCAL` remains available for the GUI
calibration-state indicator.

**Parser mapping:** `$PQTMINS` is:

```text
$PQTMINS,<Timestamp>,<SolType>,<Lat>,<Lon>,<Height>,
         <VEL_N>,<VEL_E>,<VEL_D>,<Roll>,<Pitch>,<Heading>
```

So:

- speed is derived from `sqrt(VEL_N^2 + VEL_E^2)`
- roll is field 9
- pitch is field 10
- heading is field 11
- lat/lon/height are ignored in favour of GGA

`SolType=0` must still parse roll/pitch. It means DR is not ready, but roll and
pitch can be ready. Heading is only trusted from PQTMINS when `SolType >= 1`.

**No runtime hot-start:** Although the Quectel docs say config changes take
effect after reset, probe work showed `$PAIR007` can destroy the current fix for
longer than a normal field startup can tolerate. The app saves config to NVS and
expects the next power cycle/startup to stream DR data. If PQTMINS is already
streaming, startup skips the DR config write to avoid unnecessary NVS wear.

**Required field procedure:** Code fixes alone do not produce non-zero roll.
Each module still needs a DR calibration drive: rigid mount, clear sky, drive
above 2 m/s with several turns for roughly three minutes until `$PQTMDRCAL`
reports calibrated.

**Revises:** The DR portions of #026. The hardware simplification remains, but
the previous inline comments and parser assumptions about PQTMINS were wrong.


---

## #029 — Manual roll calibration: mounting-bias capture + direction toggle

**Date:** 30 May 2026
**Status:** Accepted
**Context:** Roll compensation (the lateral antenna-swing correction added under
#028) assumed two things that are not true across different tractor installs:
(1) that the GPS module reads exactly 0° roll when the vehicle is level, and
(2) that positive `$PQTMINS` roll means right-side-down with the antenna swinging
right (corrected leftward). Neither holds in general. A module bolted to a cab
roof that isn't perfectly flat, or on an angled bracket, reads a constant non-zero
roll on level ground — so the correction injects a constant lateral position error
(`antenna_height × sin(bias)`, e.g. ~10 cm at 3 m and 2°) on every pass, even on
the flat. And the roll *sign* depends on module orientation/firmware, which the
code could only assume, not verify. Both are install-to-install differences that
made deployment to a second tractor a guessing game, and a wrong sign would only
show up as unexplained XTE drift on slopes — exactly the kind of error that gets
harder to attribute as the rest of the stack improves.

**Decision:** Add a manual roll calibration with two persisted, per-machine
parameters, plus telemetry of the actually-applied correction.

1. **Roll mounting-bias offset** (`roll_offset_deg`, config key `roll_offset_deg`).
   The parser computes `effective_roll = smoothed_roll - roll_offset_deg` and uses
   that for the correction. Captured at install by parking on flat, level ground
   and pressing **Capture Level**, which records the current (raw, EMA-smoothed)
   module roll as the bias zero. A constant offset, so subtracting it before vs
   after the EMA is identical — capture reads the already-smoothed resting value.
2. **Roll direction invert** (`roll_invert`, config key `roll_invert`). A
   Normal/Inverted toggle that negates the computed lateral correction, turning
   the previously-buried sign assumption into a setting the operator verifies on a
   slope.
3. **Applied-correction telemetry** (`roll_corr_m` on `GpsFix` and in the
   telemetry `IterRecord`). The signed lateral shift actually applied to each
   fix's position (positive = shifted left of travel, i.e. bearing heading−90°),
   after offset and invert. Previously the correction was silently baked into
   lat/lon with no record; now it is visible in the `.jsonl` logs and the GUI, so
   a wrong sign or bad offset is diagnosable directly instead of as mystery XTE.

**Correction math (in `gps/parser.rs::try_build_fix`):**
```
effective_roll = smoothed_roll - roll_offset_deg
if has_ins_attitude && antenna_height_m > 0 && |effective_roll| > 0.1°:
    lateral = antenna_height_m * sin(effective_roll)
    if roll_invert: lateral = -lateral
    bearing = normalise(corrected_heading - 90°)   # heading-90 = leftward
    position = apply_offset(lat, lon, bearing, lateral)   # negative dist reverses
    roll_corr_m = lateral
else:
    roll_corr_m = 0.0
```
The skip-tiny-roll gate now tests `effective_roll`, not raw roll, so a calibrated
bias of 2° correctly yields ~0 correction on the flat. With `roll_offset_deg = 0`
and `roll_invert = false` the math reduces exactly to the pre-existing #028
correction — the feature is a no-op until calibrated, so it is safe to ship before
any capture is done.

**Plumbing:** Two new shared atomics on the existing heading-offset/antenna-height
pattern — `SharedRollOffset` (`Arc<AtomicI32>`, centidegrees) and
`SharedRollInvert` (`Arc<AtomicBool>`) — created in `main.rs`, cloned into the GPS
reader thread, and polled into the parser each sentence. The GUI holds local
copies, loads both from SQLite on startup and pushes them to the atomics, and
persists on change via `apply_roll_offset()` / `apply_roll_invert()`.

**UI (Setup page, ROLL CORRECTION section):** below the existing antenna-height
controls, a "Level calibration" block shows the current mounting bias with a
**◎ Capture Level** button (enabled once attitude data is present) and a Clear
button; a "Roll direction" block shows a Normal/Inverted toggle. The live readout
and the SENSORS attitude readout now show *effective* roll (raw minus bias) and
the *actually-applied* `roll_corr_m` from the fix, rather than recomputing from
raw roll — so what the operator sees matches what was applied.

**Required field procedure:**
1. Park on the flattest, most level ground available; confirm the live roll is
   stable; press Capture Level. Effective roll should drop to ~0°.
2. On a known cross-slope, watch the live correction and XTE. If the correction
   pushes XTE *away* from zero, press Direction: Inverted once. The `roll_corr_m`
   telemetry then confirms the sign from the logs. This single observation
   resolves the sign question for that machine permanently.

**Why not a dynamic two-direction (drive-the-line-both-ways) auto-calibration:**
considered and deferred. It would estimate bias, sign, and effective lever-arm
automatically by overlaying reciprocal passes, but it is a multi-part build
(track recording, pass-pair association, a solver, an error-prone operator
procedure) and mostly automates values that can be set by hand in under a minute.
The one thing it adds that the manual path can't — the sign check — is delivered
far more cheaply by the explicit invert toggle plus the live/telemetry readout.
Revisit if curved-line work or fleet scale makes per-machine manual calibration
burdensome.

**Alternatives considered:** static level capture only, no invert (rejected: leaves
the sign unverifiable, which is the more dangerous of the two unknowns on slopes);
changing `GpsFix.roll` to carry effective roll (rejected: changes the meaning of an
existing field and the capture needs the raw value — instead the GUI derives
effective roll locally from `roll - roll_offset_deg` and the new `roll_corr_m`
carries the applied truth); storing the offset on the ESP32 like WAS cal (rejected:
roll correction is entirely PC-side, so the calibration belongs with the PC config
in SQLite).

**Files touched:** `common/src/types.rs` (`roll_corr_m` on `GpsFix`),
`gps/parser.rs` (effective-roll correction + invert + `roll_corr_m`, 5 tests),
`gps/reader.rs` (two shared atomics + polling), `main.rs` (create + thread the
atomics), `telemetry/logger.rs` (`roll_corr_m` in `IterRecord`),
`guidance/steer_thread.rs` (populate it), `gui/app.rs` (controls, persistence,
readouts). Also a small dead-code cleanup in `steer_thread.rs` (removed unused
`current_implement_w` / `current_overlap` captures).

**Companion work this session (not strictly part of this decision):** the AB-line
sign-convention consolidation in `ab_line.rs` — `signed_line_offset()` /
`travel_sign()` primitives, the return-pass `align_grid_to_position` fix, the new
`snap_to_nearest_pass()`, and correction of the inverted sign-convention comments
in `coords.rs` / `types.rs` (code was always correct; comments lied). See the
commit history and `ab_line.rs` module docs for the full convention.


---

## #030 — Bound GPS config ack-wait loops with a wall-clock deadline

**Date:** 1 May 2026 (Session 29; written up Session 30)
**Status:** Accepted — hot-fixed live during seeding
**Numbering note:** chronologically this is 1 May work and sits before #029 (30
May), but it was written up later during a docs tidy. Decision numbers here are
assignment order, not strictly date order.

**Context:** During seeding the app came up with "no GPS fix in GUI". Diagnosed
live via Windows-MCP / Filesystem-MCP on the tractor laptop while Tom drove. Root
cause: the rate-set ack-wait loop in `ensure_module_config()` (`gps/reader.rs`)
waited for the module's `PAIR001,050` acknowledgement using only a per-read
timeout, with a fallback `_ => break`. But the LC29H BA streams ~11 NMEA
sentences/second continuously (1 Hz GGA + 10 Hz PQTMINS), so every `port.read()`
returned data within the timeout and the fallback never fired. The loop ran
forever; the GPS reader thread never reached its main NMEA processing loop; no
fixes ever reached the GUI or the steer thread. The distinctive symptom was the
log stopping at `Trying 10Hz (100ms): $PAIR050,100*22` with the GPS thread then
silent forever — it looked like a clean startup followed by a dead GPS, but was
actually a hung config thread.

**Decision:** Bound the ack-wait loop with a 500 ms `Instant`-based wall-clock
deadline in addition to the per-read timeout (which was reduced 300 ms → 100 ms,
no longer the safety net). After the deadline the loop exits and control flows
normally through SAVEPAR → main read loop. On the rebuild, first fix arrived 1.2 s
after launch. **General rule for this codebase: any `port.read()` ack-loop against
a continuously-streaming device must have an `Instant`-based deadline — a per-read
timeout alone is insufficient because the read never times out.**

**Hardware finding documented alongside:** the LC29H BA NR11A02S firmware caps
GGA/VTG output at 1 Hz regardless of `$PAIR050`. The module actually returns
`PAIR001,050,1` ("unsupported") to `$PAIR050,100`, but the buffered-ack reader
misses it in the NMEA flood and falls through to the "may still be applied" log
path; SAVEPAR then persists the still-1 Hz state to NVS. **This is fine and matches
the architecture** — `gps/parser.rs` uses GGA only for position and PQTMINS for
10 Hz heading/velocity/attitude, and the interpolator dead-reckons between 1 Hz
GGA fixes. Field coverage points landed at exactly 1.00 Hz (deltas 997–1003 ms
across 50 consecutive points), confirming. This **resolves FT8 Finding 5** ("GPS
effective rate ~1–2 Hz, not 10 Hz") as by-design hardware behaviour, not a bug.

**Companion to #027:** both decisions are about not letting a blocking or looping
construct in the I/O path stall the whole pipeline.

**Alternatives considered:** parse the `PAIR001,050` response properly to detect
the "unsupported" code and skip cleanly (a reasonable future improvement, but the
deadline is the robust catch-all regardless of what the module returns); remove the
`$PAIR050` send entirely on this firmware since it's a no-op that costs ~700 ms of
startup (noted as a possible cleanup; left in place as harmless for now).

**Open follow-up:** the log line "No ack for 10Hz (may still be applied)" is
misleading — it should name the NR11 1 Hz cap explicitly so the next debugger
doesn't lose an hour. Tracked in ACTIVE_CONTEXT.
