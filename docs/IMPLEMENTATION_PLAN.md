# FINN Guidance - Implementation Plan

## Session Continuity

> **New session?** Read these files first, in this order:
> 1. `docs/ACTIVE_CONTEXT.md` — current state, what to work on next
> 2. `docs/DECISIONS.md` — settled architectural decisions (don't revisit)
> 3. This file — for task tracking and session history
>
> Use `codesnip:edit_snippet` for all code changes. Never rewrite entire files.

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
   - "Set A" button captures current GPS position
   - "Set B" button captures second position
   - Draw the AB line on the field view
   - Extend AB line infinitely in both directions

2. **Cross-track error calculation** ✅
   - Calculate perpendicular distance from current position to AB line
   - Calculate heading error (current heading vs line bearing)
   - Display as large, readable number (cm) with left/right indicator
   - Colour code: green (<5cm), yellow (5-15cm), red (>15cm)

3. **Pass management** ✅ (manual next/prev)
   - Set implement width
   - Next/previous pass buttons offset the guidance line
   - Display current pass number
   - Draw parallel pass lines on field view

4. **Auto-pass selection** ✅
   - Distance-based approach: continuously monitors cross-track error from the
     current pass line. When the operator drifts more than `snap_threshold` (default
     60%) of the implement width from the current line, snaps to the nearest pass.
   - Works for all scenarios: headland U-turns, driving around obstacles, skipping
     rows, diagonal approaches — no heading analysis needed.
   - Hysteresis via snap_threshold (0.6 = must cross 60% before snapping, prevents
     flickering at the boundary between two passes)
   - Speed gate: auto-pass disabled below 0.5 m/s (~1.8 km/h) to avoid GPS jitter
     at standstill causing false snaps
   - Manual override still available via next/prev buttons
   - "Auto ✓"/"Auto ✗" toggle button in controls bar
   - Blue notification ("Auto → Pass N") on screen for ~3 seconds when triggered
   - Active pass line rendered in blue (3px) to distinguish from red AB line and
     faint red hypothetical pass lines
   - Design note: skip factor (Decision #002) is no longer needed for auto-pass
     selection — the distance approach is inherently skip-agnostic. Skip factor
     remains relevant only for future auto-turn path generation (Phase 5+).

5. **Lightbar indicator** ✅
   - 31 segments (15 per side + 1 centre) rendered as painter overlay at top of
     working page field view
   - Segments light up in the direction you need to steer TOWARDS (left of line →
     right segments light up)
   - Colour ramp: green (centre) → yellow (mid) → red (edges), computed via
     `lightbar_colour()` helper function
   - Sensitivity: `lightbar_cm_per_seg` (default 2.0 cm/segment, 30cm full scale)
   - Semi-transparent black background bar for readability over field view
   - Unlit segments shown as dim outlines
   - Centre segment always lit green when guidance is active

6. **AB line persistence** ✅
   - Save AB lines to SQLite database with optional name
   - Load saved AB lines from database
   - Two-level field→line model, JSON export/import, last-loaded auto-restore
   - Associate active AB line with coverage segments

### Done when
- Can set AB line, drive parallel passes, and see accurate cross-track error ✅
- System automatically selects the correct pass line when operator moves laterally ✅
- Active pass line visually distinct (blue) from reference lines (red) ✅
- Lightbar provides intuitive at-a-glance guidance ✅
- Pass offset works correctly for implement width ✅
- Implement width and overlap adjustable in the UI ✅
- AB lines persist across sessions via database ✅
- Field-verified: AB line save/load and nudge confirmed working ✅

### Estimated effort: 3-4 sessions

---

## Phase 3: GUI Pages, Coverage Display & Config
**Goal:** Split the interface into purpose-built pages for working vs setup,
render real-time coverage on the field view, and manage data volume efficiently.

### Tasks
1. **GUI page system** ✅
   - **Working page**: full-screen field view with lightbar overlay (31 segments),
     large overlaid XTE readout (56pt, semi-transparent pill), auto-pass notification,
     large engage/auto buttons in bottom bar, "⚙ Setup" button. All buttons sized
     for tractor-cab touch targets (min_size).
   - **Setup page**: right side panel (240px, scrollable) with sections for AB LINE,
     IMPLEMENT (width +/- 0.5m steps, overlap +/- 5cm steps, pass spacing readout),
     GUIDANCE, COVERAGE, POSITION, VIEW. Field view still visible alongside panel.
   - Page switching via prominent buttons ("⚙ Setup" / "◄ Working View")
   - ActivePage enum (Working/Setup) — not yet persisted across restarts
   - Both pages share the GPS status strip (fix quality, sats, HDOP, speed, heading,
     REC indicator)

2. **Coverage rendering on field view** ✅
   - Draw logged coverage points as quad strips between consecutive points
     (using implement width and per-point heading for correct strip orientation)
   - Colour-code strips by fix quality (green=RTK, yellow=float, blue=DGPS,
     orange=GPS, red=NoFix) — semi-transparent so grid/lines show through
   - Segment boundaries respected (no strip drawn across disengage/engage gaps)
   - Viewport culling: skip off-screen points before computing quad geometry
   - Last point in a segment draws as a small standalone rectangle
   - Rendered on Layer 1.5 (above grid, below guidance lines and vehicle)
   - Still needed: area calculation (hectares), spatial indexing for large datasets

3. **Coverage data management** ✅
   - Zoom-dependent render thinning in draw_coverage() (step 1–4 based on visible width)
   - Memory cap at 100,000 points with oldest-half downsample (CSV unaffected)
   - "🗑 Clear Coverage" button for task transitions (CSVs on disk untouched)

4. **Job management** ✅
   - JOB HISTORY section in Setup page COVERAGE area
   - List last 10 jobs (date, points, segments) with delete button
   - list_jobs() and delete_job() methods in db.rs

5. **Configuration persistence** ✅
   - SQLite config table (key-value store, already existed in db.rs)
   - Implement width, overlap, lightbar sensitivity saved on every UI change
   - Last-loaded AB line ID saved on load, auto-restored on startup
   - Nudge deliberately excluded (session-specific, cleared by align-grid)
   - Lightbar sensitivity adjustable in new LIGHTBAR section (1–50 cm/seg, ±1 steps)

### Done when
- Working page is clean enough to use while driving — large readouts, no clutter ✅
- Setup page has all the controls for configuration and AB line management ✅
- Field view shows coverage strips in real-time while working ✅
- Coverage logging runs at a sustainable data rate for full-day operation ✅
- Can resume work with saved AB lines and see previous coverage ✅
- All settings persist across sessions ✅

### Estimated effort: 4-5 sessions

---

## Phase 4: ESP32 Steering Controller
**Goal:** ESP32 firmware that reads sensors and controls the steering motor.

### Tasks
1. **ESP32 Rust setup**
   - Install ESP Rust toolchain (`espup install`)
   - Create firmware crate with `esp-idf-hal`
   - Blink LED to verify toolchain works

2. **Wheel angle sensor**
   - Wire WAS through voltage divider (5V -> 3.3V)
   - Read ADC on GPIO 34
   - Calibration routine (record centre position, counts per degree)
   - Filter/smooth readings

3. **BNO055 IMU**
   - Wire BNO055 on I2C (GPIO 21/22)
   - Read roll, pitch, heading at 100Hz
   - Axis remapping for mounting orientation
   - Calibration save/restore

4. **Motor control**
   - Wire IBT-2 H-bridge to GPIO 25/26/27
   - PWM output for speed control
   - Direction control via RPWM/LPWM
   - Safety: motor stops if no command received for 500ms (watchdog)

5. **UDP communication**
   - ESP32 connects to WiFi
   - Send sensor data (WAS + IMU) to PC at 10Hz
   - Receive steer commands from PC
   - Heartbeat / connection monitoring

### Done when
- ESP32 reads WAS and IMU, sends to PC
- ESP32 drives motor in response to PC commands
- Motor stops automatically if communication lost

### Estimated effort: 4-5 sessions

---

## Phase 5: Closed-Loop Auto-Steer
**Goal:** Full auto-steer with PID controller.

### Tasks
1. **PID controller on PC**
   - Use `pid` crate
   - Input: cross-track error + heading error
   - Output: steer command (-255 to 255)
   - Tuneable P, I, D gains
   - GUI controls for tuning

2. **Steer integration**
   - Send PID output to ESP32 as steer command
   - ESP32 applies PWM to motor
   - WAS feedback closes the inner loop
   - IMU roll compensation (adjust for hillside)

3. **Safety systems**
   - Maximum steer rate limiting
   - Maximum wheel angle limiting
   - GPS fix quality gate (disable steer if fix degrades)
   - Speed-dependent gain adjustment
   - Physical disengage switch input on ESP32
   - Watchdog: motor off if any sensor times out

4. **Auto-turn at headland** 🔲
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
- Tractor steers itself along AB line with centimetre accuracy
- Safety systems prevent dangerous behaviour
- Can tune PID without reflashing firmware

### Estimated effort: 4-6 sessions

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
- [ ] Rust toolchain on field laptop (Dell Latitude 7390 2-in-1 ordered, in the post)

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

### Next up: Phase 4 — ESP32 Steering Controller
- Waiting on Dell Latitude 7390 2-in-1 laptops for field testing
- Phase 4 is hardware + firmware (ESP32, WAS, IMU, motor control)
- Review hardware shopping list before starting

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
  - Root cause: GUI loop at ~200Hz draining channel, logging each received fix
  - Parser emitted fix on every NMEA sentence, not just GGA
  - 3830 CSV rows for only 138 unique GPS positions
- Fixed NMEA parser: epoch-based emission, only emit fix on GGA sentences
  - VTG data accumulated but doesn't trigger a fix
  - One fix per GPS epoch instead of one per NMEA sentence
- Rewrote coverage logger with three-gate filtering:
  1. Epoch deduplication (skip if same timestamp_ms as last log)
  2. Time filter (configurable minimum interval)
  3. Distance filter (configurable minimum distance)
- Added LogFilter presets: every_fix(), distance_based(), time_based()
- Added SQLite coverage database (rusqlite with bundled feature):
  - Jobs table: CSV filename, implement width, timestamps, totals
  - Segments table: per-engage/disengage cycle within a job
  - AB Lines table: save/load guidance lines across sessions
  - Config table: key-value store for persistent settings
- Reorganised Phase 1-3 in implementation plan:
  - Phase 1 now includes field view canvas and coverage logging
  - Phase 2 focuses on AB line guidance with persistence
  - Phase 3 covers coverage rendering, job management, and config

### Session 4 (Mar 2026) — Ute road test + planning
- Second field test: drove ute along road with laptop + GPS
  - Confirmed 42 satellites, HDOP 0.4 (excellent signal quality)
  - Position trail, AB line, and pass lines all rendering correctly
  - Cross-track error readout working (showing -28331cm = ~283m off-line, expected for road test)
  - Coverage logging operational with 22 points recorded
- Reviewed real-world usability and identified key improvements needed:
  - **Auto-pass selection**: system must detect headland turns and automatically
    snap to the correct next pass line (how existing AG systems work)
  - **GUI page split**: current single-page layout too cluttered for tractor cab use.
    Need a clean "working page" (big lightbar, big XTD, big engage) and separate
    "setup page" (AB lines, config, detailed stats). HDOP + sats always visible.
  - **Coverage data management**: logging volume is a real concern for full-day
    operation. Need display-layer thinning, memory budgets, and rate monitoring
    separate from the CSV recording fidelity.
- Updated implementation plan:
  - Phase 2 expanded with auto-pass selection as a key task
  - Phase 3 restructured around GUI pages, coverage display, and data management
  - Added priority ordering for next development tasks

### Session 5 (27 Mar 2026) — Auto-pass, coverage rendering, field test + fixes
- Implemented auto-pass selection (initially heading-based with 90° threshold):
  - TravelDirection enum, heading reversal detection, debounce (30m + 0.5 m/s speed gate)
  - "Auto ✓"/"Auto ✗" toggle button and blue "Auto → Pass N" notification in GUI
- Implemented coverage strip rendering on field view canvas:
  - Quad strips between consecutive CoveragePoints using heading + implement width
  - Colour-coded by fix quality (green RTK, yellow float, blue DGPS, orange GPS, red NoFix)
  - Semi-transparent (alpha ~90/255) so grid and guidance lines show through
  - Viewport culling for performance, segment boundaries respected
  - Rendered on Layer 1.5 (above grid, below guidance lines)
- Third field test: ute drive with coverage + auto-pass
  - 46-50 satellites, HDOP 0.4 — coverage strips painting correctly in orange
  - Discovered pass line normal vector sign mismatch: drawing used `(-uy, ux)` (CCW)
    but cross_track_distance uses CW convention → blue highlight on wrong side
  - Fixed normal to `(uy, -ux)` to match cross-track sign convention
  - Discovered auto-pass heading approach was fragile: driving around obstacles or
    curves could trigger false pass changes when heading crossed 90° threshold
- Replaced heading-based auto-pass with distance-based approach (Decision #005):
  - Continuously monitors cross-track error from current pass line
  - Snaps to nearest pass when error exceeds 60% of implement width (7.2m for 12m)
  - Inherently handles all scenarios: turns, obstacles, row skipping, diagonal approaches
  - No heading analysis, no debounce position tracking, no TravelDirection state
  - Removed TravelDirection enum, last_direction, last_turn_position, turn_debounce_distance_m
  - Added snap_threshold (0.6) as the single tuning parameter
- Changed active pass line colour from green to blue (rgb 80, 160, 255) at 3px width
  to clearly distinguish from red AB line and faint red hypothetical passes

### Session 6 (27 Mar 2026) — GUI page split, lightbar, implement width/overlap
- Implemented GUI page split (Decision #004):
  - `ActivePage` enum (Working/Setup) controls which page is shown
  - **Working page**: full-screen field view, no side panel. Overlaid lightbar at top,
    large XTE readout (56pt) in semi-transparent pill top-right, auto-pass notification
    top-centre. Bottom bar: large ENGAGE button (20pt, 160×40px), Auto toggle, pass
    indicator, "⚙ Setup" button. All buttons have min_size for touch targets.
  - **Setup page**: right SidePanel (240px, scrollable) with sections — AB LINE
    (Set A/B, pass next/prev, auto-pass toggle), IMPLEMENT (width +/-, overlap +/-,
    pass spacing readout), GUIDANCE (XTE + heading error), COVERAGE (engage, stats),
    POSITION (lat/lon/alt/speed/heading), VIEW (heading-up/north-up, grid, zoom).
    Field view still visible in central panel.
  - GPS status bar extracted to `draw_status_bar()` — shared by both pages, shows
    fix quality, sats, HDOP, speed, heading, REC indicator.
  - `fix_quality_display()` helper replaced duplicated match blocks.
- Implemented lightbar indicator:
  - 31 segments (15 per side + 1 centre) rendered via egui Painter API
  - Segments light up towards the direction you need to steer (left of line → right
    segments illuminate)
  - Colour ramp via `lightbar_colour()`: green (centre) → yellow (mid) → red (edges)
  - Sensitivity: `lightbar_cm_per_seg` field (default 2.0), full scale = 30cm
  - Semi-transparent black background bar, unlit segments shown as dim outlines
  - Centre segment always lit green when guidance is active
- Implemented adjustable implement width:
  - +/− buttons in 0.5m steps (range 0.5–36.0m) in setup page
  - Changes synced to both `AbLineGuide.implement_width_m` and
    `CoverageLogger.set_implement_width()`
  - Pass offset recalculated immediately on width change
- Implemented overlap setting (Decision #006):
  - Added `overlap_m` field to `AbLineGuide` (default 0.0)
  - Added `pass_spacing()` method: `(implement_width_m - overlap_m).max(0.1)`
  - Updated all pass logic to use `pass_spacing()`: `next_pass()`, `prev_pass()`,
    `update_auto_pass()` (threshold + offset), `find_nearest_pass()`, field_view
    pass line rendering
  - Coverage strips still use full implement width (correct: shows actual swath)
  - UI: +/− in 5cm steps, capped at 90% of width, displayed in cm
  - Pass spacing shown as computed readout below controls
- All changes tested and verified working

### Session 7 (27 Mar 2026) — GPS 5Hz, auto-detect, distance-based logging
- Identified GUI sluggishness caused by 1Hz GPS output rate — the module was only
  sending one GGA sentence per second, so position/lightbar/XTE only updated once
  per second regardless of the GUI's 60fps refresh rate.
- Confirmed the existing architecture already supports decoupled rates: the GPS
  reader sends every fix through the channel, the GUI drains all available fixes
  per frame, and the coverage logger has independent three-gate filtering.
- Implemented GPS module configuration on startup (Decision #007):
  - `ensure_module_config()` sends PAIR commands to set 5Hz fix rate (200ms interval)
    and disable GSA/GSV sentences (we only use GGA + VTG)
  - Commands are idempotent — safe to send every boot
  - `format_pair_command()` helper computes NMEA checksums correctly
  - Configuration sent after port open, before the read loop starts
- Implemented auto-detection of GPS serial port:
  - `auto_detect_gps_port()` uses `serialport::available_ports()` to enumerate ports
  - USB ports probed first (most likely to be GPS), then others
  - Each port opened briefly at configured baud rate, checks for NMEA prefixes
    (`$G`, `$PAIR`, `$PQTM`) in up to 20 lines
  - `GpsConfig.port_name` defaults to `"auto"` — no manual COM port needed
  - Logs port type (USB product name, PCI, Bluetooth, Unknown) during scan
- Changed coverage logger default filter from every-fix to distance-based:
  - `LogFilter::default()` now uses `min_distance_m: 1.0` (was 0.0)
  - CSV only records when machine has moved ≥1m since last logged point
  - Naturally adapts to speed: more points when moving fast, zero when stationary
  - ~2,800 points/hr at 10km/h working speed (vs ~18,000/hr at 5Hz every-fix)
- Updated `GpsConfig`: added `fix_rate_hz` field (default 10), default baud 115200,
  default port "auto". `main.rs` now uses `GpsConfig::default()` instead of
  hardcoded values.
- Improved `ensure_module_config()` to disable sentences before setting rate (frees
  module CPU first), read back PAIR001 acks, and fall back through rates on rejection.
  Now disables GLL, GSA, GSV, RMC (not just GSA/GSV) — only GGA + VTG remain.
- Discovered LC29H DA variant only accepts 1Hz PVT: PAIR001,050 returned error code
  2 ("invalid parameter") for 100ms, 200ms, and 500ms intervals. 1000ms accepted.
  The EA variant would be needed for higher-rate hardware output.
- Implemented position interpolation (Decision #008):
  - New `PositionInterpolator` in `pc/src/position/interpolator.rs`
  - Dead-reckons between 1Hz fixes using `destination_point()` (spherical Earth
    model, same as existing coord math)
  - `display_fix` updated every GUI frame (~60fps) for smooth vehicle movement
  - Real fixes (`current_fix`) still used for: coverage logging, auto-pass detection,
    Set A/B, status bar metadata
  - Trail uses real fixes only (1Hz breadcrumb path is sufficient)
  - Safety: no interpolation below 0.3 m/s, capped at 2 seconds max extrapolation
  - Unit tests for `destination_point()` (north, east, zero distance)
- Changed lightbar sensitivity from 2 cm/segment to 20 cm/segment:
  - Old: 30cm full scale — far too acute for standalone GPS
  - New: 3.0m full scale — usable feedback without permanent max-out
  - Will reduce to 5-10 cm/segment when RTK is added
- All changes tested and verified working — smooth GUI, correct coverage logging,
  auto-detect and module config functioning correctly

### Session 8 (31 Mar 2026) — Nudge, align grid to here
- Implemented nudge feature (Decision #009):
  - `nudge_m` field on `AbLineGuide`, applied in `calculate_error()` and
    `find_nearest_pass()`. Three methods: `nudge_right()`, `nudge_left()`, `nudge_reset()`
  - Field view renders all pass lines at nudged positions
  - UI: NUDGE section in Setup page (±5cm standard, ±1cm fine, Reset button)
  - Working page: amber "Nudge N cm →R/←L" indicator when nudge ≠ 0
  - Hard cap ±200 cm to prevent accidental large shifts
- Implemented "Align Grid to Here" (Decision #011):
  - "⊕ Align Grid to Here" button in AB LINE section
  - Snaps pass grid so current GPS position falls on nearest whole pass line
  - Resets nudge to zero, preserves AB line geometry
  - Greyed out until line loaded + GPS fix available

### Session 9 (31 Mar 2026) — AB line persistence, field→line model
- Implemented AB line persistence (Decision #010):
  - Two-level field→line model: fields (paddocks) group AB lines
  - `fields` table added to SQLite schema, `ab_lines` has optional `field_id` FK
  - "💾 Save Line…" button with inline dialog (name + field picker)
  - SAVED LINES section with collapsible field headers, Load/Delete per line
  - "+ Field" button for creating new field groups
  - "⬆ Export JSON" / "⬇ Import JSON" for cross-PC transfer
    (`data/finn_ab_lines.json`, idempotent import with duplicate detection)
- Session was interrupted but all intended work was completed

### Session 10 (1 Apr 2026) — Bugfixes, config persistence, coverage management, Phase 3 completion
- Fixed compile errors from egui 0.29 API changes (Decision #012):
  - 3× curly-quote `format!()` errors (smart-quote corruption)
  - 1× `show_tooltip_text` signature change (added `LayerId` parameter)
  - 3× deprecated `id_source` → `id_salt` renames
  - 1× unused `PathBuf` import in db.rs
  - 1× borrow checker fix (unassigned lines Vec held references into self)
- Field tested — AB line save/load and nudge both confirmed working
- Implemented configuration persistence (Decision #013):
  - Implement width and overlap saved to SQLite config table on every UI change
  - Lightbar sensitivity saved (new LIGHTBAR section in Setup, ±1 cm/seg, 1–50 range)
  - Last-loaded AB line ID saved, auto-restored on startup
  - Nudge deliberately excluded (session-specific)
- Implemented coverage data management (Decision #014):
  - Zoom-dependent render thinning (step 1 at ≤100m visible, step 4 at ≥500m)
  - Memory cap at 100k points with oldest-half downsample
  - "🗑 Clear Coverage" button for task transitions (CSVs untouched)
- Implemented job management:
  - `list_jobs()` and `delete_job()` methods added to db.rs, `SavedJob` struct
  - JOB HISTORY section in Setup page (last 10 jobs, date/points/segments, delete)
- **Phase 3 complete.** Phases 1, 2, and 3 all done and field-verified.

---

## Hardware Shopping List

Already have:
- [x] 24V 500rpm brushed DC motor
- [x] Wheel angle sensor (WAS)
- [x] H-bridge motor driver (IBT-2)
- [x] GPS receiver (Quectel lc29h on ArduSimple board)

Still needed:
- [ ] BNO055 IMU breakout board (~$35 AUD from Adafruit/Core Electronics)
- [ ] ESP32 DevKit (~$15 AUD)
- [ ] Voltage divider resistors for WAS (10kΩ + 20kΩ, ~$0.50)
- [ ] Mounting hardware for motor + GPS antenna
- [ ] RTK base station OR NTRIP subscription for corrections

---

## Key Dependencies (Rust Crates)

| Crate           | Version | Purpose                          |
|-----------------|---------|----------------------------------|
| nmea            | 0.7     | NMEA sentence parsing            |
| serialport      | 4.0     | Serial port for GPS              |
| eframe/egui     | 0.29    | GUI framework                    |
| rusqlite        | 0.31    | Coverage database (bundled SQLite)|
| pid             | 4.0     | PID controller (Phase 5)         |
| bno055          | 0.4     | IMU driver (Phase 4)             |
| esp-idf-hal     | 0.44    | ESP32 hardware abstraction       |
| serde           | 1.0     | Serialisation for UDP protocol   |
| tokio           | 1.0     | Async runtime                    |
| crossbeam       | 0.5     | Thread-safe channels             |
| tracing         | 0.1     | Structured logging               |
| chrono          | 0.4     | Timestamps                       |
| ublox           | 0.9     | UBX protocol (optional, for F9P config) |
