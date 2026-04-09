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
