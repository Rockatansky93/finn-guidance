# FINN Guidance — Active Context

> **Purpose**: This file is the first thing any new AI session should read.
> It captures the current state of work, recent decisions, and what to do next.
> Updated at the end of each working session.

## Last updated
Session 10 — 1 April 2026

## What we're working on
**Phase 3 is COMPLETE.** Phase 1 and Phase 2 were completed in earlier sessions.
All three guidance-display phases are done and field-verified. The system is a
fully functional GPS guidance display with AB line management, auto-pass selection,
coverage logging, configurable lightbar, and persistent settings.

Session 10 covered:
- Bugfixes for egui 0.29 API changes and smart-quote corruption (DECISIONS.md #012)
- Field-verified AB line save/load and nudge (completed Phase 2 sign-off)
- Configuration persistence: implement width, overlap, lightbar sensitivity, and
  last-loaded AB line all saved to SQLite and restored on startup (#013)
- Coverage data management: zoom-dependent render thinning, 100k-point memory cap
  with oldest-half downsample, and "🗑 Clear Coverage" button (#014)
- Job management UI: JOB HISTORY section with list/delete
- Lightbar sensitivity UI: LIGHTBAR section in Setup page (±1 cm/seg, 1–50 range)

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

## What's blocked
- **Field laptop**: original laptop couldn't fill the role. Two Dell Latitude 7390
  2-in-1 units (i5 8350U, 8GB, 256GB SSD, touchscreen) ordered — still in the post.
  Blocks tractor cab field testing.
- **RTK**: no base station or NTRIP subscription yet. Running standalone GPS
  (HDOP 0.4 with 42–50 sats — usable for guidance display, not centimetre-accurate)

## Next session should
1. Review this file and `DECISIONS.md` for context
2. **Field test on Dell 2-in-1** when laptops arrive — install Rust toolchain,
   build, verify touchscreen interaction, test full workflow in tractor cab
   (lightbar readability, touch targets, auto-pass, nudge, save/load, overlap)
3. **Phase 4 planning** — ESP32 steering controller. Review hardware shopping list,
   plan wiring, set up ESP Rust toolchain. This is a fundamentally different kind
   of work (hardware + firmware) compared to Phases 1–3.
4. Consider adding area calculation (hectares from coverage points) before moving
   to Phase 4 — useful for the operator and relatively simple to implement

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
common/src/protocol.rs       — UDP message types (for ESP32 comms, Phase 4)
docs/IMPLEMENTATION_PLAN.md  — full phase plan, task tracking, session log
docs/DECISIONS.md            — architectural decision log (#001–#014)
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
