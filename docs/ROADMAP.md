# FINN Guidance Roadmap

Reviewed: 19 July 2026

The phased plan for FINN Guidance lives in
[IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md). This file exists so the
project matches the FINN memory-doc convention (every project carries an
ACTIVE_CONTEXT, ARCHITECTURE, DECISIONS, ROADMAP, and CORE_INTEGRATION doc) and
records where that plan currently stands. Do not duplicate phase detail here —
update the implementation plan and keep this pointer honest.

## Current Position

- Field-proven: AB line guidance, pass selection, lightbar, coverage logging,
  SQLite storage, GPS auto-detect/NMEA parsing, motor ESP32 protocol with
  watchdog-aware steering commands.
- In progress: closed-loop auto-steer work; manual roll calibration still needs
  tractor field verification (capture level, cross-slope test, confirm
  `roll_corr_m` moves XTE toward zero).
- Pending hardware inputs: physical auto-steer engage button and seed-engage
  switch for coverage logging.
- Blocked: RTK corrections need either the finn-base field unit or an NTRIP
  subscription; the rover-side NTRIP client task is not yet implemented (see
  `../../finn-base/Docs/INTEGRATION.md` §4 for the agreed integration points).
- Planned handoff: once finn-pilot is built and field-proven, pilot becomes the
  primary coverage logger and guidance coverage becomes backup (see
  `COVERAGE_OWNERSHIP.md`).

## Where Detail Lives

- Phase-by-phase plan: `IMPLEMENTATION_PLAN.md`
- Latest session state: `ACTIVE_CONTEXT.md`
- Release channel definitions: `RELEASE_CHANNELS.md`
- Workspace-level sequencing: `../../Docs/FINN_PROJECT_STATUS_AND_UNIFICATION_ROADMAP.md`
