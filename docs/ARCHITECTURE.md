# FINN Guidance Architecture

Reviewed: 19 July 2026

FINN Guidance is the tractor/cab guidance and auto-steer project: a Rust
PC/tablet application plus an ESP32 motor controller. It is the most mature and
field-proven FINN project, so its architecture is the reference the other
projects adapt around — Core, Base, Pilot, Interface, and the website integrate
with what guidance has actually proven in the tractor.

This document describes the project-local architecture. For the whole FINN map,
see `../../Docs/ARCHITECTURE.md`. For wiring and hardware detail, see
`HARDWARE_ARCHITECTURE.md`.

## Component Model

```text
        FINN Base                          FINN Core
      NTRIP / RTCM3                   field-run uploads,
  (rover client planned,              analysis tasks
   see finn-base                            ^
   Docs/INTEGRATION.md §4)                  |
            |                               |
            v                               |
   PC / tablet Rust app  ------> scripts/upload_field_run.py
   - LC29H BA GGA/PQTMINS over USB serial
   - egui field view, AB lines, pass selection, lightbar
   - SQLite: jobs, fields, AB lines, config, coverage
   - position tracking and interpolation
   - steering target computed at a fixed rate
            |
            v  FINNSTEER commands over USB serial
   Motor ESP32 (firmware-motor-pio)
   - wheel angle sensor on ADC
   - IBT-2 / BTS7960 H-bridge, 24V steering motor
   - closes the inner steering loop
   - enforces motor watchdog safety
```

The maintained tractor hardware direction is LC29H BA direct to the laptop plus
one motor ESP32. The older separate sensor ESP32 path
(`firmware-sensor-pio`) is retained only as reference material.

## Code Layout

- `common/` — shared Rust types, coordinate math, serial protocol.
- `pc/src/gps/` — GPS serial reader and NMEA parser.
- `pc/src/guidance/` — AB line, pass selection, steering logic.
- `pc/src/gui/` — egui application and field view.
- `pc/src/comms/` — serial comms with the motor ESP32.
- `pc/src/coverage/` — SQLite coverage/job/field storage.
- `pc/src/position/` — position tracking and interpolation.
- `pc/src/telemetry/` — steering telemetry logs (`.jsonl`).
- `firmware-motor-pio/` — maintained ESP32 motor controller firmware.
- `scripts/upload_field_run.py` — field-run registration into FINN Core.

## Safety Boundaries

- The tractor is BA-critical: the LC29H BA plus motor ESP32 remain the local
  safety-critical steering path. Nothing networked sits in the steering loop.
- Guidance must stay safe and usable without Core, Base, Pilot, Interface, or
  the website (offline-first rule).
- Loss of RTK corrections is a normal state, not an error: RTK FIX degrades to
  FLOAT to autonomous without interrupting steering.
- Pilot data enters only through the receiver gate specified in
  `../../finn-pilot/docs/RECEIVER_CONTRACT.md` — schema, freshness, sequence,
  clamp, decay, local primacy, logging. Default mode is advisory; authority
  only widens by explicit operator action on the tractor PC.
- Coverage logging is currently owned by guidance (`guidance_manual`). Once
  finn-pilot is field-proven it becomes the primary coverage authority and
  guidance logging becomes the backup path (`guidance_backup`).

## Data Products

- Steering telemetry: `logs/steer_*.jsonl`, designed for FINN Core analysis.
- Coverage/jobs/fields/AB lines: `data/coverage.db` (SQLite).
- Field-run records: created in Core by `scripts/upload_field_run.py`, which
  summarises the latest telemetry and coverage and can create analysis tasks.

## Related Docs

- `ACTIVE_CONTEXT.md` — latest field-test state and next work.
- `DECISIONS.md` — decision log.
- `IMPLEMENTATION_PLAN.md` — phased plan (see also `ROADMAP.md` pointer).
- `HARDWARE_ARCHITECTURE.md` — current hardware architecture.
- `COVERAGE_OWNERSHIP.md` — coverage authority split with finn-pilot.
- `CORE_INTEGRATION.md` — integration into finn-core.
- `RELEASE_CHANNELS.md` — field prototype / lightbar / auto-steer channels.
