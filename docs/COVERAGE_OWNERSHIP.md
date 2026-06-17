# FINN Coverage Ownership

Reviewed: 30 May 2026

This document defines which part of the FINN system is responsible for coverage
logging now, and how that responsibility moves to `finn-pilot` later.

## Current Authority

Today, `finn-guidance` is the coverage authority.

It records coverage from the tractor guidance app using:

- The tractor LC29H BA position stream.
- The active job, field, implement width, overlap, and AB-line settings.
- Manual coverage engagement in the guidance UI.
- SQLite storage under the guidance app's coverage/job database.

This is the correct Phase 1 behavior. It keeps coverage available while the
pilot stack is still a plan and while seeding-era tractor behavior is being
protected.

## Future Authority

When `finn-pilot` is built and proven, it becomes the primary coverage
authority.

That handover should happen only after `finn-pilot` can reliably provide:

- Implement-mounted position or implement-relative position.
- Shaft-speed or equivalent movement confirmation.
- Implement state, such as seeding/working/not-working.
- Time-stamped records that can be matched with tractor guidance data.
- Enough field evidence to trust it more than a manual guidance coverage
  toggle.

At that point, guidance coverage becomes the backup log, not the main coverage
truth.

## Transition Stages

| Stage | Primary coverage source | Notes |
| --- | --- | --- |
| Guidance only | `finn-guidance` | Current Phase 1 behavior. Manual coverage engagement remains the source of truth. |
| Pilot advisory | `finn-guidance` | `finn-pilot` logs implement state beside guidance coverage for comparison only. |
| Coverage assist | `finn-guidance` | Pilot state can warn the operator or suggest coverage on/off, but guidance still writes the official coverage. |
| Pilot primary | `finn-pilot` | Pilot writes official coverage from implement state. Guidance continues a backup guidance log. |

## DA, BA, And Coverage

The DA module is acceptable for implement-side pilot work where the implement
fix is combined with shaft-speed and implement state. The BA module remains
critical for the tractor auto-steer stack.

Do not let the future implement DA path blur the current tractor requirement:
tractor guidance and steering use the LC29H BA direct-to-laptop architecture.

## Handover Requirements

Before pilot-primary coverage is enabled:

- Coverage records need a source label, for example `guidance_manual`,
  `pilot_advisory`, or `pilot_primary`.
- Field run IDs need to be shared between guidance, pilot, and core.
- Clock behavior needs to be understood well enough to compare records.
- The UI needs to show which source is currently authoritative.
- Export/upload paths need to preserve both the official coverage and the
  backup guidance log.

## Open Until Field Testing

The following decisions should wait for real pilot data:

- Shaft-speed thresholds for coverage on/off.
- How much implement-state dropout is acceptable.
- Whether pilot coverage should auto-stop on low confidence.
- Whether guidance should auto-fallback when pilot data disappears.
