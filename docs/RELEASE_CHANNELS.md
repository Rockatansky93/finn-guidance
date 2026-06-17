# FINN Guidance Release Channels

Reviewed: 30 May 2026

FINN Guidance needs separate release channels because the same repo serves a
working field tractor, a public lightbar product, and experimental auto-steer
development.

## Field Prototype / Internal

Purpose: protect the working tractor during seeding and controlled internal
testing.

Hardware:

- LC29H BA connected directly to the field laptop.
- Motor ESP32 running `firmware-motor-pio`.
- Wheel angle sensor connected to the motor ESP32.
- IBT-2 / BTS7960 steering motor driver.

Rules:

- Keep a known-good field laptop build available for rollback.
- Avoid behavior changes unless they fix a field problem or are needed for a
  controlled validation run.
- Enable telemetry for validation runs where practical, especially roll
  calibration and cross-slope checks.
- Do not make telemetry globally default-on until the delayed field test
  confirms the operator impact and log volume are acceptable.

As of 30 May 2026, the field laptop is intentionally still on the old working
code while the newer Phase 1 code is available from GitHub.

## Public Lightbar

Purpose: guidance-only public release with a simpler support surface.

Hardware:

- Laptop/tablet running the guidance app.
- Direct USB serial GNSS receiver.
- No steering motor.
- No motor ESP32 required.

Rules:

- Hide or clearly disable auto-steer controls when no motor ESP32 is present.
- Keep setup focused on field/job/AB-line/lightbar/coverage workflows.
- Do not require BA-specific auto-steer assumptions for guidance-only use.
- Release with a simple install guide, known limitations, and a tagged build.

## Auto-Steer Experimental

Purpose: controlled closed-loop steering development for operators who
understand the hardware and risk.

Hardware:

- LC29H BA direct to the laptop.
- Motor ESP32 direct to the laptop.
- Wheel angle sensor, steering motor, and motor driver installed and tested.

Rules:

- Treat BA as required for the maintained tractor steering stack.
- Require steering bench tests before field use.
- Require WAS calibration before engagement.
- Require manual roll calibration before cross-slope evaluation.
- Keep telemetry enabled for release candidates and field validation.
- Document rollback steps beside every experimental build.

## Pilot / Implement Experimental

Purpose: implement-side sensing and future coverage authority.

Hardware:

- `finn-pilot` hardware as it is proven.
- DA module is acceptable for implement fix work.
- Shaft-speed and implement-state sensors become the coverage truth only after
  field validation.

Rules:

- Pilot coverage starts as advisory.
- Guidance remains the primary coverage log until pilot-primary coverage is
  proven.
- Guidance keeps backup coverage logging after pilot-primary coverage is
  enabled.

## Minimum Release Checklist

- Build from a clean checkout.
- Record the git commit or tag.
- Confirm the target channel in the release notes.
- Check that docs match the channel hardware.
- Run a smoke test on the target machine.
- For auto-steer channels, complete bench safety checks before field use.
- For field prototype and auto-steer experimental channels, record whether
  telemetry is enabled.
