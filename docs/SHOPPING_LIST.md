# FINN Guidance Hardware Shopping List

Updated: 22 July 2026

This list covers the maintained tractor stack: an LC29H BA connected directly
to the field laptop, plus one motor ESP32 that reads the wheel-angle sensor and
drives the steering motor through an IBT-2/BTS7960 stage.

Seeding is active and the installed field build is working. Inventory and
photograph the current tractor before buying replacements or changing wiring.
This list is a purchasing and spares scaffold, not an approved steering,
vehicle-electrical, mechanical, or functional-safety design.

## Maintained Baseline

```text
LC29H BA -> USB -> field laptop -> USB -> motor ESP32
                                      -> wheel-angle sensor input
                                      -> IBT-2/BTS7960 -> steering motor
```

- Use LC29H BA for tractor steering. DA is for lower-authority implement roles;
  BS belongs to the base station.
- Do not add the retired sensor ESP32/BNO055 path to a maintained tractor build.
- Retain manual disengage, the PC GPS/speed gates, motor watchdog, wheel-angle
  limits, current protection, and operator supervision.

## Audit Before Ordering

- [ ] Photograph every installed module, connector, fuse, mount, and cable run.
- [ ] Record laptop model, power adapter, ports, operating system, and sunlight
  visibility.
- [ ] Record LC29H BA carrier revision, firmware, antenna, connector, cable, and
  USB device identity.
- [ ] Record ESP32 board revision, firmware build, USB connector, and pin map.
- [ ] Record wheel-angle sensor type, supply, output range, centre/lock values,
  linkage, and mechanical travel.
- [ ] Measure steering motor free/running/stall current and supply transients.
- [ ] Confirm IBT-2/BTS7960 source, current/thermal performance, mounting, and
  whether a more robust field driver is required.
- [ ] Map fuse sizes, wire gauges, grounds, disconnects, USB retention, ingress,
  heat, vibration, and chafe points.

## A. Bench/Replacement Kit for One Tractor

Quantities are target inventory. Buy only what the audit shows is missing.

| Qty | Item | Minimum requirement | Notes |
| ---: | --- | --- | --- |
| 1 | Field laptop/tablet | Runs the release build, two reliable USB data ports, daylight-readable, secured mount | Keep the current working unit if adequate. |
| 1 | Quectel LC29H BA carrier | BA module with direct USB serial and documented PQTMINS output | Do not substitute DA or BS. |
| 1 | Active multi-band GNSS antenna | Compatible with carrier bias/bands; rigid tractor mount | Cable and connector must match. |
| 1 | Motor ESP32 DevKit | Matches the maintained firmware and pin map | Keep one flashed spare after validation. |
| 1 | Wheel-angle sensor | Stable, repeatable output wholly within the ESP32 ADC limit after the approved interface | Mechanical linkage must not bind at lock. |
| 1 | Steering motor/gearmotor | Mechanically compatible; adequate torque/speed; measured current documented | Guard pinch points and preserve manual control. |
| 1 | IBT-2/BTS7960 stage or reviewed replacement | Correct logic interface, voltage/current margin, heat sinking and protection | Bench modules vary; validate the actual unit. |
| 2 | Short shielded USB data cables | Correct connectors, positive strain relief, vibration-resistant routing | One installed, one known-good spare per critical link where practical. |
| 1 | Current-limited bench supply | Covers logic and controlled motor tests with visible current limit | Use a restrained steering test rig. |
| 1 | Emergency/manual disengage test control | Removes motor authority independently of the PC software | Exact vehicle implementation requires review. |
| 1 set | Fuses, terminals, wire, ferrules, labels | Sized from measured current and automotive environment | No unfused battery feed. |

## B. Tractor Installation Hardware

| Qty | Item | Requirement |
| ---: | --- | --- |
| 1 | Rigid GNSS antenna mount | Known position, clear sky, repeatable orientation, protected coax route |
| 1 | Laptop mount | Crash-conscious retention, no obstruction of controls/view, vibration managed |
| 1 | Electronics enclosure | Secured, serviceable, protected from dust, moisture, heat, vibration, and loose metal |
| 1 | Fused DC power input | Vehicle-rated transient/reverse-polarity protection and labelled disconnect |
| As required | Isolated/regulated DC converters | Stable laptop and logic rails with load/transient margin |
| 1 | Steering branch fuse/disconnect | Sized from measured motor current and cable rating; accessible to operator |
| 1 | Manual disengage path | Positive removal of drive authority with a tested mechanical/manual fallback |
| 1 set | Automotive harness | Abrasion sleeve, sealed connectors where needed, strain relief, service loops, labels |
| 1 | Motor/driver thermal arrangement | Heat sink, airflow, temperature check, no combustible contact |
| 1 set | Guards and mechanical stops | Protect motor/linkage pinch points and prevent electrical command beyond safe travel |

## C. Recommended Spares and Test Equipment

| Qty | Item | Requirement |
| ---: | --- | --- |
| 1 | Flashed motor ESP32 | Same validated firmware/configuration; clearly labelled spare |
| 1 | Known-good USB cable per type | Data-tested, not charge-only |
| 1 | Pre-imaged laptop SSD or recovery drive | Versioned release, configuration backup, offline recovery notes |
| 1 set | Fuses and critical connectors | Exact installed values and contact tooling |
| 1 | Digital multimeter | Suitable category and automotive probes |
| 1 | DC clamp meter | Range and resolution suitable for steering motor current |
| 1 | USB/serial diagnostic kit | Lets BA and motor telemetry be checked independently |

## Do Not Buy for the Maintained Tractor Path

- LC29H DA as a replacement for the tractor BA.
- LC29H BS for rover use.
- A second sensor ESP32 or BNO055 to recreate the retired architecture.
- Higher-current steering hardware before measuring the existing motor and
  driver under controlled conditions.
- Any part whose addition bypasses watchdog, angle limits, manual disengage, or
  operator authority.

## Suggested Purchasing Sequence

1. Audit and photograph the working tractor; record models and measured values.
2. Buy only known-good cables, fuses, recovery media, and one critical spare.
3. Recreate the maintained stack on a current-limited restrained bench if a
   replacement component must be qualified.
4. Complete wheel-angle, direction, watchdog, disengage, thermal, and power-loss
   checks before installation.
5. Make field changes one at a time with immediate rollback available.

## Purchase Record Template

| Project item | Manufacturer/model | Supplier | Qty | Unit cost | Voltage/current | Firmware/config | Bench result | Tractor/date | Approved by |
| --- | --- | --- | ---: | ---: | --- | --- | --- | --- | --- |
| Example: motor ESP32 spare | TBD | TBD | 1 | TBD | TBD | TBD | Not tested | Not installed | TBD |

## Official Reference

- [Quectel LC29H Series GNSS specification](https://www.quectel.com/content/uploads/2024/03/Quectel_LC29H_Series_GNSS_Specification_V1.7.pdf)

Read `HARDWARE_ARCHITECTURE.md`, `INSTALLATION_GUIDE.md`, and
`STEERING_TUNING_GUIDE.md` before changing the tractor.
