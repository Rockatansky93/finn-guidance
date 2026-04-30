# DR Remediation Strategy

> **Status:** Probe complete. Findings recorded in §Probe Results. Ready to
> execute Steps 1-6.
> **Date:** 30 April 2026 (drafted), 30 April 2026 (probe results added)
> **Owner:** Tom + Claude (this session)
> **Triggering issue:** Roll compensation read zero in the field (Jamestown,
> sown paddock) despite the GUI claiming the LC29H BA's onboard DR was wired
> in. Lateral drift on slope made guidance effectively useless.

## TL;DR

The DR integration in `pc/src/gps/reader.rs` and `pc/src/gps/parser.rs` was
written against documentation that does not match the LC29H BA's actual
protocol. Both the enable command and the `$PQTMINS` field offsets are wrong.
The module has also never been calibrated, which on its own would have
prevented DR from working even if the code were correct. The probe phase has
confirmed which firmware variant is in use and validated that PQTMINS streams
correctly once enabled with the right command. The hardware is sound; the
remediation is now a code fix plus a calibration drive.

## Probe Results (30 April 2026)

Three probe runs completed against the dev laptop bench setup, then a fourth
run outside under clear sky to obtain a real GNSS fix. Findings:

### Firmware identified

```
$PQTMVERNO,LC29HBANR11A02S_CSA2,2023/05/09,20:49:45
```

LC29H BA, firmware family `NR11`, revision `A02S`, variant suffix `_CSA2`,
built May 2023. The variant suffix isn't human-readable but the firmware
behaviour confirms it's a **two-wheel software build** (PQTMCFGEINSMSG
accepted, PQTMINS streamed once enabled).

### Command syntax confirmed

The corrected command form works exactly as the spec describes:

```
-> $PQTMCFGEINSMSG,1,1,0,0,10*3F
<- $PQTMCFGEINSMSG,OK*3A
```

The Get-form reply on this firmware uses the truncated `$PQTMEINSMSG` shape
rather than `$PQTMCFGEINSMSG`. Probe handles both. NVS save and persistence
across power cycles confirmed working: a clean power-cycle produced a module
that came up already streaming PQTMINS at 10Hz with no further commands
needed.

### Data path validated outdoors

- 150 PQTMINS samples in 15s, clean 10Hz, no drops.
- `SolType=1` ("DR not ready, GNSS+roll/pitch+rel-heading") once GNSS
  acquired 13+ satellites.
- Heading reported correctly (matched VTG-derived heading).
- Lat/lon reported correctly (Jamestown coordinates, ~7m from GGA).
- **Roll and pitch reported as 0.00 across every single sample.**

### IMU hardware verified alive

With PQTMIMU additionally enabled, raw IMU data streams cleanly:

```
$PQTMIMU,7707,8.312666,4.599005,2.206182,1.496264,0.274824,0.916080,0,0
```

Accelerometer magnitude √(ACC_X² + ACC_Y² + ACC_Z²) ≈ 9.75 m/s² across all
samples, distributed across all three axes (module wasn't level — fine). Gyro
rates were small but non-zero on a hand-held module, consistent with
ordinary noise plus minor hand motion. **The IMU sensor is healthy and
reporting correctly at 10Hz.**

### Conclusion: roll-zero is a calibration issue, not a hardware or code issue

The IMU hardware works. The firmware exposes both fused (PQTMINS) and raw
(PQTMIMU) streams. The module simply refuses to populate the Roll, Pitch,
and absolute Heading fields of PQTMINS until DR calibration has at least
started — which requires the module to see real motion (>2 m/s, with
turning) per spec §2.1.3. None of these BA modules has ever been calibrated;
today in the paddock was effectively their first real outing, and they
weren't given a calibration drive before guidance was engaged.

This means the lateral drift in the hilly paddock had two causes operating
simultaneously:

1. **The roll compensation code path was reading the wrong fields** out of
   PQTMINS (Bug 3 below) — even with a calibrated module producing real
   roll values, the parser would have been reading Pitch into Roll and
   Heading into Pitch.
2. **The module had no roll values to provide anyway** because
   calibration had never been performed.

Fixing only #1 will leave guidance reading clean zeros indefinitely. Fixing
only #2 will leave guidance reading garbage attitude values. **Both fixes
are required.** Steps 2-3 (code fixes) are the prerequisite for Step 5
(calibration drive) to produce useful output.

### Bonus finding: PQTMIMU enables independently

PQTMIMU also worked when enabled via PQTMCFGEINSMSG. We've confirmed both
streams can run simultaneously at 10Hz with no bandwidth issues. The Step
2 fix will enable both, even though Step 3 only parses PQTMINS — the raw
IMU data costs ~700 bytes/sec extra (negligible at 115200 baud) and gives
future features access to gyro/acc data without requiring a re-config.

### Behavioural notes worth preserving

- **`$PAIR007` (hot start) trashes the GNSS fix indoors.** Even though
  it's supposed to retain almanac/ephemeris, post-reset the module took
  longer than 15 seconds to reacquire indoors. In `reader.rs`, prefer
  power-cycle-on-launch semantics over runtime resets where possible.
  If we ever need to reset at runtime, budget at least 30s for
  reacquisition outdoors and accept that DR will be temporarily
  unavailable.
- **PQTMINS at SolType=0 reports lat/lon/height as exactly
  `0.00000000`.** The position fields are only populated at
  SolType≥1. `parser.rs` must therefore continue to source position
  from GGA (which the existing code does), not from PQTMINS. PQTMINS
  is the **attitude** source, not a position source.
- **`$PAIR535` and `$PQTMTXT` appeared occasionally.** Not in either
  spec; appear to be internal status. Ignore.

## How we got here

The existing `reader.rs` references "Decision #026" and includes detailed
comments about `$PQTMINS` being the DR-fused output. None of the field offsets,
parameter forms, or message-name conventions in those comments come from the
authoritative Quectel documentation:

- *Quectel LC29H Series & LC79H (AL) GNSS Protocol Specification v1.3*
- *Quectel LC29H (BA,CA,DA,EA) DR & RTK Application Note v1.1*

Both documents are now archived under `data/` for offline reference. The
existing implementation appears to have been written from training-data
recollection rather than the real spec.

The bench-sniffer run (30 April 2026) confirmed the symptoms:

- Module is alive, emitting `$GNGGA`, `$GNVTG`, etc., plus `$PQTMDRCAL` at 1Hz.
- `$PQTMDRCAL` reports `CalState=0` (uncalibrated) and `NavType=1` (GNSS only).
- Sending the enable command from `reader.rs` produced no `$PQTMINS` output.
- The reply we got (`$PQTMEINSMSG,0,0,0,0,0`) is consistent with the module
  silently rejecting a malformed Set and echoing back its current
  (all-disabled) state.

## What is actually wrong

Six independent bugs, in roughly increasing scope of fix:

### Bug 1 — `PQTMCFGEINSMSG` first parameter is wrong

Spec §3.1.9 defines:

```
$PQTMCFGEINSMSG,<Type>,<INS_Enabled>,<IMU_Enabled>,<GPS_Enabled>,<Rate>
```

with `<Type>` numeric (`0`=Get, `1`=Set). `reader.rs` sends the literal string
`W` for `<Type>`, which is neither value. The module rejects the write and
emits a non-standard echo. **Fix:** send `1` instead of `W` for Set, `0`
for Get.

### Bug 2 — Success / failure response shapes are non-standard

Spec §3.1.9 documents:

```
//Set OK:    $PQTMCFGEINSMSGOK*16
//Set fail:  $PQTMCFGEINSMSGERROR*4A
```

These are concatenated, not the usual `$PQTM<NAME>,OK*<cs>` form. Any
response parser added to `reader.rs` needs to handle both shapes. The probe
tool already does.

### Bug 3 — `$PQTMINS` field offsets are completely wrong

Spec §3.1.6 defines `$PQTMINS` as 11 fields after the header:

| Index | Field |
|---|---|
| 1 | Timestamp |
| 2 | SolType |
| 3 | Lat |
| 4 | Lon |
| 5 | Height |
| 6 | VEL_N |
| 7 | VEL_E |
| 8 | VEL_D |
| 9 | Roll |
| 10 | Pitch |
| 11 | Heading |

`parser.rs` reads `nav_type=parts[3]` (which is `Lat`), `Speed2D=parts[8]`
(which is `VEL_D`), `Roll=parts[10]` (which is `Pitch`), `Pitch=parts[11]`
(which is `Heading`), `Heading=parts[12]` (which doesn't exist). Every single
DR field is offset by one or more positions. There is no scalar speed in
`$PQTMINS` at all — it has to be derived from VEL_N/VEL_E (`sqrt(N² + E²)`).

### Bug 4 — `SolType` enum mapping is wrong

Spec §3.1.6 defines:

```
0 = DR not ready. Roll and pitch ready.
1 = DR not ready. GNSS, roll, pitch, and relative heading ready.
2 = GNSS + DR mode. DR calibrated.
3 = DR only mode.
```

`parser.rs` treats `SolType=0` as "no solution" and bails before reading roll
or pitch. But per the spec, **`SolType=0` is the state where roll and pitch
are valid** (just heading isn't). Skipping the parse on 0 throws away the
only useful pre-cal attitude data. Should bail only when the roll/pitch
fields are themselves empty.

### Bug 5 — Master enable for `$PQTMDRCAL` was never sent by us

Per spec §3.2.1, `$PAIR6010` controls a separate set of DR-telemetry sentences
(`PQTMVEHMSG`, `PQTMSENMSG`, `PQTMDRCAL`, `PQTMIMUTYPE`, `PQTMVEHMOT`). The
module is currently emitting `$PQTMDRCAL` because someone enabled it
historically (most likely via QGNSS during early bench setup) and the setting
was saved to NVS. `reader.rs` doesn't issue this command at all. If the
module is ever restored or replaced, `$PQTMDRCAL` will silently disappear and
the GUI's calibration-state indicator will go permanently dead. The fix needs
to assert this enable on every startup (it's idempotent and cheap).

### Bug 6 — Configuration takes effect only after reset

Spec §3.1.9 ends with:

> Send `$PQTMSAVEPAR*5A` and reset the module for `$PQTMCFGEINSMSG` to take
> effect.

`reader.rs` sends save but never resets. Even with the correct command form,
the new config wouldn't activate until the next power cycle. In practice this
means a fresh-from-factory module would never produce DR data on its first
session — only after the operator power-cycled the laptop. Easy to miss, easy
to misdiagnose. The fix is to issue `$PAIR007` (hot start) after save, then
wait ~4 seconds before reading.

## The variant question

Both `$PQTMINS` (§3.1.6) and `$PQTMCFGEINSMSG` (§3.1.9) carry an explicit
note in the spec:

> This message is only supported by LC29H (BA) and LC29H (CA) with software
> versions dedicated for **two-wheel** vehicles. Contact Quectel Technical
> Support for details about the software versions.

The four-wheel firmware build does not expose `$PQTMINS`. It uses
`$PQTMDRPVA` (§3.1.14) instead, which is the same shape (timestamp,
SolType, lat/lon/alt, velocities, roll/pitch/heading) but with a UTC time
field, geoidal separation, and a slightly different SolType enum:

```
0 = no fix
1 = GNSS only
2 = combined (GNSS + DR)
3 = DR only
```

Note this is **not** the same enum as PQTMINS (which uses `0` to mean "roll
and pitch ready, DR not ready"). The probe tool decodes both correctly.

We don't yet know which variant is on Tom's two BA modules. We will find out
by running `data/lc29h_probe.py` and reading the `$PQTMVERNO` reply.
Behaviour-wise:

- If the module accepts `PQTMCFGEINSMSG,1,...` and starts emitting
  `$PQTMINS` after reset → two-wheel firmware. Use the PQTMINS path.
- If `PQTMCFGEINSMSG` is rejected even with correct syntax → four-wheel
  firmware. Use the `$PQTMDRPVA` path with `$PQTMCFGDR,W,1` (§3.1.16)
  to enable DR.
- If the firmware is genuinely incapable, fall back to the existing VTG
  heading and the GUI's roll display permanently reads zero (which is what
  it does now).

For tractor / agricultural use, the four-wheel firmware is the more
appropriate variant — but both will work, since neither needs the wheel-tick
input that ADR uses (we'll be in UDR mode either way per §1.1).

## Plan

Strict ordering. Don't skip ahead — each step's verification is the input
to the next step.

### Step 0 — Probe the module

Run `data/lc29h_probe.py --port COM3` with both BA modules in turn. Capture:

- Firmware version string from `$PQTMVERNO`
- Current `PQTMCFGEINSMSG` get response (or absence of one)
- Whether `$PQTMINS` or `$PQTMDRPVA` appears after the corrected enable + reset

This tells us which path to take in Step 2. **Estimated time: 10 minutes.**

### Step 1 — Record the findings as Decision #028

A new entry in `docs/DECISIONS.md` covering: which firmware variant is in use,
which DR sentence is the source of truth for attitude going forward, and
whether the existing "Decision #026" comment block in `reader.rs` is being
superseded (it is). This locks in the architectural choice before we touch
code, so a future session doesn't second-guess it.

### Step 2 — Fix `pc/src/gps/reader.rs`

Edits, in this order, all via `codesnip:edit_snippet`:

1. Replace the `PQTMCFGEINSMSG` send with the correct numeric `<Type>=1`
   form. Confirmed working with `$PQTMCFGEINSMSG,1,1,1,0,10` (INS+IMU
   enabled, GPS off, 10Hz).
2. Add an idempotent `$PAIR6010,2,1` to assert `$PQTMDRCAL` on every
   startup, so the GUI's cal-state indicator can never go permanently dead.
3. Issue `$PQTMSAVEPAR` to persist. **Do not issue `$PAIR007` (hot start)
   at runtime.** The probe confirmed it kills the GNSS fix and recovery
   takes longer than typical session boundaries. Instead: rely on the
   fact that the saved config takes effect on the next power cycle, which
   is also when `reader.rs` itself starts. The first session after this
   fix lands will not have DR; every subsequent session will. Document
   this clearly in the user-facing notes.
4. Add ack-handling for the non-standard `$PQTMCFGEINSMSGOK` /
   `$PQTMCFGEINSMSGERROR` reply shapes (and the bare `$PQTMEINSMSG`
   form returned by the get-query) so we surface failures in the log
   instead of silently proceeding.
5. Verify by checking on startup whether PQTMINS is already streaming
   (skip the config write if so — keeps cold-boot logs clean and avoids
   unnecessary NVS writes that wear flash).

After this step, the module should be emitting PQTMINS and PQTMIMU at
boot. Verify with the probe before touching the parser.

### Step 3 — Fix `pc/src/gps/parser.rs`

Four changes:

1. Replace the `$PQTMINS` field-offset constants with the correct ones from
   §3.1.6:
   ```
   parts[1]  = Timestamp
   parts[2]  = SolType
   parts[3]  = Lat       (ignore — use GGA's instead)
   parts[4]  = Lon       (ignore — use GGA's instead)
   parts[5]  = Height    (ignore — use GGA's altitude instead)
   parts[6]  = VEL_N
   parts[7]  = VEL_E
   parts[8]  = VEL_D
   parts[9]  = Roll      ← this is what we actually want
   parts[10] = Pitch     ← and this
   parts[11] = Heading   ← and this
   ```
2. Fix the SolType handling. The correct enum (PQTMINS variant per
   §3.1.6) is:
   ```
   0 = DR not ready, roll/pitch ready (only — no heading yet)
   1 = DR not ready, GNSS+roll/pitch+rel-heading ready
   2 = GNSS+DR (calibrated)
   3 = DR only
   ```
   Don't bail on SolType=0 — read roll/pitch but skip heading. Bail only
   when the roll/pitch fields themselves are empty strings.
3. Source scalar speed from `sqrt(VEL_N² + VEL_E²)` rather than reading a
   non-existent `Speed2D` field. (The existing VTG-fallback path can stay
   as a safety net.)
4. **Source position from GGA only**, never from PQTMINS. The probe
   confirmed PQTMINS reports lat/lon as 0.00000000 at SolType=0, and the
   non-zero values it reports at SolType=1+ are derived from GNSS anyway,
   so GGA is the canonical source. This was already the case in the
   existing parser; just don't accidentally introduce a position read
   from PQTMINS during the field-offset edit.

Unit-test with sample sentences from the spec (§3.1.6 example) and from
the probe's outdoor capture
(`$PQTMINS,775312,1,-33.27565284,138.59023261,414.533028,0.378698,-0.717217,0.024944,0.00,0.00,336.60`).

### Step 4 — Bench validation

Re-run the probe and confirm:

- PQTMINS and PQTMIMU both stream at 10Hz from boot (no manual enable
  needed because the saved config persists from prior probe runs).
- The GUI shows non-zero roll reading **after a calibration drive**.
  Before calibration, GUI roll will still read zero — this is
  expected and will be misleading on the bench. Don't chase it.
- Coverage logging still works (no regression).
- Log output shows the correct ack messages from the new
  config-handling code.

Note that bench validation cannot prove roll compensation is *correct*,
only that it's wired through. Verifying correctness needs Step 5.

### Step 5 — Field calibration drive

This is the actual fix for the in-paddock symptom. The module has
never been calibrated, and PQTMINS will continue reporting roll=0,
pitch=0 indefinitely until calibration runs at least once.

Procedure per §2.1.3:

1. Module rigidly fixed to the vehicle (no flex, no relative motion).
   The ute is fine for this; the module's mounting orientation is
   learned during calibration so we don't need to pre-configure it.
2. Good GNSS, clear sky.
3. Drive >2 m/s, perform 3–4 turning movements.
4. ~3 minutes of varied driving.
5. Watch for `$PQTMDRCAL` `CalState` → 2.

The finn-guidance GUI's calibration-state indicator should reflect
DRCAL state once Steps 2-3 land — that's the operator-facing signal
that calibration is complete.

Once calibrated, the module persists calibration to NVS automatically
per §2.1.3 (no separate save command needed for calibration data,
unlike command-driven config). However, per §3.1.15 (PQTMCFGDRHOT)
the DR-hot-start feature governs whether calibration survives power
cycles. Default behaviour for this firmware family appears to be that
calibration *does* persist, but if Step 6 reveals re-calibration
required after every cold boot, we may need to explicitly enable
DR hot start mode 1.

Best done on a quiet bit of road with no implement attached. The
tractor itself can also be used, but the ute is faster and the
module isn't fussy about which vehicle calibrates it as long as
the mounting is rigid.

### Step 6 — In-paddock validation

Repeat the conditions where the bug was first observed: drive the same
hilly paddock with guidance engaged. Confirm:

- Roll reading is non-zero on slope.
- Lateral drift between adjacent passes is reduced vs the previous run.
- AB-line tracking holds across the worst topography.

If this fails, we need to revisit either the calibration drive or the
roll-correction lever-arm logic (`apply_offset` in `parser.rs` looks
mathematically right but hasn't been verified against ground truth).

## Risks and gotchas

**Bricking from a bad save.** `$PQTMSAVEPAR` writes to NVS. If we ever
write a config that prevents the module from booting cleanly we'd need the
QGNSS tool to recover. Mitigation: the corrected commands match the spec
exactly and have been validated end-to-end by the probe.
`$PQTMRESTOREPAR` (§3.1.5) is the documented recovery command;
it's available from the probe if needed.

**Two modules diverging.** Only the first BA module has been probed.
The second module may be on a different firmware revision. Probe both
before Step 2 to confirm. If they differ, `reader.rs` may need to detect
variant at runtime — but most likely they're identical builds from the
same batch.

**Calibration loss on cold start.** Per §3.1.15, `PQTMCFGDRHOT` controls
whether DR cal persists across power cycles. Default appears to be enabled
on this firmware (mode 1) but we haven't explicitly verified. Re-check
during Step 6 — if calibration is lost on every cold boot, set mode 1
explicitly.

**The lever-arm correction in `parser.rs::apply_offset` is unverified.**
Even once we get correct roll values flowing through, we don't actually
know if the antenna-height shift math is right. The convention ("positive
roll = right side down" → shift bearing = heading + 90°) is consistent
with the spec's Figure 2 module orientation, but only field testing on a
known slope will confirm sign and magnitude. If Step 6 still shows
lateral drift on slopes, this is the next thing to look at. The
convention can be verified empirically by parking the tractor across a
slope and comparing the GUI roll reading to a phone level.

**The roll EMA smoothing (alpha=0.15) was tuned without real data.** The
existing parser smooths raw roll into `smoothed_roll` with a 0.15 alpha
(~1s settling at 10Hz). This was a reasonable guess but never validated
against real cab vibration. Worth revisiting after Step 6 if roll
readings look either too noisy (raise alpha) or too laggy on slope
transitions (lower alpha).

## Out of scope for this remediation

- Adding wheel-tick input via `$PQTMVEHMSG` (would upgrade UDR → ADR for
  better DR-only performance during GNSS dropouts, e.g. under tree cover).
  Worth doing later, requires a hardware tap into the tractor's speed
  signal, not blocking current work.
- Switching to RTK. The BA supports it (§2.2) but we're in UDR-without-RTK
  for now. Roll correction works the same regardless of RTK status.
- Rewriting `Decision #026` block in `reader.rs`. The DECISIONS.md update
  is the source of truth; the inline comments will be replaced naturally
  during the Step 2 edits.

## References

- Quectel LC29H Series & LC79H (AL) GNSS Protocol Specification v1.3 (uploaded)
- Quectel LC29H (BA,CA,DA,EA) DR & RTK Application Note v1.1
  (`data/Quectel_LC29HBACADAEA_DRRTK_Application_Note_V1_1.pdf`)
- Probe script: `data/lc29h_probe.py`
- Original sniffer (kept for run comparison): `data/lc29h_sniffer.py`
