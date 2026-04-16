# FINN Guidance — Auto-Steer Tuning Guide

A practical guide to setting up and tuning the auto-steer system in the field.

## How It Works

The steering controller uses two nested control loops to keep the tractor on the AB line.

**Outer loop — line seeking:** looks at how far you are from the AB line (cross-track error) AND what direction you're pointed relative to the line (heading error), then decides what angle the wheels should be pointing. Far from the line → large wheel angle to get back. Pointed at an angle to the line → keep turning until aligned. Close to the line AND pointed along it → straight ahead.

**Inner loop — wheel position:** compares the desired wheel angle (from the outer loop) to the actual wheel angle (from the WAS potentiometer) and drives the motor to close the gap. When the wheels reach the target angle, the motor stops. This is what makes the wheels return to straight — when you're back on the line AND pointed along it, the target angle is zero, so if the wheels are still turned, the motor straightens them.

**Why heading matters:** without heading error, the controller only knows how far off-line you are — not which way you're pointed. As the tractor approaches the line, XTE decreases and the controller straightens the wheels. But if you approached at an angle (say 15°), "straight wheels" means driving a straight line AT 15° to the AB line. You punch right through and keep going. With heading error in the loop, the controller keeps turning until you're both on the line AND pointed along it.


## Pre-Flight Checklist

Before engaging auto-steer for the first time in a session:

1. **WAS calibrated** — check the SENSORS section in Setup. You should see a calibrated angle reading (e.g. "Angle: 2.3° RIGHT"). If it says "WAS: no data" or the calibration values are missing, run the three-point calibration wizard in the WAS CALIBRATION section.

2. **Motor direction verified** — go to MOTOR TEST, press +50 PWM, and watch which way the wheels turn. Positive PWM should steer RIGHT. If it steers left, go to MOTOR DIRECTION and press "Invert Motor Direction."

3. **AB line loaded** — you need a complete AB line (A and B set at different positions, or a saved line loaded). The AUTO-STEER button will be greyed out without one.

4. **Motor ESP32 connected** — the MOTOR TEST section should show "Motor ESP32 connected" with a green dot. If not, check the USB cable to the motor ESP32.

Once all four conditions are met, the ⊕ AUTO-STEER button on the working page will become active.


## Engaging Auto-Steer

1. Drive onto or near the AB line at working speed (at least 2 km/h — the system won't steer below 0.5 m/s to avoid GPS drift steering at standstill).
2. Tap **⊕ AUTO-STEER** on the working page toolbar.
3. A green overlay appears at the top-left showing: `AUTO-STEER  PWM 42  T:-12° A:-8° H:5°`
   - **PWM** — the motor command being sent right now
   - **T** — target wheel angle (what the outer loop wants)
   - **A** — actual wheel angle (what the WAS is reading)
   - **H** — heading error (how many degrees off the line bearing you're pointed)
4. The system is now steering. Keep your hands near the wheel.
5. To disengage: tap **⊗ STEER OFF** or simply stop the tractor (speed gate cuts output below 0.5 m/s).


## Tuning the Controller

All tuning is done in the **AUTO-STEER** section of the Setup page. Changes take effect immediately — you can adjust while driving.

### Outer Loop — Kp (°/m)

This controls how aggressively the system seeks the line.

**What it does:** converts cross-track error (metres off-line) into a desired steering angle (degrees). A Kp of 30 means: 1 metre off-line → command 30° of wheel turn.

**Too low (sluggish):** the tractor drifts slowly back toward the line, taking a long arc. You'll see the XTE staying high and the target angle (T:) being small.

**Too high (aggressive/oscillating):** the tractor snaps back quickly but overshoots the line, then corrects the other way, creating a weaving pattern. You'll see the target angle swinging between positive and negative rapidly.

**Starting point:** 30 °/m. This is conservative — 1m off-line commands 30° of turn, which is well within the steering range.

**Tuning procedure:**
- Start at 30. Drive a pass and watch the XTE readout.
- If the tractor is slow to correct: increase by 5 at a time (try 35, 40, 45).
- If the tractor weaves or oscillates: decrease by 5 (try 25, 20).
- The sweet spot is where the tractor returns to the line smoothly without overshooting.

**Note for standalone GPS (no RTK):** with 1-2m position accuracy, you'll see the XTE wander even when the tractor is driving perfectly straight. The controller will chase this noise. Setting Kp lower (15-25) and accepting wider tracking tolerance is usually better than fighting the GPS wander with high gain. The deadband setting (see below) also helps with this.

### Outer Loop — Kh (°/°)

This controls how strongly the system corrects for heading misalignment with the AB line.

**What it does:** converts heading error (degrees between your heading and the line bearing) into additional desired steering angle. A Kh of 0.5 means: pointed 10° off the line bearing → add 5° of desired steering correction.

**Why it exists:** without heading error, the controller only sees cross-track distance. As XTE drops to zero on approach, it commands "straighten up" — but if you approached at an angle, straightening the wheels means driving THROUGH the line at that angle. The heading term keeps turning until you're aligned with the line, not just on it. This was the root cause of the diagonal overshoot bug in the second field test.

**Too low / zero (overshooting):** the tractor approaches the line at an angle, hits XTE=0, straightens wheels, and punches through. This is the old broken behaviour. You'll see H: staying large even as XTE passes through zero.

**Too high (oscillating on approach):** the system over-corrects the heading, swings past alignment, and starts weaving during the approach phase. You'll see H: swinging between positive and negative.

**Starting point:** 0.5 °/°.

**Tuning procedure:**
- Start at 0.5. Drive 3-5m to the side of the line, then engage.
- Watch H: as you approach. It should decrease smoothly toward 0° as the tractor aligns.
- If the tractor still punches through: increase by 0.1 at a time (try 0.6, 0.7, 0.8).
- If the approach feels wobbly or the tractor swings past alignment: decrease by 0.1.
- At Kh = 0, heading correction is disabled and you're back to pure XTE control.
- **Key diagnostic:** if H: is small (under 3°) when the overshoot happens, the problem is Kp, not Kh. If H: is large (over 5°) when the tractor crosses the line, you need more Kh.

**Interaction with Kp:** Kp and Kh work together. High Kp gets you back to the line fast; high Kh makes sure you arrive aligned. If both are too high, you'll oscillate. If you're oscillating, try reducing Kp before Kh — aggressive line-seeking with good heading alignment is better than the reverse.

### Inner Loop — Kp Angle (PWM/°)

This controls how hard the motor drives to reach the desired wheel angle.

**What it does:** converts the difference between desired and actual wheel angle into motor power. A Kp angle of 4 means: 10° of angle error → 40 PWM.

**Too low (slow wheel response):** the motor turns the wheels slowly. You'll see a persistent gap between T: and A: in the overlay — the actual angle lags behind the target.

**Too high (jerky/buzzy):** the motor slams the wheels to position and may overshoot, oscillate around the target angle, or produce an audible buzzing as it hunts.

**Starting point:** 4.0 PWM/°.

**Tuning procedure:**
- Watch the T: and A: values in the overlay while steering is engaged.
- If A: is consistently behind T: (e.g. T:-20° A:-12°): increase by 0.5 at a time.
- If the steering feels jerky or you hear the motor buzzing: decrease by 0.5.
- Ideal: A: tracks T: closely with smooth transitions.

### Max PWM

Hard ceiling on motor power output, regardless of what the control loops calculate.

**Default:** 180 (out of 255 max).

**When to change:** if the motor seems like it's working too hard (hot, noisy) reduce it. If the motor can't achieve the commanded wheel angle fast enough (persistent gap between T: and A:), you may need to increase it — but try increasing Kp angle first.

### Deadband

Minimum cross-track error (in cm) before the controller does anything. Below this, the target angle is zero (drive straight).

**Default:** 3 cm.

**With standalone GPS:** increase to 10-15 cm. GPS position wanders by 1-2 metres, and a 3 cm deadband means the controller is always active, constantly chasing noise. A larger deadband lets the tractor drive straight when it's "close enough" and only corrects for real drift.

**With RTK GPS:** 3-5 cm is appropriate since position accuracy is ±2 cm.


## Reading the Working Page Overlay

When auto-steer is engaged, the top-left overlay shows:

```
AUTO-STEER  PWM 42  T:-12° A:-8° H:5°
```

What each value tells you:

| Value | Meaning | Healthy range |
|-------|---------|---------------|
| PWM | Motor command being sent | Should swing between ±max_pwm, hover near 0 when on-line |
| T: | Target wheel angle from outer loop | Negative = steer left, positive = steer right |
| A: | Actual wheel angle from WAS | Should track T: closely |
| H: | Heading error from line bearing | Should be near 0° when tracking well, large during approach |

**Green overlay** = normal operation.
**Amber overlay** = WAS data is stale (no reading for >2 seconds). The system continues steering using the last known wheel angle. Usually a brief serial hiccup — it should recover. If it stays amber for more than a few seconds, check the sensor ESP32 USB cable.


## Safety Systems

The controller will automatically disengage and send PWM 0 if:

- **GPS fix lost** — no real GPS fix for more than 2 seconds. The amber status message will show "Auto-steer OFF: GPS fix lost". Check GPS antenna connection and sky view.

- **WAS data lost** — no wheel angle reading for more than 5 seconds. Brief dropouts (under 2 seconds) are tolerated with a warning. Extended loss means the sensor ESP32 has disconnected.

- **Manual disengage** — tap ⊗ STEER OFF at any time. This is always available, no matter what state the system is in.

- **App closed** — the ESP32 motor firmware has a 500ms watchdog. If it doesn't receive a serial command within half a second, it kills the motor. This catches crashes, USB disconnection, or any other failure where the PC stops talking.

- **Speed gate** — below 0.5 m/s (~1.8 km/h), the controller outputs zero PWM. The tractor has to be moving for auto-steer to work. This prevents GPS drift from causing random steering while parked.

After a safety disengage, you can re-engage immediately by tapping ⊕ AUTO-STEER once the condition is resolved (e.g. GPS fix reacquired).


## Troubleshooting

**Tractor drives through the line diagonally / grabs the next pass line**
This is the heading error problem. The tractor approaches at an angle, XTE hits zero, wheels straighten, but the tractor keeps going at that angle. Check H: in the overlay — if it's large (>5°) when the tractor crosses the line, you need more Kh. Increase by 0.1 at a time. If Kh is already at 0, that's the problem — it should be at least 0.3–0.5.

**"Auto-steer OFF: WAS data lost" keeps triggering**
The WAS timeout was increased to 5 seconds to handle brief serial hiccups. If it still triggers, check the USB cable from the sensor ESP32 — vibration in the tractor cab can cause intermittent connections. Try a shorter cable or secure the connection with tape.

**Tractor weaves side to side**
Outer loop Kp is too high. Reduce by 5 at a time until the oscillation stops. With standalone GPS, 15-25 is often the practical ceiling.

**Tractor is slow to return to line**
Outer loop Kp is too low. Increase by 5 at a time. Also check that max PWM isn't limiting the motor — if you see A: consistently below T:, the motor can't apply enough force.

**Motor buzzes or chatters when on-line**
Inner loop Kp angle is too high, or deadband is too small. Try reducing Kp angle by 0.5. If using standalone GPS, increase the deadband to 10-15 cm.

**Wheels turn the wrong way when engaged**
Motor direction is inverted. Immediately tap ⊗ STEER OFF, go to Setup → MOTOR DIRECTION, and press "Invert Motor Direction."

**AUTO-STEER button is greyed out**
One or more preconditions aren't met. Check: AB line loaded? Motor ESP32 connected? WAS calibrated (all three points: centre, left lock, right lock)?

**PWM shows 0 even though XTE is large**
Either the tractor is below the speed gate (0.5 m/s) or the XTE is within the deadband. Check speed in the status bar.
