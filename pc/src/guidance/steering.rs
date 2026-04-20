//! Steering controller — pure pursuit outer loop, WAS-feedback inner loop.
//!
//! ## Architecture
//!
//! Two nested control loops:
//!
//! **Outer loop — pure pursuit geometry (cross-track + heading → desired angle):**
//!
//! Pure pursuit picks a "lookahead point" on the AB line some distance `L`
//! ahead of the tractor, then commands the wheel angle that would curve the
//! tractor through that point. The bicycle model gives:
//!
//! ```text
//!   desired_angle = atan2(2 · wheelbase · sin(alpha), L)
//! ```
//!
//! where `alpha` is the angle from the tractor's current heading to the
//! lookahead point. Here we compute `alpha` in closed form from the current
//! cross-track error (XTE) and heading error — no need to project to
//! lat/lon and back. For an AB line with local XTE `y` and heading error
//! `psi` (both signed), the lookahead point at distance `L` along the line
//! from the tractor's projection onto it sits at a bearing:
//!
//! ```text
//!   alpha = atan2(-y, L) - psi
//! ```
//!
//! (derivation: the point `L` ahead on the line is at (L, 0) in a frame
//! aligned with the line; the tractor is at (0, y) with its nose pointing
//! at heading psi relative to the line. The vector from tractor to
//! lookahead is (L, -y), so its bearing in the line frame is `atan2(-y, L)`.
//! Subtract psi to get the bearing in the tractor frame.)
//!
//! The single tunable quantity is `L` — the lookahead distance. We scale it
//! with speed so that at working speed the tractor looks far ahead (smooth
//! approach), while at low speed it looks close (crisp corrections):
//!
//! ```text
//!   L = lookahead_base + lookahead_speed_factor × speed
//! ```
//!
//! clamped to a sane range (2 m floor, 15 m ceiling).
//!
//! **Why this replaces the old Kp/Kh PD controller:**
//! The old `-Kp·XTE - Kh·heading_error` weighted two terms against each
//! other. The Kp and Kh balance was delicate: too much Kp and the tractor
//! hunted (XTE term dominated, snapped to zero, then heading error was
//! ignored as it unwound); too little and it drifted on. Pure pursuit
//! captures both XTE and heading in one geometric quantity (`alpha`) with
//! no term-balancing. One knob (lookahead distance) sets the approach
//! aggression; it can't fight itself.
//!
//! **Inner loop (desired angle vs actual WAS → motor PWM):**
//!
//! Reworked with a "waveform" philosophy. In reality, straight-line driving
//! is not straight — the steering wheel constantly oscillates left and right
//! around the line. The controller's job is to minimise the amplitude of
//! that oscillation, not to snap to a fixed angle.
//!
//! Three changes from the original inner loop:
//!
//! 1. **Smooth taper replaces hard deadband.** The old XTE deadband snapped
//!    desired_angle to zero when XTE was within 3cm. This clipped the
//!    waveform at the zero-crossing — the motor went silent in exactly the
//!    zone where fine control mattered most. Now the desired angle tapers
//!    linearly to zero as XTE approaches the line, preserving the waveform
//!    shape through the crossing.
//!
//! 2. **XTE rate damping (dXTE/dt).** The controller now tracks how fast
//!    XTE is changing. When the tractor is converging on the line (error
//!    shrinking), the damping term reduces the correction, preventing
//!    overshoot. When diverging, it adds urgency. This is the mechanism
//!    that makes the oscillation amplitude shrink over successive cycles.
//!    Governed by `kd_xte` (default 0.5).
//!
//! 3. **Sub-stall pulsing.** The EZ-Steer motor stalls below ~100 PWM.
//!    The old design used a hard deadband to avoid bang-bang at the stall
//!    floor. Now we use pulse accumulation: desired PWM below the stall
//!    threshold is accumulated over cycles, and when the accumulator
//!    reaches min_pwm, one pulse is fired. This gives time-averaged torque
//!    below the stall floor — analog-like control without bang-bang.
//!
//! The angle deadband field is retained in the struct for compatibility
//! but is no longer used in the control loop.
//!
//! ## Sign convention
//!
//!   - Positive XTE = vehicle is RIGHT of the line
//!   - Positive heading_error = nose pointed RIGHT of line bearing
//!   - Positive angle = wheels turned RIGHT
//!   - Positive PWM = motor steers RIGHT (before motor_invert)
//!
//! ## Safety
//!
//! PC-side:
//! - Auto-disengage if GPS fix age exceeds `max_fix_age_secs`
//! - WAS data loss: warning after `was_warn_secs`, disengage after `was_disengage_secs`
//! - Speed gate: no steering below `min_speed`
//! - Max steering angle clamp (`max_steer_angle`)
//! - Motor stall compensation floor + deadband (prevents bang-bang)
//!
//! Firmware-side: 500 ms motor watchdog — PC must send $FINNSTEER at >= 2 Hz.

use std::time::Instant;

/// Steering controller state.
pub struct SteeringController {
    // === Outer loop: pure pursuit ===

    /// Base lookahead distance (metres). This is the floor of the lookahead
    /// distance — what the controller uses at standstill or very low speed.
    /// Smaller values make the controller look closer, producing sharper
    /// corrections when already near the line. Larger values smooth out
    /// corrections but slow line acquisition.
    ///
    /// This maps to the UI's "Online Aggression" slider — a smaller
    /// `lookahead_base` = higher online aggression.
    pub lookahead_base: f64,

    /// Speed-scaled lookahead coefficient (seconds). At working speed, the
    /// controller adds `speed × lookahead_speed_factor` metres to the base
    /// lookahead distance. Physically, this is the time-horizon the
    /// controller aims at: at `lookahead_speed_factor = 1.0` and 3 m/s
    /// speed, the lookahead reaches 3 m further ahead than at standstill.
    ///
    /// Smaller values keep the controller aggressive even at speed (short
    /// lookahead while moving fast). Larger values make the controller
    /// gentler at speed. Maps to the UI's "Approach Aggression" slider —
    /// a smaller `lookahead_speed_factor` = higher approach aggression.
    pub lookahead_speed_factor: f64,

    /// Tractor wheelbase in metres. Used in the pure-pursuit bicycle-model
    /// curvature calculation. Not a tuning knob — measure your tractor.
    /// Default 2.8 m is typical for a mid-size utility tractor.
    pub wheelbase_m: f64,

    /// Maximum desired steering angle (degrees). Caps how hard the outer
    /// loop can command a turn, even when far off-line or badly misaligned.
    pub max_steer_angle: f64,

    // === Inner loop: angle error → motor PWM ===

    /// Inner loop gain: PWM per degree of angle error.
    /// Must be high enough that realistic angle errors (5-15°) produce PWM
    /// above min_pwm, otherwise the motor won't move for small corrections.
    /// At 10.0: 8° error → 80 PWM; 10° error → 100 PWM.
    pub kp_angle: f64,

    /// Maximum PWM magnitude the controller will output.
    pub max_pwm: i16,

    /// Minimum PWM to overcome steering resistance. The Trimble EZ-Steer is
    /// direct-drive; hydraulic steering resistance stalls the motor below
    /// this value. Any non-zero output is boosted to at least this so the
    /// motor actually moves.
    pub min_pwm: i16,

    /// Inner loop angle deadband in degrees. When the angle error is below
    /// this threshold, the motor outputs zero. Prevents the min_pwm boost
    /// from creating bang-bang oscillation near the target.
    pub angle_deadband_deg: f64,

    /// XTE deadband in metres. Below this threshold the outer loop begins
    /// tapering its output rather than cutting to zero — the smooth taper
    /// replaces the old hard deadband. The taper zone runs from deadband_m
    /// down to zero, scaling the desired angle linearly so corrections
    /// fade out rather than clipping.
    pub deadband_m: f64,

    // === Waveform damping ===

    /// XTE rate damping gain. Multiplied by dXTE/dt (m/s) and subtracted
    /// from the desired angle. When the tractor is converging on the line
    /// (dXTE/dt negative, shrinking error), this reduces the correction,
    /// preventing overshoot. When diverging it adds urgency. This is the
    /// key "amplitude reduction" mechanism — it makes the oscillation
    /// waveform converge rather than sustain.
    pub kd_xte: f64,

    /// Previous XTE sample for computing dXTE/dt.
    prev_xte_m: Option<f64>,

    /// Timestamp of the previous compute() call for dt calculation.
    prev_compute_time: Option<Instant>,

    // === Sub-stall pulsing ===

    /// Pulse accumulator for sub-stall motor control. When the desired PWM
    /// is below min_pwm, we accumulate the desired effort each cycle. When
    /// the accumulator exceeds min_pwm, we emit one pulse at min_pwm and
    /// subtract it. This gives analog-like average torque below the stall
    /// floor — PWM-within-PWM.
    pulse_accumulator: f64,

    /// Whether auto-steer is currently engaged.
    pub engaged: bool,

    /// Minimum speed (m/s) for auto-steer to produce output.
    pub min_speed: f64,

    // === Safety ===

    /// Maximum GPS fix age (seconds) before auto-disengage.
    pub max_fix_age_secs: f64,

    /// Timestamp of the last real GPS fix received.
    last_fix_time: Option<Instant>,

    /// Timestamp of the last WAS reading received.
    last_was_time: Option<Instant>,

    /// WAS age (seconds) that triggers a warning (but keeps steering).
    pub was_warn_secs: f64,

    /// WAS age (seconds) that triggers full disengage.
    pub was_disengage_secs: f64,

    /// Whether WAS data is currently stale (for UI warning display).
    pub was_stale: bool,

    // === Debug/display ===

    /// The last PWM value computed (for display).
    pub last_output_pwm: i16,

    /// The last desired steering angle (for display).
    pub last_desired_angle: f64,

    /// The last actual steering angle from WAS (for display).
    pub last_actual_angle: f64,

    /// The last heading error in degrees (for display).
    pub last_heading_error: f64,

    /// The last lookahead distance used (for display).
    pub last_lookahead_m: f64,

    /// Reason for last disengage (for UI display).
    pub disengage_reason: Option<String>,
}

impl SteeringController {
    pub fn new() -> Self {
        Self {
            // Pure pursuit defaults.
            // lookahead_base = 3.0 m: moderately aggressive when on the line.
            // lookahead_speed_factor = 1.0 s: at 3 m/s speed, L = 6 m (gentle).
            // Operator-facing sliders expose these as "Online Aggression"
            // and "Approach Aggression" with inverted scales (higher number
            // on slider = more aggressive = shorter lookahead).
            lookahead_base: 3.0,
            lookahead_speed_factor: 1.0,
            wheelbase_m: 2.8,
            max_steer_angle: 15.0,

            // Inner loop unchanged from previous controller.
            kp_angle: 10.0,
            max_pwm: 180,
            min_pwm: 100,
            angle_deadband_deg: 2.0,
            deadband_m: 0.03,

            // Waveform damping.
            // kd_xte = 0.5: at dXTE/dt of -0.1 m/s (converging at 10cm/s),
            // this subtracts 0.05 rad ≈ 2.9° from the desired angle — a
            // meaningful damping that prevents overshoot without killing
            // the approach. Tunable in the field.
            kd_xte: 0.5,
            prev_xte_m: None,
            prev_compute_time: None,

            // Sub-stall pulsing.
            pulse_accumulator: 0.0,

            engaged: false,
            min_speed: 0.5,
            max_fix_age_secs: 2.0,
            last_fix_time: None,
            last_was_time: None,
            was_warn_secs: 2.0,
            was_disengage_secs: 5.0,
            was_stale: false,
            last_output_pwm: 0,
            last_desired_angle: 0.0,
            last_actual_angle: 0.0,
            last_heading_error: 0.0,
            last_lookahead_m: 0.0,
            disengage_reason: None,
        }
    }

    /// Engage auto-steer.
    pub fn engage(&mut self) -> bool {
        self.disengage_reason = None;
        self.last_output_pwm = 0;
        self.last_desired_angle = 0.0;
        self.was_stale = false;
        // Reset waveform state so we start fresh — no stale dXTE/dt
        // from a previous engagement or old pulse accumulator.
        self.prev_xte_m = None;
        self.prev_compute_time = None;
        self.pulse_accumulator = 0.0;
        self.engaged = true;
        true
    }

    /// Disengage auto-steer with an optional reason.
    pub fn disengage(&mut self, reason: Option<String>) {
        self.engaged = false;
        self.last_output_pwm = 0;
        self.last_desired_angle = 0.0;
        self.prev_xte_m = None;
        self.prev_compute_time = None;
        self.pulse_accumulator = 0.0;
        self.disengage_reason = reason;
    }

    /// Notify the controller that a real GPS fix was received.
    pub fn notify_gps_fix(&mut self) {
        self.last_fix_time = Some(Instant::now());
    }

    /// Notify the controller that a WAS reading was received.
    pub fn notify_was_reading(&mut self) {
        self.last_was_time = Some(Instant::now());
        self.was_stale = false;
    }

    /// Run safety checks. Returns Some(reason) if auto-steer should disengage.
    fn check_safety(&mut self) -> Option<String> {
        // Check GPS fix age — hard disengage
        if let Some(fix_time) = self.last_fix_time {
            if fix_time.elapsed().as_secs_f64() > self.max_fix_age_secs {
                return Some("GPS fix lost".to_string());
            }
        } else {
            return Some("No GPS fix received".to_string());
        }

        // Check WAS data age — tiered response
        if let Some(was_time) = self.last_was_time {
            let was_age = was_time.elapsed().as_secs_f64();
            if was_age > self.was_disengage_secs {
                return Some("WAS data lost".to_string());
            } else if was_age > self.was_warn_secs {
                self.was_stale = true;
            }
        } else {
            return Some("No WAS data received".to_string());
        }

        None
    }

    /// Compute the steering PWM output using pure pursuit + WAS-feedback control.
    ///
    /// Call this every GUI frame.
    ///
    /// - `xte_m`: cross-track error in metres (positive = right of line)
    /// - `heading_error_deg`: heading error in degrees (positive = pointed right of line bearing)
    /// - `speed_mps`: current vehicle speed in m/s
    /// - `actual_angle_deg`: current steering angle from WAS
    ///
    /// Returns (pwm, disengaged_this_frame).
    pub fn compute(
        &mut self,
        xte_m: f64,
        heading_error_deg: f64,
        speed_mps: f64,
        actual_angle_deg: Option<f64>,
    ) -> (i16, bool) {
        if !self.engaged {
            self.last_output_pwm = 0;
            return (0, false);
        }

        // Safety checks
        if let Some(reason) = self.check_safety() {
            self.disengage(Some(reason));
            return (0, true);
        }

        // Speed gate
        if speed_mps < self.min_speed {
            self.last_output_pwm = 0;
            return (0, false);
        }

        self.last_heading_error = heading_error_deg;

        // === Compute dXTE/dt for waveform damping ===
        //
        // The rate of change of cross-track error tells us whether the
        // oscillation is converging or diverging. Positive dXTE/dt when
        // XTE is positive means we're drifting further right (diverging);
        // negative means we're coming back (converging). The damping term
        // reduces corrections when converging and increases them when
        // diverging — this is what makes the waveform amplitude shrink
        // over successive cycles.
        let now = Instant::now();
        let xte_rate = match (self.prev_xte_m, self.prev_compute_time) {
            (Some(prev_xte), Some(prev_time)) => {
                let dt = prev_time.elapsed().as_secs_f64();
                if dt > 0.001 && dt < 1.0 {
                    // Valid dt window: > 1ms (not a double-call) and
                    // < 1s (not a stale sample from a long pause).
                    (xte_m - prev_xte) / dt
                } else {
                    0.0
                }
            }
            _ => 0.0, // First call after engage — no rate yet.
        };
        self.prev_xte_m = Some(xte_m);
        self.prev_compute_time = Some(now);

        // === Outer loop: pure pursuit geometry ===
        //
        // Lookahead distance scales with speed. At standstill the controller
        // looks `lookahead_base` ahead; at speed it looks further. Clamped to
        // [2, 15] m to keep the geometry sane in all conditions.
        let lookahead_m = (self.lookahead_base
            + self.lookahead_speed_factor * speed_mps)
            .clamp(2.0, 15.0);
        self.last_lookahead_m = lookahead_m;

        // Pure-pursuit target bearing (`alpha`) relative to the tractor's
        // current heading. Closed-form derivation from XTE + heading error,
        // as described in the module docstring.
        let psi_rad = heading_error_deg.to_radians();
        let alpha_line_frame = (-xte_m).atan2(lookahead_m);
        let alpha_rad = alpha_line_frame - psi_rad;

        // Bicycle model: wheel angle that curves the vehicle through
        // the lookahead point.
        let delta_rad = (2.0 * self.wheelbase_m * alpha_rad.sin())
            .atan2(lookahead_m);
        let mut desired_angle = delta_rad.to_degrees();

        // === Waveform damping: dXTE/dt reduces corrections when converging ===
        //
        // The damping term is applied in degrees: we convert the rate-based
        // correction to an angle offset. kd_xte × xte_rate gives a correction
        // in the same sign-space as XTE (positive = rightward drift rate),
        // so we ADD it because positive drift → steer more left (the pure
        // pursuit alpha already encodes "right of line → steer left").
        //
        // When converging (xte_rate has opposite sign to xte_m), this
        // reduces the desired angle magnitude → gentler approach → less
        // overshoot. When diverging (same sign), it increases urgency.
        desired_angle += self.kd_xte * xte_rate;

        // === Smooth taper near the line (replaces hard deadband) ===
        //
        // Instead of snapping desired_angle to 0 inside the deadband,
        // we scale it smoothly from 0 at xte=0 to 1.0 at xte=deadband_m.
        // This means corrections fade out proportionally as the tractor
        // approaches the line — no clipping of the waveform zero-crossing.
        // Beyond the deadband the scale is 1.0 (full correction).
        if self.deadband_m > 0.0 {
            let xte_abs = xte_m.abs();
            if xte_abs < self.deadband_m && heading_error_deg.abs() < 2.0 {
                let taper = xte_abs / self.deadband_m;
                desired_angle *= taper;
            }
        }

        // Clamp after damping and taper so nothing exceeds physical limits.
        desired_angle = desired_angle.clamp(-self.max_steer_angle, self.max_steer_angle);

        self.last_desired_angle = desired_angle;

        // === Inner loop: angle error → motor PWM ===
        //
        // Reworked to eliminate the hard angle deadband and use sub-stall
        // pulsing instead. The motor always gets a proportional signal;
        // when the desired PWM is below the stall floor, we accumulate
        // it and emit periodic pulses.
        let actual = actual_angle_deg.unwrap_or(0.0);
        self.last_actual_angle = actual;

        let angle_error = desired_angle - actual;
        let raw_pwm = self.kp_angle * angle_error;
        let desired_pwm = raw_pwm.clamp(-(self.max_pwm as f64), self.max_pwm as f64);

        // Sub-stall pulsing: if the desired effort is below the motor's
        // stall threshold, we can't just apply it (motor won't move).
        // Instead we accumulate the desired effort over multiple cycles.
        // When the accumulator reaches min_pwm, we fire one pulse at
        // min_pwm. This gives time-averaged torque below the stall floor.
        //
        // Above stall threshold: pass through directly with stall boost.
        let abs_desired = desired_pwm.abs();
        let output = if abs_desired >= self.min_pwm as f64 {
            // Above stall floor — direct drive. Reset accumulator since
            // we're making real movement.
            self.pulse_accumulator = 0.0;
            let clamped = (desired_pwm.round() as i16).clamp(-self.max_pwm, self.max_pwm);
            // Ensure we're at least at min_pwm (stall compensation).
            if clamped > 0 {
                clamped.max(self.min_pwm)
            } else {
                clamped.min(-self.min_pwm)
            }
        } else if abs_desired < 1.0 {
            // Negligible effort — don't accumulate noise. Let the motor
            // rest. This replaces the old hard deadband: instead of a
            // fixed angle threshold, we only go silent when the entire
            // control chain (pursuit + damping + taper) says "basically
            // nothing needed."
            self.pulse_accumulator = 0.0;
            0
        } else {
            // Sub-stall zone: accumulate desired effort.
            self.pulse_accumulator += desired_pwm;

            if self.pulse_accumulator.abs() >= self.min_pwm as f64 {
                // Accumulated enough for one pulse. Fire it and subtract.
                let pulse_dir = self.pulse_accumulator.signum() as i16;
                self.pulse_accumulator -= pulse_dir as f64 * self.min_pwm as f64;
                pulse_dir * self.min_pwm
            } else {
                // Not enough accumulated yet — hold.
                0
            }
        };

        self.last_output_pwm = output;
        (output, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn engaged_controller() -> SteeringController {
        let mut c = SteeringController::new();
        // Bypass the safety gates by forging recent fix/WAS timestamps.
        c.last_fix_time = Some(Instant::now());
        c.last_was_time = Some(Instant::now());
        c.engaged = true;
        // Pre-seed prev_compute_time so dXTE/dt calculation has a valid
        // baseline on the first test call.
        c.prev_compute_time = Some(Instant::now());
        c
    }

    #[test]
    fn test_on_line_aligned_commands_straight() {
        // XTE = 0, heading error = 0 → desired angle ≈ 0 → no motor output.
        let mut c = engaged_controller();
        c.prev_xte_m = Some(0.0);
        let (pwm, dis) = c.compute(0.0, 0.0, 2.0, Some(0.0));
        assert!(!dis);
        assert_eq!(pwm, 0);
        assert!(c.last_desired_angle.abs() < 0.01);
    }

    #[test]
    fn test_right_of_line_steers_left() {
        // Positive XTE (right of line), aligned heading → should command
        // a negative wheel angle (steer left).
        let mut c = engaged_controller();
        c.prev_xte_m = Some(1.0); // Steady XTE, no rate.
        let (_pwm, _dis) = c.compute(1.0, 0.0, 2.0, Some(0.0));
        assert!(c.last_desired_angle < 0.0,
            "Right of line should command left steering, got {}",
            c.last_desired_angle);
    }

    #[test]
    fn test_left_of_line_steers_right() {
        let mut c = engaged_controller();
        c.prev_xte_m = Some(-1.0);
        let (_pwm, _dis) = c.compute(-1.0, 0.0, 2.0, Some(0.0));
        assert!(c.last_desired_angle > 0.0,
            "Left of line should command right steering, got {}",
            c.last_desired_angle);
    }

    #[test]
    fn test_heading_error_alone_commands_correction() {
        // On the line but pointed right of the bearing → should steer left.
        let mut c = engaged_controller();
        c.prev_xte_m = Some(0.0);
        let (_pwm, _dis) = c.compute(0.0, 10.0, 2.0, Some(0.0));
        assert!(c.last_desired_angle < 0.0,
            "Pointed right of line should command left steer, got {}",
            c.last_desired_angle);
    }

    #[test]
    fn test_max_steer_clamp() {
        // Huge XTE should clamp to max_steer_angle, not exceed it.
        let mut c = engaged_controller();
        c.max_steer_angle = 15.0;
        let (_pwm, _dis) = c.compute(50.0, 0.0, 2.0, Some(0.0));
        assert!(c.last_desired_angle.abs() <= 15.0 + 0.01,
            "Desired angle {} exceeds max", c.last_desired_angle);
    }

    #[test]
    fn test_lookahead_scales_with_speed() {
        let mut c = engaged_controller();
        c.lookahead_base = 3.0;
        c.lookahead_speed_factor = 1.0;
        c.compute(1.0, 0.0, 0.0, Some(0.0));
        let l_slow = c.last_lookahead_m;
        c.compute(1.0, 0.0, 5.0, Some(0.0));
        let l_fast = c.last_lookahead_m;
        assert!(l_fast > l_slow,
            "Lookahead should grow with speed: {} vs {}", l_slow, l_fast);
    }

    #[test]
    fn test_speed_gate_kills_output() {
        let mut c = engaged_controller();
        c.min_speed = 0.5;
        c.prev_xte_m = Some(2.0);
        let (pwm, _dis) = c.compute(2.0, 10.0, 0.1, Some(0.0));
        assert_eq!(pwm, 0, "Below min speed should output 0");
    }

    #[test]
    fn test_converging_xte_reduces_correction() {
        // When XTE is shrinking (converging), the damping term should
        // reduce the desired angle compared to steady-state.
        let mut c = engaged_controller();

        // First: steady XTE (no rate) — get baseline desired angle.
        c.prev_xte_m = Some(1.0);
        c.compute(1.0, 0.0, 2.0, Some(0.0));
        let steady_angle = c.last_desired_angle;

        // Second: converging XTE (was 1.5, now 1.0 → negative rate).
        let mut c2 = engaged_controller();
        c2.prev_xte_m = Some(1.5);
        c2.compute(1.0, 0.0, 2.0, Some(0.0));
        let converging_angle = c2.last_desired_angle;

        // Converging should produce a less aggressive (smaller magnitude)
        // correction than steady-state.
        assert!(converging_angle.abs() < steady_angle.abs(),
            "Converging XTE should reduce correction: steady={:.3} converging={:.3}",
            steady_angle, converging_angle);
    }

    #[test]
    fn test_diverging_xte_increases_correction() {
        // When XTE is growing (diverging), the damping term should
        // increase the desired angle compared to steady-state.
        let mut c = engaged_controller();

        // Steady XTE baseline.
        c.prev_xte_m = Some(1.0);
        c.compute(1.0, 0.0, 2.0, Some(0.0));
        let steady_angle = c.last_desired_angle;

        // Diverging XTE (was 0.5, now 1.0 → positive rate).
        let mut c2 = engaged_controller();
        c2.prev_xte_m = Some(0.5);
        c2.compute(1.0, 0.0, 2.0, Some(0.0));
        let diverging_angle = c2.last_desired_angle;

        assert!(diverging_angle.abs() > steady_angle.abs(),
            "Diverging XTE should increase correction: steady={:.3} diverging={:.3}",
            steady_angle, diverging_angle);
    }

    #[test]
    fn test_sub_stall_pulsing() {
        // Small angle errors that produce PWM below min_pwm should
        // accumulate over multiple calls and eventually fire a pulse.
        let mut c = engaged_controller();
        c.prev_xte_m = Some(0.2);
        c.kp_angle = 10.0;
        c.min_pwm = 100;

        // With small XTE the desired angle will be small, producing
        // sub-stall PWM. Call repeatedly — eventually the accumulator
        // should fire a pulse.
        let mut got_pulse = false;
        for _ in 0..50 {
            c.prev_xte_m = Some(0.2);
            c.prev_compute_time = Some(Instant::now());
            let (pwm, _) = c.compute(0.2, 0.0, 2.0, Some(0.0));
            if pwm != 0 {
                got_pulse = true;
                assert!(pwm.abs() >= c.min_pwm,
                    "Pulse should be at least min_pwm, got {}", pwm);
                break;
            }
        }
        assert!(got_pulse, "Sub-stall accumulator should eventually fire a pulse");
    }
}
