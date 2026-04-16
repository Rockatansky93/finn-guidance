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
//! Unchanged from the previous design. The inner loop takes whatever
//! desired angle the outer loop commands and drives the motor to hit it:
//!
//! ```text
//!   angle_error = desired_angle - actual_angle
//!   pwm = clamp(Kp_angle × angle_error, ±max_pwm)
//!   if |pwm| > 0: pwm = sign(pwm) × max(|pwm|, min_pwm)  // stall-torque boost
//! ```
//!
//! An angle deadband (default 2°) prevents hunting around the target. The
//! min_pwm floor (default 100) is there because the Trimble EZ-Steer
//! direct-drive motor stalls below that PWM against hydraulic steering
//! resistance. The deadband makes sure the floor doesn't produce bang-bang.
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

    /// XTE deadband in metres. When the tractor is within this of the line
    /// AND heading error is small, the outer loop commands straight wheels
    /// instead of trying to correct micro-offsets (which GPS noise would
    /// drive the motor to chase).
    pub deadband_m: f64,

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
        self.engaged = true;
        true
    }

    /// Disengage auto-steer with an optional reason.
    pub fn disengage(&mut self, reason: Option<String>) {
        self.engaged = false;
        self.last_output_pwm = 0;
        self.last_desired_angle = 0.0;
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
        //
        // Sign: XTE positive = right of line. To pull back to the line, the
        // lookahead point must be to the LEFT of the tractor (bearing < 0).
        // We use -xte_m in the atan2 so positive XTE yields negative alpha.
        let psi_rad = heading_error_deg.to_radians();
        let alpha_line_frame = (-xte_m).atan2(lookahead_m);
        let alpha_rad = alpha_line_frame - psi_rad;

        // Deadband: if both XTE and heading are inside the deadband, command
        // straight wheels. Prevents GPS-noise-driven micro-corrections from
        // chasing the motor when we're already on the line.
        let desired_angle = if xte_m.abs() < self.deadband_m
            && heading_error_deg.abs() < 2.0
        {
            0.0
        } else {
            // Bicycle model: wheel angle that curves the vehicle through
            // the lookahead point.
            //   delta = atan2(2 · L_w · sin(alpha), L)
            let delta_rad = (2.0 * self.wheelbase_m * alpha_rad.sin())
                .atan2(lookahead_m);
            let delta_deg = delta_rad.to_degrees();
            delta_deg.clamp(-self.max_steer_angle, self.max_steer_angle)
        };

        self.last_desired_angle = desired_angle;

        // === Inner loop: angle error → motor PWM ===
        let actual = actual_angle_deg.unwrap_or(0.0);
        self.last_actual_angle = actual;

        let angle_error = desired_angle - actual;

        // Angle deadband — prevents bang-bang hunting around the target.
        if angle_error.abs() < self.angle_deadband_deg {
            self.last_output_pwm = 0;
            return (0, false);
        }

        let raw_pwm = self.kp_angle * angle_error;
        let clamped = (raw_pwm.round() as i16).clamp(-self.max_pwm, self.max_pwm);

        // Motor stall torque compensation: non-zero output gets boosted to
        // at least min_pwm so the direct-drive motor actually moves against
        // hydraulic steering resistance.
        let output = if clamped == 0 {
            0
        } else if clamped > 0 {
            clamped.max(self.min_pwm)
        } else {
            clamped.min(-self.min_pwm)
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
        c
    }

    #[test]
    fn test_on_line_aligned_commands_straight() {
        // XTE = 0, heading error = 0 → desired angle = 0 → no motor output.
        let mut c = engaged_controller();
        c.angle_deadband_deg = 2.0;
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
        let (_pwm, _dis) = c.compute(1.0, 0.0, 2.0, Some(0.0));
        assert!(c.last_desired_angle < 0.0,
            "Right of line should command left steering, got {}",
            c.last_desired_angle);
    }

    #[test]
    fn test_left_of_line_steers_right() {
        let mut c = engaged_controller();
        let (_pwm, _dis) = c.compute(-1.0, 0.0, 2.0, Some(0.0));
        assert!(c.last_desired_angle > 0.0,
            "Left of line should command right steering, got {}",
            c.last_desired_angle);
    }

    #[test]
    fn test_heading_error_alone_commands_correction() {
        // On the line but pointed right of the bearing → should steer left.
        let mut c = engaged_controller();
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
        let (pwm, _dis) = c.compute(2.0, 10.0, 0.1, Some(0.0));
        assert_eq!(pwm, 0, "Below min speed should output 0");
    }
}
