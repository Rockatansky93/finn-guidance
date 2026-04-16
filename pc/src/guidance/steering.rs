//! Steering controller — two-loop architecture for auto-steer.
//!
//! ## Architecture
//!
//! Two nested control loops:
//!
//! **Outer loop (GPS → desired steering angle):**
//!   desired_angle = clamp(-kp_xte × xte_m + kh × heading_error, ±max_steer_angle)
//!
//!   This converts cross-track error (how far off the line we are) AND
//!   heading error (how far off the line bearing we're pointed) into a
//!   desired wheel angle. The XTE term drives toward the line; the heading
//!   term ensures the tractor arrives *aligned* with it, not at an angle.
//!
//!   Without heading error: as XTE→0 the controller commands "straighten
//!   up", but if the tractor approached at an angle it's still pointed
//!   diagonally. The wheels straighten, the tractor drives through the
//!   line and keeps going — potentially overshooting into the next pass.
//!   Traditional systems with this bug produce a weaving wave pattern;
//!   ours was overshooting so far it grabbed the next AB line and drove
//!   perpendicular.
//!
//!   With heading error: the controller keeps the wheels turned until the
//!   tractor is both close to the line AND pointed along it. This is
//!   standard in commercial guidance systems.
//!
//! **Inner loop (desired angle vs actual WAS → motor PWM):**
//!   angle_error = desired_angle - actual_angle
//!   pwm = clamp(kp_angle × angle_error, ±max_pwm)
//!
//!   This drives the motor to achieve the desired wheel position. It uses
//!   the WAS (wheel angle sensor) as feedback, so it knows when the wheels
//!   have reached the target angle and stops driving. This is what makes
//!   the wheels return to straight — when XTE is zero AND heading error
//!   is zero, desired_angle is zero, and the inner loop straightens up.
//!
//! ## Sign convention
//!
//!   - Positive XTE = vehicle is RIGHT of the line
//!   - Positive angle = wheels turned RIGHT
//!   - Positive PWM = motor steers RIGHT (before motor_invert)
//!   - So: positive XTE → negative desired angle (steer LEFT to correct)
//!
//! ## Safety
//!
//! The ESP32 motor firmware has a 500ms watchdog — if it doesn't receive a
//! `$FINNSTEER` command within 500ms, it kills the motor. The GUI sends
//! commands at ~20Hz (throttled in app.rs), keeping the watchdog fed.
//!
//! PC-side safety:
//! - Auto-disengage if GPS fix age exceeds `max_fix_age_secs`
//! - WAS data loss: warning after `was_warn_secs`, disengage after `was_disengage_secs`
//! - PWM clamped to `max_pwm` (never sends full 255 unless configured to)
//! - Motor deadzone: output boosted to `min_pwm` (default 80) if non-zero,
//!   because the Trimble EZ-Steer doesn't spin below ~80 PWM
//! - Speed gate: no steering below minimum speed

use std::time::Instant;

/// Steering controller state.
pub struct SteeringController {
    // === Outer loop: XTE → desired steering angle ===

    /// Outer loop gain: degrees of desired steering angle per metre of XTE.
    /// Higher = more aggressive line-seeking.
    /// 30.0 means 1m off-line → command 30° of steering.
    pub kp: f64,

    /// Heading error gain: degrees of desired steering angle per degree of
    /// heading error. This is the key term that prevents diagonal approach.
    /// Without it, the controller straightens the wheels when XTE→0 but
    /// the tractor is still pointed at an angle to the line, causing it to
    /// drive through and overshoot. With it, the controller keeps turning
    /// until the tractor is both on the line AND pointed along it.
    /// 0.5 means 10° off the line bearing → add 5° of desired steering.
    /// Range: 0.0 (disabled, pure XTE) to ~1.5 (very heading-aggressive).
    pub kh: f64,

    /// Maximum desired steering angle (degrees). Caps how hard the outer
    /// loop can command a turn, even when far off-line.
    pub max_steer_angle: f64,

    // === Inner loop: angle error → motor PWM ===

    /// Inner loop gain: PWM per degree of angle error.
    /// Controls how aggressively the motor drives to reach the desired angle.
    /// 4.0 means 10° error → 40 PWM.
    pub kp_angle: f64,

    /// Maximum PWM magnitude the controller will output.
    pub max_pwm: i16,

    /// Minimum PWM to actually move the motor. Below this the Trimble
    /// EZ-Steer motor doesn't spin — the controller was computing PWM
    /// values in the 0–79 dead zone where nothing happened, causing
    /// delayed corrections and lurching. Any non-zero output is boosted
    /// to at least this value.
    pub min_pwm: i16,

    /// Deadband in metres. XTE below this produces zero desired angle.
    /// Prevents the motor from hunting when already on the line.
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
    /// Uses the last known angle — the inner loop still works.
    pub was_warn_secs: f64,

    /// WAS age (seconds) that triggers full disengage.
    /// If we truly lose the sensor for this long, stop steering.
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

    /// Reason for last disengage (for UI display).
    pub disengage_reason: Option<String>,
}

impl SteeringController {
    pub fn new() -> Self {
        Self {
            kp: 30.0,
            kh: 0.5,
            max_steer_angle: 25.0,
            kp_angle: 4.0,
            max_pwm: 180,
            min_pwm: 80,
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
                // Truly lost — disengage
                return Some("WAS data lost".to_string());
            } else if was_age > self.was_warn_secs {
                // Stale but not lost — flag warning, keep steering with last known angle
                self.was_stale = true;
            }
        } else {
            return Some("No WAS data received".to_string());
        }

        None
    }

    /// Compute the steering PWM output using two-loop control.
    ///
    /// Call this every GUI frame.
    ///
    /// `xte_m`: cross-track error in metres (positive = right of line)
    /// `heading_error_deg`: heading error in degrees (positive = pointed right of line bearing)
    /// `speed_mps`: current vehicle speed in m/s
    /// `actual_angle_deg`: current steering angle from WAS (negative = left, positive = right).
    ///                     Pass None if WAS is not calibrated (shouldn't happen if engage
    ///                     preconditions are met, but handled gracefully).
    ///
    /// Returns (pwm, disengaged_this_frame)
    pub fn compute(&mut self, xte_m: f64, heading_error_deg: f64, speed_mps: f64, actual_angle_deg: Option<f64>) -> (i16, bool) {
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

        // === Outer loop: XTE + heading error → desired steering angle ===
        //
        // Two terms work together:
        //   -kp * xte: drives toward the line (proportional to distance off)
        //   kh * heading_error: aligns with the line bearing
        //
        // Without kh: as XTE→0 the controller commands "straighten up", but
        // if the tractor approached at an angle (say 15°), "straight wheels"
        // means driving a straight line AT 15° TO THE AB LINE. The tractor
        // punches through and overshoots. This was causing our tractor to
        // grab the next AB line and drive perpendicular.
        //
        // With kh: the controller keeps turning until BOTH errors are small.
        // The heading term naturally damps the approach — as the tractor
        // aligns with the line, both terms go to zero together.
        //
        // Sign: positive heading_error = pointed right of line bearing.
        // kh * positive_heading_error = positive desired angle = steer right.
        // But we want to steer LEFT to correct rightward heading error, so
        // we SUBTRACT: -kh * heading_error (same sign logic as XTE term).
        self.last_heading_error = heading_error_deg;

        let desired_angle = if xte_m.abs() < self.deadband_m && heading_error_deg.abs() < 2.0 {
            // Within deadband for both XTE and heading — target straight ahead.
            // Heading threshold of 2° prevents hunting when well-aligned.
            0.0
        } else {
            // XTE term: positive XTE (right of line) → negative desired angle (steer left)
            // Heading term: positive heading error (pointed right) → negative desired angle (steer left)
            let raw = -self.kp * xte_m - self.kh * heading_error_deg;
            raw.clamp(-self.max_steer_angle, self.max_steer_angle)
        };

        self.last_desired_angle = desired_angle;

        // === Inner loop: angle error → motor PWM ===
        let actual = actual_angle_deg.unwrap_or(0.0);
        self.last_actual_angle = actual;

        let angle_error = desired_angle - actual;

        let raw_pwm = self.kp_angle * angle_error;
        let clamped = (raw_pwm.round() as i16).clamp(-self.max_pwm, self.max_pwm);

        // Motor deadzone compensation: the Trimble EZ-Steer doesn't spin below
        // ~80 PWM. If the controller wants any non-zero output, boost it to at
        // least min_pwm so the motor actually moves. Without this, small
        // corrections sit in the 0–79 dead zone doing nothing until the error
        // grows large enough to produce PWM ≥ 80, causing delayed lurching.
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
