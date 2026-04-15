//! Steering controller — converts cross-track error into motor PWM commands.
//!
//! This is the auto-steer loop. It takes the XTE from the AB line guidance
//! and outputs a PWM value to send to the motor ESP32 via `$FINNSTEER`.
//!
//! ## Control strategy
//!
//! Phase 1 (current): Pure proportional control.
//!   pwm = kp × xte_m
//!
//! The sign convention is:
//!   - Positive XTE = vehicle is RIGHT of the line
//!   - Positive PWM = steer RIGHT (before motor_invert is applied)
//!   - So positive XTE should produce NEGATIVE PWM (steer left to correct)
//!   - The controller outputs: pwm = -kp × xte
//!
//! Motor direction inversion (`apply_motor_direction`) is applied by the
//! caller (app.rs) AFTER this controller returns, so this module always
//! uses the "positive PWM = steer right" convention.
//!
//! ## Safety
//!
//! The ESP32 motor firmware has a 500ms watchdog — if it doesn't receive a
//! `$FINNSTEER` command within 500ms, it kills the motor. The GUI loop
//! sends a command every frame (~16ms at 60fps), keeping the watchdog fed.
//! If the app freezes or exits, the watchdog fires and the motor stops.
//!
//! Additional PC-side safety:
//! - Auto-disengage if GPS fix age exceeds `max_fix_age_secs`
//! - Auto-disengage if WAS data stops arriving (sensor ESP32 disconnected)
//! - PWM clamped to `max_pwm` (never sends full 255 unless configured to)
//! - Deadband prevents hunting when already close to the line

use std::time::Instant;

/// Steering controller state.
pub struct SteeringController {
    /// Proportional gain: PWM per metre of cross-track error.
    /// Higher = more aggressive correction.
    /// Starting value: 100 (1m off-line → 100 PWM ≈ 40% power).
    pub kp: f64,

    /// Maximum PWM magnitude the controller will output.
    /// Provides a hard ceiling below the absolute 255 limit.
    pub max_pwm: i16,

    /// Deadband in metres. XTE below this produces zero output.
    /// Prevents the motor from hunting/buzzing when on-line.
    /// 0.03 = 3cm deadband (reasonable for standalone GPS).
    pub deadband_m: f64,

    /// Whether auto-steer is currently engaged.
    pub engaged: bool,

    /// Minimum speed (m/s) for auto-steer to produce output.
    /// Below this, the controller outputs zero to avoid steering
    /// at standstill based on GPS drift.
    pub min_speed: f64,

    /// Maximum GPS fix age (seconds) before auto-disengage.
    /// If we haven't received a real fix in this long, GPS is
    /// probably disconnected — stop steering.
    pub max_fix_age_secs: f64,

    /// Timestamp of the last real GPS fix received.
    /// Used for the fix-age safety check.
    last_fix_time: Option<Instant>,

    /// Timestamp of the last WAS reading received.
    /// Used for sensor-health safety check.
    last_was_time: Option<Instant>,

    /// Maximum WAS data age (seconds) before auto-disengage.
    pub max_was_age_secs: f64,

    /// The last PWM value computed (for display/debug).
    pub last_output_pwm: i16,

    /// Reason for last disengage (for UI display).
    pub disengage_reason: Option<String>,
}

impl SteeringController {
    pub fn new() -> Self {
        Self {
            kp: 100.0,
            max_pwm: 180,
            deadband_m: 0.03,
            engaged: false,
            min_speed: 0.5, // ~1.8 km/h
            max_fix_age_secs: 2.0,
            last_fix_time: None,
            last_was_time: None,
            max_was_age_secs: 1.0,
            last_output_pwm: 0,
            disengage_reason: None,
        }
    }

    /// Engage auto-steer. Returns false if preconditions aren't met.
    pub fn engage(&mut self) -> bool {
        self.disengage_reason = None;
        self.last_output_pwm = 0;
        self.engaged = true;
        true
    }

    /// Disengage auto-steer with an optional reason.
    pub fn disengage(&mut self, reason: Option<String>) {
        self.engaged = false;
        self.last_output_pwm = 0;
        self.disengage_reason = reason;
    }

    /// Notify the controller that a real GPS fix was received.
    /// Call this from the GPS fix processing loop (not the interpolated path).
    pub fn notify_gps_fix(&mut self) {
        self.last_fix_time = Some(Instant::now());
    }

    /// Notify the controller that a WAS reading was received.
    /// Call this when processing FINN WAS messages.
    pub fn notify_was_reading(&mut self) {
        self.last_was_time = Some(Instant::now());
    }

    /// Run safety checks. Returns Some(reason) if auto-steer should disengage.
    fn check_safety(&self) -> Option<String> {
        // Check GPS fix age
        if let Some(fix_time) = self.last_fix_time {
            if fix_time.elapsed().as_secs_f64() > self.max_fix_age_secs {
                return Some("GPS fix lost".to_string());
            }
        } else {
            return Some("No GPS fix received".to_string());
        }

        // Check WAS data age
        if let Some(was_time) = self.last_was_time {
            if was_time.elapsed().as_secs_f64() > self.max_was_age_secs {
                return Some("WAS data lost".to_string());
            }
        } else {
            return Some("No WAS data received".to_string());
        }

        None
    }

    /// Compute the steering PWM output from cross-track error.
    ///
    /// Call this every GUI frame. Returns the PWM value to send (before
    /// motor_invert is applied), or 0 if disengaged or safety-tripped.
    ///
    /// `xte_m`: cross-track error in metres (positive = right of line)
    /// `speed_mps`: current vehicle speed in m/s
    ///
    /// Returns (pwm, disengaged_this_frame)
    pub fn compute(&mut self, xte_m: f64, speed_mps: f64) -> (i16, bool) {
        if !self.engaged {
            self.last_output_pwm = 0;
            return (0, false);
        }

        // Safety checks — auto-disengage if something is wrong
        if let Some(reason) = self.check_safety() {
            self.disengage(Some(reason));
            return (0, true);
        }

        // Don't steer at standstill (GPS drift would cause random steering)
        if speed_mps < self.min_speed {
            self.last_output_pwm = 0;
            return (0, false);
        }

        // Deadband — if we're close enough, don't steer
        if xte_m.abs() < self.deadband_m {
            self.last_output_pwm = 0;
            return (0, false);
        }

        // Proportional control:
        // Positive XTE = vehicle is right of line → need to steer LEFT → negative PWM
        // So: pwm = -kp * xte
        let raw_pwm = -self.kp * xte_m;

        // Clamp to max_pwm
        let clamped = raw_pwm.round() as i16;
        let clamped = clamped.clamp(-self.max_pwm, self.max_pwm);

        self.last_output_pwm = clamped;
        (clamped, false)
    }
}
