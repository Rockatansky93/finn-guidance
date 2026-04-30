//! Steering controller — pure pursuit outer loop only.
//!
//! ## Decision #026 Architecture
//!
//! The inner loop (WAS feedback → PWM with sub-stall pulsing) has moved to
//! the motor ESP32 firmware, where it runs at ~100Hz. This module now only
//! computes the **desired steering angle** using pure pursuit geometry, XTE
//! rate damping, and smooth taper. The angle is sent to the ESP32 as a
//! `$FINNSTEER,<angle_x100>` command at ~10Hz.
//!
//! ## Pure pursuit outer loop
//!
//! Picks a lookahead point on the AB line some distance `L` ahead of the
//! tractor, then commands the wheel angle that would curve the vehicle
//! through that point (bicycle model):
//!
//! ```text
//!   L = lookahead_base + lookahead_speed_factor × speed
//!   alpha = atan2(-xte, L) - heading_error_rad
//!   desired_angle = atan2(2 · wheelbase · sin(alpha), L)
//! ```
//!
//! ## Waveform damping (stays on PC)
//!
//! XTE rate damping (dXTE/dt) operates on the desired angle before sending
//! it to the ESP32. When converging on the line, the damping reduces the
//! correction to prevent overshoot.
//!
//! ## Sign convention
//!
//!   - Positive XTE = vehicle is RIGHT of the line
//!   - Positive heading_error = nose pointed RIGHT of line bearing
//!   - Positive angle = wheels turned RIGHT
//!
//! ## Safety (PC-side)
//!
//! - Auto-disengage if GPS fix age exceeds `max_fix_age_secs`
//! - Speed gate: no steering below `min_speed` (sends angle = 0)
//! - Max steering angle clamp (`max_steer_angle`)
//!
//! Motor-side safety (on ESP32): 500ms watchdog, WAS-local feedback.

use std::time::Instant;

/// Steering controller state — outer loop only (Decision #026).
pub struct SteeringController {
    // === Outer loop: pure pursuit ===
    /// Base lookahead distance (metres). Maps to "Online Aggression" slider.
    pub lookahead_base: f64,

    /// Speed-scaled lookahead coefficient (seconds). Maps to "Approach Aggression" slider.
    pub lookahead_speed_factor: f64,

    /// Tractor wheelbase in metres. Measure your tractor — not a tuning knob.
    pub wheelbase_m: f64,

    /// Maximum desired steering angle (degrees).
    pub max_steer_angle: f64,

    // === XTE deadband / smooth taper ===
    /// XTE deadband in metres. Within this zone, desired angle tapers
    /// linearly to zero rather than cutting hard.
    pub deadband_m: f64,

    // === Waveform damping ===
    /// XTE rate damping gain. Reduces corrections when converging,
    /// increases urgency when diverging.
    pub kd_xte: f64,

    /// Previous XTE sample for computing dXTE/dt.
    prev_xte_m: Option<f64>,

    /// Timestamp of the previous compute() call for dt calculation.
    prev_compute_time: Option<Instant>,

    // === Control state ===
    /// Whether auto-steer is currently engaged.
    pub engaged: bool,

    /// Minimum speed (m/s) for auto-steer to produce output.
    pub min_speed: f64,

    // === Safety ===
    /// Maximum GPS fix age (seconds) before auto-disengage.
    pub max_fix_age_secs: f64,

    /// Timestamp of the last real GPS fix received.
    last_fix_time: Option<Instant>,

    // === Debug/display ===
    /// The last desired steering angle computed (for display).
    pub last_desired_angle: f64,

    /// The last actual steering angle from motor ESP32 status (for display).
    pub last_actual_angle: f64,

    /// The last PWM from motor ESP32 status (for display).
    pub last_output_pwm: i16,

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
            lookahead_base: 3.0,
            lookahead_speed_factor: 1.0,
            wheelbase_m: 2.8,
            max_steer_angle: 15.0,
            deadband_m: 0.03,

            kd_xte: 0.5,
            prev_xte_m: None,
            prev_compute_time: None,

            engaged: false,
            min_speed: 0.5,
            max_fix_age_secs: 2.0,
            last_fix_time: None,
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
        self.prev_xte_m = None;
        self.prev_compute_time = None;
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
        self.disengage_reason = reason;
    }

    /// Notify the controller that a real GPS fix was received.
    pub fn notify_gps_fix(&mut self) {
        self.last_fix_time = Some(Instant::now());
    }

    /// Update display fields from motor ESP32 status feedback.
    /// Called when a $FINNMTR message is received.
    pub fn update_motor_feedback(&mut self, pwm: i16, actual_angle: f64) {
        self.last_output_pwm = pwm;
        self.last_actual_angle = actual_angle;
    }

    /// Run safety checks. Returns Some(reason) if auto-steer should disengage.
    fn check_safety(&mut self) -> Option<String> {
        if let Some(fix_time) = self.last_fix_time {
            if fix_time.elapsed().as_secs_f64() > self.max_fix_age_secs {
                return Some("GPS fix lost".to_string());
            }
        } else {
            return Some("No GPS fix received".to_string());
        }

        // Note: WAS safety is now handled on the ESP32 side (local ADC read,
        // no timeout needed). The motor ESP32 watchdog handles command loss.

        None
    }

    /// Compute the desired steering angle using pure pursuit geometry.
    ///
    /// Decision #026: returns a desired angle (f64, degrees) instead of PWM.
    /// The caller sends this to the motor ESP32 via $FINNSTEER. The ESP32's
    /// inner loop converts it to PWM using local WAS feedback at ~100Hz.
    ///
    /// - `xte_m`: cross-track error in metres (positive = right of line)
    /// - `heading_error_deg`: heading error in degrees (positive = pointed right)
    /// - `speed_mps`: current vehicle speed in m/s
    ///
    /// Returns (desired_angle_deg, disengaged_this_frame).
    pub fn compute(&mut self, xte_m: f64, heading_error_deg: f64, speed_mps: f64) -> (f64, bool) {
        if !self.engaged {
            self.last_desired_angle = 0.0;
            return (0.0, false);
        }

        // Safety checks
        if let Some(reason) = self.check_safety() {
            self.disengage(Some(reason));
            return (0.0, true);
        }

        // Speed gate — send zero angle (ESP32 centres wheels)
        if speed_mps < self.min_speed {
            self.last_desired_angle = 0.0;
            return (0.0, false);
        }

        self.last_heading_error = heading_error_deg;

        // === Compute dXTE/dt for waveform damping ===
        let now = Instant::now();
        let xte_rate = match (self.prev_xte_m, self.prev_compute_time) {
            (Some(prev_xte), Some(prev_time)) => {
                let dt = prev_time.elapsed().as_secs_f64();
                if dt > 0.001 && dt < 1.0 {
                    (xte_m - prev_xte) / dt
                } else {
                    0.0
                }
            }
            _ => 0.0,
        };
        self.prev_xte_m = Some(xte_m);
        self.prev_compute_time = Some(now);

        // === Outer loop: pure pursuit geometry ===
        let base_lookahead =
            (self.lookahead_base + self.lookahead_speed_factor * speed_mps).clamp(2.0, 15.0);

        // Heading-aware lookahead reduction: when the tractor is pointed
        // significantly off the line bearing but XTE is small (approaching
        // the crossing point), shorten the lookahead so the heading error
        // dominates the pure pursuit alpha. This makes the controller
        // anticipate the line crossing and begin counter-steering earlier.
        //
        // Without this, at long lookahead the XTE component drives the
        // alpha toward zero as the tractor reaches the line, and the
        // heading term is too weak to command a reversal before crossing.
        //
        // The reduction factor scales with |heading_error| / max_heading
        // and inversely with |XTE| — most active when near the line with
        // large heading offset. Clamped so lookahead never drops below 2m.
        let heading_abs = heading_error_deg.abs();
        let xte_abs = xte_m.abs();
        let lookahead_m = if heading_abs > 2.0 && xte_abs < 1.0 {
            // Scale: at 10° heading error and 0m XTE, reduce lookahead by ~40%
            // At 2° heading error, no reduction. Linear ramp between.
            let heading_factor = ((heading_abs - 2.0) / 8.0).clamp(0.0, 1.0);
            let proximity_factor = 1.0 - (xte_abs / 1.0).clamp(0.0, 1.0);
            let reduction = 0.4 * heading_factor * proximity_factor;
            (base_lookahead * (1.0 - reduction)).max(2.0)
        } else {
            base_lookahead
        };
        self.last_lookahead_m = lookahead_m;

        let psi_rad = heading_error_deg.to_radians();
        let alpha_line_frame = (-xte_m).atan2(lookahead_m);
        let alpha_rad = alpha_line_frame - psi_rad;

        let delta_rad = (2.0 * self.wheelbase_m * alpha_rad.sin()).atan2(lookahead_m);
        let mut desired_angle = delta_rad.to_degrees();

        // === Waveform damping ===
        // dXTE/dt tells us if we're converging or diverging.
        // We want to REDUCE the desired angle when converging (prevent
        // overshoot) and INCREASE it when diverging (add urgency).
        //
        // Sign analysis: if XTE > 0 (right of line), desired_angle < 0
        // (steer left). When converging, xte_rate < 0 (XTE shrinking).
        // We want to make desired_angle less negative (reduce magnitude).
        // Since xte_rate is negative and desired_angle is negative, we
        // SUBTRACT kd * xte_rate: desired_angle -= kd * (-rate) means
        // desired_angle += kd * |rate|, which makes it less negative. Correct.
        //
        // When diverging, xte_rate > 0, so desired_angle -= kd * (+rate)
        // makes it more negative (stronger correction). Also correct.
        desired_angle -= self.kd_xte * xte_rate;

        // === Smooth taper near the line ===
        if self.deadband_m > 0.0 {
            let xte_abs = xte_m.abs();
            if xte_abs < self.deadband_m && heading_error_deg.abs() < 2.0 {
                let taper = xte_abs / self.deadband_m;
                desired_angle *= taper;
            }
        }

        // Clamp to physical limits
        desired_angle = desired_angle.clamp(-self.max_steer_angle, self.max_steer_angle);

        self.last_desired_angle = desired_angle;
        (desired_angle, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn engaged_controller() -> SteeringController {
        let mut c = SteeringController::new();
        c.last_fix_time = Some(Instant::now());
        c.engaged = true;
        c.prev_compute_time = Some(Instant::now());
        c
    }

    #[test]
    fn test_on_line_aligned_commands_zero() {
        let mut c = engaged_controller();
        c.prev_xte_m = Some(0.0);
        let (angle, dis) = c.compute(0.0, 0.0, 2.0);
        assert!(!dis);
        assert!(angle.abs() < 0.01);
    }

    #[test]
    fn test_right_of_line_steers_left() {
        let mut c = engaged_controller();
        c.prev_xte_m = Some(1.0);
        let (angle, _) = c.compute(1.0, 0.0, 2.0);
        assert!(
            angle < 0.0,
            "Right of line should command left steering, got {}",
            angle
        );
    }

    #[test]
    fn test_left_of_line_steers_right() {
        let mut c = engaged_controller();
        c.prev_xte_m = Some(-1.0);
        let (angle, _) = c.compute(-1.0, 0.0, 2.0);
        assert!(
            angle > 0.0,
            "Left of line should command right steering, got {}",
            angle
        );
    }

    #[test]
    fn test_heading_error_alone_commands_correction() {
        let mut c = engaged_controller();
        c.prev_xte_m = Some(0.0);
        let (angle, _) = c.compute(0.0, 10.0, 2.0);
        assert!(
            angle < 0.0,
            "Pointed right of line should command left steer, got {}",
            angle
        );
    }

    #[test]
    fn test_max_steer_clamp() {
        let mut c = engaged_controller();
        c.max_steer_angle = 15.0;
        let (angle, _) = c.compute(50.0, 0.0, 2.0);
        assert!(
            angle.abs() <= 15.0 + 0.01,
            "Desired angle {} exceeds max",
            angle
        );
    }

    #[test]
    fn test_lookahead_scales_with_speed() {
        let mut c = engaged_controller();
        c.lookahead_base = 3.0;
        c.lookahead_speed_factor = 1.0;
        c.compute(1.0, 0.0, 0.0);
        let l_slow = c.last_lookahead_m;
        c.compute(1.0, 0.0, 5.0);
        let l_fast = c.last_lookahead_m;
        assert!(
            l_fast > l_slow,
            "Lookahead should grow with speed: {} vs {}",
            l_slow,
            l_fast
        );
    }

    #[test]
    fn test_speed_gate_returns_zero() {
        let mut c = engaged_controller();
        c.min_speed = 0.5;
        c.prev_xte_m = Some(2.0);
        let (angle, _) = c.compute(2.0, 10.0, 0.1);
        assert!((angle).abs() < 0.001, "Below min speed should output 0");
    }

    #[test]
    fn test_converging_xte_reduces_correction() {
        // When XTE is shrinking (converging), the damping term should
        // reduce the desired angle magnitude compared to steady-state.
        let mut c = engaged_controller();
        c.prev_xte_m = Some(1.0);
        c.compute(1.0, 0.0, 2.0);
        let steady_angle = c.last_desired_angle;

        // Converging XTE (was 1.5, now 1.0 -> negative rate).
        // With kd subtracted, converging should produce LESS aggressive correction.
        let mut c2 = engaged_controller();
        c2.prev_xte_m = Some(1.5);
        c2.compute(1.0, 0.0, 2.0);
        let converging_angle = c2.last_desired_angle;

        assert!(
            converging_angle.abs() < steady_angle.abs(),
            "Converging XTE should reduce correction: steady={:.3} converging={:.3}",
            steady_angle,
            converging_angle
        );
    }

    #[test]
    fn test_diverging_xte_increases_correction() {
        // When XTE is growing (diverging), the damping term should
        // increase the desired angle magnitude compared to steady-state.
        let mut c = engaged_controller();
        c.prev_xte_m = Some(1.0);
        c.compute(1.0, 0.0, 2.0);
        let steady_angle = c.last_desired_angle;

        // Diverging XTE (was 0.5, now 1.0 -> positive rate).
        let mut c2 = engaged_controller();
        c2.prev_xte_m = Some(0.5);
        c2.compute(1.0, 0.0, 2.0);
        let diverging_angle = c2.last_desired_angle;

        assert!(
            diverging_angle.abs() > steady_angle.abs(),
            "Diverging XTE should increase correction: steady={:.3} diverging={:.3}",
            steady_angle,
            diverging_angle
        );
    }
}
