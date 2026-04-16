//! Heading filter — fuses GPS course-over-ground with BNO055 IMU heading.
//!
//! ## Why this exists
//!
//! The raw GPS heading (VTG course-over-ground) is:
//! - Updated at 1 Hz (LC29H DA limitation)
//! - Derived from successive position fixes, so noisy at low speed
//! - Meaningless at standstill
//!
//! The BNO055 IMU provides a magnetometer-anchored yaw reading at 10 Hz with
//! calibration status. Gyro integration gives us high-frequency responsiveness;
//! GPS COG gives long-term truth that corrects gyro drift.
//!
//! ## Algorithm
//!
//! Complementary filter structure, but since the BNO055 already sends an
//! absolute heading (not raw gyro rate), we differentiate its heading signal
//! internally to recover yaw rate, then blend with GPS COG:
//!
//! ```text
//!   dt = elapsed since last IMU update
//!   imu_yaw_rate = wrap_diff(imu_heading - prev_imu_heading) / dt
//!   predicted = wrap(fused + imu_yaw_rate * dt)
//!   fused = alpha * predicted + (1 - alpha) * gps_cog
//! ```
//!
//! `alpha` is close to 1 (default 0.98), meaning most of the new estimate comes
//! from the IMU prediction, with a small pull toward GPS COG each time one
//! arrives. With GPS at 1 Hz and alpha=0.98, the filter trusts the IMU almost
//! completely between fixes, then corrects a small amount at each fix.
//!
//! ## Fallbacks
//!
//! - If the BNO055 `cal_sys` calibration status is below the gate, the filter
//!   falls back to pure GPS COG (no IMU prediction).
//! - If speed is below the speed gate, GPS COG is ignored (noise dominates at
//!   low speed) and the filter holds on IMU prediction only.
//! - If neither IMU nor GPS have been seen, `current_heading()` returns None.
//!
//! ## Wrap-around
//!
//! Heading is in 0-360° range. All differences and integrations must respect
//! the wrap boundary. Helper `wrap_diff()` returns the signed shortest-path
//! difference in -180..+180.

use std::time::Instant;
use finn_guidance_common::types::{GpsFix, ImuData};

/// Minimum BNO055 system calibration level to trust the IMU. The BNO055
/// reports `cal_sys` from 0 (uncalibrated) to 3 (fully calibrated).
/// At level 2 the magnetometer reference is reliable enough for guidance.
const MIN_IMU_CAL_SYS: u8 = 2;

/// Minimum speed (m/s) for GPS COG to be trusted. Below this, course-over-ground
/// is essentially noise — a stationary GPS wandering a few cm produces random
/// bearings that would corrupt the fused heading. At 0.8 m/s (~2.9 km/h) the
/// tractor has moved enough per GPS epoch to give a meaningful COG.
const MIN_SPEED_FOR_GPS_HEADING: f64 = 0.8;

/// Complementary filter weight. Higher = trust IMU prediction more per step,
/// pull toward GPS COG less. At 10 Hz IMU updates and 0.98, the IMU effectively
/// dominates between GPS fixes (1 Hz), with GPS providing long-term anchoring.
const ALPHA: f64 = 0.98;

pub struct HeadingFilter {
    /// The current fused heading estimate (degrees, 0-360).
    /// None until at least one heading source has been seen.
    fused: Option<f64>,

    /// Last IMU heading reading (degrees, 0-360).
    last_imu_heading: Option<f64>,

    /// Wall-clock time of last IMU update, for dt computation.
    last_imu_time: Option<Instant>,

    /// Most recent BNO055 system calibration status (0-3).
    last_cal_sys: u8,

    /// Whether the IMU is currently considered trusted (cal_sys >= gate).
    /// Exposed for UI display.
    pub imu_trusted: bool,

    /// Whether the filter currently has a fix (either source seen at least once).
    pub has_heading: bool,
}

impl HeadingFilter {
    pub fn new() -> Self {
        Self {
            fused: None,
            last_imu_heading: None,
            last_imu_time: None,
            last_cal_sys: 0,
            imu_trusted: false,
            has_heading: false,
        }
    }

    /// Feed in a new IMU reading. Updates the fused heading using gyro-style
    /// integration (derived from successive IMU heading samples).
    ///
    /// Call this every time an `$FINNIMU` message is received.
    pub fn update_imu(&mut self, imu: &ImuData) {
        self.last_cal_sys = imu.cal_sys;
        self.imu_trusted = imu.cal_sys >= MIN_IMU_CAL_SYS;

        let now = Instant::now();
        let imu_heading = normalise_360(imu.heading);

        // Derive IMU yaw rate from successive heading samples, respecting wrap.
        let yaw_rate_deg_per_sec = match (self.last_imu_heading, self.last_imu_time) {
            (Some(prev_h), Some(prev_t)) => {
                let dt = now.duration_since(prev_t).as_secs_f64();
                if dt > 0.0 && dt < 1.0 {
                    // Reasonable dt — compute rate
                    Some(wrap_diff(imu_heading - prev_h) / dt)
                } else {
                    // Gap too long or too short — can't trust the rate
                    None
                }
            }
            _ => None,
        };

        // If we have a yaw rate and the IMU is trusted, advance the fused estimate
        // by gyro integration. Otherwise, if this is the first IMU sample, seed
        // the filter with the raw IMU heading (if we don't already have one).
        if self.imu_trusted {
            if let (Some(rate), Some(dt), Some(fused)) = (
                yaw_rate_deg_per_sec,
                self.last_imu_time.map(|t| now.duration_since(t).as_secs_f64()),
                self.fused,
            ) {
                let predicted = normalise_360(fused + rate * dt);
                // Without a fresh GPS COG this call, just take the prediction.
                // (GPS pull happens in update_gps_fix.)
                self.fused = Some(predicted);
            } else if self.fused.is_none() {
                // First sample — seed with the IMU's absolute heading.
                self.fused = Some(imu_heading);
                self.has_heading = true;
            }
        } else if self.fused.is_none() {
            // IMU not yet calibrated; don't seed from it. Wait for GPS.
        }

        self.last_imu_heading = Some(imu_heading);
        self.last_imu_time = Some(now);
    }

    /// Feed in a new GPS fix. Pulls the fused heading toward GPS COG by a small
    /// amount, subject to speed gating.
    ///
    /// Call this every time a real `GpsFix` is received from the parser.
    pub fn update_gps_fix(&mut self, fix: &GpsFix) {
        let gps_heading = normalise_360(fix.heading);

        // Only trust GPS COG when moving fast enough. At low speed, COG is
        // derived from noisy position deltas and adds noise to the filter.
        let gps_trustworthy = fix.speed >= MIN_SPEED_FOR_GPS_HEADING;

        match (self.fused, gps_trustworthy, self.imu_trusted) {
            (None, true, _) => {
                // No fused estimate yet, GPS is good — seed from GPS.
                self.fused = Some(gps_heading);
                self.has_heading = true;
            }
            (Some(current), true, true) => {
                // Both sources usable: pull the IMU-integrated estimate toward
                // GPS by (1 - alpha). Using wrap-aware interpolation.
                let diff = wrap_diff(gps_heading - current);
                let pulled = normalise_360(current + (1.0 - ALPHA) * diff);
                self.fused = Some(pulled);
            }
            (Some(_), true, false) => {
                // IMU not trusted — snap to GPS heading (GPS-only mode).
                self.fused = Some(gps_heading);
            }
            (Some(_), false, true) => {
                // Stationary or slow — don't pull toward noisy GPS COG.
                // Fused estimate continues from IMU prediction.
            }
            (Some(_), false, false) => {
                // Slow and IMU not trusted — no good source. Hold last estimate.
            }
            (None, false, _) => {
                // No fused estimate and GPS is too slow to trust. Wait.
            }
        }
    }

    /// Get the current fused heading. Returns None if the filter has no data yet.
    pub fn current_heading(&self) -> Option<f64> {
        self.fused
    }
}

/// Normalise an angle to the 0-360 range.
fn normalise_360(mut angle: f64) -> f64 {
    while angle < 0.0 {
        angle += 360.0;
    }
    while angle >= 360.0 {
        angle -= 360.0;
    }
    angle
}

/// Shortest signed difference between two angles, in -180..+180.
/// Handles wrap-around correctly: wrap_diff(5 - 355) = +10, not -350.
fn wrap_diff(mut diff: f64) -> f64 {
    while diff > 180.0 {
        diff -= 360.0;
    }
    while diff < -180.0 {
        diff += 360.0;
    }
    diff
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrap_diff_basic() {
        assert!((wrap_diff(10.0) - 10.0).abs() < 1e-9);
        assert!((wrap_diff(-10.0) - (-10.0)).abs() < 1e-9);
    }

    #[test]
    fn test_wrap_diff_across_zero() {
        // From 355° to 5° is +10° (shortest path), not -350°
        assert!((wrap_diff(5.0 - 355.0) - 10.0).abs() < 1e-9);
        // From 5° to 355° is -10°, not +350°
        assert!((wrap_diff(355.0 - 5.0) - (-10.0)).abs() < 1e-9);
    }

    #[test]
    fn test_normalise_360() {
        assert!((normalise_360(370.0) - 10.0).abs() < 1e-9);
        assert!((normalise_360(-10.0) - 350.0).abs() < 1e-9);
        assert!((normalise_360(180.0) - 180.0).abs() < 1e-9);
        assert!((normalise_360(0.0) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_filter_seeds_from_gps_when_fast() {
        let mut f = HeadingFilter::new();
        let fix = GpsFix {
            latitude: 0.0, longitude: 0.0, altitude: 0.0,
            speed: 2.0, heading: 90.0,
            fix_quality: finn_guidance_common::types::FixQuality::Gps,
            satellites: 10, hdop: 1.0, timestamp_ms: 0,
        };
        f.update_gps_fix(&fix);
        assert!((f.current_heading().unwrap() - 90.0).abs() < 0.1);
    }

    #[test]
    fn test_filter_ignores_slow_gps() {
        let mut f = HeadingFilter::new();
        let fix = GpsFix {
            latitude: 0.0, longitude: 0.0, altitude: 0.0,
            speed: 0.1, heading: 90.0,
            fix_quality: finn_guidance_common::types::FixQuality::Gps,
            satellites: 10, hdop: 1.0, timestamp_ms: 0,
        };
        f.update_gps_fix(&fix);
        assert!(f.current_heading().is_none(),
            "Should not seed from slow GPS");
    }

    #[test]
    fn test_filter_seeds_from_calibrated_imu() {
        let mut f = HeadingFilter::new();
        let imu = ImuData {
            roll: 0.0, pitch: 0.0, heading: 120.0,
            cal_sys: 3, cal_gyro: 3, cal_accel: 3, cal_mag: 3,
        };
        f.update_imu(&imu);
        assert!((f.current_heading().unwrap() - 120.0).abs() < 0.1);
    }

    #[test]
    fn test_filter_rejects_uncalibrated_imu_seed() {
        let mut f = HeadingFilter::new();
        let imu = ImuData {
            roll: 0.0, pitch: 0.0, heading: 120.0,
            cal_sys: 1, cal_gyro: 0, cal_accel: 0, cal_mag: 0,
        };
        f.update_imu(&imu);
        assert!(f.current_heading().is_none(),
            "Should not seed from uncalibrated IMU");
    }
}
