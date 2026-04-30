//! GPS position interpolator — smooth GUI updates between 10Hz fixes.
//!
//! The LC29H BA outputs fixes at 10Hz via onboard dead-reckoning fusion.
//! This module dead-reckons intermediate positions between real fixes
//! using the last known speed and heading, bridging 100ms gaps for smooth
//! ~30fps GUI rendering.
//!
//! ## How it works
//!
//! 1. When a real GPS fix arrives, call `update_fix()` — this resets the
//!    interpolator with the true position, speed, and heading.
//! 2. On every GUI frame, call `interpolate(override_heading)` — this
//!    extrapolates the position forward by `speed × dt` along the heading.
//! 3. The returned `GpsFix` is a synthetic fix suitable for display and guidance
//!    calculations. It carries the same metadata (sats, HDOP, fix quality) as
//!    the last real fix, but with updated position.
//!
//! ## What uses interpolated vs real positions
//!
//! - **Field view, trail, lightbar, XTE readout**: interpolated (smooth)
//! - **Coverage logging**: real fixes only (distance-based filter needs true positions)
//! - **Auto-pass detection**: real fixes only (avoid interpolation jitter triggering snaps)
//!
//! ## Accuracy
//!
//! At tractor speeds (10-15 km/h ≈ 3-4 m/s), 100ms of dead reckoning
//! between 10Hz fixes accumulates roughly 1-2cm of error before the next
//! real fix corrects it. With the BA's DR-fused heading, projection
//! direction stays correct through turns.

use finn_guidance_common::coords;
use finn_guidance_common::types::GpsFix;
use std::time::Instant;

/// Position interpolator for smooth GUI updates between GPS fixes.
pub struct PositionInterpolator {
    /// The last real GPS fix received
    last_fix: Option<GpsFix>,
    /// Wall-clock time when the last real fix was received
    last_fix_time: Option<Instant>,
    /// The most recent interpolated position (cached for the current frame)
    current_interpolated: Option<GpsFix>,
    /// Maximum interpolation time (seconds). Beyond this, we stop extrapolating
    /// to avoid runaway drift if GPS drops out.
    max_extrapolation_secs: f64,
    /// Minimum speed (m/s) to interpolate. Below this, just hold the last position.
    /// Prevents jitter from noisy heading at near-standstill.
    min_speed_for_interpolation: f64,
}

impl PositionInterpolator {
    pub fn new() -> Self {
        Self {
            last_fix: None,
            last_fix_time: None,
            current_interpolated: None,
            max_extrapolation_secs: 2.0,
            min_speed_for_interpolation: 0.3, // ~1 km/h
        }
    }

    /// Update with a new real GPS fix. Resets the interpolation origin.
    pub fn update_fix(&mut self, fix: &GpsFix) {
        self.last_fix = Some(fix.clone());
        self.last_fix_time = Some(Instant::now());
        // Immediately set the interpolated position to the real fix
        self.current_interpolated = Some(fix.clone());
    }

    /// Get the current interpolated position. Call this every GUI frame.
    ///
    /// Returns `None` if no GPS fix has been received yet.
    /// Between real fixes, extrapolates the position using dead reckoning.
    /// When a new real fix arrives (via `update_fix`), the position snaps
    /// back to truth.
    ///
    /// If `override_heading` is provided, it replaces `fix.heading` for both
    /// the dead-reckon projection AND as the `heading` field of the returned
    /// fix. This lets the IMU-fused heading flow through to everything
    /// downstream (field view, guidance error, steering).
    pub fn interpolate(&mut self, override_heading: Option<f64>) -> Option<&GpsFix> {
        let fix = self.last_fix.as_ref()?;
        let fix_time = self.last_fix_time?;

        let elapsed = fix_time.elapsed().as_secs_f64();

        let heading_for_projection = override_heading.unwrap_or(fix.heading);

        // Don't extrapolate if:
        // - No time has passed (we're on the same frame as the fix)
        // - Speed is too low (heading is unreliable at standstill)
        // - Too much time has passed (GPS might have dropped out)
        if elapsed < 0.001
            || fix.speed < self.min_speed_for_interpolation
            || elapsed > self.max_extrapolation_secs
        {
            let mut held = fix.clone();
            if let Some(h) = override_heading {
                held.heading = h;
            }
            self.current_interpolated = Some(held);
            return self.current_interpolated.as_ref();
        }

        // Dead reckon: move position forward along heading by speed × dt
        let distance_m = fix.speed * elapsed;
        let (new_lat, new_lon) = destination_point(
            fix.latitude,
            fix.longitude,
            heading_for_projection,
            distance_m,
        );

        // Build the interpolated fix — same metadata as the real fix,
        // just with an updated position and (optionally) a fresher heading
        let mut interp = fix.clone();
        interp.latitude = new_lat;
        interp.longitude = new_lon;
        if let Some(h) = override_heading {
            interp.heading = h;
        }

        self.current_interpolated = Some(interp);
        self.current_interpolated.as_ref()
    }

    /// Returns true if we have a valid fix to work with.
    pub fn has_fix(&self) -> bool {
        self.last_fix.is_some()
    }

    /// Get the last real (non-interpolated) fix, for display of true GPS metadata.
    pub fn last_real_fix(&self) -> Option<&GpsFix> {
        self.last_fix.as_ref()
    }
}

/// Calculate the destination point given a start point, bearing, and distance.
///
/// Uses the spherical Earth model (same as the rest of our coord math).
/// Returns (latitude, longitude) in degrees.
fn destination_point(lat: f64, lon: f64, bearing_deg: f64, distance_m: f64) -> (f64, f64) {
    const EARTH_RADIUS: f64 = 6_371_000.0;

    let lat_r = coords::deg_to_rad(lat);
    let lon_r = coords::deg_to_rad(lon);
    let brg_r = coords::deg_to_rad(bearing_deg);
    let angular_dist = distance_m / EARTH_RADIUS;

    let new_lat_r =
        (lat_r.sin() * angular_dist.cos() + lat_r.cos() * angular_dist.sin() * brg_r.cos()).asin();

    let new_lon_r = lon_r
        + (brg_r.sin() * angular_dist.sin() * lat_r.cos())
            .atan2(angular_dist.cos() - lat_r.sin() * new_lat_r.sin());

    (coords::rad_to_deg(new_lat_r), coords::rad_to_deg(new_lon_r))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_destination_point_north() {
        // Moving 100m north from a known point
        let (lat, lon) = destination_point(-33.275, 138.590, 0.0, 100.0);
        // Should be ~0.0009° further north
        assert!(lat > -33.275, "Should move north, got {}", lat);
        assert!(
            (lon - 138.590).abs() < 0.0001,
            "Lon should stay similar, got {}",
            lon
        );
    }

    #[test]
    fn test_destination_point_east() {
        // Moving 100m east
        let (lat, lon) = destination_point(-33.275, 138.590, 90.0, 100.0);
        assert!((lat - (-33.275)).abs() < 0.0001, "Lat should stay similar");
        assert!(lon > 138.590, "Should move east, got {}", lon);
    }

    #[test]
    fn test_destination_point_zero_distance() {
        let (lat, lon) = destination_point(-33.275, 138.590, 45.0, 0.0);
        assert!((lat - (-33.275)).abs() < 1e-10);
        assert!((lon - 138.590).abs() < 1e-10);
    }
}
