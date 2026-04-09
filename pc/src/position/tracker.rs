//! Position tracker - maintains vehicle state and position history.

use finn_guidance_common::types::GpsFix;

pub struct PositionTracker {
    /// Current position
    pub current: Option<GpsFix>,
    /// Position history for trail drawing
    pub trail: Vec<(f64, f64)>,
    /// Maximum trail points to keep
    max_trail: usize,
    /// Total distance travelled in metres
    pub odometer_m: f64,
}

impl PositionTracker {
    pub fn new(max_trail: usize) -> Self {
        Self {
            current: None,
            trail: Vec::with_capacity(max_trail),
            max_trail,
            odometer_m: 0.0,
        }
    }

    /// Update with a new GPS fix
    pub fn update(&mut self, fix: GpsFix) {
        // Calculate distance from last position for odometer
        if let Some(prev) = &self.current {
            let dist = finn_guidance_common::coords::haversine_distance(
                prev.latitude, prev.longitude,
                fix.latitude, fix.longitude,
            );
            // Only add to odometer if distance is plausible (< 10m per update, filters GPS jumps)
            if dist < 10.0 {
                self.odometer_m += dist;
            }
        }

        // Add to trail
        self.trail.push((fix.latitude, fix.longitude));
        if self.trail.len() > self.max_trail {
            self.trail.remove(0);
        }

        self.current = Some(fix);
    }
}
