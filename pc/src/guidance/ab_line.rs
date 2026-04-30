//! AB Line guidance - set two points, get cross-track error for any position.
//! Includes auto-pass selection: monitors cross-track distance and snaps to
//! the nearest pass line when the operator has moved to a different one.

use finn_guidance_common::coords;
use finn_guidance_common::types::{CrossTrackError, GpsFix, GuidanceLine};

/// Event emitted when auto-pass triggers a pass change.
#[derive(Debug, Clone)]
pub struct PassChangeEvent {
    pub old_pass: i32,
    pub new_pass: i32,
}

pub struct AbLineGuide {
    /// The reference AB line
    pub line: Option<GuidanceLine>,
    /// Current pass offset (pass_spacing * pass number)
    pub pass_offset_m: f64,
    /// Implement width in metres
    pub implement_width_m: f64,
    /// Overlap between passes in metres (positive = passes overlap, no gaps)
    pub overlap_m: f64,
    /// Current pass number (0 = on AB line, positive = right, negative = left)
    pub pass_number: i32,
    /// Fine lateral shift of the whole AB line system, in metres.
    /// Positive = shift right, negative = shift left.
    /// Applied on top of pass_offset — use for inter-row sowing alignment
    /// or correcting overlap/underlap without changing the pass number.
    /// Typical range: ±0.05–0.20 m (5–20 cm). Hard cap: ±implement_width_m.
    pub nudge_m: f64,

    // --- Auto-pass state ---
    /// Whether auto-pass selection is enabled
    pub auto_pass_enabled: bool,
    /// How far off the current pass line (as a fraction of pass spacing)
    /// the operator must be before auto-pass snaps to the nearest line.
    /// 0.5 = snap when you're closer to the next line than the current one.
    /// 0.6 = require a bit more drift before snapping (adds hysteresis).
    pub snap_threshold: f64,
    /// Minimum speed (m/s) for auto-pass to engage — ignore position at standstill
    pub min_speed_for_auto_pass: f64,
}

impl AbLineGuide {
    pub fn new(implement_width: f64) -> Self {
        Self {
            line: None,
            pass_offset_m: 0.0,
            implement_width_m: implement_width,
            overlap_m: 0.0,
            pass_number: 0,
            nudge_m: 0.0,
            auto_pass_enabled: true,
            snap_threshold: 0.6, // Require 60% of pass spacing drift before snapping
            min_speed_for_auto_pass: 0.5, // ~1.8 km/h — ignore GPS jitter at standstill
        }
    }

    /// Effective pass spacing: implement width minus overlap.
    /// This is the distance between adjacent pass lines.
    pub fn pass_spacing(&self) -> f64 {
        (self.implement_width_m - self.overlap_m).max(0.1)
    }

    /// Set point A (start of line) from current GPS position
    pub fn set_point_a(&mut self, fix: &GpsFix) {
        self.line = Some(GuidanceLine::AbLine {
            a: (fix.latitude, fix.longitude),
            b: (fix.latitude, fix.longitude), // B same as A until set
        });
        self.pass_number = 0;
        self.pass_offset_m = 0.0;
    }

    /// Set point B (end of line) from current GPS position
    pub fn set_point_b(&mut self, fix: &GpsFix) {
        if let Some(GuidanceLine::AbLine { a, .. }) = &self.line {
            let a = *a;
            self.line = Some(GuidanceLine::AbLine {
                a,
                b: (fix.latitude, fix.longitude),
            });
        }
    }

    /// Returns true if both A and B have been set and are distinct points.
    pub fn has_complete_line(&self) -> bool {
        match &self.line {
            Some(GuidanceLine::AbLine { a, b }) => {
                (a.0 - b.0).abs() > 1e-10 || (a.1 - b.1).abs() > 1e-10
            }
            _ => false,
        }
    }

    /// Returns the raw A/B coordinates if a complete line is set.
    pub fn ab_points(&self) -> Option<AbPoints> {
        match &self.line {
            Some(GuidanceLine::AbLine { a, b })
                if (a.0 - b.0).abs() > 1e-10 || (a.1 - b.1).abs() > 1e-10 =>
            {
                Some(AbPoints {
                    a_lat: a.0,
                    a_lon: a.1,
                    b_lat: b.0,
                    b_lon: b.1,
                })
            }
            _ => None,
        }
    }

    /// Load a saved AB line directly from coordinates (e.g. from the database).
    /// Resets pass number and offset so the operator starts on the base line.
    pub fn load_ab_line(&mut self, a_lat: f64, a_lon: f64, b_lat: f64, b_lon: f64) {
        self.line = Some(GuidanceLine::AbLine {
            a: (a_lat, a_lon),
            b: (b_lat, b_lon),
        });
        self.pass_number = 0;
        self.pass_offset_m = 0.0;
        self.nudge_m = 0.0;
    }

    /// Shift the AB line system laterally so the current GPS position reads
    /// exactly zero cross-track error on the current pass line.
    ///
    /// This is the "Snap line to me" function — compensates for GPS drift.
    /// Example: seeder is physically on the mark but XTE shows +1.0m right.
    /// Tapping this absorbs that 1.0m into the nudge offset so XTE drops to
    /// zero instantly.  The saved AB line geometry and pass number are both
    /// unchanged — only `nudge_m` is adjusted.
    ///
    /// Returns the applied shift in metres, or None if no line is loaded.
    pub fn align_grid_to_position(&mut self, fix: &GpsFix) -> Option<f64> {
        match &self.line {
            Some(GuidanceLine::AbLine { a, b }) => {
                if (a.0 - b.0).abs() < 1e-10 && (a.1 - b.1).abs() < 1e-10 {
                    return None;
                }

                // Get raw cross-track distance in the A→B frame.
                let raw_xtd =
                    coords::cross_track_distance(fix.latitude, fix.longitude, a.0, a.1, b.0, b.1);

                // The residual error in the A→B frame is:
                //   adjusted_xtd = raw_xtd - pass_offset_m - nudge_m
                // We want adjusted_xtd = 0, so:
                //   nudge_m_new = raw_xtd - pass_offset_m
                let old_nudge = self.nudge_m;
                self.nudge_m = raw_xtd - self.pass_offset_m;

                // Clamp to hard cap (±implement width covers full pass shifts).
                let cap = self.implement_width_m;
                self.nudge_m = self.nudge_m.clamp(-cap, cap);

                let shift = self.nudge_m - old_nudge;
                Some(shift)
            }
            _ => None,
        }
    }

    /// Shift the AB line to the right by `amount_m` metres.
    /// This nudges the entire system (all passes shift with it).
    /// Hard cap at ±implement_width_m to allow full pass-width shifts.
    pub fn nudge_right(&mut self, amount_m: f64) {
        let cap = self.implement_width_m;
        self.nudge_m = (self.nudge_m + amount_m).clamp(-cap, cap);
    }

    /// Shift the AB line to the left by `amount_m` metres.
    pub fn nudge_left(&mut self, amount_m: f64) {
        let cap = self.implement_width_m;
        self.nudge_m = (self.nudge_m - amount_m).clamp(-cap, cap);
    }

    /// Reset nudge back to zero.
    pub fn nudge_reset(&mut self) {
        self.nudge_m = 0.0;
    }

    /// Shift to the next pass (right)
    pub fn next_pass(&mut self) {
        self.pass_number += 1;
        self.pass_offset_m = self.pass_spacing() * self.pass_number as f64;
    }

    /// Shift to the previous pass (left)
    pub fn prev_pass(&mut self) {
        self.pass_number -= 1;
        self.pass_offset_m = self.pass_spacing() * self.pass_number as f64;
    }

    /// Update auto-pass state with a new GPS fix. Call this every fix.
    ///
    /// Distance-based approach: continuously checks which pass line is nearest.
    /// If the operator has drifted more than `snap_threshold` × pass_spacing
    /// from the current pass line, snaps to the nearest one. This works for
    /// headland turns, driving around obstacles, skipping rows, or any other
    /// lateral movement — no heading analysis needed.
    ///
    /// Returns Some(PassChangeEvent) if a pass change was triggered.
    pub fn update_auto_pass(&mut self, fix: &GpsFix) -> Option<PassChangeEvent> {
        if !self.auto_pass_enabled {
            return None;
        }
        // Don't evaluate when stationary — GPS jitter could cause false snaps
        if fix.speed < self.min_speed_for_auto_pass {
            return None;
        }

        // Calculate current cross-track error from the active pass line
        let error = self.calculate_error(fix)?;

        // Check if we've drifted far enough to be on a different pass
        let threshold_m = self.pass_spacing() * self.snap_threshold;
        if error.distance_m.abs() < threshold_m {
            // Still close enough to the current pass — no change
            return None;
        }

        // We've drifted past the threshold. Snap to whichever pass is nearest.
        let old_pass = self.pass_number;
        let new_pass = self.find_nearest_pass(fix);

        if new_pass == old_pass {
            // Nearest is still the current pass (can happen at exactly the
            // threshold boundary). No change.
            return None;
        }

        // Snap to the new pass
        self.pass_number = new_pass;
        self.pass_offset_m = self.pass_spacing() * new_pass as f64;

        Some(PassChangeEvent { old_pass, new_pass })
    }

    /// Find the pass number whose line is nearest to the current position.
    /// Works by calculating the raw cross-track distance from the AB base line
    /// and converting that to the nearest whole pass number.
    fn find_nearest_pass(&self, fix: &GpsFix) -> i32 {
        match &self.line {
            Some(GuidanceLine::AbLine { a, b }) => {
                let raw_xtd =
                    coords::cross_track_distance(fix.latitude, fix.longitude, a.0, a.1, b.0, b.1);
                // Subtract nudge before rounding so the nearest pass is found
                // relative to the nudged line system, not the raw AB origin.
                let nudge_adjusted = raw_xtd - self.nudge_m;
                (nudge_adjusted / self.pass_spacing()).round() as i32
            }
            _ => self.pass_number,
        }
    }

    /// Calculate cross-track error from the current (offset) guidance line
    pub fn calculate_error(&self, fix: &GpsFix) -> Option<CrossTrackError> {
        match &self.line {
            Some(GuidanceLine::AbLine { a, b }) => {
                // Don't calculate if A and B are the same point
                if (a.0 - b.0).abs() < 1e-10 && (a.1 - b.1).abs() < 1e-10 {
                    return None;
                }

                let raw_xtd =
                    coords::cross_track_distance(fix.latitude, fix.longitude, a.0, a.1, b.0, b.1);

                // Apply pass offset only — nudge is applied AFTER the
                // direction flip because it's world-fixed (see below).
                let pass_xtd = raw_xtd - self.pass_offset_m;

                // Heading error: difference between current heading and line bearing.
                //
                // The AB line defines a direction (A→B), but the tractor drives
                // both ways — north on one pass, south on the return. We detect
                // which direction the vehicle is traveling by checking if the raw
                // heading error exceeds 90°. If so, the vehicle is on a return
                // pass and we flip the reference bearing by 180° so heading_error
                // stays small and pure pursuit steers correctly.
                //
                // The XTE sign is defined relative to the A→B bearing. When we
                // flip the bearing for a return pass, the "right of line" / "left
                // of line" sense inverts, so we also negate the XTE to keep the
                // sign convention consistent with the flipped bearing.
                let line_bearing = coords::bearing(a.0, a.1, b.0, b.1);
                let raw_heading_error = normalise_angle(fix.heading - line_bearing);

                // Direction flip first, then apply nudge. Nudge is world-fixed:
                // "nudge left 50cm" always shifts the target 50cm to the left in
                // the real world, regardless of whether driving A→B or B→A.
                // Previously nudge was applied before the flip, which caused it
                // to reverse direction on return passes.
                let (heading_error, final_xtd) = if raw_heading_error.abs() > 90.0 {
                    // Return pass: flip bearing and negate XTE, then apply nudge
                    let return_error = normalise_angle(raw_heading_error - 180.0);
                    (return_error, -pass_xtd - self.nudge_m)
                } else {
                    // Forward pass: apply nudge directly
                    (raw_heading_error, pass_xtd - self.nudge_m)
                };

                Some(CrossTrackError {
                    distance_m: final_xtd,
                    heading_error,
                })
            }
            _ => None,
        }
    }
}

/// Normalise an angle to -180..+180 range
fn normalise_angle(mut angle: f64) -> f64 {
    while angle > 180.0 {
        angle -= 360.0;
    }
    while angle < -180.0 {
        angle += 360.0;
    }
    angle
}

/// Raw A/B point coordinates returned by `AbLineGuide::ab_points()`.
#[derive(Debug, Clone, Copy)]
pub struct AbPoints {
    pub a_lat: f64,
    pub a_lon: f64,
    pub b_lat: f64,
    pub b_lon: f64,
}
