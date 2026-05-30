//! AB Line guidance - set two points, get cross-track error for any position.
//! Includes auto-pass selection: monitors cross-track distance and snaps to
//! the nearest pass line when the operator has moved to a different one.
//!
//! ## Sign convention (single source of truth)
//!
//! Everything here is built from two primitives:
//!
//!   - [`AbLineGuide::signed_line_offset`] — the signed perpendicular distance
//!     from the *current target track* (pass line + nudge), measured in the
//!     fixed A→B frame. This is **direction-independent**: it does not change
//!     when the tractor turns around for a return pass. Its sign follows
//!     `coords::cross_track_distance_local`.
//!   - [`AbLineGuide::travel_sign`] — `+1.0` driving roughly A→B (forward),
//!     `-1.0` driving roughly B→A (return).
//!
//! The cross-track error the controller consumes is simply
//! `travel_sign * signed_line_offset`, re-expressed into the vehicle's own
//! travel frame so pure pursuit sees a consistent signal in both directions.
//! The resulting `CrossTrackError::distance_m` matches the convention
//! documented in `steering.rs`: **positive = vehicle is LEFT of the line,
//! negative = vehicle is RIGHT of the line**.
//!
//! Anything that needs "where am I relative to the line" (lightbar, nudge
//! alignment, nearest-pass selection, pure pursuit) derives from these two
//! primitives rather than re-deriving the direction flip locally. Adding a
//! new consumer must not reintroduce a local sign flip.

use finn_guidance_common::coords;
use finn_guidance_common::types::{CrossTrackError, GpsFix, GuidanceLine};

/// Distinct-point epsilon (degrees). Below this, A and B are treated as the
/// same point and no line geometry is computed.
const DISTINCT_POINT_EPS: f64 = 1e-10;

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
    /// Positive = shift the target track toward the positive `signed_line_offset`
    /// side (i.e. the +`cross_track_distance_local` side of A→B).
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
        self.nudge_m = 0.0;
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
        self.ab_points().is_some()
    }

    /// Returns the raw A/B coordinates if a complete line is set.
    pub fn ab_points(&self) -> Option<AbPoints> {
        match &self.line {
            Some(GuidanceLine::AbLine { a, b }) if points_distinct(*a, *b) => Some(AbPoints {
                a_lat: a.0,
                a_lon: a.1,
                b_lat: b.0,
                b_lon: b.1,
            }),
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

    // ─────────────────────────────────────────────────────────────────
    // Primitives — the single source of truth for line geometry & sign
    // ─────────────────────────────────────────────────────────────────

    /// Internal: raw geometry for the current fix in the fixed A→B frame.
    /// Returns `(raw_xtd, raw_heading_error)` or `None` if the line is
    /// missing or degenerate.
    ///
    /// - `raw_xtd`: signed cross-track distance from the *base* A→B line
    ///   (before any pass offset or nudge). Sign per
    ///   `coords::cross_track_distance_local`.
    /// - `raw_heading_error`: `fix.heading - line_bearing`, normalised to
    ///   ±180°. Used only to decide travel direction; the controller's
    ///   reported heading error is derived from this in `calculate_error`.
    fn line_geometry(&self, fix: &GpsFix) -> Option<(f64, f64)> {
        match &self.line {
            Some(GuidanceLine::AbLine { a, b }) if points_distinct(*a, *b) => {
                let raw_xtd = coords::cross_track_distance_local(
                    fix.latitude,
                    fix.longitude,
                    a.0,
                    a.1,
                    b.0,
                    b.1,
                );
                let line_bearing = coords::bearing_local(a.0, a.1, b.0, b.1);
                let raw_heading_error = normalise_angle(fix.heading - line_bearing);
                Some((raw_xtd, raw_heading_error))
            }
            _ => None,
        }
    }

    /// Signed perpendicular offset from the **current target track**
    /// (pass line + nudge), in the fixed A→B frame.
    ///
    /// This is THE primitive. It is direction-independent — it does not
    /// depend on which way the tractor is travelling — so it is the right
    /// quantity for anything that reasons about world position: nudge
    /// alignment, nearest-pass selection, coverage, etc. Zero means the
    /// vehicle is exactly on the target track for the current pass.
    ///
    /// Sign follows `coords::cross_track_distance_local`. Returns `None` if
    /// no complete line is set.
    pub fn signed_line_offset(&self, fix: &GpsFix) -> Option<f64> {
        let (raw_xtd, _) = self.line_geometry(fix)?;
        Some(raw_xtd - self.pass_offset_m - self.nudge_m)
    }

    /// Travel direction relative to the AB line: `+1.0` when driving roughly
    /// A→B (forward), `-1.0` when driving roughly B→A (return pass).
    ///
    /// Determined by whether the heading error exceeds 90°. Returns `None` if
    /// no complete line is set.
    pub fn travel_sign(&self, fix: &GpsFix) -> Option<f64> {
        let (_, raw_heading_error) = self.line_geometry(fix)?;
        Some(if raw_heading_error.abs() > 90.0 {
            -1.0
        } else {
            1.0
        })
    }

    // ─────────────────────────────────────────────────────────────────
    // Nudge
    // ─────────────────────────────────────────────────────────────────

    /// Shift the AB line system laterally so the current GPS position reads
    /// exactly zero cross-track error on the current pass line.
    ///
    /// This is the "Snap line to me" function — compensates for GPS drift.
    /// Example: seeder is physically on the mark but XTE shows +1.0m.
    /// Tapping this absorbs that offset into `nudge_m` so XTE drops to zero
    /// instantly. The saved AB line geometry and pass number are unchanged.
    ///
    /// Because the target offset (`signed_line_offset`) is direction-
    /// independent, this needs **no** heading logic: it simply absorbs the
    /// current offset into the nudge. It zeroes the operator's displayed XTE
    /// correctly on both forward and return passes.
    ///
    /// Returns the applied shift in metres (after clamping), or `None` if no
    /// complete line is loaded.
    pub fn align_grid_to_position(&mut self, fix: &GpsFix) -> Option<f64> {
        let offset = self.signed_line_offset(fix)?;
        let cap = self.implement_width_m;
        let old_nudge = self.nudge_m;
        // signed_line_offset == raw_xtd - pass_offset - nudge, so adding it to
        // the current nudge sets nudge = raw_xtd - pass_offset, which makes the
        // new signed_line_offset (and hence the displayed XTE) zero.
        self.nudge_m = (self.nudge_m + offset).clamp(-cap, cap);
        Some(self.nudge_m - old_nudge)
    }

    /// Shift the AB line toward the negative `signed_line_offset` side by
    /// `amount_m` metres (all passes shift with it).
    /// Hard cap at ±implement_width_m to allow full pass-width shifts.
    ///
    /// Note: this is world-fixed in the A→B frame. The GUI is responsible for
    /// mapping the operator's cab-relative "left/right" intent onto these
    /// using `travel_sign`.
    pub fn nudge_right(&mut self, amount_m: f64) {
        let cap = self.implement_width_m;
        self.nudge_m = (self.nudge_m - amount_m).clamp(-cap, cap);
    }

    /// Shift the AB line toward the positive `signed_line_offset` side by
    /// `amount_m` metres.
    pub fn nudge_left(&mut self, amount_m: f64) {
        let cap = self.implement_width_m;
        self.nudge_m = (self.nudge_m + amount_m).clamp(-cap, cap);
    }

    /// Reset nudge back to zero.
    pub fn nudge_reset(&mut self) {
        self.nudge_m = 0.0;
    }

    // ─────────────────────────────────────────────────────────────────
    // Pass selection
    // ─────────────────────────────────────────────────────────────────

    /// Shift to the next pass (positive direction)
    pub fn next_pass(&mut self) {
        self.pass_number += 1;
        self.pass_offset_m = self.pass_spacing() * self.pass_number as f64;
    }

    /// Shift to the previous pass (negative direction)
    pub fn prev_pass(&mut self) {
        self.pass_number -= 1;
        self.pass_offset_m = self.pass_spacing() * self.pass_number as f64;
    }

    /// Snap the pass grid so the current position falls on the nearest whole
    /// pass line, and clear the nudge. The saved AB line geometry is unchanged
    /// — only the pass number (and offset) move.
    ///
    /// Distinct from [`align_grid_to_position`](Self::align_grid_to_position):
    /// that zeroes the XTE *exactly* by nudging (sub-pass correction); this
    /// renumbers to the nearest whole pass and drops the nudge (coarse,
    /// pass-level). Use when changing implements or re-establishing the grid
    /// from a fence line. Returns the new pass number, or `None` if no
    /// complete line is set.
    pub fn snap_to_nearest_pass(&mut self, fix: &GpsFix) -> Option<i32> {
        let (raw_xtd, _) = self.line_geometry(fix)?;
        // Nearest pass on the *un-nudged* grid, since we are clearing the nudge.
        let new_pass = (raw_xtd / self.pass_spacing()).round() as i32;
        self.pass_number = new_pass;
        self.pass_offset_m = self.pass_spacing() * new_pass as f64;
        self.nudge_m = 0.0;
        Some(new_pass)
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

        // Distance from the current target track (direction-independent magnitude).
        let offset = self.signed_line_offset(fix)?;

        // Check if we've drifted far enough to be on a different pass
        let threshold_m = self.pass_spacing() * self.snap_threshold;
        if offset.abs() < threshold_m {
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
    /// Works in the fixed A→B frame (direction-independent): converts the raw
    /// cross-track distance, less the nudge, to the nearest whole pass number.
    fn find_nearest_pass(&self, fix: &GpsFix) -> i32 {
        match self.line_geometry(fix) {
            Some((raw_xtd, _)) => {
                // Subtract nudge before rounding so the nearest pass is found
                // relative to the nudged line system, not the raw AB origin.
                let nudge_adjusted = raw_xtd - self.nudge_m;
                (nudge_adjusted / self.pass_spacing()).round() as i32
            }
            None => self.pass_number,
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // Cross-track error (the controller-facing output)
    // ─────────────────────────────────────────────────────────────────

    /// Calculate cross-track error from the current (offset + nudged)
    /// guidance line, expressed in the vehicle's own travel frame.
    ///
    /// `distance_m = travel_sign * signed_line_offset`. On a return pass the
    /// reference bearing is flipped 180° so `heading_error` stays small and
    /// pure pursuit steers correctly. See the module-level sign convention.
    pub fn calculate_error(&self, fix: &GpsFix) -> Option<CrossTrackError> {
        let (raw_xtd, raw_heading_error) = self.line_geometry(fix)?;

        // Direction-independent offset from the current target track.
        let line_offset = raw_xtd - self.pass_offset_m - self.nudge_m;

        // Travel direction. On a return pass both the cross-track sign and the
        // reference bearing invert together.
        let is_return = raw_heading_error.abs() > 90.0;
        let direction_sign = if is_return { -1.0 } else { 1.0 };

        let distance_m = direction_sign * line_offset;
        let heading_error = if is_return {
            normalise_angle(raw_heading_error - 180.0)
        } else {
            raw_heading_error
        };

        Some(CrossTrackError {
            distance_m,
            heading_error,
            is_return_pass: is_return,
        })
    }
}

/// True if A and B are distinct enough to define a line direction.
fn points_distinct(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() > DISTINCT_POINT_EPS || (a.1 - b.1).abs() > DISTINCT_POINT_EPS
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

#[cfg(test)]
mod tests {
    use super::*;
    use finn_guidance_common::types::FixQuality;

    // A straight North–South reference line near Jamestown SA.
    // A is the SOUTH end, B is the NORTH end, so the A→B bearing is ~0° (north).
    // Driving forward = heading ~0°, return pass = heading ~180°.
    const A_LAT: f64 = -33.0100;
    const A_LON: f64 = 138.6000;
    const B_LAT: f64 = -33.0000;
    const B_LON: f64 = 138.6000;
    const MID_LAT: f64 = -33.0050;

    // ~9.3 m of easting per 0.0001° longitude at this latitude.
    const LON_EAST: f64 = 138.6001; // east of the line
    const LON_WEST: f64 = 138.5999; // west of the line

    const FWD: f64 = 0.0; // heading driving A→B (north)
    const RET: f64 = 180.0; // heading driving B→A (south)

    fn fix_at(lat: f64, lon: f64, heading: f64) -> GpsFix {
        GpsFix {
            latitude: lat,
            longitude: lon,
            altitude: 0.0,
            speed: 2.0,
            heading,
            fix_quality: FixQuality::Rtk,
            satellites: 12,
            hdop: 0.8,
            timestamp_ms: 0,
            roll: 0.0,
            pitch: 0.0,
            roll_corr_m: 0.0,
            diag_vtg_heading: f64::NAN,
            diag_ins_heading: f64::NAN,
        }
    }

    fn guide() -> AbLineGuide {
        let mut g = AbLineGuide::new(12.0); // 12 m implement, 0 overlap → 12 m spacing
        g.load_ab_line(A_LAT, A_LON, B_LAT, B_LON);
        g
    }

    #[test]
    fn on_line_reads_zero() {
        let g = guide();
        let e = g.calculate_error(&fix_at(MID_LAT, A_LON, FWD)).unwrap();
        assert!(e.distance_m.abs() < 0.05, "on-line XTE was {}", e.distance_m);
        assert!(!e.is_return_pass);
    }

    #[test]
    fn east_of_northbound_line_is_right_negative() {
        // East of a northbound line is to the driver's right → negative XTE
        // (steering.rs convention: negative = right of line).
        let g = guide();
        let e = g.calculate_error(&fix_at(MID_LAT, LON_EAST, FWD)).unwrap();
        assert!(e.distance_m < 0.0, "east-of-line XTE was {}", e.distance_m);
    }

    #[test]
    fn west_of_northbound_line_is_left_positive() {
        let g = guide();
        let e = g.calculate_error(&fix_at(MID_LAT, LON_WEST, FWD)).unwrap();
        assert!(e.distance_m > 0.0, "west-of-line XTE was {}", e.distance_m);
    }

    #[test]
    fn return_pass_detected_and_xte_flips_sign() {
        // Same physical position (east of line). Forward it reads negative
        // (right of line); on the return pass the same spot is to the cab's
        // LEFT, so the reported XTE flips sign but keeps the same magnitude.
        let g = guide();
        let fwd = g.calculate_error(&fix_at(MID_LAT, LON_EAST, FWD)).unwrap();
        let ret = g.calculate_error(&fix_at(MID_LAT, LON_EAST, RET)).unwrap();

        assert!(!fwd.is_return_pass);
        assert!(ret.is_return_pass);
        assert!(ret.distance_m > 0.0, "return XTE should flip to +, was {}", ret.distance_m);
        assert!(
            (fwd.distance_m + ret.distance_m).abs() < 1e-6,
            "forward/return XTE should be equal & opposite: {} vs {}",
            fwd.distance_m,
            ret.distance_m
        );
        // Heading error stays small on the return pass (bearing flipped 180°).
        assert!(ret.heading_error.abs() < 1.0, "return heading_error {}", ret.heading_error);
    }

    #[test]
    fn signed_line_offset_is_direction_independent() {
        let g = guide();
        let off_fwd = g.signed_line_offset(&fix_at(MID_LAT, LON_EAST, FWD)).unwrap();
        let off_ret = g.signed_line_offset(&fix_at(MID_LAT, LON_EAST, RET)).unwrap();
        assert!(
            (off_fwd - off_ret).abs() < 1e-9,
            "signed_line_offset must not depend on heading: {} vs {}",
            off_fwd,
            off_ret
        );
    }

    #[test]
    fn travel_sign_forward_and_return() {
        let g = guide();
        assert_eq!(g.travel_sign(&fix_at(MID_LAT, A_LON, FWD)).unwrap(), 1.0);
        assert_eq!(g.travel_sign(&fix_at(MID_LAT, A_LON, RET)).unwrap(), -1.0);
    }

    #[test]
    fn align_zeroes_xte_forward() {
        let mut g = guide();
        let fix = fix_at(MID_LAT, LON_EAST, FWD);
        let shift = g.align_grid_to_position(&fix);
        assert!(shift.is_some());
        let e = g.calculate_error(&fix).unwrap();
        assert!(e.distance_m.abs() < 1e-6, "post-align XTE was {}", e.distance_m);
    }

    #[test]
    fn align_zeroes_xte_return() {
        // The bug this refactor fixes: on a return pass the old code set the
        // nudge with the wrong sign and DOUBLED the error. After the fix the
        // displayed XTE must be ~0 on a return pass too.
        let mut g = guide();
        let fix = fix_at(MID_LAT, LON_EAST, RET);
        let shift = g.align_grid_to_position(&fix);
        assert!(shift.is_some());
        let e = g.calculate_error(&fix).unwrap();
        assert!(
            e.distance_m.abs() < 1e-6,
            "post-align return XTE was {} (should be ~0)",
            e.distance_m
        );
    }

    #[test]
    fn align_is_direction_independent() {
        // Aligning at the same physical spot must produce the same nudge
        // regardless of travel direction.
        let mut g_fwd = guide();
        g_fwd.align_grid_to_position(&fix_at(MID_LAT, LON_EAST, FWD));

        let mut g_ret = guide();
        g_ret.align_grid_to_position(&fix_at(MID_LAT, LON_EAST, RET));

        assert!(
            (g_fwd.nudge_m - g_ret.nudge_m).abs() < 1e-9,
            "align nudge differs by direction: fwd {} vs ret {}",
            g_fwd.nudge_m,
            g_ret.nudge_m
        );
    }

    #[test]
    fn next_pass_shifts_target_by_spacing() {
        let mut g = guide();
        let spacing = g.pass_spacing();
        g.next_pass();
        assert_eq!(g.pass_number, 1);
        assert!((g.pass_offset_m - spacing).abs() < 1e-9);

        // A vehicle still on the base AB line is now one full spacing off the
        // pass-1 line. Magnitude ≈ spacing.
        let e = g.calculate_error(&fix_at(MID_LAT, A_LON, FWD)).unwrap();
        assert!(
            (e.distance_m.abs() - spacing).abs() < 0.05,
            "expected ~{} m off pass 1, got {}",
            spacing,
            e.distance_m
        );
    }

    #[test]
    fn nudge_left_right_are_opposite() {
        let mut g = guide();
        g.nudge_right(0.5);
        let after_right = g.nudge_m;
        g.nudge_left(0.5); // back to zero
        assert!(g.nudge_m.abs() < 1e-9);
        assert!(after_right < 0.0, "nudge_right should move toward -offset side");
    }

    #[test]
    fn nudge_clamps_to_implement_width() {
        let mut g = guide();
        g.nudge_left(1000.0);
        assert!((g.nudge_m - g.implement_width_m).abs() < 1e-9);
        g.nudge_right(1000.0);
        assert!((g.nudge_m + g.implement_width_m).abs() < 1e-9);
    }

    #[test]
    fn auto_pass_snaps_to_nearest_when_drifted() {
        // Park ~9.3 m east of pass 0 (closer to pass -1 at -12 m than to 0).
        let mut g = guide();
        let fix = fix_at(MID_LAT, LON_EAST, FWD);
        let ev = g.update_auto_pass(&fix);
        assert!(ev.is_some(), "should have snapped");
        let ev = ev.unwrap();
        assert_eq!(ev.old_pass, 0);
        assert_eq!(ev.new_pass, -1, "nearest pass to ~9.3 m east should be -1");
        assert_eq!(g.pass_number, -1);
    }

    #[test]
    fn auto_pass_no_snap_when_on_line() {
        let mut g = guide();
        let ev = g.update_auto_pass(&fix_at(MID_LAT, A_LON, FWD));
        assert!(ev.is_none(), "on-line should not snap");
        assert_eq!(g.pass_number, 0);
    }

    #[test]
    fn auto_pass_ignores_standstill() {
        let mut g = guide();
        let mut fix = fix_at(MID_LAT, LON_EAST, FWD);
        fix.speed = 0.0;
        assert!(g.update_auto_pass(&fix).is_none(), "must not snap at standstill");
    }

    #[test]
    fn degenerate_line_yields_no_error() {
        let mut g = AbLineGuide::new(12.0);
        // A == B
        g.load_ab_line(A_LAT, A_LON, A_LAT, A_LON);
        assert!(!g.has_complete_line());
        assert!(g.calculate_error(&fix_at(MID_LAT, A_LON, FWD)).is_none());
        assert!(g.signed_line_offset(&fix_at(MID_LAT, A_LON, FWD)).is_none());
    }

    #[test]
    fn snap_to_nearest_pass_renumbers_and_clears_nudge() {
        // ~9.3 m east of pass 0 → nearest whole pass is -1 (at -12 m).
        let mut g = guide();
        g.nudge_left(0.30); // pre-existing nudge that must be cleared
        let fix = fix_at(MID_LAT, LON_EAST, FWD);
        let new_pass = g.snap_to_nearest_pass(&fix);
        assert_eq!(new_pass, Some(-1));
        assert_eq!(g.pass_number, -1);
        assert!(g.nudge_m.abs() < 1e-9, "nudge should be cleared, was {}", g.nudge_m);
        // Residual XTE is the operator's offset from the nearest row: ≤ half a spacing.
        let e = g.calculate_error(&fix).unwrap();
        assert!(e.distance_m.abs() <= g.pass_spacing() / 2.0 + 0.01);
    }

    #[test]
    fn snap_to_nearest_pass_is_direction_independent() {
        let mut g_fwd = guide();
        g_fwd.snap_to_nearest_pass(&fix_at(MID_LAT, LON_EAST, FWD));
        let mut g_ret = guide();
        g_ret.snap_to_nearest_pass(&fix_at(MID_LAT, LON_EAST, RET));
        assert_eq!(g_fwd.pass_number, g_ret.pass_number);
    }
}
