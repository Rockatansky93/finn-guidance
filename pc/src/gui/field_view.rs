//! Field view canvas - layered 2D rendering of the guidance display.
//!
//! Draws on the egui canvas using the Painter API with layers:
//!   - Layer 0: Background (future: drone orthomosaic)
//!   - Layer 1: Grid overlay for spatial reference
//!   - Layer 2: Guidance lines (AB line, parallel passes)
//!   - Layer 3: Vehicle position, trail, and heading indicator
//!
//! Each layer draws independently using the shared FieldProjection.

use std::collections::VecDeque;
use eframe::egui;
use finn_guidance_common::types::{GpsFix, FixQuality, GuidanceLine};
use crate::coverage::logger::CoveragePoint;
use super::field_projection::FieldProjection;

/// Colours used across the field view
struct FieldColours;

impl FieldColours {
    const BACKGROUND: egui::Color32 = egui::Color32::from_rgb(30, 35, 30);
    const GRID_MAJOR: egui::Color32 = egui::Color32::from_rgba_premultiplied(80, 90, 80, 180);
    const GRID_MINOR: egui::Color32 = egui::Color32::from_rgba_premultiplied(50, 58, 50, 120);
    const TRAIL: egui::Color32 = egui::Color32::from_rgb(100, 180, 255);
    const TRAIL_OLD: egui::Color32 = egui::Color32::from_rgb(40, 70, 100);
    const VEHICLE: egui::Color32 = egui::Color32::from_rgb(255, 255, 255);
    const AB_LINE: egui::Color32 = egui::Color32::from_rgb(255, 50, 50);
    const PASS_LINE: egui::Color32 = egui::Color32::from_rgba_premultiplied(255, 50, 50, 80);
    const CURRENT_PASS: egui::Color32 = egui::Color32::from_rgb(80, 160, 255);
    // Coverage strip colours by fix quality (semi-transparent so grid/lines show through)
    const COVERAGE_RTK: egui::Color32 = egui::Color32::from_rgba_premultiplied(30, 160, 30, 90);
    const COVERAGE_FLOAT: egui::Color32 = egui::Color32::from_rgba_premultiplied(180, 180, 30, 90);
    const COVERAGE_DGPS: egui::Color32 = egui::Color32::from_rgba_premultiplied(30, 120, 180, 90);
    const COVERAGE_GPS: egui::Color32 = egui::Color32::from_rgba_premultiplied(200, 120, 30, 90);
    const COVERAGE_NOFIX: egui::Color32 = egui::Color32::from_rgba_premultiplied(180, 30, 30, 90);
}

/// The field view canvas widget.
pub struct FieldView {
    pub projection: FieldProjection,
    /// Whether to lock the view to the vehicle (heading-up, centred)
    pub follow_vehicle: bool,
    /// Whether to draw the grid
    pub show_grid: bool,
    /// Whether to use heading-up mode (vs north-up)
    pub heading_up: bool,
}

impl FieldView {
    pub fn new() -> Self {
        Self {
            projection: FieldProjection::new(),
            follow_vehicle: true,
            show_grid: true,
            heading_up: true,
        }
    }

    /// Main draw method - renders the field view into an egui Ui area.
    ///
    /// Call this from the central panel's UI code.
    pub fn draw(
        &mut self,
        ui: &mut egui::Ui,
        current_fix: &Option<GpsFix>,
        trail: &VecDeque<(f64, f64)>,
        guide: &crate::guidance::ab_line::AbLineGuide,
        coverage_points: &[CoveragePoint],
        implement_width_m: f64,
    ) {
        // Allocate the available space for our canvas
        let available = ui.available_size();
        let (response, painter) = ui.allocate_painter(available, egui::Sense::click_and_drag());
        let rect = response.rect;

        // Update projection screen size
        self.projection.set_screen_size(rect.width(), rect.height());

        // Set up reference point from first GPS fix
        if let Some(fix) = current_fix {
            if !self.projection.has_reference() {
                self.projection.set_reference(fix.latitude, fix.longitude);
            }

            // If following the vehicle, centre on it and rotate to heading
            if self.follow_vehicle {
                let (lx, ly) = self.projection.world_to_local(fix.latitude, fix.longitude);
                self.projection.camera_offset_x = lx;
                self.projection.camera_offset_y = ly;

                if self.heading_up && fix.speed > 0.3 {
                    // Only rotate when moving (avoids jitter when stationary)
                    self.projection.set_heading(fix.heading);
                }
            }
        }

        // Handle scroll wheel for zoom
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll > 0.0 {
                self.projection.zoom_in();
            } else if scroll < 0.0 {
                self.projection.zoom_out();
            }
        }

        // === Layer 0: Background ===
        painter.rect_filled(rect, 0.0, FieldColours::BACKGROUND);
        // Future: draw drone orthomosaic here using painter.image()

        // === Layer 1: Grid ===
        if self.show_grid {
            self.draw_grid(&painter, rect);
        }

        // === Layer 1.5: Coverage strips ===
        if !coverage_points.is_empty() {
            self.draw_coverage(&painter, rect, coverage_points, implement_width_m);
        }

        // === Layer 2: Guidance lines ===
        self.draw_guidance_lines(&painter, guide);

        // === Layer 3: Trail and vehicle ===
        self.draw_trail(&painter, trail);
        if let Some(fix) = current_fix {
            self.draw_vehicle(&painter, fix);
        }

        // === Overlay: Scale indicator ===
        self.draw_scale_bar(&painter, rect);
    }

    /// Layer 1: Draw reference grid lines
    fn draw_grid(&self, painter: &egui::Painter, _rect: egui::Rect) {
        let spacing = self.projection.grid_spacing_m();

        // Compute a generous bound of visible local space
        let visible_half = self.projection.visible_width_m() * 0.75;
        let cx = self.projection.camera_offset_x;
        let cy = self.projection.camera_offset_y;

        // Snap grid start to spacing interval
        let grid_min_x = ((cx - visible_half) / spacing).floor() as i64;
        let grid_max_x = ((cx + visible_half) / spacing).ceil() as i64;
        let grid_min_y = ((cy - visible_half) / spacing).floor() as i64;
        let grid_max_y = ((cy + visible_half) / spacing).ceil() as i64;

        // Draw vertical grid lines (constant X)
        for ix in grid_min_x..=grid_max_x {
            let x = ix as f64 * spacing;
            let is_origin = ix == 0;
            let colour = if is_origin { FieldColours::GRID_MAJOR } else { FieldColours::GRID_MINOR };
            let width = if is_origin { 1.5 } else { 0.5 };

            // Draw a line segment from bottom to top of visible area
            let (sx1, sy1) = self.projection.local_to_screen(x, (grid_min_y as f64) * spacing);
            let (sx2, sy2) = self.projection.local_to_screen(x, (grid_max_y as f64) * spacing);

            painter.line_segment(
                [egui::pos2(sx1, sy1), egui::pos2(sx2, sy2)],
                egui::Stroke::new(width, colour),
            );
        }

        // Draw horizontal grid lines (constant Y)
        for iy in grid_min_y..=grid_max_y {
            let y = iy as f64 * spacing;
            let is_origin = iy == 0;
            let colour = if is_origin { FieldColours::GRID_MAJOR } else { FieldColours::GRID_MINOR };
            let width = if is_origin { 1.5 } else { 0.5 };

            let (sx1, sy1) = self.projection.local_to_screen((grid_min_x as f64) * spacing, y);
            let (sx2, sy2) = self.projection.local_to_screen((grid_max_x as f64) * spacing, y);

            painter.line_segment(
                [egui::pos2(sx1, sy1), egui::pos2(sx2, sy2)],
                egui::Stroke::new(width, colour),
            );
        }
    }

    /// Layer 2: Draw AB line and parallel pass lines
    fn draw_guidance_lines(
        &self,
        painter: &egui::Painter,
        guide: &crate::guidance::ab_line::AbLineGuide,
    ) {
        let line = match &guide.line {
            Some(GuidanceLine::AbLine { a, b }) => {
                // Don't draw if A == B
                if (a.0 - b.0).abs() < 1e-10 && (a.1 - b.1).abs() < 1e-10 {
                    return;
                }
                (a, b)
            }
            _ => return,
        };

        let (a, b) = line;
        let (ax, ay) = self.projection.world_to_local(a.0, a.1);
        let (bx, by) = self.projection.world_to_local(b.0, b.1);

        // Direction vector of the AB line
        let dx = bx - ax;
        let dy = by - ay;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 0.01 {
            return;
        }
        let ux = dx / len; // Unit vector along line
        let uy = dy / len;
        let nx = uy;  // Normal vector (perpendicular, pointing right of A→B direction)
        let ny = -ux; // Must match cross_track_distance sign convention: positive = right

        // Extend the line far enough to cross the visible area
        let extend = self.projection.visible_width_m();

        // Draw several pass lines on each side
        let max_passes = 20;
        let pass_spacing = guide.pass_spacing();
        for pass in -max_passes..=max_passes {
            // Nudge shifts the entire line system laterally.
            // Add nudge_m to the pass offset so all lines move together.
            let offset = pass as f64 * pass_spacing + guide.nudge_m;
            let ox = ax + nx * offset;
            let oy = ay + ny * offset;

            // Line endpoints extended in both directions
            let p1x = ox - ux * extend;
            let p1y = oy - uy * extend;
            let p2x = ox + ux * extend;
            let p2y = oy + uy * extend;

            let (sx1, sy1) = self.projection.local_to_screen(p1x, p1y);
            let (sx2, sy2) = self.projection.local_to_screen(p2x, p2y);

            let (colour, width) = if pass == guide.pass_number {
                (FieldColours::CURRENT_PASS, 3.0)
            } else if pass == 0 {
                (FieldColours::AB_LINE, 1.5)
            } else {
                (FieldColours::PASS_LINE, 0.8)
            };

            painter.line_segment(
                [egui::pos2(sx1, sy1), egui::pos2(sx2, sy2)],
                egui::Stroke::new(width, colour),
            );
        }

        // Draw A and B point markers
        let (sa_x, sa_y) = self.projection.local_to_screen(ax, ay);
        let (sb_x, sb_y) = self.projection.local_to_screen(bx, by);

        painter.circle_filled(egui::pos2(sa_x, sa_y), 6.0, egui::Color32::RED);
        painter.text(
            egui::pos2(sa_x + 10.0, sa_y - 10.0),
            egui::Align2::LEFT_BOTTOM,
            "A",
            egui::FontId::proportional(14.0),
            egui::Color32::WHITE,
        );

        painter.circle_filled(egui::pos2(sb_x, sb_y), 6.0, egui::Color32::RED);
        painter.text(
            egui::pos2(sb_x + 10.0, sb_y - 10.0),
            egui::Align2::LEFT_BOTTOM,
            "B",
            egui::FontId::proportional(14.0),
            egui::Color32::WHITE,
        );
    }

    /// Layer 1.5: Draw coverage strips showing where the implement has been.
    ///
    /// Each coverage point becomes a small rectangle oriented along the vehicle's
    /// heading at that point, with width = implement width. Consecutive points
    /// form a continuous painted swath. Points are colour-coded by GPS fix quality.
    ///
    /// Zoom-dependent thinning: when zoomed out, individual metre-resolution strips
    /// overlap into a solid mass — we skip points to reduce draw calls without any
    /// visible difference. At close zoom, every point is drawn for full fidelity.
    /// When thinning, quads bridge directly from point[i] to point[i+step] so the
    /// coverage remains a continuous band with no gaps.
    fn draw_coverage(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        points: &[CoveragePoint],
        implement_width_m: f64,
    ) {
        let half_width = implement_width_m / 2.0;
        // Screen-space margin for viewport culling (pixels outside the visible rect to skip)
        let margin = 50.0;
        let cull_rect = rect.expand(margin);

        // Zoom-dependent render thinning: at wide zoom levels, individual 1m strips
        // are sub-pixel and overlap into a solid mass. Skip points we can't see.
        // Visible width < 100m → draw every point (step=1)
        // Visible width > 500m → draw every 4th point (step=4)
        // Linear ramp between those thresholds.
        let visible_w = self.projection.visible_width_m();
        let step = if visible_w <= 100.0 {
            1usize
        } else if visible_w >= 500.0 {
            4usize
        } else {
            let t = (visible_w - 100.0) / 400.0; // 0.0..1.0
            (1.0 + t * 3.0).round() as usize // 1..4
        };

        let mut i = 0;
        while i < points.len() {
            let pt = &points[i];

            let colour = match pt.fix_quality {
                FixQuality::Rtk => FieldColours::COVERAGE_RTK,
                FixQuality::RtkFloat => FieldColours::COVERAGE_FLOAT,
                FixQuality::Dgps => FieldColours::COVERAGE_DGPS,
                FixQuality::Gps => FieldColours::COVERAGE_GPS,
                FixQuality::NoFix => FieldColours::COVERAGE_NOFIX,
            };

            // Quick screen-space check: skip points whose centre is way off screen
            let (cx, cy) = self.projection.world_to_screen(pt.latitude, pt.longitude);
            if !cull_rect.contains(egui::pos2(cx, cy)) {
                i += step;
                continue;
            }

            // Build a strip segment between this point and the next rendered point.
            // When step > 1 (zoomed out), we bridge directly to point[i+step] so
            // the coverage band stays continuous — no gaps from skipping intermediate
            // points. Within the same segment only.
            let (lx, ly) = self.projection.world_to_local(pt.latitude, pt.longitude);

            let heading_rad = pt.heading * std::f64::consts::PI / 180.0;
            let cos_h = heading_rad.cos();
            let sin_h = heading_rad.sin();

            // Perpendicular direction (left/right of travel)
            let perp_x = cos_h; // cos(heading) points east-of-north perpendicular
            let perp_y = -sin_h;
            // Note: heading 0° = north, so forward = (sin(h), cos(h)) in local coords
            // and perpendicular (to the right) = (cos(h), -sin(h))

            // Find the next point to bridge to: step ahead, but must be same segment
            let next_idx = i + step;
            let bridge_target = if next_idx < points.len() && points[next_idx].segment == pt.segment {
                Some(next_idx)
            } else if i + 1 < points.len() && points[i + 1].segment == pt.segment {
                // Step landed outside segment or past end — fall back to i+1
                Some(i + 1)
            } else {
                None
            };

            if let Some(ni) = bridge_target {
                // Draw a quad from this point to the bridge target
                let next = &points[ni];
                let (nlx, nly) = self.projection.world_to_local(next.latitude, next.longitude);

                // Four corners: left/right of current point, left/right of next point
                let c1 = self.projection.local_to_screen(
                    lx - perp_x * half_width, ly - perp_y * half_width,
                );
                let c2 = self.projection.local_to_screen(
                    lx + perp_x * half_width, ly + perp_y * half_width,
                );

                // Use next point's heading for the far end
                let nh_rad = next.heading * std::f64::consts::PI / 180.0;
                let ncos = nh_rad.cos();
                let nsin = nh_rad.sin();
                let nperp_x = ncos;
                let nperp_y = -nsin;

                let c3 = self.projection.local_to_screen(
                    nlx + nperp_x * half_width, nly + nperp_y * half_width,
                );
                let c4 = self.projection.local_to_screen(
                    nlx - nperp_x * half_width, nly - nperp_y * half_width,
                );

                painter.add(egui::Shape::convex_polygon(
                    vec![
                        egui::pos2(c1.0, c1.1),
                        egui::pos2(c2.0, c2.1),
                        egui::pos2(c3.0, c3.1),
                        egui::pos2(c4.0, c4.1),
                    ],
                    colour,
                    egui::Stroke::NONE,
                ));
            } else {
                // Last point in a segment or isolated point — draw a small square
                let along_x = sin_h;
                let along_y = cos_h;
                let half_step = 0.5; // 0.5m forward/back

                let c1 = self.projection.local_to_screen(
                    lx - perp_x * half_width - along_x * half_step,
                    ly - perp_y * half_width - along_y * half_step,
                );
                let c2 = self.projection.local_to_screen(
                    lx + perp_x * half_width - along_x * half_step,
                    ly + perp_y * half_width - along_y * half_step,
                );
                let c3 = self.projection.local_to_screen(
                    lx + perp_x * half_width + along_x * half_step,
                    ly + perp_y * half_width + along_y * half_step,
                );
                let c4 = self.projection.local_to_screen(
                    lx - perp_x * half_width + along_x * half_step,
                    ly - perp_y * half_width + along_y * half_step,
                );

                painter.add(egui::Shape::convex_polygon(
                    vec![
                        egui::pos2(c1.0, c1.1),
                        egui::pos2(c2.0, c2.1),
                        egui::pos2(c3.0, c3.1),
                        egui::pos2(c4.0, c4.1),
                    ],
                    colour,
                    egui::Stroke::NONE,
                ));
            }

            i += step;
        }
    }

    /// Layer 3: Draw the position trail
    fn draw_trail(&self, painter: &egui::Painter, trail: &VecDeque<(f64, f64)>) {
        if trail.len() < 2 {
            return;
        }

        let total = trail.len();
        for i in 1..total {
            let (lat1, lon1) = trail[i - 1];
            let (lat2, lon2) = trail[i];

            let (sx1, sy1) = self.projection.world_to_screen(lat1, lon1);
            let (sx2, sy2) = self.projection.world_to_screen(lat2, lon2);

            // Fade older trail points
            let age = (total - i) as f32 / total as f32; // 0.0 = newest, 1.0 = oldest
            let colour = lerp_colour(FieldColours::TRAIL, FieldColours::TRAIL_OLD, age);

            painter.line_segment(
                [egui::pos2(sx1, sy1), egui::pos2(sx2, sy2)],
                egui::Stroke::new(2.5, colour),
            );
        }
    }

    /// Layer 3: Draw the vehicle position and heading indicator
    fn draw_vehicle(&self, painter: &egui::Painter, fix: &GpsFix) {
        let (sx, sy) = self.projection.world_to_screen(fix.latitude, fix.longitude);
        let pos = egui::pos2(sx, sy);

        // Draw heading indicator as a triangle pointing in the direction of travel
        // In heading-up mode this always points up, in north-up it rotates
        let heading_rad = if self.heading_up {
            // Vehicle always faces up in heading-up mode
            0.0_f64
        } else {
            // Positive rotation: heading clockwise from north.
            // The vertex math already accounts for screen-Y-down,
            // so we need a positive angle here (the map rotation in
            // field_projection uses negative because it rotates the
            // *world*, not the icon).
            fix.heading * std::f64::consts::PI / 180.0
        };

        let size = 12.0_f32;
        let cos_h = heading_rad.cos() as f32;
        let sin_h = heading_rad.sin() as f32;

        // Triangle: nose, left wing, right wing
        let nose = egui::pos2(
            pos.x + sin_h * size * 1.5,
            pos.y - cos_h * size * 1.5,
        );
        let left = egui::pos2(
            pos.x - (cos_h * size * 0.7 + sin_h * size * 0.5),
            pos.y - (sin_h * size * 0.7 - cos_h * size * 0.5),
        );
        let right = egui::pos2(
            pos.x + (cos_h * size * 0.7 - sin_h * size * 0.5),
            pos.y + (sin_h * size * 0.7 + cos_h * size * 0.5),
        );

        // Fix quality colour ring
        let ring_colour = match fix.fix_quality {
            FixQuality::Rtk => egui::Color32::GREEN,
            FixQuality::RtkFloat => egui::Color32::YELLOW,
            FixQuality::Dgps => egui::Color32::LIGHT_BLUE,
            FixQuality::Gps => egui::Color32::ORANGE,
            FixQuality::NoFix => egui::Color32::RED,
        };

        painter.circle_stroke(pos, size + 4.0, egui::Stroke::new(2.0, ring_colour));
        painter.add(egui::Shape::convex_polygon(
            vec![nose, left, right],
            FieldColours::VEHICLE,
            egui::Stroke::NONE,
        ));
    }

    /// Overlay: Draw a scale bar in the bottom-left corner
    fn draw_scale_bar(&self, painter: &egui::Painter, rect: egui::Rect) {
        let spacing = self.projection.grid_spacing_m();

        // Scale bar length in pixels
        let bar_px = (spacing * self.projection.pixels_per_metre) as f32;
        let bar_px = bar_px.min(rect.width() * 0.3); // Cap at 30% of screen width

        let margin = 15.0;
        let bar_y = rect.bottom() - margin;
        let bar_x_start = rect.left() + margin;
        let bar_x_end = bar_x_start + bar_px;

        // Draw the bar
        painter.line_segment(
            [egui::pos2(bar_x_start, bar_y), egui::pos2(bar_x_end, bar_y)],
            egui::Stroke::new(2.0, egui::Color32::WHITE),
        );
        // End caps
        painter.line_segment(
            [egui::pos2(bar_x_start, bar_y - 4.0), egui::pos2(bar_x_start, bar_y + 4.0)],
            egui::Stroke::new(1.5, egui::Color32::WHITE),
        );
        painter.line_segment(
            [egui::pos2(bar_x_end, bar_y - 4.0), egui::pos2(bar_x_end, bar_y + 4.0)],
            egui::Stroke::new(1.5, egui::Color32::WHITE),
        );

        // Label
        let label = format_distance(spacing);
        painter.text(
            egui::pos2((bar_x_start + bar_x_end) / 2.0, bar_y - 8.0),
            egui::Align2::CENTER_BOTTOM,
            label,
            egui::FontId::proportional(12.0),
            egui::Color32::WHITE,
        );
    }
}

/// Linearly interpolate between two colours
fn lerp_colour(a: egui::Color32, b: egui::Color32, t: f32) -> egui::Color32 {
    let t = t.clamp(0.0, 1.0);
    egui::Color32::from_rgb(
        (a.r() as f32 + (b.r() as f32 - a.r() as f32) * t) as u8,
        (a.g() as f32 + (b.g() as f32 - a.g() as f32) * t) as u8,
        (a.b() as f32 + (b.b() as f32 - a.b() as f32) * t) as u8,
    )
}

/// Format a distance in metres for display (e.g., "50m", "200m", "1.0km")
fn format_distance(metres: f64) -> String {
    if metres >= 1000.0 {
        format!("{:.1}km", metres / 1000.0)
    } else {
        format!("{}m", metres as i32)
    }
}
