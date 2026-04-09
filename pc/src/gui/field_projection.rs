//! Coordinate projection for the field view canvas.
//!
//! Converts between three coordinate spaces:
//!   1. World (lat/lon in degrees, WGS84)
//!   2. Local (metres from a reference point, using tangent plane approximation)
//!   3. Screen (pixels on the egui canvas)
//!
//! The local coordinate system uses:
//!   - X = East (positive) / West (negative)
//!   - Y = North (positive) / South (negative)
//!
//! This projection is accurate to centimetre level within a few kilometres
//! of the reference point, which is more than enough for paddock-scale work.
//!
//! ## Future: Georeferenced Image Overlay
//!
//! To overlay a drone orthomosaic or camera imagery, convert the image's
//! corner coordinates to local metres using `world_to_local()`, then project
//! to screen coordinates. The image draws on Layer 0 before all other layers.

use std::f64::consts::PI;

/// Converts between world (lat/lon), local (metres), and screen (pixels) coordinates.
pub struct FieldProjection {
    /// Reference point for the local coordinate system (lat, lon)
    /// All local coordinates are relative to this point.
    ref_lat: f64,
    ref_lon: f64,
    /// Whether a reference point has been set
    has_reference: bool,

    /// Precomputed metres-per-degree at the reference latitude
    metres_per_deg_lat: f64,
    metres_per_deg_lon: f64,

    /// Zoom level: how many screen pixels per metre of real world
    pub pixels_per_metre: f64,

    /// Heading-up rotation angle in radians (0 = north up)
    pub rotation_rad: f64,

    /// Screen centre point (pixels) - where the vehicle is drawn
    pub screen_centre_x: f32,
    pub screen_centre_y: f32,

    /// Camera offset in local metres (for panning)
    pub camera_offset_x: f64,
    pub camera_offset_y: f64,
}

impl FieldProjection {
    pub fn new() -> Self {
        Self {
            ref_lat: 0.0,
            ref_lon: 0.0,
            has_reference: false,
            metres_per_deg_lat: 111_320.0,
            metres_per_deg_lon: 111_320.0,
            pixels_per_metre: 2.0, // Default: 2px per metre (reasonable paddock view)
            rotation_rad: 0.0,
            screen_centre_x: 400.0,
            screen_centre_y: 400.0,
            camera_offset_x: 0.0,
            camera_offset_y: 0.0,
        }
    }

    /// Set or update the reference point. Call this once with the first GPS fix,
    /// or whenever you want to re-centre the coordinate system.
    pub fn set_reference(&mut self, lat: f64, lon: f64) {
        self.ref_lat = lat;
        self.ref_lon = lon;
        self.has_reference = true;

        // Precompute conversion factors at this latitude
        // 1 degree of latitude ≈ 111,320m everywhere
        // 1 degree of longitude ≈ 111,320m × cos(latitude)
        let lat_rad = lat.abs() * PI / 180.0;
        self.metres_per_deg_lat = 111_320.0;
        self.metres_per_deg_lon = 111_320.0 * lat_rad.cos();
    }

    /// Returns true if a reference point has been established
    pub fn has_reference(&self) -> bool {
        self.has_reference
    }

    /// Convert world coordinates (lat/lon) to local coordinates (metres from reference).
    /// Returns (x_east, y_north) in metres.
    pub fn world_to_local(&self, lat: f64, lon: f64) -> (f64, f64) {
        let x = (lon - self.ref_lon) * self.metres_per_deg_lon;
        let y = (lat - self.ref_lat) * self.metres_per_deg_lat;
        (x, y)
    }

    /// Convert local coordinates (metres) to world coordinates (lat/lon).
    /// Useful for converting screen clicks back to GPS positions.
    pub fn local_to_world(&self, x: f64, y: f64) -> (f64, f64) {
        let lat = self.ref_lat + y / self.metres_per_deg_lat;
        let lon = self.ref_lon + x / self.metres_per_deg_lon;
        (lat, lon)
    }

    /// Convert local coordinates (metres) to screen coordinates (pixels).
    /// Applies heading-up rotation and zoom.
    pub fn local_to_screen(&self, x: f64, y: f64) -> (f32, f32) {
        // Apply camera offset (for panning)
        let cx = x - self.camera_offset_x;
        let cy = y - self.camera_offset_y;

        // Apply heading-up rotation
        let cos_r = self.rotation_rad.cos();
        let sin_r = self.rotation_rad.sin();
        let rx = cx * cos_r - cy * sin_r;
        let ry = cx * sin_r + cy * cos_r;

        // Scale metres to pixels and convert to screen space
        // Note: screen Y is inverted (down is positive), so we negate ry
        let sx = self.screen_centre_x + (rx * self.pixels_per_metre) as f32;
        let sy = self.screen_centre_y - (ry * self.pixels_per_metre) as f32;
        (sx, sy)
    }

    /// Convert world coordinates directly to screen coordinates.
    pub fn world_to_screen(&self, lat: f64, lon: f64) -> (f32, f32) {
        let (lx, ly) = self.world_to_local(lat, lon);
        self.local_to_screen(lx, ly)
    }

    /// Update the screen centre (call when the canvas is resized).
    pub fn set_screen_size(&mut self, width: f32, height: f32) {
        self.screen_centre_x = width / 2.0;
        // Place the vehicle slightly below centre so you see more ahead
        self.screen_centre_y = height * 0.65;
    }

    /// Set heading-up rotation from a heading in degrees (0-360 from north).
    pub fn set_heading(&mut self, heading_deg: f64) {
        // To rotate the world so the heading points up, we rotate by negative heading
        self.rotation_rad = -heading_deg * PI / 180.0;
    }

    /// Zoom in (increase pixels per metre)
    pub fn zoom_in(&mut self) {
        self.pixels_per_metre = (self.pixels_per_metre * 1.2).min(100.0);
    }

    /// Zoom out (decrease pixels per metre)
    pub fn zoom_out(&mut self) {
        self.pixels_per_metre = (self.pixels_per_metre / 1.2).max(0.05);
    }

    /// Get the current scale for display (metres visible across the screen width)
    pub fn visible_width_m(&self) -> f64 {
        (self.screen_centre_x as f64 * 2.0) / self.pixels_per_metre
    }

    /// Get a sensible grid spacing in metres for the current zoom level.
    /// Returns a "round" interval that gives roughly 4-8 grid lines across the view.
    pub fn grid_spacing_m(&self) -> f64 {
        let visible = self.visible_width_m();
        let target_lines = 6.0;
        let raw_spacing = visible / target_lines;

        // Round to a "nice" interval: 1, 2, 5, 10, 20, 50, 100, 200, 500...
        let magnitude = 10.0_f64.powf(raw_spacing.log10().floor());
        let normalised = raw_spacing / magnitude;

        let nice = if normalised < 1.5 {
            1.0
        } else if normalised < 3.5 {
            2.0
        } else if normalised < 7.5 {
            5.0
        } else {
            10.0
        };

        nice * magnitude
    }
}
