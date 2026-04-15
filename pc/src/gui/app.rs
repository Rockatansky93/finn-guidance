//! Main GUI application - displays GPS position, guidance line, and cross-track error.
//!
//! The GUI is split into two pages:
//! - **Working page**: Full-screen field view with large overlaid guidance readouts.
//!   Designed for glancing at while driving — minimal controls, big numbers.
//! - **Setup page**: AB line management, implement width, coverage stats, position
//!   details, and view controls. Used when stopped or configuring.
//!
//! The GPS status bar is always visible on both pages (safety-critical).

use std::collections::VecDeque;
use std::time::{Duration, Instant};
use eframe::egui;
use crossbeam_channel::Receiver;
use finn_guidance_common::types::{GpsFix, CrossTrackError, FixQuality, ImuData, WasReading, MotorStatus};
use finn_guidance_common::protocol::FinnMessage;
use crate::comms::serial::MotorHandle;
use crate::guidance::ab_line::AbLineGuide;
use crate::guidance::steering::SteeringController;
use crate::coverage::logger::CoverageLogger;
use crate::coverage::db::{CoverageDb, SavedField, SavedAbLine};
use crate::position::interpolator::PositionInterpolator;
use super::field_view::FieldView;

/// Target frame interval — 30fps is smooth for guidance display while
/// keeping CPU load low on field laptops (Dell 7390 etc).
const FRAME_INTERVAL: Duration = Duration::from_millis(33);

/// Minimum interval between motor serial writes. The ESP32 watchdog is
/// 500ms, so 50ms (20Hz) keeps it well fed without flooding the serial
/// bus. Matches the WAS update rate so we react to every new reading.
const STEER_SEND_INTERVAL: Duration = Duration::from_millis(50);

/// Which page is currently displayed
#[derive(Debug, Clone, Copy, PartialEq)]
enum ActivePage {
    /// Full-screen field view with large overlaid readouts — for driving
    Working,
    /// Configuration, AB lines, coverage stats — for when stopped
    Setup,
}

/// Main application state
pub struct GuidanceApp {
    /// Receiver for GPS fixes from the reader thread
    gps_rx: Receiver<GpsFix>,
    /// Receiver for FINN sensor messages (WAS, IMU, heartbeat) from the reader thread
    finn_rx: Receiver<FinnMessage>,
    /// Latest real GPS fix (from the module, not interpolated)
    current_fix: Option<GpsFix>,
    /// Position interpolator for smooth GUI updates between 1Hz fixes
    interpolator: PositionInterpolator,
    /// The display fix — either interpolated (when moving) or real (when stopped).
    /// Used for field view, guidance, lightbar. Updated every frame.
    display_fix: Option<GpsFix>,
    /// AB line guidance calculator
    guide: AbLineGuide,
    /// Current cross-track error
    current_error: Option<CrossTrackError>,
    /// Trail of recent positions for drawing
    position_trail: VecDeque<(f64, f64)>,
    /// Max trail length
    max_trail: usize,
    /// Field view canvas
    field_view: FieldView,
    /// Coverage logger
    coverage: CoverageLogger,
    /// Brief notification when auto-pass triggers (message, expiry frame count)
    auto_pass_notification: Option<(String, u32)>,
    /// Currently active GUI page
    active_page: ActivePage,
    /// Lightbar sensitivity: how many centimetres of error each segment represents.
    lightbar_cm_per_seg: f64,

    // --- ESP32 sensor state ---
    /// Latest wheel angle sensor reading (from sensor ESP32, 20Hz)
    latest_was: Option<WasReading>,
    /// Latest IMU reading (from sensor ESP32, 20Hz)
    latest_imu: Option<ImuData>,
    /// Latest motor controller status (from motor ESP32, 5Hz)
    latest_motor: Option<MotorStatus>,
    /// Sensor ESP32 uptime from last heartbeat (ms)
    sensor_uptime_ms: Option<u64>,
    /// Handle for sending steer commands to the motor ESP32
    motor_handle: MotorHandle,
    /// Current test PWM value for manual motor testing (Setup page)
    test_pwm: i16,
    /// Status message from motor test actions
    motor_test_msg: Option<(String, u32)>,

    // --- Auto-steer ---
    /// Steering controller (PID / proportional control)
    steering: SteeringController,
    /// Status message for auto-steer events (shown on working page)
    steer_status_msg: Option<(String, u32)>,
    /// Last time a steer command was sent to the motor ESP32.
    /// Throttles serial writes to ~20Hz instead of every frame.
    last_steer_send: Instant,

    // --- WAS calibration ---
    /// WAS ADC value at steering centre (wheels straight)
    was_centre: Option<u16>,
    /// WAS ADC value at full left steering lock
    was_left_lock: Option<u16>,
    /// WAS ADC value at full right steering lock
    was_right_lock: Option<u16>,
    /// Whether to invert the motor PWM sign (true = positive PWM steers left)
    motor_invert: bool,
    /// Status message for WAS calibration actions
    was_cal_msg: Option<(String, u32)>,

    // --- AB line persistence ---
    /// SQLite database (opened once, held for the session)
    db: Option<CoverageDb>,
    /// Cached list of all fields, refreshed after any save/delete
    saved_fields: Vec<SavedField>,
    /// Cached list of all AB lines, refreshed after any save/delete
    saved_ab_lines: Vec<SavedAbLine>,
    /// Which field is currently expanded in the load list (None = all collapsed)
    expanded_field: Option<i64>,
    /// Whether the save-line dialog is open
    show_save_dialog: bool,
    /// Text buffer for the line name in the save dialog
    save_line_name: String,
    /// Selected field_id for the save dialog (None = no field / unassigned)
    save_field_id: Option<i64>,
    /// Whether the new-field dialog is open
    show_new_field_dialog: bool,
    /// Text buffer for the new field name
    new_field_name: String,
    /// Transient status message shown below the AB LINE section (cleared after ~120 frames)
    ab_status_msg: Option<(String, u32)>,
    /// Import/export status message
    io_status_msg: Option<(String, u32)>,
}

impl GuidanceApp {
    pub fn new(gps_rx: Receiver<GpsFix>, finn_rx: Receiver<FinnMessage>, motor_handle: MotorHandle, implement_width: f64) -> Self {
        // Open the coverage database.  `CoverageLogger` also holds a reference
        // in some configurations; here we open a second handle for persistence.
        let db = CoverageDb::open(std::path::Path::new("data/coverage.db")).ok();

        // Pre-load the field/line lists so the UI is ready immediately.
        let saved_fields = db.as_ref()
            .and_then(|d| d.list_fields().ok())
            .unwrap_or_default();
        let saved_ab_lines = db.as_ref()
            .and_then(|d| d.list_ab_lines().ok())
            .unwrap_or_default();

        // Load persisted settings from database, falling back to defaults
        let implement_width = db.as_ref()
            .and_then(|d| d.get_config("implement_width_m"))
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or(implement_width);

        let overlap = db.as_ref()
            .and_then(|d| d.get_config("overlap_m"))
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or(0.0);

        let lightbar_cm_per_seg = db.as_ref()
            .and_then(|d| d.get_config("lightbar_cm_per_seg"))
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or(20.0);

        // Load WAS calibration from database (None if not yet calibrated)
        let was_centre = db.as_ref()
            .and_then(|d| d.get_config("was_centre"))
            .and_then(|v| v.parse::<u16>().ok());
        let was_left_lock = db.as_ref()
            .and_then(|d| d.get_config("was_left_lock"))
            .and_then(|v| v.parse::<u16>().ok());
        let was_right_lock = db.as_ref()
            .and_then(|d| d.get_config("was_right_lock"))
            .and_then(|v| v.parse::<u16>().ok());
        let motor_invert = db.as_ref()
            .and_then(|d| d.get_config("motor_invert"))
            .map(|v| v == "true")
            .unwrap_or(false);

        // Load steering controller gains from database
        let steer_kp = db.as_ref()
            .and_then(|d| d.get_config("steer_kp"))
            .and_then(|v| v.parse::<f64>().ok())
            .and_then(|v| if v > 50.0 { None } else { Some(v) }) // ignore old PWM/m values
            .unwrap_or(30.0);
        let steer_kp_angle = db.as_ref()
            .and_then(|d| d.get_config("steer_kp_angle"))
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or(4.0);

        // Auto-load last-used AB line on startup
        let mut guide = AbLineGuide::new(implement_width);
        guide.overlap_m = overlap;
        if let Some(db_ref) = &db {
            if let Some(line_id_str) = db_ref.get_config("last_ab_line_id") {
                if let Ok(line_id) = line_id_str.parse::<i64>() {
                    // Find this line in the saved list and load it
                    if let Some(line) = saved_ab_lines.iter().find(|l| l.id == line_id) {
                        guide.load_ab_line(line.a_lat, line.a_lon, line.b_lat, line.b_lon);
                    }
                }
            }
        }

        Self {
            gps_rx,
            finn_rx,
            current_fix: None,
            interpolator: PositionInterpolator::new(),
            display_fix: None,
            guide,
            current_error: None,
            position_trail: VecDeque::new(),
            max_trail: 5000,
            field_view: FieldView::new(),
            coverage: CoverageLogger::new(implement_width),
            auto_pass_notification: None,
            active_page: ActivePage::Working,
            lightbar_cm_per_seg,
            latest_was: None,
            latest_imu: None,
            latest_motor: None,
            sensor_uptime_ms: None,
            motor_handle,
            test_pwm: 0,
            motor_test_msg: None,
            steering: {
                let mut s = SteeringController::new();
                s.kp = steer_kp;
                s.kp_angle = steer_kp_angle;
                s
            },
            steer_status_msg: None,
            last_steer_send: Instant::now(),
            was_centre,
            was_left_lock,
            was_right_lock,
            motor_invert,
            was_cal_msg: None,
            db,
            saved_fields,
            saved_ab_lines,
            expanded_field: None,
            show_save_dialog: false,
            save_line_name: String::new(),
            save_field_id: None,
            show_new_field_dialog: false,
            new_field_name: String::new(),
            ab_status_msg: None,
            io_status_msg: None,
        }
    }

    /// Reload the field/line caches from the DB.  Call after any save or delete.
    fn refresh_ab_cache(&mut self) {
        if let Some(db) = &self.db {
            self.saved_fields = db.list_fields().unwrap_or_default();
            self.saved_ab_lines = db.list_ab_lines().unwrap_or_default();
        }
    }

    /// Compute a calibrated steering angle from a raw WAS ADC value.
    ///
    /// Uses piecewise linear mapping from the three calibration points:
    ///   left_lock → -max_angle,  centre → 0,  right_lock → +max_angle
    ///
    /// Returns None if calibration is incomplete. The max angle is estimated
    /// from the ADC range (we don't know the physical lock angle, so we
    /// normalise to ±45° as a reasonable default — the PID only needs a
    /// proportional signal, not true degrees).
    fn was_calibrated_angle(&self, raw: u16) -> Option<f64> {
        let centre = self.was_centre? as f64;
        let left = self.was_left_lock? as f64;
        let right = self.was_right_lock? as f64;
        let raw = raw as f64;

        // Normalised steering angle in degrees.
        // Convention: negative = left, positive = right.
        const MAX_ANGLE: f64 = 45.0;

        let angle = if raw <= centre {
            // Left half: map [left_lock .. centre] → [-MAX_ANGLE .. 0]
            let range = centre - left;
            if range.abs() < 1.0 { return Some(0.0); }
            -MAX_ANGLE * (centre - raw) / range
        } else {
            // Right half: map [centre .. right_lock] → [0 .. MAX_ANGLE]
            let range = right - centre;
            if range.abs() < 1.0 { return Some(0.0); }
            MAX_ANGLE * (raw - centre) / range
        };

        Some(angle.clamp(-MAX_ANGLE, MAX_ANGLE))
    }

    /// Apply the motor_invert setting to a PWM value for the PID controller.
    /// When motor_invert is true, the sign is flipped so that positive always
    /// means "steer right" from the PID's perspective.
    fn apply_motor_direction(&self, pwm: i16) -> i16 {
        if self.motor_invert { -pwm } else { pwm }
    }
}

impl eframe::App for GuidanceApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // === Process real GPS fixes from the reader thread ===
        // These are 1Hz from the LC29H DA. Used for coverage logging and auto-pass
        // detection (things that need true positions, not interpolated ones).
        while let Ok(fix) = self.gps_rx.try_recv() {
            // Feed the interpolator with the true position
            self.interpolator.update_fix(&fix);

            // Add real fix to trail (1Hz is fine for the breadcrumb path)
            self.position_trail.push_back((fix.latitude, fix.longitude));
            if self.position_trail.len() > self.max_trail {
                self.position_trail.pop_front();
            }

            // Auto-pass: detect headland turn and snap to nearest pass (real fix only)
            if let Some(event) = self.guide.update_auto_pass(&fix) {
                self.auto_pass_notification = Some((
                    format!("Auto → Pass {}", event.new_pass),
                    180, // ~3 seconds at 60fps
                ));
            }

            // Log coverage if engaged (real fix only — distance filter needs true positions)
            self.coverage.log_fix(&fix, self.db.as_ref());

            // Notify steering controller of fresh GPS data (for safety timeout)
            self.steering.notify_gps_fix();

            self.current_fix = Some(fix);
        }

        // === Process FINN sensor messages from ESP32 ===
        // These arrive at 20Hz (WAS + IMU) plus heartbeat every 2s.
        // We only keep the latest value of each — no need to buffer history yet.
        while let Ok(msg) = self.finn_rx.try_recv() {
            match msg {
                FinnMessage::Was(was) => {
                    self.latest_was = Some(was);
                    self.steering.notify_was_reading();
                }
                FinnMessage::Imu(imu) => {
                    self.latest_imu = Some(imu);
                }
                FinnMessage::SensorHeartbeat(hb) => {
                    self.sensor_uptime_ms = Some(hb.uptime_ms);
                }
                FinnMessage::MotorStatus(mtr) => {
                    self.latest_motor = Some(mtr);
                }
            }
        }

        // === Interpolate position for smooth display (every frame, ~30fps) ===
        // Between 1Hz real fixes, the interpolator dead-reckons the position
        // using speed × heading. This makes the vehicle triangle, trail, lightbar,
        // and XTE readout update smoothly instead of jumping once per second.
        if let Some(interp_fix) = self.interpolator.interpolate() {
            let interp_fix = interp_fix.clone();

            // Recalculate guidance error with interpolated position (smooth lightbar/XTE)
            self.current_error = self.guide.calculate_error(&interp_fix);

            // === Auto-steer: compute and send motor command ===
            // Throttled to ~20Hz — matches WAS update rate and avoids flooding
            // the serial bus. The ESP32 watchdog is 500ms so 50ms is well within.
            if self.steering.engaged {
                let now = Instant::now();
                let should_send = now.duration_since(self.last_steer_send) >= STEER_SEND_INTERVAL;

                if let Some(error) = &self.current_error {
                    // Get current steering angle from WAS (if calibrated and available)
                    let actual_angle = self.latest_was.as_ref()
                        .and_then(|was| self.was_calibrated_angle(was.raw_value));

                    let (raw_pwm, disengaged) = self.steering.compute(
                        error.distance_m,
                        interp_fix.speed,
                        actual_angle,
                    );
                    if disengaged {
                        // Safety disengage — send stop immediately (bypass throttle)
                        let _ = self.motor_handle.send_steer(0);
                        self.last_steer_send = now;
                        let reason = self.steering.disengage_reason.clone()
                            .unwrap_or_else(|| "Unknown".to_string());
                        self.steer_status_msg = Some((
                            format!("Auto-steer OFF: {}", reason),
                            300, // ~10 seconds at 30fps
                        ));
                    } else if should_send {
                        let pwm = self.apply_motor_direction(raw_pwm);
                        let _ = self.motor_handle.send_steer(pwm);
                        self.last_steer_send = now;
                    }
                } else {
                    // No guidance error (no AB line?) — send zero (throttled)
                    if should_send {
                        let _ = self.motor_handle.send_steer(0);
                        self.last_steer_send = now;
                    }
                }
            }

            self.display_fix = Some(interp_fix);
        }

        // Tick down notification expiry
        if let Some((_, ref mut frames)) = self.auto_pass_notification {
            if *frames == 0 {
                self.auto_pass_notification = None;
            } else {
                *frames -= 1;
            }
        }
        if let Some((_, ref mut frames)) = self.ab_status_msg {
            if *frames == 0 { self.ab_status_msg = None; } else { *frames -= 1; }
        }
        if let Some((_, ref mut frames)) = self.io_status_msg {
            if *frames == 0 { self.io_status_msg = None; } else { *frames -= 1; }
        }
        if let Some((_, ref mut frames)) = self.motor_test_msg {
            if *frames == 0 { self.motor_test_msg = None; } else { *frames -= 1; }
        }
        if let Some((_, ref mut frames)) = self.was_cal_msg {
            if *frames == 0 { self.was_cal_msg = None; } else { *frames -= 1; }
        }
        if let Some((_, ref mut frames)) = self.steer_status_msg {
            if *frames == 0 { self.steer_status_msg = None; } else { *frames -= 1; }
        }

        // Request repaint at ~30fps — smooth for guidance display while
        // keeping CPU/GPU load manageable on field laptops (Dell 7390).
        // Previously this was uncapped (request_repaint()) which pegged
        // the CPU at 100% running 200+ fps of serial writes and coverage
        // polygon rendering.
        ctx.request_repaint_after(FRAME_INTERVAL);

        // === Top panel: GPS status bar (always visible on both pages) ===
        self.draw_status_bar(ctx);

        // === Page-specific content ===
        match self.active_page {
            ActivePage::Working => self.draw_working_page(ctx),
            ActivePage::Setup => self.draw_setup_page(ctx),
        }
    }
}

// =============================================================================
// Page drawing methods
// =============================================================================
impl GuidanceApp {
    /// GPS status bar — always visible on both pages (safety-critical info)
    fn draw_status_bar(&self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                if let Some(fix) = &self.current_fix {
                    let (fix_colour, fix_text) = fix_quality_display(fix.fix_quality);
                    ui.colored_label(fix_colour, format!("● {}", fix_text));
                    ui.separator();
                    ui.label(format!("Sats: {}", fix.satellites));
                    ui.separator();
                    ui.label(format!("HDOP: {:.1}", fix.hdop));
                    ui.separator();
                    ui.label(format!("{:.1} km/h", fix.speed * 3.6));
                    ui.separator();
                    ui.label(format!("{:.0}°", fix.heading));
                } else {
                    ui.colored_label(egui::Color32::RED, "● No GPS data");
                }

                // Recording indicator (always visible)
                if self.coverage.is_engaged() {
                    ui.separator();
                    ui.colored_label(
                        egui::Color32::from_rgb(255, 60, 60),
                        format!("● REC {}", self.coverage.total_points()),
                    );
                }
            });
        });
    }

    /// Working page: full-screen field view with overlaid guidance readouts.
    /// Designed for tractor cab — big numbers, minimal controls, glanceable.
    fn draw_working_page(&mut self, ctx: &egui::Context) {
        // Bottom bar: just ENGAGE and page switch — big touch targets
        egui::TopBottomPanel::bottom("working_controls")
            .min_height(50.0)
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    ui.spacing_mut().item_spacing.x = 12.0;

                    // Large engage button
                    let engage_text = if self.coverage.is_engaged() { "⏹ DISENGAGE" } else { "▶ ENGAGE" };
                    let engage_colour = if self.coverage.is_engaged() {
                        egui::Color32::from_rgb(255, 60, 60)
                    } else {
                        egui::Color32::from_rgb(60, 200, 60)
                    };
                    let engage_btn = egui::Button::new(
                        egui::RichText::new(engage_text).size(20.0).strong().color(engage_colour)
                    ).min_size(egui::vec2(160.0, 40.0));
                    if ui.add(engage_btn).clicked() {
                        self.coverage.toggle_engage(self.db.as_ref());
                    }

                    // Auto-steer engage/disengage button
                    let can_auto_steer = self.guide.has_complete_line()
                        && self.motor_handle.is_connected()
                        && self.was_centre.is_some()
                        && self.was_left_lock.is_some()
                        && self.was_right_lock.is_some();

                    let (steer_text, steer_colour) = if self.steering.engaged {
                        ("⊗ STEER OFF", egui::Color32::from_rgb(255, 60, 60))
                    } else {
                        ("⊕ AUTO-STEER", egui::Color32::from_rgb(100, 200, 255))
                    };
                    let steer_btn = egui::Button::new(
                        egui::RichText::new(steer_text).size(18.0).strong().color(steer_colour)
                    ).min_size(egui::vec2(140.0, 40.0));
                    let steer_resp = ui.add_enabled(
                        can_auto_steer || self.steering.engaged,
                        steer_btn,
                    );
                    if steer_resp.clicked() {
                        if self.steering.engaged {
                            self.steering.disengage(Some("Manual disengage".to_string()));
                            let _ = self.motor_handle.send_steer(0);
                            self.steer_status_msg = Some(("Auto-steer OFF".to_string(), 180));
                        } else {
                            self.steering.engage();
                            self.steer_status_msg = Some(("Auto-steer ON".to_string(), 180));
                        }
                    }

                    // Auto-pass toggle
                    let auto_label = if self.guide.auto_pass_enabled { "Auto ✓" } else { "Auto ✗" };
                    let auto_colour = if self.guide.auto_pass_enabled {
                        egui::Color32::from_rgb(60, 200, 60)
                    } else {
                        egui::Color32::GRAY
                    };
                    let auto_btn = egui::Button::new(
                        egui::RichText::new(auto_label).size(18.0).color(auto_colour)
                    ).min_size(egui::vec2(90.0, 40.0));
                    if ui.add(auto_btn).clicked() {
                        self.guide.auto_pass_enabled = !self.guide.auto_pass_enabled;
                    }

                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        // Page switch: go to setup
                        let setup_btn = egui::Button::new(
                            egui::RichText::new("⚙ Setup").size(16.0)
                        ).min_size(egui::vec2(100.0, 40.0));
                        if ui.add(setup_btn).clicked() {
                            self.active_page = ActivePage::Setup;
                        }

                        // Pass indicator
                        ui.label(
                            egui::RichText::new(format!("Pass {}", self.guide.pass_number))
                                .size(16.0)
                                .color(egui::Color32::from_rgb(80, 160, 255)),
                        );

                        // Nudge indicator — only shown when nudge is non-zero
                        let nudge_cm = (self.guide.nudge_m * 100.0).round() as i32;
                        if nudge_cm != 0 {
                            ui.separator();
                            let dir = if nudge_cm > 0 { "→R" } else { "←L" };
                            ui.label(
                                egui::RichText::new(format!("Nudge {} cm {}", nudge_cm.abs(), dir))
                                    .size(14.0)
                                    .color(egui::Color32::from_rgb(255, 200, 60)),
                            );
                        }
                    });
                });
            });

        // Central panel: field view with overlaid guidance
        let canvas_frame = egui::Frame::default()
            .inner_margin(egui::Margin::same(0.0))
            .fill(egui::Color32::TRANSPARENT);

        egui::CentralPanel::default()
            .frame(canvas_frame)
            .show(ctx, |ui| {
                // Draw the field view canvas (uses interpolated position for smooth movement)
                self.field_view.draw(
                    ui,
                    &self.display_fix,
                    &self.position_trail,
                    &self.guide,
                    self.coverage.points(),
                    self.coverage.implement_width(),
                );

                // Overlay: lightbar across the top of the field view
                let overlay_rect = ui.max_rect();
                let painter = ui.painter();

                self.draw_lightbar(&painter, overlay_rect);

                // Overlay: large cross-track error readout

                if let Some(error) = &self.current_error {
                    let xtd_cm = (error.distance_m * 100.0) as i32;
                    let colour = if xtd_cm.abs() < 5 {
                        egui::Color32::GREEN
                    } else if xtd_cm.abs() < 15 {
                        egui::Color32::YELLOW
                    } else {
                        egui::Color32::from_rgb(255, 60, 60)
                    };

                    // Large XTE number — top right, below lightbar
                    let xte_pos = egui::pos2(overlay_rect.right() - 20.0, overlay_rect.top() + 55.0);

                    // Background pill for readability
                    let text_galley = painter.layout_no_wrap(
                        format!("{} cm", xtd_cm),
                        egui::FontId::proportional(56.0),
                        colour,
                    );
                    let pill_rect = egui::Rect::from_min_size(
                        egui::pos2(
                            xte_pos.x - text_galley.size().x - 20.0,
                            xte_pos.y,
                        ),
                        egui::vec2(text_galley.size().x + 30.0, text_galley.size().y + 10.0),
                    );
                    painter.rect_filled(
                        pill_rect,
                        8.0,
                        egui::Color32::from_rgba_premultiplied(0, 0, 0, 160),
                    );
                    painter.galley(
                        egui::pos2(pill_rect.left() + 15.0, pill_rect.top() + 5.0),
                        text_galley,
                        colour,
                    );

                    // Direction indicator below XTE
                    let dir_text = if xtd_cm < 0 { "◄ LEFT" } else { "RIGHT ►" };
                    painter.text(
                        egui::pos2(pill_rect.center().x, pill_rect.bottom() + 8.0),
                        egui::Align2::CENTER_TOP,
                        dir_text,
                        egui::FontId::proportional(20.0),
                        egui::Color32::WHITE,
                    );
                }

                // Auto-pass notification (blue, below lightbar, centre)
                if let Some((ref msg, _)) = self.auto_pass_notification {
                    let notif_pos = egui::pos2(overlay_rect.center().x, overlay_rect.top() + 55.0);
                    let notif_galley = painter.layout_no_wrap(
                        msg.clone(),
                        egui::FontId::proportional(24.0),
                        egui::Color32::from_rgb(100, 200, 255),
                    );
                    let notif_rect = egui::Rect::from_min_size(
                        egui::pos2(
                            notif_pos.x - notif_galley.size().x / 2.0 - 10.0,
                            notif_pos.y,
                        ),
                        egui::vec2(notif_galley.size().x + 20.0, notif_galley.size().y + 10.0),
                    );
                    painter.rect_filled(
                        notif_rect,
                        6.0,
                        egui::Color32::from_rgba_premultiplied(0, 0, 0, 140),
                    );
                    painter.galley(
                        egui::pos2(notif_rect.left() + 10.0, notif_rect.top() + 5.0),
                        notif_galley,
                        egui::Color32::from_rgb(100, 200, 255),
                    );
                }

                // Auto-steer status indicator (top-left, below lightbar)
                if self.steering.engaged {
                    let steer_text = format!(
                        "AUTO-STEER  PWM {}  T:{:.0}° A:{:.0}°",
                        self.steering.last_output_pwm,
                        self.steering.last_desired_angle,
                        self.steering.last_actual_angle,
                    );
                    let steer_pos = egui::pos2(overlay_rect.left() + 20.0, overlay_rect.top() + 55.0);
                    let indicator_colour = if self.steering.was_stale {
                        egui::Color32::from_rgb(255, 200, 60) // amber when WAS stale
                    } else {
                        egui::Color32::from_rgb(100, 255, 100) // green normally
                    };
                    let steer_galley = painter.layout_no_wrap(
                        steer_text,
                        egui::FontId::proportional(16.0),
                        indicator_colour,
                    );
                    let pill_rect = egui::Rect::from_min_size(
                        egui::pos2(steer_pos.x - 8.0, steer_pos.y),
                        egui::vec2(steer_galley.size().x + 16.0, steer_galley.size().y + 8.0),
                    );
                    painter.rect_filled(
                        pill_rect,
                        6.0,
                        egui::Color32::from_rgba_premultiplied(0, 60, 0, 180),
                    );
                    painter.galley(
                        egui::pos2(pill_rect.left() + 8.0, pill_rect.top() + 4.0),
                        steer_galley,
                        indicator_colour,
                    );
                }

                // Auto-steer status message (e.g. "Auto-steer OFF: GPS fix lost")
                if let Some((ref msg, _)) = self.steer_status_msg {
                    let msg_pos = egui::pos2(overlay_rect.center().x, overlay_rect.top() + 90.0);
                    let msg_galley = painter.layout_no_wrap(
                        msg.clone(),
                        egui::FontId::proportional(20.0),
                        egui::Color32::from_rgb(255, 200, 100),
                    );
                    let msg_rect = egui::Rect::from_min_size(
                        egui::pos2(
                            msg_pos.x - msg_galley.size().x / 2.0 - 10.0,
                            msg_pos.y,
                        ),
                        egui::vec2(msg_galley.size().x + 20.0, msg_galley.size().y + 10.0),
                    );
                    painter.rect_filled(
                        msg_rect,
                        6.0,
                        egui::Color32::from_rgba_premultiplied(0, 0, 0, 160),
                    );
                    painter.galley(
                        egui::pos2(msg_rect.left() + 10.0, msg_rect.top() + 5.0),
                        msg_galley,
                        egui::Color32::from_rgb(255, 200, 100),
                    );
                }
            });
    }

    /// Draw the lightbar overlay across the top of the field view.
    ///
    /// The lightbar is a row of segments that light up to show how far off-line
    /// you are and which direction to steer. Segments light up on the side you
    /// need to steer TOWARDS — if you're left of the line, the right segments
    /// light up telling you to steer right.
    ///
    /// Layout: 15 segments left + 1 centre + 15 segments right = 31 total.
    /// Colour ramp: green (close) → yellow (moderate) → red (far off).
    fn draw_lightbar(&self, painter: &egui::Painter, rect: egui::Rect) {
        const SEGS_PER_SIDE: i32 = 15;
        const TOTAL_SEGS: i32 = SEGS_PER_SIDE * 2 + 1; // 31
        let bar_height: f32 = 30.0;
        let gap: f32 = 2.0;
        let margin_x: f32 = 10.0;
        let margin_top: f32 = 6.0;

        // Background bar
        let bar_rect = egui::Rect::from_min_size(
            egui::pos2(rect.left() + margin_x, rect.top() + margin_top),
            egui::vec2(rect.width() - margin_x * 2.0, bar_height),
        );
        painter.rect_filled(
            bar_rect,
            4.0,
            egui::Color32::from_rgba_premultiplied(0, 0, 0, 160),
        );

        // Segment dimensions
        let total_gap = gap * (TOTAL_SEGS - 1) as f32;
        let seg_width = (bar_rect.width() - total_gap - gap * 2.0) / TOTAL_SEGS as f32;
        let seg_height = bar_height - gap * 2.0;
        let seg_y = bar_rect.top() + gap;

        // Determine how many segments to light up and in which direction
        let xtd_cm = self.current_error.as_ref()
            .map(|e| (e.distance_m * 100.0) as i32)
            .unwrap_or(0);

        // Number of segments to light (capped at SEGS_PER_SIDE)
        let lit_count = if self.current_error.is_some() && self.guide.line.is_some() {
            ((xtd_cm.abs() as f64) / self.lightbar_cm_per_seg).ceil() as i32
        } else {
            0
        };
        let lit_count = lit_count.min(SEGS_PER_SIDE);

        // Draw each segment
        for i in 0..TOTAL_SEGS {
            let seg_index = i - SEGS_PER_SIDE; // -15 to +15, 0 = centre
            let seg_x = bar_rect.left() + gap + i as f32 * (seg_width + gap);

            let seg_rect = egui::Rect::from_min_size(
                egui::pos2(seg_x, seg_y),
                egui::vec2(seg_width, seg_height),
            );

            // Determine if this segment should be lit
            let is_lit = if self.current_error.is_none() || self.guide.line.is_none() {
                false
            } else if seg_index == 0 {
                // Centre segment: always lit when we have guidance
                true
            } else if xtd_cm < 0 {
                // Vehicle is LEFT of line → light up RIGHT segments (positive)
                // to tell operator to steer right
                seg_index > 0 && seg_index <= lit_count
            } else {
                // Vehicle is RIGHT of line → light up LEFT segments (negative)
                // to tell operator to steer left
                seg_index < 0 && seg_index >= -lit_count
            };

            let colour = if is_lit {
                let distance_from_centre = seg_index.unsigned_abs() as f32;
                lightbar_colour(distance_from_centre, SEGS_PER_SIDE as f32)
            } else {
                // Unlit segment — very dim outline
                egui::Color32::from_rgba_premultiplied(60, 60, 60, 100)
            };

            if is_lit {
                painter.rect_filled(seg_rect, 2.0, colour);
            } else {
                painter.rect_stroke(seg_rect, 2.0, egui::Stroke::new(1.0, colour));
            }
        }
    }

    /// Setup page: AB line controls, coverage stats, position details, view settings.
    /// Used when stopped or configuring before a run.
    fn draw_setup_page(&mut self, ctx: &egui::Context) {
        // Bottom bar: page switch back to working
        egui::TopBottomPanel::bottom("setup_controls")
            .min_height(50.0)
            .show(ctx, |ui| {
                ui.horizontal_centered(|ui| {
                    let work_btn = egui::Button::new(
                        egui::RichText::new("◄ Working View").size(18.0).strong()
                    ).min_size(egui::vec2(160.0, 40.0));
                    if ui.add(work_btn).clicked() {
                        self.active_page = ActivePage::Working;
                    }
                });
            });

        // Right panel: all setup controls
        egui::SidePanel::right("setup_panel")
            .resizable(true)
            .default_width(240.0)
            .min_width(200.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    // === AB Line Section ===
                    ui.add_space(10.0);
                    ui.label(egui::RichText::new("AB LINE").size(14.0).strong());
                    ui.add_space(6.0);

                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("Set A").min_size(egui::vec2(70.0, 30.0))).clicked() {
                            if let Some(fix) = &self.current_fix {
                                self.guide.set_point_a(fix);
                            }
                        }
                        if ui.add(egui::Button::new("Set B").min_size(egui::vec2(70.0, 30.0))).clicked() {
                            if let Some(fix) = &self.current_fix {
                                self.guide.set_point_b(fix);
                            }
                        }
                    });

                    ui.add_space(6.0);

                    // Pass controls
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("◄ Pass").min_size(egui::vec2(70.0, 30.0))).clicked() {
                            self.guide.prev_pass();
                        }
                        ui.label(
                            egui::RichText::new(format!("Pass {}", self.guide.pass_number))
                                .size(16.0).strong(),
                        );
                        if ui.add(egui::Button::new("Pass ►").min_size(egui::vec2(70.0, 30.0))).clicked() {
                            self.guide.next_pass();
                        }
                    });

                    ui.add_space(6.0);

                    // Auto-pass toggle
                    let auto_label = if self.guide.auto_pass_enabled { "Auto-Pass ✓" } else { "Auto-Pass ✗" };
                    let auto_colour = if self.guide.auto_pass_enabled {
                        egui::Color32::from_rgb(60, 200, 60)
                    } else {
                        egui::Color32::GRAY
                    };
                    if ui.add(egui::Button::new(
                        egui::RichText::new(auto_label).color(auto_colour)
                    ).min_size(egui::vec2(150.0, 30.0))).clicked() {
                        self.guide.auto_pass_enabled = !self.guide.auto_pass_enabled;
                    }

                    ui.add_space(6.0);

                    // ── Align grid to here ─────────────────────────────────
                    // Snaps the pass grid so the current GPS position falls exactly
                    // on the nearest whole pass line. Nudge is cleared. The saved
                    // AB line geometry is unchanged — only pass number shifts.
                    // Use when changing implements or starting from a fence line.
                    let can_align = self.guide.has_complete_line() && self.current_fix.is_some();
                    let align_btn = egui::Button::new(
                        egui::RichText::new("⊕ Align Grid to Here").size(13.0)
                    ).min_size(egui::vec2(150.0, 32.0));
                    let align_resp = ui.add_enabled(can_align, align_btn);
                    if align_resp.hovered() {
                        egui::show_tooltip_text(
                            ui.ctx(),
                            egui::LayerId::new(egui::Order::Tooltip, egui::Id::new("align_tooltip_layer")),
                            egui::Id::new("align_tooltip"),
                            "Snap the pass grid so your current position\nfalls on the nearest whole pass line.\nNudge is reset to zero.",
                        );
                    }
                    if align_resp.clicked() {
                        if let Some(fix) = self.current_fix.clone() {
                            if let Some(new_pass) = self.guide.align_grid_to_position(&fix) {
                                self.ab_status_msg = Some((
                                    format!("Grid aligned — now on Pass {}", new_pass),
                                    240,
                                ));
                            }
                        }
                    }
                    if !can_align && self.guide.has_complete_line() {
                        ui.label(egui::RichText::new("Waiting for GPS fix…").size(11.0).weak());
                    } else if !can_align {
                        ui.label(egui::RichText::new("Load a line first").size(11.0).weak());
                    }

                    ui.add_space(8.0);

                    // ── Save current line ──────────────────────────────────
                    let line_is_set = self.guide.has_complete_line();
                    if !self.show_save_dialog {
                        let save_btn = egui::Button::new("💾 Save Line…")
                            .min_size(egui::vec2(150.0, 30.0));
                        let btn_resp = ui.add_enabled(line_is_set, save_btn);
                        if btn_resp.clicked() {
                            // Pre-fill name with a timestamp
                            let ts = chrono::Local::now().format("%Y%m%d_%H%M").to_string();
                            self.save_line_name = format!("Line_{}", ts);
                            self.save_field_id = None;
                            self.show_save_dialog = true;
                        }
                        if !line_is_set {
                            ui.label(egui::RichText::new("Set A and B first").size(11.0).weak());
                        }
                    } else {
                        // Inline save dialog
                        ui.group(|ui| {
                            ui.label(egui::RichText::new("Save as:").size(12.0));
                            ui.text_edit_singleline(&mut self.save_line_name);

                            // Field selector
                            ui.add_space(4.0);
                            ui.label(egui::RichText::new("Field:").size(12.0));
                            let field_label = self.save_field_id
                                .and_then(|id| self.saved_fields.iter().find(|f| f.id == id))
                                .map(|f| f.name.as_str())
                                .unwrap_or("— none —");
                            egui::ComboBox::from_id_salt("save_field_picker")
                                .selected_text(field_label)
                                .show_ui(ui, |ui| {
                                    if ui.selectable_value(
                                        &mut self.save_field_id, None, "— none —"
                                    ).clicked() {}
                                    // Clone ids to avoid borrow conflict
                                    let field_ids: Vec<(i64, String)> = self.saved_fields
                                        .iter().map(|f| (f.id, f.name.clone())).collect();
                                    for (fid, fname) in field_ids {
                                        ui.selectable_value(
                                            &mut self.save_field_id,
                                            Some(fid),
                                            fname,
                                        );
                                    }
                                });

                            ui.add_space(4.0);
                            ui.horizontal(|ui| {
                                if ui.add(egui::Button::new("✔ Save")
                                    .min_size(egui::vec2(70.0, 28.0))).clicked()
                                {
                                    if let Some(line) = self.guide.ab_points() {
                                        let name = self.save_line_name.trim().to_string();
                                        let name = if name.is_empty() { "Unnamed".to_string() } else { name };
                                        if let Some(db) = &self.db {
                                            match db.save_ab_line(
                                                self.save_field_id,
                                                &name,
                                                line.a_lat, line.a_lon,
                                                line.b_lat, line.b_lon,
                                            ) {
                                                Ok(_) => {
                                                    self.ab_status_msg = Some((
                                                        format!("Saved \"{}\"", name), 180
                                                    ));
                                                }
                                                Err(e) => {
                                                    self.ab_status_msg = Some((
                                                        format!("Save failed: {}", e), 240
                                                    ));
                                                }
                                            }
                                        }
                                        self.refresh_ab_cache();
                                    }
                                    self.show_save_dialog = false;
                                }
                                if ui.add(egui::Button::new("✖ Cancel")
                                    .min_size(egui::vec2(70.0, 28.0))).clicked()
                                {
                                    self.show_save_dialog = false;
                                }
                            });
                        });
                    }

                    // Status message (fades out after ~3 seconds)
                    if let Some((ref msg, _)) = self.ab_status_msg {
                        ui.add_space(2.0);
                        ui.label(egui::RichText::new(msg).size(11.0)
                            .color(egui::Color32::from_rgb(100, 220, 100)));
                    }

                    ui.add_space(8.0);

                    // ── Fields + Load list ────────────────────────────────
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("SAVED LINES").size(12.0).strong());
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("+ Field").clicked() {
                                self.new_field_name.clear();
                                self.show_new_field_dialog = true;
                            }
                        });
                    });

                    // New-field inline dialog
                    if self.show_new_field_dialog {
                        ui.group(|ui| {
                            ui.label(egui::RichText::new("Field name:").size(12.0));
                            ui.text_edit_singleline(&mut self.new_field_name);
                            ui.horizontal(|ui| {
                                if ui.small_button("✔ Create").clicked() {
                                    let name = self.new_field_name.trim().to_string();
                                    if !name.is_empty() {
                                        if let Some(db) = &self.db {
                                            let _ = db.create_field(&name);
                                            self.refresh_ab_cache();
                                        }
                                    }
                                    self.show_new_field_dialog = false;
                                }
                                if ui.small_button("✖ Cancel").clicked() {
                                    self.show_new_field_dialog = false;
                                }
                            });
                        });
                    }

                    // Load list: fields as collapsible headers, lines beneath
                    egui::ScrollArea::vertical()
                        .id_salt("ab_load_list")
                        .max_height(200.0)
                        .show(ui, |ui| {
                            if self.saved_fields.is_empty() && self.saved_ab_lines.is_empty() {
                                ui.label(egui::RichText::new("No saved lines yet").size(11.0).weak());
                            }

                            // Lines not assigned to any field
                            let unassigned: Vec<(i64, String, f64, f64, f64, f64)> = self.saved_ab_lines.iter()
                                .filter(|l| l.field_id.is_none())
                                .map(|l| (l.id, l.name.clone(), l.a_lat, l.a_lon, l.b_lat, l.b_lon))
                                .collect();
                            if !unassigned.is_empty() {
                                ui.collapsing("Unassigned", |ui| {
                                    for (id, name, a_lat, a_lon, b_lat, b_lon) in &unassigned {
                                        ui.horizontal(|ui| {
                                            if ui.small_button("Load").clicked() {
                                                self.guide.load_ab_line(*a_lat, *a_lon, *b_lat, *b_lon);
                                                if let Some(db) = &self.db {
                                                    let _ = db.set_config("last_ab_line_id", &id.to_string());
                                                }
                                                self.ab_status_msg = Some((
                                                    format!("Loaded \"{}\"", name), 180
                                                ));
                                            }
                                            if ui.small_button("🗑").on_hover_text("Delete").clicked() {
                                                if let Some(db) = &self.db {
                                                    let _ = db.delete_ab_line(*id);
                                                    self.refresh_ab_cache();
                                                }
                                            }
                                            ui.label(egui::RichText::new(name).size(12.0));
                                        });
                                    }
                                });
                            }

                            // Fields with their lines
                            let field_ids: Vec<(i64, String)> = self.saved_fields
                                .iter().map(|f| (f.id, f.name.clone())).collect();
                            for (fid, fname) in field_ids {
                                let lines_in_field: Vec<(i64, String, f64, f64, f64, f64)> =
                                    self.saved_ab_lines.iter()
                                        .filter(|l| l.field_id == Some(fid))
                                        .map(|l| (l.id, l.name.clone(), l.a_lat, l.a_lon, l.b_lat, l.b_lon))
                                        .collect();
                                let count = lines_in_field.len();
                                let header = format!("📍 {} ({})", fname, count);

                                egui::CollapsingHeader::new(&header)
                                    .id_salt(fid)
                                    .show(ui, |ui| {
                                        if lines_in_field.is_empty() {
                                            ui.label(egui::RichText::new("No lines yet").size(11.0).weak());
                                        }
                                        for (id, name, a_lat, a_lon, b_lat, b_lon) in lines_in_field {
                                            ui.horizontal(|ui| {
                                                if ui.small_button("Load").clicked() {
                                                    self.guide.load_ab_line(a_lat, a_lon, b_lat, b_lon);
                                                    if let Some(db) = &self.db {
                                                        let _ = db.set_config("last_ab_line_id", &id.to_string());
                                                    }
                                                    self.ab_status_msg = Some((
                                                        format!("Loaded \"{}\"", name), 180
                                                    ));
                                                }
                                                if ui.small_button("🗑").on_hover_text("Delete line").clicked() {
                                                    if let Some(db) = &self.db {
                                                        let _ = db.delete_ab_line(id);
                                                        self.refresh_ab_cache();
                                                    }
                                                }
                                                ui.label(egui::RichText::new(&name).size(12.0));
                                            });
                                        }
                                        // Delete field button (only shown when inside the header)
                                        ui.add_space(2.0);
                                        if ui.small_button("🗑 Delete field").on_hover_text(
                                            "Remove field (lines become unassigned)"
                                        ).clicked() {
                                            if let Some(db) = &self.db {
                                                let _ = db.delete_field(fid);
                                                self.refresh_ab_cache();
                                            }
                                        }
                                    });
                            }
                        });

                    ui.add_space(4.0);

                    // ── Export / Import ───────────────────────────────────
                    ui.horizontal(|ui| {
                        if ui.small_button("⬆ Export JSON").clicked() {
                            if let Some(db) = &self.db {
                                match db.export_ab_lines_json() {
                                    Ok(bundle) => {
                                        match serde_json::to_string_pretty(&bundle) {
                                            Ok(json) => {
                                                let path = "data/finn_ab_lines.json";
                                                match std::fs::write(path, &json) {
                                                    Ok(_) => self.io_status_msg = Some((
                                                        format!("Exported to {}", path), 300
                                                    )),
                                                    Err(e) => self.io_status_msg = Some((
                                                        format!("Write failed: {}", e), 300
                                                    )),
                                                }
                                            }
                                            Err(e) => self.io_status_msg = Some((
                                                format!("Serialise failed: {}", e), 300
                                            )),
                                        }
                                    }
                                    Err(e) => self.io_status_msg = Some((
                                        format!("Export failed: {}", e), 300
                                    )),
                                }
                            }
                        }

                        if ui.small_button("⬇ Import JSON").clicked() {
                            let path = "data/finn_ab_lines.json";
                            match std::fs::read_to_string(path) {
                                Ok(json) => {
                                    match serde_json::from_str::<crate::coverage::db::ExportBundle>(&json) {
                                        Ok(bundle) => {
                                            if let Some(db) = &self.db {
                                                match db.import_ab_lines_json(&bundle) {
                                                    Ok(stats) => {
                                                        self.refresh_ab_cache();
                                                        self.io_status_msg = Some((
                                                            format!("Imported: {} fields, {} lines, {} skipped",
                                                                stats.fields_added, stats.lines_added, stats.lines_skipped),
                                                            360,
                                                        ));
                                                    }
                                                    Err(e) => self.io_status_msg = Some((
                                                        format!("Import failed: {}", e), 300
                                                    )),
                                                }
                                            }
                                        }
                                        Err(e) => self.io_status_msg = Some((
                                            format!("Parse failed: {}", e), 300
                                        )),
                                    }
                                }
                                Err(e) => self.io_status_msg = Some((
                                    format!("Read failed: {}", e), 300
                                )),
                            }
                        }
                    });

                    if let Some((ref msg, _)) = self.io_status_msg.clone() {
                        ui.label(egui::RichText::new(msg).size(10.0).weak());
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Implement Width & Overlap ===
                    ui.label(egui::RichText::new("IMPLEMENT").size(14.0).strong());
                    ui.add_space(4.0);

                    // Width control
                    ui.label("Width");
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("−").min_size(egui::vec2(36.0, 30.0))).clicked() {
                            let new_width = (self.guide.implement_width_m - 0.5).max(0.5);
                            self.guide.implement_width_m = new_width;
                            self.coverage.set_implement_width(new_width);
                            self.guide.pass_offset_m = self.guide.pass_spacing() * self.guide.pass_number as f64;
                            if let Some(db) = &self.db {
                                let _ = db.set_config("implement_width_m", &format!("{:.1}", new_width));
                            }
                        }
                        ui.label(
                            egui::RichText::new(format!("{:.1} m", self.guide.implement_width_m))
                                .size(18.0).strong()
                        );
                        if ui.add(egui::Button::new("+").min_size(egui::vec2(36.0, 30.0))).clicked() {
                            let new_width = (self.guide.implement_width_m + 0.5).min(36.0);
                            self.guide.implement_width_m = new_width;
                            self.coverage.set_implement_width(new_width);
                            self.guide.pass_offset_m = self.guide.pass_spacing() * self.guide.pass_number as f64;
                            if let Some(db) = &self.db {
                                let _ = db.set_config("implement_width_m", &format!("{:.1}", new_width));
                            }
                        }
                    });

                    ui.add_space(4.0);

                    // Overlap control (in cm, stored as metres)
                    let overlap_cm = (self.guide.overlap_m * 100.0).round() as i32;
                    ui.label("Overlap");
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("−").min_size(egui::vec2(36.0, 30.0))).clicked() {
                            let new_cm = (overlap_cm - 5).max(0);
                            self.guide.overlap_m = new_cm as f64 / 100.0;
                            self.guide.pass_offset_m = self.guide.pass_spacing() * self.guide.pass_number as f64;
                            if let Some(db) = &self.db {
                                let _ = db.set_config("overlap_m", &format!("{:.2}", self.guide.overlap_m));
                            }
                        }
                        ui.label(
                            egui::RichText::new(format!("{} cm", overlap_cm))
                                .size(18.0).strong()
                        );
                        if ui.add(egui::Button::new("+").min_size(egui::vec2(36.0, 30.0))).clicked() {
                            // Cap overlap at 90% of implement width
                            let max_cm = ((self.guide.implement_width_m * 0.9) * 100.0) as i32;
                            let new_cm = (overlap_cm + 5).min(max_cm);
                            self.guide.overlap_m = new_cm as f64 / 100.0;
                            self.guide.pass_offset_m = self.guide.pass_spacing() * self.guide.pass_number as f64;
                            if let Some(db) = &self.db {
                                let _ = db.set_config("overlap_m", &format!("{:.2}", self.guide.overlap_m));
                            }
                        }
                    });

                    // Show effective pass spacing
                    ui.add_space(2.0);
                    ui.label(
                        egui::RichText::new(format!("Pass spacing: {:.2} m", self.guide.pass_spacing()))
                            .size(12.0).weak()
                    );

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Nudge Section ===
                    // Fine lateral shift of the entire AB line system.
                    // Used for inter-row sowing (shift to stitch tines between
                    // last year's furrows) or correcting overlap/underlap.
                    ui.label(egui::RichText::new("NUDGE").size(14.0).strong());
                    ui.add_space(4.0);

                    let nudge_cm = (self.guide.nudge_m * 100.0).round() as i32;
                    let nudge_colour = if nudge_cm == 0 {
                        egui::Color32::GRAY
                    } else {
                        egui::Color32::from_rgb(255, 200, 60) // amber when active
                    };

                    // Current nudge value display
                    ui.label(
                        egui::RichText::new(format!(
                            "Offset: {} cm{}",
                            nudge_cm,
                            if nudge_cm == 0 { "" } else if nudge_cm > 0 { " →R" } else { " ←L" }
                        ))
                        .size(18.0)
                        .strong()
                        .color(nudge_colour),
                    );

                    ui.add_space(4.0);

                    // Standard ±5 cm nudge buttons
                    ui.label(egui::RichText::new("5 cm steps:").size(11.0).weak());
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("◄◄ 5").min_size(egui::vec2(56.0, 30.0))).clicked() {
                            self.guide.nudge_left(0.05);
                        }
                        if ui.add(egui::Button::new("Reset").min_size(egui::vec2(50.0, 30.0))).clicked() {
                            self.guide.nudge_reset();
                        }
                        if ui.add(egui::Button::new("5 ►►").min_size(egui::vec2(56.0, 30.0))).clicked() {
                            self.guide.nudge_right(0.05);
                        }
                    });

                    ui.add_space(2.0);

                    // Fine ±1 cm nudge buttons
                    ui.label(egui::RichText::new("1 cm fine:").size(11.0).weak());
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("◄ 1").min_size(egui::vec2(56.0, 30.0))).clicked() {
                            self.guide.nudge_left(0.01);
                        }
                        ui.add_space(50.0); // keep alignment with row above
                        if ui.add(egui::Button::new("1 ►").min_size(egui::vec2(56.0, 30.0))).clicked() {
                            self.guide.nudge_right(0.01);
                        }
                    });

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Guidance Readout ===
                    ui.label(egui::RichText::new("GUIDANCE").size(14.0).strong());
                    ui.add_space(4.0);

                    if let Some(error) = &self.current_error {
                        let xtd_cm = (error.distance_m * 100.0) as i32;
                        let colour = if xtd_cm.abs() < 5 {
                            egui::Color32::GREEN
                        } else if xtd_cm.abs() < 15 {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::RED
                        };
                        ui.colored_label(
                            colour,
                            egui::RichText::new(format!("{} cm", xtd_cm)).size(32.0).strong(),
                        );
                        ui.label(if xtd_cm < 0 { "◄ LEFT" } else { "RIGHT ►" });
                        ui.add_space(4.0);
                        ui.label(format!("Heading err: {:.1}°", error.heading_error));
                    } else {
                        ui.label("No guidance line set");
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Coverage Section ===
                    ui.label(egui::RichText::new("COVERAGE").size(14.0).strong());
                    ui.add_space(4.0);

                    // Engage button (also available here for convenience)
                    let engage_text = if self.coverage.is_engaged() { "⏹ DISENGAGE" } else { "▶ ENGAGE" };
                    let engage_colour = if self.coverage.is_engaged() {
                        egui::Color32::from_rgb(255, 60, 60)
                    } else {
                        egui::Color32::from_rgb(60, 200, 60)
                    };
                    if ui.add(egui::Button::new(
                        egui::RichText::new(engage_text).strong().color(engage_colour)
                    ).min_size(egui::vec2(150.0, 30.0))).clicked() {
                        self.coverage.toggle_engage(self.db.as_ref());
                    }

                    if self.coverage.is_engaged() {
                        ui.colored_label(
                            egui::Color32::from_rgb(255, 60, 60),
                            format!("● Recording - Seg {}", self.coverage.segment()),
                        );
                    } else {
                        ui.label("Not recording");
                    }
                    ui.label(format!("Points: {}", self.coverage.total_points()));

                    ui.add_space(4.0);

                    // Clear coverage: wipe in-memory display for starting a new task.
                    // CSV files on disk are untouched.
                    let can_clear = self.coverage.total_points() > 0 || !self.coverage.points().is_empty();
                    let clear_btn = egui::Button::new(
                        egui::RichText::new("🗑 Clear Coverage").size(13.0)
                    ).min_size(egui::vec2(150.0, 30.0));
                    let clear_resp = ui.add_enabled(can_clear && !self.coverage.is_engaged(), clear_btn);
                    if clear_resp.clicked() {
                        self.coverage.clear_coverage(self.db.as_ref());
                        self.ab_status_msg = Some((
                            "Coverage cleared — DB data untouched".to_string(), 240
                        ));
                    }
                    if self.coverage.is_engaged() && can_clear {
                        ui.label(egui::RichText::new("Disengage first").size(11.0).weak());
                    }

                    ui.add_space(4.0);

                    // Job history list
                    ui.label(egui::RichText::new("JOB HISTORY").size(12.0).strong());
                    ui.add_space(2.0);
                    if let Some(db) = &self.db {
                        if let Ok(jobs) = db.list_jobs() {
                            if jobs.is_empty() {
                                ui.label(egui::RichText::new("No jobs recorded yet").size(11.0).weak());
                            } else {
                                // Clone job data to avoid borrow conflict with self
                                let job_data: Vec<(i64, String, String, i64, i32)> = jobs.iter()
                                    .take(10) // Show last 10 jobs
                                    .map(|j| (j.id, j.name.clone(), j.started_at.clone(), j.total_points, j.total_segments))
                                    .collect();

                                egui::ScrollArea::vertical()
                                    .id_salt("job_history_list")
                                    .max_height(120.0)
                                    .show(ui, |ui| {
                                        for (id, name, started, points, segments) in &job_data {
                                            ui.horizontal(|ui| {
                                                // Extract just date+time from the started_at string
                                                let display_date = if started.len() >= 16 {
                                                    &started[..16]
                                                } else {
                                                    started.as_str()
                                                };
                                                ui.label(egui::RichText::new(
                                                    format!("{} — {}pts, {}segs", display_date, points, segments)
                                                ).size(11.0));
                                                if ui.small_button("🗑").on_hover_text(
                                                    format!("Delete job\n{}", name)
                                                ).clicked() {
                                                    if let Some(db) = &self.db {
                                                        let _ = db.delete_job(*id);
                                                    }
                                                }
                                            });
                                        }
                                    });
                            }
                        }
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Position Section ===
                    ui.label(egui::RichText::new("POSITION").size(14.0).strong());
                    ui.add_space(4.0);

                    if let Some(fix) = &self.current_fix {
                        ui.label(format!("Lat:  {:.7}°", fix.latitude));
                        ui.label(format!("Lon:  {:.7}°", fix.longitude));
                        ui.label(format!("Alt:  {:.1} m", fix.altitude));
                        ui.add_space(4.0);
                        ui.label(format!("Speed:   {:.1} km/h", fix.speed * 3.6));
                        ui.label(format!("Heading: {:.1}°", fix.heading));
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Sensors Section (ESP32 data) ===
                    ui.label(egui::RichText::new("SENSORS").size(14.0).strong());
                    ui.add_space(4.0);

                    // WAS — show raw + calibrated angle if calibrated
                    if let Some(was) = &self.latest_was {
                        ui.label(format!("WAS:  {} raw  ({} mV)", was.raw_value, was.voltage_mv));
                        if let Some(angle) = self.was_calibrated_angle(was.raw_value) {
                            let dir = if angle < -0.5 { "LEFT" } else if angle > 0.5 { "RIGHT" } else { "CENTRE" };
                            ui.label(
                                egui::RichText::new(format!("Angle: {:.1}° {}", angle, dir))
                                    .size(14.0).strong()
                            );
                        }
                    } else {
                        ui.label(egui::RichText::new("WAS: no data").size(12.0).weak());
                    }

                    // IMU
                    if let Some(imu) = &self.latest_imu {
                        ui.label(format!("IMU:  R{:.1}° P{:.1}° H{:.1}°", imu.roll, imu.pitch, imu.heading));
                        let cal_colour = if imu.cal_sys >= 2 {
                            egui::Color32::GREEN
                        } else if imu.cal_sys >= 1 {
                            egui::Color32::YELLOW
                        } else {
                            egui::Color32::from_rgb(255, 120, 60)
                        };
                        ui.colored_label(cal_colour, format!(
                            "Cal: S{} G{} A{} M{}",
                            imu.cal_sys, imu.cal_gyro, imu.cal_accel, imu.cal_mag
                        ));
                    } else {
                        ui.label(egui::RichText::new("IMU: no data").size(12.0).weak());
                    }

                    // Motor
                    if let Some(mtr) = &self.latest_motor {
                        let en_label = if mtr.enabled { "ON" } else { "OFF" };
                        ui.label(format!("Motor: PWM {} [{}]", mtr.current_pwm, en_label));
                    }

                    // Heartbeat
                    if let Some(uptime) = self.sensor_uptime_ms {
                        let secs = uptime / 1000;
                        let mins = secs / 60;
                        ui.label(
                            egui::RichText::new(format!("ESP32 uptime: {}m {}s", mins, secs % 60))
                                .size(11.0).weak()
                        );
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === WAS Calibration Section ===
                    // Three-point calibration: centre, left lock, right lock.
                    // Stores raw ADC values in SQLite config table. The PC computes
                    // the steering angle from raw ADC using piecewise linear mapping.
                    ui.label(egui::RichText::new("WAS CALIBRATION").size(14.0).strong());
                    ui.add_space(4.0);

                    let has_was_data = self.latest_was.is_some();
                    let is_calibrated = self.was_centre.is_some()
                        && self.was_left_lock.is_some()
                        && self.was_right_lock.is_some();

                    // Status display
                    if is_calibrated {
                        ui.colored_label(egui::Color32::GREEN, "● Calibrated");
                        ui.label(egui::RichText::new(format!(
                            "L:{} C:{} R:{}",
                            self.was_left_lock.unwrap(),
                            self.was_centre.unwrap(),
                            self.was_right_lock.unwrap(),
                        )).size(11.0).weak());
                    } else {
                        ui.colored_label(egui::Color32::from_rgb(255, 200, 60), "● Not calibrated");
                    }

                    ui.add_space(4.0);

                    // Live WAS readout for feedback during calibration
                    if let Some(was) = &self.latest_was {
                        ui.label(
                            egui::RichText::new(format!("Current: {} raw", was.raw_value))
                                .size(14.0).strong()
                        );
                    }

                    ui.add_space(4.0);

                    // Step 1: Set Centre
                    ui.label(egui::RichText::new("1. Wheels straight:").size(12.0));
                    let centre_btn = egui::Button::new(
                        egui::RichText::new("Set Centre").size(13.0)
                    ).min_size(egui::vec2(120.0, 30.0));
                    if ui.add_enabled(has_was_data, centre_btn).clicked() {
                        if let Some(was) = &self.latest_was {
                            self.was_centre = Some(was.raw_value);
                            if let Some(db) = &self.db {
                                let _ = db.set_config("was_centre", &was.raw_value.to_string());
                            }
                            self.was_cal_msg = Some((
                                format!("Centre set: {}", was.raw_value), 180
                            ));
                        }
                    }

                    ui.add_space(2.0);

                    // Step 2: Set Left Lock
                    ui.label(egui::RichText::new("2. Full LEFT lock:").size(12.0));
                    let left_btn = egui::Button::new(
                        egui::RichText::new("Set Left Lock").size(13.0)
                    ).min_size(egui::vec2(120.0, 30.0));
                    if ui.add_enabled(has_was_data, left_btn).clicked() {
                        if let Some(was) = &self.latest_was {
                            self.was_left_lock = Some(was.raw_value);
                            if let Some(db) = &self.db {
                                let _ = db.set_config("was_left_lock", &was.raw_value.to_string());
                            }
                            self.was_cal_msg = Some((
                                format!("Left lock set: {}", was.raw_value), 180
                            ));
                        }
                    }

                    ui.add_space(2.0);

                    // Step 3: Set Right Lock
                    ui.label(egui::RichText::new("3. Full RIGHT lock:").size(12.0));
                    let right_btn = egui::Button::new(
                        egui::RichText::new("Set Right Lock").size(13.0)
                    ).min_size(egui::vec2(120.0, 30.0));
                    if ui.add_enabled(has_was_data, right_btn).clicked() {
                        if let Some(was) = &self.latest_was {
                            self.was_right_lock = Some(was.raw_value);
                            if let Some(db) = &self.db {
                                let _ = db.set_config("was_right_lock", &was.raw_value.to_string());
                            }
                            self.was_cal_msg = Some((
                                format!("Right lock set: {}", was.raw_value), 180
                            ));
                        }
                    }

                    ui.add_space(4.0);

                    // Recalibrate button — clears all three values
                    if is_calibrated {
                        if ui.add(egui::Button::new(
                            egui::RichText::new("↻ Recalibrate").size(12.0)
                        ).min_size(egui::vec2(120.0, 28.0))).clicked() {
                            self.was_centre = None;
                            self.was_left_lock = None;
                            self.was_right_lock = None;
                            if let Some(db) = &self.db {
                                // Remove the config keys (set to empty to effectively clear)
                                let _ = db.set_config("was_centre", "");
                                let _ = db.set_config("was_left_lock", "");
                                let _ = db.set_config("was_right_lock", "");
                            }
                            self.was_cal_msg = Some(("Calibration cleared".to_string(), 180));
                        }
                    }

                    if !has_was_data {
                        ui.label(egui::RichText::new("Waiting for WAS data...").size(11.0).weak());
                    }

                    if let Some((ref msg, _)) = self.was_cal_msg {
                        ui.label(egui::RichText::new(msg).size(11.0)
                            .color(egui::Color32::from_rgb(100, 220, 100)));
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Motor Direction Section ===
                    ui.label(egui::RichText::new("MOTOR DIRECTION").size(14.0).strong());
                    ui.add_space(4.0);

                    let invert_label = if self.motor_invert {
                        "+PWM = steer LEFT"
                    } else {
                        "+PWM = steer RIGHT"
                    };
                    let invert_colour = if self.motor_invert {
                        egui::Color32::from_rgb(255, 200, 60)
                    } else {
                        egui::Color32::GREEN
                    };
                    ui.colored_label(invert_colour,
                        egui::RichText::new(invert_label).size(14.0).strong()
                    );
                    ui.add_space(2.0);
                    ui.label(egui::RichText::new(
                        "Use MOTOR TEST to verify, then toggle if needed"
                    ).size(11.0).weak());
                    ui.add_space(4.0);

                    if ui.add(egui::Button::new(
                        egui::RichText::new("Invert Motor Direction").size(13.0)
                    ).min_size(egui::vec2(160.0, 30.0))).clicked() {
                        self.motor_invert = !self.motor_invert;
                        if let Some(db) = &self.db {
                            let _ = db.set_config("motor_invert",
                                if self.motor_invert { "true" } else { "false" }
                            );
                        }
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Auto-Steer Settings Section ===
                    ui.label(egui::RichText::new("AUTO-STEER").size(14.0).strong());
                    ui.add_space(4.0);

                    // Status
                    if self.steering.engaged {
                        ui.colored_label(egui::Color32::GREEN,
                            egui::RichText::new(format!("● ENGAGED  PWM {}", self.steering.last_output_pwm))
                                .size(14.0).strong()
                        );
                        // Show inner loop debug: desired vs actual angle
                        ui.label(egui::RichText::new(format!(
                            "Target: {:.1}°  Actual: {:.1}°",
                            self.steering.last_desired_angle,
                            self.steering.last_actual_angle,
                        )).size(12.0));

                        // WAS stale warning
                        if self.steering.was_stale {
                            ui.colored_label(
                                egui::Color32::from_rgb(255, 200, 60),
                                egui::RichText::new("⚠ WAS data stale — using last known angle").size(11.0)
                            );
                        }
                    } else {
                        ui.colored_label(egui::Color32::GRAY, "● Disengaged");
                        if let Some(ref reason) = self.steering.disengage_reason {
                            ui.label(egui::RichText::new(format!("Last: {}", reason)).size(11.0).weak());
                        }
                    }

                    ui.add_space(6.0);

                    // Outer loop: Kp (XTE → desired angle)
                    ui.label(egui::RichText::new("Outer loop — line seeking:").size(12.0).strong());
                    ui.add_space(2.0);
                    ui.label(egui::RichText::new("Kp (°/m):").size(12.0));
                    let old_kp = self.steering.kp;
                    ui.add(egui::Slider::new(&mut self.steering.kp, 5.0..=60.0)
                        .step_by(1.0)
                        .suffix(" °/m")
                    );
                    if (self.steering.kp - old_kp).abs() > 0.1 {
                        if let Some(db) = &self.db {
                            let _ = db.set_config("steer_kp", &format!("{:.0}", self.steering.kp));
                        }
                    }
                    ui.label(egui::RichText::new(
                        format!("1m off-line → {:.0}° desired turn", self.steering.kp)
                    ).size(11.0).weak());

                    ui.add_space(6.0);

                    // Inner loop: Kp_angle (angle error → PWM)
                    ui.label(egui::RichText::new("Inner loop — wheel position:").size(12.0).strong());
                    ui.add_space(2.0);
                    ui.label(egui::RichText::new("Kp angle (PWM/°):").size(12.0));
                    let old_kp_angle = self.steering.kp_angle;
                    ui.add(egui::Slider::new(&mut self.steering.kp_angle, 1.0..=15.0)
                        .step_by(0.5)
                        .suffix(" PWM/°")
                    );
                    if (self.steering.kp_angle - old_kp_angle).abs() > 0.1 {
                        if let Some(db) = &self.db {
                            let _ = db.set_config("steer_kp_angle", &format!("{:.1}", self.steering.kp_angle));
                        }
                    }
                    ui.label(egui::RichText::new(
                        format!("10° error → {} PWM", (self.steering.kp_angle * 10.0) as i32)
                    ).size(11.0).weak());

                    ui.add_space(4.0);

                    // Max PWM slider
                    ui.label(egui::RichText::new("Max PWM:").size(12.0));
                    let mut max_pwm_f = self.steering.max_pwm as f64;
                    ui.add(egui::Slider::new(&mut max_pwm_f, 50.0..=255.0)
                        .step_by(5.0)
                    );
                    self.steering.max_pwm = max_pwm_f as i16;

                    ui.add_space(4.0);

                    // Min PWM slider (motor deadzone compensation)
                    ui.label(egui::RichText::new("Min PWM (motor deadzone):").size(12.0));
                    let mut min_pwm_f = self.steering.min_pwm as f64;
                    ui.add(egui::Slider::new(&mut min_pwm_f, 0.0..=120.0)
                        .step_by(5.0)
                    );
                    self.steering.min_pwm = min_pwm_f as i16;
                    ui.label(egui::RichText::new(
                        "Motor won't spin below this — set to lowest PWM that moves"
                    ).size(11.0).weak());

                    ui.add_space(4.0);

                    // Deadband slider
                    let mut deadband_cm = self.steering.deadband_m * 100.0;
                    ui.label(egui::RichText::new("Deadband:").size(12.0));
                    ui.add(egui::Slider::new(&mut deadband_cm, 0.0..=20.0)
                        .step_by(1.0)
                        .suffix(" cm")
                    );
                    self.steering.deadband_m = deadband_cm / 100.0;

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Motor Test Section ===
                    ui.label(egui::RichText::new("MOTOR TEST").size(14.0).strong());
                    ui.add_space(4.0);

                    if self.motor_handle.is_connected() {
                        ui.colored_label(egui::Color32::GREEN, "● Motor ESP32 connected");
                        ui.add_space(4.0);

                        // Current test PWM display
                        ui.label(
                            egui::RichText::new(format!("PWM: {}", self.test_pwm))
                                .size(18.0).strong()
                        );

                        ui.add_space(4.0);

                        // Preset buttons
                        ui.label(egui::RichText::new("Presets:").size(11.0).weak());
                        ui.horizontal(|ui| {
                            for &pwm in &[-100i16, -50, -25, 0, 25, 50, 100] {
                                let label = if pwm == 0 {
                                    "STOP".to_string()
                                } else {
                                    format!("{}", pwm)
                                };
                                let colour = if pwm == 0 {
                                    egui::Color32::from_rgb(255, 60, 60)
                                } else {
                                    egui::Color32::LIGHT_GRAY
                                };
                                if ui.add(egui::Button::new(
                                    egui::RichText::new(&label).size(12.0).color(colour)
                                ).min_size(egui::vec2(38.0, 28.0))).clicked() {
                                    self.test_pwm = pwm;
                                    match self.motor_handle.send_steer(pwm) {
                                        Ok(_) => {
                                            self.motor_test_msg = Some((
                                                format!("Sent PWM {}", pwm), 120
                                            ));
                                        }
                                        Err(e) => {
                                            self.motor_test_msg = Some((
                                                format!("Send failed: {}", e), 240
                                            ));
                                        }
                                    }
                                }
                            }
                        });

                        ui.add_space(4.0);

                        // Fine adjust
                        ui.horizontal(|ui| {
                            if ui.add(egui::Button::new("−10").min_size(egui::vec2(40.0, 28.0))).clicked() {
                                self.test_pwm = (self.test_pwm - 10).max(-255);
                                let _ = self.motor_handle.send_steer(self.test_pwm);
                            }
                            if ui.add(egui::Button::new("+10").min_size(egui::vec2(40.0, 28.0))).clicked() {
                                self.test_pwm = (self.test_pwm + 10).min(255);
                                let _ = self.motor_handle.send_steer(self.test_pwm);
                            }
                            // Emergency stop
                            if ui.add(egui::Button::new(
                                egui::RichText::new("⏹ STOP").size(14.0).strong()
                                    .color(egui::Color32::from_rgb(255, 60, 60))
                            ).min_size(egui::vec2(70.0, 28.0))).clicked() {
                                self.test_pwm = 0;
                                let _ = self.motor_handle.send_steer(0);
                                self.motor_test_msg = Some(("Motor stopped".to_string(), 120));
                            }
                        });

                        // Motor status feedback
                        if let Some(mtr) = &self.latest_motor {
                            ui.add_space(4.0);
                            let en_colour = if mtr.enabled {
                                egui::Color32::GREEN
                            } else {
                                egui::Color32::GRAY
                            };
                            ui.colored_label(en_colour, format!(
                                "Motor: PWM {} | {} | {}s uptime",
                                mtr.current_pwm,
                                if mtr.enabled { "ENABLED" } else { "DISABLED" },
                                mtr.uptime_ms / 1000
                            ));
                        }

                        // WAS feedback for verifying motor→steering→WAS loop
                        if let Some(was) = &self.latest_was {
                            ui.label(format!("WAS feedback: {} raw ({} mV)", was.raw_value, was.voltage_mv));
                        }
                    } else {
                        ui.colored_label(egui::Color32::GRAY, "● Motor ESP32 not connected");
                        ui.label(egui::RichText::new("Plug in motor ESP32 USB and restart").size(11.0).weak());
                    }

                    if let Some((ref msg, _)) = self.motor_test_msg {
                        ui.label(egui::RichText::new(msg).size(11.0)
                            .color(egui::Color32::from_rgb(100, 220, 100)));
                    }

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === View Controls ===
                    ui.label(egui::RichText::new("VIEW").size(14.0).strong());
                    ui.add_space(4.0);

                    ui.horizontal(|ui| {
                        let mode_text = if self.field_view.heading_up { "Heading ▲" } else { "North ▲" };
                        if ui.button(mode_text).clicked() {
                            self.field_view.heading_up = !self.field_view.heading_up;
                            if !self.field_view.heading_up {
                                self.field_view.projection.rotation_rad = 0.0;
                            }
                        }
                        if ui.button("Grid").clicked() {
                            self.field_view.show_grid = !self.field_view.show_grid;
                        }
                    });

                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("+").min_size(egui::vec2(40.0, 30.0))).clicked() {
                            self.field_view.projection.zoom_in();
                        }
                        if ui.add(egui::Button::new("−").min_size(egui::vec2(40.0, 30.0))).clicked() {
                            self.field_view.projection.zoom_out();
                        }
                    });

                    let vis_w = self.field_view.projection.visible_width_m();
                    if vis_w >= 1000.0 {
                        ui.label(format!("Visible: {:.1} km", vis_w / 1000.0));
                    } else {
                        ui.label(format!("Visible: {:.0} m", vis_w));
                    }
                    ui.label(format!("Grid: {} m", self.field_view.projection.grid_spacing_m() as i32));

                    ui.add_space(10.0);
                    ui.separator();
                    ui.add_space(10.0);

                    // === Lightbar Sensitivity ===
                    ui.label(egui::RichText::new("LIGHTBAR").size(14.0).strong());
                    ui.add_space(4.0);
                    let cm_val = self.lightbar_cm_per_seg as i32;
                    let full_scale = cm_val * 15;
                    ui.label(format!("{} cm/seg ({} cm full scale)", cm_val, full_scale));
                    ui.horizontal(|ui| {
                        if ui.add(egui::Button::new("−").min_size(egui::vec2(36.0, 30.0))).clicked() {
                            let new_val = (self.lightbar_cm_per_seg - 1.0).max(1.0);
                            self.lightbar_cm_per_seg = new_val;
                            if let Some(db) = &self.db {
                                let _ = db.set_config("lightbar_cm_per_seg", &format!("{:.0}", new_val));
                            }
                        }
                        ui.label(
                            egui::RichText::new(format!("{} cm", cm_val))
                                .size(18.0).strong()
                        );
                        if ui.add(egui::Button::new("+").min_size(egui::vec2(36.0, 30.0))).clicked() {
                            let new_val = (self.lightbar_cm_per_seg + 1.0).min(50.0);
                            self.lightbar_cm_per_seg = new_val;
                            if let Some(db) = &self.db {
                                let _ = db.set_config("lightbar_cm_per_seg", &format!("{:.0}", new_val));
                            }
                        }
                    });
                    ui.label(
                        egui::RichText::new("Lower = more sensitive (use with RTK)")
                            .size(11.0).weak()
                    );
                });
            });

        // Central panel: field view (smaller, with side panel taking space)
        let canvas_frame = egui::Frame::default()
            .inner_margin(egui::Margin::same(0.0))
            .fill(egui::Color32::TRANSPARENT);

        egui::CentralPanel::default()
            .frame(canvas_frame)
            .show(ctx, |ui| {
                self.field_view.draw(
                    ui,
                    &self.display_fix,
                    &self.position_trail,
                    &self.guide,
                    self.coverage.points(),
                    self.coverage.implement_width(),
                );
            });
    }
}

/// Returns (colour, label) for a GPS fix quality value
fn fix_quality_display(quality: FixQuality) -> (egui::Color32, &'static str) {
    match quality {
        FixQuality::Rtk => (egui::Color32::GREEN, "RTK Fixed"),
        FixQuality::RtkFloat => (egui::Color32::YELLOW, "RTK Float"),
        FixQuality::Dgps => (egui::Color32::LIGHT_BLUE, "DGPS"),
        FixQuality::Gps => (egui::Color32::ORANGE, "GPS"),
        FixQuality::NoFix => (egui::Color32::RED, "No Fix"),
    }
}

/// Compute the lightbar segment colour based on distance from centre.
/// Green near centre → yellow in the middle → red at the edges.
fn lightbar_colour(distance: f32, max_distance: f32) -> egui::Color32 {
    if distance < 0.5 {
        // Centre segment — bright green
        return egui::Color32::from_rgb(0, 255, 0);
    }
    let t = (distance / max_distance).clamp(0.0, 1.0);
    if t < 0.5 {
        // Green → Yellow (first half)
        let u = t * 2.0; // 0..1 within this band
        egui::Color32::from_rgb(
            (255.0 * u) as u8,
            255,
            0,
        )
    } else {
        // Yellow → Red (second half)
        let u = (t - 0.5) * 2.0; // 0..1 within this band
        egui::Color32::from_rgb(
            255,
            (255.0 * (1.0 - u)) as u8,
            0,
        )
    }
}
