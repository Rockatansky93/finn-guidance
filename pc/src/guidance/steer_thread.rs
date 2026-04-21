//! Dedicated steering thread — decoupled from GUI frame rate.
//!
//! ## Why this exists
//!
//! Previously, the steering compute and motor serial write lived inside the
//! egui `update()` method, meaning steer commands were hostage to the GUI
//! frame rate. If field view rendering took 50ms, the steer command that
//! should have gone out at T+100ms slipped to T+150ms. On the Dell 7390
//! with coverage polygons and trail rendering, this added 20-50ms of
//! unpredictable jitter to the control loop.
//!
//! This thread runs a fixed 100ms loop (10Hz, matching the GPS fix rate)
//! independent of rendering. The GUI becomes display-only for steering —
//! it reads the latest state from `SharedSteerState` for the HUD overlay
//! but never calls `compute()` or `send_steer_angle()`.
//!
//! ## Architecture
//!
//! ```text
//! GPS thread ──► gps_rx ──► steer_thread (10Hz loop)
//!                                │
//! Motor thread ──► finn_rx ──────┤
//!                                │
//!                          SharedSteerState (Arc<Mutex<>>)
//!                                │
//!                           GUI thread (reads for display)
//! ```
//!
//! The GUI communicates commands (engage, disengage, tuning changes, AB line
//! updates) by writing to `SharedSteerState`. The steer thread picks them up
//! on the next loop iteration.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use crossbeam_channel::Receiver;
use finn_guidance_common::types::GpsFix;
use finn_guidance_common::protocol::FinnMessage;
use crate::comms::serial::MotorHandle;
use crate::guidance::ab_line::AbLineGuide;
use crate::guidance::steering::SteeringController;
use crate::position::interpolator::PositionInterpolator;

/// Fixed loop interval — 10Hz matches the LC29H BA GPS output rate.
const STEER_LOOP_INTERVAL: Duration = Duration::from_millis(100);

// ─────────────────────────────────────────────────────────────────────
// Shared state between steer thread and GUI
// ─────────────────────────────────────────────────────────────────────

/// Commands from the GUI to the steer thread.
/// Written by the GUI, consumed by the steer thread on each loop tick.
#[derive(Debug, Clone)]
pub enum SteerCommand {
    Engage,
    Disengage,
}

/// Snapshot of steering state for GUI display.
/// Written by the steer thread, read by the GUI.
#[derive(Debug, Clone)]
pub struct SteerDisplayState {
    pub engaged: bool,
    pub desired_angle: f64,
    pub actual_angle: f64,
    pub output_pwm: i16,
    pub heading_error: f64,
    pub lookahead_m: f64,
    pub disengage_reason: Option<String>,
    /// Set to true for one read cycle when disengage happens, so GUI can show message
    pub just_disengaged: bool,
}

impl Default for SteerDisplayState {
    fn default() -> Self {
        Self {
            engaged: false,
            desired_angle: 0.0,
            actual_angle: 0.0,
            output_pwm: 0,
            heading_error: 0.0,
            lookahead_m: 0.0,
            disengage_reason: None,
            just_disengaged: false,
        }
    }
}

/// Thread-safe shared state for steering.
pub struct SharedSteerState {
    /// Commands queued by the GUI for the steer thread.
    pub commands: Vec<SteerCommand>,

    /// Latest display snapshot (written by steer thread, read by GUI).
    pub display: SteerDisplayState,

    /// Tuning parameters — GUI writes, steer thread reads.
    pub lookahead_base: f64,
    pub lookahead_speed_factor: f64,
    pub wheelbase_m: f64,
    pub max_steer_angle: f64,
    pub kd_xte: f64,
    pub deadband_m: f64,

    /// AB line — GUI writes new A/B points, steer thread reads for XTE calc.
    /// We store the four coords + metadata the steer thread needs.
    pub ab_line: Option<AbLineData>,
    pub implement_width_m: f64,
    pub overlap_m: f64,
    pub pass_number: i32,
    pub nudge_m: f64,

    /// Whether the AB line data has changed since the steer thread last read it.
    pub ab_line_dirty: bool,
}

/// Minimal AB line data for the steer thread's own AbLineGuide copy.
#[derive(Debug, Clone)]
pub struct AbLineData {
    pub a_lat: f64,
    pub a_lon: f64,
    pub b_lat: f64,
    pub b_lon: f64,
}

impl SharedSteerState {
    pub fn new(
        lookahead_base: f64,
        lookahead_speed_factor: f64,
        wheelbase_m: f64,
        max_steer_angle: f64,
        kd_xte: f64,
        deadband_m: f64,
        implement_width_m: f64,
        overlap_m: f64,
    ) -> Self {
        Self {
            commands: Vec::new(),
            display: SteerDisplayState::default(),
            lookahead_base,
            lookahead_speed_factor,
            wheelbase_m,
            max_steer_angle,
            kd_xte,
            deadband_m,
            ab_line: None,
            implement_width_m,
            overlap_m,
            pass_number: 0,
            nudge_m: 0.0,
            ab_line_dirty: false,
        }
    }
}

/// Thread-safe handle to the shared state.
pub type SteerStateHandle = Arc<Mutex<SharedSteerState>>;

// ─────────────────────────────────────────────────────────────────────
// Steer thread entry point
// ─────────────────────────────────────────────────────────────────────

/// Run the steering loop. Blocks — call from a dedicated thread.
///
/// Reads GPS fixes and motor status from their channels, computes
/// guidance + steering at a fixed 10Hz, and sends steer commands
/// directly to the motor ESP32. All display state is written to
/// `shared` for the GUI to read.
pub fn run_steer_thread(
    gps_rx: Receiver<GpsFix>,
    finn_rx: Receiver<FinnMessage>,
    motor_handle: MotorHandle,
    shared: SteerStateHandle,
) {
    tracing::info!("Steer thread started (10Hz fixed loop)");

    let mut steering = SteeringController::new();
    let mut guide = AbLineGuide::new(12.0);
    let mut interpolator = PositionInterpolator::new();

    // Latest fix and motor state (local to this thread)
    let mut _latest_fix: Option<GpsFix> = None;
    let mut latest_motor_angle: f64 = 0.0;
    let mut latest_motor_pwm: i16 = 0;

    loop {
        let loop_start = Instant::now();

        // ── 1. Drain GPS fixes ──────────────────────────────────────
        while let Ok(fix) = gps_rx.try_recv() {
            interpolator.update_fix(&fix);
            steering.notify_gps_fix();
            _latest_fix = Some(fix);
        }

        // ── 2. Drain motor status messages ──────────────────────────
        while let Ok(msg) = finn_rx.try_recv() {
            match msg {
                FinnMessage::MotorStatus(mtr) => {
                    latest_motor_angle = mtr.actual_angle;
                    latest_motor_pwm = mtr.current_pwm;
                }
                FinnMessage::ConfigAck(_) => {
                    // Config acks are informational; GUI handles display
                }
            }
        }

        // ── 3. Process commands and sync tuning from GUI ────────────
        {
            let mut state = shared.lock().unwrap();

            // Process commands
            let commands: Vec<SteerCommand> = state.commands.drain(..).collect();
            for cmd in commands {
                match cmd {
                    SteerCommand::Engage => {
                        steering.engage();
                        tracing::info!("Steer thread: ENGAGED");
                    }
                    SteerCommand::Disengage => {
                        steering.disengage(Some("Manual disengage".to_string()));
                        let _ = motor_handle.send_steer_angle(0.0);
                        tracing::info!("Steer thread: DISENGAGED");
                    }
                }
            }

            // Sync tuning parameters from GUI
            steering.lookahead_base = state.lookahead_base;
            steering.lookahead_speed_factor = state.lookahead_speed_factor;
            steering.wheelbase_m = state.wheelbase_m;
            steering.max_steer_angle = state.max_steer_angle;
            steering.kd_xte = state.kd_xte;
            steering.deadband_m = state.deadband_m;

            // Sync AB line if it changed
            if state.ab_line_dirty {
                guide.implement_width_m = state.implement_width_m;
                guide.overlap_m = state.overlap_m;
                if let Some(ref ab) = state.ab_line {
                    guide.load_ab_line(ab.a_lat, ab.a_lon, ab.b_lat, ab.b_lon);
                }
                guide.pass_number = state.pass_number;
                guide.pass_offset_m = guide.pass_spacing() * state.pass_number as f64;
                guide.nudge_m = state.nudge_m;
                state.ab_line_dirty = false;
            } else {
                // Always sync pass/nudge (these change without a full AB reload)
                if guide.pass_number != state.pass_number {
                    guide.pass_number = state.pass_number;
                    guide.pass_offset_m = guide.pass_spacing() * state.pass_number as f64;
                }
                guide.nudge_m = state.nudge_m;
                guide.implement_width_m = state.implement_width_m;
                guide.overlap_m = state.overlap_m;
            }
        }

        // ── 4. Interpolate position and compute steering ────────────
        if let Some(interp_fix) = interpolator.interpolate(None) {
            let interp_fix = interp_fix.clone();

            let error = guide.calculate_error(&interp_fix);

            if steering.engaged {
                if let Some(ref err) = error {
                    let (desired_angle, disengaged) = steering.compute(
                        err.distance_m,
                        err.heading_error,
                        interp_fix.speed,
                    );

                    if disengaged {
                        // Safety disengage — send zero immediately
                        let _ = motor_handle.send_steer_angle(0.0);
                        let reason = steering.disengage_reason.clone()
                            .unwrap_or_else(|| "Unknown".to_string());
                        tracing::warn!("Steer thread: safety disengage — {}", reason);

                        // Update shared state
                        let mut state = shared.lock().unwrap();
                        state.display.engaged = false;
                        state.display.desired_angle = 0.0;
                        state.display.disengage_reason = Some(reason);
                        state.display.just_disengaged = true;
                    } else {
                        // Send steer command to ESP32
                        let _ = motor_handle.send_steer_angle(desired_angle);

                        // Update shared state
                        let mut state = shared.lock().unwrap();
                        state.display.engaged = true;
                        state.display.desired_angle = desired_angle;
                        state.display.heading_error = steering.last_heading_error;
                        state.display.lookahead_m = steering.last_lookahead_m;
                        state.display.actual_angle = latest_motor_angle;
                        state.display.output_pwm = latest_motor_pwm;
                        state.display.just_disengaged = false;
                    }
                } else {
                    // No AB line or no error — send zero
                    let _ = motor_handle.send_steer_angle(0.0);
                    let mut state = shared.lock().unwrap();
                    state.display.desired_angle = 0.0;
                    state.display.engaged = steering.engaged;
                }
            } else {
                // Not engaged — just update display state
                let mut state = shared.lock().unwrap();
                state.display.engaged = false;
                state.display.actual_angle = latest_motor_angle;
                state.display.output_pwm = latest_motor_pwm;
            }
        }

        // ── 5. Sleep until next tick ────────────────────────────────
        let elapsed = loop_start.elapsed();
        if elapsed < STEER_LOOP_INTERVAL {
            std::thread::sleep(STEER_LOOP_INTERVAL - elapsed);
        }
    }
}
