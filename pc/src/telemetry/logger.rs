//! Steer telemetry logger — writes control loop data to `.jsonl` files.
//!
//! ## Usage
//!
//! Created by the steer thread. A new log file is opened on each engage
//! and closed on disengage (or dropped). The steer thread calls
//! `log_iteration()` on every loop tick while engaged.
//!
//! ## FINN Core integration
//!
//! The `.jsonl` files are self-contained — a FINN worker node can read
//! the 1Hz summary records to recommend tuning changes, or the full 10Hz
//! records for detailed analysis. A `run_id` links the log to the
//! guidance job/segment in SQLite for cross-referencing field conditions.
//!
//! ## Performance
//!
//! At 10Hz, a 10-minute run produces ~6,000 lines (~1.8 MB). Writes are
//! buffered (8 KB BufWriter) so individual `log_iteration()` calls don't
//! hit the filesystem. The buffer is flushed every second alongside the
//! summary record, and on close.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::time::Instant;
use chrono::Local;
use serde::Serialize;

// ─────────────────────────────────────────────────────────────────────
// Log record types — serialised as JSON, one per line
// ─────────────────────────────────────────────────────────────────────

/// Per-iteration record (10Hz). Full control loop snapshot.
#[derive(Debug, Serialize)]
pub struct IterRecord {
    /// Record type discriminator for JSON parsing.
    #[serde(rename = "type")]
    pub record_type: &'static str,

    // ── Timing ──
    /// Milliseconds since this steer run started.
    pub t_ms: u64,
    /// This loop iteration's wall-clock duration in microseconds.
    pub loop_us: u64,
    /// Age of the GPS fix used for this iteration (ms since last real fix).
    pub fix_age_ms: u64,

    // ── Position ──
    pub lat: f64,
    pub lon: f64,
    pub speed: f64,
    pub heading: f64,
    pub fix_quality: u8,
    pub sats: u8,
    pub hdop: f64,

    // ── Guidance ──
    pub pass: i32,
    pub xte_m: f64,
    pub heading_err: f64,

    // ── Control ──
    pub desired_angle: f64,
    pub actual_angle: f64,
    pub pwm: i16,
    pub lookahead_m: f64,
}

/// 1Hz summary record — aggregated over the last second of iterations.
#[derive(Debug, Serialize)]
pub struct SummaryRecord {
    #[serde(rename = "type")]
    pub record_type: &'static str,

    /// Milliseconds since run start.
    pub t_ms: u64,

    // ── XTE statistics ──
    pub mean_xte_m: f64,
    pub max_xte_abs_m: f64,
    pub min_xte_m: f64,
    pub max_xte_m: f64,

    // ── Control statistics ──
    /// Mean absolute angle error (desired − actual).
    pub mean_angle_err: f64,
    /// Number of times PWM changed sign in the last second (oscillation indicator).
    pub corrections: u32,
    /// Mean desired angle magnitude.
    pub mean_desired_angle: f64,

    // ── Timing statistics ──
    pub mean_loop_us: u64,
    pub max_loop_us: u64,
    /// Number of iterations in this summary window.
    pub iterations: u32,

    // ── Health ──
    /// GPS drops on gui channel since last summary.
    pub gps_drops_gui: u64,
    /// GPS drops on steer channel since last summary.
    pub gps_drops_steer: u64,
    /// Motor msg drops on gui channel since last summary.
    pub mtr_drops_gui: u64,
    /// Motor msg drops on steer channel since last summary.
    pub mtr_drops_steer: u64,
}

/// Discrete event record (engage, disengage, pass change, etc.).
#[derive(Debug, Serialize)]
pub struct EventRecord {
    #[serde(rename = "type")]
    pub record_type: &'static str,

    /// Milliseconds since run start.
    pub t_ms: u64,
    /// Event name: "engage", "disengage", "pass_change".
    pub event: String,
    /// Optional detail (e.g. disengage reason, new pass number).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detail: Option<String>,
}

/// Header record — written once at the start of each log file.
/// Captures the tuning parameters and system state at engage time.
#[derive(Debug, Serialize)]
pub struct HeaderRecord {
    #[serde(rename = "type")]
    pub record_type: &'static str,

    /// ISO 8601 timestamp of engage.
    pub started_at: String,
    /// Run ID for cross-referencing with coverage database.
    pub run_id: String,

    // ── Tuning snapshot ──
    pub lookahead_base: f64,
    pub lookahead_speed_factor: f64,
    pub wheelbase_m: f64,
    pub max_steer_angle: f64,
    pub kd_xte: f64,
    pub deadband_m: f64,
    pub implement_width_m: f64,
    pub overlap_m: f64,
}

// ─────────────────────────────────────────────────────────────────────
// Summary accumulator — collects per-iteration data for 1Hz rollup
// ─────────────────────────────────────────────────────────────────────

struct SummaryAccumulator {
    xte_sum: f64,
    xte_abs_max: f64,
    xte_min: f64,
    xte_max: f64,
    angle_err_sum: f64,
    desired_angle_sum: f64,
    loop_us_sum: u64,
    loop_us_max: u64,
    corrections: u32,
    last_pwm_sign: i8,
    iterations: u32,
}

impl SummaryAccumulator {
    fn new() -> Self {
        Self {
            xte_sum: 0.0,
            xte_abs_max: 0.0,
            xte_min: f64::MAX,
            xte_max: f64::MIN,
            angle_err_sum: 0.0,
            desired_angle_sum: 0.0,
            loop_us_sum: 0,
            loop_us_max: 0,
            corrections: 0,
            last_pwm_sign: 0,
            iterations: 0,
        }
    }

    fn accumulate(&mut self, record: &IterRecord) {
        self.iterations += 1;
        self.xte_sum += record.xte_m;
        let xte_abs = record.xte_m.abs();
        if xte_abs > self.xte_abs_max {
            self.xte_abs_max = xte_abs;
        }
        if record.xte_m < self.xte_min {
            self.xte_min = record.xte_m;
        }
        if record.xte_m > self.xte_max {
            self.xte_max = record.xte_m;
        }

        self.angle_err_sum += (record.desired_angle - record.actual_angle).abs();
        self.desired_angle_sum += record.desired_angle.abs();

        self.loop_us_sum += record.loop_us;
        if record.loop_us > self.loop_us_max {
            self.loop_us_max = record.loop_us;
        }

        // Count PWM sign changes (direction reversals = oscillation).
        let sign = if record.pwm > 0 {
            1
        } else if record.pwm < 0 {
            -1
        } else {
            0
        };
        if sign != 0 && self.last_pwm_sign != 0 && sign != self.last_pwm_sign {
            self.corrections += 1;
        }
        if sign != 0 {
            self.last_pwm_sign = sign;
        }
    }

    fn to_summary(&self, t_ms: u64, drop_counts: &DropCounts) -> SummaryRecord {
        let n = self.iterations.max(1) as f64;
        SummaryRecord {
            record_type: "summary",
            t_ms,
            mean_xte_m: self.xte_sum / n,
            max_xte_abs_m: self.xte_abs_max,
            min_xte_m: if self.xte_min == f64::MAX { 0.0 } else { self.xte_min },
            max_xte_m: if self.xte_max == f64::MIN { 0.0 } else { self.xte_max },
            mean_angle_err: self.angle_err_sum / n,
            corrections: self.corrections,
            mean_desired_angle: self.desired_angle_sum / n,
            mean_loop_us: if self.iterations > 0 {
                self.loop_us_sum / self.iterations as u64
            } else {
                0
            },
            max_loop_us: self.loop_us_max,
            iterations: self.iterations,
            gps_drops_gui: drop_counts.gps_gui,
            gps_drops_steer: drop_counts.gps_steer,
            mtr_drops_gui: drop_counts.mtr_gui,
            mtr_drops_steer: drop_counts.mtr_steer,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
        // Preserve last_pwm_sign across windows so the first iteration
        // of the next window can still detect a sign change.
    }
}

// ─────────────────────────────────────────────────────────────────────
// Drop counts — passed in from shared atomics
// ─────────────────────────────────────────────────────────────────────

/// Snapshot of channel drop counts for a summary window.
/// The steer thread reads-and-resets the atomics each second and passes
/// the snapshot here.
pub struct DropCounts {
    pub gps_gui: u64,
    pub gps_steer: u64,
    pub mtr_gui: u64,
    pub mtr_steer: u64,
}

impl Default for DropCounts {
    fn default() -> Self {
        Self {
            gps_gui: 0,
            gps_steer: 0,
            mtr_gui: 0,
            mtr_steer: 0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// TelemetryLogger — public API
// ─────────────────────────────────────────────────────────────────────

/// Writes steer telemetry to a `.jsonl` file during an auto-steer run.
///
/// Create one when auto-steer engages; drop it on disengage. The steer
/// thread calls `log_iteration()` on every 10Hz tick while engaged.
pub struct TelemetryLogger {
    writer: BufWriter<File>,
    run_start: Instant,
    run_id: String,
    accumulator: SummaryAccumulator,
    last_summary_time: Instant,
    iteration_count: u64,
    log_path: PathBuf,
}

impl TelemetryLogger {
    /// Create a new telemetry logger and write the header record.
    ///
    /// The log file is created in `<app_dir>/logs/` with a timestamped name.
    /// Returns `None` if the log directory can't be created or the file can't
    /// be opened (non-fatal — steering continues without telemetry).
    pub fn new(tuning: &TuningSnapshot) -> Option<Self> {
        let now = Local::now();
        let run_id = now.format("%Y%m%d_%H%M%S").to_string();
        let filename = format!("steer_{}.jsonl", run_id);

        // Create logs/ directory next to the executable
        let log_dir = Self::log_directory();
        if let Err(e) = fs::create_dir_all(&log_dir) {
            tracing::error!("Failed to create telemetry log directory {:?}: {}", log_dir, e);
            return None;
        }

        let log_path = log_dir.join(&filename);
        let file = match File::create(&log_path) {
            Ok(f) => f,
            Err(e) => {
                tracing::error!("Failed to create telemetry log {:?}: {}", log_path, e);
                return None;
            }
        };

        let mut writer = BufWriter::with_capacity(8192, file);

        // Write header record
        let header = HeaderRecord {
            record_type: "header",
            started_at: now.to_rfc3339(),
            run_id: run_id.clone(),
            lookahead_base: tuning.lookahead_base,
            lookahead_speed_factor: tuning.lookahead_speed_factor,
            wheelbase_m: tuning.wheelbase_m,
            max_steer_angle: tuning.max_steer_angle,
            kd_xte: tuning.kd_xte,
            deadband_m: tuning.deadband_m,
            implement_width_m: tuning.implement_width_m,
            overlap_m: tuning.overlap_m,
        };

        if let Ok(json) = serde_json::to_string(&header) {
            let _ = writeln!(writer, "{}", json);
        }

        let start = Instant::now();
        tracing::info!("Telemetry logging started: {:?}", log_path);

        Some(Self {
            writer,
            run_start: start,
            run_id,
            accumulator: SummaryAccumulator::new(),
            last_summary_time: start,
            iteration_count: 0,
            log_path,
        })
    }

    /// Log one steer loop iteration. Called at 10Hz by the steer thread.
    ///
    /// Also triggers a 1Hz summary when ≥1 second has elapsed since the
    /// last summary. Returns `true` if a summary was emitted this call
    /// (so the caller can reset its drop count accumulator).
    pub fn log_iteration(&mut self, record: &IterRecord, drop_counts: Option<&DropCounts>) -> bool {
        self.iteration_count += 1;

        // Write the iteration record
        if let Ok(json) = serde_json::to_string(record) {
            let _ = writeln!(self.writer, "{}", json);
        }

        // Accumulate for summary
        self.accumulator.accumulate(record);

        // Emit 1Hz summary
        if self.last_summary_time.elapsed().as_secs_f64() >= 1.0 {
            let drops = drop_counts
                .map(|d| DropCounts {
                    gps_gui: d.gps_gui,
                    gps_steer: d.gps_steer,
                    mtr_gui: d.mtr_gui,
                    mtr_steer: d.mtr_steer,
                })
                .unwrap_or_default();

            let summary = self.accumulator.to_summary(record.t_ms, &drops);

            if let Ok(json) = serde_json::to_string(&summary) {
                let _ = writeln!(self.writer, "{}", json);
            }

            // Flush to disk once per second
            let _ = self.writer.flush();

            self.accumulator.reset();
            self.last_summary_time = Instant::now();
            return true;
        }

        false
    }

    /// Log a discrete event (engage, disengage, pass change, etc.).
    pub fn log_event(&mut self, event: &str, detail: Option<String>) {
        let record = EventRecord {
            record_type: "event",
            t_ms: self.run_start.elapsed().as_millis() as u64,
            event: event.to_string(),
            detail,
        };

        if let Ok(json) = serde_json::to_string(&record) {
            let _ = writeln!(self.writer, "{}", json);
            let _ = self.writer.flush();
        }
    }

    /// Get the run ID for this telemetry session.
    pub fn run_id(&self) -> &str {
        &self.run_id
    }

    /// Get the elapsed time since the run started.
    pub fn elapsed(&self) -> std::time::Duration {
        self.run_start.elapsed()
    }

    /// Get the total number of iterations logged.
    pub fn iteration_count(&self) -> u64 {
        self.iteration_count
    }

    /// Resolve the log directory path.
    fn log_directory() -> PathBuf {
        // Put logs next to the executable, or fall back to current dir
        if let Ok(exe) = std::env::current_exe() {
            if let Some(dir) = exe.parent() {
                return dir.join("logs");
            }
        }
        PathBuf::from("logs")
    }
}

impl Drop for TelemetryLogger {
    fn drop(&mut self) {
        // Write a final disengage event if we haven't already
        self.log_event("log_close", Some(format!(
            "iterations={} elapsed_s={:.1}",
            self.iteration_count,
            self.run_start.elapsed().as_secs_f64(),
        )));

        let _ = self.writer.flush();
        tracing::info!(
            "Telemetry log closed: {:?} ({} iterations, {:.1}s)",
            self.log_path,
            self.iteration_count,
            self.run_start.elapsed().as_secs_f64(),
        );
    }
}

// ─────────────────────────────────────────────────────────────────────
// Tuning snapshot — passed in at logger creation
// ─────────────────────────────────────────────────────────────────────

/// Snapshot of tuning parameters at the moment auto-steer is engaged.
/// Used to write the header record so analysis knows what config was active.
pub struct TuningSnapshot {
    pub lookahead_base: f64,
    pub lookahead_speed_factor: f64,
    pub wheelbase_m: f64,
    pub max_steer_angle: f64,
    pub kd_xte: f64,
    pub deadband_m: f64,
    pub implement_width_m: f64,
    pub overlap_m: f64,
}

// ─────────────────────────────────────────────────────────────────────
// Helper: map FixQuality enum to u8 for compact JSON
// ─────────────────────────────────────────────────────────────────────

/// Convert FixQuality to a numeric code for the log.
///   0=NoFix, 1=GPS, 2=DGPS, 3=RTK, 4=RtkFloat
pub fn fix_quality_to_u8(q: finn_guidance_common::types::FixQuality) -> u8 {
    use finn_guidance_common::types::FixQuality;
    match q {
        FixQuality::NoFix => 0,
        FixQuality::Gps => 1,
        FixQuality::Dgps => 2,
        FixQuality::Rtk => 3,
        FixQuality::RtkFloat => 4,
    }
}
