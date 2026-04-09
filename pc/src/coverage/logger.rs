//! Coverage logger - records GPS positions while the implement is engaged.
//!
//! Writes coverage data to CSV files in a configurable directory.
//! Each job gets its own CSV file, and each engage/disengage cycle is a segment.
//!
//! ## Deduplication & Filtering
//!
//! The GPS receiver (LC29H) updates at a fixed rate (typically 1-10Hz),
//! but the GUI loop runs much faster (~200Hz). Without filtering, we'd log
//! the same GPS fix 20-200 times per actual position update.
//!
//! Three filters prevent redundant data:
//!   1. **Epoch dedup**: Only logs when `timestamp_ms` changes from last logged point
//!   2. **Distance filter**: Skip if moved less than `min_distance_m` since last log
//!   3. **Time filter**: Skip if less than `min_interval_ms` since last log
//!
//! The distance and time filters are AND-combined: a point must satisfy BOTH
//! to be logged (after passing the epoch dedup gate).
//!
//! ## CSV format
//!
//! ```text
//! segment,timestamp_ms,latitude,longitude,altitude,speed,heading,fix_quality,satellites,hdop
//! ```
//!
//! Files are named by date/time: `coverage_2026-03-26_120000.csv`

use std::fs::{self, File};
use std::io::Write;
use std::path::{Path, PathBuf};
use finn_guidance_common::types::{GpsFix, FixQuality};
use finn_guidance_common::coords;

/// Configuration for coverage log filtering.
#[derive(Debug, Clone)]
pub struct LogFilter {
    /// Minimum distance (metres) between logged points. 0.0 = log every new epoch.
    pub min_distance_m: f64,
    /// Minimum time (milliseconds) between logged points. 0 = log every new epoch.
    pub min_interval_ms: u64,
}

impl Default for LogFilter {
    fn default() -> Self {
        Self {
            // Default: distance-based at 1m — only logs when the machine has moved.
            // At 5Hz GPS, this avoids flooding the CSV when stationary and keeps
            // ~1 point per metre at working speed (~3600/hr at 10km/h).
            min_distance_m: 1.0,
            min_interval_ms: 0,
        }
    }
}

impl LogFilter {
    /// Preset: log every unique GPS fix (1Hz = ~3600/hr)
    pub fn every_fix() -> Self {
        Self {
            min_distance_m: 0.0,
            min_interval_ms: 0,
        }
    }

    /// Preset: distance-based, good for coverage mapping at working speed
    pub fn distance_based(metres: f64) -> Self {
        Self {
            min_distance_m: metres,
            min_interval_ms: 0,
        }
    }

    /// Preset: time-based at a fixed interval
    pub fn time_based(interval_ms: u64) -> Self {
        Self {
            min_distance_m: 0.0,
            min_interval_ms: interval_ms,
        }
    }
}

/// A single logged coverage point (kept in memory for rendering).
#[derive(Debug, Clone)]
pub struct CoveragePoint {
    pub segment: u32,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub speed: f64,
    pub heading: f64,
    pub fix_quality: FixQuality,
    pub satellites: u8,
    pub hdop: f64,
    pub timestamp_ms: u64,
}

/// Coverage logger state.
pub struct CoverageLogger {
    /// Directory to store coverage CSV files
    log_dir: PathBuf,
    /// Whether the implement is currently engaged (logging active)
    engaged: bool,
    /// Current segment number (increments each engage/disengage cycle)
    current_segment: u32,
    /// Active file handle for writing
    active_file: Option<File>,
    /// Path to the active file (for display / database)
    active_file_path: Option<PathBuf>,
    /// Total points logged in the current job
    points_logged: u64,
    /// Points logged in the current segment
    segment_points: u64,
    /// In-memory coverage points for rendering on the canvas
    coverage_points: Vec<CoveragePoint>,
    /// Implement width in metres (for coverage strip rendering)
    implement_width_m: f64,
    /// Log filter configuration
    filter: LogFilter,
    /// Maximum number of in-memory coverage points before downsampling.
    /// CSV recording is unaffected — this only limits display memory.
    max_display_points: usize,

    // === Deduplication state ===
    /// Timestamp of the last point we actually logged
    last_logged_timestamp_ms: u64,
    /// Position of the last point we actually logged
    last_logged_lat: f64,
    last_logged_lon: f64,
}

impl CoverageLogger {
    pub fn new(log_dir: impl Into<PathBuf>, implement_width: f64) -> Self {
        let log_dir = log_dir.into();
        let _ = fs::create_dir_all(&log_dir);

        Self {
            log_dir,
            engaged: false,
            current_segment: 0,
            active_file: None,
            active_file_path: None,
            points_logged: 0,
            segment_points: 0,
            coverage_points: Vec::new(),
            implement_width_m: implement_width,
            filter: LogFilter::default(),
            max_display_points: 100_000,
            last_logged_timestamp_ms: 0,
            last_logged_lat: 0.0,
            last_logged_lon: 0.0,
        }
    }

    /// Returns whether the implement is currently engaged.
    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Get the active CSV filename (just the name, not full path).
    pub fn active_filename(&self) -> Option<String> {
        self.active_file_path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().to_string())
    }

    /// Set the log filter configuration.
    pub fn set_filter(&mut self, filter: LogFilter) {
        self.filter = filter;
        tracing::info!(
            "Coverage filter set: min_distance={}m, min_interval={}ms",
            self.filter.min_distance_m,
            self.filter.min_interval_ms,
        );
    }

    /// Get the current filter configuration.
    pub fn filter(&self) -> &LogFilter {
        &self.filter
    }

    /// Toggle engage/disengage. Returns the new engaged state.
    pub fn toggle_engage(&mut self) -> bool {
        if self.engaged {
            self.disengage();
        } else {
            self.engage();
        }
        self.engaged
    }

    /// Engage the implement - start logging.
    fn engage(&mut self) {
        self.engaged = true;
        self.current_segment += 1;
        self.segment_points = 0;

        // Reset dedup state so the first point of a new segment always logs
        self.last_logged_timestamp_ms = 0;

        // Create a new CSV file if we don't have one yet for this job
        if self.active_file.is_none() {
            self.start_new_job();
        }

        tracing::info!(
            "Coverage ENGAGED - segment {}, file: {:?}",
            self.current_segment,
            self.active_file_path
        );
    }

    /// Disengage the implement - stop logging (but keep file open for next engage).
    fn disengage(&mut self) {
        self.engaged = false;
        if let Some(ref mut file) = self.active_file {
            let _ = file.flush();
        }
        tracing::info!(
            "Coverage DISENGAGED - segment {} had {} points, {} total",
            self.current_segment,
            self.segment_points,
            self.points_logged,
        );
    }

    /// Start a new job (new CSV file).
    fn start_new_job(&mut self) {
        let now = chrono::Local::now();
        let filename = format!("coverage_{}.csv", now.format("%Y-%m-%d_%H%M%S"));
        let path = self.log_dir.join(&filename);

        match File::create(&path) {
            Ok(mut file) => {
                // Write CSV header
                let _ = writeln!(
                    file,
                    "segment,timestamp_ms,latitude,longitude,altitude,speed,heading,fix_quality,satellites,hdop"
                );
                self.active_file = Some(file);
                self.active_file_path = Some(path.clone());
                self.points_logged = 0;
                self.current_segment = 0;
                self.coverage_points.clear();
                tracing::info!("Started new coverage job: {}", filename);
            }
            Err(e) => {
                tracing::error!("Failed to create coverage file {}: {}", path.display(), e);
            }
        }
    }

    /// Log a GPS fix (only records if engaged AND the fix passes filters).
    ///
    /// Call this on every GPS fix received. The logger handles deduplication
    /// internally — it's safe and expected to call this at the GUI refresh rate.
    pub fn log_fix(&mut self, fix: &GpsFix) {
        if !self.engaged {
            return;
        }

        // === Gate 1: Epoch deduplication ===
        // Only proceed if this is a genuinely new GPS fix (different timestamp).
        // The ZED-F9P updates once per epoch; the GUI loop calls us many times
        // per epoch with the same fix data.
        if fix.timestamp_ms == self.last_logged_timestamp_ms && self.last_logged_timestamp_ms != 0 {
            return;
        }

        // === Gate 2: Time filter ===
        if self.filter.min_interval_ms > 0 && self.last_logged_timestamp_ms > 0 {
            let elapsed = fix.timestamp_ms.saturating_sub(self.last_logged_timestamp_ms);
            if elapsed < self.filter.min_interval_ms {
                return;
            }
        }

        // === Gate 3: Distance filter ===
        if self.filter.min_distance_m > 0.0 && self.last_logged_timestamp_ms > 0 {
            let dist = coords::haversine_distance(
                self.last_logged_lat,
                self.last_logged_lon,
                fix.latitude,
                fix.longitude,
            );
            if dist < self.filter.min_distance_m {
                return;
            }
        }

        // === Passed all filters — log it ===

        let quality_str = match fix.fix_quality {
            FixQuality::NoFix => "NoFix",
            FixQuality::Gps => "GPS",
            FixQuality::Dgps => "DGPS",
            FixQuality::Rtk => "RTK",
            FixQuality::RtkFloat => "RtkFloat",
        };

        // Write to CSV
        if let Some(ref mut file) = self.active_file {
            let _ = writeln!(
                file,
                "{},{},{:.8},{:.8},{:.2},{:.3},{:.2},{},{},{}",
                self.current_segment,
                fix.timestamp_ms,
                fix.latitude,
                fix.longitude,
                fix.altitude,
                fix.speed,
                fix.heading,
                quality_str,
                fix.satellites,
                fix.hdop,
            );
        }

        // Store in memory for rendering
        self.coverage_points.push(CoveragePoint {
            segment: self.current_segment,
            latitude: fix.latitude,
            longitude: fix.longitude,
            altitude: fix.altitude,
            speed: fix.speed,
            heading: fix.heading,
            fix_quality: fix.fix_quality,
            satellites: fix.satellites,
            hdop: fix.hdop,
            timestamp_ms: fix.timestamp_ms,
        });

        // Memory cap: if we exceed the limit, downsample the oldest half by
        // dropping every second point. Preserves spatial coverage while halving
        // memory. The CSV still has full fidelity.
        if self.coverage_points.len() > self.max_display_points {
            let half = self.coverage_points.len() / 2;
            // Keep every 2nd point from the older half, keep all of the newer half
            let mut thinned: Vec<CoveragePoint> = self.coverage_points[..half]
                .iter()
                .step_by(2)
                .cloned()
                .collect();
            thinned.extend_from_slice(&self.coverage_points[half..]);
            self.coverage_points = thinned;
            tracing::info!(
                "Coverage display thinned: {} points after downsample",
                self.coverage_points.len()
            );
        }

        // Update dedup state
        self.last_logged_timestamp_ms = fix.timestamp_ms;
        self.last_logged_lat = fix.latitude;
        self.last_logged_lon = fix.longitude;

        self.points_logged += 1;
        self.segment_points += 1;
    }

    /// End the current job (close file, reset state).
    pub fn end_job(&mut self) {
        if self.engaged {
            self.disengage();
        }
        if let Some(ref mut file) = self.active_file {
            let _ = file.flush();
        }
        self.active_file = None;
        tracing::info!(
            "Coverage job ended - {} points logged to {:?}",
            self.points_logged,
            self.active_file_path
        );
    }

    /// Get coverage points for rendering.
    pub fn points(&self) -> &[CoveragePoint] {
        &self.coverage_points
    }

    /// Clear all in-memory coverage points.
    ///
    /// Use this when moving to a new task (e.g. switching from seeding to spraying).
    /// The CSV files on disk are untouched — this only clears the display.
    /// Also ends the current job so the next engage starts a fresh CSV file.
    pub fn clear_coverage(&mut self) {
        if self.engaged {
            self.disengage();
        }
        if let Some(ref mut file) = self.active_file {
            let _ = file.flush();
        }
        self.active_file = None;
        self.active_file_path = None;
        self.coverage_points.clear();
        self.points_logged = 0;
        self.current_segment = 0;
        self.segment_points = 0;
        self.last_logged_timestamp_ms = 0;
        tracing::info!("Coverage display cleared — CSV files on disk are untouched");
    }

    /// Get the total number of points logged this job.
    pub fn total_points(&self) -> u64 {
        self.points_logged
    }

    /// Get the current segment number.
    pub fn segment(&self) -> u32 {
        self.current_segment
    }

    /// Get the implement width.
    pub fn implement_width(&self) -> f64 {
        self.implement_width_m
    }

    /// Update implement width.
    pub fn set_implement_width(&mut self, width: f64) {
        self.implement_width_m = width;
    }

    /// Load a previous coverage CSV file back into memory for display.
    pub fn load_from_file(path: &Path) -> Option<Vec<CoveragePoint>> {
        let content = fs::read_to_string(path).ok()?;
        let mut points = Vec::new();

        for line in content.lines().skip(1) {
            let fields: Vec<&str> = line.split(',').collect();
            if fields.len() < 10 {
                continue;
            }

            let fix_quality = match fields[7] {
                "RTK" => FixQuality::Rtk,
                "RtkFloat" => FixQuality::RtkFloat,
                "DGPS" => FixQuality::Dgps,
                "GPS" => FixQuality::Gps,
                _ => FixQuality::NoFix,
            };

            if let (Ok(seg), Ok(ts), Ok(lat), Ok(lon), Ok(alt), Ok(spd), Ok(hdg), Ok(sats), Ok(hdop)) = (
                fields[0].parse::<u32>(),
                fields[1].parse::<u64>(),
                fields[2].parse::<f64>(),
                fields[3].parse::<f64>(),
                fields[4].parse::<f64>(),
                fields[5].parse::<f64>(),
                fields[6].parse::<f64>(),
                fields[8].parse::<u8>(),
                fields[9].parse::<f64>(),
            ) {
                points.push(CoveragePoint {
                    segment: seg,
                    latitude: lat,
                    longitude: lon,
                    altitude: alt,
                    speed: spd,
                    heading: hdg,
                    fix_quality,
                    satellites: sats,
                    hdop,
                    timestamp_ms: ts,
                });
            }
        }

        Some(points)
    }
}
