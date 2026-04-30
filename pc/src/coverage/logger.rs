//! Coverage logger - records GPS positions while the implement is engaged.
//!
//! Writes coverage data to SQLite via the CoverageDb. Points are buffered
//! in memory and flushed in batches for good write performance.
//!
//! ## Deduplication & Filtering
//!
//! The GPS receiver (LC29H) updates at a fixed rate (typically 1Hz),
//! but the GUI loop runs much faster (~200Hz). Without filtering, we'd log
//! the same GPS fix many times per actual position update.
//!
//! Three filters prevent redundant data:
//!   1. **Epoch dedup**: Only logs when `timestamp_ms` changes from last logged point
//!   2. **Distance filter**: Skip if moved less than `min_distance_m` since last log
//!   3. **Time filter**: Skip if less than `min_interval_ms` since last log
//!
//! ## Data flow
//!
//! GPS fix → filter → write buffer → batch insert to SQLite (every 50 points)
//!                                  → append to render cache (for field_view)

use crate::coverage::db::{CoverageDb, CoveragePointRow};
use finn_guidance_common::coords;
use finn_guidance_common::types::{FixQuality, GpsFix};

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
            // Default: 0.25m — gives smooth, gap-free coverage strips at working speed.
            // At 10km/h (~2.78m/s) and 1Hz GPS, that's ~11 points per fix (if GPS were
            // faster) but in practice at 1Hz we get 1 point per fix when moving > 0.25m/s.
            min_distance_m: 0.25,
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

/// A single coverage point for rendering on the field view canvas.
/// Kept in memory as a render cache, sourced from SQLite.
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

impl CoveragePoint {
    /// Convert from a database row to a render point.
    pub fn from_row(row: &CoveragePointRow) -> Self {
        Self {
            segment: row.segment,
            latitude: row.latitude,
            longitude: row.longitude,
            altitude: row.altitude,
            speed: row.speed,
            heading: row.heading,
            fix_quality: row.fix_quality,
            satellites: row.satellites,
            hdop: row.hdop,
            timestamp_ms: row.timestamp_ms,
        }
    }
}

/// How many points to buffer before flushing to SQLite.
const WRITE_BUFFER_SIZE: usize = 50;

/// Coverage logger state.
pub struct CoverageLogger {
    /// Whether the implement is currently engaged (logging active)
    engaged: bool,
    /// Current segment number (increments each engage/disengage cycle)
    current_segment: u32,
    /// Total points logged in the current job
    points_logged: u64,
    /// Points logged in the current segment
    segment_points: u64,
    /// In-memory coverage points for rendering on the canvas.
    /// Grows as points are logged — no downsample needed since SQLite
    /// is the source of truth and we can reload from DB if needed.
    render_cache: Vec<CoveragePoint>,
    /// Implement width in metres (for coverage strip rendering)
    implement_width_m: f64,
    /// Log filter configuration
    filter: LogFilter,
    /// Write buffer — accumulates points before batch-inserting to SQLite
    write_buffer: Vec<CoveragePointRow>,
    /// Current job ID in the database (None if no job started)
    current_job_id: Option<i64>,

    // === Deduplication state ===
    /// Timestamp of the last point we actually logged
    last_logged_timestamp_ms: u64,
    /// Position of the last point we actually logged
    last_logged_lat: f64,
    last_logged_lon: f64,
}

impl CoverageLogger {
    pub fn new(implement_width: f64) -> Self {
        Self {
            engaged: false,
            current_segment: 0,
            points_logged: 0,
            segment_points: 0,
            render_cache: Vec::new(),
            implement_width_m: implement_width,
            filter: LogFilter::default(),
            write_buffer: Vec::with_capacity(WRITE_BUFFER_SIZE),
            current_job_id: None,
            last_logged_timestamp_ms: 0,
            last_logged_lat: 0.0,
            last_logged_lon: 0.0,
        }
    }

    /// Returns whether the implement is currently engaged.
    pub fn is_engaged(&self) -> bool {
        self.engaged
    }

    /// Get the current job ID (if a job is active).
    pub fn current_job_id(&self) -> Option<i64> {
        self.current_job_id
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
    /// Requires a database reference to create jobs/flush data.
    pub fn toggle_engage(&mut self, db: Option<&CoverageDb>) -> bool {
        if self.engaged {
            self.disengage(db);
        } else {
            self.engage(db);
        }
        self.engaged
    }

    /// Engage the implement - start logging.
    fn engage(&mut self, db: Option<&CoverageDb>) {
        self.engaged = true;
        self.current_segment += 1;
        self.segment_points = 0;

        // Reset dedup state so the first point of a new segment always logs
        self.last_logged_timestamp_ms = 0;

        // Create a new job in the database if we don't have one yet
        if self.current_job_id.is_none() {
            self.start_new_job(db);
        }

        tracing::info!(
            "Coverage ENGAGED - segment {}, job_id: {:?}",
            self.current_segment,
            self.current_job_id
        );
    }

    /// Disengage the implement - stop logging (but keep job open for next engage).
    fn disengage(&mut self, db: Option<&CoverageDb>) {
        self.engaged = false;
        // Flush any remaining buffered points
        self.flush_buffer(db);
        tracing::info!(
            "Coverage DISENGAGED - segment {} had {} points, {} total",
            self.current_segment,
            self.segment_points,
            self.points_logged,
        );
    }

    /// Start a new job in the database.
    fn start_new_job(&mut self, db: Option<&CoverageDb>) {
        let now = chrono::Local::now();
        let job_name = format!("Job_{}", now.format("%Y-%m-%d_%H%M%S"));

        if let Some(db) = db {
            match db.create_job(&job_name, self.implement_width_m) {
                Ok(job_id) => {
                    self.current_job_id = Some(job_id);
                    self.points_logged = 0;
                    self.current_segment = 0;
                    self.render_cache.clear();
                    tracing::info!("Started new coverage job: {} (id={})", job_name, job_id);
                }
                Err(e) => {
                    tracing::error!("Failed to create coverage job: {}", e);
                }
            }
        } else {
            tracing::warn!("No database available — coverage points will only be in memory");
            self.points_logged = 0;
            self.current_segment = 0;
            self.render_cache.clear();
        }
    }

    /// Log a GPS fix (only records if engaged AND the fix passes filters).
    ///
    /// Call this on every GPS fix received. The logger handles deduplication
    /// internally — it's safe and expected to call this at the GUI refresh rate.
    pub fn log_fix(&mut self, fix: &GpsFix, db: Option<&CoverageDb>) {
        if !self.engaged {
            return;
        }

        // === Gate 1: Epoch deduplication ===
        if fix.timestamp_ms == self.last_logged_timestamp_ms && self.last_logged_timestamp_ms != 0 {
            return;
        }

        // === Gate 2: Time filter ===
        if self.filter.min_interval_ms > 0 && self.last_logged_timestamp_ms > 0 {
            let elapsed = fix
                .timestamp_ms
                .saturating_sub(self.last_logged_timestamp_ms);
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

        let row = CoveragePointRow {
            segment: self.current_segment,
            timestamp_ms: fix.timestamp_ms,
            latitude: fix.latitude,
            longitude: fix.longitude,
            altitude: fix.altitude,
            speed: fix.speed,
            heading: fix.heading,
            fix_quality: fix.fix_quality,
            satellites: fix.satellites,
            hdop: fix.hdop,
        };

        // Add to render cache for immediate display
        self.render_cache.push(CoveragePoint::from_row(&row));

        // Add to write buffer
        self.write_buffer.push(row);

        // Flush to SQLite when buffer is full
        if self.write_buffer.len() >= WRITE_BUFFER_SIZE {
            self.flush_buffer(db);
        }

        // Update dedup state
        self.last_logged_timestamp_ms = fix.timestamp_ms;
        self.last_logged_lat = fix.latitude;
        self.last_logged_lon = fix.longitude;

        self.points_logged += 1;
        self.segment_points += 1;
    }

    /// Flush the write buffer to SQLite.
    fn flush_buffer(&mut self, db: Option<&CoverageDb>) {
        if self.write_buffer.is_empty() {
            return;
        }
        if let (Some(db), Some(job_id)) = (db, self.current_job_id) {
            if let Err(e) = db.insert_coverage_batch(job_id, &self.write_buffer) {
                tracing::error!("Failed to flush coverage buffer: {}", e);
                return; // Keep buffer for retry
            }
        }
        self.write_buffer.clear();
    }

    /// End the current job (flush, update stats in DB, reset state).
    pub fn end_job(&mut self, db: Option<&CoverageDb>) {
        if self.engaged {
            self.disengage(db);
        }
        // Update job stats in database
        if let (Some(db), Some(job_id)) = (db, self.current_job_id) {
            let _ = db.end_job(job_id, self.points_logged, self.current_segment);
        }
        self.current_job_id = None;
        tracing::info!("Coverage job ended - {} points logged", self.points_logged,);
    }

    /// Get coverage points for rendering.
    pub fn points(&self) -> &[CoveragePoint] {
        &self.render_cache
    }

    /// Clear the render cache and end the current job.
    /// Use this when moving to a new task (e.g. switching from seeding to spraying).
    /// The SQLite data is untouched — this only clears the in-memory display
    /// and ends the job so the next engage starts fresh.
    pub fn clear_coverage(&mut self, db: Option<&CoverageDb>) {
        if self.engaged {
            self.disengage(db);
        }
        // End the current job in the database
        if let (Some(db), Some(job_id)) = (db, self.current_job_id) {
            let _ = db.end_job(job_id, self.points_logged, self.current_segment);
        }
        self.current_job_id = None;
        self.render_cache.clear();
        self.points_logged = 0;
        self.current_segment = 0;
        self.segment_points = 0;
        self.last_logged_timestamp_ms = 0;
        tracing::info!("Coverage display cleared — SQLite data is untouched");
    }

    /// Load coverage points from a previous job into the render cache.
    pub fn load_job_coverage(&mut self, db: &CoverageDb, job_id: i64) {
        match db.load_coverage_points(job_id) {
            Ok(rows) => {
                self.render_cache = rows.iter().map(CoveragePoint::from_row).collect();
                tracing::info!(
                    "Loaded {} coverage points from job {}",
                    self.render_cache.len(),
                    job_id
                );
            }
            Err(e) => {
                tracing::error!("Failed to load coverage for job {}: {}", job_id, e);
            }
        }
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

    /// Estimate covered area in hectares.
    ///
    /// Sums the haversine distance between consecutive points within each
    /// segment, multiplies by implement width, and converts to hectares.
    /// Points from different segments are not connected (headland turns).
    pub fn covered_hectares(&self) -> f64 {
        if self.render_cache.len() < 2 {
            return 0.0;
        }

        let mut total_area_m2 = 0.0;

        for i in 1..self.render_cache.len() {
            let prev = &self.render_cache[i - 1];
            let curr = &self.render_cache[i];

            // Only sum within the same segment (skip headland gaps)
            if curr.segment != prev.segment {
                continue;
            }

            let dist = coords::haversine_distance(
                prev.latitude,
                prev.longitude,
                curr.latitude,
                curr.longitude,
            );
            total_area_m2 += dist * self.implement_width_m;
        }

        total_area_m2 / 10_000.0
    }

    /// Update implement width.
    pub fn set_implement_width(&mut self, width: f64) {
        self.implement_width_m = width;
    }
}
