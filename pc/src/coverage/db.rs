//! Coverage database - SQLite storage for coverage points, job metadata, AB lines, and fields.
//!
//! The database stores all coverage data:
//!   - Coverage points: GPS positions recorded while the implement is engaged
//!   - Fields: named paddocks, each containing a set of AB lines
//!   - Jobs: each recording session with start/end times, total area
//!   - Segments: each engage/disengage cycle within a job
//!   - AB Lines: saved guidance lines grouped by field, for reuse across sessions
//!   - Config: persisted settings (implement width, log filters, etc.)
//!
//! Coverage points are the primary data store (replaces CSV files). Points are
//! written in batches via transactions for performance. At 0.25m spacing and
//! 10km/h, that's ~11k points/hour — SQLite handles this easily.
//!
//! AB line organisation:
//!   Fields group related AB lines (e.g. a paddock typically has up to 4 lines:
//!   N/S header, E/W header, and diagonals). This grouping makes it easy to find
//!   the right line at the start of a run, especially when transferring between PCs.

use finn_guidance_common::types::FixQuality;
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Coverage database backed by SQLite.
pub struct CoverageDb {
    conn: rusqlite::Connection,
}

impl CoverageDb {
    /// Open or create the coverage database at the given path.
    pub fn open(db_path: &Path) -> Result<Self, String> {
        // Ensure parent directory exists
        if let Some(parent) = db_path.parent() {
            let _ = std::fs::create_dir_all(parent);
        }

        let conn = rusqlite::Connection::open(db_path)
            .map_err(|e| format!("Failed to open coverage database: {}", e))?;

        let db = Self { conn };
        db.create_tables()?;
        Ok(db)
    }

    /// Create database tables if they don't exist.
    fn create_tables(&self) -> Result<(), String> {
        self.conn
            .execute_batch(
                "
            CREATE TABLE IF NOT EXISTS fields (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                notes TEXT,
                created_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
            );

            CREATE TABLE IF NOT EXISTS jobs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL DEFAULT '',
                started_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime')),
                ended_at TEXT,
                implement_width_m REAL NOT NULL,
                total_points INTEGER NOT NULL DEFAULT 0,
                total_segments INTEGER NOT NULL DEFAULT 0,
                notes TEXT
            );

            CREATE TABLE IF NOT EXISTS segments (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                job_id INTEGER NOT NULL REFERENCES jobs(id),
                segment_number INTEGER NOT NULL,
                started_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime')),
                ended_at TEXT,
                point_count INTEGER NOT NULL DEFAULT 0,
                ab_line_id INTEGER REFERENCES ab_lines(id)
            );

            CREATE TABLE IF NOT EXISTS coverage_points (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                job_id INTEGER NOT NULL REFERENCES jobs(id) ON DELETE CASCADE,
                segment INTEGER NOT NULL,
                timestamp_ms INTEGER NOT NULL,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                altitude REAL NOT NULL,
                speed REAL NOT NULL,
                heading REAL NOT NULL,
                fix_quality TEXT NOT NULL,
                satellites INTEGER NOT NULL,
                hdop REAL NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_coverage_job_seg
                ON coverage_points(job_id, segment);

            CREATE TABLE IF NOT EXISTS ab_lines (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                field_id INTEGER REFERENCES fields(id) ON DELETE SET NULL,
                name TEXT NOT NULL,
                a_lat REAL NOT NULL,
                a_lon REAL NOT NULL,
                b_lat REAL NOT NULL,
                b_lon REAL NOT NULL,
                created_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
            );

            CREATE TABLE IF NOT EXISTS config (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            );
        ",
            )
            .map_err(|e| format!("Failed to create tables: {}", e))?;

        // Migration: add 'name' column to jobs if upgrading from old schema
        // that had 'csv_filename'. Ignore errors (column already exists).
        let _ = self.conn.execute(
            "ALTER TABLE jobs ADD COLUMN name TEXT NOT NULL DEFAULT ''",
            [],
        );

        Ok(())
    }

    // === Job operations ===

    /// Create a new job and return its ID.
    pub fn create_job(&self, name: &str, implement_width: f64) -> Result<i64, String> {
        self.conn
            .execute(
                "INSERT INTO jobs (name, implement_width_m) VALUES (?1, ?2)",
                rusqlite::params![name, implement_width],
            )
            .map_err(|e| format!("Failed to create job: {}", e))?;

        Ok(self.conn.last_insert_rowid())
    }

    /// End a job, recording its final stats.
    pub fn end_job(
        &self,
        job_id: i64,
        total_points: u64,
        total_segments: u32,
    ) -> Result<(), String> {
        self.conn.execute(
            "UPDATE jobs SET ended_at = datetime('now', 'localtime'), total_points = ?1, total_segments = ?2 WHERE id = ?3",
            rusqlite::params![total_points as i64, total_segments as i32, job_id],
        ).map_err(|e| format!("Failed to end job: {}", e))?;
        Ok(())
    }

    /// List all jobs, most recent first.
    pub fn list_jobs(&self) -> Result<Vec<SavedJob>, String> {
        let mut stmt = self.conn.prepare(
            "SELECT id, name, started_at, ended_at, implement_width_m, total_points, total_segments, notes
             FROM jobs ORDER BY started_at DESC"
        ).map_err(|e| format!("Failed to query jobs: {}", e))?;

        let jobs = stmt
            .query_map([], |row| {
                Ok(SavedJob {
                    id: row.get(0)?,
                    name: row.get(1)?,
                    started_at: row.get(2)?,
                    ended_at: row.get(3)?,
                    implement_width_m: row.get(4)?,
                    total_points: row.get(5)?,
                    total_segments: row.get(6)?,
                    notes: row.get(7)?,
                })
            })
            .map_err(|e| format!("Failed to read jobs: {}", e))?
            .filter_map(|r| r.ok())
            .collect();

        Ok(jobs)
    }

    /// Delete a job, its segments, and its coverage points by ID.
    pub fn delete_job(&self, id: i64) -> Result<(), String> {
        self.conn
            .execute(
                "DELETE FROM coverage_points WHERE job_id = ?1",
                rusqlite::params![id],
            )
            .map_err(|e| format!("Failed to delete coverage points: {}", e))?;
        self.conn
            .execute(
                "DELETE FROM segments WHERE job_id = ?1",
                rusqlite::params![id],
            )
            .map_err(|e| format!("Failed to delete segments: {}", e))?;
        self.conn
            .execute("DELETE FROM jobs WHERE id = ?1", rusqlite::params![id])
            .map_err(|e| format!("Failed to delete job: {}", e))?;
        Ok(())
    }

    // === Segment operations ===

    /// Create a new segment within a job and return its ID.
    pub fn create_segment(&self, job_id: i64, segment_number: u32) -> Result<i64, String> {
        self.conn
            .execute(
                "INSERT INTO segments (job_id, segment_number) VALUES (?1, ?2)",
                rusqlite::params![job_id, segment_number as i32],
            )
            .map_err(|e| format!("Failed to create segment: {}", e))?;

        Ok(self.conn.last_insert_rowid())
    }

    /// End a segment, recording its point count.
    pub fn end_segment(&self, segment_id: i64, point_count: u64) -> Result<(), String> {
        self.conn.execute(
            "UPDATE segments SET ended_at = datetime('now', 'localtime'), point_count = ?1 WHERE id = ?2",
            rusqlite::params![point_count as i64, segment_id],
        ).map_err(|e| format!("Failed to end segment: {}", e))?;
        Ok(())
    }

    // === AB Line operations ===

    /// Save an AB line, optionally associated with a field. Returns its ID.
    pub fn save_ab_line(
        &self,
        field_id: Option<i64>,
        name: &str,
        a_lat: f64,
        a_lon: f64,
        b_lat: f64,
        b_lon: f64,
    ) -> Result<i64, String> {
        self.conn
            .execute(
                "INSERT INTO ab_lines (field_id, name, a_lat, a_lon, b_lat, b_lon)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
                rusqlite::params![field_id, name, a_lat, a_lon, b_lat, b_lon],
            )
            .map_err(|e| format!("Failed to save AB line: {}", e))?;
        Ok(self.conn.last_insert_rowid())
    }

    /// Load all saved AB lines, ordered by field then creation date.
    pub fn list_ab_lines(&self) -> Result<Vec<SavedAbLine>, String> {
        let mut stmt = self
            .conn
            .prepare(
                "SELECT id, field_id, name, a_lat, a_lon, b_lat, b_lon, created_at
             FROM ab_lines
             ORDER BY field_id ASC NULLS LAST, created_at DESC",
            )
            .map_err(|e| format!("Failed to query AB lines: {}", e))?;

        let lines = stmt
            .query_map([], |row| {
                Ok(SavedAbLine {
                    id: row.get(0)?,
                    field_id: row.get(1)?,
                    name: row.get(2)?,
                    a_lat: row.get(3)?,
                    a_lon: row.get(4)?,
                    b_lat: row.get(5)?,
                    b_lon: row.get(6)?,
                    created_at: row.get(7)?,
                })
            })
            .map_err(|e| format!("Failed to read AB lines: {}", e))?
            .filter_map(|r| r.ok())
            .collect();

        Ok(lines)
    }

    /// Delete a saved AB line by ID.
    pub fn delete_ab_line(&self, id: i64) -> Result<(), String> {
        self.conn
            .execute("DELETE FROM ab_lines WHERE id = ?1", rusqlite::params![id])
            .map_err(|e| format!("Failed to delete AB line: {}", e))?;
        Ok(())
    }

    // === Field operations ===

    /// Create a new field and return its ID.
    pub fn create_field(&self, name: &str) -> Result<i64, String> {
        self.conn
            .execute(
                "INSERT INTO fields (name) VALUES (?1)",
                rusqlite::params![name],
            )
            .map_err(|e| format!("Failed to create field: {}", e))?;
        Ok(self.conn.last_insert_rowid())
    }

    /// Rename an existing field.
    pub fn rename_field(&self, id: i64, new_name: &str) -> Result<(), String> {
        self.conn
            .execute(
                "UPDATE fields SET name = ?1 WHERE id = ?2",
                rusqlite::params![new_name, id],
            )
            .map_err(|e| format!("Failed to rename field: {}", e))?;
        Ok(())
    }

    /// Delete a field. Its AB lines have their field_id set to NULL (not deleted).
    pub fn delete_field(&self, id: i64) -> Result<(), String> {
        self.conn
            .execute("DELETE FROM fields WHERE id = ?1", rusqlite::params![id])
            .map_err(|e| format!("Failed to delete field: {}", e))?;
        Ok(())
    }

    /// List all fields ordered by name.
    pub fn list_fields(&self) -> Result<Vec<SavedField>, String> {
        let mut stmt = self
            .conn
            .prepare("SELECT id, name, created_at FROM fields ORDER BY name ASC")
            .map_err(|e| format!("Failed to query fields: {}", e))?;

        let fields = stmt
            .query_map([], |row| {
                Ok(SavedField {
                    id: row.get(0)?,
                    name: row.get(1)?,
                    created_at: row.get(2)?,
                })
            })
            .map_err(|e| format!("Failed to read fields: {}", e))?
            .filter_map(|r| r.ok())
            .collect();

        Ok(fields)
    }

    // === Export / Import (for cross-PC transfer) ===

    /// Export all fields and AB lines to a portable JSON structure.
    /// Write the result to disk; the caller can copy the file to another PC
    /// and import it there with `import_ab_lines_json`.
    pub fn export_ab_lines_json(&self) -> Result<ExportBundle, String> {
        Ok(ExportBundle {
            fields: self.list_fields()?,
            ab_lines: self.list_ab_lines()?,
        })
    }

    /// Import fields and AB lines from an `ExportBundle`.
    /// Fields are matched by name — an existing field with the same name is
    /// reused rather than duplicated. Lines are matched by (field_id, name, a_lat,
    /// a_lon) to avoid duplicates when importing the same file twice.
    pub fn import_ab_lines_json(&self, bundle: &ExportBundle) -> Result<ImportStats, String> {
        let mut fields_added = 0u32;
        let mut lines_added = 0u32;
        let mut lines_skipped = 0u32;

        // Build a name→id map for existing fields
        let existing_fields = self.list_fields()?;
        let mut field_name_to_id: std::collections::HashMap<String, i64> = existing_fields
            .iter()
            .map(|f| (f.name.clone(), f.id))
            .collect();

        // Map from the *bundle's* field id to the *local* field id
        let mut bundle_field_id_map: std::collections::HashMap<i64, i64> =
            std::collections::HashMap::new();

        for f in &bundle.fields {
            let local_id = if let Some(&id) = field_name_to_id.get(&f.name) {
                id
            } else {
                let id = self.create_field(&f.name)?;
                field_name_to_id.insert(f.name.clone(), id);
                fields_added += 1;
                id
            };
            bundle_field_id_map.insert(f.id, local_id);
        }

        for line in &bundle.ab_lines {
            // Map the bundle field_id to the local field_id (may be None for
            // lines that weren't assigned to a field when exported)
            let local_field_id: Option<i64> = line
                .field_id
                .and_then(|bid| bundle_field_id_map.get(&bid).copied());

            // Check for duplicate: same name + a-point inside the same field
            let exists: bool = self
                .conn
                .query_row(
                    "SELECT COUNT(*) FROM ab_lines
                 WHERE name = ?1
                   AND ABS(a_lat - ?2) < 1e-8 AND ABS(a_lon - ?3) < 1e-8
                   AND (field_id = ?4 OR (field_id IS NULL AND ?4 IS NULL))",
                    rusqlite::params![line.name, line.a_lat, line.a_lon, local_field_id],
                    |row| row.get::<_, i64>(0),
                )
                .unwrap_or(0)
                > 0;

            if exists {
                lines_skipped += 1;
            } else {
                self.save_ab_line(
                    local_field_id,
                    &line.name,
                    line.a_lat,
                    line.a_lon,
                    line.b_lat,
                    line.b_lon,
                )?;
                lines_added += 1;
            }
        }

        Ok(ImportStats {
            fields_added,
            lines_added,
            lines_skipped,
        })
    }

    // === Config operations ===

    /// Get a config value by key.
    pub fn get_config(&self, key: &str) -> Option<String> {
        self.conn
            .query_row(
                "SELECT value FROM config WHERE key = ?1",
                rusqlite::params![key],
                |row| row.get(0),
            )
            .ok()
    }

    /// Set a config value (upsert).
    pub fn set_config(&self, key: &str, value: &str) -> Result<(), String> {
        self.conn.execute(
            "INSERT INTO config (key, value) VALUES (?1, ?2) ON CONFLICT(key) DO UPDATE SET value = ?2",
            rusqlite::params![key, value],
        ).map_err(|e| format!("Failed to set config: {}", e))?;
        Ok(())
    }

    // === Coverage point operations ===

    /// Insert a batch of coverage points in a single transaction.
    /// Call this periodically (e.g. every 50 points) for good throughput
    /// without blocking the GPS thread.
    pub fn insert_coverage_batch(
        &self,
        job_id: i64,
        points: &[CoveragePointRow],
    ) -> Result<(), String> {
        if points.is_empty() {
            return Ok(());
        }

        self.conn
            .execute("BEGIN", [])
            .map_err(|e| format!("Failed to begin transaction: {}", e))?;

        {
            let mut stmt = self.conn.prepare_cached(
                "INSERT INTO coverage_points (job_id, segment, timestamp_ms, latitude, longitude,
                 altitude, speed, heading, fix_quality, satellites, hdop)
                 VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11)"
            ).map_err(|e| format!("Failed to prepare insert: {}", e))?;

            for pt in points {
                let quality_str = fix_quality_to_str(pt.fix_quality);
                stmt.execute(rusqlite::params![
                    job_id,
                    pt.segment,
                    pt.timestamp_ms as i64,
                    pt.latitude,
                    pt.longitude,
                    pt.altitude,
                    pt.speed,
                    pt.heading,
                    quality_str,
                    pt.satellites as i32,
                    pt.hdop,
                ])
                .map_err(|e| format!("Failed to insert coverage point: {}", e))?;
            }
        }

        self.conn
            .execute("COMMIT", [])
            .map_err(|e| format!("Failed to commit transaction: {}", e))?;

        Ok(())
    }

    /// Load all coverage points for a job, ordered by segment then timestamp.
    pub fn load_coverage_points(&self, job_id: i64) -> Result<Vec<CoveragePointRow>, String> {
        let mut stmt = self
            .conn
            .prepare(
                "SELECT segment, timestamp_ms, latitude, longitude, altitude,
                    speed, heading, fix_quality, satellites, hdop
             FROM coverage_points
             WHERE job_id = ?1
             ORDER BY segment ASC, timestamp_ms ASC",
            )
            .map_err(|e| format!("Failed to query coverage points: {}", e))?;

        let points = stmt
            .query_map(rusqlite::params![job_id], |row| {
                let quality_str: String = row.get(7)?;
                Ok(CoveragePointRow {
                    segment: row.get(0)?,
                    timestamp_ms: row.get::<_, i64>(1)? as u64,
                    latitude: row.get(2)?,
                    longitude: row.get(3)?,
                    altitude: row.get(4)?,
                    speed: row.get(5)?,
                    heading: row.get(6)?,
                    fix_quality: str_to_fix_quality(&quality_str),
                    satellites: row.get::<_, i32>(8)? as u8,
                    hdop: row.get(9)?,
                })
            })
            .map_err(|e| format!("Failed to read coverage points: {}", e))?
            .filter_map(|r| r.ok())
            .collect();

        Ok(points)
    }

    /// Count coverage points for a job.
    pub fn count_coverage_points(&self, job_id: i64) -> Result<u64, String> {
        self.conn
            .query_row(
                "SELECT COUNT(*) FROM coverage_points WHERE job_id = ?1",
                rusqlite::params![job_id],
                |row| row.get::<_, i64>(0),
            )
            .map(|c| c as u64)
            .map_err(|e| format!("Failed to count coverage points: {}", e))
    }

    /// Delete all coverage points for a job (without deleting the job itself).
    pub fn clear_coverage_points(&self, job_id: i64) -> Result<(), String> {
        self.conn
            .execute(
                "DELETE FROM coverage_points WHERE job_id = ?1",
                rusqlite::params![job_id],
            )
            .map_err(|e| format!("Failed to clear coverage points: {}", e))?;
        Ok(())
    }

    /// Export coverage points for a job to CSV format string.
    pub fn export_coverage_csv(&self, job_id: i64) -> Result<String, String> {
        let points = self.load_coverage_points(job_id)?;
        let mut csv = String::from(
            "segment,timestamp_ms,latitude,longitude,altitude,speed,heading,fix_quality,satellites,hdop\n"
        );
        for pt in &points {
            csv.push_str(&format!(
                "{},{},{:.8},{:.8},{:.2},{:.3},{:.2},{},{},{}\n",
                pt.segment,
                pt.timestamp_ms,
                pt.latitude,
                pt.longitude,
                pt.altitude,
                pt.speed,
                pt.heading,
                fix_quality_to_str(pt.fix_quality),
                pt.satellites,
                pt.hdop,
            ));
        }
        Ok(csv)
    }
}

/// A saved field (paddock) from the database.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SavedField {
    pub id: i64,
    pub name: String,
    pub created_at: String,
}

/// A saved job (coverage recording session) from the database.
#[derive(Debug, Clone)]
pub struct SavedJob {
    pub id: i64,
    pub name: String,
    pub started_at: String,
    pub ended_at: Option<String>,
    pub implement_width_m: f64,
    pub total_points: i64,
    pub total_segments: i32,
    pub notes: Option<String>,
}

/// A saved AB line from the database.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SavedAbLine {
    pub id: i64,
    /// The field this line belongs to, if any.
    pub field_id: Option<i64>,
    pub name: String,
    pub a_lat: f64,
    pub a_lon: f64,
    pub b_lat: f64,
    pub b_lon: f64,
    pub created_at: String,
}

/// Portable bundle for cross-PC transfer of fields and AB lines.
/// Serialise to JSON, copy the file, then call `import_ab_lines_json`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportBundle {
    pub fields: Vec<SavedField>,
    pub ab_lines: Vec<SavedAbLine>,
}

/// Result of an import operation.
#[derive(Debug, Clone)]
pub struct ImportStats {
    pub fields_added: u32,
    pub lines_added: u32,
    pub lines_skipped: u32,
}

/// A coverage point row for database read/write.
/// Used for batch inserts and query results.
#[derive(Debug, Clone)]
pub struct CoveragePointRow {
    pub segment: u32,
    pub timestamp_ms: u64,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub speed: f64,
    pub heading: f64,
    pub fix_quality: FixQuality,
    pub satellites: u8,
    pub hdop: f64,
}

/// Convert FixQuality to its database string representation.
fn fix_quality_to_str(q: FixQuality) -> &'static str {
    match q {
        FixQuality::NoFix => "NoFix",
        FixQuality::Gps => "GPS",
        FixQuality::Dgps => "DGPS",
        FixQuality::Rtk => "RTK",
        FixQuality::RtkFloat => "RtkFloat",
    }
}

/// Parse a database string back to FixQuality.
fn str_to_fix_quality(s: &str) -> FixQuality {
    match s {
        "RTK" => FixQuality::Rtk,
        "RtkFloat" => FixQuality::RtkFloat,
        "DGPS" => FixQuality::Dgps,
        "GPS" => FixQuality::Gps,
        _ => FixQuality::NoFix,
    }
}
