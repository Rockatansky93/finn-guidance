//! Coverage database - SQLite storage for job metadata, segments, AB lines, and fields.
//!
//! The database stores structured metadata alongside the CSV coverage files:
//!   - Fields: named paddocks, each containing a set of AB lines
//!   - Jobs: each recording session with start/end times, total area, filename
//!   - Segments: each engage/disengage cycle within a job
//!   - AB Lines: saved guidance lines grouped by field, for reuse across sessions
//!   - Config: persisted settings (implement width, log filters, etc.)
//!
//! AB line organisation:
//!   Fields group related AB lines (e.g. a paddock typically has up to 4 lines:
//!   N/S header, E/W header, and diagonals). This grouping makes it easy to find
//!   the right line at the start of a run, especially when transferring between PCs.

use std::path::Path;
use serde::{Deserialize, Serialize};

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
        self.conn.execute_batch("
            CREATE TABLE IF NOT EXISTS fields (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                notes TEXT,
                created_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
            );

            CREATE TABLE IF NOT EXISTS jobs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                csv_filename TEXT NOT NULL,
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
        ").map_err(|e| format!("Failed to create tables: {}", e))?;

        Ok(())
    }

    // === Job operations ===

    /// Create a new job and return its ID.
    pub fn create_job(&self, csv_filename: &str, implement_width: f64) -> Result<i64, String> {
        self.conn.execute(
            "INSERT INTO jobs (csv_filename, implement_width_m) VALUES (?1, ?2)",
            rusqlite::params![csv_filename, implement_width],
        ).map_err(|e| format!("Failed to create job: {}", e))?;

        Ok(self.conn.last_insert_rowid())
    }

    /// End a job, recording its final stats.
    pub fn end_job(&self, job_id: i64, total_points: u64, total_segments: u32) -> Result<(), String> {
        self.conn.execute(
            "UPDATE jobs SET ended_at = datetime('now', 'localtime'), total_points = ?1, total_segments = ?2 WHERE id = ?3",
            rusqlite::params![total_points as i64, total_segments as i32, job_id],
        ).map_err(|e| format!("Failed to end job: {}", e))?;
        Ok(())
    }

    /// List all jobs, most recent first.
    pub fn list_jobs(&self) -> Result<Vec<SavedJob>, String> {
        let mut stmt = self.conn.prepare(
            "SELECT id, csv_filename, started_at, ended_at, implement_width_m, total_points, total_segments, notes
             FROM jobs ORDER BY started_at DESC"
        ).map_err(|e| format!("Failed to query jobs: {}", e))?;

        let jobs = stmt.query_map([], |row| {
            Ok(SavedJob {
                id: row.get(0)?,
                csv_filename: row.get(1)?,
                started_at: row.get(2)?,
                ended_at: row.get(3)?,
                implement_width_m: row.get(4)?,
                total_points: row.get(5)?,
                total_segments: row.get(6)?,
                notes: row.get(7)?,
            })
        }).map_err(|e| format!("Failed to read jobs: {}", e))?
        .filter_map(|r| r.ok())
        .collect();

        Ok(jobs)
    }

    /// Delete a job and its segments by ID.
    pub fn delete_job(&self, id: i64) -> Result<(), String> {
        self.conn.execute("DELETE FROM segments WHERE job_id = ?1", rusqlite::params![id])
            .map_err(|e| format!("Failed to delete segments: {}", e))?;
        self.conn.execute("DELETE FROM jobs WHERE id = ?1", rusqlite::params![id])
            .map_err(|e| format!("Failed to delete job: {}", e))?;
        Ok(())
    }

    // === Segment operations ===

    /// Create a new segment within a job and return its ID.
    pub fn create_segment(&self, job_id: i64, segment_number: u32) -> Result<i64, String> {
        self.conn.execute(
            "INSERT INTO segments (job_id, segment_number) VALUES (?1, ?2)",
            rusqlite::params![job_id, segment_number as i32],
        ).map_err(|e| format!("Failed to create segment: {}", e))?;

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
        a_lat: f64, a_lon: f64,
        b_lat: f64, b_lon: f64,
    ) -> Result<i64, String> {
        self.conn.execute(
            "INSERT INTO ab_lines (field_id, name, a_lat, a_lon, b_lat, b_lon)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
            rusqlite::params![field_id, name, a_lat, a_lon, b_lat, b_lon],
        ).map_err(|e| format!("Failed to save AB line: {}", e))?;
        Ok(self.conn.last_insert_rowid())
    }

    /// Load all saved AB lines, ordered by field then creation date.
    pub fn list_ab_lines(&self) -> Result<Vec<SavedAbLine>, String> {
        let mut stmt = self.conn.prepare(
            "SELECT id, field_id, name, a_lat, a_lon, b_lat, b_lon, created_at
             FROM ab_lines
             ORDER BY field_id ASC NULLS LAST, created_at DESC"
        ).map_err(|e| format!("Failed to query AB lines: {}", e))?;

        let lines = stmt.query_map([], |row| {
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
        }).map_err(|e| format!("Failed to read AB lines: {}", e))?
        .filter_map(|r| r.ok())
        .collect();

        Ok(lines)
    }

    /// Delete a saved AB line by ID.
    pub fn delete_ab_line(&self, id: i64) -> Result<(), String> {
        self.conn.execute("DELETE FROM ab_lines WHERE id = ?1", rusqlite::params![id])
            .map_err(|e| format!("Failed to delete AB line: {}", e))?;
        Ok(())
    }

    // === Field operations ===

    /// Create a new field and return its ID.
    pub fn create_field(&self, name: &str) -> Result<i64, String> {
        self.conn.execute(
            "INSERT INTO fields (name) VALUES (?1)",
            rusqlite::params![name],
        ).map_err(|e| format!("Failed to create field: {}", e))?;
        Ok(self.conn.last_insert_rowid())
    }

    /// Rename an existing field.
    pub fn rename_field(&self, id: i64, new_name: &str) -> Result<(), String> {
        self.conn.execute(
            "UPDATE fields SET name = ?1 WHERE id = ?2",
            rusqlite::params![new_name, id],
        ).map_err(|e| format!("Failed to rename field: {}", e))?;
        Ok(())
    }

    /// Delete a field. Its AB lines have their field_id set to NULL (not deleted).
    pub fn delete_field(&self, id: i64) -> Result<(), String> {
        self.conn.execute("DELETE FROM fields WHERE id = ?1", rusqlite::params![id])
            .map_err(|e| format!("Failed to delete field: {}", e))?;
        Ok(())
    }

    /// List all fields ordered by name.
    pub fn list_fields(&self) -> Result<Vec<SavedField>, String> {
        let mut stmt = self.conn.prepare(
            "SELECT id, name, created_at FROM fields ORDER BY name ASC"
        ).map_err(|e| format!("Failed to query fields: {}", e))?;

        let fields = stmt.query_map([], |row| {
            Ok(SavedField {
                id: row.get(0)?,
                name: row.get(1)?,
                created_at: row.get(2)?,
            })
        }).map_err(|e| format!("Failed to read fields: {}", e))?
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
        let mut field_name_to_id: std::collections::HashMap<String, i64> =
            existing_fields.iter().map(|f| (f.name.clone(), f.id)).collect();

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
            let local_field_id: Option<i64> = line.field_id
                .and_then(|bid| bundle_field_id_map.get(&bid).copied());

            // Check for duplicate: same name + a-point inside the same field
            let exists: bool = self.conn.query_row(
                "SELECT COUNT(*) FROM ab_lines
                 WHERE name = ?1
                   AND ABS(a_lat - ?2) < 1e-8 AND ABS(a_lon - ?3) < 1e-8
                   AND (field_id = ?4 OR (field_id IS NULL AND ?4 IS NULL))",
                rusqlite::params![line.name, line.a_lat, line.a_lon, local_field_id],
                |row| row.get::<_, i64>(0),
            ).unwrap_or(0) > 0;

            if exists {
                lines_skipped += 1;
            } else {
                self.save_ab_line(
                    local_field_id,
                    &line.name,
                    line.a_lat, line.a_lon,
                    line.b_lat, line.b_lon,
                )?;
                lines_added += 1;
            }
        }

        Ok(ImportStats { fields_added, lines_added, lines_skipped })
    }

    // === Config operations ===

    /// Get a config value by key.
    pub fn get_config(&self, key: &str) -> Option<String> {
        self.conn.query_row(
            "SELECT value FROM config WHERE key = ?1",
            rusqlite::params![key],
            |row| row.get(0),
        ).ok()
    }

    /// Set a config value (upsert).
    pub fn set_config(&self, key: &str, value: &str) -> Result<(), String> {
        self.conn.execute(
            "INSERT INTO config (key, value) VALUES (?1, ?2) ON CONFLICT(key) DO UPDATE SET value = ?2",
            rusqlite::params![key, value],
        ).map_err(|e| format!("Failed to set config: {}", e))?;
        Ok(())
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
    pub csv_filename: String,
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
