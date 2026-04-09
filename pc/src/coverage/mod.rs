//! Coverage logging module - records implement coverage for field mapping.
//!
//! Architecture:
//!   - CSV files for spatial GPS data (human-readable, QGIS-compatible)
//!   - SQLite database for job/segment metadata, AB lines, and config
//!
//! Coverage is filtered to avoid duplicate/redundant points:
//!   - Deduplication: only logs when GPS epoch changes (new fix from receiver)
//!   - Distance filter: configurable minimum distance between logged points
//!   - Time filter: configurable minimum time between logged points
//!
//! Each engage/disengage cycle creates a segment within the current job.

pub mod logger;
pub mod db;
