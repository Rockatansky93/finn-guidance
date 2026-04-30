//! Coverage logging module - records implement coverage for field mapping.
//!
//! Architecture:
//!   - SQLite database for all coverage data (points, job/segment metadata,
//!     AB lines, and config)
//!   - Points written in batches via transactions for performance
//!   - In-memory render cache for the field view canvas
//!   - CSV export available as an option for external tools (QGIS etc.)
//!
//! Coverage is filtered to avoid duplicate/redundant points:
//!   - Deduplication: only logs when GPS epoch changes (new fix from receiver)
//!   - Distance filter: configurable minimum distance between logged points (default 0.25m)
//!   - Time filter: configurable minimum time between logged points
//!
//! Each engage/disengage cycle creates a segment within the current job.

pub mod db;
pub mod logger;
