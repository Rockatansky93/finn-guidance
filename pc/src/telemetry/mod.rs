//! Telemetry logging for steering performance analysis.
//!
//! Captures 10Hz control loop data and 1Hz summaries to newline-delimited
//! JSON files (`.jsonl`). Designed for post-run analysis — either manually
//! or via a FINN Core worker node for automated tuning recommendations.
//!
//! ## File format
//!
//! Each line is a self-contained JSON object with a `type` field:
//! - `"iter"` — one per steer loop iteration (10Hz), full control state
//! - `"summary"` — one per second, aggregated statistics
//! - `"event"` — discrete events (engage, disengage, pass change)
//!
//! ## File naming
//!
//! `logs/steer_YYYY-MM-DD_HHMMSS.jsonl` — new file per auto-steer engage.

pub mod logger;

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

// ─────────────────────────────────────────────────────────────────────
// Shared atomic drop counters — incremented by reader threads,
// read-and-reset by the steer thread each second for telemetry.
// ─────────────────────────────────────────────────────────────────────

/// Atomic drop counters shared between reader threads and the steer thread.
///
/// Each reader thread (GPS, motor) has two consumers (GUI, steer). When a
/// bounded channel is full, the reader increments the corresponding counter
/// instead of blocking (Decision #027). The steer thread periodically
/// swaps these to zero and records the values in the telemetry summary.
#[derive(Clone)]
pub struct SharedDropCounters {
    pub gps_gui: Arc<AtomicU64>,
    pub gps_steer: Arc<AtomicU64>,
    pub mtr_gui: Arc<AtomicU64>,
    pub mtr_steer: Arc<AtomicU64>,
}

impl SharedDropCounters {
    /// Create a new set of zero-initialised counters.
    pub fn new() -> Self {
        Self {
            gps_gui: Arc::new(AtomicU64::new(0)),
            gps_steer: Arc::new(AtomicU64::new(0)),
            mtr_gui: Arc::new(AtomicU64::new(0)),
            mtr_steer: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Atomically read and reset all counters. Returns a snapshot.
    /// Called by the steer thread once per second for the telemetry summary.
    pub fn swap_all(&self) -> logger::DropCounts {
        logger::DropCounts {
            gps_gui: self.gps_gui.swap(0, Ordering::Relaxed),
            gps_steer: self.gps_steer.swap(0, Ordering::Relaxed),
            mtr_gui: self.mtr_gui.swap(0, Ordering::Relaxed),
            mtr_steer: self.mtr_steer.swap(0, Ordering::Relaxed),
        }
    }
}
