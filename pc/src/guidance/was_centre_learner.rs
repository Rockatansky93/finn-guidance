//! Auto-learning WAS centre adjustment.
//!
//! ## What this solves
//!
//! The WAS pot reading at "wheels straight" drifts with cab temperature
//! (resistive element warm-up, ESP32 ADC reference drift). Observed drift
//! is 50+ ADC counts over a morning's warm-up — at ~414 counts of WAS
//! range, that's ~12% of full scale, which translates to several degrees
//! of perceived steering angle offset. The ESP32's inner loop then drives
//! the wheels off-centre when the PC commands zero angle.
//!
//! ## How it works
//!
//! While auto-steer is engaged AND the GPS evidence says we're tracking
//! straight (no XTE drift, no heading rate, no commanded angle), whatever
//! the WAS is currently reading is by definition the true centre. We
//! exponentially blend the current raw WAS reading into the running
//! "learned centre" estimate and push it down to the ESP32 via
//! `$FINNCFG,WASCNT` so the inner loop adjusts in real time.
//!
//! Updates happen every 10s of continuous "definitely straight" evidence,
//! with a 1% blend per update — full absorption of a step change takes
//! ~5 minutes of stable straight running. That's deliberately slow so
//! no plausible field condition (long bend, sustained crosswind) can
//! corrupt it inside a single engage.
//!
//! ## Safety bounds
//!
//! The learned centre is hard-clamped to the manual three-point calibration
//! ±100 ADC counts. The observed drift is ~50 counts so this gives 2×
//! headroom; anything bigger is a bug and gets rejected.
//!
//! ## Lifecycle
//!
//! - Created with the manual cal value at engage time.
//! - Persists across engage/disengage cycles within one process run
//!   (so a headland turn doesn't reset learned drift).
//! - Power cycle resets to manual cal (NVS is the source of truth).
//! - Manual recalibration via the GUI three-point wizard wipes learned
//!   state (the learner is recreated with the new manual value).

use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// How often the learner emits a new centre value when conditions allow.
const UPDATE_INTERVAL: Duration = Duration::from_secs(10);

/// Blend factor per update — 1% means a step change is fully absorbed
/// after ~30 updates = 5 minutes of stable straight running.
const BLEND_ALPHA: f64 = 0.01;

/// Max counts the learned centre can deviate from the manual cal.
/// Observed thermal drift is ~50 counts; 100 gives 2× headroom.
const MAX_OFFSET_FROM_MANUAL: i32 = 100;

/// Window over which we measure heading-error stability (samples at 10Hz).
const STABILITY_WINDOW: usize = 50; // 5 seconds at 10Hz

/// Gate thresholds for "definitely straight" detection.
const MAX_HEADING_ERROR_DEG: f64 = 1.5;
const MAX_XTE_M: f64 = 0.30;
const MAX_XTE_RATE_MPS: f64 = 0.05; // 5 cm/s
const MAX_HEADING_STDDEV_DEG: f64 = 0.30;
const MAX_DESIRED_ANGLE_DEG: f64 = 0.5;
const MIN_SPEED_MPS: f64 = 1.0;

/// Per-tick input to the learner.
#[derive(Debug, Clone)]
pub struct LearnerTick {
    /// Current cross-track error in metres.
    pub xte_m: f64,
    /// Current heading error in degrees (vehicle vs line bearing).
    pub heading_error_deg: f64,
    /// Current speed in m/s.
    pub speed_mps: f64,
    /// Most recent desired steering angle from the controller (degrees).
    pub desired_angle_deg: f64,
    /// Latest raw WAS reading from the ESP32 (`$FINNMTR` `was_raw` field).
    pub was_raw: u16,
    /// Wall-clock time of this tick.
    pub now: Instant,
}

/// Per-tick output from the learner.
#[derive(Debug, Clone, Default)]
pub struct LearnerOutput {
    /// If `Some`, the steer thread should send this value to the ESP32 via
    /// `$FINNCFG,WASCNT`. The learner has already updated its internal state.
    pub new_centre: Option<u16>,
    /// Whether this tick met all "definitely straight" gate conditions.
    /// For telemetry and diagnostics.
    pub gate_passed: bool,
    /// Current learned centre value (always populated, for display/telemetry).
    pub current_learned_centre: u16,
}

pub struct WasCentreLearner {
    /// The manual three-point cal centre value. Reference point for the
    /// allowed-offset clamp.
    manual_centre: u16,

    /// Learned centre as a float, so the 1% blend can accumulate sub-count
    /// changes that round into integer movements over time.
    learned_centre_f: f64,

    /// Last integer centre we sent to the ESP32. We only emit a new
    /// `$FINNCFG,WASCNT` when the rounded value changes, to avoid
    /// hammering the serial link with identical commands.
    last_sent_centre: u16,

    /// Rolling window of recent heading_error samples, for std-dev gate.
    heading_window: VecDeque<f64>,

    /// Last time we emitted a centre update (or learner was created).
    last_update_time: Instant,

    /// Time when the gate FIRST started passing continuously. None when
    /// gate is currently failing. Used to ensure we have a sustained
    /// straight section before counting it.
    gate_pass_start: Option<Instant>,
}

impl WasCentreLearner {
    pub fn new(manual_centre: u16, now: Instant) -> Self {
        Self {
            manual_centre,
            learned_centre_f: manual_centre as f64,
            last_sent_centre: manual_centre,
            heading_window: VecDeque::with_capacity(STABILITY_WINDOW),
            last_update_time: now,
            gate_pass_start: None,
        }
    }

    /// Update the manual cal reference (e.g. after a fresh three-point cal
    /// from the GUI). Resets the learned state.
    pub fn reset_to_manual(&mut self, manual_centre: u16, now: Instant) {
        self.manual_centre = manual_centre;
        self.learned_centre_f = manual_centre as f64;
        self.last_sent_centre = manual_centre;
        self.heading_window.clear();
        self.last_update_time = now;
        self.gate_pass_start = None;
    }

    /// Get the current learned centre as a rounded integer.
    pub fn current_centre(&self) -> u16 {
        self.learned_centre_f.round() as u16
    }

    /// Process one steer-thread tick and produce an output.
    /// Caller should only invoke this while auto-steer is engaged.
    pub fn tick(&mut self, t: &LearnerTick, xte_rate_mps: f64) -> LearnerOutput {
        // ── Maintain heading-error stability window ─────────────────────
        self.heading_window.push_back(t.heading_error_deg);
        while self.heading_window.len() > STABILITY_WINDOW {
            self.heading_window.pop_front();
        }

        // ── Evaluate "definitely straight" gate ─────────────────────────
        let gate_passed = self.gate_passes(t, xte_rate_mps);

        // Track sustained gate-pass time.
        if gate_passed {
            if self.gate_pass_start.is_none() {
                self.gate_pass_start = Some(t.now);
            }
        } else {
            self.gate_pass_start = None;
        }

        let mut out = LearnerOutput {
            new_centre: None,
            gate_passed,
            current_learned_centre: self.current_centre(),
        };

        // ── Decide whether to emit an update ───────────────────────────
        // Conditions for an update:
        //   1. Gate currently passing
        //   2. Gate has been passing continuously for the full window
        //   3. Heading window is full (have a real std-dev)
        //   4. UPDATE_INTERVAL has elapsed since last update
        let sustained = self
            .gate_pass_start
            .map(|s| t.now.duration_since(s) >= Duration::from_millis(5_000))
            .unwrap_or(false);

        if gate_passed
            && sustained
            && self.heading_window.len() >= STABILITY_WINDOW
            && t.now.duration_since(self.last_update_time) >= UPDATE_INTERVAL
        {
            // Blend the current raw WAS into the learned centre.
            let target = t.was_raw as f64;
            let new_f = self.learned_centre_f + BLEND_ALPHA * (target - self.learned_centre_f);

            // Clamp to manual_centre ± MAX_OFFSET_FROM_MANUAL.
            let lo = (self.manual_centre as i32 - MAX_OFFSET_FROM_MANUAL).max(0) as f64;
            let hi = (self.manual_centre as i32 + MAX_OFFSET_FROM_MANUAL).min(u16::MAX as i32)
                as f64;
            let clamped = new_f.clamp(lo, hi);

            self.learned_centre_f = clamped;
            self.last_update_time = t.now;

            // Only emit if the rounded value actually changed.
            let rounded = clamped.round() as u16;
            if rounded != self.last_sent_centre {
                self.last_sent_centre = rounded;
                out.new_centre = Some(rounded);
            }
            out.current_learned_centre = rounded;
        }

        out
    }

    /// Check all gate conditions for "definitely going straight".
    fn gate_passes(&self, t: &LearnerTick, xte_rate_mps: f64) -> bool {
        if t.speed_mps < MIN_SPEED_MPS {
            return false;
        }
        if t.heading_error_deg.abs() > MAX_HEADING_ERROR_DEG {
            return false;
        }
        if t.xte_m.abs() > MAX_XTE_M {
            return false;
        }
        if xte_rate_mps.abs() > MAX_XTE_RATE_MPS {
            return false;
        }
        if t.desired_angle_deg.abs() > MAX_DESIRED_ANGLE_DEG {
            return false;
        }

        // Heading-error std-dev gate — only meaningful with a full window.
        if self.heading_window.len() >= STABILITY_WINDOW {
            let n = self.heading_window.len() as f64;
            let mean: f64 = self.heading_window.iter().sum::<f64>() / n;
            let var: f64 = self
                .heading_window
                .iter()
                .map(|x| (x - mean).powi(2))
                .sum::<f64>()
                / n;
            let stddev = var.sqrt();
            if stddev > MAX_HEADING_STDDEV_DEG {
                return false;
            }
        } else {
            // Don't allow updates until the window is full.
            return false;
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_tick(was_raw: u16, now: Instant) -> LearnerTick {
        LearnerTick {
            xte_m: 0.0,
            heading_error_deg: 0.0,
            speed_mps: 2.0,
            desired_angle_deg: 0.0,
            was_raw,
            now,
        }
    }

    #[test]
    fn starts_at_manual_centre() {
        let now = Instant::now();
        let l = WasCentreLearner::new(1832, now);
        assert_eq!(l.current_centre(), 1832);
    }

    #[test]
    fn no_update_below_speed_gate() {
        let now = Instant::now();
        let mut l = WasCentreLearner::new(1832, now);
        // Fill window with stable heading, but at zero speed.
        for i in 0..STABILITY_WINDOW + 10 {
            let mut t = straight_tick(1900, now + Duration::from_millis((i as u64) * 100));
            t.speed_mps = 0.0;
            let out = l.tick(&t, 0.0);
            assert!(!out.gate_passed, "gate must fail when stationary");
            assert!(out.new_centre.is_none());
        }
        assert_eq!(l.current_centre(), 1832);
    }

    #[test]
    fn learns_drift_over_time() {
        let start = Instant::now();
        let mut l = WasCentreLearner::new(1832, start);

        // Simulate 6 minutes of straight running with WAS reading 1882
        // (50-count drift). Tick at 10Hz.
        let drifted_raw = 1882u16;
        let mut last_seen_centre = 1832u16;
        for i in 0..(60 * 6 * 10) {
            let now = start + Duration::from_millis((i as u64) * 100);
            let t = straight_tick(drifted_raw, now);
            let out = l.tick(&t, 0.0);
            if let Some(c) = out.new_centre {
                last_seen_centre = c;
            }
        }

        // After 6 minutes of stable conditions with 10-second update
        // interval and 1% blend, learned centre should have moved most
        // of the way toward 1882. Expect at least 30 of the 50 counts.
        let drift = last_seen_centre as i32 - 1832;
        assert!(
            drift >= 30,
            "expected significant drift learning, got {}",
            drift
        );
        assert!(
            drift <= 50,
            "learned centre should not overshoot the WAS reading, got {}",
            drift
        );
    }

    #[test]
    fn clamps_to_max_offset() {
        let start = Instant::now();
        let mut l = WasCentreLearner::new(1832, start);

        // Drive a wildly wrong WAS reading for an extended period.
        // Should never exceed ±100 from manual.
        let wild_raw = 2500u16;
        for i in 0..(60 * 60 * 10) {
            // 1 hour at 10Hz
            let now = start + Duration::from_millis((i as u64) * 100);
            let t = straight_tick(wild_raw, now);
            l.tick(&t, 0.0);
        }

        let c = l.current_centre();
        assert!(
            c <= 1832 + MAX_OFFSET_FROM_MANUAL as u16,
            "learned centre {} exceeded clamp ceiling",
            c
        );
        assert!(
            c >= 1832 - MAX_OFFSET_FROM_MANUAL as u16,
            "learned centre {} below clamp floor",
            c
        );
    }

    #[test]
    fn rejects_high_heading_error() {
        let start = Instant::now();
        let mut l = WasCentreLearner::new(1832, start);
        for i in 0..STABILITY_WINDOW + 200 {
            let mut t = straight_tick(1900, start + Duration::from_millis((i as u64) * 100));
            t.heading_error_deg = 5.0; // Too high — gate must fail
            let out = l.tick(&t, 0.0);
            assert!(!out.gate_passed);
        }
        assert_eq!(l.current_centre(), 1832);
    }

    #[test]
    fn rejects_high_xte_rate() {
        let start = Instant::now();
        let mut l = WasCentreLearner::new(1832, start);
        for i in 0..STABILITY_WINDOW + 200 {
            let t = straight_tick(1900, start + Duration::from_millis((i as u64) * 100));
            let out = l.tick(&t, 0.20); // Drifting sideways at 20 cm/s
            assert!(!out.gate_passed);
        }
        assert_eq!(l.current_centre(), 1832);
    }

    #[test]
    fn reset_to_manual_clears_state() {
        let start = Instant::now();
        let mut l = WasCentreLearner::new(1832, start);

        // Learn some drift.
        for i in 0..(60 * 6 * 10) {
            let now = start + Duration::from_millis((i as u64) * 100);
            l.tick(&straight_tick(1880, now), 0.0);
        }
        assert!(l.current_centre() > 1832);

        // Reset to new manual cal.
        l.reset_to_manual(1850, start + Duration::from_secs(60 * 7));
        assert_eq!(l.current_centre(), 1850);
    }
}
