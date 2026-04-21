//! Position module - vehicle position tracking, trail management, and interpolation.
//!
//! Decision #026: heading_filter removed — the LC29H BA handles IMU+GPS
//! heading fusion internally via its onboard dead-reckoning engine.

pub mod tracker;
pub mod interpolator;
