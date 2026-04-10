//! Communications module — serial messaging between PC and ESP32.
//!
//! The motor ESP32 is on a separate USB serial port. This module will handle
//! opening that port and sending $FINNSTEER commands for the PID controller.

pub mod serial;
