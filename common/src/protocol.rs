//! Serial protocol definitions for FINN ESP32 <-> PC communication.
//!
//! ## Decision #026 architecture:
//!
//! Only ONE ESP32 (motor controller) communicates with the PC.
//! The GPS module (LC29H BA) connects directly to the PC via USB.
//!
//! ## PC -> Motor ESP32
//! - `$FINNSTEER,<desired_angle_x100>*<checksum>`  (desired angle x 100, ~10Hz)
//! - `$FINNCFG,WAS,<centre>,<left>,<right>*<checksum>`  (WAS calibration — writes NVS, resets EMA)
//! - `$FINNCFG,WASCNT,<centre>*<checksum>`  (live WAS centre update — RAM only, no EMA reset)
//! - `$FINNCFG,PID,<kp_x100>,<min_pwm>,<max_pwm>*<checksum>`  (inner loop tuning)
//! - `$FINNCFG,INVERT,<0|1>*<checksum>`  (motor direction)
//! - `$FINNCFG,WASF,<ema_alpha_x100>,<deadzone_x100>,<curve_exp_x100>*<checksum>`  (WAS filtering)
//!
//! ## Motor ESP32 -> PC
//! - `$FINNMTR,<pwm>,<was_raw>,<angle_x100>,<enabled>,<uptime_ms>*<checksum>` (10Hz)
//! - `$FINNACK,<param>,<OK|ERR>*<checksum>`  (config acknowledgement)

use crate::types::*;

/// All possible messages parsed from the motor ESP32 serial stream
#[derive(Debug, Clone)]
pub enum FinnMessage {
    /// Motor controller status with WAS feedback (from motor ESP32, 10Hz)
    MotorStatus(MotorStatus),
    /// Config acknowledgement (from motor ESP32, after $FINNCFG)
    ConfigAck(ConfigAck),
}

/// GPS serial baud rate (used by LC29H BA and motor ESP32)
pub const SERIAL_BAUD_RATE: u32 = 115200;

/// Compute NMEA-style XOR checksum over a message body (between $ and *)
pub fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// Format a complete FINN sentence from a body string.
/// Adds $, *, checksum, and \r\n.
fn format_finn_sentence(body: &str) -> String {
    let cs = nmea_checksum(body);
    format!("${}*{:02X}\r\n", body, cs)
}

/// Format a steer command for sending to the motor ESP32.
/// The value is the desired steering angle multiplied by 100.
/// e.g. -5.23 degrees -> format_steer_command(-523)
///
/// # Example
/// ```
/// use finn_guidance_common::protocol::format_steer_command;
/// let cmd = format_steer_command(-523);
/// assert!(cmd.starts_with("$FINNSTEER,-523*"));
/// assert!(cmd.ends_with("\r\n"));
/// ```
pub fn format_steer_command(angle_x100: i16) -> String {
    let body = format!("FINNSTEER,{}", angle_x100);
    format_finn_sentence(&body)
}

/// Format a WAS calibration config command.
/// Sends three-point calibration values (raw ADC counts) to the ESP32 NVS.
pub fn format_was_config(centre: u16, left: u16, right: u16) -> String {
    let body = format!("FINNCFG,WAS,{},{},{}", centre, left, right);
    format_finn_sentence(&body)
}

/// Format a PID config command.
/// kp_x100 = kp_angle * 100 (e.g. 1000 = 10.0 PWM/degree)
pub fn format_pid_config(kp_x100: u16, min_pwm: u16, max_pwm: u16) -> String {
    let body = format!("FINNCFG,PID,{},{},{}", kp_x100, min_pwm, max_pwm);
    format_finn_sentence(&body)
}

/// Format a motor invert config command.
pub fn format_invert_config(invert: bool) -> String {
    let body = format!("FINNCFG,INVERT,{}", if invert { 1 } else { 0 });
    format_finn_sentence(&body)
}

/// Format a live WAS centre update command.
///
/// Updates the ESP32's working `wasCentre` value in RAM only. Does NOT
/// write to NVS, and does NOT reset the EMA smoother state. Intended for
/// continuous fine adjustment from the PC's auto-centre learner during
/// auto-steer engagement, where we want to track thermal drift in the
/// WAS pot without grinding through flash write cycles.
///
/// The manual three-point calibration (`format_was_config`) remains the
/// source of truth on power-up; this command applies a session-only
/// adjustment on top of it.
pub fn format_was_centre_live(centre: u16) -> String {
    let body = format!("FINNCFG,WASCNT,{}", centre);
    format_finn_sentence(&body)
}

/// Format a WAS filtering config command.
/// All values are x100 integers:
///   ema_alpha_x100:  EMA smoothing factor (e.g. 15 = 0.15)
///   deadzone_x100:   Dead zone half-width in degrees (e.g. 200 = 2.00°)
///   curve_exp_x100:  Non-linear curve exponent (e.g. 200 = 2.00)
pub fn format_wasf_config(ema_alpha_x100: u16, deadzone_x100: u16, curve_exp_x100: u16) -> String {
    let body = format!(
        "FINNCFG,WASF,{},{},{}",
        ema_alpha_x100, deadzone_x100, curve_exp_x100
    );
    format_finn_sentence(&body)
}
