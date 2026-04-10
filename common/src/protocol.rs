//! Serial protocol definitions for FINN ESP32 ↔ PC communication.
//!
//! Both ESP32 modules communicate with the PC over USB serial using text-based
//! NMEA-style sentences with XOR checksums.
//!
//! ## Sensor ESP32 → PC
//! - `$FINNWAS,<raw_adc>,<voltage_mv>*<checksum>`      (20Hz)
//! - `$FINNIMU,<roll>,<pitch>,<heading>,<cal_sys>,<cal_gyro>,<cal_accel>,<cal_mag>*<checksum>` (20Hz)
//! - `$FINNHB,<uptime_ms>*<checksum>`                   (every 2s)
//! - Raw NMEA passthrough: `$GNGGA,...`, `$GNVTG,...`    (1Hz from LC29H DA)
//!
//! ## PC → Motor ESP32
//! - `$FINNSTEER,<pwm_value>*<checksum>`   (pwm: -255 to 255)
//!
//! ## Motor ESP32 → PC
//! - `$FINNMTR,<current_pwm>,<enabled>,<uptime_ms>*<checksum>`  (5Hz)

use crate::types::*;

/// All possible messages parsed from ESP32 serial streams
#[derive(Debug, Clone)]
pub enum FinnMessage {
    /// Wheel angle sensor reading (from sensor ESP32, 20Hz)
    Was(WasReading),
    /// IMU orientation + calibration (from sensor ESP32, 20Hz)
    Imu(ImuData),
    /// Sensor ESP32 heartbeat (every 2s)
    SensorHeartbeat(EspHeartbeat),
    /// Motor controller status (from motor ESP32, 5Hz)
    MotorStatus(MotorStatus),
}

/// GPS serial baud rate (LC29H DA default, also used by both ESP32s)
pub const SERIAL_BAUD_RATE: u32 = 115200;

/// Compute NMEA-style XOR checksum over a message body (between $ and *)
pub fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// Format a steer command for sending to the motor ESP32.
/// Returns a complete sentence including `$`, `*`, checksum, and `\r\n`.
///
/// # Example
/// ```
/// use finn_guidance_common::protocol::format_steer_command;
/// let cmd = format_steer_command(128);
/// assert!(cmd.starts_with("$FINNSTEER,128*"));
/// assert!(cmd.ends_with("\r\n"));
/// ```
pub fn format_steer_command(pwm: i16) -> String {
    let pwm = pwm.clamp(-255, 255);
    let body = format!("FINNSTEER,{}", pwm);
    let cs = nmea_checksum(&body);
    format!("${}*{:02X}\r\n", body, cs)
}
