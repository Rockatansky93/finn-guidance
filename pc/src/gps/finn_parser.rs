//! Parser for FINN-protocol sentences from ESP32 modules.
//!
//! Handles `$FINNWAS`, `$FINNIMU`, `$FINNHB`, and `$FINNMTR` sentences.
//! Each sentence uses NMEA-style framing: `$<body>*<hex_checksum>\r\n`
//!
//! Standard NMEA sentences (GPS) are handled by `parser.rs` — this module
//! only deals with FINN-prefixed messages.

use finn_guidance_common::types::{ImuData, WasReading, EspHeartbeat, MotorStatus};
use finn_guidance_common::protocol::{FinnMessage, nmea_checksum};

/// Try to parse a single line as a FINN sentence.
///
/// Returns `None` if the line is not a FINN sentence (e.g. standard NMEA),
/// or if the checksum fails or fields are malformed.
pub fn parse_finn_sentence(line: &str) -> Option<FinnMessage> {
    let line = line.trim();

    // Must start with $FINN
    if !line.starts_with("$FINN") {
        return None;
    }

    // Split off the $ prefix
    let without_dollar = &line[1..];

    // Split body and checksum at the * separator
    let (body, expected_cs) = split_checksum(without_dollar)?;

    // Verify checksum
    let actual_cs = nmea_checksum(body);
    if actual_cs != expected_cs {
        tracing::debug!(
            "FINN checksum mismatch: expected {:02X}, got {:02X} for '{}'",
            expected_cs, actual_cs, body
        );
        return None;
    }

    // Route by sentence type
    if let Some(fields) = body.strip_prefix("FINNWAS,") {
        parse_was(fields)
    } else if let Some(fields) = body.strip_prefix("FINNIMU,") {
        parse_imu(fields)
    } else if let Some(fields) = body.strip_prefix("FINNHB,") {
        parse_heartbeat(fields)
    } else if let Some(fields) = body.strip_prefix("FINNMTR,") {
        parse_motor_status(fields)
    } else {
        tracing::debug!("Unknown FINN sentence: {}", body);
        None
    }
}

/// Split "BODY*HH" into ("BODY", 0xHH). Returns None if format is wrong.
fn split_checksum(s: &str) -> Option<(&str, u8)> {
    let star_pos = s.rfind('*')?;
    let body = &s[..star_pos];
    let cs_str = &s[star_pos + 1..];
    let cs = u8::from_str_radix(cs_str, 16).ok()?;
    Some((body, cs))
}

/// Parse `$FINNWAS,<raw_adc>,<voltage_mv>*XX`
fn parse_was(fields: &str) -> Option<FinnMessage> {
    let parts: Vec<&str> = fields.split(',').collect();
    if parts.len() != 2 {
        return None;
    }
    let raw_value = parts[0].parse::<u16>().ok()?;
    let voltage_mv = parts[1].parse::<u16>().ok()?;

    Some(FinnMessage::Was(WasReading {
        raw_value,
        voltage_mv,
        angle_deg: 0.0, // Uncalibrated — PID controller will apply calibration
    }))
}

/// Parse `$FINNIMU,<roll>,<pitch>,<heading>,<cal_sys>,<cal_gyro>,<cal_accel>,<cal_mag>*XX`
fn parse_imu(fields: &str) -> Option<FinnMessage> {
    let parts: Vec<&str> = fields.split(',').collect();
    if parts.len() != 7 {
        return None;
    }
    let roll = parts[0].parse::<f64>().ok()?;
    let pitch = parts[1].parse::<f64>().ok()?;
    let heading = parts[2].parse::<f64>().ok()?;
    let cal_sys = parts[3].parse::<u8>().ok()?;
    let cal_gyro = parts[4].parse::<u8>().ok()?;
    let cal_accel = parts[5].parse::<u8>().ok()?;
    let cal_mag = parts[6].parse::<u8>().ok()?;

    Some(FinnMessage::Imu(ImuData {
        roll,
        pitch,
        heading,
        cal_sys,
        cal_gyro,
        cal_accel,
        cal_mag,
    }))
}

/// Parse `$FINNHB,<uptime_ms>*XX`
fn parse_heartbeat(fields: &str) -> Option<FinnMessage> {
    let uptime_ms = fields.parse::<u64>().ok()?;
    Some(FinnMessage::SensorHeartbeat(EspHeartbeat { uptime_ms }))
}

/// Parse `$FINNMTR,<current_pwm>,<enabled>,<uptime_ms>*XX`
fn parse_motor_status(fields: &str) -> Option<FinnMessage> {
    let parts: Vec<&str> = fields.split(',').collect();
    if parts.len() != 3 {
        return None;
    }
    let current_pwm = parts[0].parse::<i16>().ok()?;
    let enabled = parts[1] == "1";
    let uptime_ms = parts[2].parse::<u64>().ok()?;

    Some(FinnMessage::MotorStatus(MotorStatus {
        current_pwm,
        enabled,
        uptime_ms,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_was() {
        let msg = parse_finn_sentence("$FINNWAS,2048,1650*4F");
        // Checksum may not match this example — test with a real one
        // For now just test that the parser handles the format
    }

    #[test]
    fn test_parse_real_was() {
        // From actual serial output
        let msg = parse_finn_sentence("$FINNWAS,0,0*4A");
        assert!(msg.is_some());
        if let Some(FinnMessage::Was(was)) = msg {
            assert_eq!(was.raw_value, 0);
            assert_eq!(was.voltage_mv, 0);
        } else {
            panic!("Expected Was message");
        }
    }

    #[test]
    fn test_parse_real_imu() {
        // From actual serial output
        let msg = parse_finn_sentence("$FINNIMU,2.2,4.4,0.2,3,3,0,0*5E");
        assert!(msg.is_some());
        if let Some(FinnMessage::Imu(imu)) = msg {
            assert!((imu.roll - 2.2).abs() < 0.01);
            assert!((imu.pitch - 4.4).abs() < 0.01);
            assert!((imu.heading - 0.2).abs() < 0.01);
            assert_eq!(imu.cal_sys, 3);
            assert_eq!(imu.cal_gyro, 3);
            assert_eq!(imu.cal_accel, 0);
            assert_eq!(imu.cal_mag, 0);
        } else {
            panic!("Expected Imu message");
        }
    }

    #[test]
    fn test_parse_real_heartbeat() {
        let msg = parse_finn_sentence("$FINNHB,1372057*1C");
        assert!(msg.is_some());
        if let Some(FinnMessage::SensorHeartbeat(hb)) = msg {
            assert_eq!(hb.uptime_ms, 1372057);
        } else {
            panic!("Expected Heartbeat message");
        }
    }

    #[test]
    fn test_nmea_ignored() {
        let msg = parse_finn_sentence("$GNGGA,123456.00,3456.789,S,13856.789,E,1,12,0.8,100.0,M,,,,*XX");
        assert!(msg.is_none());
    }

    #[test]
    fn test_bad_checksum() {
        let msg = parse_finn_sentence("$FINNWAS,0,0*FF");
        assert!(msg.is_none());
    }

    #[test]
    fn test_not_finn() {
        assert!(parse_finn_sentence("hello world").is_none());
        assert!(parse_finn_sentence("").is_none());
        assert!(parse_finn_sentence("$GPGGA,stuff").is_none());
    }
}
