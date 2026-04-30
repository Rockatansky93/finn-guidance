//! Parser for FINN-protocol sentences from the motor ESP32.
//!
//! Decision #026: Only `$FINNMTR` and `$FINNACK` sentences are received.
//! The sensor ESP32 has been removed — WAS data now comes embedded in
//! the extended `$FINNMTR` status sentence from the motor controller.
//!
//! Each sentence uses NMEA-style framing: `$<body>*<hex_checksum>\r\n`
//!
//! Standard NMEA sentences (GPS) are handled by `parser.rs` — this module
//! only deals with FINN-prefixed messages.

use finn_guidance_common::protocol::{nmea_checksum, FinnMessage};
use finn_guidance_common::types::{ConfigAck, MotorStatus};

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
            expected_cs,
            actual_cs,
            body
        );
        return None;
    }

    // Route by sentence type
    if let Some(fields) = body.strip_prefix("FINNMTR,") {
        parse_motor_status(fields)
    } else if let Some(fields) = body.strip_prefix("FINNACK,") {
        parse_config_ack(fields)
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

/// Parse `$FINNMTR,<pwm>,<was_raw>,<angle_x100>,<enabled>,<uptime_ms>*XX`
///
/// Decision #026 extended format: includes WAS raw ADC and calibrated angle
/// so the PC can display steering feedback without a separate sensor port.
fn parse_motor_status(fields: &str) -> Option<FinnMessage> {
    let parts: Vec<&str> = fields.split(',').collect();
    if parts.len() != 5 {
        return None;
    }
    let current_pwm = parts[0].parse::<i16>().ok()?;
    let was_raw = parts[1].parse::<u16>().ok()?;
    let angle_x100 = parts[2].parse::<i16>().ok()?;
    let enabled = parts[3] == "1";
    let uptime_ms = parts[4].parse::<u64>().ok()?;

    Some(FinnMessage::MotorStatus(MotorStatus {
        current_pwm,
        was_raw,
        actual_angle: angle_x100 as f64 / 100.0,
        enabled,
        uptime_ms,
    }))
}

/// Parse `$FINNACK,<param>,<OK|ERR>*XX`
fn parse_config_ack(fields: &str) -> Option<FinnMessage> {
    let parts: Vec<&str> = fields.split(',').collect();
    if parts.len() != 2 {
        return None;
    }
    let param = parts[0].to_string();
    let success = parts[1] == "OK";

    Some(FinnMessage::ConfigAck(ConfigAck { param, success }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_motor_status() {
        // Build a valid sentence with correct checksum
        let body = "FINNMTR,120,1832,0,1,5000";
        let cs = nmea_checksum(body);
        let sentence = format!("${}*{:02X}", body, cs);

        let msg = parse_finn_sentence(&sentence);
        assert!(msg.is_some());
        if let Some(FinnMessage::MotorStatus(status)) = msg {
            assert_eq!(status.current_pwm, 120);
            assert_eq!(status.was_raw, 1832);
            assert!((status.actual_angle - 0.0).abs() < 0.01);
            assert!(status.enabled);
            assert_eq!(status.uptime_ms, 5000);
        } else {
            panic!("Expected MotorStatus message");
        }
    }

    #[test]
    fn test_parse_motor_status_with_angle() {
        let body = "FINNMTR,-100,1900,315,1,12000";
        let cs = nmea_checksum(body);
        let sentence = format!("${}*{:02X}", body, cs);

        let msg = parse_finn_sentence(&sentence);
        assert!(msg.is_some());
        if let Some(FinnMessage::MotorStatus(status)) = msg {
            assert_eq!(status.current_pwm, -100);
            assert_eq!(status.was_raw, 1900);
            assert!((status.actual_angle - 3.15).abs() < 0.01);
            assert!(status.enabled);
        } else {
            panic!("Expected MotorStatus message");
        }
    }

    #[test]
    fn test_parse_config_ack_ok() {
        let body = "FINNACK,WAS,OK";
        let cs = nmea_checksum(body);
        let sentence = format!("${}*{:02X}", body, cs);

        let msg = parse_finn_sentence(&sentence);
        assert!(msg.is_some());
        if let Some(FinnMessage::ConfigAck(ack)) = msg {
            assert_eq!(ack.param, "WAS");
            assert!(ack.success);
        } else {
            panic!("Expected ConfigAck message");
        }
    }

    #[test]
    fn test_parse_config_ack_err() {
        let body = "FINNACK,PID,ERR";
        let cs = nmea_checksum(body);
        let sentence = format!("${}*{:02X}", body, cs);

        let msg = parse_finn_sentence(&sentence);
        assert!(msg.is_some());
        if let Some(FinnMessage::ConfigAck(ack)) = msg {
            assert_eq!(ack.param, "PID");
            assert!(!ack.success);
        } else {
            panic!("Expected ConfigAck message");
        }
    }

    #[test]
    fn test_nmea_ignored() {
        let msg =
            parse_finn_sentence("$GNGGA,123456.00,3456.789,S,13856.789,E,1,12,0.8,100.0,M,,,,*XX");
        assert!(msg.is_none());
    }

    #[test]
    fn test_bad_checksum() {
        let msg = parse_finn_sentence("$FINNMTR,0,0,0,0,0*FF");
        assert!(msg.is_none());
    }

    #[test]
    fn test_not_finn() {
        assert!(parse_finn_sentence("hello world").is_none());
        assert!(parse_finn_sentence("").is_none());
        assert!(parse_finn_sentence("$GPGGA,stuff").is_none());
    }
}
