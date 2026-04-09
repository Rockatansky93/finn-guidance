//! NMEA sentence parser - converts raw NMEA strings into GpsFix structs.
//!
//! We use the `nmea` crate for parsing but maintain our own state to combine
//! data from multiple sentence types (GGA for position, VTG for speed/heading).
//!
//! ## Epoch-based fix emission
//!
//! The Quectel lc29h sends multiple NMEA sentences per GPS epoch (GGA, VTG, GSA, etc.).
//! We only emit one GpsFix per epoch, triggered by the GGA sentence (which
//! contains the position). VTG data (speed/heading) is accumulated and included
//! in the next emitted fix.
//!
//! This means the channel receives exactly one fix per GPS update rate
//! (typically 1Hz for standard, up to 10Hz for LC29H in high-rate mode).

use nmea::Nmea;
use nmea::sentences::FixType;
use finn_guidance_common::types::{FixQuality, GpsFix};

pub struct NmeaState {
    nmea: Nmea,
    last_speed: f64,
    last_heading: f64,
    /// Track the last GGA timestamp to detect new epochs.
    /// We use the system time at which we received a GGA fix as the epoch marker,
    /// since the nmea crate doesn't expose the raw GGA time field easily.
    last_gga_time_ms: u64,
}

impl NmeaState {
    pub fn new() -> Self {
        Self {
            nmea: Nmea::default(),
            last_speed: 0.0,
            last_heading: 0.0,
            last_gga_time_ms: 0,
        }
    }

    /// Parse a single NMEA sentence. Returns a GpsFix only on GGA sentences
    /// (position updates), ensuring one fix per GPS epoch.
    pub fn parse_sentence(&mut self, sentence: &str) -> Option<GpsFix> {
        // Extract speed/heading from VTG sentences before passing to nmea crate
        if sentence.contains("VTG") {
            self.parse_vtg(sentence);
            // Parse into nmea state but don't emit a fix for VTG
            let _ = self.nmea.parse(sentence);
            return None;
        }

        // For GGA sentences, parse and emit a fix
        let is_gga = sentence.contains("GGA");

        match self.nmea.parse(sentence) {
            Ok(_) => {
                if is_gga {
                    self.try_build_fix()
                } else {
                    // Non-GGA, non-VTG sentences (GSA, GSV, etc.) update
                    // nmea state but don't emit a fix
                    None
                }
            }
            Err(_) => None,
        }
    }

    /// Extract speed and heading from VTG sentence manually
    /// (the nmea crate handles GGA well but VTG needs help)
    fn parse_vtg(&mut self, sentence: &str) {
        let parts: Vec<&str> = sentence.split(',').collect();
        if parts.len() >= 8 {
            // Field 1: heading (true north)
            if let Ok(heading) = parts[1].parse::<f64>() {
                self.last_heading = heading;
            }
            // Field 7: speed in km/h
            if let Ok(speed_kmh) = parts[7].parse::<f64>() {
                self.last_speed = speed_kmh / 3.6; // Convert to m/s
            }
        }
    }

    /// Try to build a GpsFix from current nmea state
    fn try_build_fix(&mut self) -> Option<GpsFix> {
        let lat = self.nmea.latitude?;
        let lon = self.nmea.longitude?;

        let fix_quality = match self.nmea.fix_type {
            Some(FixType::Invalid) | None => FixQuality::NoFix,
            Some(FixType::Gps) => FixQuality::Gps,
            Some(FixType::DGps) => FixQuality::Dgps,
            Some(FixType::Rtk) => FixQuality::Rtk,
            Some(FixType::FloatRtk) => FixQuality::RtkFloat,
            Some(_) => FixQuality::Gps,
        };

        // Use system time but only advance once per GGA parse.
        // This ensures each fix gets a unique, monotonically increasing timestamp.
        let now_ms = chrono::Utc::now().timestamp_millis() as u64;
        self.last_gga_time_ms = now_ms;

        Some(GpsFix {
            latitude: lat,
            longitude: lon,
            altitude: self.nmea.altitude.unwrap_or(0.0) as f64,
            speed: self.last_speed,
            heading: self.last_heading,
            fix_quality,
            satellites: self.nmea.num_of_fix_satellites.unwrap_or(0) as u8,
            hdop: self.nmea.hdop.unwrap_or(99.9) as f64,
            timestamp_ms: now_ms,
        })
    }
}
