//! NMEA sentence parser - converts raw NMEA strings into GpsFix structs.
//!
//! We use the `nmea` crate for parsing but maintain our own state to combine
//! data from multiple sentence types (GGA for position, VTG for speed/heading).
//!
//! ## Heading sources (Decision #026)
//!
//! The LC29H BA has onboard IMU dead-reckoning fusion. When enabled via
//! $PQTMCFGEINSMSG, it outputs $PQTMINS sentences containing DR-fused
//! heading that is stable at low speed (unlike VTG which is derived from
//! position deltas and becomes noisy below ~2 m/s).
//!
//! Priority: PQTMINS heading > VTG heading.
//! VTG is kept as fallback for when DR is uncalibrated or unavailable.
//!
//! ## Epoch-based fix emission
//!
//! The Quectel lc29h sends multiple NMEA sentences per GPS epoch (GGA, VTG, etc.).
//! We only emit one GpsFix per epoch, triggered by the GGA sentence (which
//! contains the position). VTG/PQTMINS data is accumulated and included
//! in the next emitted fix.

use nmea::Nmea;
use nmea::sentences::FixType;
use finn_guidance_common::types::{DrCalState, FixQuality, GpsFix};

pub struct NmeaState {
    nmea: Nmea,
    last_speed: f64,
    last_heading: f64,
    /// Whether the last heading came from PQTMINS (DR-fused) rather than VTG.
    /// PQTMINS heading is preferred as it's stable at low speed.
    heading_from_ins: bool,
    /// DR calibration state from $PQTMDRCAL
    pub dr_cal_state: DrCalState,
    /// Track the last GGA timestamp to detect new epochs.
    last_gga_time_ms: u64,
}

impl NmeaState {
    pub fn new() -> Self {
        Self {
            nmea: Nmea::default(),
            last_speed: 0.0,
            last_heading: 0.0,
            heading_from_ins: false,
            dr_cal_state: DrCalState::Uncalibrated,
            last_gga_time_ms: 0,
        }
    }

    /// Parse a single NMEA sentence. Returns a GpsFix only on GGA sentences
    /// (position updates), ensuring one fix per GPS epoch.
    pub fn parse_sentence(&mut self, sentence: &str) -> Option<GpsFix> {
        // Parse PQTM proprietary sentences (DR-related)
        if sentence.starts_with("$PQTMINS") {
            self.parse_pqtmins(sentence);
            return None;
        }
        if sentence.starts_with("$PQTMDRCAL") {
            self.parse_pqtmdrcal(sentence);
            return None;
        }

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
                    None
                }
            }
            Err(_) => None,
        }
    }

    /// Extract speed and heading from VTG sentence manually
    /// (the nmea crate handles GGA well but VTG needs help).
    /// Only updates heading if we don't already have a PQTMINS heading
    /// for this epoch (INS heading is preferred over VTG).
    fn parse_vtg(&mut self, sentence: &str) {
        let parts: Vec<&str> = sentence.split(',').collect();
        if parts.len() >= 8 {
            // Field 1: heading (true north) — only use if no INS heading
            if !self.heading_from_ins {
                if let Ok(heading) = parts[1].parse::<f64>() {
                    self.last_heading = heading;
                }
            }
            // Field 7: speed in km/h — always use VTG speed as fallback
            // (PQTMINS speed overwrites if available)
            if let Ok(speed_kmh) = parts[7].parse::<f64>() {
                if !self.heading_from_ins {
                    self.last_speed = speed_kmh / 3.6;
                }
            }
        }
    }

    /// Parse $PQTMINS sentence — DR-fused navigation solution from LC29H BA.
    ///
    /// Format: $PQTMINS,<MsgVer>,<TOW>,<InsNavType>,<Lat>,<Lon>,<Alt>,
    ///         <AltMSL>,<Speed2D>,<Speed3D>,<Roll>,<Pitch>,<Heading>,
    ///         <HACC>,<HDOP>,<PDOP>,<NumSV>*<checksum>
    ///
    /// We extract heading and speed from this. The heading is gyro-stabilised
    /// and remains accurate at low speed where VTG heading degrades.
    fn parse_pqtmins(&mut self, sentence: &str) {
        // Strip checksum for parsing
        let body = if let Some(star) = sentence.find('*') {
            &sentence[..star]
        } else {
            sentence
        };

        let parts: Vec<&str> = body.split(',').collect();
        // Need at least 13 fields: $PQTMINS + MsgVer + TOW + InsNavType + Lat + Lon
        //   + Alt + AltMSL + Speed2D + Speed3D + Roll + Pitch + Heading = 13
        if parts.len() < 13 {
            return;
        }

        // Field 3 (index 3): InsNavType — check it's a valid solution
        // 0 = no solution, 1 = GNSS only, 2 = DR only, 3 = combined GNSS+DR
        let nav_type: u8 = parts[3].parse().unwrap_or(0);
        if nav_type == 0 {
            return; // No valid solution
        }

        // Field 8 (index 8): Speed2D in m/s
        if let Ok(speed) = parts[8].parse::<f64>() {
            self.last_speed = speed;
        }

        // Field 12 (index 12): Heading in degrees (0-360)
        if let Ok(heading) = parts[12].parse::<f64>() {
            self.last_heading = heading;
            self.heading_from_ins = true;
        }
    }

    /// Parse $PQTMDRCAL sentence — DR calibration state.
    ///
    /// Format: $PQTMDRCAL,<MsgVer>,<CalState>*<checksum>
    /// CalState: 0 = uncalibrated, 1 = calibrating, 2 = calibrated
    fn parse_pqtmdrcal(&mut self, sentence: &str) {
        let body = if let Some(star) = sentence.find('*') {
            &sentence[..star]
        } else {
            sentence
        };

        let parts: Vec<&str> = body.split(',').collect();
        if parts.len() >= 3 {
            match parts[2] {
                "0" => self.dr_cal_state = DrCalState::Uncalibrated,
                "1" => self.dr_cal_state = DrCalState::Calibrating,
                "2" => self.dr_cal_state = DrCalState::Calibrated,
                _ => {}
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

        let now_ms = chrono::Utc::now().timestamp_millis() as u64;
        self.last_gga_time_ms = now_ms;

        // Reset the INS heading flag after building the fix —
        // next epoch starts fresh, VTG will be used unless a new
        // PQTMINS arrives before the next GGA.
        let fix = GpsFix {
            latitude: lat,
            longitude: lon,
            altitude: self.nmea.altitude.unwrap_or(0.0) as f64,
            speed: self.last_speed,
            heading: self.last_heading,
            fix_quality,
            satellites: self.nmea.num_of_fix_satellites.unwrap_or(0) as u8,
            hdop: self.nmea.hdop.unwrap_or(99.9) as f64,
            timestamp_ms: now_ms,
        };

        self.heading_from_ins = false;

        Some(fix)
    }
}
