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

use finn_guidance_common::coords;
use finn_guidance_common::types::{DrCalState, FixQuality, GpsFix};
use nmea::sentences::FixType;
use nmea::Nmea;

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

    // === Heading offset calibration ===
    /// User-configurable heading offset in degrees, added to the heading
    /// before it enters the fix. Corrects for GPS antenna/module mounting
    /// misalignment. Positive = clockwise correction (rotate heading right).
    pub heading_offset_deg: f64,

    // === Roll/pitch from PQTMINS (for antenna roll correction) ===
    /// EMA-smoothed roll in degrees (positive = right side down).
    /// Smoothing eliminates cab bounce noise from the raw DR roll.
    smoothed_roll: f64,
    /// Raw pitch from last PQTMINS (positive = nose up).
    last_pitch: f64,
    /// Whether we've received at least one PQTMINS with valid roll/pitch.
    has_ins_attitude: bool,
    /// EMA alpha for roll smoothing (0..1, lower = smoother).
    /// Default 0.15 gives ~1s settling at 10Hz updates — enough to
    /// remove cab bounce without lagging slope transitions.
    roll_ema_alpha: f64,
    /// Antenna height above ground in metres. Used to compute the lateral
    /// offset from roll: offset = antenna_height * sin(roll). Set via GUI.
    pub antenna_height_m: f64,

    // === Diagnostic heading sources (for GUI comparison display) ===
    /// Last raw VTG heading before offset applied (NaN if no VTG received)
    pub last_vtg_heading: f64,
    /// Last raw PQTMINS heading before offset applied (NaN if no INS received)
    pub last_ins_heading: f64,
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
            heading_offset_deg: 0.0,
            smoothed_roll: 0.0,
            last_pitch: 0.0,
            has_ins_attitude: false,
            roll_ema_alpha: 0.15,
            antenna_height_m: 0.0,
            last_vtg_heading: f64::NAN,
            last_ins_heading: f64::NAN,
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
            if let Ok(heading) = parts[1].parse::<f64>() {
                self.last_vtg_heading = heading;
                if !self.heading_from_ins {
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

    /// Parse $PQTMINS sentence — DR-fused attitude/velocity from LC29H BA.
    ///
    /// Confirmed NR11 two-wheel firmware format:
    /// `$PQTMINS,<Timestamp>,<SolType>,<Lat>,<Lon>,<Height>,
    /// <VEL_N>,<VEL_E>,<VEL_D>,<Roll>,<Pitch>,<Heading>*<checksum>`
    ///
    /// Position still comes from GGA. PQTMINS is used for roll/pitch, heading
    /// when available, and scalar speed derived from north/east velocity.
    fn parse_pqtmins(&mut self, sentence: &str) {
        // Strip checksum for parsing
        let body = if let Some(star) = sentence.find('*') {
            &sentence[..star]
        } else {
            sentence
        };

        let parts: Vec<&str> = body.split(',').collect();
        // Need: $PQTMINS + 11 fields through Heading.
        if parts.len() < 12 {
            return;
        }

        // Field 2 (index 2): SolType.
        // 0 = DR not ready, roll/pitch ready only
        // 1 = DR not ready, GNSS + roll/pitch + relative heading ready
        // 2 = GNSS + DR calibrated
        // 3 = DR only
        let sol_type: u8 = parts[2].parse().unwrap_or(0);

        // Fields 6/7: north/east velocity in m/s. PQTMINS has no scalar speed.
        if let (Ok(vel_n), Ok(vel_e)) = (parts[6].parse::<f64>(), parts[7].parse::<f64>()) {
            self.last_speed = (vel_n * vel_n + vel_e * vel_e).sqrt();
        }

        // Fields 9/10: roll/pitch in degrees. Do not bail on SolType=0:
        // that state can still provide useful roll/pitch before heading is ready.
        if let Ok(roll) = parts[9].parse::<f64>() {
            if !self.has_ins_attitude {
                // First sample — seed the EMA directly (no smoothing on init)
                self.smoothed_roll = roll;
                self.has_ins_attitude = true;
            } else {
                // EMA: smoothed = alpha * new + (1 - alpha) * previous
                self.smoothed_roll =
                    self.roll_ema_alpha * roll + (1.0 - self.roll_ema_alpha) * self.smoothed_roll;
            }
        }

        if let Ok(pitch) = parts[10].parse::<f64>() {
            self.last_pitch = pitch;
        }

        // Field 11: heading in degrees. SolType=0 does not have a reliable
        // heading yet, so keep VTG/current heading until SolType >= 1.
        if sol_type >= 1 {
            if let Ok(heading) = parts[11].parse::<f64>() {
                self.last_ins_heading = heading;
                self.last_heading = heading;
                self.heading_from_ins = true;
            }
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

        // Apply heading offset calibration. This corrects for GPS
        // antenna/module mounting misalignment. The offset is added
        // to the raw heading so all downstream consumers (guidance,
        // steering, field view) see the corrected value.
        let corrected_heading = normalise_heading(self.last_heading + self.heading_offset_deg);

        // === Roll correction: shift antenna position to ground-truth ===
        // The GPS antenna is mounted on the cab roof at `antenna_height_m`
        // above the axle. When the tractor rolls, the antenna swings
        // laterally by `antenna_height * sin(roll)`. We subtract this
        // offset to get the position at ground/axle level.
        //
        // Convention: positive roll = right side down. When the right
        // side is down, the antenna moves LEFT relative to the heading.
        // We correct by shifting the position RIGHT (perpendicular
        // clockwise from heading), i.e. bearing = heading + 90°.
        let (corrected_lat, corrected_lon) = if self.has_ins_attitude
            && self.antenna_height_m > 0.0
            && self.smoothed_roll.abs() > 0.1
        // Skip tiny roll (< 0.1°)
        {
            let lateral_offset_m = self.antenna_height_m * self.smoothed_roll.to_radians().sin();
            // Shift perpendicular to heading: heading + 90° = rightward.
            // lateral_offset_m is positive when roll is positive (right down),
            // meaning the antenna moved left, so we correct rightward.
            let correction_bearing = normalise_heading(corrected_heading + 90.0);
            apply_offset(lat, lon, correction_bearing, lateral_offset_m)
        } else {
            (lat, lon)
        };

        let fix = GpsFix {
            latitude: corrected_lat,
            longitude: corrected_lon,
            altitude: self.nmea.altitude.unwrap_or(0.0) as f64,
            speed: self.last_speed,
            heading: corrected_heading,
            fix_quality,
            satellites: self.nmea.num_of_fix_satellites.unwrap_or(0) as u8,
            hdop: self.nmea.hdop.unwrap_or(99.9) as f64,
            timestamp_ms: now_ms,
            roll: self.smoothed_roll,
            pitch: self.last_pitch,
            diag_vtg_heading: self.last_vtg_heading,
            diag_ins_heading: self.last_ins_heading,
        };

        // Reset the INS heading flag after building the fix —
        // next epoch starts fresh, VTG will be used unless a new
        // PQTMINS arrives before the next GGA.
        self.heading_from_ins = false;

        Some(fix)
    }
}

/// Normalise a heading to 0..360 range
fn normalise_heading(mut heading: f64) -> f64 {
    while heading >= 360.0 {
        heading -= 360.0;
    }
    while heading < 0.0 {
        heading += 360.0;
    }
    heading
}

/// Apply a lateral offset to a lat/lon position along a given bearing.
/// Uses the same spherical earth model as the rest of our coord math.
/// Returns (new_lat, new_lon) in degrees.
fn apply_offset(lat: f64, lon: f64, bearing_deg: f64, distance_m: f64) -> (f64, f64) {
    const EARTH_RADIUS: f64 = 6_371_000.0;

    let lat_r = coords::deg_to_rad(lat);
    let lon_r = coords::deg_to_rad(lon);
    let brg_r = coords::deg_to_rad(bearing_deg);
    let angular_dist = distance_m / EARTH_RADIUS;

    let new_lat_r =
        (lat_r.sin() * angular_dist.cos() + lat_r.cos() * angular_dist.sin() * brg_r.cos()).asin();

    let new_lon_r = lon_r
        + (brg_r.sin() * angular_dist.sin() * lat_r.cos())
            .atan2(angular_dist.cos() - lat_r.sin() * new_lat_r.sin());

    (coords::rad_to_deg(new_lat_r), coords::rad_to_deg(new_lon_r))
}

#[cfg(test)]
mod tests {
    use super::NmeaState;

    #[test]
    fn pqtmins_uses_confirmed_nr11_field_offsets() {
        let mut state = NmeaState::new();

        state.parse_pqtmins(
            "$PQTMINS,775312,1,-33.27565284,138.59023261,414.533028,\
             0.378698,-0.717217,0.024944,1.25,-2.50,336.60*00",
        );

        assert!((state.last_speed - 0.811).abs() < 0.001);
        assert!((state.smoothed_roll - 1.25).abs() < f64::EPSILON);
        assert!((state.last_pitch + 2.50).abs() < f64::EPSILON);
        assert!((state.last_ins_heading - 336.60).abs() < f64::EPSILON);
        assert!((state.last_heading - 336.60).abs() < f64::EPSILON);
        assert!(state.heading_from_ins);
        assert!(state.has_ins_attitude);
    }

    #[test]
    fn pqtmins_soltype_zero_keeps_roll_pitch_but_not_heading() {
        let mut state = NmeaState::new();
        state.last_heading = 123.4;

        state.parse_pqtmins(
            "$PQTMINS,775312,0,0.00000000,0.00000000,0.000000,\
             0.000000,0.000000,0.000000,3.00,4.00,270.00*00",
        );

        assert!((state.smoothed_roll - 3.00).abs() < f64::EPSILON);
        assert!((state.last_pitch - 4.00).abs() < f64::EPSILON);
        assert!((state.last_heading - 123.4).abs() < f64::EPSILON);
        assert!(state.last_ins_heading.is_nan());
        assert!(!state.heading_from_ins);
        assert!(state.has_ins_attitude);
    }
}
