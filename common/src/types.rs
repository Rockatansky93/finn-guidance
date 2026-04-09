use serde::{Deserialize, Serialize};

/// GPS fix quality levels
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum FixQuality {
    NoFix,
    Gps,       // Standard GPS
    Dgps,      // Differential GPS
    Rtk,       // RTK Fixed - centimetre accuracy
    RtkFloat,  // RTK Float - decimetre accuracy
}

/// A GPS position fix with all relevant data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsFix {
    pub latitude: f64,       // Degrees (negative = south)
    pub longitude: f64,      // Degrees (negative = west)
    pub altitude: f64,       // Metres above sea level
    pub speed: f64,          // m/s
    pub heading: f64,        // Degrees from true north (0-360)
    pub fix_quality: FixQuality,
    pub satellites: u8,
    pub hdop: f64,           // Horizontal dilution of precision
    pub timestamp_ms: u64,   // Milliseconds since epoch
}

/// IMU orientation data from BNO055
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuData {
    pub roll: f64,    // Degrees, positive = right side down
    pub pitch: f64,   // Degrees, positive = nose up
    pub heading: f64, // Degrees from magnetic north
}

/// Wheel angle sensor reading
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WasReading {
    pub raw_value: u16,   // Raw ADC value (0-4095 on ESP32)
    pub angle_deg: f64,   // Calibrated angle in degrees (negative = left, positive = right)
}

/// Combined vehicle state used for guidance calculations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleState {
    pub position: GpsFix,
    pub imu: ImuData,
    pub wheel_angle: WasReading,
}

/// Guidance line types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum GuidanceLine {
    AbLine {
        a: (f64, f64), // (lat, lon) of point A
        b: (f64, f64), // (lat, lon) of point B
    },
    AbCurve {
        points: Vec<(f64, f64)>, // Series of (lat, lon) waypoints
    },
}

/// Cross-track error: how far off the guidance line we are
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CrossTrackError {
    pub distance_m: f64,   // Metres from line (negative = left, positive = right)
    pub heading_error: f64, // Degrees difference from desired heading
}
