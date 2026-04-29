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
    pub heading: f64,        // Degrees from true north (0-360), with offset applied
    pub fix_quality: FixQuality,
    pub satellites: u8,
    pub hdop: f64,           // Horizontal dilution of precision
    pub timestamp_ms: u64,   // Milliseconds since epoch

    // === Roll/pitch from PQTMINS DR fusion (for roll correction) ===
    /// Vehicle roll in degrees (positive = right side down). EMA-smoothed.
    /// Used for lateral GPS antenna offset correction.
    #[serde(default = "f64_zero")]
    pub roll: f64,
    /// Vehicle pitch in degrees (positive = nose up).
    #[serde(default = "f64_zero")]
    pub pitch: f64,

    // === Diagnostic heading sources (for GUI comparison display) ===
    /// Raw VTG course-over-ground before offset (NaN if unavailable)
    #[serde(default = "f64_nan")]
    pub diag_vtg_heading: f64,
    /// Raw PQTMINS DR-fused heading before offset (NaN if unavailable)
    #[serde(default = "f64_nan")]
    pub diag_ins_heading: f64,
}

fn f64_nan() -> f64 { f64::NAN }
fn f64_zero() -> f64 { 0.0 }

/// IMU orientation data from BNO055
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImuData {
    pub roll: f64,    // Degrees, positive = right side down
    pub pitch: f64,   // Degrees, positive = nose up
    pub heading: f64, // Degrees from magnetic north
    pub cal_sys: u8,   // System calibration 0-3
    pub cal_gyro: u8,  // Gyro calibration 0-3
    pub cal_accel: u8, // Accelerometer calibration 0-3
    pub cal_mag: u8,   // Magnetometer calibration 0-3
}

/// Wheel angle sensor reading
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WasReading {
    pub raw_value: u16,   // Raw ADC value (0-4095 on ESP32)
    pub voltage_mv: u16,  // Millivolts (0-3300)
    pub angle_deg: f64,   // Calibrated angle in degrees (negative = left, positive = right)
}

/// ESP32 heartbeat
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EspHeartbeat {
    pub uptime_ms: u64,
}

/// Motor controller status (from motor ESP32, includes WAS feedback)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorStatus {
    pub current_pwm: i16,   // -255 to 255
    pub was_raw: u16,       // Raw ADC value from WAS pot
    pub actual_angle: f64,  // Calibrated angle in degrees (from ESP32)
    pub enabled: bool,
    pub uptime_ms: u64,
}

/// DR (dead reckoning) calibration state from LC29H BA
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DrCalState {
    Uncalibrated,  // CalState 0 — drive >3 m/s with turns to calibrate
    Calibrating,   // CalState 1 — calibration in progress
    Calibrated,    // CalState 2 — DR fully operational
}

/// Config acknowledgement from motor ESP32
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigAck {
    pub param: String,   // "WAS", "PID", "INVERT"
    pub success: bool,
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
