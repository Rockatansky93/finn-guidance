use serde::{Deserialize, Serialize};
use crate::types::*;

/// Messages sent from ESP32 to PC
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EspToPC {
    /// Periodic sensor report (sent at ~10Hz)
    SensorReport {
        imu: ImuData,
        wheel_angle: WasReading,
    },
    /// Heartbeat / alive signal
    Heartbeat {
        uptime_ms: u64,
    },
}

/// Messages sent from PC to ESP32
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PCToEsp {
    /// Steering command from PID controller
    SteerCommand {
        pwm_value: i16,  // -255 to 255 (negative = left, positive = right)
    },
    /// Request current sensor state
    RequestSensors,
    /// Calibration command for WAS
    CalibrateWas {
        centre_value: u16,  // ADC value when wheels are straight
        counts_per_degree: f64,
    },
    /// Heartbeat / alive signal
    Heartbeat,
}

/// Default UDP port for ESP32 -> PC communication
pub const ESP_TO_PC_PORT: u16 = 9500;

/// Default UDP port for PC -> ESP32 communication  
pub const PC_TO_ESP_PORT: u16 = 9501;

/// Default GPS serial baud rate (ZED-F9P default)
pub const GPS_BAUD_RATE: u32 = 115200;

/// Sensor report rate in Hz
pub const SENSOR_REPORT_HZ: u32 = 10;
