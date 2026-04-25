//! Serial communication with the ESP32 motor controller.
//!
//! Decision #026: The motor ESP32 is the sole microcontroller. It receives
//! desired steering angles (not raw PWM) and runs the inner loop locally.
//! It also reads the WAS and reports status including WAS feedback.
//!
//! Auto-detection scans for ports sending $FINNMTR sentences, excluding
//! the GPS port (which sends $GNGGA/$GNVTG from the LC29H BA).
//!
//! Decision #027: channel sends to GUI / steer thread are non-blocking
//! (`try_send`) — a slow consumer will lose a motor status update rather
//! than backpressure the whole reader thread. Drop counters are logged
//! every ~5s so consumer stalls are visible in field logs.

use std::io::{BufRead, BufReader, Write};
use std::sync::{Arc, Mutex};
use std::sync::atomic::Ordering;
use std::time::Duration;
use crossbeam_channel::Sender;
use serialport::{self, SerialPortType};
use finn_guidance_common::protocol::{self, FinnMessage};
use crate::gps::finn_parser;
use crate::telemetry::SharedDropCounters;

/// Thread-safe handle for sending commands to the motor ESP32.
/// Cloneable — the GUI thread holds one, the motor reader thread holds another.
#[derive(Clone)]
pub struct MotorHandle {
    port: Arc<Mutex<Option<Box<dyn serialport::SerialPort + Send>>>>,
}

impl MotorHandle {
    pub fn new() -> Self {
        Self {
            port: Arc::new(Mutex::new(None)),
        }
    }

    /// Send a desired steering angle to the motor ESP32.
    /// angle_deg is the desired wheel angle in degrees (positive = right).
    /// It is transmitted as angle * 100 (integer centidegrees).
    pub fn send_steer_angle(&self, angle_deg: f64) -> Result<(), String> {
        let mut guard = self.port.lock().map_err(|e| e.to_string())?;
        let port = guard.as_mut().ok_or("Motor serial port not open")?;
        let angle_x100 = (angle_deg * 100.0).round() as i16;
        let cmd = protocol::format_steer_command(angle_x100);
        port.write_all(cmd.as_bytes()).map_err(|e| e.to_string())?;
        Ok(())
    }

    /// Send a raw FINN sentence string to the motor ESP32.
    /// Used for config commands ($FINNCFG).
    pub fn send_raw(&self, sentence: &str) -> Result<(), String> {
        let mut guard = self.port.lock().map_err(|e| e.to_string())?;
        let port = guard.as_mut().ok_or("Motor serial port not open")?;
        port.write_all(sentence.as_bytes()).map_err(|e| e.to_string())?;
        Ok(())
    }

    /// Send WAS calibration values to the ESP32 NVS.
    pub fn send_was_config(&self, centre: u16, left: u16, right: u16) -> Result<(), String> {
        let cmd = protocol::format_was_config(centre, left, right);
        self.send_raw(&cmd)
    }

    /// Send PID tuning parameters to the ESP32 NVS.
    pub fn send_pid_config(&self, kp_x100: u16, min_pwm: u16, max_pwm: u16) -> Result<(), String> {
        let cmd = protocol::format_pid_config(kp_x100, min_pwm, max_pwm);
        self.send_raw(&cmd)
    }

    /// Send motor invert flag to the ESP32 NVS.
    pub fn send_invert_config(&self, invert: bool) -> Result<(), String> {
        let cmd = protocol::format_invert_config(invert);
        self.send_raw(&cmd)
    }

    /// Check if the motor port is connected.
    pub fn is_connected(&self) -> bool {
        self.port.lock().map(|g| g.is_some()).unwrap_or(false)
    }
}

/// Auto-detect the motor ESP32 serial port.
///
/// Scans USB serial ports, skipping `exclude_port` (the GPS module).
/// Looks for $FINNMTR sentences to identify the motor controller.
fn auto_detect_motor_port(baud_rate: u32, exclude_port: &str) -> Option<String> {
    let ports = serialport::available_ports().ok()?;

    tracing::info!("Scanning for motor ESP32 (excluding {})...", exclude_port);

    // USB ports first
    let mut sorted = ports;
    sorted.sort_by_key(|p| match &p.port_type {
        SerialPortType::UsbPort(_) => 0,
        _ => 1,
    });

    for port_info in &sorted {
        let name = &port_info.port_name;

        // Skip the GPS module's port
        if name == exclude_port {
            tracing::debug!("  Skipping {} (GPS port)", name);
            continue;
        }

        tracing::info!("  Probing {} for motor ESP32...", name);

        let port = match serialport::new(name, baud_rate)
            .timeout(Duration::from_millis(500))
            .open()
        {
            Ok(p) => p,
            Err(e) => {
                tracing::debug!("    Could not open {}: {}", name, e);
                continue;
            }
        };

        let reader = BufReader::new(port);

        for line in reader.lines().take(30) {
            if let Ok(sentence) = line {
                if sentence.starts_with("$FINNMTR") {
                    tracing::info!("    Found motor ESP32 on {} — {}", name, &sentence[..sentence.len().min(60)]);
                    return Some(name.clone());
                }
            }
        }

        tracing::debug!("    No $FINNMTR on {}", name);
    }

    None
}

/// Start the motor serial reader loop. Blocks — call from a dedicated thread.
///
/// Opens the motor ESP32's COM port, reads $FINNMTR and $FINNACK messages,
/// and sends them to the GUI via `finn_tx`. Also stores the write half
/// of the port in `handle` so the GUI can send steer and config commands.
///
/// Decision #027: channel sends are non-blocking — if a consumer falls
/// behind, motor messages are dropped rather than backpressuring the
/// reader (which would also stall the second consumer via shared thread).
pub fn run_motor_reader(
    gps_port: String,
    baud_rate: u32,
    handle: MotorHandle,
    finn_tx: Sender<FinnMessage>,
    finn_tx_steer: Sender<FinnMessage>,
    drop_counters: SharedDropCounters,
) {
    // Brief delay to let the GPS reader claim its port first
    std::thread::sleep(Duration::from_secs(2));

    let port_name = match auto_detect_motor_port(baud_rate, &gps_port) {
        Some(name) => name,
        None => {
            tracing::warn!("Motor ESP32 not found — steering commands unavailable");
            return;
        }
    };

    tracing::info!("Opening motor ESP32 on {} at {} baud", port_name, baud_rate);

    let port = match serialport::new(&port_name, baud_rate)
        .timeout(Duration::from_millis(1000))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            tracing::error!("Failed to open motor port: {}", e);
            return;
        }
    };

    // Clone the port for writing (steer + config commands) and store in handle
    let write_port = port.try_clone().expect("Failed to clone motor serial port");
    {
        let mut guard = handle.port.lock().unwrap();
        *guard = Some(write_port);
    }
    tracing::info!("Motor ESP32 connected — steer and config commands available");

    // Read loop — parse $FINNMTR and $FINNACK, forward to GUI + steer thread.
    // Decision #027: drop-on-full to prevent consumer stalls cascading back
    // through this reader thread. Drop counts tracked via shared atomics
    // for telemetry, plus local counters for the tracing WARN log.
    let reader = BufReader::new(port);
    let mut line_count: u64 = 0;
    let mut local_gui_drops: u64 = 0;
    let mut local_steer_drops: u64 = 0;
    let mut gui_closed = false;
    let mut steer_closed = false;
    let mut last_drop_log = std::time::Instant::now();

    for line in reader.lines() {
        match line {
            Ok(sentence) => {
                line_count += 1;
                if line_count <= 5 {
                    tracing::info!("Motor serial [{}]: {}", line_count, sentence);
                }

                if sentence.starts_with("$FINN") {
                    if let Some(msg) = finn_parser::parse_finn_sentence(&sentence) {
                        // Send to GUI channel (drop if full).
                        match finn_tx.try_send(msg.clone()) {
                            Ok(()) => {}
                            Err(crossbeam_channel::TrySendError::Full(_)) => {
                                drop_counters.mtr_gui.fetch_add(1, Ordering::Relaxed);
                                local_gui_drops += 1;
                            }
                            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                                gui_closed = true;
                            }
                        }

                        // Send to steer thread channel (drop if full).
                        match finn_tx_steer.try_send(msg) {
                            Ok(()) => {}
                            Err(crossbeam_channel::TrySendError::Full(_)) => {
                                drop_counters.mtr_steer.fetch_add(1, Ordering::Relaxed);
                                local_steer_drops += 1;
                            }
                            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                                steer_closed = true;
                            }
                        }

                        if gui_closed && steer_closed {
                            tracing::warn!("All FINN channels closed, stopping motor reader");
                            return;
                        }

                        // Log drop counters periodically.
                        if last_drop_log.elapsed() >= Duration::from_secs(5) {
                            if local_gui_drops > 0 || local_steer_drops > 0 {
                                tracing::warn!(
                                    "Motor msg drops in last ~5s: gui={} steer={} (consumer stalled)",
                                    local_gui_drops, local_steer_drops
                                );
                                local_gui_drops = 0;
                                local_steer_drops = 0;
                            }
                            last_drop_log = std::time::Instant::now();
                        }
                    }
                }
            }
            Err(e) => {
                tracing::warn!("Motor serial read error: {}", e);
            }
        }
    }

    tracing::warn!("Motor reader loop ended (port closed?)");
}
