//! Serial communication with the ESP32 motor controller.
//!
//! Opens a second USB serial port (separate from the sensor ESP32) for
//! the motor controller. Sends $FINNSTEER commands and reads $FINNMTR status.
//!
//! The motor port is auto-detected by scanning USB serial ports that are
//! NOT already claimed by the sensor reader. The motor ESP32 identifies
//! itself by sending $FINNMTR sentences.

use std::io::{BufRead, BufReader, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use crossbeam_channel::Sender;
use serialport::{self, SerialPortType};
use finn_guidance_common::protocol::{self, FinnMessage};
use crate::gps::finn_parser;

/// Thread-safe handle for sending steer commands to the motor ESP32.
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

    /// Send a steer command. Returns Ok if sent, Err if port not open.
    pub fn send_steer(&self, pwm: i16) -> Result<(), String> {
        let mut guard = self.port.lock().map_err(|e| e.to_string())?;
        let port = guard.as_mut().ok_or("Motor serial port not open")?;
        let cmd = protocol::format_steer_command(pwm);
        port.write_all(cmd.as_bytes()).map_err(|e| e.to_string())?;
        Ok(())
    }

    /// Check if the motor port is connected.
    pub fn is_connected(&self) -> bool {
        self.port.lock().map(|g| g.is_some()).unwrap_or(false)
    }
}

/// Auto-detect the motor ESP32 serial port.
///
/// Scans USB serial ports, skipping `exclude_port` (the sensor ESP32).
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

        // Skip the sensor ESP32's port
        if name == exclude_port {
            tracing::debug!("  Skipping {} (sensor port)", name);
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
                    tracing::info!("    ✓ Found motor ESP32 on {} — {}", name, &sentence[..sentence.len().min(50)]);
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
/// Opens the motor ESP32's COM port, reads $FINNMTR status messages,
/// and sends them to the GUI via `finn_tx`. Also stores the write half
/// of the port in `handle` so the GUI can send $FINNSTEER commands.
pub fn run_motor_reader(
    sensor_port: String,
    baud_rate: u32,
    handle: MotorHandle,
    finn_tx: Sender<FinnMessage>,
) {
    // Brief delay to let the sensor reader claim its port first
    std::thread::sleep(Duration::from_secs(2));

    let port_name = match auto_detect_motor_port(baud_rate, &sensor_port) {
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

    // Clone the port for writing (steer commands) and store in handle
    let write_port = port.try_clone().expect("Failed to clone motor serial port");
    {
        let mut guard = handle.port.lock().unwrap();
        *guard = Some(write_port);
    }
    tracing::info!("Motor ESP32 connected — steer commands available");

    // Read loop — parse $FINNMTR status and forward to GUI
    let reader = BufReader::new(port);
    let mut line_count: u64 = 0;

    for line in reader.lines() {
        match line {
            Ok(sentence) => {
                line_count += 1;
                if line_count <= 5 {
                    tracing::info!("Motor serial [{}]: {}", line_count, sentence);
                }

                if sentence.starts_with("$FINN") {
                    if let Some(msg) = finn_parser::parse_finn_sentence(&sentence) {
                        if finn_tx.send(msg).is_err() {
                            tracing::warn!("FINN channel closed, stopping motor reader");
                            return;
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
