//! Serial port GPS reader - runs in its own thread, sends GpsFix via channel.
//!
//! ## Auto-detection
//!
//! If no port is specified, the reader scans all available serial ports looking
//! for one that emits NMEA sentences (specifically `$G` prefixed lines). This
//! allows the system to be deployed on any PC without manual COM port config.
//!
//! ## Module configuration (ensure config)
//!
//! On startup, the reader sends PAIR commands to the LC29H to:
//! - Disable unnecessary NMEA sentences (GLL, GSA, GSV, RMC) — we only use GGA + VTG
//! - Set the fix rate to 10Hz (100ms interval), falling back to 5Hz/2Hz/1Hz if rejected
//!
//! Sentences are disabled BEFORE setting the rate, because the research doc shows
//! that module CPU load from formatting ASCII can prevent high-rate fixes.
//! The rate command reads back the PAIR001 acknowledgement to verify acceptance.

use crossbeam_channel::Sender;
use serialport::{self, SerialPortType};
use std::io::{BufRead, Read, Write};
use std::io::BufReader;
use std::time::Duration;
use tracing;

use finn_guidance_common::types::GpsFix;
use super::parser;

/// Configuration for the GPS serial connection
pub struct GpsConfig {
    /// Serial port name. If "auto", the reader will scan for a GPS module.
    pub port_name: String,
    pub baud_rate: u32,
    /// Desired fix rate in Hz (1-10). Default 5.
    pub fix_rate_hz: u8,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            port_name: String::from("auto"),
            baud_rate: 115200,
            fix_rate_hz: 10,
        }
    }
}

/// Scan all available serial ports and return the first one that produces NMEA data.
///
/// Strategy: open each port at the configured baud rate, read for up to 2 seconds,
/// and check if any line starts with `$G` (NMEA sentence prefix for GPS constellations).
/// USB serial devices are tried first as they're the most likely GPS connection.
fn auto_detect_gps_port(baud_rate: u32) -> Option<String> {
    let ports = match serialport::available_ports() {
        Ok(p) => p,
        Err(e) => {
            tracing::error!("Failed to enumerate serial ports: {}", e);
            return None;
        }
    };

    if ports.is_empty() {
        tracing::warn!("No serial ports found on this system");
        return None;
    }

    tracing::info!("Scanning {} serial port(s) for GPS module...", ports.len());

    // Sort: USB ports first (most likely to be GPS), then others
    let mut sorted_ports = ports;
    sorted_ports.sort_by_key(|p| match &p.port_type {
        SerialPortType::UsbPort(_) => 0,
        _ => 1,
    });

    for port_info in &sorted_ports {
        let port_name = &port_info.port_name;
        let type_desc = match &port_info.port_type {
            SerialPortType::UsbPort(info) => {
                format!("USB ({})",
                    info.product.as_deref().unwrap_or("unknown"))
            }
            SerialPortType::PciPort => "PCI".to_string(),
            SerialPortType::BluetoothPort => "Bluetooth".to_string(),
            SerialPortType::Unknown => "Unknown".to_string(),
        };

        tracing::info!("  Probing {} [{}]...", port_name, type_desc);

        let port = match serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(500))
            .open()
        {
            Ok(p) => p,
            Err(e) => {
                tracing::debug!("    Could not open {}: {}", port_name, e);
                continue;
            }
        };

        let reader = BufReader::new(port);
        let mut found_nmea = false;

        // Read lines for up to ~2 seconds (4 attempts × 500ms timeout)
        for line in reader.lines().take(20) {
            match line {
                Ok(sentence) => {
                    if sentence.starts_with("$G") || sentence.starts_with("$PAIR")
                        || sentence.starts_with("$PQTM")
                    {
                        tracing::info!("    ✓ Found GPS on {} — NMEA: {}", port_name, &sentence[..sentence.len().min(40)]);
                        found_nmea = true;
                        break;
                    }
                }
                Err(_) => continue,
            }
        }

        if found_nmea {
            return Some(port_name.clone());
        }

        tracing::debug!("    No NMEA data on {}", port_name);
    }

    None
}

/// Send module configuration commands to ensure desired fix rate and minimal NMEA output.
///
/// Commands are idempotent — safe to send on every boot. Disables unnecessary
/// sentences FIRST (freeing CPU), then sets the fix rate. If the requested rate
/// is rejected by the module, falls back to lower rates until one is accepted.
fn ensure_module_config(port: &mut Box<dyn serialport::SerialPort>, fix_rate_hz: u8) {
    let interval_ms = 1000 / fix_rate_hz as u16;
    tracing::info!("Configuring GPS module: target {}Hz ({}ms interval)", fix_rate_hz, interval_ms);

    // Step 1: Disable all unnecessary NMEA sentences FIRST.
    // This frees up module CPU before we ask it to run at a higher rate.
    // PAIR062 format: $PAIR062,<sentence_type>,<enable>
    //   0=GGA, 1=GLL, 2=GSA, 3=GSV, 4=RMC, 5=VTG, 6=ZDA
    let disable_cmds = [
        ("GLL", format_pair_command("PAIR062,1,0")),
        ("GSA", format_pair_command("PAIR062,2,0")),
        ("GSV", format_pair_command("PAIR062,3,0")),
        ("RMC", format_pair_command("PAIR062,4,0")),
    ];

    for (name, cmd) in &disable_cmds {
        tracing::debug!("  Disabling {} sentence: {}", name, cmd.trim());
        if let Err(e) = port.write_all(cmd.as_bytes()) {
            tracing::warn!("  Failed to send disable {}: {}", name, e);
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    // Brief pause to let the module process all the disables
    let _ = port.flush();
    std::thread::sleep(Duration::from_millis(300));

    // Step 2: Set the fix rate. Try the requested rate, fall back if rejected.
    // We read back the PAIR001 acknowledgement to check if it was accepted.
    let rates_to_try: Vec<u8> = {
        let mut rates = vec![fix_rate_hz];
        // Add fallback rates in descending order
        for &r in &[10, 5, 2, 1] {
            if r < fix_rate_hz && !rates.contains(&r) {
                rates.push(r);
            }
        }
        rates
    };

    let mut rate_set = false;
    for rate in &rates_to_try {
        let interval = 1000u16 / *rate as u16;
        let cmd = format_pair_command(&format!("PAIR050,{}", interval));
        tracing::info!("  Trying {}Hz ({}ms): {}", rate, interval, cmd.trim());

        if let Err(e) = port.write_all(cmd.as_bytes()) {
            tracing::warn!("  Failed to send rate command: {}", e);
            continue;
        }
        let _ = port.flush();

        // Read response lines for up to 500ms looking for PAIR001,050 ack
        std::thread::sleep(Duration::from_millis(200));

        let mut buf = [0u8; 512];
        let mut response_data = String::new();

        // Try reading available data (non-blocking-ish via short timeout)
        let original_timeout = port.timeout();
        let _ = port.set_timeout(Duration::from_millis(300));

        loop {
            match port.read(&mut buf) {
                Ok(n) if n > 0 => {
                    response_data.push_str(&String::from_utf8_lossy(&buf[..n]));
                    if response_data.contains("PAIR001,050") {
                        break;
                    }
                }
                _ => break,
            }
        }

        let _ = port.set_timeout(original_timeout);

        // Check the ack: $PAIR001,050,0 = success, ,1 = unsupported, ,2 = invalid
        if let Some(ack_pos) = response_data.find("PAIR001,050,") {
            let result_char = response_data.as_bytes().get(ack_pos + 12);
            match result_char {
                Some(b'0') => {
                    tracing::info!("  ✓ Module accepted {}Hz", rate);
                    rate_set = true;
                    break;
                }
                Some(b'1') => {
                    tracing::warn!("  ✗ {}Hz unsupported by this module variant", rate);
                }
                Some(b'2') => {
                    tracing::warn!("  ✗ {}Hz rejected (invalid parameter)", rate);
                }
                Some(c) => {
                    tracing::warn!("  ? {}Hz unknown response code: {}", rate, *c as char);
                }
                None => {
                    tracing::warn!("  ? {}Hz ack truncated", rate);
                }
            }
        } else {
            // No ack received — might still have worked, log and continue
            tracing::info!("  ? No ack for {}Hz (may still be applied)", rate);
            // Assume it worked if we got no explicit rejection
            rate_set = true;
            break;
        }
    }

    if !rate_set {
        tracing::warn!("Could not set any fix rate — module may be running at default 1Hz");
    }

    tracing::info!("GPS module configuration complete");
}

/// Format a PAIR command with the correct NMEA checksum.
///
/// Input: "PAIR050,200" → Output: "$PAIR050,200*XX\r\n"
fn format_pair_command(body: &str) -> String {
    let checksum = body.bytes().fold(0u8, |acc, b| acc ^ b);
    format!("${}*{:02X}\r\n", body, checksum)
}

/// Start the GPS reader loop. Blocks - call from a dedicated thread.
pub fn run_gps_reader(config: GpsConfig, tx: Sender<GpsFix>) {
    // === Step 1: Resolve port name (auto-detect or use specified) ===
    let port_name = if config.port_name == "auto" {
        tracing::info!("GPS port set to 'auto' — scanning for GPS module...");
        match auto_detect_gps_port(config.baud_rate) {
            Some(name) => {
                tracing::info!("Auto-detected GPS on {}", name);
                name
            }
            None => {
                tracing::error!("No GPS module found on any serial port");
                return;
            }
        }
    } else {
        config.port_name.clone()
    };

    tracing::info!("Opening GPS on {} at {} baud", port_name, config.baud_rate);

    let mut port = match serialport::new(&port_name, config.baud_rate)
        .timeout(Duration::from_millis(1000))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            tracing::error!("Failed to open GPS port: {}", e);
            return;
        }
    };

    tracing::info!("GPS port opened successfully");

    // === Step 2: Configure module (disable unused sentences, set fix rate) ===
    ensure_module_config(&mut port, config.fix_rate_hz);

    let reader = BufReader::new(port);
    let mut nmea_parser = parser::NmeaState::new();
    let mut line_count: u64 = 0;

    for line in reader.lines() {
        match line {
            Ok(sentence) => {
                line_count += 1;
                // Log first 10 raw sentences so we can see what's coming through
                if line_count <= 10 {
                    tracing::info!("Raw NMEA [{}]: {}", line_count, sentence);
                }

                if let Some(fix) = nmea_parser.parse_sentence(&sentence) {
                    if line_count <= 20 {
                        tracing::info!(
                            "Got fix: lat={:.6} lon={:.6} sats={} quality={:?}",
                            fix.latitude, fix.longitude, fix.satellites, fix.fix_quality
                        );
                    }
                    if tx.send(fix).is_err() {
                        tracing::warn!("GPS channel closed, stopping reader");
                        return;
                    }
                }
            }
            Err(e) => {
                tracing::warn!("GPS read error: {}", e);
            }
        }
    }

    tracing::warn!("GPS reader loop ended (port closed?)");
}
