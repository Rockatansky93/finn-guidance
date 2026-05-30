//! Serial port GPS reader - runs in its own thread, sends GpsFix via channel.
//!
//! ## Decision #026 architecture
//!
//! The LC29H BA GPS module connects directly to the laptop via USB serial.
//! No sensor ESP32 in the path — this reader talks to the GPS module directly.
//! The BA variant outputs at 10Hz with onboard IMU dead-reckoning fusion.
//!
//! ## Auto-detection
//!
//! Scans all available serial ports looking for NMEA sentences ($GNGGA, $GNVTG).
//! Skips any port sending $FINNMTR (that's the motor ESP32).
//!
//! ## Module configuration
//!
//! On startup, sends PAIR commands to:
//! - Disable unnecessary NMEA sentences (GLL, GSA, GSV, RMC)
//! - Set the fix rate to 10Hz (100ms interval)
//!
//! ## Decision #027 — drop-on-full channel sends
//!
//! Previously both channel sends used blocking `send()`, which coupled the
//! GPS reader thread to the slowest consumer. A GUI hiccup could fill
//! `gps_tx` (bounded 64), block the reader, and starve the steer thread of
//! fixes via the same blocked thread — producing cascading multi-second
//! freezes in the field. Guidance wants latest-value semantics, not backlog
//! catch-up, so we `try_send` and drop on full, logging drop counts so
//! consumer stalls are visible in field logs.

use crossbeam_channel::Sender;
use serialport::{self, SerialPortType};
use std::io::BufReader;
use std::io::{BufRead, Read, Write};
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tracing;

use super::parser;
use crate::telemetry::SharedDropCounters;
use finn_guidance_common::types::GpsFix;

/// Configuration for the GPS serial connection
pub struct GpsConfig {
    /// Serial port name. If "auto", the reader will scan for a GPS module.
    pub port_name: String,
    pub baud_rate: u32,
    /// Desired fix rate in Hz (1-10). Default 10 for LC29H BA.
    pub fix_rate_hz: u8,
    /// Initial heading offset in degrees. Applied to raw heading before
    /// emitting the fix. Positive = clockwise correction.
    pub heading_offset_deg: f64,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            port_name: String::from("auto"),
            baud_rate: 115200,
            fix_rate_hz: 10,
            heading_offset_deg: 0.0,
        }
    }
}

/// Shared atomic for the heading offset so the GUI can adjust it at runtime.
/// Stored as offset_deg × 100 (i.e. centidegrees) to fit in an AtomicI32.
/// Range: ±180° = ±18000.
pub type SharedHeadingOffset = Arc<AtomicI32>;

/// Shared atomic for the antenna height so the GUI can adjust it at runtime.
/// Stored as height_m × 100 (i.e. centimetres) to fit in an AtomicI32.
/// Range: 0–10m = 0–1000.
pub type SharedAntennaHeight = Arc<AtomicI32>;

/// Shared atomic for the roll mounting-bias offset so the GUI can capture
/// and adjust it at runtime. Stored as offset_deg × 100 (centidegrees) to
/// fit in an AtomicI32. Range: ±45° = ±4500.
pub type SharedRollOffset = Arc<AtomicI32>;

/// Shared flag for inverting the roll-correction direction, settable from
/// the GUI for installs whose module reports roll with the opposite sign.
pub type SharedRollInvert = Arc<AtomicBool>;

/// Create a new shared heading offset, initialised to the given value.
pub fn new_shared_heading_offset(initial_deg: f64) -> SharedHeadingOffset {
    Arc::new(AtomicI32::new((initial_deg * 100.0).round() as i32))
}

/// Create a new shared antenna height, initialised to the given value.
pub fn new_shared_antenna_height(initial_m: f64) -> SharedAntennaHeight {
    Arc::new(AtomicI32::new((initial_m * 100.0).round() as i32))
}

/// Create a new shared roll offset (centidegrees), initialised to the given value.
pub fn new_shared_roll_offset(initial_deg: f64) -> SharedRollOffset {
    Arc::new(AtomicI32::new((initial_deg * 100.0).round() as i32))
}

/// Create a new shared roll-invert flag, initialised to the given value.
pub fn new_shared_roll_invert(initial: bool) -> SharedRollInvert {
    Arc::new(AtomicBool::new(initial))
}

/// Scan all available serial ports and return the first one that produces
/// GPS NMEA data (not FINN motor sentences).
///
/// Strategy: open each port, read for up to 2 seconds, check for $G prefixed
/// lines. Skip any port that sends $FINNMTR (motor ESP32).
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

    // Sort: USB ports first
    let mut sorted_ports = ports;
    sorted_ports.sort_by_key(|p| match &p.port_type {
        SerialPortType::UsbPort(_) => 0,
        _ => 1,
    });

    for port_info in &sorted_ports {
        let port_name = &port_info.port_name;
        let type_desc = match &port_info.port_type {
            SerialPortType::UsbPort(info) => {
                format!("USB ({})", info.product.as_deref().unwrap_or("unknown"))
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
        let mut is_motor_esp32 = false;

        for line in reader.lines().take(40) {
            match line {
                Ok(sentence) => {
                    if sentence.starts_with("$FINNMTR") || sentence.starts_with("$FINNACK") {
                        tracing::info!("    {} is motor ESP32 (saw $FINN) — skipping", port_name);
                        is_motor_esp32 = true;
                        break;
                    }
                    if sentence.starts_with("$G")
                        || sentence.starts_with("$PAIR")
                        || sentence.starts_with("$PQTM")
                    {
                        if !found_nmea {
                            tracing::info!("    Found GPS NMEA on {}", port_name);
                            found_nmea = true;
                        }
                    }
                }
                Err(_) => continue,
            }
        }

        if found_nmea && !is_motor_esp32 {
            tracing::info!("    GPS module detected on {}", port_name);
            return Some(port_name.clone());
        }

        if !found_nmea && !is_motor_esp32 {
            tracing::debug!("    No NMEA data on {}", port_name);
        }
    }

    None
}

/// Briefly listen for PQTMINS before writing DR config. If the module is
/// already streaming from saved NVS settings, avoid another flash write.
fn pqtmins_already_streaming(port: &mut Box<dyn serialport::SerialPort>) -> bool {
    let response = read_config_response(port, Duration::from_millis(400));
    response.lines().any(|line| line.starts_with("$PQTMINS"))
}

/// Read any immediate response/data from the GPS module for a bounded time.
fn read_config_response(port: &mut Box<dyn serialport::SerialPort>, timeout: Duration) -> String {
    let original_timeout = port.timeout();
    let _ = port.set_timeout(Duration::from_millis(100));

    let start = std::time::Instant::now();
    let mut buf = [0u8; 512];
    let mut response = String::new();

    while start.elapsed() < timeout {
        match port.read(&mut buf) {
            Ok(n) if n > 0 => {
                response.push_str(&String::from_utf8_lossy(&buf[..n]));
            }
            _ => {
                std::thread::sleep(Duration::from_millis(20));
            }
        }
    }

    let _ = port.set_timeout(original_timeout);
    response
}

/// Log the non-standard response forms seen from LC29H BA firmware.
fn log_pqtm_config_response(command: &str, response: &str) {
    if response.contains("$PQTMCFGEINSMSGOK") || response.contains("$PQTMCFGEINSMSG,OK") {
        tracing::info!("  {} accepted", command);
    } else if response.contains("$PQTMCFGEINSMSGERROR")
        || response.contains("$PQTMCFGEINSMSG,ERROR")
    {
        tracing::warn!("  {} rejected: {}", command, compact_response(response));
    } else if response.contains("$PQTMEINSMSG") {
        tracing::warn!(
            "  {} returned get/echo response instead of OK: {}",
            command,
            compact_response(response)
        );
    } else if response.trim().is_empty() {
        tracing::warn!("  No {} acknowledgement received", command);
    } else {
        tracing::debug!("  {} response: {}", command, compact_response(response));
    }
}

fn compact_response(response: &str) -> String {
    response.lines().take(4).collect::<Vec<_>>().join(" | ")
}

/// Send module configuration commands to the LC29H BA.
///
/// Disables unnecessary NMEA sentences, then sets the fix rate to 10Hz.
/// If 10Hz is rejected, falls back to lower rates.
fn ensure_module_config(port: &mut Box<dyn serialport::SerialPort>, fix_rate_hz: u8) {
    let interval_ms = 1000 / fix_rate_hz as u16;
    tracing::info!(
        "Configuring GPS module: target {}Hz ({}ms interval)",
        fix_rate_hz,
        interval_ms
    );

    // Step 1: Enable DR telemetry.
    // Confirmed LC29H BA NR11 two-wheel syntax:
    // $PQTMCFGEINSMSG,<Type>,<INS_Enabled>,<IMU_Enabled>,<GPS_Enabled>,<Rate>
    // Type 1 = Set. We enable PQTMINS and PQTMIMU at 10Hz; PQTMGPS stays off
    // because position continues to come from GGA.
    if pqtmins_already_streaming(port) {
        tracing::info!("  PQTMINS already streaming — skipping DR config write");
    } else {
        let ins_cmd = format_pair_command("PQTMCFGEINSMSG,1,1,1,0,10");
        tracing::info!("  Enabling PQTMINS/PQTMIMU at 10Hz: {}", ins_cmd.trim());
        if let Err(e) = port.write_all(ins_cmd.as_bytes()) {
            tracing::warn!("  Failed to send PQTMCFGEINSMSG: {}", e);
        }
        let _ = port.flush();
        std::thread::sleep(Duration::from_millis(200));

        let response = read_config_response(port, Duration::from_millis(500));
        log_pqtm_config_response("PQTMCFGEINSMSG", &response);

        // Save INS config to NVS so it persists across power cycles. Per Quectel
        // docs the setting becomes active after reset/power-cycle. We avoid a
        // runtime hot-start here because field probes showed it can trash a fix
        // for longer than a normal startup budget.
        let save_cmd = "$PQTMSAVEPAR*5A\r\n";
        tracing::info!("  Saving DR config to NVS: {}", save_cmd.trim());
        if let Err(e) = port.write_all(save_cmd.as_bytes()) {
            tracing::warn!("  Failed to send PQTMSAVEPAR: {}", e);
        }
        let _ = port.flush();
        std::thread::sleep(Duration::from_millis(300));
    }

    // Step 1b: Ensure DRCAL telemetry is enabled. This is independent of
    // PQTMINS/PQTMIMU and drives the GUI calibration-state indicator.
    let drcal_cmd = format_pair_command("PAIR6010,2,1");
    tracing::info!("  Enabling PQTMDRCAL telemetry: {}", drcal_cmd.trim());
    if let Err(e) = port.write_all(drcal_cmd.as_bytes()) {
        tracing::warn!("  Failed to send PAIR6010: {}", e);
    }
    let _ = port.flush();
    std::thread::sleep(Duration::from_millis(150));

    // Step 2: Disable unnecessary NMEA sentences
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

    let _ = port.flush();
    std::thread::sleep(Duration::from_millis(300));

    // Step 3: Set the fix rate
    let rates_to_try: Vec<u8> = {
        let mut rates = vec![fix_rate_hz];
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

        std::thread::sleep(Duration::from_millis(200));

        let mut buf = [0u8; 512];
        let mut response_data = String::new();

        let original_timeout = port.timeout();
        let _ = port.set_timeout(Duration::from_millis(100));

        // BUGFIX: bound the ack-wait by wall-clock, not just per-read timeout.
        // The LC29H BA streams NMEA continuously (GGA + PQTMINS = ~11 sentences/sec),
        // so per-read timeout never fires — we always have data to read. Without
        // a wall-clock cap, this loop runs forever and the reader thread never
        // reaches the main NMEA processing loop. Symptom: app starts, port opens,
        // module gets configured, but no fixes ever reach the GUI.
        let ack_deadline = std::time::Instant::now() + Duration::from_millis(500);
        while std::time::Instant::now() < ack_deadline {
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

        if let Some(ack_pos) = response_data.find("PAIR001,050,") {
            let result_char = response_data.as_bytes().get(ack_pos + 12);
            match result_char {
                Some(b'0') => {
                    tracing::info!("  Module accepted {}Hz", rate);
                    rate_set = true;
                    break;
                }
                Some(b'1') => {
                    tracing::warn!("  {}Hz unsupported by this module variant", rate);
                }
                Some(b'2') => {
                    tracing::warn!("  {}Hz rejected (invalid parameter)", rate);
                }
                Some(c) => {
                    tracing::warn!("  {}Hz unknown response code: {}", rate, *c as char);
                }
                None => {
                    tracing::warn!("  {}Hz ack truncated", rate);
                }
            }
        } else {
            tracing::info!("  No ack for {}Hz (may still be applied)", rate);
            rate_set = true;
            break;
        }
    }

    if !rate_set {
        tracing::warn!("Could not set any fix rate — module may be running at default");
    }

    // Step 4: Persist runtime config to NVS so it survives power cycles.
    //
    // Without this, every GPS power-cycle (USB unplug, inverter blip, laptop
    // sleep + USB re-enumerate) reverts the module to its NVS defaults — most
    // critically the 1Hz fix rate — and the next launch of this app is
    // required to bump it back to 10Hz. If the app crashes or hasn't started
    // yet when the operator queries the GPS, position updates land at 1Hz.
    //
    // PQTMSAVEPAR commits the disable-sentences config + PAIR050 fix rate
    // (and any earlier writes) to flash. The PQTMCFGEINSMSG block above has
    // its own SAVEPAR call only when DR is freshly written; this one covers
    // the rate + sentence filter so they persist independently.
    if rate_set {
        let save_cmd = "$PQTMSAVEPAR*5A\r\n";
        tracing::info!("  Persisting runtime config to NVS: {}", save_cmd.trim());
        if let Err(e) = port.write_all(save_cmd.as_bytes()) {
            tracing::warn!("  Failed to send PQTMSAVEPAR: {}", e);
        }
        let _ = port.flush();
        std::thread::sleep(Duration::from_millis(300));
    }

    tracing::info!("GPS module configuration complete");
}

/// Format a PAIR command with the correct NMEA checksum.
fn format_pair_command(body: &str) -> String {
    let checksum = body.bytes().fold(0u8, |acc, b| acc ^ b);
    format!("${}*{:02X}\r\n", body, checksum)
}

/// Start the GPS reader loop. Blocks — call from a dedicated thread.
///
/// Decision #026: reads directly from the LC29H BA GPS module (no ESP32
/// intermediary). Only GPS NMEA sentences are processed here — FINN messages
/// from the motor ESP32 are handled by the motor reader in comms/serial.rs.
///
/// Decision #027: channel sends are non-blocking (`try_send`) — if a
/// consumer is slow, fixes are dropped rather than backpressured. Drop
/// counters are logged every ~5s so stalls are visible in field logs.
///
/// Once the port is opened, the port name is sent via `port_name_tx` so the
/// motor reader thread knows which port to exclude during auto-detect.
pub fn run_gps_reader(
    config: GpsConfig,
    gps_tx: Sender<GpsFix>,
    gps_tx_steer: Sender<GpsFix>,
    port_name_tx: Sender<String>,
    drop_counters: SharedDropCounters,
    heading_offset: SharedHeadingOffset,
    antenna_height: SharedAntennaHeight,
    roll_offset: SharedRollOffset,
    roll_invert: SharedRollInvert,
) {
    // === Step 1: Resolve port name ===
    let port_name = if config.port_name == "auto" {
        tracing::info!("GPS port set to 'auto' — scanning for GPS module...");
        match auto_detect_gps_port(config.baud_rate) {
            Some(name) => {
                tracing::info!("Auto-detected GPS module on {}", name);
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

    // Notify the motor reader thread which port we claimed
    let _ = port_name_tx.send(port_name.clone());

    // === Step 2: Configure GPS module for 10Hz ===
    // Direct connection to LC29H BA — PAIR commands go straight to the module.
    ensure_module_config(&mut port, config.fix_rate_hz);

    let reader = BufReader::new(port);
    let mut nmea_parser = parser::NmeaState::new();
    nmea_parser.heading_offset_deg = config.heading_offset_deg;
    let mut line_count: u64 = 0;

    // Decision #027: drop-on-full channel sends with shared atomic counters.
    // Blocking sends on bounded channels previously coupled the GPS reader
    // thread to the slowest consumer — a GUI hiccup filled `gps_tx`, blocked
    // the reader, and starved the steer thread through the same blocked
    // thread, producing multi-second cascading freezes. `try_send` + drop
    // gives guidance the latest-value semantics it actually wants (a stale
    // 3s-old fix is worse than no fix).
    //
    // Drop counts are tracked via shared AtomicU64 counters that the steer
    // thread reads-and-resets each second for telemetry. A local rolling
    // WARN log is kept as well for tracing output.
    let mut local_gui_drops: u64 = 0;
    let mut local_steer_drops: u64 = 0;
    let mut gui_closed = false;
    let mut steer_closed = false;
    let mut last_drop_log = std::time::Instant::now();

    for line in reader.lines() {
        match line {
            Ok(sentence) => {
                line_count += 1;
                if line_count <= 10 {
                    tracing::info!("GPS serial [{}]: {}", line_count, sentence);
                }

                // Poll the shared heading offset in case the GUI changed it.
                // This is cheap (one atomic read) and ensures the parser
                // always uses the latest user-configured value.
                nmea_parser.heading_offset_deg =
                    heading_offset.load(Ordering::Relaxed) as f64 / 100.0;

                // Poll the shared antenna height for roll correction.
                nmea_parser.antenna_height_m =
                    antenna_height.load(Ordering::Relaxed) as f64 / 100.0;

                // Poll the shared roll calibration (mounting-bias offset +
                // invert flag) for roll correction.
                nmea_parser.roll_offset_deg =
                    roll_offset.load(Ordering::Relaxed) as f64 / 100.0;
                nmea_parser.roll_invert = roll_invert.load(Ordering::Relaxed);

                // Parse NMEA GPS sentences only — no FINN sentences on this port
                if let Some(fix) = nmea_parser.parse_sentence(&sentence) {
                    if line_count <= 20 {
                        tracing::info!(
                            "Got fix: lat={:.6} lon={:.6} sats={} quality={:?}",
                            fix.latitude,
                            fix.longitude,
                            fix.satellites,
                            fix.fix_quality
                        );
                    }

                    // Send to GUI channel (drop if full — GUI will get the next fix).
                    match gps_tx.try_send(fix.clone()) {
                        Ok(()) => {}
                        Err(crossbeam_channel::TrySendError::Full(_)) => {
                            drop_counters.gps_gui.fetch_add(1, Ordering::Relaxed);
                            local_gui_drops += 1;
                        }
                        Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                            gui_closed = true;
                        }
                    }

                    // Send to steer thread channel (drop if full — steer thread
                    // will compute off the next fix, which is what we want).
                    match gps_tx_steer.try_send(fix) {
                        Ok(()) => {}
                        Err(crossbeam_channel::TrySendError::Full(_)) => {
                            drop_counters.gps_steer.fetch_add(1, Ordering::Relaxed);
                            local_steer_drops += 1;
                        }
                        Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                            steer_closed = true;
                        }
                    }

                    if gui_closed && steer_closed {
                        tracing::warn!("All GPS channels closed, stopping reader");
                        return;
                    }

                    // Log drop counters periodically via tracing (human-readable).
                    // The atomic counters are the source of truth for telemetry;
                    // these local counters are just for the WARN log.
                    if last_drop_log.elapsed() >= Duration::from_secs(5) {
                        if local_gui_drops > 0 || local_steer_drops > 0 {
                            tracing::warn!(
                                "GPS fix drops in last ~5s: gui={} steer={} (consumer stalled)",
                                local_gui_drops,
                                local_steer_drops
                            );
                            local_gui_drops = 0;
                            local_steer_drops = 0;
                        }
                        last_drop_log = std::time::Instant::now();
                    }
                }
            }
            Err(e) => {
                tracing::warn!("Serial read error: {}", e);
            }
        }
    }

    tracing::warn!("GPS reader loop ended (port closed?)");
}
