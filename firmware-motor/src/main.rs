//! FINN Guidance — ESP32 Motor Controller Firmware
//!
//! Receives steer commands from the PC over USB serial and drives the IBT-2
//! H-bridge to control the steering motor.
//!
//! Pinout:
//!   GPIO 25 — IBT-2 RPWM (steer right, PWM)
//!   GPIO 26 — IBT-2 LPWM (steer left, PWM)
//!   GPIO 27 — IBT-2 R_EN (right enable)
//!   GPIO 14 — IBT-2 L_EN (left enable)
//!
//! Serial protocol (text-based, NMEA-style):
//!   Receive: $FINNSTEER,<pwm_value>*<checksum>\r\n
//!            pwm_value: -255 to 255 (negative=left, positive=right)
//!   Send:    $FINNMTR,<current_pwm>,<uptime_ms>*<checksum>\r\n  (status at 5Hz)
//!
//! Safety:
//!   - Motor stops if no valid command received within 500ms (watchdog)
//!   - PWM clamped to ±255
//!   - Enable lines driven LOW (motor off) on startup and watchdog trip

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::PinDriver;
use esp_idf_hal::ledc::{self, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;
use esp_idf_sys as _;
use log::{info, warn, error};

// ── Timing ────────────────────────────────────────────────────────────
const WATCHDOG_TIMEOUT_MS: u32 = 500;
const STATUS_INTERVAL_MS: u32 = 200; // 5Hz status reports

fn main() -> anyhow::Result<()> {
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("FINN Motor Controller starting...");

    let peripherals = Peripherals::take()?;

    // ── IBT-2 enable lines (GPIO 27 = R_EN, GPIO 14 = L_EN) ─────────
    let mut r_en = PinDriver::output(peripherals.pins.gpio27)?;
    let mut l_en = PinDriver::output(peripherals.pins.gpio14)?;
    r_en.set_low()?; // Motor disabled on startup
    l_en.set_low()?;
    info!("IBT-2 enable lines LOW (motor disabled)");

    // ── IBT-2 PWM channels (GPIO 25 = RPWM, GPIO 26 = LPWM) ────────
    // Using LEDC peripheral for hardware PWM at 20kHz
    let timer0 = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &ledc::config::TimerConfig::new().frequency(Hertz(20_000)).resolution(ledc::Resolution::Bits8),
    )?;
    let timer1 = LedcTimerDriver::new(
        peripherals.ledc.timer1,
        &ledc::config::TimerConfig::new().frequency(Hertz(20_000)).resolution(ledc::Resolution::Bits8),
    )?;

    let mut rpwm = LedcDriver::new(
        peripherals.ledc.channel0,
        &timer0,
        peripherals.pins.gpio25,
    )?;
    let mut lpwm = LedcDriver::new(
        peripherals.ledc.channel1,
        &timer1,
        peripherals.pins.gpio26,
    )?;
    rpwm.set_duty(0)?;
    lpwm.set_duty(0)?;
    info!("PWM configured — RPWM=GPIO25, LPWM=GPIO26, 20kHz 8-bit");

    // ── USB serial input (UART0 = stdin) ─────────────────────────────
    info!("All peripherals initialised. Entering main loop.");

    let mut line_buf = [0u8; 128];
    let mut line_pos: usize = 0;
    let mut last_command_ms: u32 = uptime_ms();
    let mut last_status_ms: u32 = 0;
    let mut current_pwm: i16 = 0;
    let mut motor_enabled = false;

    loop {
        let now = uptime_ms();

        // ── Read serial input, parse complete lines ──────────────────
        // Read from stdin byte by byte (non-blocking via esp-idf)
        let mut byte_buf = [0u8; 1];
        while unsafe { read_stdin_byte(&mut byte_buf) } {
            let b = byte_buf[0];
            if b == b'\n' {
                if line_pos > 0 {
                    if let Ok(line) = core::str::from_utf8(&line_buf[..line_pos]) {
                        let trimmed = line.trim();
                        if let Some(pwm) = parse_steer_command(trimmed) {
                            current_pwm = pwm;
                            last_command_ms = now;

                            if !motor_enabled {
                                r_en.set_high()?;
                                l_en.set_high()?;
                                motor_enabled = true;
                                info!("Motor ENABLED (first command received)");
                            }

                            apply_pwm(&mut rpwm, &mut lpwm, current_pwm)?;
                        }
                    }
                    line_pos = 0;
                }
            } else if b != b'\r' && line_pos < line_buf.len() {
                line_buf[line_pos] = b;
                line_pos += 1;
            }
        }

        // ── Watchdog: stop motor if no command for 500ms ─────────────
        if motor_enabled && now.wrapping_sub(last_command_ms) > WATCHDOG_TIMEOUT_MS {
            warn!("WATCHDOG: no command for {}ms — motor STOPPED", WATCHDOG_TIMEOUT_MS);
            current_pwm = 0;
            rpwm.set_duty(0)?;
            lpwm.set_duty(0)?;
            r_en.set_low()?;
            l_en.set_low()?;
            motor_enabled = false;
        }

        // ── Status report at 5Hz ─────────────────────────────────────
        if now.wrapping_sub(last_status_ms) >= STATUS_INTERVAL_MS {
            last_status_ms = now;
            let enabled_flag = if motor_enabled { 1 } else { 0 };
            let body = format!("FINNMTR,{},{},{}", current_pwm, enabled_flag, now);
            let checksum = nmea_checksum(&body);
            println!("${}*{:02X}", body, checksum);
        }

        FreeRtos::delay_ms(1);
    }
}

// ── Parse $FINNSTEER,<pwm>*<checksum> ────────────────────────────────
fn parse_steer_command(line: &str) -> Option<i16> {
    // Strip $ prefix
    let line = line.strip_prefix('$')?;

    // Split off checksum (after *)
    let (body, expected_cs) = line.rsplit_once('*')?;

    // Verify checksum
    let expected = u8::from_str_radix(expected_cs.trim(), 16).ok()?;
    let actual = nmea_checksum(body);
    if expected != actual {
        warn!("Checksum mismatch: expected {:02X}, got {:02X} for '{}'", expected, actual, body);
        return None;
    }

    // Parse command
    let body = body.strip_prefix("FINNSTEER,")?;
    let pwm: i16 = body.trim().parse().ok()?;

    // Clamp to valid range
    Some(pwm.clamp(-255, 255))
}

// ── Apply PWM value to IBT-2 ─────────────────────────────────────────
fn apply_pwm(
    rpwm: &mut LedcDriver,
    lpwm: &mut LedcDriver,
    pwm: i16,
) -> anyhow::Result<()> {
    let duty = pwm.unsigned_abs().min(255) as u32;

    if pwm > 0 {
        // Steer right: RPWM active, LPWM off
        lpwm.set_duty(0)?;
        rpwm.set_duty(duty)?;
    } else if pwm < 0 {
        // Steer left: LPWM active, RPWM off
        rpwm.set_duty(0)?;
        lpwm.set_duty(duty)?;
    } else {
        // Stop: both off
        rpwm.set_duty(0)?;
        lpwm.set_duty(0)?;
    }

    Ok(())
}

// ── NMEA checksum ────────────────────────────────────────────────────
fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

// ── Uptime in milliseconds ───────────────────────────────────────────
fn uptime_ms() -> u32 {
    unsafe { esp_idf_sys::esp_timer_get_time() as u32 / 1000 }
}

// ── Non-blocking stdin read (one byte) ───────────────────────────────
/// Attempts to read a single byte from UART0 (USB serial) without blocking.
/// Returns true if a byte was read, false if no data available.
unsafe fn read_stdin_byte(buf: &mut [u8; 1]) -> bool {
    // Use the ESP-IDF UART driver directly for non-blocking reads
    let len = esp_idf_sys::uart_read_bytes(
        esp_idf_sys::uart_port_t_UART_NUM_0,
        buf.as_mut_ptr() as *mut _,
        1,
        0, // timeout_ticks = 0 for non-blocking
    );
    len == 1
}
