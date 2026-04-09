//! FINN Guidance — ESP32 Sensor Module Firmware
//!
//! Reads three sensor sources and sends them to the PC over USB serial:
//! - GPS: NMEA passthrough from Quectel LC29H DA on UART2 (GPIO 16 RX / 17 TX)
//! - WAS: 10kΩ pot on ADC1 channel 6 (GPIO 34), powered by GPIO 33 (3.3V HIGH)
//! - IMU: BNO055 on I2C (GPIO 21 SDA / 22 SCL)
//!
//! Serial protocol (text-based, NMEA-style for easy debugging):
//!   GPS:  forwarded raw — $GPGGA,...  $GPVTG,...
//!   WAS:  $FINNWAS,<raw_adc>,<voltage_mv>*<checksum>\r\n
//!   IMU:  $FINNIMU,<roll>,<pitch>,<heading>,<cal_sys>,<cal_gyro>,<cal_accel>,<cal_mag>*<checksum>\r\n
//!   HB:   $FINNHB,<uptime_ms>*<checksum>\r\n
//!
//! Rates: WAS + IMU at ~20Hz, GPS passthrough at whatever the module sends (1Hz),
//!        heartbeat every 2 seconds.

use esp_idf_hal::adc::config::Config as AdcConfig;
use esp_idf_hal::adc::{self, AdcChannelDriver, AdcDriver};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{self, PinDriver};
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::{self, UartDriver};
use esp_idf_hal::units::Hertz;
use esp_idf_sys as _;
use log::{info, warn, error};

// ── BNO055 constants ──────────────────────────────────────────────────
const BNO055_ADDR: u8 = 0x28; // Default I2C address (AD0 low)
const BNO055_CHIP_ID_REG: u8 = 0x00;
const BNO055_CHIP_ID_VALUE: u8 = 0xA0;
const BNO055_OPR_MODE_REG: u8 = 0x3D;
const BNO055_SYS_TRIGGER_REG: u8 = 0x3F;
const BNO055_CALIB_STAT_REG: u8 = 0x35;
// Euler angle registers (LSB first, 1 degree = 16 LSB for heading, 1/16 for roll/pitch)
const BNO055_EUL_HEADING_LSB: u8 = 0x1A;
// Operating modes
const BNO055_MODE_CONFIG: u8 = 0x00;
const BNO055_MODE_NDOF: u8 = 0x0C; // 9DOF fusion mode

// ── Timing ────────────────────────────────────────────────────────────
const SENSOR_INTERVAL_MS: u32 = 50; // 20Hz for WAS + IMU
const HEARTBEAT_INTERVAL_MS: u32 = 2000;

fn main() -> anyhow::Result<()> {
    // Initialise ESP-IDF logging
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("FINN Sensor Module starting...");

    let peripherals = Peripherals::take()?;

    // ── GPIO 33: WAS pot power (3.3V reference) ──────────────────────
    let mut was_power = PinDriver::output(peripherals.pins.gpio33)?;
    was_power.set_high()?;
    info!("GPIO 33 set HIGH — WAS pot powered (3.3V)");

    // ── ADC: WAS pot wiper on GPIO 34 ────────────────────────────────
    let adc1 = AdcDriver::new(peripherals.adc1, &AdcConfig::new().calibration(true))?;
    let was_pin: AdcChannelDriver<{ adc::attenuation::DB_11 }, _> =
        AdcChannelDriver::new(peripherals.pins.gpio34)?;
    info!("ADC1 CH6 (GPIO 34) configured — WAS input");

    // ── I2C: BNO055 IMU on GPIO 21/22 ────────────────────────────────
    let i2c_config = I2cConfig::new().baudrate(Hertz(400_000));
    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio21, // SDA
        peripherals.pins.gpio22, // SCL
        &i2c_config,
    )?;
    info!("I2C0 configured — SDA=21, SCL=22, 400kHz");

    let bno_ok = bno055_init(&i2c);
    if bno_ok {
        info!("BNO055 initialised in NDOF mode");
    } else {
        warn!("BNO055 init failed — IMU data will be zeros. Check wiring.");
    }

    // ── UART2: GPS NMEA from ArduSimple (LC29H DA) ───────────────────
    let uart_config = uart::config::Config::new().baudrate(Hertz(115_200));
    let gps_uart = UartDriver::new(
        peripherals.uart2,
        peripherals.pins.gpio17, // TX (to GPS RX — for config commands)
        peripherals.pins.gpio16, // RX (from GPS TX — NMEA sentences)
        Option::<gpio::Gpio0>::None, // no CTS
        Option::<gpio::Gpio0>::None, // no RTS
        &uart_config,
    )?;
    info!("UART2 configured — RX=16, TX=17, 115200 baud (GPS)");

    // ── USB serial output (UART0) is stdout — we use print macros ────
    info!("All peripherals initialised. Entering main loop.");

    // ── Main loop ────────────────────────────────────────────────────
    let mut last_sensor_ms: u32 = 0;
    let mut last_heartbeat_ms: u32 = 0;
    let mut gps_line_buf = [0u8; 256];
    let mut gps_line_pos: usize = 0;

    loop {
        let now = uptime_ms();

        // ── GPS passthrough: read UART2, forward complete NMEA lines ─
        loop {
            let mut byte = [0u8; 1];
            match gps_uart.read(&mut byte, 0) {
                Ok(1) => {
                    let b = byte[0];
                    if b == b'\n' {
                        // Forward the complete line (already has \r from GPS)
                        if gps_line_pos > 0 {
                            if let Ok(line) = core::str::from_utf8(&gps_line_buf[..gps_line_pos]) {
                                let trimmed = line.trim_end_matches('\r');
                                // Only forward NMEA sentences (starts with $)
                                if trimmed.starts_with('$') {
                                    println!("{}", trimmed);
                                }
                            }
                            gps_line_pos = 0;
                        }
                    } else if gps_line_pos < gps_line_buf.len() {
                        gps_line_buf[gps_line_pos] = b;
                        gps_line_pos += 1;
                    } else {
                        // Buffer overflow — discard line
                        gps_line_pos = 0;
                    }
                }
                _ => break, // No more bytes available
            }
        }

        // ── Sensor reads at 20Hz ─────────────────────────────────────
        if now.wrapping_sub(last_sensor_ms) >= SENSOR_INTERVAL_MS {
            last_sensor_ms = now;

            // Read WAS
            let was_raw = read_was(&adc1, &was_pin);
            let was_mv = (was_raw as u32 * 3300) / 4095;
            let was_checksum = nmea_checksum(&format!("FINNWAS,{},{}", was_raw, was_mv));
            println!("$FINNWAS,{},{}*{:02X}", was_raw, was_mv, was_checksum);

            // Read IMU
            if bno_ok {
                let (heading, roll, pitch, cal) = bno055_read_euler(&i2c);
                let imu_body = format!(
                    "FINNIMU,{:.1},{:.1},{:.1},{},{},{},{}",
                    roll, pitch, heading, cal.0, cal.1, cal.2, cal.3
                );
                let imu_checksum = nmea_checksum(&imu_body);
                println!("${}*{:02X}", imu_body, imu_checksum);
            }
        }

        // ── Heartbeat every 2s ───────────────────────────────────────
        if now.wrapping_sub(last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS {
            last_heartbeat_ms = now;
            let hb_body = format!("FINNHB,{}", now);
            let hb_checksum = nmea_checksum(&hb_body);
            println!("${}*{:02X}", hb_body, hb_checksum);
        }

        // Short sleep to avoid busy-spinning — GPS bytes buffered by UART hardware
        FreeRtos::delay_ms(1);
    }
}

// ── NMEA checksum (XOR of all chars between $ and *) ─────────────────
fn nmea_checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

// ── Uptime in milliseconds ───────────────────────────────────────────
fn uptime_ms() -> u32 {
    unsafe { esp_idf_sys::esp_timer_get_time() as u32 / 1000 }
}

// ── WAS ADC read ─────────────────────────────────────────────────────
fn read_was<'a>(
    adc: &AdcDriver<'a, adc::ADC1>,
    pin: &AdcChannelDriver<'a, { adc::attenuation::DB_11 }, gpio::Gpio34>,
) -> u16 {
    match adc.read(pin) {
        Ok(val) => val as u16,
        Err(e) => {
            warn!("ADC read error: {:?}", e);
            0
        }
    }
}

// ── BNO055 I2C helpers ───────────────────────────────────────────────

fn bno055_init(i2c: &I2cDriver) -> bool {
    // Check chip ID
    let mut buf = [0u8; 1];
    if i2c.write_read(BNO055_ADDR, &[BNO055_CHIP_ID_REG], &mut buf, 100).is_err() {
        error!("BNO055: no response on I2C");
        return false;
    }
    if buf[0] != BNO055_CHIP_ID_VALUE {
        error!("BNO055: unexpected chip ID 0x{:02X} (expected 0xA0)", buf[0]);
        return false;
    }

    // Reset
    let _ = i2c.write(BNO055_ADDR, &[BNO055_SYS_TRIGGER_REG, 0x20], 100);
    FreeRtos::delay_ms(700); // BNO055 needs ~650ms after reset

    // Wait for chip ID to come back
    for _ in 0..10 {
        if i2c.write_read(BNO055_ADDR, &[BNO055_CHIP_ID_REG], &mut buf, 100).is_ok()
            && buf[0] == BNO055_CHIP_ID_VALUE
        {
            break;
        }
        FreeRtos::delay_ms(100);
    }

    // Set to config mode first (should already be after reset)
    let _ = i2c.write(BNO055_ADDR, &[BNO055_OPR_MODE_REG, BNO055_MODE_CONFIG], 100);
    FreeRtos::delay_ms(25);

    // Set to NDOF mode (full 9DOF sensor fusion)
    let _ = i2c.write(BNO055_ADDR, &[BNO055_OPR_MODE_REG, BNO055_MODE_NDOF], 100);
    FreeRtos::delay_ms(25);

    true
}

fn bno055_read_euler(i2c: &I2cDriver) -> (f64, f64, f64, (u8, u8, u8, u8)) {
    // Read 6 bytes: heading LSB/MSB, roll LSB/MSB, pitch LSB/MSB
    let mut buf = [0u8; 6];
    if i2c
        .write_read(BNO055_ADDR, &[BNO055_EUL_HEADING_LSB], &mut buf, 100)
        .is_err()
    {
        return (0.0, 0.0, 0.0, (0, 0, 0, 0));
    }

    let heading_raw = i16::from_le_bytes([buf[0], buf[1]]);
    let roll_raw = i16::from_le_bytes([buf[2], buf[3]]);
    let pitch_raw = i16::from_le_bytes([buf[4], buf[5]]);

    // BNO055 euler angles: 1 LSB = 1/16 degree
    let heading = heading_raw as f64 / 16.0;
    let roll = roll_raw as f64 / 16.0;
    let pitch = pitch_raw as f64 / 16.0;

    // Read calibration status
    let mut cal_buf = [0u8; 1];
    let cal = if i2c
        .write_read(BNO055_ADDR, &[BNO055_CALIB_STAT_REG], &mut cal_buf, 100)
        .is_ok()
    {
        let c = cal_buf[0];
        (
            (c >> 6) & 0x03, // sys
            (c >> 4) & 0x03, // gyro
            (c >> 2) & 0x03, // accel
            c & 0x03,        // mag
        )
    } else {
        (0, 0, 0, 0)
    };

    (heading, roll, pitch, cal)
}
