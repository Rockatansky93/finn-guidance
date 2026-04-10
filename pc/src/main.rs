mod gps;
mod guidance;
mod gui;
mod comms;
mod position;
mod coverage;

use tracing_subscriber;
use crossbeam_channel;
use finn_guidance_common::types::GpsFix;
use finn_guidance_common::protocol::FinnMessage;
use crate::comms::serial::MotorHandle;
use std::thread;

fn main() {
    // Initialise logging
    tracing_subscriber::fmt::init();
    tracing::info!("FINN Guidance starting...");

    // Channel for GPS data: sensor serial thread -> gui
    let (gps_tx, gps_rx) = crossbeam_channel::bounded::<GpsFix>(16);

    // Channel for FINN sensor data (WAS, IMU, heartbeat, motor status)
    // Both the sensor reader and motor reader send into this channel.
    let (finn_tx, finn_rx) = crossbeam_channel::bounded::<FinnMessage>(128);

    // Channel for the sensor reader to report which COM port it claimed,
    // so the motor reader can avoid it during auto-detect.
    let (port_tx, port_rx) = crossbeam_channel::bounded::<String>(1);

    // Motor handle — shared between the motor reader thread and the GUI
    let motor_handle = MotorHandle::new();

    // Start sensor serial reader thread
    let finn_tx_sensor = finn_tx.clone();
    thread::spawn(move || {
        gps::reader::run_gps_reader(
            gps::reader::GpsConfig::default(),
            gps_tx,
            finn_tx_sensor,
            port_tx,
        );
    });
    tracing::info!("Sensor serial reader thread launched");

    // Start motor serial reader thread
    let motor_handle_thread = motor_handle.clone();
    let finn_tx_motor = finn_tx.clone();
    thread::spawn(move || {
        // Wait for the sensor reader to report its port
        let sensor_port = port_rx.recv().unwrap_or_default();
        if sensor_port.is_empty() {
            tracing::warn!("Sensor port unknown — motor auto-detect may conflict");
        }
        comms::serial::run_motor_reader(
            sensor_port,
            115200,
            motor_handle_thread,
            finn_tx_motor,
        );
    });
    tracing::info!("Motor serial reader thread launched");

    // Launch GUI
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([800.0, 600.0])
            .with_title("FINN Guidance"),
        ..Default::default()
    };

    let _ = eframe::run_native(
        "FINN Guidance",
        native_options,
        Box::new(move |_cc| {
            Ok(Box::new(gui::app::GuidanceApp::new(
                gps_rx,
                finn_rx,
                motor_handle,
                12.0,
            )))
        }),
    );

    tracing::info!("FINN Guidance shutting down");
}
