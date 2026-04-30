mod comms;
mod coverage;
mod gps;
mod gui;
mod guidance;
mod position;
mod telemetry;

use crate::comms::serial::MotorHandle;
use crate::guidance::steer_thread::{SharedSteerState, SteerStateHandle};
use crate::telemetry::SharedDropCounters;
use crossbeam_channel;
use finn_guidance_common::protocol::FinnMessage;
use finn_guidance_common::types::GpsFix;
use std::sync::{Arc, Mutex};
use std::thread;
use tracing_subscriber;

fn main() {
    // Initialise logging
    tracing_subscriber::fmt::init();
    tracing::info!("FINN Guidance starting (Decision #026 architecture)...");

    // Channel for GPS data: GPS reader thread -> GUI (coverage, trail, display)
    let (gps_tx_gui, gps_rx_gui) = crossbeam_channel::bounded::<GpsFix>(64);
    // Channel for GPS data: GPS reader thread -> steer thread (steering compute)
    let (gps_tx_steer, gps_rx_steer) = crossbeam_channel::bounded::<GpsFix>(64);

    // Channel for FINN motor data -> GUI (motor status display, config acks)
    let (finn_tx_gui, finn_rx_gui) = crossbeam_channel::bounded::<FinnMessage>(128);
    // Channel for FINN motor data -> steer thread (motor feedback)
    let (finn_tx_steer, finn_rx_steer) = crossbeam_channel::bounded::<FinnMessage>(128);

    // Channel for the GPS reader to report which COM port it claimed,
    // so the motor reader can avoid it during auto-detect.
    let (port_tx, port_rx) = crossbeam_channel::bounded::<String>(1);

    // Motor handle — shared between the motor reader thread, steer thread, and GUI
    let motor_handle = MotorHandle::new();

    // Shared steering state — steer thread + GUI
    let steer_state: SteerStateHandle = Arc::new(Mutex::new(SharedSteerState::new(
        3.0,  // lookahead_base
        1.0,  // lookahead_speed_factor
        2.8,  // wheelbase_m
        15.0, // max_steer_angle
        0.5,  // kd_xte
        0.03, // deadband_m
        12.0, // implement_width_m
        0.0,  // overlap_m
    )));

    // Shared drop counters — incremented by reader threads, consumed by steer thread telemetry
    let drop_counters = SharedDropCounters::new();

    // Shared heading offset — GPS reader applies it to each fix,
    // GUI can adjust it in real time via the setup page slider.
    // Initial value loaded from SQLite in the app constructor;
    // for now start at 0.0 and let the app update it.
    let heading_offset = gps::reader::new_shared_heading_offset(0.0);

    // Shared antenna height — GPS reader uses it for roll correction.
    // Initial value loaded from SQLite in the app constructor;
    // for now start at 0.0 (disabled) and let the app update it.
    let antenna_height = gps::reader::new_shared_antenna_height(0.0);

    // Start GPS serial reader thread
    // Decision #026: reads directly from LC29H BA (no sensor ESP32)
    // Sends fixes to BOTH the GUI and steer thread channels.
    let drop_counters_gps = drop_counters.clone();
    let heading_offset_gps = heading_offset.clone();
    let antenna_height_gps = antenna_height.clone();
    thread::spawn(move || {
        gps::reader::run_gps_reader(
            gps::reader::GpsConfig::default(),
            gps_tx_gui,
            gps_tx_steer,
            port_tx,
            drop_counters_gps,
            heading_offset_gps,
            antenna_height_gps,
        );
    });
    tracing::info!("GPS serial reader thread launched");

    // Start motor serial reader thread
    // Sends motor status to BOTH the GUI and steer thread channels.
    let motor_handle_thread = motor_handle.clone();
    let drop_counters_mtr = drop_counters.clone();
    thread::spawn(move || {
        // Wait for the GPS reader to report its port
        let gps_port = port_rx.recv().unwrap_or_default();
        if gps_port.is_empty() {
            tracing::warn!("GPS port unknown — motor auto-detect may conflict");
        }
        comms::serial::run_motor_reader(
            gps_port,
            115200,
            motor_handle_thread,
            finn_tx_gui,
            finn_tx_steer,
            drop_counters_mtr,
        );
    });
    tracing::info!("Motor serial reader thread launched");

    // Start dedicated steering thread (10Hz fixed loop)
    let motor_handle_steer = motor_handle.clone();
    let steer_state_thread = steer_state.clone();
    let drop_counters_steer = drop_counters.clone();
    thread::spawn(move || {
        guidance::steer_thread::run_steer_thread(
            gps_rx_steer,
            finn_rx_steer,
            motor_handle_steer,
            steer_state_thread,
            drop_counters_steer,
        );
    });
    tracing::info!("Steering thread launched (10Hz fixed loop)");

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
                gps_rx_gui,
                finn_rx_gui,
                motor_handle,
                steer_state,
                12.0,
                heading_offset,
                antenna_height,
            )))
        }),
    );

    tracing::info!("FINN Guidance shutting down");
}
