mod gps;
mod guidance;
mod gui;
mod comms;
mod position;
mod coverage;

use tracing_subscriber;
use crossbeam_channel;
use finn_guidance_common::types::GpsFix;
use std::thread;

fn main() {
    // Initialise logging
    tracing_subscriber::fmt::init();
    tracing::info!("FINN Guidance starting...");

    // Channel for GPS data: gps thread -> gui
    // Buffer of 16 is plenty even at 10Hz GPS with 60fps GUI drain
    let (gps_tx, gps_rx) = crossbeam_channel::bounded::<GpsFix>(16);

    // Start GPS reader thread (auto-detects COM port, configures module for 5Hz)
    let gps_config = gps::reader::GpsConfig::default();

    thread::spawn(move || {
        gps::reader::run_gps_reader(gps_config, gps_tx);
    });

    tracing::info!("GPS reader thread launched (auto-detect mode, 5Hz)");

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
            Ok(Box::new(gui::app::GuidanceApp::new(gps_rx, 12.0)))
        }),
    );

    tracing::info!("FINN Guidance shutting down");
}
