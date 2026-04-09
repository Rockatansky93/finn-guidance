//! UDP communication with the ESP32 steering controller.
//! 
//! Placeholder for Phase 4 (steering implementation).
//! Will handle sending steer commands and receiving sensor data.

use std::net::UdpSocket;
use finn_guidance_common::protocol;

pub struct EspConnection {
    socket: Option<UdpSocket>,
    esp_address: String,
}

impl EspConnection {
    pub fn new(esp_ip: &str) -> Self {
        Self {
            socket: None,
            esp_address: format!("{}:{}", esp_ip, protocol::PC_TO_ESP_PORT),
        }
    }

    /// Connect to the ESP32 (bind local UDP socket)
    pub fn connect(&mut self) -> Result<(), String> {
        let bind_addr = format!("0.0.0.0:{}", protocol::ESP_TO_PC_PORT);
        match UdpSocket::bind(&bind_addr) {
            Ok(sock) => {
                sock.set_nonblocking(true).map_err(|e| e.to_string())?;
                self.socket = Some(sock);
                Ok(())
            }
            Err(e) => Err(format!("Failed to bind UDP socket: {}", e)),
        }
    }

    // TODO Phase 4: send_steer_command(), receive_sensor_data()
}
