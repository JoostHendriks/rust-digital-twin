use std::sync::Arc;
use tokio::sync::Mutex as TokioMutex;

use can_socket::CanFrame;
use can_socket::tokio::CanSocket;

use crate::config::BusConfig;

pub struct CanBuffer {
    pub can_socket: CanSocket,
    pub read_queue: Arc<TokioMutex<Vec<CanFrame>>>
}

impl CanBuffer {
    pub fn new(bus_config: &BusConfig) -> Self {
        let canbus = Self {
            can_socket: start_can_socket(bus_config).unwrap(),
            read_queue: Arc::new(TokioMutex::new(Vec::new()))
        };
        canbus
    }
}

pub async fn run_can_socket(socket: CanSocket, queue_clone: Arc<TokioMutex<Vec<CanFrame>>>) {
    loop {
        match socket.recv().await {
            Ok(frame) => {
                let mut queue = queue_clone.lock().await;
                queue.push(frame);
            }
            Err(e) => {
                log::warn!("CAN receive error: {:?}", e);
            }
        }
    }
}


pub fn start_can_socket(bus_config: &BusConfig) -> Result<CanSocket, ()> {

    // Open the CAN bus.
    log::info!("Opening CAN bus on interface {}", bus_config.interface);
    let socket = CanSocket::bind(&bus_config.interface).map_err(|e| {
        log::error!(
            "Failed to create CAN socket for interface {}: {e}",
            bus_config.interface
        )
    })?;
    log::info!("CAN bus on interface {} opened", bus_config.interface);

    Ok(socket)
}