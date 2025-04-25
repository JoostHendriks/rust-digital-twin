use cia402_runner::MotorController;
use std::path::PathBuf;
use tokio::task;
use std::sync::Arc;
use tokio::sync::Mutex as TokioMutex;
use futures::future;
use std::time::{Instant, Duration};

mod canbus;
mod eds;
mod config;
mod cia301;
mod nmt;
mod sdo;
mod cia402_runner;

use crate::config::Config;
use crate::canbus::CanBuffer;

#[derive(clap::Parser)]
struct Options {
    /// The path of the configuration file to use.
    #[clap(long, short)]
    #[clap(value_name = "CONFIG.toml")]
    config: PathBuf,
}

#[tokio::main]
async fn main() {

    // Initialize the logging system.
    env_logger::builder()
        .filter_module(module_path!(), log::LevelFilter::Info)
        .parse_default_env()
        .init();


    // Run the server and set a non-zero exit code if we had an error.
    do_main(clap::Parser::parse()).await.ok();

}

async fn do_main(options: Options) -> Result<(), ()> {

    // Read the configuration file.
    let config = Config::read_from_file(&options.config)?;

    let speed_factor = config.general.speed_factor;

    // Initialize can bus
    let can_buffer = CanBuffer::new(&config.bus);

    // Run can receive socket
    let queue_clone = Arc::clone(&can_buffer.read_queue);
    let can_rx_handle = task::spawn(async move {
        canbus::run_can_socket(can_buffer.can_socket, queue_clone).await
    });

    let can_socket_tx = Arc::new(TokioMutex::new(canbus::start_can_socket(&config.bus)?));

    // Initialze and run fake controllers
    for node in config.node.iter() {

        // Parse eds data
        let node_id = node.node_id;
        let node_data = eds::parse_eds(&node_id, &node.eds_file).unwrap();

        // Initialize controller
        let controller = Arc::new(TokioMutex::new(MotorController::initialize(Arc::clone(&can_socket_tx), Arc::clone(&can_buffer.read_queue), node_id, node_data).await));
        log::info!("Node {} initialized", node_id);

        cia402_runner::run(Arc::clone(&controller), &speed_factor).await
    }

    // Keep the rx handle running
    can_rx_handle.await.ok();
    
    Ok(())
}