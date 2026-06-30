use can_socket::tokio::CanSocket;
use std::path::PathBuf;
use tokio::task;
use futures::future;

mod eds;
mod config;
mod cia301;
mod cia402_runner;

use crate::cia301::Node;
use crate::config::Config;

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
    
    // Initialize nodes
    let mut nodes = Vec::new();

    // Build nodes from eds files and bind socket
    for node in config.node.iter() {

        // Bind socket
        let socket = CanSocket::bind(&config.bus.interface).map_err(|e| {
            log::error!("Failed to create CAN socket for interface {}: {e}", &config.bus.interface)
        })?;
        log::info!("CAN bus on interface {} opened for node {}", &config.bus.interface, node.node_id);

        // Parse eds data
        let node_data = eds::parse_eds(&node.node_id, &node.eds_file).map_err(|e| {
            log::error!("Failed to parse EDS file {} for node {}: {e}", node.eds_file, node.node_id)
        })?;

        // Initialize controller
        let node = Node::initialize(socket, node.node_id, node_data).await.map_err(|_| {
            log::error!("Failed to initialize node {}", node.node_id)
        })?;
        nodes.push(node);
    }

    let mut futures = Vec::new();

    // Start nodes
    for mut node in nodes.into_iter() {
        futures.push(
            task::spawn(async move {
                node.start_socket().await;
            })
        );
    }

    future::join_all(futures).await;
    
    Ok(())
}

