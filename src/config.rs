use std::path::Path;

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
#[serde(rename_all = "PascalCase")]
pub struct Config {

    /// Configuration of the CANopen bus.
    pub bus: BusConfig,

    /// Configuration of the nodes.
    pub node: Vec<Node>,

    /// Factor by which simulated motion time is divided. Values above 1.0
    /// speed up motion (e.g. 10.0 makes a 5 s move finish in 0.5 s). Useful
    /// for testing without waiting for real motion durations.
    /// Must appear before any section headers (i.e. at the very top of the TOML file).
    #[serde(rename = "time_scale", default = "default_time_scale")]
    pub time_scale: f64,

}

fn default_time_scale() -> f64 { 1.0 }

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusConfig {

    /// The CAN interface to use.
    pub interface: String,

    /// The baudrate of the bus.
    pub baud_rate: usize,

}

#[derive(Debug, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Node {

    /// Node id
    pub node_id: u8,

    /// Eds file
    pub eds_file: String,

}



impl Config {
    /// Read the configuration from a TOML file.
    pub fn read_from_file(path: impl AsRef<Path>) -> Result<Self, ()> {
        use std::io::Read;

        let path = path.as_ref();
        let mut file = std::fs::File::open(path)
            .map_err(|e| log::error!("Failed to open {} for reading: {e}", path.display()))?;
        let mut data = Vec::new();
        file.read_to_end(&mut data)
            .map_err(|e| log::error!("Failed to read from {}: {e}", path.display()))?;
        let data = std::str::from_utf8(&data)
            .map_err(|e| log::error!("Invalid UTF-8 in {}: {e}", path.display()))?;

        let config: Self = toml::from_str(data)
            .map_err(|e| log::error!("Failed to parse {}: {e}", path.display()))?;
        Ok(config)
    }
}