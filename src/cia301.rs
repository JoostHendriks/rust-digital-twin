use std::time::Instant;
use std::collections::VecDeque;

use can_socket::{tokio::CanSocket, CanId};
use can_socket::CanFrame;
use canopen_tokio::nmt::{NmtCommand, NmtState};

use crate::eds::{DataValue, EDSData};
use crate::cia402_runner::{Command, HomeStatus, ModeOfOperation, ProfilePositionStatus, ProfileVelocityStatus, State};

pub struct Node {
    pub node_id: u8,
    pub eds_data: EDSData,
    pub nmt_state: NmtState,
    pub socket: CanSocket,
    pub motor_controller: MotorController,
}

#[derive(Default)]
pub struct MotorController {
    pub mode_of_operation: ModeOfOperation,
    pub controlword: u16,
    pub command: Command,
    pub statusword: u16,
    pub state: State,
    pub profile_position_status: ProfilePositionStatus,
    pub profile_velocity_status: ProfileVelocityStatus,
    pub halt: bool,
    pub control_oms1: VecDeque<bool>,
    pub home_status: HomeStatus,
    pub target_reached: bool,
    pub status_oms1: bool,
    pub status_oms2: bool,
    pub timer: Option<Instant>,
}

#[derive(Debug)]
enum ServerCommand {
    
	/// The server is uploading a segment.
	_UploadSegmentResponse = 0,

	/// The server has downloaded the segment.
	_DownloadSegmentResponse = 1,

	/// The server accepts the upload request.
	InitiateUploadResponse = 2,

	/// The server accepts the download request.
	InitiateDownloadResponse = 3,

	/// The server is aborting the transfer.
	_AbortTransfer = 4,
}

#[derive(Debug)]
enum ClientCommand {

	/// Download a segment to the server.
	SegmentDownload = 0,

	/// Initiate a download to the server.
	InitiateDownload = 1,

	/// Initiate an upload from the server.
	InitiateUpload = 2,

	/// Request the server to upload a segment.
	SegmentUpload = 3,

	/// Tell the server we are aborting the transfer.
	AbortTransfer = 4,

    /// Unknown client command.
    Unknown = 5,
}

impl ClientCommand {
    fn client_command(value: u8) -> ClientCommand {
        match value {
            0 => ClientCommand::SegmentDownload,
            1 => ClientCommand::InitiateDownload,
            2 => ClientCommand::InitiateUpload,
            3 => ClientCommand::SegmentUpload,
            4 => ClientCommand::AbortTransfer,
            _ => ClientCommand::Unknown,
        }
    }
}


impl Node {
    /// Initialize the motor controller.
    pub async fn initialize(
        socket: CanSocket,
        node_id: u8,
        eds_data: EDSData,
    ) -> Result<Self, ()> {
        let mut node = Self {
            node_id,
            eds_data,
            nmt_state: NmtState::Initializing,
            socket,
            motor_controller: Default::default()
        };
        node.motor_controller.control_oms1 = VecDeque::from(vec![false; 2]);
        Ok(node)
    }

    pub async fn start_socket(&mut self) {
        loop {
            let frame = match self.socket.recv().await {
                Ok(f) => f,
                Err(e) => {
                    log::error!("Error receiving frame: {}", e);
                    continue;
                }
            };

            let cob_id = frame.id().as_u32();
            let node_id = (cob_id & 0x7F) as u8;
            let function_code = frame.id().as_u32() & (0x0F << 7);

            if node_id == 0 {
                match function_code {
                    0x000 => self.parse_nmt_command(&frame.data()).await,
                    0x080 => self.parse_sync().await,
                    _ => {},
                }
            } else if node_id == self.node_id {
                match function_code {
                    0x080 => self.parse_emcy().await,
                    0x200 => self.parse_rpdo(&1, &frame.data()).await,
                    0x300 => self.parse_rpdo(&2, &frame.data()).await,
                    0x400 => self.parse_rpdo(&3, &frame.data()).await,
                    0x500 => self.parse_rpdo(&4, &frame.data()).await,
                    0x600 => self.parse_sdo_client_request(&frame.data()).await,
                    _ => {},
                }
            }

            if cob_id == 0x080 {
                self.update_controller().await;
            }
        }
    }

    async fn parse_nmt_command(&mut self, data: &[u8]) {

        // Check if the data the correct size
        if data.len() != 2 {
            log::error!("Received incorrect frame data length for NMT state change");
        }

        // Extract data
        let requested_state = data[0];
        let addressed_node = data[1];

        // Label NMT command based on requested state
        let nmt_command = match requested_state {
            0x01 => NmtCommand::Start,
            0x02 => NmtCommand::Stop,
            0x80 => NmtCommand::GoToPreOperational,
            0x81 => NmtCommand::Reset,
            0x82 => NmtCommand::ResetCommunication,
            _ => {
                log::warn!("Unknown NMT requested state: {:#X}", requested_state);
                return;
            }
        };

        // Change NMT state
        if addressed_node == self.node_id {

            self.nmt_state = match nmt_command {
                NmtCommand::Start => NmtState::Operational,
                NmtCommand::Stop => NmtState::Stopped,
			    NmtCommand::GoToPreOperational => NmtState::PreOperational,
			    NmtCommand::Reset => NmtState::Initializing,
			    NmtCommand::ResetCommunication => NmtState::Initializing,
            };
            self.send_new_nmt_state().await;

        }

    }

    pub async fn send_new_nmt_state(&mut self) {

        let cob_id = CanId::new_base(0x0700 | self.node_id as u16).unwrap();

        let data: [u8; 1] = [match self.nmt_state {
            NmtState::Initializing => 0x00,
            NmtState::Stopped => 0x04,
            NmtState::Operational => 0x05,
            NmtState::PreOperational => 0x7f,
        }];

        let frame = &CanFrame::new(
            cob_id,
            &data,
            None,
        )
        .unwrap();

        if let Err(_) = self.socket.send(frame).await {
            log::error!("Error sending frame");
        }

        log::info!("New NMT State node {}: {}", self.node_id, self.nmt_state);

    }

    async fn parse_sdo_client_request(&mut self, data: &[u8]) {

        if data.len() > 8 {
            log::error!("Data length too long")
        };

        let ccs = (data[0] >> 5) & 0b111;
        let command = ClientCommand::client_command(ccs);
        self.sdo_response(&command,data).await;

    }

    async fn sdo_response(&mut self, command: &ClientCommand, input_data: &[u8]) {
        let index     = u16::from_le_bytes([input_data[1], input_data[2]]);
        let sub_index = input_data[3];

        let Some(var) = self.eds_data.od
            .get_mut(&index)
            .and_then(|vars| vars.get_mut(&sub_index))
        else { return };

        let mut frame_data = [0u8; 8];
        frame_data[1..3].copy_from_slice(&index.to_le_bytes());
        frame_data[3] = sub_index;

        match command {
            ClientCommand::InitiateUpload => {
                let n = sdo_fill_upload(&var.value, &mut frame_data);
                frame_data[0] = (ServerCommand::InitiateUploadResponse as u8 & 0b111) << 5
                    | (n & 0b11) << 2 | 1 << 1 | 1;
            }
            ClientCommand::InitiateDownload => {
                sdo_write_download(input_data, &mut var.value);
                frame_data[0] = (ServerCommand::InitiateDownloadResponse as u8 & 0b111) << 5;
            }
            _ => { log::error!("Client command not implemented"); return; }
        }

        self.send_sdo_frame(&frame_data).await;
    }

    async fn send_sdo_frame(&self, data: &[u8]) {
        let cob_id = CanId::new_base(0x0580 | self.node_id as u16).unwrap();
        let frame = CanFrame::new(cob_id, data, None).unwrap();
        if let Err(_) = self.socket.send(&frame).await {
            log::error!("Error sending SDO response");
        }
    }

    async fn parse_rpdo(&mut self, rpdo_number: &u16, input_data: &[u8]) {
        let mappings = self.rpdo_mappings(*rpdo_number);
        let mut data = input_data;

        for mapping in &mappings {
            if data.is_empty() { break; }

            let index     = (mapping >> 16) as u16;
            let sub_index = ((mapping >> 8) & 0xFF) as u8;
            let data_type = (mapping & 0xFF) as u8;

            if let Some(var) = self.eds_data.od
                .get_mut(&index)
                .and_then(|vars| vars.get_mut(&sub_index))
            {
                let consumed = decode_rpdo_value(data, data_type, &mut var.value);
                data = drop_front(data, consumed);
            }
        }
    }

    fn rpdo_mappings(&self, rpdo_number: u16) -> Vec<u32> {
        let vars = match self.eds_data.od.get(&(0x1600 | (rpdo_number - 1))) {
            Some(v) => v,
            None => return Vec::new(),
        };

        let count = vars.get(&0)
            .and_then(|v| if let DataValue::Unsigned8(n) = v.value { Some(n) } else { None })
            .unwrap_or(0);

        (1..=count)
            .filter_map(|i| vars.get(&i))
            .filter_map(|v| if let DataValue::Unsigned32(m) = v.value { Some(m) } else { None })
            .collect()
    }

    async fn parse_sync(&self) {
        for tpdo_idx in 0..8u16 {
            if !self.tpdo_is_sync_active(tpdo_idx) {
                continue;
            }
            let payload = self.build_tpdo_payload(tpdo_idx);
            self.send_tpdo(tpdo_idx, &payload).await;
        }
    }

    fn tpdo_is_sync_active(&self, tpdo_idx: u16) -> bool {
        let vars = match self.eds_data.od.get(&(0x1800 + tpdo_idx)) {
            Some(v) => v,
            None => return false,
        };

        let enabled = vars.get(&1)
            .and_then(|v| if let DataValue::Unsigned32(val) = v.value { Some(val) } else { None })
            .map(|val| (val & (1 << 31)) == 0)
            .unwrap_or(false);

        let sync_type = vars.get(&2)
            .and_then(|v| if let DataValue::Unsigned8(val) = v.value { Some(val) } else { None })
            .unwrap_or(0);

        enabled && sync_type == 255
    }

    fn build_tpdo_payload(&self, tpdo_idx: u16) -> Vec<u8> {
        let mut payload = Vec::new();

        let mapping_vars = match self.eds_data.od.get(&(0x1A00 + tpdo_idx)) {
            Some(v) => v,
            None => return payload,
        };

        let count = mapping_vars.get(&0)
            .and_then(|v| if let DataValue::Unsigned8(n) = v.value { Some(n) } else { None })
            .unwrap_or(0);

        for i in 1..=count {
            let mapping = match mapping_vars.get(&i)
                .and_then(|v| if let DataValue::Unsigned32(m) = v.value { Some(m) } else { None })
            {
                Some(m) => m,
                None => continue,
            };

            let index     = (mapping >> 16) as u16;
            let sub_index = ((mapping >> 8) & 0xFF) as u8;
            let data_type = (mapping & 0xFF) as u8;

            if let Some(var) = self.eds_data.od.get(&index).and_then(|vars| vars.get(&sub_index)) {
                encode_tpdo_value(&var.value, data_type, &mut payload);
            }
        }

        payload
    }

    async fn send_tpdo(&self, tpdo_idx: u16, payload: &[u8]) {
        let cob_id = CanId::new_base(0x180 + tpdo_idx * 0x100 | self.node_id as u16).unwrap();
        let frame = CanFrame::new(cob_id, payload, None).unwrap();
        if let Err(_) = self.socket.send(&frame).await {
            log::error!("Error sending TPDO {}", tpdo_idx + 1);
        }
    }

    async fn parse_emcy(&mut self) {

        println!("Emcy");

    }

}

fn drop_front(slice: &[u8], count: usize) -> &[u8] {
    if count > slice.len() {
        &[]
    } else {
        &slice[count..]
    }
}

fn sdo_fill_upload(value: &DataValue, buf: &mut [u8; 8]) -> u8 {
    match value {
        DataValue::Integer8(v)   => { buf[4] = *v as u8;                               3 }
        DataValue::Unsigned8(v)  => { buf[4] = *v;                                     3 }
        DataValue::Integer16(v)  => { buf[4..6].copy_from_slice(&v.to_le_bytes());     2 }
        DataValue::Unsigned16(v) => { buf[4..6].copy_from_slice(&v.to_le_bytes());     2 }
        DataValue::Integer32(v)  => { buf[4..8].copy_from_slice(&v.to_le_bytes());     0 }
        DataValue::Unsigned32(v) => { buf[4..8].copy_from_slice(&v.to_le_bytes());     0 }
        _ => { log::error!("Data type not implemented for SDO upload"); 0 }
    }
}

fn sdo_write_download(input_data: &[u8], target: &mut DataValue) {
    match target {
        DataValue::Integer8(v)   => *v = input_data[4] as i8,
        DataValue::Unsigned8(v)  => *v = input_data[4],
        DataValue::Integer16(v)  => *v = i16::from_le_bytes([input_data[4], input_data[5]]),
        DataValue::Unsigned16(v) => *v = u16::from_le_bytes([input_data[4], input_data[5]]),
        DataValue::Integer32(v)  => *v = i32::from_le_bytes([input_data[4], input_data[5], input_data[6], input_data[7]]),
        DataValue::Unsigned32(v) => *v = u32::from_le_bytes([input_data[4], input_data[5], input_data[6], input_data[7]]),
        _ => log::error!("Data type not implemented for SDO download"),
    }
}

fn decode_rpdo_value(data: &[u8], data_type: u8, target: &mut DataValue) -> usize {
    match (data_type, &*target) {
        (0x08, DataValue::Unsigned8(_))  => { *target = DataValue::Unsigned8(data[0]);                                                    1 }
        (0x08, DataValue::Integer8(_))   => { *target = DataValue::Integer8(data[0] as i8);                                               1 }
        (0x10, DataValue::Unsigned16(_)) => { *target = DataValue::Unsigned16(u16::from_le_bytes([data[0], data[1]]));                    2 }
        (0x10, DataValue::Integer16(_))  => { *target = DataValue::Integer16(i16::from_le_bytes([data[0], data[1]]));                     2 }
        (0x20, DataValue::Unsigned32(_)) => { *target = DataValue::Unsigned32(u32::from_le_bytes([data[0], data[1], data[2], data[3]])); 4 }
        (0x20, DataValue::Integer32(_))  => { *target = DataValue::Integer32(i32::from_le_bytes([data[0], data[1], data[2], data[3]])); 4 }
        _ => { log::error!("Data type 0x{:X} not implemented for value: {:?}", data_type, target); 0 }
    }
}

fn encode_tpdo_value(value: &DataValue, data_type: u8, buf: &mut Vec<u8>) {
    match data_type {
        0x08 => match value {
            DataValue::Unsigned8(v)  => buf.extend(v.to_le_bytes()),
            DataValue::Integer8(v)   => buf.extend(v.to_le_bytes()),
            _ => {},
        },
        0x10 => match value {
            DataValue::Unsigned16(v) => buf.extend(v.to_le_bytes()),
            DataValue::Integer16(v)  => buf.extend(v.to_le_bytes()),
            _ => {},
        },
        0x20 => match value {
            DataValue::Unsigned32(v) => buf.extend(v.to_le_bytes()),
            DataValue::Integer32(v)  => buf.extend(v.to_le_bytes()),
            _ => {},
        },
        _ => {},
    }
}