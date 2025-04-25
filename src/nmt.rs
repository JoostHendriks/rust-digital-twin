use canopen_tokio::nmt::{NmtCommand, NmtState};
use can_socket::{CanFrame, CanId};

pub fn new_nmt_state(requested_state: u8) -> Result<NmtState, String> {

    // Label NMT command based on requested state
    let nmt_command = match requested_state {
        0x01 => NmtCommand::Start,
        0x02 => NmtCommand::Stop,
        0x80 => NmtCommand::GoToPreOperational,
        0x81 => NmtCommand::Reset,
        0x82 => NmtCommand::ResetCommunication,
        _ => return Err(format!("Unexpected requested state: {:#X}", requested_state)),
    };

    // Change NMT state
    let new_nmt_state = match nmt_command {
        NmtCommand::Start => NmtState::Operational,
        NmtCommand::Stop => NmtState::Stopped,
        NmtCommand::GoToPreOperational => NmtState::PreOperational,
        NmtCommand::Reset => NmtState::Initializing,
        NmtCommand::ResetCommunication => NmtState::Initializing,
    };
    Ok(new_nmt_state)

}

pub fn create_nmt_frame(node_id: u8, nmt_state: NmtState) -> CanFrame {

    let cob = u16::from_str_radix("700", 16).unwrap();
    let cob_id = CanId::new_base(cob | node_id as u16).unwrap();

    let data: [u8; 1] = [match nmt_state {
        NmtState::Initializing => 0x00,
        NmtState::Stopped => 0x04,
        NmtState::Operational => 0x05,
        NmtState::PreOperational => 0x7f,
    }];

    CanFrame::new(
        cob_id,
        &data,
        None,
    )
    .unwrap()

}