use std::collections::BTreeMap;
use std::sync::Arc;
use tokio::task;
use tokio::sync::Mutex as TokioMutex;
use can_socket::{tokio::CanSocket, CanId};
use can_socket::CanFrame;
use canopen_tokio::nmt::NmtState;

use crate::eds::{DataValue, EDSData};
use crate::nmt;
use crate::sdo;

pub struct Node {
    pub id: u8,
    pub eds_data: EDSData,
    pub nmt_state: NmtState,
    pub socket: Arc<TokioMutex<CanSocket>>,
}

impl Node {

    pub async fn new(
        socket: Arc<TokioMutex<CanSocket>>,
        node_id: u8,
        eds_data: EDSData,
    ) -> Self {
        let node = Self {
            id: node_id,
            eds_data,
            nmt_state: NmtState::Initializing,
            socket,
        };
        node
    }

    // async fn handle_frame(&mut self, frame: &CanFrame) {

    //     // Extract id and cob_id
    //     let cob_id = frame.id().as_u32();
    //     let node_id = (cob_id & 0x7F) as u8;
    //     let function_code = frame.id().as_u32() & (0x0F << 7);

    //     // Parse frame
    //     if node_id == 0 {

    //         match function_code {
    //             0x000 => self.parse_nmt_command(&frame.data()).await,
    //             0x080 => self.parse_sync().await,
    //             _ => {},
    //         }

    //     } else if node_id == self.id {

    //         match function_code {
    //             0x080 => self.parse_emcy().await,
    //             0x200 => self.parse_rpdo(&1, &frame.data()).await,
    //             0x300 => self.parse_rpdo(&2, &frame.data()).await,
    //             0x400 => self.parse_rpdo(&3, &frame.data()).await,
    //             0x500 => self.parse_rpdo(&4, &frame.data()).await,
    //             0x600 => self.parse_sdo_client_request(&frame.data()).await,
    //             _ => {},
    //         }
            
    //     }

    // }

    async fn parse_nmt_command(&mut self, data: &[u8]) -> bool {

        // Check if the data the correct size
        if data.len() != 2 {
            log::error!("Received incorrect frame data length for NMT state change");
        }

        let requested_state = data[0];
        let addressed_node = data[1];

        if addressed_node == self.id {
            match nmt::new_nmt_state(requested_state) {
                Ok(new_nmt_state) => {
                    self.nmt_state = new_nmt_state;
                    let frame = nmt::create_nmt_frame(self.id, self.nmt_state);
                    if let Err(err) = self.socket.lock().await.send(&frame).await {
                        log::error!("Failed to send NMT frame: {err}");
                    }
                    return true
                }
                Err(e) => log::error!("Changing nmt state failed, with error: {e}"),
            }
        }
        false
    }

    async fn parse_sdo_client_request(&mut self, data: &[u8]) {

        match sdo::sdo_response(self.id, &mut self.eds_data, data) {
            Ok(frame) => {
                if let Err(err) = self.socket.lock().await.send(&frame).await {
                    log::error!("Failed to send sdo frame: {err}");
                }
            }
            Err(()) => log::error!("Failed to process sdo frame"),
        }
    }

    async fn parse_rpdo(&mut self, rpdo_number: &u16, input_data: &[u8]) {

        let mut enabled_sub_indices: u8 = 0;
        let mut rpdo_indices: BTreeMap<u8, u32> = BTreeMap::new();

        if let Some(var) = self.eds_data.od.get(&(0x1600 | (*rpdo_number - 1)))
            .and_then(|vars| vars.get(&0)) {
                match var.value {
                    DataValue::Unsigned8(value) => {
                        enabled_sub_indices = value;
                    }
                    _ => {},
                }
            }

        if let Some(vars) = self.eds_data.od.get(&(0x1600 | (*rpdo_number - 1))) {
            for (sub_index, var) in vars.iter() {
                if *sub_index != 0 as u8 {
                    match var.value {
                        DataValue::Unsigned32(value) => {
                            rpdo_indices.insert(*sub_index, value);
                        }
                        _ => {},
                    }
                }
            }
        }

        let mut data = input_data;

        for i in 0..enabled_sub_indices {

            if let Some(rpdo_index_value) = rpdo_indices.get(&(i + 1)) {

                let index_to_set = (rpdo_index_value >> 16) as u16;
                let sub_index_to_set = ((rpdo_index_value >> 8) & 0xFF) as u8;
                let data_type = (rpdo_index_value & 0xFF) as u8;

                if let Some(vars) = self.eds_data.od.get_mut(&index_to_set) {

                    for (sub_index, var) in vars.iter_mut() {

                        if sub_index == &sub_index_to_set {

                            // Break loop when there is no data left
                            if data == &[] {
                                break;
                            }

                            match (data_type, &var.value) {
                                (0x08, DataValue::Unsigned8(_)) => {
                                    var.value = DataValue::Unsigned8(data[0]);
                                    data = drop_front(data, 1);
                                }
                                (0x08, DataValue::Integer8(_)) => {
                                    var.value = DataValue::Integer8(data[0] as i8);
                                    data = drop_front(data, 1);
                                }
                                (0x10, DataValue::Unsigned16(_)) => {
                                  var.value = DataValue::Unsigned16(u16::from_le_bytes([data[0], data[1]]));
                                    data = drop_front(data, 2);
                                }
                                (0x10, DataValue::Integer16(_)) => {
                                    var.value = DataValue::Integer16(i16::from_le_bytes([data[0], data[1]]));
                                    data = drop_front(data, 2);
                                }
                                (0x20, DataValue::Unsigned32(_)) => {
                                    var.value = DataValue::Unsigned32(u32::from_le_bytes([data[0], data[1], data[2], data[3]]));
                                    data = drop_front(data, 4);
                                }
                                (0x20, DataValue::Integer32(_)) => {
                                    var.value = DataValue::Integer32(i32::from_le_bytes([data[0], data[1], data[2], data[3]]));
                                    data = drop_front(data, 4);
                                }
                                _ => log::error!("Data type not implemented. Data type: 0x{:X}, data value: {:?}", data_type, var.value)
                            };
                        }
                    }
                }
            }
        }
    }  

    async fn parse_sync(&self) {
        let mut tpdos_enabled = BTreeMap::new();
        let mut tpdos_sync_type = BTreeMap::new();
        let mut number_of_entries = BTreeMap::new();
        let mut tpdo_objects = BTreeMap::new();
    
        for i in 0..8 {
            self.extract_tpdo_data(i, &mut tpdos_enabled, &mut tpdos_sync_type);
            self.extract_entry_count(i, &mut number_of_entries);
        }
    
        for (&tpdo_number, &tpdo_enabled) in tpdos_enabled.iter() {
            if let Some(&sync_type) = tpdos_sync_type.get(&tpdo_number) {
                if tpdo_enabled && sync_type == 0x01 {
                    self.collect_tpdo_objects(tpdo_number, number_of_entries.get(&tpdo_number), &mut tpdo_objects);
                }
            }
        }
    
        for (&tpdo_number, tpdo_sub_indices) in tpdo_objects.iter() {
            self.process_and_send_tpdo(tpdo_number, tpdo_sub_indices).await;
        }
    }
    
    fn extract_tpdo_data(&self, index: u16, tpdos_enabled: &mut BTreeMap<u16, bool>, tpdos_sync_type: &mut BTreeMap<u16, u8>) {
        if let Some(vars) = self.eds_data.od.get(&(0x1800 + index)) {
            for (&sub_index, var) in vars.iter() {
                match (sub_index, &var.value) {
                    (1, DataValue::Unsigned32(value)) => {
                        tpdos_enabled.insert(index, (value & (1 << 31)) == 0);
                    }
                    (2, DataValue::Unsigned8(value)) => {
                        tpdos_sync_type.insert(index, *value);
                    }
                    _ => {}
                }
            }
        }
    }
    
    fn extract_entry_count(&self, index: u16, number_of_entries: &mut BTreeMap<u16, u8>) {
        if let Some(vars) = self.eds_data.od.get(&(0x1A00 + index)) {
            for (&sub_index, var) in vars.iter() {
                if sub_index == 0 {
                    if let DataValue::Unsigned8(value) = var.value {
                        number_of_entries.insert(index, value);
                    }
                }
            }
        }
    }
    
    fn collect_tpdo_objects(
        &self,
        tpdo_number: u16,
        entry_count: Option<&u8>,
        tpdo_objects: &mut BTreeMap<u16, BTreeMap<u8, u32>>,
    ) {
        if let Some(&entries) = entry_count {
            if let Some(vars) = self.eds_data.od.get(&(0x1A00 + tpdo_number)) {
                for i in 1..=entries {
                    if let Some(var) = vars.get(&i) {
                        if let DataValue::Unsigned32(value) = var.value {
                            tpdo_objects.entry(tpdo_number).or_default().insert(i, value);
                        }
                    }
                }
            }
        }
    }
    
    async fn process_and_send_tpdo(&self, tpdo_number: u16, tpdo_sub_indices: &BTreeMap<u8, u32>) {
        let mut data_to_send = Vec::new();
    
        for (&_sub_index, &tpdo_content) in tpdo_sub_indices {
            if let Some(data) = self.extract_tpdo_content(tpdo_content) {
                data_to_send.extend(data);
            }
        }
    
        let functions_code = u16::from_str_radix(&format!("{:X}80", tpdo_number + 1), 16).unwrap();
        let cob_id = CanId::new_base(functions_code | self.id as u16).unwrap();
    
        if let Err(_) = self.socket.lock().await.send(&CanFrame::new(cob_id, &data_to_send, None).unwrap()).await {
            log::error!("Error sending frame");
        }
    }
    
    fn extract_tpdo_content(&self, tpdo_content: u32) -> Option<Vec<u8>> {
        let index_to_find = (tpdo_content >> 16) as u16;
        let sub_index_to_find = ((tpdo_content >> 8) & 0xFF) as u8;
        let data_type = (tpdo_content & 0xFF) as u8;
    
        if let Some(vars) = self.eds_data.od.get(&index_to_find) {
            if let Some(var) = vars.get(&sub_index_to_find) {
                return match (data_type, &var.value) {
                    (0x08, DataValue::Unsigned8(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x08, DataValue::Integer8(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x10, DataValue::Unsigned16(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x10, DataValue::Integer16(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x20, DataValue::Unsigned32(value)) => Some(value.to_le_bytes().to_vec()),
                    (0x20, DataValue::Integer32(value)) => Some(value.to_le_bytes().to_vec()),
                    _ => None,
                };
            }
        }
        None
    }

    async fn parse_emcy(&mut self) {

        println!("Emcy");

    }

}

pub async fn consume_read_queue(node_clone: Arc<TokioMutex<Node>>, queue_clone: Arc<TokioMutex<Vec<CanFrame>>>) {
    task::spawn(async move {
        loop {
            let mut queue = queue_clone.lock().await;
            let mut i = 0;
            while i < queue.len() {
                let frame = &queue[i];
                let cob_id = frame.id().as_u32();
                let function_code = cob_id & !0x7F;
                let id = (cob_id & 0x7F) as u8;
                let mut node = node_clone.lock().await;
                let node_id = node.id;

                if id == node_id {
                    match function_code {
                        // TPDO frames
                        0x180 | 0x280 | 0x380 | 0x480 => {
                            queue.remove(i);
                            continue;
                        }
                        // RPDO frames
                        0x200 => {
                            node.parse_rpdo(&1, &frame.data()).await;
                            queue.remove(i);
                            continue;
                        }
                        0x300 => {
                            node.parse_rpdo(&2, &frame.data()).await;
                            queue.remove(i);
                            continue;
                        }
                        0x400 => {
                            node.parse_rpdo(&3, &frame.data()).await;
                            queue.remove(i);
                            continue;
                        }
                        0x500 => {
                            node.parse_rpdo(&4, &frame.data()).await;
                            queue.remove(i);
                            continue;
                        }
                        // SDO frames
                        0x580 => {
                            queue.remove(i);
                            continue;
                        }
                        0x600 => {
                            node.parse_sdo_client_request(&frame.data()).await;
                            queue.remove(i);
                            continue;
                        }
                        // Heartbeat
                        0x700 => {
                            queue.remove(i);
                            continue;
                        }
                        _ => {}
                    }
                } else {
                    match function_code {
                        // NMT control
                        0x000 => {
                            match node.parse_nmt_command(&frame.data()).await {
                                true => {
                                    queue.remove(i);
                                    continue;
                                }
                                false => break,
                            }
                        }
                        // Sync
                        0x080 => {
                            node.parse_sync().await;
                            queue.remove(i);
                            continue;
                        }
                        _ => {},
                    }
                }
                i += 1; // Only increment if nothing was removed
            }

            // if !queue.is_empty() {
            //     for frame in queue.iter() {
            //         println!("{:?}", frame.id());
            //     }
            // }
        }
    });
}

fn drop_front(slice: &[u8], count: usize) -> &[u8] {
    if count > slice.len() {
        &[]
    } else {
        &slice[count..]
    }
}