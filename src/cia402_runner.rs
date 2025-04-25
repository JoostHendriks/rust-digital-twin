use std::time::{Instant, Duration};
use std::collections::{BTreeMap, HashMap};
use std::collections::VecDeque;
use s_curve::*;
use std::sync::Arc;
use tokio::sync::Mutex as TokioMutex;
use can_socket::tokio::CanSocket;
use can_socket::CanFrame;
use tokio::task;
use tokio::time::sleep;

use crate::eds::{get_dataval, get_val, set_dataval, DataValue, EDSData};
use crate::cia301::{Node, consume_read_queue};

pub struct MotorController {
    pub node: Arc<TokioMutex<Node>>,
    pub status: Status,
    pub mode_of_operation_display: ModeOfOperation,
    pub controlword: u16,
    pub command: Command,
    pub state: State,
    pub profile_position_status: ProfilePositionStatus,
    pub profile_velocity_status: ProfileVelocityStatus,
    pub halt: bool,
    pub control_oms1: VecDeque<bool>,
    pub home_status: HomeStatus,
    pub target_reached: bool,
    pub start_travel: bool,
    pub status_oms1: bool,
    pub status_oms2: bool,
    pub timer: Instant,
    pub acceleration: Option<f64>,
    pub max_acceleration: Option<f64>,
    pub profile_velocity: Option<f64>,
    pub actual_position: f64,
    pub actual_velocity: f64,
    pub target_position: Option<f64>,
    pub relative: bool,
    pub motion_map: BTreeMap<usize, f64>,
    pub move_duration: Duration,
    pub target_velocity: Option<f64>,
}

#[derive(Default, PartialEq, Clone)]
pub struct Status {
    pub statusword: u16,
    pub actual_position: i32,
}

/// Operation mode
#[derive(Default, Debug, PartialEq, Clone)]
pub enum ModeOfOperation {
    #[default]
	NoMode = 0,
	ProfilePosition = 1,
	ProfileVelocity = 3,
	Homing = 6,
}

/// Controlword
#[derive(Default, Debug, PartialEq)]
#[allow(dead_code)]
pub enum Command {
    #[default]
    None,
	Shutdown,
	SwitchOn,
	DisableVoltage,
	QuickStop,
	DisableOperation,
	EnableOperation,
	EnableOperationAfterQuickStop,
	FaultReset
}

/// Statusword
#[derive(Default, Debug, PartialEq, Hash, Eq)]
pub enum State {
    #[default]
	NotReadyToSwitchOn,
	SwitchedOnDisabled,
	ReadyToSwitchOn,
	SwitchedOn,
	OperationEnabled,
	QuickStopActive,
	FaultReactionActive,
	Fault
}

/// Homing status
#[derive(Default, Debug)]
pub enum ProfilePositionStatus {
    #[default]
    SetpointAcknownlegde,
    Moving,
}

/// Homing status
#[derive(Default, Debug)]
pub enum ProfileVelocityStatus {
    #[default]
    WaitingForStart,
    RamingUp,
    Rotating,
    RampingDown
}

/// Homing status
#[derive(Default, Debug)]
pub enum HomeStatus {
    #[default]
    WaitingForStart,
    Homing,
}

impl ModeOfOperation {
    fn mode_of_operation(value: i8) -> ModeOfOperation {
        match value {
            0 => ModeOfOperation::NoMode,
            1 => ModeOfOperation::ProfilePosition,
            3 => ModeOfOperation::ProfileVelocity,
            6 => ModeOfOperation::Homing,
            _ => panic!("Mode of operation not implemented")
        }
    }
}

impl MotorController {

    /// Initialize the motor controller.
    pub async fn initialize(
        socket: Arc<TokioMutex<CanSocket>>,
        read_queue: Arc<TokioMutex<Vec<CanFrame>>>,
        node_id: u8,
        eds_data: EDSData,
    ) -> Self {

        let node = Arc::new(TokioMutex::new(Node::new(socket, node_id, eds_data).await));

        consume_read_queue(Arc::clone(&node), read_queue).await;

        let mut controller = Self {
            node,
            status: Default::default(),
            mode_of_operation_display: Default::default(),
            controlword: Default::default(),
            command: Default::default(),
            state: Default::default(),
            profile_position_status: Default::default(),
            profile_velocity_status: Default::default(),
            halt: Default::default(),
            control_oms1: Default::default(),
            home_status: Default::default(),
            target_reached: Default::default(),
            start_travel: Default::default(),
            status_oms1: Default::default(),
            status_oms2: Default::default(),
            timer: Instant::now(),
            acceleration: Default::default(),
            max_acceleration: Default::default(),
            profile_velocity: Default::default(),
            actual_position: Default::default(),
            actual_velocity: Default::default(),
            target_position: Default::default(),
            relative: Default::default(),
            motion_map: BTreeMap::new(),
            move_duration: Default::default(),
            target_velocity: Default::default(),
        };
        controller.control_oms1 = VecDeque::from(vec![false; 2]);
        controller
    }

    async fn update_operation(&mut self, speed_factor: &f64) {

        let mut node = self.node.lock().await;

        match (&self.mode_of_operation_display, &self.state) {

            (ModeOfOperation::ProfilePosition, State::OperationEnabled) => {

                match &self.profile_position_status {

                    ProfilePositionStatus::SetpointAcknownlegde => {

                        if !self.start_travel && self.control_oms1[0] && !self.control_oms1[1] {
                            self.start_travel = true;
                            self.target_reached = false;
                        }

                        if self.start_travel {

                            let acceleration = get_val(0x6083, 0, &mut node.eds_data);
                            let max_acceleration = get_val(0x60C5, 0, &mut node.eds_data);
                            let profile_velocity = get_val(0x6081, 0, &mut node.eds_data);
                            let target_position = get_val(0x607A, 0, &mut node.eds_data);

                            match (acceleration, max_acceleration, profile_velocity, target_position) {
                                (Some(acceleration), Some(max_acceleration), Some(profile_velocity), Some(target_position)) => {

                                    log::debug!("Trying to move node {} with: max_acc: {} acc: {}, vel: {}, curr: {}, dest: {}", node.id, max_acceleration, acceleration, profile_velocity, self.actual_position, target_position);

                                    match position_motion_map(node.id, &acceleration, &max_acceleration, &profile_velocity, &self.actual_position, &target_position, &self.relative, speed_factor) {
                                        Ok(motion_map) => {
                                            self.motion_map = motion_map;
                                            self.max_acceleration = None;
                                            self.acceleration = None;
                                            self.profile_velocity = None;
                                            self.target_position = None;
                                            self.status_oms1 = true;
                                            self.start_travel = false;
                                            self.timer = Instant::now();
                                            self.profile_position_status = ProfilePositionStatus::Moving;
                                        }
                                        Err(_e) => {
                                            self.status_oms1 = false;
                                        }
                                    }
                                }
                                _ => {},
                            }
                        }
                    }

                    ProfilePositionStatus::Moving => {

                        self.target_reached = false;

                        let elapsed_time = self.timer.elapsed().as_millis() as usize;
                        if let Some(just_passed_point) = self.motion_map.range(..=elapsed_time).next_back().map(|(&key, _)| key) {
                            if let Some(new_actual_position) = self.motion_map.get(&just_passed_point) {

                                log::info!("Actual position node {}: {}", node.id, new_actual_position);

                                if self.actual_position != *new_actual_position {
                                    self.actual_position = *new_actual_position;

                                }
                            }
                        }

                        let (end_time, _end_position) = self.motion_map.last_key_value().unwrap();

                        if self.timer.elapsed() > Duration::from_millis(*end_time as u64) {
                            self.target_reached = true;
                            self.profile_position_status = ProfilePositionStatus::SetpointAcknownlegde
                        }

                    }

                }

            }

            (ModeOfOperation::ProfileVelocity, State::OperationEnabled) => {

                match &self.profile_velocity_status {

                    ProfileVelocityStatus::WaitingForStart => {

                        self.target_reached = true;

                        if !&self.halt {

                            match (&self.max_acceleration, &self.acceleration, &self.target_velocity) {
                                (Some(max_acceleration), Some(mut acceleration), Some(target_velocity)) => {
                                    acceleration = match acceleration {
                                        0. => max_acceleration.clone(),
                                        _ => acceleration,
                                    };
                                    self.move_duration = Duration::from_millis((target_velocity / acceleration * 1000.) as u64);
                                    self.max_acceleration = None;
                                    self.acceleration = None;
                                    self.target_velocity = None;
                                    self.timer = Instant::now();
                                    self.profile_velocity_status = ProfileVelocityStatus::RamingUp
                                }
                                _ => {},
                            }
                        }
                    }

                    ProfileVelocityStatus::RamingUp => {


                        self.target_reached = false;

                        if self.timer.elapsed() > self.move_duration {
                            self.profile_velocity_status = ProfileVelocityStatus::Rotating
                        } else if self.halt {
                            self.timer = Instant::now();
                            self.profile_velocity_status = ProfileVelocityStatus::RampingDown
                        }

                    }

                    ProfileVelocityStatus::Rotating => {

                        self.target_reached = true;

                        match &self.target_velocity {
                            Some(target_velocity) => self.actual_velocity = target_velocity.clone(),
                            _ => {},
                        }

                        if self.halt {

                            match (&self.max_acceleration, &self.acceleration) {
                                (Some(max_acceleration), Some(mut acceleration)) => {
                                    acceleration = match acceleration {
                                        0. => max_acceleration.clone(),
                                        _ => acceleration,
                                    };
                                    self.move_duration = Duration::from_millis((self.actual_velocity / acceleration * 1000.) as u64);
                                    self.max_acceleration = None;
                                    self.acceleration = None;
                                    self.target_velocity = None;
                                    self.timer = Instant::now();
                                    self.profile_velocity_status = ProfileVelocityStatus::RampingDown
                                }
                                _ => {},
                            }
                        }

                    }

                    ProfileVelocityStatus::RampingDown => {

                        self.target_reached = false;

                        if self.timer.elapsed() > self.move_duration {
                            self.profile_velocity_status = ProfileVelocityStatus::WaitingForStart
                        }

                    }

                }

            }

            (ModeOfOperation::Homing, State::OperationEnabled) => {

                match &self.home_status {

                    HomeStatus::WaitingForStart => {

                        self.target_reached = true;
                        self.status_oms2 = false;

                        if node.id == 1 {
                            println!("Waiting to home");
                        }

                        if self.control_oms1[0] && !self.control_oms1[1] {
                            if node.id == 1 {
                                println!("Start homing");
                            }
                            self.timer = Instant::now();
                            self.home_status = HomeStatus::Homing
                        }
                    }
                    HomeStatus::Homing => {

                        self.target_reached = false;
                        self.status_oms1 = false;
                        self.status_oms2 = false;

                        if self.timer.elapsed() > Duration::from_millis(100) {

                            self.target_reached = true;
                            self.status_oms1 = true;
                            self.status_oms2 = false;

                            self.home_status = HomeStatus::WaitingForStart
                        }
                    }

                }
            }

            _ => {},
        }

    }

    async fn update_command(&mut self) {

        let mut node = self.node.lock().await;

        let controlword = match get_dataval(0x6040, 0, &mut node.eds_data) {
            Some(DataValue::Unsigned16(value)) => value,
            _ => panic!("Controlword not found"),
        };

        const BIT_INDICES: [usize; 5] = [0, 1, 2, 3, 7];
        
        let bits: Vec<bool> = BIT_INDICES.iter().map(|&i| get_bit_16(&controlword, i)).collect();
    
        self.command = match (bits[4], bits[3], bits[2], bits[1], bits[0]) {
            (false, _, true, true, false) => Command::Shutdown,
            (false, false, true, true, true) => Command::SwitchOn,
            (false, _, _, false, _) => Command::DisableVoltage,
            (false, _, false, true, _) => Command::QuickStop,
            (false, true, true, true, true) => Command::EnableOperation,
            (true, _, _, _, _) => Command::FaultReset,
        };

        self.control_oms1.push_front(get_bit_16(&self.controlword, 4));
        self.control_oms1.pop_back();

        self.relative = get_bit_16(&self.controlword, 6);
        self.halt = get_bit_16(&self.controlword, 8);
    }

    async fn update_mode_of_operation(&mut self) {

        let mut node = self.node.lock().await;

        let mode_of_operation = match get_dataval(0x6060, 0, &mut node.eds_data) {
            Some(DataValue::Integer8(value)) => ModeOfOperation::mode_of_operation(value),
            _ => panic!("Mode of operation not found"),
        };

        if self.mode_of_operation_display != mode_of_operation {

            self.mode_of_operation_display = mode_of_operation;

            self.profile_position_status = ProfilePositionStatus::SetpointAcknownlegde;
            self.profile_velocity_status = ProfileVelocityStatus::WaitingForStart;

            set_dataval(0x6061, 0, DataValue::Integer8(self.mode_of_operation_display.clone() as i8), &mut node.eds_data);

        }
    }

    fn update_state(&mut self) {

        self.state = match self.state {
            State::NotReadyToSwitchOn => State::SwitchedOnDisabled,
            State::SwitchedOnDisabled => match &self.command {
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOnDisabled,
            }
            State::ReadyToSwitchOn => match &self.command {
                Command::SwitchOn => State::SwitchedOn,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                _ => State::ReadyToSwitchOn,
            }
            State::SwitchedOn => match &self.command {
                Command::EnableOperation => State::OperationEnabled,
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOn,
            }
            State::OperationEnabled => match &self.command {
                Command::QuickStop => State::QuickStopActive,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::SwitchOn => State::SwitchedOn,
                _ => State::OperationEnabled,
            }
            State::QuickStopActive => match &self.command {
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::EnableOperationAfterQuickStop => State::OperationEnabled,
                _ => State::QuickStopActive,
            }
            State::FaultReactionActive => State::Fault,
            State::Fault => match &self.command {
                Command::FaultReset => State::SwitchedOnDisabled,
                _ => State::Fault,
            }

        };

    }
    
    async fn set_statusword(&mut self) {

        let mut node = self.node.lock().await;

        let mut statusword = self.status.statusword;

        let bit_configs: HashMap<State, Vec<(usize, bool)>> = HashMap::from([
            (State::NotReadyToSwitchOn, vec![(0, false), (1, false), (2, false), (3, false), (5, false), (6, false)]),
            (State::SwitchedOnDisabled, vec![(0, false), (1, false), (2, false), (3, false), (6, true)]),
            (State::ReadyToSwitchOn, vec![(0, true), (1, false), (2, false), (3, false), (5, true), (6, false)]),
            (State::SwitchedOn, vec![(0, true), (1, true), (2, false), (3, false), (5, true), (6, false)]),
            (State::OperationEnabled, vec![(0, true), (1, true), (2, true), (3, false), (5, true), (6, false)]),
            (State::QuickStopActive, vec![(0, true), (1, true), (2, true), (3, false), (5, false), (6, false)]),
            (State::FaultReactionActive, vec![(0, true), (1, true), (2, true), (3, true), (6, false)]),
            (State::Fault, vec![(0, false), (1, false), (2, false), (3, true), (6, false)]),
        ]);
    
        if let Some(bits) = bit_configs.get(&self.state) {
            set_bits(&mut statusword, bits);
        }
    
        statusword = set_bit_16(&statusword, 10, self.target_reached);
        statusword = set_bit_16(&statusword, 12, self.status_oms1);
        statusword = set_bit_16(&statusword, 13, self.status_oms2);

        if self.status.statusword != statusword {

            self.status.statusword = statusword;

            set_dataval(0x6041, 0, DataValue::Unsigned16(self.status.statusword.clone()), &mut node.eds_data);

        }
    }

}

fn get_bit_16(u16_value: &u16, index: usize) -> bool {
    let mask = 1 << index;
    (u16_value & mask) != 0
}

fn set_bit_16(u16_value: &u16, bit_position: usize, value: bool) -> u16 {
    let mask = 1 << bit_position;
    if value {
        u16_value | mask
    } else {
        u16_value & !mask
    }
}

fn set_bits(statusword: &mut u16, bits: &[(usize, bool)]) {
    for &(bit, value) in bits {
        *statusword = set_bit_16(statusword, bit, value);
    }
}

fn position_motion_map(
    node_id: u8, 
    profile_acceleration: &f64, 
    max_acceleration: &f64, 
    profile_velocity: &f64, 
    actual_position: &f64, 
    target_position: &f64, 
    relative: &bool,
    speed_factor: &f64
) -> Result<BTreeMap<usize, f64>, String> {

    if *max_acceleration == 0. {
        return Err(format!("Max acceleration 0 for node: {}", node_id));
    }

    let acceleration = match profile_acceleration {
        0. => max_acceleration.clone(),
        _ => profile_acceleration.clone(),
    };

    if acceleration == 0. || *profile_velocity == 0. {
        return Err(format!("Target position invalid for node: {}", node_id));
    }

    const RPM_TO_RPS: f64 = 1./60.;
    const INC_PER_ROT: f64 = 3600.;
    const SEC_TO_MSEC: f64 = 1000.;

    let constraints = SCurveConstraints {
        max_jerk: 10.,
        max_acceleration: acceleration * RPM_TO_RPS,
        max_velocity: profile_velocity * RPM_TO_RPS,
    };

    let travel_distance = target_position - actual_position;

    let mut motion_map: BTreeMap<usize, f64> = BTreeMap::new();

    if travel_distance == 0. {
        motion_map.insert(0, 0.);
        return Ok(motion_map);
    }

    let actual_position_rot = actual_position / INC_PER_ROT;
    let target_position_rot = target_position / INC_PER_ROT;
    
    let q1 = match *relative {
        true => actual_position_rot + target_position_rot,
        false => target_position_rot,
    };

    let  start_conditions = SCurveStartConditions {
        q0: actual_position_rot, // start position
        q1, // end position
        v0: 0., // start velocity
        v1: 0. // end velocity
    };

    let input  =  SCurveInput{constraints, start_conditions};
    let (params, s_curve) = s_curve_generator(&input, Derivative::Position);

    let points = (travel_distance.abs() / 10.) as u64;

    for point in 0..(points + 1) {
        let time = (point as f64 * ((params.time_intervals.total_duration() * SEC_TO_MSEC) / speed_factor) / points as f64) as usize;
        let position = s_curve(point as f64 * params.time_intervals.total_duration() / points as f64) * INC_PER_ROT;
        motion_map.insert(time, position);
    }

    let total_duration = params.time_intervals.total_duration();

    log::debug!("Duration move: {}", total_duration);

    Ok(motion_map)
}


pub async fn run(controller: Arc<TokioMutex<MotorController>>, speed_factor: &f64) {
    task::spawn(async move {
        loop {
            let mut controller = controller.lock().await;
            controller.update_mode_of_operation().await;
            // self.update_command().await;
            // self.update_state();
            // self.update_operation(speed_factor).await;
            // self.set_statusword().await;

        sleep(Duration::from_millis(10)).await;
        }
    });
}