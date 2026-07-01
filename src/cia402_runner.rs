use std::time::{Instant, Duration};

use crate::cia301::Node;
use crate::eds::DataValue;

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
pub enum Command {
    #[default]
    None,
	Shutdown,
	SwitchOn,
	DisableVoltage,
	QuickStop,
	EnableOperation,
	EnableOperationAfterQuickStop,
	FaultReset
}

/// Statusword
#[derive(Default, Debug, PartialEq)]
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
    AxisSpeedZero,
    TargetSpeedNotReached,
    TargetSpeedReached,
    AxisBraking,
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

impl Node {

    /// Compute expected travel time for a trapezoidal (or triangular) velocity
    /// profile using the standard CIA 402 OD objects:
    ///   0x6064 – position actual value (Integer32)
    ///   0x607A – target position      (Integer32)
    ///   0x6081 – profile velocity     (Unsigned32, units/s)
    ///   0x6083 – profile acceleration (Unsigned32, units/s²)
    ///   0x6084 – profile deceleration (Unsigned32, units/s²)
    fn compute_movement_duration(&self) -> Duration {
        let get_i32 = |index: u16| -> Option<f64> {
            self.eds_data.od.get(&index)?.get(&0).and_then(|v| {
                if let DataValue::Integer32(x) = v.value { Some(x as f64) } else { None }
            })
        };
        let get_u32 = |index: u16| -> Option<f64> {
            self.eds_data.od.get(&index)?.get(&0).and_then(|v| {
                if let DataValue::Unsigned32(x) = v.value { Some(x as f64) } else { None }
            })
        };

        let actual = get_i32(0x6064).unwrap_or(0.0);
        let target = get_i32(0x607A).unwrap_or(0.0);
        let vel    = get_u32(0x6081).unwrap_or(0.0);
        let acc    = get_u32(0x6083).unwrap_or(0.0);
        let dec    = get_u32(0x6084).unwrap_or(0.0);

        let delta = (target - actual).abs();

        if delta == 0.0 || vel <= 0.0 || acc <= 0.0 || dec <= 0.0 {
            return Duration::ZERO;
        }

        let d_acc = vel * vel / (2.0 * acc);
        let d_dec = vel * vel / (2.0 * dec);

        let secs = if d_acc + d_dec <= delta {
            // Trapezoidal profile: reaches full profile velocity
            vel / acc + (delta - d_acc - d_dec) / vel + vel / dec
        } else {
            // Triangular profile: peak velocity is lower than profile velocity
            let v_peak = (2.0 * delta * acc * dec / (acc + dec)).sqrt();
            v_peak / acc + v_peak / dec
        };

        Duration::from_secs_f64(secs / self.time_scale)
    }

    fn compute_acceleration_duration(&self) -> Duration {
        let get_i32 = |index: u16| -> Option<f64> {
            self.eds_data.od.get(&index)?.get(&0).and_then(|v| {
                if let DataValue::Integer32(x) = v.value { Some(x as f64) } else { None }
            })
        };
        let get_u32 = |index: u16| -> Option<f64> {
            self.eds_data.od.get(&index)?.get(&0).and_then(|v| {
                if let DataValue::Unsigned32(x) = v.value { Some(x as f64) } else { None }
            })
        };

        let vel = get_i32(0x60FF).unwrap_or(0.0).abs();
        let acc = get_u32(0x6083).unwrap_or(0.0);

        if vel <= 0.0 || acc <= 0.0 {
            return Duration::ZERO;
        }

        Duration::from_secs_f64(vel / acc / self.time_scale)
    }

    fn compute_deceleration_duration(&self) -> Duration {
        let get_i32 = |index: u16| -> Option<f64> {
            self.eds_data.od.get(&index)?.get(&0).and_then(|v| {
                if let DataValue::Integer32(x) = v.value { Some(x as f64) } else { None }
            })
        };
        let get_u32 = |index: u16| -> Option<f64> {
            self.eds_data.od.get(&index)?.get(&0).and_then(|v| {
                if let DataValue::Unsigned32(x) = v.value { Some(x as f64) } else { None }
            })
        };

        let vel = get_i32(0x60FF).unwrap_or(0.0).abs();
        let dec = get_u32(0x6084).unwrap_or(0.0);

        if vel <= 0.0 || dec <= 0.0 {
            return Duration::ZERO;
        }

        Duration::from_secs_f64(vel / dec / self.time_scale)
    }

    pub async fn update_controller(&mut self) {

        if let Some(var) = self.eds_data.od.get(&0x6060)
            .and_then(|vars| vars.get(&0)) {
                match var.value {
                    DataValue::Integer8(value) => {
                        self.motor_controller.mode_of_operation = ModeOfOperation::mode_of_operation(value);
                    }
                    _ => {},
                }
            }

        if let Some(var) = self.eds_data.od.get(&0x6040)
            .and_then(|vars| vars.get(&0)) {
                match var.value {
                    DataValue::Unsigned16(value) => {
                        self.motor_controller.controlword = value;
                    }
                    _ => {},
                }
            }

        // Do logic based on input
        self.parse_controlword();
        self.update_state();

        match (&self.motor_controller.mode_of_operation, &self.motor_controller.state) {

            (ModeOfOperation::ProfilePosition, State::OperationEnabled) => {
                self.motor_controller.status_oms1 = self.motor_controller.control_oms1[1];
                match &self.motor_controller.profile_position_status {
                    ProfilePositionStatus::SetpointAcknownlegde => {
                        if self.motor_controller.control_oms1[0] && !self.motor_controller.control_oms1[1] {
                            self.motor_controller.target_reached = false;
                            self.motor_controller.timer = Some(Instant::now());
                            self.motor_controller.movement_duration = Some(self.compute_movement_duration());
                            self.motor_controller.profile_position_status = ProfilePositionStatus::Moving;
                        } else {
                            self.motor_controller.target_reached = true;
                        }
                    }
                    ProfilePositionStatus::Moving => {
                        let elapsed = self.motor_controller.timer.unwrap().elapsed();
                        let duration = self.motor_controller.movement_duration.unwrap_or(Duration::ZERO);
                        if elapsed >= duration {
                            self.motor_controller.profile_position_status = ProfilePositionStatus::SetpointAcknownlegde;
                        }
                    }

                }

            }

            (ModeOfOperation::ProfileVelocity, State::OperationEnabled) => {
                match &self.motor_controller.profile_velocity_status {
                    ProfileVelocityStatus::AxisSpeedZero => {
                        self.motor_controller.target_reached = true;
                        if !self.motor_controller.halt {
                            self.motor_controller.timer = Some(Instant::now());
                            self.motor_controller.movement_duration = Some(self.compute_acceleration_duration());
                            self.motor_controller.profile_velocity_status = ProfileVelocityStatus::TargetSpeedNotReached
                        }
                    }
                    ProfileVelocityStatus::TargetSpeedNotReached => {
                        self.motor_controller.target_reached = false;
                        let elapsed = self.motor_controller.timer.unwrap().elapsed();
                        let duration = self.motor_controller.movement_duration.unwrap_or(Duration::ZERO);
                        if elapsed >= duration {
                            self.motor_controller.profile_velocity_status = ProfileVelocityStatus::TargetSpeedReached
                        }
                    }
                    ProfileVelocityStatus::TargetSpeedReached => {
                        self.motor_controller.target_reached = true;
                        if self.motor_controller.halt {
                            self.motor_controller.timer = Some(Instant::now());
                            self.motor_controller.movement_duration = Some(self.compute_deceleration_duration());
                            self.motor_controller.profile_velocity_status = ProfileVelocityStatus::AxisBraking
                        }
                    }
                    ProfileVelocityStatus::AxisBraking => {
                        self.motor_controller.target_reached = false;
                        let elapsed = self.motor_controller.timer.unwrap().elapsed();
                        let duration = self.motor_controller.movement_duration.unwrap_or(Duration::ZERO);
                        if elapsed >= duration {
                            self.motor_controller.profile_velocity_status = ProfileVelocityStatus::AxisSpeedZero
                        }
                    }
                }
            }

            (ModeOfOperation::Homing, State::OperationEnabled) => {
                match &self.motor_controller.home_status {
                    HomeStatus::WaitingForStart => {
                        self.motor_controller.target_reached = true;
                        self.motor_controller.status_oms2 = false;
                        if self.motor_controller.control_oms1[0] && !self.motor_controller.control_oms1[1] {
                            self.motor_controller.timer = Some(Instant::now());
                            self.motor_controller.home_status = HomeStatus::Homing
                        }
                    }
                    HomeStatus::Homing => {
                        self.motor_controller.target_reached = false;
                        self.motor_controller.status_oms1 = false;
                        self.motor_controller.status_oms2 = false;
                        if self.motor_controller.timer.unwrap().elapsed() > Duration::from_millis(100) {
                            self.motor_controller.target_reached = true;
                            self.motor_controller.status_oms1 = true;
                            self.motor_controller.status_oms2 = false;
                            self.motor_controller.home_status = HomeStatus::WaitingForStart
                        }
                    }
                }
            }

            _ => {},
        }
        
        self.set_statusword();

        // Adjust eds according to motor controller status
        if let Some(var) = self.eds_data.od.get_mut(&0x6061)
            .and_then(|vars| vars.get_mut(&0)) {
                match var.value {
                    DataValue::Integer8(_) => var.value = DataValue::Integer8(self.motor_controller.mode_of_operation.clone() as i8),
                    _ => {},
                }
            }

        if let Some(var) = self.eds_data.od.get_mut(&0x6041)
            .and_then(|vars| vars.get_mut(&0)) {
                match var.value {
                    DataValue::Unsigned16(_) => var.value = DataValue::Unsigned16(self.motor_controller.statusword),
                    _ => {},
                }
            }

    }

    fn parse_controlword(&mut self) {

        const BIT_INDICES: [usize; 5] = [0, 1, 2, 3, 7];
        
        let bits: Vec<bool> = BIT_INDICES.iter().map(|&i| get_bit_16(&self.motor_controller.controlword, i)).collect();
    
        self.motor_controller.command = match (bits[4], bits[3], bits[2], bits[1], bits[0]) {
            (false, _, true, true, false) => Command::Shutdown,
            (false, false, true, true, true) => Command::SwitchOn,
            (false, _, _, false, _) => Command::DisableVoltage,
            (false, _, false, true, _) => Command::QuickStop,
            (false, true, true, true, true) => Command::EnableOperation,
            (true, _, _, _, _) => Command::FaultReset,
        };

        self.motor_controller.control_oms1.push_front(get_bit_16(&self.motor_controller.controlword, 4));
        self.motor_controller.control_oms1.pop_back();

        self.motor_controller.halt = get_bit_16(&self.motor_controller.controlword, 8)
    }

    fn update_state(&mut self) {


        self.motor_controller.state = match self.motor_controller.state {
            State::NotReadyToSwitchOn => State::SwitchedOnDisabled,
            State::SwitchedOnDisabled => match &self.motor_controller.command {
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOnDisabled,
            }
            State::ReadyToSwitchOn => match &self.motor_controller.command {
                Command::SwitchOn => State::SwitchedOn,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                _ => State::ReadyToSwitchOn,
            }
            State::SwitchedOn => match &self.motor_controller.command {
                Command::EnableOperation => State::OperationEnabled,
                Command::Shutdown => State::ReadyToSwitchOn,
                _ => State::SwitchedOn,
            }
            State::OperationEnabled => match &self.motor_controller.command {
                Command::QuickStop => State::QuickStopActive,
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::SwitchOn => State::SwitchedOn,
                _ => State::OperationEnabled,
            }
            State::QuickStopActive => match &self.motor_controller.command {
                Command::DisableVoltage => State::SwitchedOnDisabled,
                Command::EnableOperationAfterQuickStop => State::OperationEnabled,
                _ => State::QuickStopActive,
            }
            State::FaultReactionActive => State::Fault,
            State::Fault => match &self.motor_controller.command {
                Command::FaultReset => State::SwitchedOnDisabled,
                _ => State::Fault,
            }

        };

    }
    
    fn set_statusword(&mut self) {
        let bits: &[(usize, bool)] = match self.motor_controller.state {
            State::NotReadyToSwitchOn  => &[(0, false), (1, false), (2, false), (3, false), (5, false), (6, false)],
            State::SwitchedOnDisabled  => &[(0, false), (1, false), (2, false), (3, false), (6, true)],
            State::ReadyToSwitchOn     => &[(0, true),  (1, false), (2, false), (3, false), (5, true),  (6, false)],
            State::SwitchedOn          => &[(0, true),  (1, true),  (2, false), (3, false), (5, true),  (6, false)],
            State::OperationEnabled    => &[(0, true),  (1, true),  (2, true),  (3, false), (5, true),  (6, false)],
            State::QuickStopActive     => &[(0, true),  (1, true),  (2, true),  (3, false), (5, false), (6, false)],
            State::FaultReactionActive => &[(0, true),  (1, true),  (2, true),  (3, true),  (6, false)],
            State::Fault               => &[(0, false), (1, false), (2, false), (3, true),  (6, false)],
        };
        set_bits(&mut self.motor_controller.statusword, bits);

        self.motor_controller.statusword = set_bit_16(&self.motor_controller.statusword, 10, self.motor_controller.target_reached);
        self.motor_controller.statusword = set_bit_16(&self.motor_controller.statusword, 12, self.motor_controller.status_oms1);
        self.motor_controller.statusword = set_bit_16(&self.motor_controller.statusword, 13, self.motor_controller.status_oms2);
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