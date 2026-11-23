use crate::logic::Quat;

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[repr(u8)]
pub enum FlightMode {
    Safe = 0b00,
    Standby = 0b01,
    Nominal = 0b10,
    Critical = 0b11,
}

impl FlightMode {
    #[inline]
    pub const fn from_quat(q: Quat) -> Self {
        match q {
            Quat::Q00 => FlightMode::Safe,
            Quat::Q01 => FlightMode::Standby,
            Quat::Q10 => FlightMode::Nominal,
            Quat::Q11 => FlightMode::Critical,
        }
    }

    #[inline]
    pub const fn to_quat(self) -> Quat {
        match self {
            FlightMode::Safe => Quat::Q00,
            FlightMode::Standby => Quat::Q01,
            FlightMode::Nominal => Quat::Q10,
            FlightMode::Critical => Quat::Q11,
        }
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum FaultReason {
    WatchdogTimeout,
    SensorDisagreement,
    BusCrcMismatch,
    RadiationUpset,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum TransitionError {
    JumpTooLarge { from: FlightMode, to: FlightMode },
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct FlightModeController {
    mode: FlightMode,
    fault_streak: u8,
    max_faults_before_safe: u8,
    last_fault: Option<FaultReason>,
}

impl FlightModeController {
    pub const fn new(initial: FlightMode, max_faults_before_safe: u8) -> Self {
        Self {
            mode: initial,
            fault_streak: 0,
            max_faults_before_safe: if max_faults_before_safe == 0 {
                1
            } else {
                max_faults_before_safe
            },
            last_fault: None,
        }
    }

    #[inline]
    pub const fn mode(&self) -> FlightMode {
        self.mode
    }

    #[inline]
    pub const fn fault_streak(&self) -> u8 {
        self.fault_streak
    }

    #[inline]
    pub const fn last_fault(&self) -> Option<FaultReason> {
        self.last_fault
    }

    pub fn clear_faults(&mut self) {
        self.fault_streak = 0;
        self.last_fault = None;
    }

    pub fn step_to(&mut self, target: FlightMode) -> Result<FlightMode, TransitionError> {
        if matches!(target, FlightMode::Safe) {
            self.mode = FlightMode::Safe;
            return Ok(self.mode);
        }

        let cur = self.mode.to_quat().bits();
        let nxt = target.to_quat().bits();
        if nxt.abs_diff(cur) > 1 {
            return Err(TransitionError::JumpTooLarge {
                from: self.mode,
                to: target,
            });
        }

        self.mode = target;
        Ok(self.mode)
    }

    /// Records a fault and deterministically degrades flight mode when `max_faults_before_safe` is hit.
    pub fn report_fault(&mut self, reason: FaultReason) -> FlightMode {
        self.last_fault = Some(reason);
        self.fault_streak = self.fault_streak.saturating_add(1);

        if self.fault_streak >= self.max_faults_before_safe {
            self.mode = FlightMode::Safe;
        } else if matches!(self.mode, FlightMode::Critical) {
            self.mode = FlightMode::Nominal;
        } else if matches!(self.mode, FlightMode::Nominal) {
            self.mode = FlightMode::Standby;
        }

        self.mode
    }
}