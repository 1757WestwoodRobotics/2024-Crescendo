from enum import Enum, auto
from wpilib import SmartDashboard
from commands2.subsystem import Subsystem

import constants
from util.simtalon import Talon


class VelocityControl(Subsystem):
    class ControlState(Enum):
        Setpoint1 = auto()
        Setpoint2 = auto()
        Off = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        SmartDashboard.putNumber(constants.kVelocitySetpoint1ControlKey, 0)
        SmartDashboard.putNumber(constants.kVelocitySetpoint2ControlKey, 0)
        SmartDashboard.putNumber(constants.kVelocityControlGearRatio, 1)

        self.motor = Talon(
            constants.kVelocityControlCANId,
            "Velocity Control",
            constants.kVelocityControlPGain,
            constants.kVelocityControlIGain,
            constants.kVelocityControlDGain,
            False,
            constants.kCANivoreName,
            constants.kVelocityControlkV,
        )

        self.state = VelocityControl.ControlState.Off

    def periodic(self) -> None:
        if self.state == VelocityControl.ControlState.Off:
            self.motor.neutralOutput()
        elif self.state == VelocityControl.ControlState.Setpoint1:
            self.motor.set(
                Talon.ControlMode.Velocity,
                SmartDashboard.getNumber(constants.kVelocitySetpoint1ControlKey, 0)
                / SmartDashboard.getNumber(constants.kVelocityControlGearRatio, 1),
            )
        elif self.state == VelocityControl.ControlState.Setpoint2:
            self.motor.set(
                Talon.ControlMode.Velocity,
                SmartDashboard.getNumber(constants.kVelocitySetpoint2ControlKey, 0)
                / SmartDashboard.getNumber(constants.kVelocityControlGearRatio, 1),
            )

    def setState(self, state: ControlState) -> None:
        self.state = state