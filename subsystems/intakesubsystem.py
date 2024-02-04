from enum import Enum, auto
from commands2 import SubsystemBase

from util.simfalcon import Falcon

import constants


class IntakeSubsystem(SubsystemBase):
    class IntakeState(Enum):
        Floor = auto()  # arm is down, motor is running inwards
        Trap = auto()  # arm is up, motor holding or running outwards ?
        Hold = auto()  # arm is relaxed, motor is fixed
        Feed = auto()  # arm is halfway up, motor running outwards
        Idle = auto() # arm is down, motor off

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)  # basic subsystem boilerplate

        self.intakeMotor = Falcon(
            constants.kIntakeCANID,
            constants.kIntakePIDSlot,
            constants.kIntakePGain,
            constants.kIntakeIGain,
            constants.kIntakeDGain,
        )
        self.armMotor = Falcon(
            constants.kArmCANID,
            constants.kArmPIDSlot,
            constants.kArmPGain,
            constants.kArmIGain,
            constants.kArmGain,
        )

        self.state = IntakeSubsystem.IntakeState.Idle

# all this motor stuff is wrong I copied it from ball and don't know what to do with it yet

    def periodic(self) -> None:
        if self.state == IntakeSubsystem.IntakeState.Floor:
            self.intakeMotor.set(Falcon.ControlMode.Velocity, 0)
            self.armMotor.set(Falcon.ControlMode.Velocity, 0)

        elif self.state == IntakeSubsystem.IntakeState.Trap:
            self.intakeMotor.set(Falcon.ControlMode.Velocity, 0)
            self.armMotor.set(Falcon.ControlMode.Velocity, 0)

        elif self.state == IntakeSubsystem.IntakeState.Hold:
            self.intakeMotor.set(Falcon.ControlMode.Velocity, 0)
            self.armMotor.set(Falcon.ControlMode.Velocity, 0)

        elif self.state == IntakeSubsystem.IntakeState.Feed:
            self.intakeMotor.set(Falcon.ControlMode.Velocity, 0)
            self.armMotor.set(Falcon.ControlMode.Velocity, 0)
            
        elif self.state == IntakeSubsystem.IntakeState.Idle:
            self.intakeMotor.set(Falcon.ControlMode.Velocity, 0)
            self.armMotor.set(Falcon.ControlMode.Velocity, 0)

    # the following methods are simply state setting, all actual motor control is done in periodic
    def setFloor(self) -> None:
        self.state = IntakeSubsystem.IntakeState.Floor

    def setTrap(self) -> None:
        self.state = IntakeSubsystem.IntakeState.Trap

    def setHold(self) -> None:
        self.state = IntakeSubsystem.IntakeState.Hold

    def setFeed(self) -> None:
        self.state = IntakeSubsystem.IntakeState.Feed

    def setIdle(self) -> None:
        self.state = IntakeSubsystem.IntakeState.Idle