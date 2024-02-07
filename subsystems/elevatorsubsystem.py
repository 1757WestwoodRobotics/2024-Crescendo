from enum import Enum, auto
from commands2 import SubsystemBase
from math import pi

from util.simtalon import Talon
from util.simcoder import CTREEncoder
import constants


class ElevatorSubsystem(SubsystemBase):
    class ElevatorState(Enum):
        BottomPosition = auto()
        AmpPosition = auto()
        TopPosition = auto()

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)  # basic subsystem boilerplate

        self.elevatorEncoder = CTREEncoder(
            constants.kElevatorEncoderID,
            constants.kElevatorBottomPositionDegrees,
        )

        self.elevatorMotor = Talon(
            constants.kElevatorCANID,
            constants.kElevatorName,
            constants.kElevatorPGain,
            constants.kElevatorIGain,
            constants.kElevatorDGain,
            constants.kElevatorInverted,
        )

        self.state = self.ElevatorState.BottomPosition

    def periodic(self) -> None:
        if self.state == self.ElevatorState.DownPosition:
            setElevatorMotorsAtPosition(self, constants.kBottomPositionBeltPosition)

        elif self.state == self.ElevatorState.AmpPosition:
            setElevatorMotorsAtPosition(self, constants.kAmpPositionBeltPosition)

        elif self.state == self.ElevatorState.TopPosition:
            setElevatorMotorsAtPosition(self, constants.kTopPositionBeltPosition)

        def setElevatorMotorsAtPosition(self, beltPosition) -> None:
            self.elevatorMotor1.set(
                Talon.ControlMode.Position,
                beltPosition
                / (constants.kPulleyGearPitchDiameter * constants.kMetersPerInch * pi)
                * constants.kMotorPulleyGearRatio,
            )
            self.elevatorMotor2.set(
                Talon.ControlMode.Position,
                beltPosition
                / (constants.kPulleyGearPitchDiameter * constants.kMetersPerInch * pi)
                * constants.kMotorPulleyGearRatio,
            )

    # the following methods are simply state setting, all actual motor control is done in periodic
    def setBottomPosition(self) -> None:
        self.state = self.ElevatorState.BottomPosition

    def setAmpPosition(self) -> None:
        self.state = self.ElevatorState.AmpPosition

    def setTopPosition(self) -> None:
        self.state = self.ElevatorState.TopPosition
