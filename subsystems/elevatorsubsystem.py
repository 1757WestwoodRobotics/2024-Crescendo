from enum import Enum, auto
from math import pi
from commands2 import SubsystemBase

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

        self.elevatorMotor1 = Talon(
            constants.kElevator1CANID,
            constants.kElevator1Name,
            constants.kElevator1PGain,
            constants.kElevator1IGain,
            constants.kElevator1DGain,
            constants.kElevator1Inverted,
        )

        self.elevatorMotor2 = Talon(
            constants.kElevator2CANID,
            constants.kElevator2Name,
            constants.kElevator2PGain,
            constants.kElevator2IGain,
            constants.kElevator2DGain,
            constants.kElevator2Inverted,
        )

        self.state = self.ElevatorState.BottomPosition

    def periodic(self) -> None:
        if self.state == self.ElevatorState.BottomPosition:
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
