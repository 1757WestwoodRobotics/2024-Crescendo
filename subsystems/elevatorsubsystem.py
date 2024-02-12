from enum import Enum, auto
from math import pi
from commands2 import Subsystem

from util.simtalon import Talon
import constants


class ElevatorSubsystem(Subsystem):
    class ElevatorState(Enum):
        BottomPosition = auto()
        AmpPosition = auto()
        TopPosition = auto()
        PullDown = auto()  # climber

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)  # basic subsystem boilerplate

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
            self.setElevatorMotorsAtPosition(constants.kBottomPositionBeltPosition)

        elif self.state == self.ElevatorState.AmpPosition:
            self.setElevatorMotorsAtPosition(constants.kAmpPositionBeltPosition)

        elif self.state == self.ElevatorState.TopPosition:
            self.setElevatorMotorsAtPosition(constants.kTopPositionBeltPosition)

        elif self.state == self.ElevatorState.PullDown:
            if (
                self.elevatorMotor1.get(Talon.ControlMode.Position)
                > constants.kPullDownBandLimit
            ):
                self.elevatorMotor1.set(
                    Talon.ControlMode.Velocity,
                    constants.kBeltPullDownSpeed
                    * constants.kMotorPulleyGearRatio
                    / (constants.kPulleyGearPitchDiameter * pi),
                )
                self.elevatorMotor2.set(
                    Talon.ControlMode.Velocity,
                    constants.kBeltPullDownSpeed
                    * constants.kMotorPulleyGearRatio
                    / (constants.kPulleyGearPitchDiameter * pi),
                )

            else:
                self.state = self.ElevatorState.BottomPosition

    def setElevatorMotorsAtPosition(self, beltPosition) -> None:
        self.elevatorMotor1.set(
            Talon.ControlMode.Position,
            (beltPosition)
            / (constants.kPulleyGearPitchDiameter * pi)
            * constants.kMotorPulleyGearRatio,
        )
        self.elevatorMotor2.set(
            Talon.ControlMode.Position,
            beltPosition
            / (constants.kPulleyGearPitchDiameter * pi)
            * constants.kMotorPulleyGearRatio,
        )

    # the following methods are simply state setting, all actual motor control is done in periodic
    def setBottomPosition(self) -> None:
        self.state = self.ElevatorState.BottomPosition

    def setAmpPosition(self) -> None:
        self.state = self.ElevatorState.AmpPosition

    def setTopPosition(self) -> None:
        self.state = self.ElevatorState.TopPosition
