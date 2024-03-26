from enum import Enum, auto
from math import pi
from commands2 import Subsystem
from wpilib import SmartDashboard

from util.simtalon import Talon
from util.convenientmath import clamp
import constants


class ElevatorSubsystem(Subsystem):
    class ElevatorState(Enum):
        BottomPosition = auto()
        AmpPosition = auto()
        Controlled = auto()
        ManualControl = auto()

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
            moMagicAccel=constants.kElevatorMaxAccel,
            moMagicVel=constants.kElevatorMaxVel,
        )

        self.elevatorMotor2 = Talon(
            constants.kElevator2CANID,
            constants.kElevator2Name,
            constants.kElevator2PGain,
            constants.kElevator2IGain,
            constants.kElevator2DGain,
            constants.kElevator2Inverted,
        )
        self.elevatorMotor1.setNeutralMode(Talon.NeutralMode.Brake)
        self.elevatorMotor2.setNeutralMode(Talon.NeutralMode.Brake)
        self.controlledPosition = 0

        self.elevatorMotor2.follow(self.elevatorMotor1, True)
        self.targetPosition = 0

        self.state = self.ElevatorState.BottomPosition
        SmartDashboard.putNumber(constants.kElevatorPositionKey, 0)

    def periodic(self) -> None:
        if self.state == self.ElevatorState.BottomPosition:
            self.setElevatorMotorsAtPosition(constants.kBottomPositionBeltPosition)

        elif self.state == self.ElevatorState.AmpPosition:
            self.setElevatorMotorsAtPosition(constants.kAmpPositionBeltPosition)

        elif self.state == self.ElevatorState.Controlled:
            self.setElevatorMotorsAtPosition(self.controlledPosition)

        SmartDashboard.putString(constants.kElevatorStateKey, str(self.state))
        SmartDashboard.putBoolean(constants.kElevatorAtPositionKey, self.atPosition())

        if self.state == self.ElevatorState.ManualControl:
            self.setElevatorMotorsAtPosition(
                SmartDashboard.getNumber(
                    constants.kElevatorPositionKey, self.getElevatorPosition()
                )
            )

        else:
            SmartDashboard.putNumber(
                constants.kElevatorPositionKey, self.getElevatorPosition()
            )

    def setElevatorMotorsAtPosition(self, beltPosition) -> None:
        self.targetPosition = beltPosition
        self.elevatorMotor1.set(
            Talon.ControlMode.MotionMagic,
            clamp(
                beltPosition,
                0 if self.state != self.ElevatorState.ManualControl else -1,
                constants.kTopPositionBeltPosition,
            )
            / (constants.kPulleyGearPitchDiameter * pi)
            * constants.kMotorPulleyGearRatio,
        )

    def atPosition(self) -> bool:
        return (
            abs(self.getElevatorPosition() - self.targetPosition)
            < constants.kElevatorTolerance
        )

    def getElevatorPosition(self) -> float:
        """returns in meters from bottom position"""
        return (
            self.elevatorMotor1.get(Talon.ControlMode.Position)
            / constants.kMotorPulleyGearRatio
            * constants.kPulleyGearPitchDiameter
            * pi
        )

    # the following methods are simply state setting, all actual motor control is done in periodic
    def setBottomPosition(self) -> None:
        self.state = self.ElevatorState.BottomPosition

    def setAmpPosition(self) -> None:
        self.state = self.ElevatorState.AmpPosition

    def setManualControl(self) -> None:
        self.state = self.ElevatorState.ManualControl

    def setTargetPosition(self, target: float) -> None:
        self.state = self.ElevatorState.Controlled
        self.controlledPosition = target
        self.targetPosition = target
