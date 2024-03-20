from wpilib import SmartDashboard
from commands2 import Subsystem
from enum import Enum, auto
import constants
from util.simtalon import Talon
from math import pi


class ClimberSubsystem(Subsystem):
    class ClimberState(Enum):
        ClimberMoving = auto()
        ClimberNeutral = auto()

    def __init__(self):
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.climberMotor = Talon(
            constants.kClimberCANID,
            constants.kClimberName,
            constants.kClimberIGain,
            constants.kClimberPGain,
            constants.kClimberDGain,
            constants.kClimberInverted,
        )
        self.climberMotor.setNeutralMode(Talon.NeutralMode.Brake)
        self.climberMotor.setCurrentLimit(constants.kClimberCurrentLimit)
        self.state = ClimberSubsystem.ClimberState.ClimberNeutral

        self.targetPosition = 0

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kClimberStateKey, str(self.state))
        # check over this math ltr ivan did this in 5 minutes also format
        climberHeight = (
            self.climberMotor.get(Talon.ControlMode.Position)
            / constants.kClimberGearRatio
            * 2
            * pi
            * constants.kClimberWinchRadius
        )
        SmartDashboard.putNumber(constants.kClimberHeightKey, climberHeight)
        SmartDashboard.putNumber(constants.kClimberTargetKey, self.targetPosition)

        if self.state == self.ClimberState.ClimberMoving:
            self.climberMotor.set(
                Talon.ControlMode.Position,
                self.targetPosition
                * constants.kClimberGearRatio
                / (2 * pi * constants.kClimberWinchRadius),
            )
        elif self.state == self.ClimberState.ClimberNeutral:
            self.climberMotor.neutralOutput()

    def setClimberTargetPosition(self, target: float) -> None:
        self.state = self.ClimberState.ClimberMoving
        self.targetPosition = target

    def setClimberHold(self) -> None:
        self.state = self.ClimberState.ClimberNeutral
