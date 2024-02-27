from commands2 import Command
from wpilib import SmartDashboard
from subsystems.elevatorsubsystem import ElevatorSubsystem

from util.convenientmath import clamp
import constants


class AscendElevator(Command):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.elevator = elevatorSubsystem
        self.addRequirements([self.elevator])

    def execute(self) -> None:
        self.elevator.setManualControl()
        SmartDashboard.putNumber(
            constants.kElevatorPositionKey,
            clamp(
                self.elevator.getElevatorPosition() + constants.kElevatorManualChange,
                constants.kBottomPositionBeltPosition,
                constants.kTopPositionBeltPosition,
            ),
        )

    def isFinished(self) -> bool:
        return True


class DescendElevator(Command):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.elevator = elevatorSubsystem
        self.addRequirements([self.elevator])

    def execute(self) -> None:
        self.elevator.setManualControl()
        SmartDashboard.putNumber(
            constants.kElevatorPositionKey,
            clamp(
                self.elevator.getElevatorPosition() - constants.kElevatorManualChange,
                constants.kBottomPositionBeltPosition,
                constants.kTopPositionBeltPosition,
            ),
        )

    def isFinished(self) -> bool:
        return True
