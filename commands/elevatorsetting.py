from commands2 import CommandBase
from subsystems.elevatorsubsystem import ElevatorSubsystem


class SetElevatorState(CommandBase):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.elevator = elevatorSubsystem
        self.addRequirements([self.elevator])

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return True


class ElevatorBottomPosition(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setBottomPosition()


class ElevatorAmpPosition(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setAmpPosition


class ElevatorTopPosition(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setTopPosition
