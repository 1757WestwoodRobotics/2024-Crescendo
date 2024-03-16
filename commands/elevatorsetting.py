from commands2 import Command
from wpilib import Timer
from subsystems.elevatorsubsystem import ElevatorSubsystem


class SetElevatorState(Command):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.elevator = elevatorSubsystem
        self.t = Timer()
        self.addRequirements(self.elevator)

    def initialize(self):
        self.t.reset()
        self.t.start()

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return self.elevator.atPosition() and self.t.get() > 0.5


class ElevatorBottomPosition(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setBottomPosition()


class ElevatorAmpPosition(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setAmpPosition()


class ElevatorTopPosition(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setTopPosition()


class ElevatorPullDown(SetElevatorState):
    def __init__(self, elevatorSubsystem: ElevatorSubsystem) -> None:
        SetElevatorState.__init__(self, elevatorSubsystem)

    def execute(self) -> None:
        self.elevator.setPullDown()
