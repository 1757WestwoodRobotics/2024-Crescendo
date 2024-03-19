from commands2 import ParallelCommandGroup, Command, SequentialCommandGroup
from wpilib import SmartDashboard

from commands.elevatorsetting import (
    ElevatorBottomPosition,
    ElevatorAmpPosition,
    ElevatorTopPosition,
)
from commands.intakesetting import (
    FloorIntake,
    StageIntake,
    FeedIntakeToShooter,
    EjectInAmp,
    HoldIntakeAtHandoff,
    EjectInTrap,
)


import constants
from subsystems.elevatorsubsystem import ElevatorSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem


class GroundIntake(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        ParallelCommandGroup.__init__(
            self, ElevatorBottomPosition(elevator), FloorIntake(intake)
        )
        self.setName(__class__.__name__)


class DefaultIntake(SequentialCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        SequentialCommandGroup.__init__(
            self,
            ElevatorBottomPosition(elevator),
            HoldIntakeAtHandoff(intake),
        )
        self.setName(__class__.__name__)


class PrepareAmp(SequentialCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        SequentialCommandGroup.__init__(
            self, StageIntake(intake), ElevatorAmpPosition(elevator)
        )
        self.setName(__class__.__name__)


class PrepareTrap(SequentialCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        SequentialCommandGroup.__init__(
            self, StageIntake(intake), ElevatorTopPosition(elevator)
        )
        self.setName(__class__.__name__)


class ScoreTrap(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        commands = []

        if (
            intake.state == intake.IntakeState.Staging or intake.IntakeState.Trap
        ) and elevator.state == elevator.ElevatorState.TopPosition:
            commands = [EjectInTrap(intake)]

        ParallelCommandGroup.__init__(self, *commands)
        self.setName(__class__.__name__)


# scores amp or feeds to shooter depending on state
class DynamicScore(Command):
    def __init__(
        self,
        elevator: ElevatorSubsystem,
        intake: IntakeSubsystem,
        shooter: ShooterSubsystem,
    ):
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.elevator = elevator
        self.intake = intake
        self.shooter = shooter

        self.command = Command()

        self.running = False
        self.addRequirements(self.elevator, self.intake)

    def initialize(self):
        self.running = True
        if (
            self.intake.state
            in (self.intake.IntakeState.Holding, self.intake.IntakeState.Feeding)
            and self.elevator.state == self.elevator.ElevatorState.BottomPosition
        ):
            self.command = FeedIntakeToShooter(self.intake)
            if SmartDashboard.getBoolean(constants.kIntakeHasNoteKey, False):
                self.shooter.addSimNote()

        elif (
            self.intake.state
            in (self.intake.IntakeState.Staging, self.intake.IntakeState.Amp)
            and self.elevator.state == self.elevator.ElevatorState.AmpPosition
        ):
            self.command = EjectInAmp(self.intake)
        self.command.initialize()

    def execute(self) -> None:
        self.command.execute()
        if self.command.isFinished():
            self.command.end(False)
            self.running = False

    def isFinished(self) -> bool:
        return False
