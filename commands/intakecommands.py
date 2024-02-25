from commands2 import ParallelCommandGroup
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
from subsystems.elevatorsubsystem import ElevatorSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem


class GroundIntake(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        ParallelCommandGroup.__init__(
            ElevatorBottomPosition(elevator), FloorIntake(intake)
        )
        self.setName(__class__.__name__)


class DefaultIntake(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        ParallelCommandGroup.__init__(
            ElevatorBottomPosition(elevator), HoldIntakeAtHandoff(intake)
        )
        self.setName(__class__.__name__)


class PrepareAmp(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        ParallelCommandGroup.__init__(
            ElevatorAmpPosition(elevator), StageIntake(intake)
        )
        self.setName(__class__.__name__)


class PrepareTrap(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        ParallelCommandGroup.__init__(
            ElevatorTopPosition(elevator), StageIntake(intake)
        )
        self.setName(__class__.__name__)


class ScoreTrap(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        commands = []

        if (
            intake.state == intake.IntakeState.Staging or intake.IntakeState.Trap
        ) and elevator.state == elevator.ElevatorState.TopPosition:
            commands = [EjectInTrap(intake)]

        ParallelCommandGroup.__init__(*commands)
        self.setName(__class__.__name__)


# scores amp or feeds to shooter depending on state
class DynamicScore(ParallelCommandGroup):
    def __init__(
        self,
        elevator: ElevatorSubsystem,
        intake: IntakeSubsystem,
        shooter: ShooterSubsystem,
    ):
        commands = []

        if (
            intake.state in (intake.IntakeState.Holding, intake.IntakeState.Feeding)
            and elevator.state == elevator.ElevatorState.BottomPosition
        ):
            commands = [FeedIntakeToShooter(intake)]
            shooter.addSimNote()

        elif (
            intake.state in (intake.IntakeState.Staging, intake.IntakeState.Amp)
            and elevator.state == elevator.ElevatorState.AmpPosition
        ):
            commands = [EjectInAmp(intake)]

        ParallelCommandGroup.__init__(*commands)
        self.setName(__class__.__name__)
