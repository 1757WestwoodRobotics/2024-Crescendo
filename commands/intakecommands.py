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
    ScoreAmp,
)
from subsystems.elevatorsubsystem import ElevatorSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem


class GroundIntake(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        super.__init__(ElevatorBottomPosition(elevator), FloorIntake(intake))
        self.setName(__class__.__name__)


class PrepareAmp(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        super.__init__(ElevatorAmpPosition(elevator), StageIntake(intake))
        self.setName(__class__.__name__)


class PrepareTrap(ParallelCommandGroup):
    def __init__(self, elevator: ElevatorSubsystem, intake: IntakeSubsystem):
        super.__init__(ElevatorTopPosition(elevator), StageIntake(intake))
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
            intake.state == intake.IntakeState.Holding
            and elevator.state == elevator.ElevatorState.BottomPosition
        ):
            commands = [
                ElevatorBottomPosition(elevator),
                FeedIntakeToShooter(intake),
            ]
            shooter.addSimNote()
        elif (
            intake.state == intake.IntakeState.Staging
            and elevator.state == elevator.ElevatorState.AmpPosition
        ):
            commands = [ElevatorAmpPosition(elevator), ScoreAmp(intake)]

        super.__init__(*commands)
        self.setName(__class__.__name__)
