from commands2.sequentialcommandgroup import SequentialCommandGroup
from commands2.waitcommand import WaitCommand
from commands.defensestate import DefenseState
from commands.intakecommands import GroundIntake
from commands.intakesetting import FeedIntakeToShooter, FloorIntake, HoldIntakeAtHandoff
from commands.shooter.alignandaim import AlignAndAim
from commands.shooter.shooterfixedshots import SafetyPosition

AimAndFire = lambda shooter, drive, intake: SequentialCommandGroup(
    HoldIntakeAtHandoff(intake),
    AlignAndAim(shooter, drive, (lambda: 0), (lambda: 0)),
    FeedIntakeToShooter(intake),
    DefenseState(drive),
)

IntakeAuto = lambda intake, shooter: SequentialCommandGroup(
    SafetyPosition(shooter), FloorIntake(intake)
)
