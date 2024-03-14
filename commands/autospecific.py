from commands2.sequentialcommandgroup import SequentialCommandGroup
from commands2.waitcommand import WaitCommand
from commands.intakecommands import GroundIntake
from commands.intakesetting import FeedIntakeToShooter
from commands.shooter.alignandaim import AlignAndAim
from commands.shooter.shooterfixedshots import SafetyPosition

AimAndFire = lambda shooter, drive, intake: SequentialCommandGroup(
    AlignAndAim(shooter, drive, (lambda: 0), (lambda: 0)), WaitCommand(0).until(shooter.readyToShoot), FeedIntakeToShooter(intake)
)

IntakeAuto = lambda intake, shooter, elevator: SequentialCommandGroup(
    SafetyPosition(shooter), GroundIntake(elevator, intake)
)
