from commands2.sequentialcommandgroup import SequentialCommandGroup
from commands.intakesetting import FeedIntakeToShooter
from commands.shooter.alignandaim import AlignAndAim

AimAndFire = lambda shooter, drive, intake: SequentialCommandGroup(
    AlignAndAim(shooter, drive, (lambda: 0), (lambda: 0)), FeedIntakeToShooter(intake)
)
