from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem

# temp command for testing note sim, general score command will be made later


class ShootSimNote(Command):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        self.shooter.addSimNote()

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True
