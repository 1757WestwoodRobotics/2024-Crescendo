from commands2 import Command
from subsystems.climbersubsystem import ClimberSubsystem

class SetClimberState(Command):
    def __init__(self, climberSubsystem: ClimberSubsystem):
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climberSubsystem
        self.addRequirements(self.climber)

    def execute(self) -> None:
        self.climber
    def isFinished(self) -> bool:
        return True