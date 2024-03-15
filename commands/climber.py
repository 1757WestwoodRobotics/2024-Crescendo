from commands2 import Command
from subsystems.climbersubsystem import ClimberSubsystem

class SetClimberState(Command):
    def __init__(self, climberSubsystem: ClimberSubsystem):
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.climber = climberSubsystem
        self.addRequirements(self.climber)

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")
    def isFinished(self) -> bool:
        return True
    
class ExtendClimberPosition(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem):
        SetClimberState.__init__(self, climberSubsystem)

    def execute(self) -> None:
        self.climber.setClimberExtend

class RetractClimberPosition(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem):
        SetClimberState.__init__(self, climberSubsystem)
    
    def execute(self) -> None:
        self.climber.setClimberRetract

class NeutralClimberState(SetClimberState):
    def __init__(self, climberSubsystem: ClimberSubsystem):
        SetClimberState.__init__(self, climberSubsystem)

    def execute(self) -> None:
        self.climber.setClimberHold