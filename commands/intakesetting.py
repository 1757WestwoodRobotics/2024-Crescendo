from commands2 import CommandBase
from subsystems.intakesubsystem import IntakeSubsystem


class SetIntakeState(
    CommandBase
):  # since all ball subsystem commands are the same, subclasses will implement calling while this serves as the base
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        CommandBase.__init__(self)
        self.setName(__class__.__name__)
        self.intake = intakeSubsystem
        self.addRequirements([self.intake])

    def execute(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def isFinished(self) -> bool:
        return True


class FloorIntake(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setIntaking()


class HoldIntakeAtHandoff(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setHolding()


class FeedIntakeToShooter(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setFeeding()


class StageIntake(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setStaging()


class ScoreAmp(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setAmp()


class ScoreTrap(SetIntakeState):
    def __init__(self, intakeSubsystem: IntakeSubsystem) -> None:
        SetIntakeState.__init__(self, intakeSubsystem)

    def execute(self) -> None:
        self.intake.setTrap()