from commands2 import Command
from wpilib import SmartDashboard

from subsystems.shootersubsystem import ShooterSubsystem
import constants


class FudgeShooter(Command):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self):
        raise NotImplementedError("Must be implemented by subclass")

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True


class DecreaseLeftMotorSpeed(FudgeShooter):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        FudgeShooter.__init__(self, shooter)

    def execute(self) -> None:
        SmartDashboard.putNumber(
            constants.kLeftMotorFudgeKey,
            SmartDashboard.getNumber(constants.kLeftMotorFudgeKey, 0)
            - constants.kShootingMotorFudgeAmount,
        )


class IncreaseLeftMotorSpeed(FudgeShooter):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        FudgeShooter.__init__(self, shooter)

    def execute(self) -> None:
        SmartDashboard.putNumber(
            constants.kLeftMotorFudgeKey,
            SmartDashboard.getNumber(constants.kLeftMotorFudgeKey, 0)
            + constants.kShootingMotorFudgeAmount,
        )


class DecreaseRightMotorSpeed(FudgeShooter):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        FudgeShooter.__init__(self, shooter)

    def execute(self) -> None:
        SmartDashboard.putNumber(
            constants.kRightMotorFudgeKey,
            SmartDashboard.getNumber(constants.kRightMotorFudgeKey, 0)
            - constants.kShootingMotorFudgeAmount,
        )


class IncreaseRightMotorSpeed(FudgeShooter):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        FudgeShooter.__init__(self, shooter)

    def execute(self) -> None:
        SmartDashboard.putNumber(
            constants.kRightMotorFudgeKey,
            SmartDashboard.getNumber(constants.kRightMotorFudgeKey, 0)
            + constants.kShootingMotorFudgeAmount,
        )


class DecreaseShooterAngle(FudgeShooter):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        FudgeShooter.__init__(self, shooter)

    def execute(self) -> None:
        SmartDashboard.putNumber(
            constants.kShooterAngleFudgeKey,
            SmartDashboard.getNumber(constants.kShooterAngleFudgeKey, 0)
            - constants.kShootingAngleFudgeAmount,
        )


class IncreaseShooterAngle(FudgeShooter):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        FudgeShooter.__init__(self, shooter)

    def execute(self) -> None:
        SmartDashboard.putNumber(
            constants.kShooterAngleFudgeKey,
            SmartDashboard.getNumber(constants.kShooterAngleFudgeKey, 0)
            + constants.kShootingAngleFudgeAmount,
        )
