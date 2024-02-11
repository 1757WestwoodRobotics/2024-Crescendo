from commands2 import Command
from wpilib import SmartDashboard
import constants
from subsystems.shootersubsystem import ShooterSubsystem


class ShooterManualMode(Command):
    def __init__(self, shooter: ShooterSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.shooter = shooter
        self.addRequirements(self.shooter)

        SmartDashboard.putNumber(constants.kLeftShootingMotorSpeedKey, 0)
        SmartDashboard.putNumber(constants.kRightShootingMotorSpeedKey, 0)
        SmartDashboard.putNumber(constants.kShooterAngleKey, 0)

    def execute(self) -> None:
        self.shooter.setLeftShootingMotorSpeed(
            SmartDashboard.getNumber(constants.kLeftShootingMotorSpeedKey, 0)
        )
        self.shooter.setRightShootingMotorSpeed(
            SmartDashboard.getNumber(constants.kRightShootingMotorSpeedKey, 0)
        )
        self.shooter.setShooterAngle(
            SmartDashboard.getNumber(constants.kShooterAngleKey, 0)
        )
