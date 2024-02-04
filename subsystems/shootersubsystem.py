from commands2 import Subsystem
from util.simtalon import Talon
from util.simcoder import CTREEncoder
import constants
from wpimath.geometry import Rotation2d
from wpilib import SmartDashboard


class ShooterSubsystem(Subsystem):
    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.angleMotor = Talon(
            constants.kAngleMotorCANId,
            constants.kAngleMotorName,
            constants.kAngleMotorPGain,
            constants.kAngleMotorIGain,
            constants.kAngleMotorDGain,
            constants.kAngleMotorInverted,
        )
        self.leftShootingMotor = Talon(
            constants.kLeftShootingMotorCANId,
            constants.kLeftShootingMotorName,
            constants.kLeftShootingMotorPGain,
            constants.kLeftShootingMotorIGain,
            constants.kLeftShootingMotorDGain,
            constants.kLeftShootingMotorInverted,
        )
        self.rightShootingMotor = Talon(
            constants.kRightShootingMotorCANId,
            constants.kRightShootingMotorName,
            constants.kRightShootingMotorPGain,
            constants.kRightShootingMotorIGain,
            constants.kRightShootingMotorDGain,
            constants.kRightShootingMotorInverted,
        )

        self.shooterEncoder = CTREEncoder(
            constants.kShooterAngleEncoderCANId,
            constants.kShooterAngleMotorOffset.degrees(),
        )

        # set motor position

        self.angleMotor.setEncoderPosition(
            constants.kShooterAngleMotorOffset.degrees()
            / constants.kDegeersPerRevolution
            * constants.kShootingMotorRatio
        )

        self.targetAngle = Rotation2d()
        self.leftTargetSpeed = 0
        self.rightTargetSpeed = 0

    def setShooterAngle(self, angle: Rotation2d) -> None:
        self.targetAngle = min(max(Rotation2d(0), angle), constants.kShooterMaxAngle)

        self.angleMotor.set(
            Talon.ControlMode.Position,
            self.targetAngle.degrees()
            / constants.kDegeersPerRevolution
            * constants.kAngleMotorRatio,
        )

    def setLeftShootingMotorSpeed(self, speed: int) -> None:
        self.leftTargetSpeed = speed
        self.leftShootingMotor.set(
            Talon.ControlMode.Velocity,
            speed * constants.kShootingMotorRatio * constants.kTalonVelocityPerRPM,
        )

    def setRightShootingMotorSpeed(self, speed: int) -> None:
        self.rightTargetSpeed = speed
        self.rightShootingMotor.set(
            Talon.ControlMode.Velocity,
            speed * constants.kShootingMotorRatio * constants.kTalonVelocityPerRPM,
        )

    def getShooterAngle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(
            self.angleMotor.get(Talon.ControlMode.Position)
            * constants.kAngleMotorRatio
            * constants.kDegeersPerRevolution
        )

    def getLeftShooterSpeed(self) -> int:
        return (
            self.leftShootingMotor.get(Talon.ControlMode.Velocity)
            / constants.kTalonVelocityPerRPM
        )

    def getRightShooterSpeed(self) -> int:
        return (
            self.rightShootingMotor.get(Talon.ControlMode.Velocity)
            / constants.kTalonVelocityPerRPM
        )

    def angleOnTarget(self) -> bool:
        return (
            abs(self.targetAngle - self.getShooterAngle())
            < constants.kShooterAngleTolerance
        )

    def leftMotorSpeedOnTarget(self) -> bool:
        return (
            abs(self.leftTargetSpeed - self.getLeftShooterSpeed)
            < constants.kShooterSpeedTolerance
        )

    def rightMotorSpeedOnTarget(self) -> bool:
        return (
            abs(self.rightTargetSpeed - self.getRightShooterSpeed)
            < constants.kShooterSpeedTolerance
        )

    def readyToShoot(self) -> bool:
        return (
            self.angleOnTarget()
            and self.leftMotorSpeedOnTarget()
            and self.rightMotorSpeedOnTarget()
        )

    def periodic(self) -> None:
        # logging
        SmartDashboard.putNumber(
            constants.kShooterAngleKey, self.getShooterAngle().degrees()
        )
        SmartDashboard.putNumber(
            constants.kLeftShootingMotorSpeedKey, self.getLeftShooterSpeed()
        )
        SmartDashboard.putNumber(
            constants.kRightShootingMotorSpeedKey, self.getRightShooterSpeed()
        )

        SmartDashboard.putBoolean(
            constants.kShooterAngleOnTargetKey, self.angleOnTarget()
        )
        SmartDashboard.putBoolean(
            constants.kLeftShootingMotorOnTargetKey, self.leftMotorSpeedOnTarget()
        )
        SmartDashboard.putBoolean(
            constants.kRightShootingMotorOnTargetKey, self.rightMotorSpeedOnTarget()
        )
        SmartDashboard.putBoolean(constants.kReadyToShoot, self.readyToShoot())
