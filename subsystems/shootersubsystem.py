from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from util.simtalon import Talon
from util.simneo import NEOBrushless
from util.simcoder import CTREEncoder
import constants


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
        self.leftShootingMotor = NEOBrushless(
            constants.kLeftShootingMotorCANId,
            constants.kLeftShootingMotorName,
            constants.kLeftShootingMotorPIDSlot,
            constants.kLeftShootingMotorPGain,
            constants.kLeftShootingMotorIGain,
            constants.kLeftShootingMotorDGain,
            constants.kLeftShootingMotorInverted,
        )
        self.rightShootingMotor = NEOBrushless(
            constants.kRightShootingMotorCANId,
            constants.kRightShootingMotorName,
            constants.kRightShootingMotorPIDSlot,
            constants.kRightShootingMotorPGain,
            constants.kRightShootingMotorIGain,
            constants.kRightShootingMotorDGain,
            constants.kRightShootingMotorInverted,
        )

        self.shooterEncoder = CTREEncoder(
            constants.kShooterAngleEncoderCANId,
            constants.kShooterAngleMotorOffset.degrees()
            / constants.kDegeersPerRevolution,
        )

        self.leftShootingMotor.setSmartCurrentLimit(
            constants.kShootingMotorCurrentLimit
        )
        self.rightShootingMotor.setSmartCurrentLimit(
            constants.kShootingMotorCurrentLimit
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

        # speeds are in RPM

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
            NEOBrushless.ControlMode.Velocity,
            speed * constants.kShootingMotorRatio,
        )

    def setRightShootingMotorSpeed(self, speed: int) -> None:
        self.rightTargetSpeed = speed
        self.rightShootingMotor.set(
            NEOBrushless.ControlMode.Velocity,
            speed * constants.kShootingMotorRatio,
        )

    def getShooterAngle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(
            self.angleMotor.get(Talon.ControlMode.Position)
            * constants.kAngleMotorRatio
            * constants.kDegeersPerRevolution
        )

    def getLeftShooterSpeed(self) -> int:
        return self.leftShootingMotor.get(NEOBrushless.ControlMode.Velocity)

    def getRightShooterSpeed(self) -> int:
        return self.rightShootingMotor.get(NEOBrushless.ControlMode.Velocity)

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
            and SmartDashboard.getBoolean(constants.kRobotAngleOnTargetKey, False)
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
