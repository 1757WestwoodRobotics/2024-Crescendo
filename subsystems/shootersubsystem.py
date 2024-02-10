from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from phoenix6.configs import CurrentLimitsConfigs
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
            constants.kLeftShootingMotorKv,
        )
        self.rightShootingMotor = NEOBrushless(
            constants.kRightShootingMotorCANId,
            constants.kRightShootingMotorName,
            constants.kRightShootingMotorPIDSlot,
            constants.kRightShootingMotorPGain,
            constants.kRightShootingMotorIGain,
            constants.kRightShootingMotorDGain,
            constants.kRightShootingMotorInverted,
            constants.kRightShootingMotorKv,
        )

        self.shooterEncoder = CTREEncoder(constants.kShooterAngleEncoderCANId, 0)

        self.leftShootingMotor.setSmartCurrentLimit(
            constants.kShootingMotorCurrentLimit
        )
        self.rightShootingMotor.setSmartCurrentLimit(
            constants.kShootingMotorCurrentLimit
        )

        self.angleMotor.setCurrentLimit(
            CurrentLimitsConfigs().with_stator_current_limit(
                constants.kAngleMotorCurrentLimit
            )
        )

        # set motor position, encoder is inverse of the shooter

        self.angleMotor.setEncoderPosition(
            self.shooterEncoder.getPosition().radians()
            / constants.kRadiansPerRevolution
            * constants.kAngleMotorRatio
            * -1
        )

        self.targetAngle = Rotation2d()
        self.leftTargetSpeed = 0
        self.rightTargetSpeed = 0

        # speeds are in RPM, same as velocity control mode

    def setShooterAngle(self, angle: Rotation2d) -> None:
        self.targetAngle = min(
            max(
                constants.kShooterMinAngle,
                angle
                + Rotation2d(SmartDashboard.getNumber(constants.kShooterAngleFudgeKey)),
                constants.kShooterMaxAngle,
            )
        )

        self.angleMotor.set(
            Talon.ControlMode.Position,
            self.targetAngle.radians()
            / constants.kRadiansPerRevolution
            * constants.kAngleMotorRatio,
        )

    def setLeftShootingMotorSpeed(self, rpm: int) -> None:
        self.leftTargetSpeed = rpm + SmartDashboard.getNumber(
            constants.kLeftMotorFudgeKey, 0
        )
        self.leftShootingMotor.set(
            NEOBrushless.ControlMode.Velocity,
            rpm * constants.kShootingMotorRatio,
        )

    def setRightShootingMotorSpeed(self, rpm: int) -> None:
        self.rightTargetSpeed = rpm + SmartDashboard.getNumber(
            constants.kRightMotorFudgeKey, 0
        )
        self.rightShootingMotor.set(
            NEOBrushless.ControlMode.Velocity,
            rpm * constants.kShootingMotorRatio,
        )

    def getShooterAngle(self) -> Rotation2d:
        return Rotation2d(self.shooterEncoder.getPosition().radians() * -1)

    def getLeftShooterSpeed(self) -> int:
        # RPM
        return self.leftShootingMotor.get(NEOBrushless.ControlMode.Velocity)

    def getRightShooterSpeed(self) -> int:
        # RPM
        return self.rightShootingMotor.get(NEOBrushless.ControlMode.Velocity)

    def angleOnTarget(self) -> bool:
        return (
            abs(self.targetAngle.radians() - self.getShooterAngle().radians())
            < constants.kShooterAngleTolerance.radians()
        )

    def leftMotorSpeedOnTarget(self) -> bool:
        return (
            abs(self.leftTargetSpeed - self.getLeftShooterSpeed())
            < constants.kShooterSpeedTolerance
        )

    def rightMotorSpeedOnTarget(self) -> bool:
        return (
            abs(self.rightTargetSpeed - self.getRightShooterSpeed())
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
            constants.kShooterAngleKey, self.getShooterAngle().radians()
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
