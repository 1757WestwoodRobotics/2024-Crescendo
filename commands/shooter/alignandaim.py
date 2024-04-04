from math import atan2, pi, sqrt, cos
import typing

from commands2.command import Command
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Pose3d, Rotation2d, Translation3d, Translation2d
from wpilib import DriverStation, SmartDashboard, Preferences, Timer


from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.drivesubsystem import DriveSubsystem

import constants
from util.convenientmath import deadband, rotationFromTranslation
from util.getsdarray import getSDArray


class AlignAndAim(Command):
    def __init__(
        self,
        shooterSubsystem: ShooterSubsystem,
        driveSubsystem: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
    ) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.shooter = shooterSubsystem
        self.drive = driveSubsystem
        self.t = Timer()

        self.doFlip = False

        self.forward = forward
        self.sideways = sideways

        self.targetPose = Pose3d()

        self.thetaController = ProfiledPIDControllerRadians(
            constants.kAutoAimPGain,
            constants.kAutoAimIGain,
            constants.kAutoAimDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.thetaController.enableContinuousInput(-pi, pi)
        self.thetaController.setTolerance(0.1)

        self.addRequirements(shooterSubsystem, driveSubsystem)

        Preferences.initDouble("Shooter Extra", 2.7)
        Preferences.initDouble("Spin Amount", 100)

    def initialize(self):
        self.targetPose = (
            constants.kSpeakerCenterRed
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed
            else constants.kSpeakerCenterBlue
        )

        # currentVel = SmartDashboard.getNumberArray(
        #     constants.kDriveVelocityKeys, [0, 0, 0]
        # )

        self.thetaController.reset(self.drive.getRotation().radians())

        self.t.reset()
        self.t.start()
        self.doFlip = not self.doFlip

    def calculateTimeVelocityAngle(
        self, position: Translation3d
    ) -> typing.Tuple[float, float, Rotation2d, Rotation2d, float]:
        botPose = self.drive.estimator.getEstimatedPosition()
        target2d = position.toTranslation2d()

        deltaTranslation = botPose.translation() - target2d

        angleToTarget = rotationFromTranslation(deltaTranslation)
        distanceToTarget = deltaTranslation.norm()

        launch_vel = 26  # m/s
        launchAngle = atan2(
            self.targetPose.Z() - constants.kRobotToShooterTransform.Z(),
            distanceToTarget,
        )
        # vy = sqrt(
        #     extraYVel**2
        #     + (self.targetPose.Z() - constants.kRobotToShooterTransform.Z())
        #     * 2
        #     * constants.kGravity
        # )
        airtime = distanceToTarget / (launch_vel * cos(launchAngle))
        # vx = distanceToTarget / airtime

        # launchAngle = atan2(vy, vx)  # radians
        angleAdjust = constants.kShooterAngleAdjustmentMappingFunction(launchAngle)

        return (
            airtime,
            launch_vel,
            Rotation2d(launchAngle + angleAdjust),
            angleToTarget,
            distanceToTarget,
        )

    def execute(self):
        botPose = self.drive.estimator.getEstimatedPosition()
        robotVelocity = getSDArray(constants.kDriveVelocityKeys, [0, 0, 0])
        time, velocity, psi, theta, distance = 0, 0, Rotation2d(), Rotation2d(), 0
        target = self.targetPose.translation()

        newTarget = target

        for _ in range(
            constants.kShooterMovingIterations
        ):  # iterative solver for moving shot
            time, velocity, psi, theta, distance = self.calculateTimeVelocityAngle(
                newTarget
            )

            positionChange = Translation2d(
                robotVelocity[0] * time, robotVelocity[1] * time
            )
            newTarget = target - Translation3d(
                positionChange.X(), positionChange.Y(), 0
            )

        launch_vel_rpm = (
            velocity
            * constants.kSecondsPerMinute
            / constants.kShooterWheelRadius
            / constants.kRadiansPerRevolution
            / constants.kShootingMotorRatio
        )

        SmartDashboard.putNumber(constants.kShooterCalcSpeed, velocity)
        SmartDashboard.putNumber(constants.kShooterCalcDistance, distance)
        SmartDashboard.putNumber(constants.kShooterCalcAngle, psi.radians())

        spinAmount = Preferences.getDouble("Spin Amount", 100) * (
            # 1 if self.doFlip else -1
            1
        )

        self.shooter.setShooterAngle(psi)
        self.shooter.setLeftShootingMotorSpeed(launch_vel_rpm - spinAmount)
        self.shooter.setRightShootingMotorSpeed(launch_vel_rpm + spinAmount)

        # rotation pid gain
        rotation = deadband(
            self.thetaController.calculate(
                botPose.rotation().radians(), theta.radians()
            ),
            constants.kRotationAlignDeadband.radians(),
        )

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.drive.arcadeDriveWithFactors(
                -self.forward(),
                -self.sideways(),
                rotation,
                DriveSubsystem.CoordinateMode.FieldRelative,
            )
        else:
            self.drive.arcadeDriveWithFactors(
                self.forward(),
                self.sideways(),
                rotation,
                DriveSubsystem.CoordinateMode.FieldRelative,
            )

    def isFinished(self) -> bool:
        return (
            self.shooter.readyToShoot()
            and self.t.get() > 0.5
            and self.thetaController.atGoal()
        )
