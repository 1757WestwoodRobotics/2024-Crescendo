from math import atan2, pi, sqrt
import typing

from commands2.command import Command
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Pose3d, Pose2d, Rotation2d
from wpilib import DriverStation, SmartDashboard, Preferences


from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.drivesubsystem import DriveSubsystem

import constants
from util.convenientmath import rotationFromTranslation


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

        self.forward = forward
        self.sideways = sideways

        self.targetPose = Pose3d()

        self.thetaController = ProfiledPIDControllerRadians(
            constants.kTrajectoryAnglePGain / 5,
            constants.kTrajectoryAngleIGain,
            constants.kTrajectoryAngleDGain,
            TrapezoidProfileRadians.Constraints(
                constants.kMaxRotationAngularVelocity,
                constants.kMaxRotationAngularAcceleration,
            ),
        )
        self.thetaController.enableContinuousInput(-pi, pi)

        self.addRequirements(shooterSubsystem, driveSubsystem)

        Preferences.initDouble("Shooter Extra", 2.7)
        Preferences.initDouble("Spin Amount", 100)

    def initialize(self):
        self.targetPose = (
            constants.kSpeakerCenterRed
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed
            else constants.kSpeakerCenterBlue
        )

        currentPose = self.drive.getPose()
        currentVel = Pose2d(
            *SmartDashboard.getNumberArray(constants.kDriveVelocityKeys, [0, 0, 0])
        )
        self.thetaController.reset(
            currentPose.rotation().radians(), currentVel.rotation().radians()
        )

    def execute(self):
        botPose = self.drive.getPose()
        targetPose2d = self.targetPose.toPose2d()

        deltaTranslation = botPose.translation() - targetPose2d.translation()

        angleToTarget = rotationFromTranslation(deltaTranslation)
        distanceToTarget = deltaTranslation.norm()

        extraYVel = Preferences.getDouble("Shooter Extra")

        vy = sqrt(
            extraYVel**2
            + (self.targetPose.Z() - constants.kRobotToShooterTransform.Z())
            * 2
            * constants.kGravity
        )
        airtime = (vy - extraYVel) / constants.kGravity
        vx = distanceToTarget / airtime

        launchAngle = atan2(vy, vx)  # radians
        launch_vel = sqrt(vx**2 + vy**2)  # m/s

        launch_vel_rpm = (
            launch_vel
            * constants.kSecondsPerMinute
            / constants.kShooterWheelRadius
            / constants.kRadiansPerRevolution
            / constants.kShootingMotorRatio
        )

        SmartDashboard.putNumber(constants.kShooterCalcSpeed, launch_vel_rpm)
        SmartDashboard.putNumber(constants.kShooterCalcAngle, launchAngle)

        spinAmount = Preferences.getDouble("Spin Amount", 100)

        self.shooter.setShooterAngle(Rotation2d(launchAngle))
        self.shooter.setLeftShootingMotorSpeed(launch_vel_rpm - spinAmount)
        self.shooter.setRightShootingMotorSpeed(launch_vel_rpm + spinAmount)

        # rotation pid gain
        rotation = self.thetaController.calculate(
            botPose.rotation().radians(), angleToTarget.radians()
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
