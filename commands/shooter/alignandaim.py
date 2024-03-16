from math import atan2, pi, sqrt
import typing

from commands2.command import Command
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.geometry import Pose3d, Rotation2d, Translation3d, Translation2d
from wpilib import DriverStation, SmartDashboard, Preferences, Timer


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
        self.t = Timer()

        self.forward = forward
        self.sideways = sideways

        self.targetPose = Pose3d()

        self.thetaController = ProfiledPIDControllerRadians(
            constants.kAlignAnglePGain,
            constants.kAlignAngleIGain,
            constants.kAlignAngleDGain,
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

    def calculateTimeVelocityAngle(
        self, position: Translation3d
    ) -> typing.Tuple[float, float, Rotation2d, Rotation2d]:
        botPose = self.drive.getPose()
        target2d = position.toTranslation2d()

        deltaTranslation = botPose.translation() - target2d

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
        angleAdjust = constants.kShooterAngleAdjustmentMappingFunction(distanceToTarget)
        launch_vel = sqrt(vx**2 + vy**2)  # m/s

        return airtime, launch_vel, Rotation2d(launchAngle + angleAdjust), angleToTarget

    def execute(self):
        botPose = self.drive.getPose()
        robotVelocity = SmartDashboard.getNumberArray(
            constants.kDriveVelocityKeys, [0, 0, 0]
        )
        time, velocity, psi, theta = 0, 0, Rotation2d(), Rotation2d()
        target = self.targetPose.translation()

        newTarget = target

        for _ in range(
            constants.kShooterMovingIterations
        ):  # iterative solver for moving shot
            time, velocity, psi, theta = self.calculateTimeVelocityAngle(newTarget)

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
        SmartDashboard.putNumber(constants.kShooterCalcAngle, psi.radians())

        spinAmount = Preferences.getDouble("Spin Amount", 100)

        self.shooter.setShooterAngle(psi)
        self.shooter.setLeftShootingMotorSpeed(launch_vel_rpm - spinAmount)
        self.shooter.setRightShootingMotorSpeed(launch_vel_rpm + spinAmount)

        # rotation pid gain
        rotation = self.thetaController.calculate(
            botPose.rotation().radians(), theta.radians()
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
        return self.shooter.readyToShoot() and self.t.get() > 0.5 and self.thetaController.atGoal()
