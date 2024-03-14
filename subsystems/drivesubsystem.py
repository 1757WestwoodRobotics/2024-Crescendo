from enum import Enum, auto
from functools import partial

from typing import Tuple
import typing
from commands2 import Subsystem
from phoenix6.configs.pigeon2_configs import Pigeon2Configuration
from phoenix6.hardware.pigeon2 import Pigeon2
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from wpilib import (
    RobotBase,
    SmartDashboard,
    Timer,
    DataLogManager,
    DriverStation,
)

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)
from wpimath.estimator import SwerveDrive4PoseEstimator

from pathplannerlib.auto import AutoBuilder

import constants
from util import convenientmath
from util.angleoptimize import optimizeAngle
from util.simcoder import CTREEncoder
from util.simtalon import Talon
from subsystems.visionsubsystem import VisionSubsystem


class SwerveModuleConfigParams:
    swerveEncoderOffset: float
    swerveEncoderID: int
    driveMotorID: int
    driveMotorInverted: bool
    steerMotorID: int
    steerMotorInverted: bool
    canbus: str = ""

    def __init__(
        self,
        driveMotorID: int,
        driveMotorInverted: bool,
        steerMotorID: int,
        steerMotorInverted: bool,
        swerveEncoderID: int,
        swerveEncoderOffset: float,
        canbus: str = "",
    ) -> None:
        self.driveMotorID = driveMotorID
        self.driveMotorInverted = driveMotorInverted
        self.steerMotorID = steerMotorID
        self.steerMotorInverted = steerMotorInverted
        self.swerveEncoderID = swerveEncoderID
        self.swerveEncoderOffset = swerveEncoderOffset
        self.canbus = canbus


class SwerveModule:
    def __init__(self, name: str) -> None:
        self.name = name

    def getSwerveAngle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelLinearVelocity(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def getWheelTotalPosition(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def reset(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def optimizedAngle(self, targetAngle: Rotation2d) -> Rotation2d:
        return optimizeAngle(self.getSwerveAngle(), targetAngle)

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getWheelTotalPosition(), self.getSwerveAngle())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.getWheelLinearVelocity(),
            self.getSwerveAngle(),
        )

    def applyState(self, state: SwerveModuleState) -> None:
        optimizedState = SwerveModuleState.optimize(state, self.getSwerveAngle())

        self.setWheelLinearVelocityTarget(optimizedState.speed)
        if (
            abs(optimizedState.speed) >= constants.kMinWheelLinearVelocity
        ):  # prevent unneccisary movement for what would otherwise not move the robot
            optimizedAngle = self.optimizedAngle(optimizedState.angle)
            self.setSwerveAngleTarget(optimizedAngle)


class CTRESwerveModule(SwerveModule):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Kraken X60 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    def __init__(self, name: str, config: SwerveModuleConfigParams) -> None:
        SwerveModule.__init__(self, name)
        DataLogManager.log(f"Initializing swerve module: {self.name}")
        DataLogManager.log(f"   Configuring drive motor: CAN ID: {config.driveMotorID}")
        self.driveMotor = Talon(
            config.driveMotorID,
            f"Drive Motor {name}",
            constants.kDrivePGain,
            constants.kDriveIGain,
            constants.kDriveDGain,
            config.driveMotorInverted,
            config.canbus,
            constants.kDriveVGain,
        )
        if RobotBase.isReal():
            self.driveMotor.setCurrentLimit(constants.kDriveCurrentLimit)
        DataLogManager.log("   ... Done")
        DataLogManager.log(f"   Configuring steer motor: CAN ID: {config.steerMotorID}")
        self.steerMotor = Talon(
            config.steerMotorID,
            f"Steer Motor {name}",
            constants.kSteerPGain,
            constants.kSteerIGain,
            constants.kSteerDGain,
            config.steerMotorInverted,
            config.canbus,
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log(
            f"   Configuring swerve encoder: CAN ID: {config.swerveEncoderID}"
        )
        self.swerveEncoder = CTREEncoder(
            config.swerveEncoderID, config.swerveEncoderOffset, config.canbus
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log("... Done")

    def getSwerveAngle(self) -> Rotation2d:
        steerRotation = self.steerMotor.get(Talon.ControlMode.Position)
        swerveAngle = (
            steerRotation
            / constants.kSteerGearingRatio
            * constants.kRadiansPerRevolution
        )
        return Rotation2d(swerveAngle)

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        steerEncoderPulses = (
            (swerveAngle.radians())
            / constants.kRadiansPerRevolution
            * constants.kSteerGearingRatio
        )
        self.steerMotor.setEncoderPosition(steerEncoderPulses)

    def getSwerveEncoderAngle(self) -> Rotation2d:
        return self.swerveEncoder.getPosition()

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        steerEncoderPulsesTarget = (
            swerveAngleTarget.radians()
            / constants.kRadiansPerRevolution
            * constants.kSteerGearingRatio
        )
        self.steerMotor.set(Talon.ControlMode.Position, steerEncoderPulsesTarget)

    def getWheelLinearVelocity(self) -> float:
        driveEncoderPulsesPerSecond = self.driveMotor.get(Talon.ControlMode.Velocity)
        wheelLinearVelocity = (
            driveEncoderPulsesPerSecond
            * constants.kWheelRadius
            * constants.kRadiansPerRevolution
            / constants.kDriveGearingRatio
        )
        return wheelLinearVelocity

    def getWheelTotalPosition(self) -> float:
        driveEncoderPulses = self.driveMotor.get(Talon.ControlMode.Position)
        driveDistance = (
            driveEncoderPulses
            * constants.kWheelRadius
            * constants.kRadiansPerRevolution
            / constants.kDriveGearingRatio
        )
        return driveDistance

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        driveEncoderPulsesPerSecond = (
            wheelLinearVelocityTarget
            / constants.kWheelRadius
            / constants.kRadiansPerRevolution
            * constants.kDriveGearingRatio
        )
        self.driveMotor.set(
            Talon.ControlMode.Velocity,
            driveEncoderPulsesPerSecond,
        )

    def reset(self) -> None:
        self.setSwerveAngle(self.swerveEncoder.getPosition())

    def getSimulator(
        self,
    ) -> tuple[
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], TalonFXSimState],
        typing.Callable[[], CANcoderSimState],
    ]:
        return (
            self.driveMotor.getSimCollection,
            self.steerMotor.getSimCollection,
            self.swerveEncoder.getSim,
        )


class DriveSubsystem(Subsystem):
    class CoordinateMode(Enum):
        RobotRelative = auto()
        FieldRelative = auto()
        TargetRelative = auto()

    def __init__(self, vision: VisionSubsystem) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        SmartDashboard.putBoolean(constants.kRobotPoseArrayKeys.validKey, False)

        self.vision = vision
        self.rotationOffset = 0

        self.frontLeftModule = CTRESwerveModule(
            constants.kFrontLeftModuleName,
            SwerveModuleConfigParams(
                constants.kFrontLeftDriveMotorId,
                constants.kFrontLeftDriveInverted,
                constants.kFrontLeftSteerMotorId,
                constants.kFrontLeftSteerInverted,
                constants.kFrontLeftSteerEncoderId,
                constants.kFrontLeftAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )
        self.frontRightModule = CTRESwerveModule(
            constants.kFrontRightModuleName,
            SwerveModuleConfigParams(
                constants.kFrontRightDriveMotorId,
                constants.kFrontRightDriveInverted,
                constants.kFrontRightSteerMotorId,
                constants.kFrontRightSteerInverted,
                constants.kFrontRightSteerEncoderId,
                constants.kFrontRightAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )
        self.backLeftModule = CTRESwerveModule(
            constants.kBackLeftModuleName,
            SwerveModuleConfigParams(
                constants.kBackLeftDriveMotorId,
                constants.kBackLeftDriveInverted,
                constants.kBackLeftSteerMotorId,
                constants.kBackLeftSteerInverted,
                constants.kBackLeftSteerEncoderId,
                constants.kBackLeftAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )
        self.backRightModule = CTRESwerveModule(
            constants.kBackRightModuleName,
            SwerveModuleConfigParams(
                constants.kBackRightDriveMotorId,
                constants.kBackRightDriveInverted,
                constants.kBackRightSteerMotorId,
                constants.kBackRightSteerInverted,
                constants.kBackRightSteerEncoderId,
                constants.kBackRightAbsoluteEncoderOffset,
                constants.kCANivoreName,
            ),
        )

        self.modules = (
            self.frontLeftModule,
            self.frontRightModule,
            self.backLeftModule,
            self.backRightModule,
        )

        self.kinematics = SwerveDrive4Kinematics(
            constants.kFrontLeftWheelPosition,
            constants.kFrontRightWheelPosition,
            constants.kBackLeftWheelPosition,
            constants.kBackRightWheelPosition,
        )

        # Create the gyro, a sensor which can indicate the heading of the robot relative
        # to a customizable position.
        self.gyro = Pigeon2(constants.kPigeonCANId, constants.kCANivoreName)

        toApply = Pigeon2Configuration()
        self.gyro.configurator.apply(toApply)
        self.gyro.get_yaw().set_update_frequency(100)

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.getRotation(),
            [
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ],
            Pose2d(),
            [0.1, 0.1, 0.1],
            [0.2, 0.2, 0.2],
        )
        # standard deviations stolen from 2910

        # Create the an object for our odometry, which will utilize sensor data to
        # keep a record of our position on the field.
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            Pose2d(),
        )
        self.printTimer = Timer()
        self.vxLimiter = SlewRateLimiter(constants.kDriveAccelLimit)
        self.vyLimiter = SlewRateLimiter(constants.kDriveAccelLimit)

        self.visionEstimate = Pose2d()

        AutoBuilder.configureHolonomic(
            self.getPose,
            self.resetGyro,
            self.getRobotRelativeSpeeds,
            partial(
                self.arcadeDriveWithSpeeds,
                coordinateMode=DriveSubsystem.CoordinateMode.RobotRelative,
            ),
            constants.kPathFollowingConfig,
            (lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed),
            self,
        )

    def getRobotRelativeSpeeds(self):
        return self.kinematics.toChassisSpeeds(self.getModuleStates())

    def getModuleStates(self):
        return (
            self.frontLeftModule.getState(),
            self.frontRightModule.getState(),
            self.backLeftModule.getState(),
            self.backRightModule.getState(),
        )

    def resetSwerveModules(self):
        for module in self.modules:
            module.reset()
        self.resetGyro(Pose2d())

    def setOdometryPosition(self, pose: Pose2d):
        # self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.rotationOffset = pose.rotation().degrees()
        self.resetOdometryAtPosition(pose)

    def resetGyro(self, pose: Pose2d):
        self.gyro.set_yaw(0)
        # self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.rotationOffset = pose.rotation().degrees()
        self.resetOdometryAtPosition(pose)

        if RobotBase.isSimulation():
            self.resetSimPosition(pose)

    def getPose(self) -> Pose2d:
        translation = self.estimator.getEstimatedPosition().translation()
        rotation = self.getRotation()
        return Pose2d(translation, rotation)

    def applyStates(self, moduleStates: Tuple[SwerveModuleState]) -> None:
        (
            frontLeftState,
            frontRightState,
            backLeftState,
            backRightState,
        ) = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            moduleStates, constants.kMaxWheelLinearVelocity
        )

        SmartDashboard.putNumberArray(
            constants.kSwerveExpectedStatesKey,
            [
                frontLeftState.angle.radians(),
                frontLeftState.speed,
                frontRightState.angle.radians(),
                frontRightState.speed,
                backLeftState.angle.radians(),
                backLeftState.speed,
                backRightState.angle.radians(),
                backRightState.speed,
            ],
        )
        self.frontLeftModule.applyState(frontLeftState)
        self.frontRightModule.applyState(frontRightState)
        self.backLeftModule.applyState(backLeftState)
        self.backRightModule.applyState(backRightState)

    def getRotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value + self.rotationOffset)

    def getAngularVelocity(self) -> float:
        """radians"""
        if RobotBase.isSimulation():
            return SmartDashboard.getNumberArray(
                constants.kSimRobotVelocityArrayKey, [0, 0, 0]
            )[2]
        return (
            self.gyro.get_angular_velocity_z_world().value * constants.kRadiansPerDegree
        )

    def getPitch(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.gyro.get_pitch().value + 180)

    def resetOdometryAtPosition(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            pose,
        )
        self.estimator.resetPosition(
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
            pose,
        )

    def getClosestWaypoint(self):
        return (
            self.getPose().nearest(constants.kWaypointsBlue)
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue
            else self.getPose().nearest(constants.kWaypointsRed)
        )

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """

        self.odometry.update(
            self.getRotation(),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ),
        )
        robotPose = self.getPose()

        SmartDashboard.putNumberArray(
            constants.kSwerveActualStatesKey,
            [
                self.frontLeftModule.getSwerveEncoderAngle().radians(),
                self.frontLeftModule.getWheelLinearVelocity(),
                self.frontRightModule.getSwerveEncoderAngle().radians(),
                self.frontRightModule.getWheelLinearVelocity(),
                self.backLeftModule.getSwerveEncoderAngle().radians(),
                self.backLeftModule.getWheelLinearVelocity(),
                self.backRightModule.getSwerveEncoderAngle().radians(),
                self.backRightModule.getWheelLinearVelocity(),
            ],
        )

        robotPoseArray = [robotPose.X(), robotPose.Y(), robotPose.rotation().radians()]

        SmartDashboard.putNumberArray(
            constants.kRobotPoseArrayKeys.valueKey, robotPoseArray
        )
        SmartDashboard.putBoolean(constants.kRobotPoseArrayKeys.validKey, True)

        if self.printTimer.hasElapsed(constants.kPrintPeriod):
            DataLogManager.log(
                # pylint:disable-next=consider-using-f-string
                "r: {:.1f}, {:.1f}, {:.0f}* fl: {:.0f}* {:.1f} fr: {:.0f}* {:.1f} bl: {:.0f}* {:.1f} br: {:.0f}* {:.1f}".format(
                    robotPose.X(),
                    robotPose.Y(),
                    robotPose.rotation().degrees(),
                    self.frontLeftModule.getSwerveAngle().degrees(),
                    self.frontLeftModule.getWheelLinearVelocity(),
                    self.frontRightModule.getSwerveAngle().degrees(),
                    self.frontRightModule.getWheelLinearVelocity(),
                    self.backLeftModule.getSwerveAngle().degrees(),
                    self.backLeftModule.getWheelLinearVelocity(),
                    self.backRightModule.getSwerveAngle().degrees(),
                    self.backRightModule.getWheelLinearVelocity(),
                )
            )

        estimatedCameraPoses = self.vision.poseList
        hasTargets = False

        for estimatedCameraPose in estimatedCameraPoses:
            if estimatedCameraPose.hasTargets:
                self.estimator.addVisionMeasurement(
                    estimatedCameraPose.pose.toPose2d(),
                    estimatedCameraPose.timestamp,
                )
                hasTargets = True

        self.estimator.updateWithTime(
            self.printTimer.getFPGATimestamp(),
            self.odometry.getPose().rotation(),
            [
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.backLeftModule.getPosition(),
                self.backRightModule.getPosition(),
            ],
        )
        self.vision.poseList.clear()

        self.visionEstimate = self.estimator.getEstimatedPosition()

        SmartDashboard.putBoolean(
            constants.kRobotVisionPoseArrayKeys.validKey, hasTargets
        )
        SmartDashboard.putNumberArray(
            constants.kRobotVisionPoseArrayKeys.valueKey,
            [
                self.visionEstimate.X(),
                self.visionEstimate.Y(),
                self.visionEstimate.rotation().radians(),
            ],
        )

    def arcadeDriveWithFactors(
        self,
        forwardSpeedFactor: float,
        sidewaysSpeedFactor: float,
        rotationSpeedFactor: float,
        coordinateMode: CoordinateMode,
    ) -> None:
        """
        Drives the robot using arcade controls.

        :param forwardSpeedFactor: the commanded forward movement
        :param sidewaysSpeedFactor: the commanded sideways movement
        :param rotationSpeedFactor: the commanded rotation
        """

        forwardSpeedFactor = convenientmath.clamp(forwardSpeedFactor, -1, 1)
        sidewaysSpeedFactor = convenientmath.clamp(sidewaysSpeedFactor, -1, 1)
        rotationSpeedFactor = convenientmath.clamp(rotationSpeedFactor, -1, 1)

        combinedLinearFactor = Translation2d(
            forwardSpeedFactor, sidewaysSpeedFactor
        ).norm()

        # prevent combined forward & sideways inputs from exceeding the max linear velocity
        if combinedLinearFactor > 1.0:
            forwardSpeedFactor = forwardSpeedFactor / combinedLinearFactor
            sidewaysSpeedFactor = sidewaysSpeedFactor / combinedLinearFactor

        chassisSpeeds = ChassisSpeeds(
            forwardSpeedFactor * constants.kMaxForwardLinearVelocity,
            sidewaysSpeedFactor * constants.kMaxSidewaysLinearVelocity,
            rotationSpeedFactor * constants.kMaxRotationAngularVelocity,
        )

        self.arcadeDriveWithSpeeds(chassisSpeeds, coordinateMode)

    def arcadeDriveWithSpeeds(
        self, chassisSpeeds: ChassisSpeeds, coordinateMode: CoordinateMode
    ) -> None:
        targetAngle = Rotation2d(
            SmartDashboard.getNumber(
                constants.kTargetAngleRelativeToRobotKeys.valueKey, 0
            )
        )
        discritizedSpeeds = ChassisSpeeds.discretize(
            chassisSpeeds, constants.kRobotUpdatePeriod
        )

        robotChassisSpeeds = None
        if coordinateMode is DriveSubsystem.CoordinateMode.RobotRelative:
            robotChassisSpeeds = discritizedSpeeds
        elif coordinateMode is DriveSubsystem.CoordinateMode.FieldRelative:
            robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                discritizedSpeeds.vx,
                discritizedSpeeds.vy,
                discritizedSpeeds.omega,
                self.getRotation()
                + Rotation2d(
                    self.getAngularVelocity() * constants.kDriveAngularVelocityCoeff
                ),
            )
        elif coordinateMode is DriveSubsystem.CoordinateMode.TargetRelative:
            if SmartDashboard.getBoolean(
                constants.kTargetAngleRelativeToRobotKeys.validKey, False
            ):
                robotSpeeds = Translation2d(chassisSpeeds.vx, chassisSpeeds.vy)
                targetAlignedSpeeds = robotSpeeds.rotateBy(targetAngle)
                robotChassisSpeeds = ChassisSpeeds(
                    targetAlignedSpeeds.X(),
                    targetAlignedSpeeds.Y(),
                    chassisSpeeds.omega,
                )
            else:
                robotChassisSpeeds = ChassisSpeeds()

        fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            robotChassisSpeeds.vx,
            robotChassisSpeeds.vy,
            robotChassisSpeeds.omega,
            -self.getRotation(),
        )

        SmartDashboard.putNumberArray(
            constants.kDriveVelocityKeys,
            [fieldSpeeds.vx, fieldSpeeds.vy, fieldSpeeds.omega],
        )

        moduleStates = self.kinematics.toSwerveModuleStates(robotChassisSpeeds)
        self.applyStates(moduleStates)
