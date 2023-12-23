from enum import Enum, auto
from functools import partial

from typing import Tuple
from commands2 import Subsystem
from wpilib import (
    Encoder,
    PWMVictorSPX,
    RobotBase,
    SmartDashboard,
    Timer,
    DataLogManager,
)

from navx import AHRS
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)

import constants
from util import convenientmath
from util.angleoptimize import optimizeAngle
from util.simcoder import CTREEncoder
from util.simfalcon import Falcon

from pathplannerlib.auto import AutoBuilder


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


# pylint: disable-next=abstract-method
class PWMSwerveModule(SwerveModule):
    """
    Implementation of SwerveModule designed for ease of simulation:
        wheelMotor: 1:1 gearing with wheel
        swerveMotor: 1:1 gearing with swerve
        wheelEncoder: wheel distance (meters)
        swerveEncoder: swerve angle (radians)
    """

    def __init__(
        self,
        name: str,
        wheelMotor: PWMVictorSPX,
        swerveMotor: PWMVictorSPX,
        wheelEncoder: Encoder,
        swerveEncoder: Encoder,
    ) -> None:
        SwerveModule.__init__(self, name)
        self.wheelMotor = wheelMotor
        self.swerveMotor = swerveMotor
        self.wheelEncoder = wheelEncoder
        self.swerveEncoder = swerveEncoder

        self.wheelEncoder.setDistancePerPulse(1 / constants.kWheelEncoderPulsesPerMeter)
        self.swerveEncoder.setDistancePerPulse(
            1 / constants.kSwerveEncoderPulsesPerRadian
        )

    def getSwerveAngle(self) -> Rotation2d:
        return Rotation2d(self.swerveEncoder.getDistance())

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        swerveError = swerveAngleTarget.radians() - self.swerveEncoder.getDistance()
        swerveErrorClamped = min(max(swerveError, -1), 1)
        self.swerveMotor.set(swerveErrorClamped)

    def getWheelLinearVelocity(self) -> float:
        return self.wheelEncoder.getRate()

    def getWheelTotalPosition(self) -> float:
        return self.wheelEncoder.getDistance()

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        speedFactor = wheelLinearVelocityTarget / constants.kMaxWheelLinearVelocity
        speedFactorClamped = min(max(speedFactor, -1), 1)
        self.wheelMotor.set(speedFactorClamped)

    def reset(self) -> None:
        pass


class CTRESwerveModule(SwerveModule):
    """
    Implementation of SwerveModule for the SDS swerve modules
    https://www.swervedrivespecialties.com/collections/kits/products/mk4-swerve-module
        driveMotor: Falcon 500 Motor (with built-in encoder) attached to wheel through gearing
        steerMotor: Falcon 500 Motor (with built-in encoder) attached to swerve through gearing
        swerveEncoder: CANCoder
    """

    def __init__(self, name: str, config: SwerveModuleConfigParams) -> None:
        SwerveModule.__init__(self, name)
        DataLogManager.log(f"Initializing swerve module: {self.name}")
        DataLogManager.log(f"   Configuring drive motor: CAN ID: {config.driveMotorID}")
        self.driveMotor = Falcon(
            config.driveMotorID,
            constants.kDrivePIDSlot,
            constants.kDrivePGain,
            constants.kDriveIGain,
            constants.kDriveDGain,
            config.driveMotorInverted,
            config.canbus,
        )
        self.driveMotor.setCurrentLimit(constants.kDriveSupplyCurrentLimitConfiguration)
        DataLogManager.log("   ... Done")
        DataLogManager.log(f"   Configuring steer motor: CAN ID: {config.steerMotorID}")
        self.steerMotor = Falcon(
            config.steerMotorID,
            constants.kSteerPIDSlot,
            constants.kSteerPGain,
            constants.kSteerIGain,
            constants.kSteerDGain,
            config.steerMotorInverted,
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log(
            f"   Configuring swerve encoder: CAN ID: {config.swerveEncoderID}"
        )
        self.swerveEncoder = CTREEncoder(
            config.swerveEncoderID, config.swerveEncoderOffset
        )
        DataLogManager.log("   ... Done")
        DataLogManager.log("... Done")

    def getSwerveAngle(self) -> Rotation2d:
        steerEncoderPulses = self.steerMotor.get(Falcon.ControlMode.Position)
        swerveAngle = steerEncoderPulses / constants.kSwerveEncoderPulsesPerRadian
        return Rotation2d(swerveAngle)

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        steerEncoderPulses = (
            swerveAngle.radians()
        ) * constants.kSwerveEncoderPulsesPerRadian
        self.steerMotor.setEncoderPosition(steerEncoderPulses)

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        steerEncoderPulsesTarget = (
            swerveAngleTarget.radians() * constants.kSwerveEncoderPulsesPerRadian
        )
        self.steerMotor.set(Falcon.ControlMode.Position, steerEncoderPulsesTarget)

    def getWheelLinearVelocity(self) -> float:
        driveEncoderPulsesPerSecond = (
            self.driveMotor.get(Falcon.ControlMode.Velocity)
            * constants.k100MillisecondsPerSecond
        )
        wheelLinearVelocity = (
            driveEncoderPulsesPerSecond / constants.kWheelEncoderPulsesPerMeter
        )
        return wheelLinearVelocity

    def getWheelTotalPosition(self) -> float:
        driveEncoderPulses = self.driveMotor.get(Falcon.ControlMode.Position)
        driveDistance = (
            driveEncoderPulses
            / constants.kWheelEncoderPulsesPerRadian
            * constants.kWheelRadius
        )
        return driveDistance

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        driveEncoderPulsesPerSecond = (
            wheelLinearVelocityTarget * constants.kWheelEncoderPulsesPerMeter
        )
        self.driveMotor.set(
            Falcon.ControlMode.Velocity,
            driveEncoderPulsesPerSecond / constants.k100MillisecondsPerSecond,
        )

    def reset(self) -> None:
        self.setSwerveAngle(self.swerveEncoder.getPosition())


class DriveSubsystem(Subsystem):
    class CoordinateMode(Enum):
        RobotRelative = auto()
        FieldRelative = auto()
        TargetRelative = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        SmartDashboard.putBoolean(constants.kRobotPoseArrayKeys.validKey, False)

        self.rotationOffset = 0

        if RobotBase.isReal():
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
        else:
            self.frontLeftModule = PWMSwerveModule(
                constants.kFrontLeftModuleName,
                PWMVictorSPX(constants.kSimFrontLeftDriveMotorPort),
                PWMVictorSPX(constants.kSimFrontLeftSteerMotorPort),
                Encoder(*constants.kSimFrontLeftDriveEncoderPorts),
                Encoder(*constants.kSimFrontLeftSteerEncoderPorts),
            )
            self.frontRightModule = PWMSwerveModule(
                constants.kFrontRightModuleName,
                PWMVictorSPX(constants.kSimFrontRightDriveMotorPort),
                PWMVictorSPX(constants.kSimFrontRightSteerMotorPort),
                Encoder(*constants.kSimFrontRightDriveEncoderPorts),
                Encoder(*constants.kSimFrontRightSteerEncoderPorts),
            )
            self.backLeftModule = PWMSwerveModule(
                constants.kBackLeftModuleName,
                PWMVictorSPX(constants.kSimBackLeftDriveMotorPort),
                PWMVictorSPX(constants.kSimBackLeftSteerMotorPort),
                Encoder(*constants.kSimBackLeftDriveEncoderPorts),
                Encoder(*constants.kSimBackLeftSteerEncoderPorts),
            )
            self.backRightModule = PWMSwerveModule(
                constants.kBackRightModuleName,
                PWMVictorSPX(constants.kSimBackRightDriveMotorPort),
                PWMVictorSPX(constants.kSimBackRightSteerMotorPort),
                Encoder(*constants.kSimBackRightDriveEncoderPorts),
                Encoder(*constants.kSimBackRightSteerEncoderPorts),
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
        self.gyro = AHRS.create_spi()

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
            self
        )

    def getRobotRelativeSpeeds(self):
        return self.kinematics.toChassisSpeeds(*self.getModuleStates())

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
        self.gyro.reset()
        # self.gyro.setAngleAdjustment(pose.rotation().degrees())
        self.rotationOffset = pose.rotation().degrees()
        self.resetOdometryAtPosition(pose)

    def getPose(self) -> Pose2d:
        translation = self.odometry.getPose().translation()
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
                frontLeftState.angle.degrees(),
                frontLeftState.speed,
                frontRightState.angle.degrees(),
                frontRightState.speed,
                backLeftState.angle.degrees(),
                backLeftState.speed,
                backRightState.angle.degrees(),
                backRightState.speed,
            ],
        )
        self.frontLeftModule.applyState(frontLeftState)
        self.frontRightModule.applyState(frontRightState)
        self.backLeftModule.applyState(backLeftState)
        self.backRightModule.applyState(backRightState)

    def getRotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(
            ((self.gyro.getRotation2d().degrees() / 0.98801) % 360)
            + self.rotationOffset
        )

    def getPitch(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.gyro.getPitch() + 180)

    def resetOdometryAtPosition(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.getRotation(),
            pose,
            self.frontLeftModule.getPosition(),
            self.frontRightModule.getPosition(),
            self.backLeftModule.getPosition(),
            self.backRightModule.getPosition(),
        )

    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's
        odometry with sensor data.
        """

        pastPose = self.odometry.getPose()

        self.odometry.update(
            self.getRotation(),
            self.frontLeftModule.getPosition(),
            self.frontRightModule.getPosition(),
            self.backLeftModule.getPosition(),
            self.backRightModule.getPosition(),
        )
        robotPose = self.getPose()

        deltaPose = robotPose - pastPose
        SmartDashboard.putNumberArray(
            constants.kSwerveActualStatesKey,
            [
                self.frontLeftModule.getSwerveAngle().degrees(),
                self.frontLeftModule.getWheelLinearVelocity(),
                self.frontRightModule.getSwerveAngle().degrees(),
                self.frontRightModule.getWheelLinearVelocity(),
                self.backLeftModule.getSwerveAngle().degrees(),
                self.backLeftModule.getWheelLinearVelocity(),
                self.backRightModule.getSwerveAngle().degrees(),
                self.backRightModule.getWheelLinearVelocity(),
            ],
        )
        SmartDashboard.putNumberArray(
            constants.kDriveVelocityKeys,
            [
                deltaPose.X()
                / constants.kRobotUpdatePeriod,  # velocity is delta pose / delta time
                deltaPose.Y() / constants.kRobotUpdatePeriod,
                deltaPose.rotation().radians() / constants.kRobotUpdatePeriod,
            ],
        )

        robotPoseArray = [robotPose.X(), robotPose.Y(), robotPose.rotation().radians()]

        if SmartDashboard.getBoolean(
            constants.kRobotVisionPoseArrayKeys.validKey, False
        ):
            visionPose = self.visionEstimate

            weightedPose = Pose2d(
                visionPose.X() * constants.kRobotVisionPoseWeight
                + robotPose.X() * (1 - constants.kRobotVisionPoseWeight),
                visionPose.Y() * constants.kRobotVisionPoseWeight
                + robotPose.Y() * (1 - constants.kRobotVisionPoseWeight),
                robotPose.rotation(),
            )
            self.resetOdometryAtPosition(weightedPose)

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

        robotChassisSpeeds = None
        if coordinateMode is DriveSubsystem.CoordinateMode.RobotRelative:
            robotChassisSpeeds = chassisSpeeds
        elif coordinateMode is DriveSubsystem.CoordinateMode.FieldRelative:
            robotChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vx,
                chassisSpeeds.vy,
                chassisSpeeds.omega,
                self.getRotation(),
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

        moduleStates = self.kinematics.toSwerveModuleStates(robotChassisSpeeds)
        self.applyStates(moduleStates)
