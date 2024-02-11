"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!

Physical constants must have their units specified
Default units:
    Length: meters
    Angle: radians

Axes Convention (right hand rule):
    Translation:
        +X: forward
        +Y: left
        +Z: up

    Rotation:
        +rotate around X: counterclockwise looking from the front, 0 aligned with +Y
        +rotate around Y: counterclockwise looking from the left, 0 aligned with +Z
        +rotate around Z: counterclockwise looking from the top, 0 aligned with +X

Swerve Module Layout:
    Drive (input) -> Drive Gearing -> Wheel (output)
    Steer (input) -> Steer Gearing -> Swerve (output)
"""

import math
from phoenix6.configs.config_groups import CurrentLimitsConfigs
from wpimath.geometry import (
    Pose3d,
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation2d,
)
from wpimath.system.plant import DCMotor
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    PIDConstants,
    ReplanningConfig,
)

from util.keyorganization import OptionalValueKeys

# Basic units
kInchesPerFoot = 12
"""inches / foot"""

kCentimetersPerInch = 2.54
"""centimeters / inch"""

kCentimetersPerMeter = 100
"""centimeters / meter"""

kMetersPerInch = kCentimetersPerInch / kCentimetersPerMeter
"""meters / inch"""

kMetersPerFoot = kMetersPerInch * kInchesPerFoot
"""meters / foot"""

kRadiansPerRevolution = 2 * math.pi
"""radians / revolution"""

kDegeersPerRevolution = 360
"""degrees / revolution"""

kRadiansPerDegree = kRadiansPerRevolution / kDegeersPerRevolution
"""radians / degree"""

kMillisecondsPerSecond = 1000 / 1
"""milliseconds / second"""

kSecondsPerMinute = 60 / 1
"""seconds / minute"""

kRPMPerAngularVelocity = (1 / kRadiansPerRevolution) * kSecondsPerMinute
"""RPM / (radians / second)"""

# Debug parameters
kPrintFrequency = 2
""" 1 / second"""

kPrintPeriod = 1 / kPrintFrequency
"""seconds"""

# Field Physical parameters
kFieldLength = 54 * kMetersPerFoot + 3.25 * kMetersPerInch
"""meters"""

kFieldWidth = 26 * kMetersPerFoot + 3.5 * kMetersPerInch
"""meters"""

# Robot Physical parameters
kRobotWidth = 26 * kMetersPerInch
"""meters"""

kRobotLength = 24 * kMetersPerInch
"""meters"""

kSwerveModuleCenterToRobotCenterWidth = 10.375 * kMetersPerInch
"""meters"""
kSwerveModuleCenterToRobotCenterLength = 9.375 * kMetersPerInch
"""meters"""

kSwerveModuleDistanceFromRobotCenter = pow(
    pow(kSwerveModuleCenterToRobotCenterWidth, 2)
    + pow(kSwerveModuleCenterToRobotCenterLength, 2),
    0.5,
)
"""meters (c = (a^2 + b^2) ^ 0.5)"""

kFrontLeftWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kFrontRightWheelPosition = Translation2d(
    kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackLeftWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kBackRightWheelPosition = Translation2d(
    -kSwerveModuleCenterToRobotCenterLength,
    -kSwerveModuleCenterToRobotCenterWidth,
)
"""[meters, meters]"""

kWheelDiameter = 4 * kMetersPerInch
"""meters"""

kWheelRadius = kWheelDiameter / 2
"""meters"""

kWheelCircumference = kWheelRadius * 2 * math.pi
"""meters"""

kWheelDistancePerRevolution = kWheelCircumference
"""meters / revolution"""

kWheelDistancePerRadian = kWheelDistancePerRevolution / kRadiansPerRevolution
"""meters / radian"""

kDriveGearingRatio = (50 / 14) * (16 / 28) * (45 / 15)
"""dimensionless"""

kSteerGearingRatio = 150 / 7
"""dimensionless"""

kMaxMotorAngularVelocity = DCMotor.krakenX60().freeSpeed
"""radians / second"""

kMaxWheelAngularVelocity = kMaxMotorAngularVelocity / kDriveGearingRatio
"""radians / second"""

kMaxWheelLinearVelocity = kWheelDistancePerRadian * kMaxWheelAngularVelocity
"""meters / second"""

kMinWheelLinearVelocity = 0.002
"""meters / second"""

kMaxSteerAngularVelocity = kMaxMotorAngularVelocity / kSteerGearingRatio
"""radians / second"""

kMaxForwardLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxSidewaysLinearVelocity = kMaxWheelLinearVelocity
"""meters / second"""

kMaxRotationAngularVelocity = (
    kMaxWheelLinearVelocity / kSwerveModuleDistanceFromRobotCenter
)
"""radians / second (omega = v / r)"""

kMaxWheelLinearAcceleration = kMaxWheelLinearVelocity / 1
"""meters / second^2"""

kMaxForwardLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxSidewaysLinearAcceleration = kMaxWheelLinearAcceleration
"""meters / second^2"""

kMaxRotationAngularAcceleration = kMaxRotationAngularVelocity / 0.5
"""radians / second^2"""

kFrontLeftModuleName = "front_left"
kFrontRightModuleName = "front_right"
kBackLeftModuleName = "back_left"
kBackRightModuleName = "back_right"

kKilogramToLbs = 0.454

# Limelight
kLimelightTargetInvalidValue = 0.0
kLimelightTargetValidValue = 1.0
kLimelightMinHorizontalFoV = Rotation2d.fromDegrees(-29.8)
kLimelightMaxHorizontalFoV = Rotation2d.fromDegrees(29.8)
kLimelightMinVerticalFoV = Rotation2d.fromDegrees(-22.85)
kLimelightMaxVerticalFoV = Rotation2d.fromDegrees(22.85)
kLimelightNetworkTableName = "limelight"
kLimelightTargetValidKey = "tv"
kLimelightTargetHorizontalAngleKey = "tx"
kLimelightTargetVerticalAngleKey = "ty"
kLimelightLEDModeKey = "ledMode"
kLimelightTrackerModuleName = "limelight"
kLimelightRelativeToRobotTransform = Transform3d(
    Pose3d(),
    Pose3d(0.236, 0.206, 0.197, Rotation3d()),
)

# Photonvision related
kPhotonvisionCameraName = "camcam"
kPhotonvisionCameraArray = ["frontLeft", "backRight"]

kPhotonvisionFrontLeftCameraKey = "frontLeftCamera"
kPhotonvisionFrontRightCameraKey = "frontRightCamera"
kPhotonvisionBackLeftCameraKey = "backLeftCamera"
kPhotonvisionBackRightCameraKey = "backRightCamera"


kRobotToFrontLeftCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        10.698 * kMetersPerInch,
        9.707 * kMetersPerInch,
        6.063 * kMetersPerInch,
        Rotation3d(),
    ),
) + Transform3d(
    0.006,
    -0.006,
    0.051,
    Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
        Rotation3d(0.0, 0.0, (270 + 82.829) * kRadiansPerDegree)
    ),
)
kRobotToFrontRightCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(0.25, -0.25, 0.18, Rotation3d()),
)
kRobotToBackLeftCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(-0.25, 0.25, 0.18, Rotation3d()),
)
kRobotToBackRightCameraTransform = Transform3d(
    Pose3d(),
    Pose3d(
        -10.698 * kMetersPerInch,
        -9.707 * kMetersPerInch,
        6.063 * kMetersPerInch,
        Rotation3d(0, 0, math.pi),
    ),
) + Transform3d(
    0.006,
    -0.006,
    0.051,
    Rotation3d(0.0, -28.125 * kRadiansPerDegree, 0.0).rotateBy(
        Rotation3d(0.0, 0.0, (270 + 82.829) * kRadiansPerDegree)
    ),
)

# CANivore
kCANivoreName = "canivore"

# Motors
kFrontLeftDriveMotorId = 10
kFrontLeftSteerMotorId = 11
kFrontRightDriveMotorId = 12
kFrontRightSteerMotorId = 13
kBackLeftDriveMotorId = 14
kBackLeftSteerMotorId = 15
kBackRightDriveMotorId = 16
kBackRightSteerMotorId = 17

kDriveCurrentLimit = (
    CurrentLimitsConfigs()
    .with_stator_current_limit(35)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(35)
    .with_supply_current_limit_enable(True)
)

kDriveAngularVelocityCoeff = 0.01  # while translating and rotating, need a bit extra motion to compensate for moving reference frame

# Pigeon
kPigeonCANId = 44

# Encoders
kFrontLeftSteerEncoderId = 40
kFrontRightSteerEncoderId = 41
kBackLeftSteerEncoderId = 42
kBackRightSteerEncoderId = 43

kCANcoderPulsesPerRevolution = 4096
"""pulses / revolution"""

kCANcoderPulsesPerRadian = kCANcoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kTalonEncoderPulsesPerRevolution = 2048
"""pulses / revolution"""

kTalonEncoderPulsesPerRadian = kTalonEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kDriveEncoderPulsesPerRadian = kDriveEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kDriveEncoderPulsesPerMeter = kDriveEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kWheelEncoderPulsesPerRevolution = kDriveEncoderPulsesPerRevolution * kDriveGearingRatio
"""pulses / revolution"""

kWheelEncoderPulsesPerRadian = kWheelEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kWheelEncoderPulsesPerMeter = kWheelEncoderPulsesPerRadian / kWheelDistancePerRadian
"""pulses / meter"""

kSteerEncoderPulsesPerRevolution = kTalonEncoderPulsesPerRevolution
"""pulses / revolution"""

kSteerEncoderPulsesPerRadian = kSteerEncoderPulsesPerRevolution / kRadiansPerRevolution
"""pulses / radian"""

kSwerveEncoderPulsesPerRevolution = (
    kSteerEncoderPulsesPerRevolution * kSteerGearingRatio
)
"""pulses / revolution"""

kSwerveEncoderPulsesPerRadian = (
    kSwerveEncoderPulsesPerRevolution / kRadiansPerRevolution
)
"""pulses / radian"""

# CTRE
k100MillisecondsPerSecond = 10 / 1  # there are 10 groups of 100 milliseconds per second
"""100 milliseconds / second
   CTRE reports velocities in units of (quantity / 100 milliseconds)
   This factor is used to convert to (quantity / 1 second)
"""

kTalonVelocityPerRPM = (
    kTalonEncoderPulsesPerRevolution / kSecondsPerMinute
) / k100MillisecondsPerSecond
"""(pulses / 100 milliseconds) / RPM"""


kTalonVelocityPerAngularVelocity = kTalonVelocityPerRPM * kRPMPerAngularVelocity
"""(pulses / 100 milliseconds) / (radians / second)"""

kConfigurationTimeoutLimit = int(5 * kMillisecondsPerSecond)
"""milliseconds"""

kDrivePIDSlot = 0
kDrivePGain = 0.001
kDriveIGain = 0.0
kDriveDGain = 0.0
kDriveVGain = 0.01

kSteerPIDSlot = 0
kSteerPGain = 2
kSteerIGain = 0.0
kSteerDGain = 0

kFrontLeftDriveInverted = False
kFrontRightDriveInverted = True
kBackLeftDriveInverted = False
kBackRightDriveInverted = True

kFrontLeftSteerInverted = False
kFrontRightSteerInverted = False
kBackLeftSteerInverted = False
kBackRightSteerInverted = False

"""
To determine encoder offsets (with robot ON and DISABLED):
  1. Rotate all swerve modules so that the wheels:
     * are running in the forwards-backwards direction
     * have the wheel bevel gears facing inwards towards the
       center-line of the robot
  2. Run Phoenix Tuner
  3. Select desired encoder
  4. Go to "Config" tab
  5. Click "Factory Default"
  6. Go to "Self-Test Snapshot" tab
  7. Click "Self-Test Snapshot"
  8. Record value from line: "Absolute Position (unsigned):"
"""
kFrontLeftAbsoluteEncoderOffset = 256.113 / kDegeersPerRevolution
"""rotations"""

kFrontRightAbsoluteEncoderOffset = 125.420 / kDegeersPerRevolution
"""rotations"""

kBackLeftAbsoluteEncoderOffset = 341.719 / kDegeersPerRevolution
"""rotations"""

kBackRightAbsoluteEncoderOffset = 331.260 / kDegeersPerRevolution
"""rotations"""

kRobotPoseArrayKeys = OptionalValueKeys("RobotOdometryPose")

kRobotVisionPoseWeight = 0.00  # 5% vision data

kDriveVelocityKeys = "robotVelocity"
kDriveAccelLimit = 7
kRobotUpdatePeriod = 1 / 50
"""seconds"""
kLimelightUpdatePeriod = 1 / 10
"""seconds"""

# Vision parameters
kTargetAngleRelativeToRobotKeys = OptionalValueKeys("TargetAngleRelativeToRobot")
kTargetDistanceRelativeToRobotKeys = OptionalValueKeys("TargetDistanceRelativeToRobot")
kTargetFacingAngleRelativeToRobotKeys = OptionalValueKeys(
    "TargetFacingAngleRelativeToRobot"
)
kTargetPoseArrayKeys = OptionalValueKeys("TargetPoseArray")
kRobotVisionPoseArrayKeys = OptionalValueKeys("EstimatedRobotPose")

kTargetName = "Target"

kApriltagPositionDict = {  # thanks 6328 for FieldConstants!
    1: Pose3d(
        (kMetersPerInch * 593.68),
        (kMetersPerInch * 9.68),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    2: Pose3d(
        (kMetersPerInch * 637.21),
        (kMetersPerInch * 34.79),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 120 * kRadiansPerDegree),
    ),
    3: Pose3d(
        (kMetersPerInch * 652.73),
        (kMetersPerInch * 196.17),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    4: Pose3d(
        (kMetersPerInch * 652.73),
        (kMetersPerInch * 218.42),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, math.pi),
    ),
    5: Pose3d(
        (kMetersPerInch * 578.77),
        (kMetersPerInch * 323.00),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    6: Pose3d(
        (kMetersPerInch * 72.5),
        (kMetersPerInch * 323.00),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, 270 * kRadiansPerDegree),
    ),
    7: Pose3d(
        (kMetersPerInch * -1.50),
        (kMetersPerInch * 218.42),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
    ),
    8: Pose3d(
        (kMetersPerInch * -1.50),
        (kMetersPerInch * 196.17),
        (kMetersPerInch * 57.13),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
    ),
    9: Pose3d(
        (kMetersPerInch * 14.02),
        (kMetersPerInch * 34.79),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
    ),
    10: Pose3d(
        (kMetersPerInch * 57.54),
        (kMetersPerInch * 9.68),
        (kMetersPerInch * 53.38),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
    ),
    11: Pose3d(
        (kMetersPerInch * 468.69),
        (kMetersPerInch * 146.19),
        (kMetersPerInch * 52.00),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 300),
    ),
    12: Pose3d(
        (kMetersPerInch * 468.69),
        (kMetersPerInch * 177.10),
        (kMetersPerInch * 52.00),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 60),
    ),
    13: Pose3d(
        (kMetersPerInch * 441.74),
        (kMetersPerInch * 161.62),
        (kMetersPerInch * 52.00),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 180),
    ),
    14: Pose3d(
        (kMetersPerInch * 209.48),
        (kMetersPerInch * 161.62),
        (kMetersPerInch * 52.00),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 0),
    ),
    15: Pose3d(
        (kMetersPerInch * 182.73),
        (kMetersPerInch * 177.10),
        (kMetersPerInch * 52.00),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 120),
    ),
    16: Pose3d(
        (kMetersPerInch * 182.73),
        (kMetersPerInch * 146.19),
        (kMetersPerInch * 52.00),
        Rotation3d(0.0, 0.0, kRadiansPerDegree * 240),
    ),
}

# Autonomous
kAutoDriveDistance = -8 * kWheelCircumference
"""meters"""

kAutoFrontwaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoSidewaysDistance = 24 * kMetersPerInch
"""meters"""

kAutoDistanceThreshold = 6 * kMetersPerInch
"""meters"""

kAutoDriveSpeedFactor = 0.5
"""dimensionless"""

kAutoWaitDuration = 1
"""seconds"""

kAutoTargetOffset = Translation2d(2, 0)
"""[meters, meters]"""

kAutoDuration = 15
"""seconds"""

# Target relative drive
kTargetRelativeDriveAnglePGain = 1
kTargetRelativeDriveAngleIGain = 0
kTargetRelativeDriveAngleDGain = 0

kRotationPGain = 0.1
kRotationIGain = 0
kRotationDGain = 0.00

# Drive to Target
kDriveToTargetDistancePGain = 0.5
kDriveToTargetDistanceIGain = 0
kDriveToTargetDistanceDGain = 0

kDriveToTargetAnglePGain = 0.5
kDriveToTargetAngleIGain = 0
kDriveToTargetAngleDGain = 0

kDriveToTargetDistanceTolerance = 10 / kCentimetersPerMeter
"""meters"""

kDriveToTargetLinearVelocityTolerance = 1 / kCentimetersPerMeter / 1
"""meters / second"""

kDriveToTargetAngleTolerance = 5 * kRadiansPerDegree
"""radians"""

kDriveToTargetAngularVelocityTolerance = 5 * kRadiansPerDegree / 1
"""radians / second"""

# Trajectory Following
kTrajectoryPositionPGain = 8
kTrajectoryPositionIGain = 0
kTrajectoryPositionDGain = 0

kTrajectoryAnglePGain = 8
kTrajectoryAngleIGain = 0
kTrajectoryAngleDGain = 0

kPathFollowingConfig = HolonomicPathFollowerConfig(
    PIDConstants(
        kTrajectoryPositionPGain, kTrajectoryPositionIGain, kTrajectoryPositionDGain
    ),
    PIDConstants(kTrajectoryAnglePGain, kTrajectoryAngleIGain, kTrajectoryAngleDGain),
    kMaxForwardLinearVelocity,
    kFrontLeftWheelPosition.norm(),
    ReplanningConfig(),
)

# Operator Interface
kXboxJoystickDeadband = 0.1
"""dimensionless"""

kKeyboardJoystickDeadband = 0.0
"""dimensionless"""

kControllerMappingFilename = "ControlScheme.json"

kChassisRotationXAxisName = "chassisXRotation"
kChassisRotationYAxisName = "chassisYRotation"
kChassisForwardsBackwardsAxisName = "chassisForwardsBackwards"
kChassisSideToSideAxisName = "chassisSideToSide"

kFieldRelativeCoordinateModeControlButtonName = "fieldRelativeCoordinateModeControl"
kResetGyroButtonName = "resetGyro"
kTargetRelativeCoordinateModeControlButtonName = "targetRelativeCoordinateModeControl"
kDriveToTargetControlButtonName = "driveToTargetControl"
kXboxTriggerActivationThreshold = 0.5

kTurboSpeedButtonName = "turboSpeed"
kNormalSpeedMultiplier = 0.80  # half full on normal
kTurboSpeedMultiplier = 0.95  # full speed!!!

# Simulation Parameters
kSimulationRotationalInertia = 0.0002
kSimMotorResistance = 0.002
"""[meters, meters, radians]"""

kSimDefaultRobotLocation = Pose2d(0, 0, 0)
kSimDefaultTargetHeight = 8 * kMetersPerFoot + 8 * kMetersPerInch  # 8ft 8in

kSimRobotPoseArrayKey = "SimRobotPoseArray"
kSimRobotVelocityArrayKey = "SimRobotVelocityArray"

"""meters"""

kMotorBaseKey = "motors"

# waypoint setter constraints
kMaxWaypointTranslationalVelocity = kMaxForwardLinearVelocity
kMaxWaypointTranslationalAcceleration = kMaxWaypointTranslationalVelocity * 3

kPossibleWaypoints = []
kWaypointJoystickVariation = 0.1
"""meters"""

kTargetWaypointPoseKey = "waypoint/target"
kTargetWaypointXControllerKey = "waypoint/x"
kTargetWaypointYControllerKey = "waypoint/y"
kTargetWaypointThetaControllerKey = "waypoint/theta"

# lights
kCANdleID = 2


# Logging
kSwerveActualStatesKey = "swerve/actual"
kSwerveExpectedStatesKey = "swerve/expected"
kConsoleLog = "log"
kPDHCanID = 1
kPDHPublishKey = "powerDistribution"

kJoystickKeyLogPrefix = "DriverStation"
kFieldSimTargetKey = "SimTargets"
kFieldRelativeTargets = "RelTargets"

# Velocity Dynamic Control
kVelocitySetpoint1ControlKey = "controls/velocity/Setpoint 1"
kVelocitySetpoint2ControlKey = "controls/velocity/Setpoint 2"
kVelocityControlGearRatio = "controls/velocity/ratio"

kVelocityControlCANId = 3
kVelocityControlPGain = 0.001
kVelocityControlIGain = 0
kVelocityControlDGain = 0

kVelocityControlMotorType = DCMotor.falcon500()
kVelocityControlkV = 0.01

# Intake Mechanism, need to replace values
kIntakeCANID = 40
kIntakeName = "IntakeMotor"
kIntakePIDSlot = 0
kIntakePGain = 0.12
kIntakeIGain = 0
kIntakeDGain = 0

kPivotCANID = 41
kPivotName = "PivotMotor"
kPivotPGain = 0.7
kPivotIGain = 0
kPivotDGain = 0

kPivotGearRatio = (4 / 1) * (50 / 16) * (84 / 16)

kIntakeInverted = False
kPivotInverted = False

kPivotEncoderID = 1

kIntakeStateKey = "intake/state"
# intake 0 degrees is handoff angle
kIntakeAngleOffset = Rotation2d.fromDegrees(-7.761653)
# all angles are relative to handoff angle
kHandoffAngle = Rotation2d(0)
kFloorPositionAngle = Rotation2d.fromDegrees(225.045433)
kStagingPositionAngle = Rotation2d.fromDegrees(69.939031)
kAmpScoringPositionAngle = kStagingPositionAngle + Rotation2d.fromDegrees(5)

# RPM
kIntakeSpeed = 1000

kPivotAngleKey = "intake/pivotAngle"
kIntakeSpeedKey = "intake/speed"

kAngleMotorRatio = (64 / 16) * (60 / 18)
kShootingMotorRatio = 24 / 36

# change numbers later

kAngleMotorCANId = 50
kAngleMotorName = "ShooterAngleMotor"
kAngleMotorPGain = 0.1
kAngleMotorIGain = 0
kAngleMotorDGain = 0
kAngleMotorInverted = False

kLeftShootingMotorCANId = 51
kLeftShootingMotorName = "LeftShootingMotor"
kLeftShootingMotorPIDSlot = 0
kLeftShootingMotorPGain = 0.01
kLeftShootingMotorIGain = 0
kLeftShootingMotorDGain = 0
kLeftShootingMotorInverted = False
kLeftShootingMotorKv = 550

# Kv taken from motor specifications

kRightShootingMotorCANId = 52
kRightShootingMotorName = "RightShootingMotor"
kRightShootingMotorPIDSlot = 0
kRightShootingMotorPGain = 0.01
kRightShootingMotorIGain = 0
kRightShootingMotorDGain = 0
kRightShootingMotorInverted = False
kRightShootingMotorKv = 550

kAngleMotorMappingFunction = lambda x, y: x * y
kLeftShootingMotorMappingFunction = lambda x, y: x * y
kRightShootingMotorMappingFunction = lambda x, y: x * y
kRobotAngleMappingFunction = lambda x, y: x * y

kShooterAngleEncoderCANId = 1

# radians
kShooterAngleKey = "shooter/angle"
kLeftShootingMotorSpeedKey = "shooter/leftMotorSpeed"
kRightShootingMotorSpeedKey = "shooter/rightMotorSpeed"

kShooterAngleTolerance = Rotation2d.fromDegrees(0.5)
# in RPM
kShooterSpeedTolerance = 100

kShooterAngleOnTargetKey = "shooter/angleOnTarget"
kLeftShootingMotorOnTargetKey = "shooter/leftMotorOnTarget"
kRightShootingMotorOnTargetKey = "shooter/rightMotorOnTarget"
kRobotAngleOnTargetKey = "shooter/robotAngleOnTarget"
kReadyToShoot = "shooter/ready"

# from horizontal
kShooterMaxAngle = Rotation2d.fromDegrees(64.028164)
kShooterMinAngle = Rotation2d.fromDegrees(10.207848)

kShootingMotorCurrentLimit = 40
kAngleMotorCurrentLimit = 40

kShooterManualModeKey = "shooter/manualMode"
kShooterAngleFudgeKey = "shooter/fudge/angle"
kLeftMotorFudgeKey = "shooter/fudge/leftMotor"
kRightMotorFudgeKey = "shooter/fudge/rightMotor"

kShootingMotorFudgeAmount = 50

# radians
kShootingAngleFudgeAmount = 0.01
# Elevator constants, replace values

kElevator1CANID = 60
kElevator1Name = "Elevator1Motor"
kElevator1PGain = 0.12
kElevator1IGain = 0
kElevator1DGain = 0
kElevator1Inverted = False

kElevator2CANID = 60
kElevator2Name = "Elevator2Motor"
kElevator2PGain = 0.12
kElevator2IGain = 0
kElevator2DGain = 0
kElevator2Inverted = True

kMotorPulleyGearRatio = 60 / 18 * 4 / 1

kPulleyGearPitchDiameter = 1.504
"""inches"""

kBottomPositionBeltPosition = 0
kAmpPositionBeltPosition = 19.125
kTopPositionBeltPosition = 26.5
"""inches"""

kBeltPullDownSpeed = 3
"""inches per second"""

kPullDownBandLimit = 0.1
"""Revolutions"""
