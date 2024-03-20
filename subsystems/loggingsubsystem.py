from functools import reduce
from math import pi
from operator import add

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import PowerDistribution, SmartDashboard, DriverStation

from wpimath.geometry import Pose2d, Transform3d, Rotation3d

from operatorinterface import OperatorInterface

import constants
from util import advantagescopeconvert
from util.convenientmath import map_range, pose3dFrom2d


class LoggingSubsystem(Subsystem):
    def __init__(self, oi: OperatorInterface) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        self.oi = oi
        self.dsTable = NetworkTableInstance.getDefault().getTable(
            constants.kJoystickKeyLogPrefix
        )

    def updateBotPositions(self) -> None:
        botPose = pose3dFrom2d(
            Pose2d(
                *SmartDashboard.getNumberArray(
                    constants.kRobotPoseArrayKeys.valueKey, [0, 0, 0]
                )
            )
        )

        elevatorHeight = SmartDashboard.getNumber(constants.kElevatorPositionKey, 0)
        elevatorPosition = (
            botPose
            + constants.kRobotToElevatorTransform
            + Transform3d(0, 0, elevatorHeight, Rotation3d())
        )

        elevatorPoses = advantagescopeconvert.convertToSendablePoses([elevatorPosition])
        SmartDashboard.putNumberArray(constants.kElevatorPoseArrayKey, elevatorPoses)

        armRotation = -SmartDashboard.getNumber(constants.kPivotAngleKey, 0)
        armRootPosition = elevatorPosition + Transform3d(
            0,
            constants.kRobotToElevatorTransform.Y(),
            0,
            Rotation3d(0, armRotation, 0),
        )
        armEndPosition = armRootPosition + Transform3d(
            constants.kIntakeArmLength,
            0,
            0,
            Rotation3d(
                0,
                # -armRotation,
                map_range(
                    armRotation,
                    0,
                    -constants.kFloorPositionAngle.radians(),
                    pi - 1.022,
                    pi / 2,
                )
                - armRotation,
                0,
            ),
        )

        intakePoses = advantagescopeconvert.convertToSendablePoses(
            [armRootPosition, armEndPosition]
        )
        SmartDashboard.putNumberArray(constants.kIntakePoseKey, intakePoses)

        shooterRotation = SmartDashboard.getNumber(constants.kShooterAngleKey, 0)
        shooterPose = (
            botPose
            + constants.kRobotToShooterTransform
            + Transform3d(0, 0, 0, Rotation3d(0, -shooterRotation, 0))
        )

        shooterPoses = advantagescopeconvert.convertToSendablePoses([shooterPose])
        SmartDashboard.putNumberArray(constants.kShooterPosesKey, shooterPoses)

        climberHeight = SmartDashboard.getNumber(constants.kClimberHeightKey, 0)
        climberPosition = (
            botPose
            + constants.kRobotToClimberTransform
            + Transform3d(0, 0, climberHeight, Rotation3d()),
        )

        climberPose = advantagescopeconvert.convertToSendablePoses(climberPosition)
        SmartDashboard.putNumberArray(constants.kClimberPositionKey, climberPose)

    def periodic(self) -> None:
        SmartDashboard.putData(self.pdh)

        for controller in self.oi.controllers.values():
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonCount",
                controller.getButtonCount(),
            )
            encodedButtonValue = reduce(
                add,
                [
                    controller.getRawButton(i) << i - 1
                    for i in range(controller.getButtonCount() + 1, 0, -1)
                ],
                0,
            )
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonValues", encodedButtonValue
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/AxisValues",
                [controller.getRawAxis(i) for i in range(controller.getAxisCount())],
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/POVs",
                [controller.getPOV(i) for i in range(controller.getPOVCount())],
            )

        self.dsTable.putBoolean("Enabled", DriverStation.isEnabled())
        self.dsTable.putBoolean("auto", DriverStation.isAutonomous())
        alliance = DriverStation.getAlliance()
        allianceNumber = (
            0
            if alliance is None
            else (1 if alliance == DriverStation.Alliance.kRed else 2)
        )
        self.dsTable.putNumber("AllianceStation", allianceNumber)
        station = DriverStation.getLocation()
        self.dsTable.putNumber("location", 0.0 if station is None else station)

        self.updateBotPositions()
