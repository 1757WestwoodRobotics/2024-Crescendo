from enum import Enum, auto
from functools import partial

from typing import Tuple
import typing

import constants
from util import convenientmath
from util.simcoder import CTREEncoder
from util.angleoptimize import optimizeAngle
from util.simtalon import Talon

from subsystems.visionsubsystem import VisionSubsystem
from subsystems.dynamicvelocitycontrol import VelocityControl

from pathplannerlib.auto import AutoBuilder

from commands2 import Subsystem
from pheonix6.configs.pigeon2 configs import Pigeon2Configuration
from pheonix6.hardware.pigeon2 import Pigeon2
from pheonix6.sim.cancoder_sim_state import CANcoderSimState
frompheonix6.sim.talon_fx_sim_state import TalonFXSimState

from wpilib import(
    RobotBase,
    SmartDashboard,
    Timer,
    DataLogManager,
    DriverStation,
    RobotOdometryPose
)

from wpimath.gemoetry import Pose2d, Rotation2d, Translation2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import ChassisSpeeds

# elevator has 4 states: bottom, climbing to amp, amp, trap
# 1:11 ratio for 2 parallel gears
# 0.01 for voltage of PID, 0 and 0 for other inputs


class SetElevatorParameters:
    def __init__(self,elevateMotorID,deelevateMotorID,encoderID,PIDVoltage,elevationOffset,elevatorState)
        self.elevateMotorID = elevateMotorID
        self.deelevateMotorID: deelevateMotorID
        self.encoderID =  encoderID
        self.PIDVoltage = PIDVoltage
        self.elevationOffset = elevationOffset
        self.elevatorState = elevatorState


class ElevatorSubsystem(Subsystem):
    class SetElevatorStates(Enum):
        Bottom = auto()
        ClimbingToAmp = auto()
        Amp = auto()
        Trap = auto()
    
    def __init__(self,vision: VisionSubsystem) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        SmartDashboard.putBoolean(constants.kRobotPoseArrayKeys.validKey,False)

        self.vision = vision

        self.kinematics = (
            constants.kTrajectoryPositionPGain,
            constants.kTrajectoryPositionIGain,
            constants.kTrajectoryPositionDGain
        )

        self.vision_estimate = Pose2d()

        AutoBuilder.configureHolonomic(
            constants.kPathFollowingConfig,
            self.get_pose,
            self.get_robot_relative_speeds,
            partial(
                self.drivesubsystem.arcadeDriveWithSpeeds, #temporary, I didn't want to write in my own
                self.drivesubsystem.arcadeDriveWithFactors, #temporary, I didn't want to write in my own
                setElevatorStates=ElevatorSubsystem.SetElevatorStates.Bottom
            )
        )

        def get_robot_relative_speeds(self):
            return self.kinematics.toChassisSpeeds()
        
        def get_pose(self):
            movement = self.estimator.getEstimatedPosition().movement()
            return Pose2d(movement)
        
        def PID_control(self):
            
