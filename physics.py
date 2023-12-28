#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import functools
import operator
import typing
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState
from phoenix6.unmanaged import feed_enable
from wpilib import RobotController, SmartDashboard
from wpilib.simulation import SimDeviceSim, DCMotorSim
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.system.plant import DCMotor
import wpimath.kinematics
from pyfrc.physics.core import PhysicsInterface
import constants
from subsystems.drivesubsystem import DriveSubsystem
from util.convenientmath import clamp


class SwerveModuleSim:
    def __init__(
        self,
        position: Translation2d,
        wheelMotorType: DCMotor,
        wheelMotorSim: typing.Callable[[], TalonFXSimState],
        driveMotorGearing,
        swerveMotorType: DCMotor,
        swerveMotorSim: typing.Callable[[], TalonFXSimState],
        steerMotorGearing,
        swerveEncoderSim: typing.Callable[[], CANcoderSimState],
    ) -> None:
        self.position = position
        self.wheelMotorSim = wheelMotorSim
        self.wheelMotorType = wheelMotorType
        self.driveMotorGearing = driveMotorGearing
        self.wheelMotorInternalSim = DCMotorSim(
            self.wheelMotorType,
            self.driveMotorGearing,
            constants.kSimulationRotationalInertia,
        )
        self.swerveMotorSim = swerveMotorSim
        self.swerveMotorType = swerveMotorType
        self.steerMotorGearing = steerMotorGearing
        self.steerMotorIntenalSim = DCMotorSim(
            self.swerveMotorType,
            self.steerMotorGearing,
            constants.kSimulationRotationalInertia,
        )
        self.swerveEncoderSim = swerveEncoderSim
        self.encoderPosition = 0

    def __str__(self) -> str:
        return f"pos: x={self.position.X():.2f} y={self.position.Y():.2f}"


class SwerveDriveSim:
    def __init__(self, swerveModuleSims: typing.Tuple[SwerveModuleSim, ...]) -> None:
        self.swerveModuleSims = swerveModuleSims
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            *(module.position for module in swerveModuleSims)
        )
        self.pose = constants.kSimDefaultRobotLocation
        self.outputs = None

    def getPose(self) -> Pose2d:
        return self.pose

    def getHeading(self) -> Rotation2d:
        return self.pose.rotation()

    def update(self, tm_diff: float, robotVoltage: float) -> None:
        deltaT = tm_diff
        states = []
        for module in self.swerveModuleSims:
            module.wheelMotorInternalSim.setInputVoltage(
                module.wheelMotorSim().motor_voltage
            )
            # print(module.wheelMotorSim().motor_voltage)
            module.wheelMotorInternalSim.update(tm_diff)
            wheel_position_rot = (
                module.wheelMotorInternalSim.getAngularPosition()
                / constants.kRadiansPerRevolution
                * module.driveMotorGearing
            )
            wheel_velocity_rps = (
                module.wheelMotorInternalSim.getAngularVelocity()
                / constants.kRadiansPerRevolution
                * module.driveMotorGearing
            )
            module.wheelMotorSim().set_raw_rotor_position(wheel_position_rot)
            module.wheelMotorSim().set_rotor_velocity(wheel_velocity_rps)
            module.wheelMotorSim().set_supply_voltage(
                clamp(
                    robotVoltage
                    - module.wheelMotorSim().supply_current
                    * constants.kSimMotorResistance,
                    0,
                    robotVoltage,
                )
            )

            module.steerMotorIntenalSim.setInputVoltage(
                module.swerveMotorSim().motor_voltage
            )
            module.steerMotorIntenalSim.update(tm_diff)
            swerve_position_rot = (
                module.steerMotorIntenalSim.getAngularPosition()
                / constants.kRadiansPerRevolution
                * module.steerMotorGearing
            )
            swerve_velocity_rps = (
                module.steerMotorIntenalSim.getAngularVelocity()
                / constants.kRadiansPerRevolution
                * module.steerMotorGearing
            )
            module.swerveMotorSim().set_raw_rotor_position(swerve_position_rot)
            module.swerveMotorSim().set_rotor_velocity(swerve_velocity_rps)
            module.swerveMotorSim().set_supply_voltage(
                clamp(
                    robotVoltage
                    - module.swerveMotorSim().supply_current
                    * constants.kSimMotorResistance,
                    0,
                    robotVoltage,
                )
            )
            module.swerveEncoderSim().set_raw_position(
                swerve_position_rot / module.steerMotorGearing
            )
            module.swerveEncoderSim().set_velocity(
                swerve_velocity_rps / module.steerMotorGearing
            )

            wheelLinearVelocity = (
                wheel_velocity_rps
                * constants.kWheelRadius
                * constants.kRadiansPerRevolution
                / constants.kDriveGearingRatio
            )

            state = wpimath.kinematics.SwerveModuleState(
                -wheelLinearVelocity,
                Rotation2d(
                    swerve_position_rot
                    / module.steerMotorGearing
                    * constants.kRadiansPerRevolution
                ),
            )
            states.append(state)

        chassisSpeed = self.kinematics.toChassisSpeeds(states)
        deltaHeading = chassisSpeed.omega * deltaT
        deltaX = chassisSpeed.vx * deltaT
        deltaY = chassisSpeed.vy * deltaT

        deltaTrans = Transform2d(deltaX, deltaY, deltaHeading)

        newPose = self.pose + deltaTrans
        self.pose = newPose


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    # pylint: disable-next=unused-argument
    def __init__(self, physics_controller: PhysicsInterface, robot: "MentorBot"):
        self.physics_controller = physics_controller

        driveSubsystem: DriveSubsystem = robot.container.drive

        frontLeftSim = driveSubsystem.frontLeftModule.getSimulator()
        self.frontLeftModuleSim = SwerveModuleSim(
            constants.kFrontLeftWheelPosition,
            DCMotor.krakenX60(),
            frontLeftSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            frontLeftSim[1],
            constants.kSteerGearingRatio,
            frontLeftSim[2],
        )
        frontRightSim = driveSubsystem.frontRightModule.getSimulator()
        self.frontRightModuleSim = SwerveModuleSim(
            constants.kFrontRightWheelPosition,
            DCMotor.krakenX60(),
            frontRightSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            frontRightSim[1],
            constants.kSteerGearingRatio,
            frontRightSim[2],
        )
        backLeftSim = driveSubsystem.backLeftModule.getSimulator()
        self.backSimLeftModule = SwerveModuleSim(
            constants.kBackLeftWheelPosition,
            DCMotor.krakenX60(),
            backLeftSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            backLeftSim[1],
            constants.kSteerGearingRatio,
            backLeftSim[2],
        )
        backRightSim = driveSubsystem.backRightModule.getSimulator()
        self.backSimRightModule = SwerveModuleSim(
            constants.kBackRightWheelPosition,
            DCMotor.krakenX60(),
            backRightSim[0],
            constants.kDriveGearingRatio,
            DCMotor.falcon500(),
            backRightSim[1],
            constants.kSteerGearingRatio,
            backRightSim[2],
        )

        self.swerveModuleSims = [
            self.frontLeftModuleSim,
            self.frontRightModuleSim,
            self.backSimLeftModule,
            self.backSimRightModule,
        ]

        self.driveSim = SwerveDriveSim(tuple(self.swerveModuleSims))

        self.gyroSim = SimDeviceSim("navX-Sensor[4]")
        self.gyroYaw = self.gyroSim.getDouble("Yaw")
        self.gyroPitch = self.gyroSim.getDouble("Pitch")

        self.sim_initialized = False

        targets = []
        for target in constants.kApriltagPositionDict.values():
            x = target.X()
            y = target.Y()
            z = target.Z()
            rotationQuaternion = target.rotation().getQuaternion()
            w_rot = rotationQuaternion.W()
            x_rot = rotationQuaternion.X()
            y_rot = rotationQuaternion.Y()
            z_rot = rotationQuaternion.Z()

            targets.append(
                [x, y, z, w_rot, x_rot, y_rot, z_rot]
            )  # https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md#cones

        SmartDashboard.putNumberArray(
            constants.kFieldSimTargetKey,
            functools.reduce(
                operator.add, targets, []
            ),  # adds all the values found within targets (converts [[]] to [])
        )

    # pylint: disable-next=unused-argument
    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        feed_enable(1 / 50)

        if not self.sim_initialized:
            self.sim_initialized = True
            # self.physics_controller.field, is not set until simulation_init

        self.gyroYaw.set(-self.driveSim.getHeading().degrees())
        x = float(SmartDashboard.getNumber("therealgyro", -30))
        self.gyroPitch.set(SmartDashboard.getNumber("thegyronumbies", x))

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.driveSim.update(tm_diff, voltage)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)

        # publish the simulated robot pose to nt
        SmartDashboard.putNumberArray(
            constants.kSimRobotPoseArrayKey,
            [simRobotPose.X(), simRobotPose.Y(), simRobotPose.rotation().radians()],
        )

        # publish the simulated target and ball pose to nt
        simTargetObject = self.physics_controller.field.getObject(
            constants.kSimTargetName
        )
        simTargetPose = simTargetObject.getPose()
        SmartDashboard.putNumberArray(
            constants.kSimTargetPoseArrayKey,
            [simTargetPose.X(), simTargetPose.Y(), simTargetPose.rotation().radians()],
        )
