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
from wpilib.simulation import DCMotorSim
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d, Pose3d
from wpimath.system.plant import DCMotor
import wpimath.kinematics
from pyfrc.physics.core import PhysicsInterface
import constants
from robot import MentorBot
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from util.advantagescopeconvert import convertToSendablePoses
from util.convenientmath import clamp, pointInCircle
from util.motorsimulator import MotorSimulator


class SwerveModuleSim:
    # pylint:disable-next=too-many-arguments
    def __init__(
        self,
        position: Translation2d,
        wheelMotorType: DCMotor,
        wheelMotorSim: typing.Callable[[], TalonFXSimState],
        driveMotorGearing: float,
        swerveMotorType: DCMotor,
        swerveMotorSim: typing.Callable[[], TalonFXSimState],
        steerMotorGearing: float,
        swerveEncoderSim: typing.Callable[[], CANcoderSimState],
        encoderOffset: float,
        inverted: bool,
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
        self.encoderOffset = encoderOffset + 0.25

        self.multiplier = -1 if inverted else 1

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
                -swerve_position_rot / module.steerMotorGearing + module.encoderOffset
            )
            module.swerveEncoderSim().set_velocity(
                -swerve_velocity_rps / module.steerMotorGearing
            )

            wheelLinearVelocity = (
                wheel_velocity_rps
                * module.multiplier
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

        SmartDashboard.putNumberArray(
            constants.kSimRobotVelocityArrayKey,
            [chassisSpeed.vx, chassisSpeed.vy, chassisSpeed.omega],
        )

        deltaTrans = Transform2d(deltaX, -deltaY, deltaHeading)

        newPose = self.pose + deltaTrans
        self.pose = newPose


class NoteSim:
    def __init__(self) -> None:
        self.midlineNotes = constants.kNotesStartingMidline
        self.blueNotes = constants.kNotesStartingBlueWing
        self.redNotes = constants.kNotesStartingRedWing

        self.loadingNotes = [
            constants.kNoteLoadingStationPositionBlue,
            constants.kNoteLoadingStationPositionRed,
        ]

    def canPickup(self, note: Pose3d, botPose) -> bool:
        if pointInCircle(botPose.translation(), note.toPose2d().translation(), 0.5):
            return True
        return False

    def update(self, _tm_diff, bot: MentorBot):
        SmartDashboard.putNumberArray(
            constants.kSimNotePositionsKey,
            convertToSendablePoses(
                [
                    *self.midlineNotes,
                    *self.blueNotes,
                    *self.redNotes,
                    *self.loadingNotes,
                ]
            ),
        )

        # check whether intaking, update sensors according to position on field

        intaking = bot.container.intake.state == IntakeSubsystem.IntakeState.Intaking

        botPose = Pose2d(
            *SmartDashboard.getNumberArray(constants.kSimRobotPoseArrayKey, [0, 0, 0])
        )

        hasNote = SmartDashboard.getBoolean(constants.kIntakeHasNoteKey, False)

        if intaking:
            notestate = hasNote
            for stationObject in self.loadingNotes:
                if self.canPickup(stationObject, botPose):
                    notestate = True

            for blueWingNote in self.blueNotes:
                # remove the note from the field
                if self.canPickup(blueWingNote, botPose):
                    notestate = True
                    self.blueNotes.remove(blueWingNote)

            for redWingNote in self.redNotes:
                # remove the note from the field
                if self.canPickup(redWingNote, botPose):
                    notestate = True
                    self.redNotes.remove(redWingNote)

            for midlineNote in self.midlineNotes:
                # remove the note from the field
                if self.canPickup(midlineNote, botPose):
                    notestate = True
                    self.midlineNotes.remove(midlineNote)

            SmartDashboard.putBoolean(
                f"{bot.container.intake.intakeMotor.getNettableIden()}/fwdLimit",
                notestate,
            )

        # shooting a note clears the note
        feeding = bot.container.intake.state == IntakeSubsystem.IntakeState.Feeding

        if feeding:
            if hasNote:
                pass  # Logic for calculating a shot

            SmartDashboard.putBoolean(
                f"{bot.container.intake.intakeMotor.getNettableIden()}/fwdLimit",
                False,
            )


class PhysicsEngine:
    """
    Simulates a drivetrain
    """

    # pylint: disable-next=unused-argument
    def __init__(self, physics_controller: PhysicsInterface, robot: MentorBot):
        self.physics_controller = physics_controller
        self.bot = robot

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
            constants.kFrontLeftAbsoluteEncoderOffset,
            constants.kFrontLeftDriveInverted,
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
            constants.kFrontRightAbsoluteEncoderOffset,
            constants.kFrontRightDriveInverted,
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
            constants.kBackLeftAbsoluteEncoderOffset,
            constants.kBackLeftDriveInverted,
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
            constants.kBackRightAbsoluteEncoderOffset,
            constants.kBackRightDriveInverted,
        )

        self.swerveModuleSims = [
            self.frontLeftModuleSim,
            self.frontRightModuleSim,
            self.backSimLeftModule,
            self.backSimRightModule,
        ]

        self.driveSim = SwerveDriveSim(tuple(self.swerveModuleSims))
        self.noteSim = NoteSim()

        self.gyroSim = driveSubsystem.gyro.sim_state

        self.sim_initialized = False

        self.motorsim = MotorSimulator()
        self.motorsim.addFalcon(
            robot.container.shooter.angleMotor,
            1,
            constants.kSimulationRotationalInertia,
        )

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

        self.gyroSim.set_raw_yaw(self.driveSim.getHeading().degrees())

        # Simulate the drivetrain
        voltage = RobotController.getInputVoltage()

        self.motorsim.update(tm_diff, voltage)
        self.driveSim.update(tm_diff, voltage)
        self.noteSim.update(tm_diff, self.bot)

        simRobotPose = self.driveSim.getPose()
        self.physics_controller.field.setRobotPose(simRobotPose)

        # publish the simulated robot pose to nt
        SmartDashboard.putNumberArray(
            constants.kSimRobotPoseArrayKey,
            [simRobotPose.X(), simRobotPose.Y(), simRobotPose.rotation().radians()],
        )
