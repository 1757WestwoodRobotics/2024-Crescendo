import os
from commands2.functionalcommand import FunctionalCommand
import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.button
from commands.velocitysetpoint import VelocitySetpoint
from pathplannerlib.auto import PathPlannerAuto, NamedCommands

import constants

from commands.resetdrive import ResetDrive
from commands.drivedistance import DriveDistance
from commands.drive.robotrelativedrive import RobotRelativeDrive
from commands.drive.fieldrelativedrive import FieldRelativeDrive
from commands.drive.anglealign import AngleAlignDrive
from commands.defensestate import DefenseState

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.dynamicvelocitycontrol import VelocityControl
from subsystems.loggingsubsystem import LoggingSubsystem
from subsystems.visionsubsystem import VisionSubsystem

from operatorinterface import OperatorInterface
from util.helpfultriggerwrappers import ModifiableJoystickButton


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The operator interface (driver controls)
        self.operatorInterface = OperatorInterface()

        # The robot's subsystems
        self.drive = DriveSubsystem()
        self.vision = VisionSubsystem(self.drive)
        self.log = LoggingSubsystem(self.operatorInterface)

        # Robot demo subsystems
        self.velocity = VelocityControl()

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = commands2.SequentialCommandGroup(
            ResetDrive(self.drive),
            DriveDistance(
                -4 * constants.kWheelCircumference,
                0.2,
                DriveDistance.Axis.X,
                self.drive,
            ),
        )
        self.nothingAuto = commands2.WaitCommand(constants.kAutoDuration)

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        NamedCommands.registerCommand(
            "log",
            FunctionalCommand(
                (lambda: None),
                (lambda: print("hello")),
                (lambda _: None),
                (lambda: True),
            ),
        )

        pathsPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for file in os.listdir(pathsPath):
            relevantName = file.split(".")[0]
            auton = PathPlannerAuto(relevantName)
            wpilib.SmartDashboard.putData(f"autos/{relevantName}", auton)
            self.chooser.addOption(relevantName, auton)

        self.chooser.addOption("Do Nothing Auto", self.nothingAuto)
        self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        self.drive.setDefaultCommand(
            FieldRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kTurboSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kTurboSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
            )
        )
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        ModifiableJoystickButton(self.operatorInterface.turboSpeed).whileTrue(
            FieldRelativeDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kNormalSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kNormalSpeedMultiplier,
                self.operatorInterface.chassisControls.rotationX,
            )
        )

        ModifiableJoystickButton(
            self.operatorInterface.fieldRelativeCoordinateModeControl
        ).toggleOnTrue(
            RobotRelativeDrive(
                self.drive,
                self.operatorInterface.chassisControls.forwardsBackwards,
                self.operatorInterface.chassisControls.sideToSide,
                self.operatorInterface.chassisControls.rotationX,
            )
        )

        ModifiableJoystickButton(self.operatorInterface.alignClosestWaypoint).whileTrue(
            AngleAlignDrive(
                self.drive,
                lambda: self.operatorInterface.chassisControls.forwardsBackwards()
                * constants.kNormalSpeedMultiplier,
                lambda: self.operatorInterface.chassisControls.sideToSide()
                * constants.kNormalSpeedMultiplier,
            )
        )

        ModifiableJoystickButton(self.operatorInterface.resetGyro).onTrue(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        ModifiableJoystickButton(self.operatorInterface.defenseStateControl).whileTrue(
            DefenseState(self.drive)
        )

        ModifiableJoystickButton(self.operatorInterface.offVelocity).onTrue(
            VelocitySetpoint(self.velocity, VelocityControl.ControlState.Off)
        )
        ModifiableJoystickButton(self.operatorInterface.velocitySetpoint1).onTrue(
            VelocitySetpoint(self.velocity, VelocityControl.ControlState.Setpoint1)
        )
        ModifiableJoystickButton(self.operatorInterface.velocitySetpoint2).onTrue(
            VelocitySetpoint(self.velocity, VelocityControl.ControlState.Setpoint2)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
