from commands2 import Command
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.filter import Debouncer
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from subsystems.visionsubsystem import VisionSubsystem


import constants


class AutoNotePickup(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        vision: VisionSubsystem,
        intake: IntakeSubsystem,
        elevator: ElevatorSubsystem,
    ) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.vision = vision
        self.intake = intake
        self.elevator = elevator
        self.addRequirements(self.drive, self.vision, self.intake, self.elevator)
        self.rotationPID = PIDController(
            constants.kRotationPGain, constants.kRotationIGain, constants.kRotationDGain
        )
        self.drivePID = PIDController(
            constants.kAutoNotePickupPGain,
            constants.kAutoNotePickupIGain,
            constants.kAutoNotePickupDGain,
        )
        self.debouncer = Debouncer(
            constants.kNoteCameraDebounceTime, Debouncer.DebounceType.kFalling
        )

    def execute(self):
        self.intake.setIntaking()
        self.elevator.setBottomPosition()

        angleOutput = self.rotationPID.calculate(self.vision.dRobotAngle)
        driveOutput = self.drivePID.calculate(
            (
                self.drive.getRobotRelativeSpeeds().vx ** 2
                + self.drive.getRobotRelativeSpeeds().vy ** 2
            )
            ** (1 / 2)
            / constants.kMaxWheelLinearVelocity,
            constants.kMaxAutoNotePickupSpeed,
        )

        if (
            abs(self.vision.dRobotAngle.radians())
            < constants.kAutoNotePickupAngleTolerance.radians()
        ):
            self.drive.arcadeDriveWithFactors(
                driveOutput, 0, angleOutput, self.drive.CoordinateMode.RobotRelative
            )
        else:
            self.drive.arcadeDriveWithFactors(
                0, 0, angleOutput, self.drive.CoordinateMode.RobotRelative
            )

    def isFinished(self) -> bool:
        return SmartDashboard.getBoolean(
            constants.kIntakeHasNoteKey, False
        ) or self.debouncer.calculate(
            SmartDashboard.getBoolean(constants.kNoteInViewKey, False)
        )
