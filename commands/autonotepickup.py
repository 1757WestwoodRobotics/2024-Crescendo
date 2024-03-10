from commands2 import Command, SequentialCommandGroup
from wpilib import SmartDashboard

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.elevatorsubsystem import ElevatorSubsystem
from subsystems.visionsubsystem import VisionSubsystem


import constants


class RotateToNote(Command):
    def __init__(self, drive: DriveSubsystem, vision: VisionSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.vision = vision

    def execute(self):
        self.drive.arcadeDriveWithFactors(
            0, 0, self.vision.dRobotAngle, self.drive.CoordinateMode.RobotRelative
        )

    def isFinished(self) -> bool:
        return (
            abs(self.vision.dRobotAngle.radians())
            < constants.kAutoNotePickupAngleTolerance.radians()
        )


class IntakeDriveUntilNote(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        intake: IntakeSubsystem,
        elevator: ElevatorSubsystem,
    ) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive
        self.intake = intake
        self.elevator = elevator

    def execute(self):
        self.drive.arcadeDriveWithFactors(
            0.5, 0, 0, self.drive.CoordinateMode.RobotRelative
        )

    def isFinished(self) -> bool:
        return SmartDashboard.getBoolean(constants.kIntakeHasNoteKey, False)


class AutoNotePickup(SequentialCommandGroup):
    def __init__(
        self,
        drive: DriveSubsystem,
        vision: VisionSubsystem,
        intake: IntakeSubsystem,
        elevator: ElevatorSubsystem,
    ) -> None:
        SequentialCommandGroup.__init__(
            RotateToNote(drive, vision), IntakeDriveUntilNote(drive, intake, elevator)
        )
        self.setName(__class__.__name__)
