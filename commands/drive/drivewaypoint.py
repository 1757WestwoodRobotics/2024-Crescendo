from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from subsystems.drivesubsystem import DriveSubsystem

import constants


class DriveWaypoint(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive

        self.command = Command()

        self.running = False
        self.addRequirements(self.drive)

    def initialize(self) -> None:
        self.running = True
        self.command = AutoBuilder.pathfindToPose(
            self.drive.getClosestWaypoint(), constants.kPathfindingConstraints
        )
        self.command.initialize()

    def execute(self) -> None:
        self.command.execute()
        if self.command.isFinished():
            self.command.end(False)
            self.running = False

    def isFinished(self) -> bool:
        return False
