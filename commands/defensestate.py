from commands2.command import Command
from wpilib import DataLogManager
from wpimath.geometry import Rotation2d
from subsystems.drivesubsystem import DriveSubsystem, SwerveModule


class DefenseState(Command):
    def __init__(self, drive: DriveSubsystem) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.drive = drive

        self.addRequirements(self.drive)

    def initialize(self) -> None:
        DataLogManager.log(f"Command: {self.getName()}")

    def execute(self) -> None:
        def setModuleTo(module: SwerveModule, angle: Rotation2d):
            module.setWheelLinearVelocityTarget(0)
            module.setSwerveAngleTarget(module.optimizedAngle(angle))

        setModuleTo(self.drive.frontLeftModule, Rotation2d.fromDegrees(45))
        setModuleTo(self.drive.frontRightModule, Rotation2d.fromDegrees(-45))
        setModuleTo(self.drive.backLeftModule, Rotation2d.fromDegrees(135))
        setModuleTo(self.drive.backRightModule, Rotation2d.fromDegrees(-135))

    def end(self, _interrupted: bool) -> None:
        DataLogManager.log("... DONE")
