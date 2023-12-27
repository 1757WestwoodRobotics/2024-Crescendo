import typing
from commands2 import Command
from wpilib import DriverStation
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d

from subsystems.drivesubsystem import DriveSubsystem
from util.angleoptimize import optimizeAngle

import constants


class AngleAlignDrive(Command):
    def __init__(
        self,
        drive: DriveSubsystem,
        forward: typing.Callable[[], float],
        sideways: typing.Callable[[], float],
    ) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

        self.drive = drive
        self.forward = forward
        self.sideways = sideways
        self.rotationPid = PIDController(
            constants.kRotationPGain, constants.kRotationIGain, constants.kRotationDGain
        )
        self.targetRotation = Rotation2d()
        self.addRequirements(self.drive)
        self.setName(__class__.__name__)

    def initialize(self) -> None:
        currentRotation = self.drive.getRotation()
        self.targetRotation = Rotation2d.fromDegrees(
            round(currentRotation.degrees() / 90) * 90
        )

    def rotation(self) -> float:
        optimizedDirection = optimizeAngle(
            self.drive.getRotation(), self.targetRotation
        ).radians()
        return self.rotationPid.calculate(
            self.drive.getRotation().radians(), optimizedDirection
        )

    def execute(self) -> None:
        self.drive.arcadeDriveWithFactors(
            self.forward(),
            self.sideways(),
            self.rotation(),
            DriveSubsystem.CoordinateMode.FieldRelative,
        )
