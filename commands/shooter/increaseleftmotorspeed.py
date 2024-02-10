from commands2 import Command
from wpilib import SmartDashboard

import constants


class IncreaseLeftMotorSpeed(Command):
    def __init__(self) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)

    def initialize(self) -> None:
        print(f"Command: {self.getName()}")

    def execute(self) -> None:
        SmartDashboard.putNumber(
            constants.kLeftMotorFudgeKey,
            SmartDashboard.getNumber(constants.kLeftMotorFudgeKey, 0)
            + constants.kShootingMotorFudgeAmount,
        )

    def end(self, _interrupted: bool) -> None:
        print("... DONE")

    def isFinished(self) -> bool:
        return True
