from commands2.command import Command
from wpilib import DataLogManager

from subsystems.dynamicvelocitycontrol import VelocityControl


class VelocitySetpoint(Command):
    def __init__(self, velocity: VelocityControl, control: VelocityControl.ControlState) -> None:
        Command.__init__(self)
        self.setName(__class__.__name__)
        self.vel = velocity
        self.control = control

        self.addRequirements(self.vel)

    def initialize(self) -> None:
        DataLogManager.log(f"Command: {self.getName()}")

    def execute(self):
        self.vel.setState(self.control)

    def end(self, _interrupted: bool) -> None:
        DataLogManager.log("... DONE")
