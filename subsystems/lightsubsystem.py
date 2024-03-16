from enum import Enum, auto
from commands2 import Subsystem
from phoenix5.led import CANdle, RainbowAnimation, ColorFlowAnimation, StrobeAnimation
from wpilib import RobotState

import constants


class LightSubsystem(Subsystem):
    class StateShoot(Enum):
        #Shooter LED commands, top half
        readyToShoot = auto()
        spinningUp = auto()
        hangingOut = auto()

    class StateIntake(Enum):
        #Intake LED commands, bottom half
        hangingOut= auto()
        intaking = auto()
        centeringNote = auto()
        holdingNote = auto()
        trapTime = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        self.light = CANdle(constants.kCANdleID)

        self.disabledAnimation1 = RainbowAnimation(1, 0.5, 17, ledOffset=8)
        self.cubeAnimation1 = ColorFlowAnimation(
            204, 0, 204, 255, 0.7, 17, ledOffset=8
        )  # purple
        self.cubeAnimation2 = ColorFlowAnimation(
            204, 0, 204, 255, 0.7, 17, ledOffset=8 + 17
        )  # purple
        self.coneAnimation1 = ColorFlowAnimation(
            255, 255, 0, 255, 0.7, 17, ledOffset=8
        )  # yellow
        self.coneAnimation2 = ColorFlowAnimation(
            255, 255, 0, 255, 0.7, 17, ledOffset=8 + 17
        )  # yellow

        self.coneFlangeAnimation1 = StrobeAnimation(255, 255, 0, 255, 0.3, 17, 8)
        self.coneFlangeAnimation2 = StrobeAnimation(255, 255, 0, 255, 0.3, 17, 8 + 17)

        self.estopAnim1 = StrobeAnimation(255, 0, 0, 255, 0.3, 17, 8)

        self.stateshooter = LightSubsystem.StateShoot.hangingOut
        self.stateintake = LightSubsystem.StateIntake.hangingOut

    def periodic(self) -> None:
        if RobotState.isEStopped():
            self.light.animate(self.estopAnim1)
        elif RobotState.isDisabled():
            self.light.animate(self.disabledAnimation1)
        else:
            if self.stateshooter == LightSubsystem.StateShoot.hangingOut:
                self.light.animate(self.disabledAnimation1)
            elif self.state == LightSubsystem.State.Cone:
                self.light.animate(self.coneAnimation1)
                self.light.animate(self.coneAnimation2, 1)
            elif self.state == LightSubsystem.State.Cube:
                self.light.animate(self.cubeAnimation1)
                self.light.animate(self.cubeAnimation2, 1)
            elif self.state == LightSubsystem.State.ConeFlange:
                self.light.animate(self.coneFlangeAnimation1)
                self.light.animate(self.coneFlangeAnimation2, 1)

            if self.stateintake == LightSubsystem.StateIntake.hangingOut:
                pass

    def offLights(self) -> None:
        self.state = LightSubsystem.State.No

    def coneLights(self) -> None:
        self.state = LightSubsystem.State.Cone

    def coneFlangeLights(self) -> None:
        self.state = LightSubsystem.State.ConeFlange

    def cubeLights(self) -> None:
        self.state = LightSubsystem.State.Cube