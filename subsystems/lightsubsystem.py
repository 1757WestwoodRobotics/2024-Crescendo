from enum import Enum, auto
from commands2 import Subsystem
from phoenix5.led import CANdle, RainbowAnimation, StrobeAnimation, SingleFadeAnimation
from wpilib import RobotState

import constants


class LightSubsystem(Subsystem):
    class StateShoot(Enum):
        #Shooter LED commands, top half
        readyToShoot = auto()
        spinningUp = auto()
        hangingOut = auto()
        climbing = auto()
        trapping = auto()

    class StateIntake(Enum):
        #Intake LED commands, bottom half
        hangingOut= auto()
        intaking = auto()
        centeringNote = auto()
        holdingNote = auto()
        climbing = auto()
        trapping = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        self.light = CANdle(constants.kCANdleID)

        self.disabledAnimation1 = RainbowAnimation(1, 0.5, 17, ledOffset=8)
        self.chillingAnimation1i = SingleFadeAnimation(
            3, 219, 252, 255, 0.7, 4, ledOffset=8
        )  # light blue
        self.chillingAnimation1s = SingleFadeAnimation(
            3, 219, 252, 255, 0.7, 4, ledOffset=12
        )  # light blue
        self.chillingAnimation2i = SingleFadeAnimation(
            3, 219, 252, 255, 0.7, 3, ledOffset=16
        )  # light blue
        self.chillingAnimation2s = SingleFadeAnimation(
            3, 219, 252, 255, 0.7, 2, ledOffset=19
        )  # light blue
        self.shooterSpinningUp1 = SingleFadeAnimation(252, 252, 3, 255, 0.7, 4, ledOffset=12) #Yellowish
        self.shooterSpinningUp2 = SingleFadeAnimation(252, 252, 3, 255, 0.7, 4, ledOffset=19) #Yellowish
        self.shooterReady1 = StrobeAnimation(3, 252, 3, 255, 0.3, 4, ledOffset=12) #Green flashing
        self.shooterReady2 = StrobeAnimation(3, 252, 3, 255, 0.3, 2, ledOffset=19) #Green flashing

        self.intakeRunning1 = StrobeAnimation(252, 252, 3, 255, 0.3, 4, ledOffset=8) #Yellow
        self.intakeRunning2 = StrobeAnimation(252, 252, 3, 255, 0.3, 3, ledOffset=16) #Yellow
        self.intakeCentering1 = StrobeAnimation(252, 3, 3, 255, 0.3, 4, ledOffset=8) #Red
        self.intakeCentering2 = StrobeAnimation(252, 3, 3, 255, 0.3, 3, ledOffset=16) #Red
        self.intakeHolding1 = SingleFadeAnimation(252, 140, 3, 255, 0.7, 4, ledOffset=8)
        self.intakeHolding2 = SingleFadeAnimation(252, 140, 3, 255, 0.7, 3, ledOffset=16)

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