from enum import Enum, auto
from commands2 import Subsystem
from phoenix5.led import CANdle, RainbowAnimation, StrobeAnimation, SingleFadeAnimation
from wpilib import RobotState

import constants
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.shootersubsystem import ShooterSubsystem

# lights are 8, bottom to top on the back
# then 5, bottom to top on the front


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

    def __init__(self, intake: IntakeSubsystem, shooter: ShooterSubsystem) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)
        self.light = CANdle(constants.kCANdleID, constants.kCANivoreName)

        self.intake = intake
        self.shooter = shooter

        self.disabledAnimation1 = RainbowAnimation(1, 0.5, 8, ledOffset=8)
        self.disabledAnimation2 = RainbowAnimation(1, 0.5, 5, ledOffset=8 + 8)
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

        self.estopAnim1 = StrobeAnimation(255, 0, 0, 255, 0.3, 8 + 5, 8)

        self.stateshooter = LightSubsystem.StateShoot.hangingOut
        self.stateintake = LightSubsystem.StateIntake.hangingOut

        # animation maps: 
        # 0: bottom back
        # 1: top back
        # 2: bottom front
        # 3: top front

    def periodic(self) -> None:
        self.updateState()

        if RobotState.isEStopped():
            self.light.animate(self.estopAnim1)
        elif RobotState.isDisabled():
            self.light.animate(self.disabledAnimation1)
            self.light.animate(self.disabledAnimation2, 1)
        else:
            if self.stateshooter == LightSubsystem.StateShoot.hangingOut:
                self.light.animate(self.chillingAnimation1s, 1)
                self.light.animate(self.chillingAnimation2s, 3)
            elif self.stateshooter == LightSubsystem.StateShoot.readyToShoot:
                self.light.animate(self.shooterReady1, 1)
                self.light.animate(self.shooterReady2, 3)
            elif self.stateshooter == LightSubsystem.StateShoot.spinningUp:
                self.light.animate(self.shooterSpinningUp1, 1)
                self.light.animate(self.shooterSpinningUp2, 3)

            if self.stateintake == LightSubsystem.StateIntake.hangingOut:
                self.light.animate(self.chillingAnimation1i, 0)
                self.light.animate(self.chillingAnimation2i, 2)
            elif self.stateintake == LightSubsystem.StateIntake.intaking:
                self.light.animate(self.intakeRunning1, 0)
                self.light.animate(self.intakeRunning2, 2)
            elif self.stateintake == LightSubsystem.StateIntake.centeringNote:
                self.light.animate(self.intakeCentering1, 0)
                self.light.animate(self.intakeCentering2, 2)
            elif self.stateintake == LightSubsystem.StateIntake.holdingNote:
                self.light.animate(self.intakeHolding1, 0)
                self.light.animate(self.intakeHolding2, 2)

    def updateState(self) -> None:
        self.stateintake = intakeStateMapping[self.intake.state]
        if self.shooter.readyToShoot():
            self.stateshooter = LightSubsystem.StateShoot.readyToShoot
        elif self.shooter.targetAngle == self.shooter.shooterInitPosition:
            self.stateshooter = LightSubsystem.StateShoot.hangingOut
        else:
            self.stateshooter = LightSubsystem.StateShoot.spinningUp


intakeStateMapping = {
    IntakeSubsystem.IntakeState.Intaking: LightSubsystem.StateIntake.intaking,
    IntakeSubsystem.IntakeState.Holding: LightSubsystem.StateIntake.holdingNote,
    IntakeSubsystem.IntakeState.Staging: LightSubsystem.StateIntake.centeringNote,
    IntakeSubsystem.IntakeState.Amp: LightSubsystem.StateIntake.holdingNote,
    IntakeSubsystem.IntakeState.Feeding: LightSubsystem.StateIntake.holdingNote,
    IntakeSubsystem.IntakeState.Trap: LightSubsystem.StateIntake.trapping,
    IntakeSubsystem.IntakeState.Ejecting: LightSubsystem.StateIntake.holdingNote,
}

