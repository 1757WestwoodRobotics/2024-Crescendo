from math import copysign
import typing
import json

from os import path
from wpilib import Joystick, DataLogManager, Preferences

import constants
from util.convenientmath import map_range, number

AnalogInput = typing.Callable[[], float]


def Deadband(inputFn: AnalogInput, deadband: float) -> AnalogInput:
    def withDeadband() -> float:
        value = inputFn()
        if abs(value) <= deadband:
            return 0
        else:
            return value

    return withDeadband


def Invert(inputFn: AnalogInput) -> AnalogInput:
    def invert() -> float:
        return -1 * inputFn()

    return invert


def SignSquare(inputFn: AnalogInput) -> AnalogInput:
    def square() -> float:
        val = inputFn()
        return copysign(val * val, val)

    return square


def MapRange(
    inputFn: AnalogInput,
    inputMin: number,
    inputMax: number,
    outputMin: number,
    outputMax: number,
) -> AnalogInput:
    return lambda: map_range(inputFn(), inputMin, inputMax, outputMin, outputMax)


def Multiply(a: AnalogInput, b: AnalogInput) -> AnalogInput:
    return lambda: a() * b()


class HolonomicInput:
    def __init__(
        self,
        forwardsBackwards: AnalogInput,
        sideToSide: AnalogInput,
        rotationX: AnalogInput,
        rotationY: AnalogInput,
    ) -> None:
        self.forwardsBackwards = forwardsBackwards
        self.sideToSide = sideToSide
        self.rotationX = rotationX
        self.rotationY = rotationY


class Control2D:
    def __init__(self, forwardsBackwards: AnalogInput, sideToSide: AnalogInput) -> None:
        self.forwardsBackwards = forwardsBackwards
        self.sideToSide = sideToSide


# pylint:disable-next=too-many-instance-attributes
class OperatorInterface:
    """
    The controls that the operator(s)/driver(s) interact with
    """

    # pylint:disable-next=too-many-instance-attributes
    def __init__(self) -> None:
        with open(
            path.join(
                path.dirname(path.realpath(__file__)),
                constants.kControllerMappingFilename,
            ),
            "r",
            encoding="utf-8",
        ) as file:
            controlScheme = json.load(file)

        controllerNumbers = set(
            i[0] for i in controlScheme.values()
        )  # set ensures no duplicates
        DataLogManager.log(f"Looking for controllers: {controllerNumbers} ...")

        self.controllers = {}

        self.prefs = Preferences

        for control in controlScheme:
            binding = controlScheme[control]
            self.prefs.setInt(f"OI/{control}/controller", binding[0])
            if "Button" in binding[1].keys():
                self.prefs.setInt(f"OI/{control}/button", binding[1]["Button"])
            elif "Axis" in binding[1].keys():
                self.prefs.setInt(f"OI/{control}/axis", binding[1]["Axis"])
            elif "POV" in binding[1].keys():
                self.prefs.setInt(f"OI/{control}/angle", binding[1]["POV"][1])
                self.prefs.setInt(f"OI/{control}/pov", binding[1]["POV"][0])

        for num in controllerNumbers:
            controller = Joystick(num)
            DataLogManager.log(
                f"Found Controller {num}:{controller.getName()}\n\tAxis: {controller.getAxisCount()}\n\tButtons: {controller.getButtonCount()}\n\tPoV Hats: {controller.getPOVCount()}"
            )
            self.controllers[num] = controller

        def getButtonBindingOfName(
            name: str,
        ) -> typing.Callable[[], typing.Tuple[Joystick, int]]:
            return lambda: (
                self.controllers[self.prefs.getInt(f"OI/{name}/controller")],
                self.prefs.getInt(f"OI/{name}/button"),
            )

        def getAxisBindingOfName(name: str) -> AnalogInput:
            return lambda: self.controllers[
                self.prefs.getInt(f"OI/{name}/controller")
            ].getRawAxis(self.prefs.getInt(f"OI/{name}/axis"))

        # pylint: disable-next=unused-variable
        def getPOVBindingOfName(name: str) -> typing.Tuple[Joystick, int, int]:
            binding = controlScheme[name]
            return (
                self.controllers[binding[0]],
                binding[1]["POV"][1],
                binding[1]["POV"][0],
            )

        self.fieldRelativeCoordinateModeControl = getButtonBindingOfName(
            constants.kFieldRelativeCoordinateModeControlButtonName
        )
        self.resetGyro = getButtonBindingOfName(constants.kResetGyroButtonName)
        self.defenseStateControl = getButtonBindingOfName("defenseStateControl")

        self.alignClosestWaypoint = getButtonBindingOfName("alignClosestWaypoint")

        self.offVelocity = getButtonBindingOfName("offVelocity")
        self.velocitySetpoint1 = getButtonBindingOfName("setpoint1velocity")
        self.velocitySetpoint2 = getButtonBindingOfName("setpoint2velocity")

        self.chassisControls = HolonomicInput(
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(
                            constants.kChassisForwardsBackwardsAxisName
                        ),
                        constants.kXboxJoystickDeadband,
                    ),
                )
            ),
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(constants.kChassisSideToSideAxisName),
                        constants.kXboxJoystickDeadband,
                    ),
                )
            ),
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(constants.kChassisRotationXAxisName),
                        constants.kXboxJoystickDeadband,
                    ),
                )
            ),
            SignSquare(
                Invert(
                    Deadband(
                        getAxisBindingOfName(constants.kChassisRotationYAxisName),
                        constants.kXboxJoystickDeadband,
                    )
                )
            ),
        )
        # intake subsystem related commands
        self.floorIntake = getButtonBindingOfName("floorIntake")
        self.trapScore = getButtonBindingOfName("trapScore")
        self.trapPrep = getButtonBindingOfName("trapPrep")
        self.hold = getButtonBindingOfName("hold")
        self.feedScore = getButtonBindingOfName("feed")

        # shooter prep
        self.prepShotDynamic = getButtonBindingOfName("prepShotDynamic")
        self.prepShotSubwoofer = getButtonBindingOfName("prepShotSubwoofer")
        self.prepShotPodium = getButtonBindingOfName("prepShotPodium")

        # elevator related
        self.elevatorClimb = getButtonBindingOfName("elevatorClimbUp")
        self.elevatorClimbSlowDown = getButtonBindingOfName("elevatorClimbSlowDown")

        # resets
        self.resetIntake = getButtonBindingOfName("resetIntake")
        self.resetElevator = getButtonBindingOfName("resetElevator")

        # overrides / offsets

        self.intakeCWJog = getButtonBindingOfName("intakePivotCWJog")
        self.intakeCCWJog = getButtonBindingOfName("intakePivotCCWJog")

        self.shooterPivotUp = getButtonBindingOfName("shooterPivotUpJog")
        self.shooterPivotDown = getButtonBindingOfName("shooterPivotDownJog")

        self.shooterRightSpeedUp = getButtonBindingOfName("shooterRightSpeedIncrease")
        self.shooterRightSpeedDown = getButtonBindingOfName("shooterRightSpeedDecrease")
        self.shooterLeftSpeedUp = getButtonBindingOfName("shooterLeftSpeedIncrease")
        self.shooterLeftSpeedDown = getButtonBindingOfName("shooterLeftSpeedDecrease")

        self.elevatorJogUp = getButtonBindingOfName("elevatorJogUp")
        self.elevatorJogDown = getButtonBindingOfName("elevatorJogDown")

        self.simshoot = getButtonBindingOfName("simshoot")
