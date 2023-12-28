from functools import reduce
from operator import add

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import PowerDistribution, SmartDashboard, DriverStation

from operatorinterface import OperatorInterface

import constants


class LoggingSubsystem(Subsystem):
    def __init__(self, oi: OperatorInterface) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)
        self.oi = oi
        self.dsTable = NetworkTableInstance.getDefault().getTable(
            constants.kJoystickKeyLogPrefix
        )

    def periodic(self) -> None:
        SmartDashboard.putData(self.pdh)

        for controller in self.oi.controllers.values():
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonCount",
                controller.getButtonCount(),
            )
            encodedButtonValue = reduce(
                add,
                [
                    controller.getRawButton(i) << i - 1
                    for i in range(controller.getButtonCount() + 1, 0, -1)
                ],
                0,
            )
            self.dsTable.putNumber(
                f"Joystick{controller.getPort()}/ButtonValues", encodedButtonValue
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/AxisValues",
                [controller.getRawAxis(i) for i in range(controller.getAxisCount())],
            )
            self.dsTable.putNumberArray(
                f"Joystick{controller.getPort()}/POVs",
                [controller.getPOV(i) for i in range(controller.getPOVCount())],
            )

            self.dsTable.putBoolean("Enabled", DriverStation.isEnabled())
            self.dsTable.putBoolean("auto", DriverStation.isAutonomous())
            alliance = DriverStation.getAlliance()
            allianceNumber = (
                0
                if alliance is None
                else (1 if alliance == DriverStation.Alliance.kRed else 2)
            )
            self.dsTable.putNumber("AllianceStation", allianceNumber)
            station = DriverStation.getLocation()
            self.dsTable.putNumber("location", 0.0 if station is None else station)
