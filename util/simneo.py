from enum import Enum, auto
from rev import CANSparkFlex, REVLibError, SparkMaxLimitSwitch
from wpilib import SmartDashboard
from wpilib._wpilib import RobotBase


def revCheckError(name: str, errorCode: REVLibError) -> bool:
    if errorCode is not None and errorCode != REVLibError.kOk:
        print(f"ERROR: {name}: {errorCode}")
        return False
    return True


class NEOBrushless:
    class ControlMode(Enum):
        Position = auto()
        Velocity = auto()
        Percent = auto()

    class NeutralMode(Enum):
        Brake = auto()
        Coast = auto()

    class LimitSwitch(Enum):
        Forwards = auto()
        Backwards = auto()

    # pylint:disable-next=too-many-arguments
    def __init__(
        self,
        canID: int,
        name: str,
        pidSlot: int = 0,
        pGain: float = 1,
        iGain: float = 0,
        dGain: float = 0,
        isInverted: bool = False,
        kV: float = 0,
        enableLimitSwitches: bool = True,
        limitSwitchPolarity: SparkMaxLimitSwitch.Type = SparkMaxLimitSwitch.Type.kNormallyOpen,
    ):
        print(f"Init Spark FLEX with port {canID} with name {name}")
        self.name = name
        self.id = canID
        self.motor = CANSparkFlex(canID, CANSparkFlex.MotorType.kBrushless)
        self.controller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()
        self.forwardSwitch = self.motor.getForwardLimitSwitch(limitSwitchPolarity)
        self.reverseSwitch = self.motor.getReverseLimitSwitch(limitSwitchPolarity)

        self._nettableidentifier = f"motors/{self.name}({self.id})"
        SmartDashboard.putNumber(f"{self._nettableidentifier}/gains/p", pGain)
        SmartDashboard.putNumber(f"{self._nettableidentifier}/gains/i", iGain)
        SmartDashboard.putNumber(f"{self._nettableidentifier}/gains/d", dGain)
        SmartDashboard.putBoolean(f"{self._nettableidentifier}/inverted", isInverted)

        SmartDashboard.putBoolean(f"{self._nettableidentifier}/fwdLimit", False)
        SmartDashboard.putBoolean(f"{self._nettableidentifier}/bckLimit", False)

        if not revCheckError("factoryConfig", self.motor.restoreFactoryDefaults()):
            return
        if not revCheckError("setP", self.controller.setP(pGain, pidSlot)):
            return
        if not revCheckError("setI", self.controller.setI(iGain, pidSlot)):
            return
        if not revCheckError("setD", self.controller.setD(dGain, pidSlot)):
            return

        self.controller.setFF(kV)

        self.forwardSwitch.enableLimitSwitch(enableLimitSwitches)

        self.reverseSwitch.enableLimitSwitch(enableLimitSwitches)

        self.motor.setInverted(isInverted)

    def set(self, controlMode: ControlMode, demand: float, ff: float = 0):
        """input is in rotations or rpm"""
        if controlMode == NEOBrushless.ControlMode.Velocity:
            self.controller.setReference(
                demand, CANSparkFlex.ControlType.kVelocity, arbFeedforward=ff
            )
        elif controlMode == NEOBrushless.ControlMode.Position:
            self.controller.setReference(
                demand, CANSparkFlex.ControlType.kPosition, arbFeedforward=ff
            )
        elif controlMode == NEOBrushless.ControlMode.Percent:
            # self.controller.setReference(demand, CANSparkFlex.ControlType.kDutyCycle)
            self.motor.setVoltage(demand * 12)

    def get(self, controlMode: ControlMode) -> float:
        if controlMode == NEOBrushless.ControlMode.Velocity:
            return self.encoder.getVelocity()
        elif controlMode == NEOBrushless.ControlMode.Position:
            return self.encoder.getPosition()
        elif controlMode == NEOBrushless.ControlMode.Percent:
            return self.motor.get()
        return 0

    def setNeutralOutput(self, output: NeutralMode) -> None:
        self.motor.setIdleMode(
            CANSparkFlex.IdleMode.kBrake
            if output == NEOBrushless.NeutralMode.Brake
            else CANSparkFlex.IdleMode.kCoast
        )

    def enableLimitSwitch(self, switch: LimitSwitch, enable: True):
        if switch == NEOBrushless.LimitSwitch.Forwards:
            self.forwardSwitch.enableLimitSwitch(enable)
        if switch == NEOBrushless.LimitSwitch.Backwards:
            self.reverseSwitch.enableLimitSwitch(enable)

    def neutralOutput(self) -> None:
        self.motor.set(0)

    def getLimitSwitch(self, switch: LimitSwitch) -> bool:
        if RobotBase.isReal():
            if switch == NEOBrushless.LimitSwitch.Forwards:
                return self.forwardSwitch.get()
            if switch == NEOBrushless.LimitSwitch.Backwards:
                return self.reverseSwitch.get()
        else:
            if switch == NEOBrushless.LimitSwitch.Forwards:
                return SmartDashboard.getBoolean(
                    f"{self._nettableidentifier}/fwdLimit", False
                )
            if switch == NEOBrushless.LimitSwitch.Backwards:
                return SmartDashboard.getBoolean(
                    f"{self._nettableidentifier}/bckLimit", False
                )

        return False

    def setSmartCurrentLimit(self, limit: int = 25) -> None:
        self.motor.setSmartCurrentLimit(limit)
        # """25 amps"""

    def getNettableIden(self) -> str:
        return self._nettableidentifier
