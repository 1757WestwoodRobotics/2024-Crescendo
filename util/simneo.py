from enum import Enum, auto
from rev import CANSparkFlex, REVLibError, SparkMaxLimitSwitch
from wpilib import SmartDashboard


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

    def __init__(
        self,
        canID: int,
        name: str,
        pidSlot: int = 0,
        pGain: float = 1,
        iGain: float = 0,
        dGain: float = 0,
        isInverted: bool = False,
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

        if not revCheckError("factoryConfig", self.motor.restoreFactoryDefaults()):
            return
        if not revCheckError("setP", self.controller.setP(pGain, pidSlot)):
            return
        if not revCheckError("setI", self.controller.setI(iGain, pidSlot)):
            return
        if not revCheckError("setD", self.controller.setD(dGain, pidSlot)):
            return

        self.forwardSwitch.enableLimitSwitch(enableLimitSwitches)

        self.reverseSwitch.enableLimitSwitch(enableLimitSwitches)

        self.motor.setInverted(isInverted)

    def set(self, controlMode: ControlMode, demand: float):
        """input is in rotations or rpm"""
        if controlMode == NEOBrushless.ControlMode.Velocity:
            self.controller.setReference(demand, CANSparkFlex.ControlType.kVelocity)
        elif controlMode == NEOBrushless.ControlMode.Position:
            self.controller.setReference(demand, CANSparkFlex.ControlType.kPosition)
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

    def neutralOutput(self) -> None:
        self.motor.set(0)

    def getLimitSwitch(self, switch: LimitSwitch) -> bool:
        if switch == NEOBrushless.LimitSwitch.Forwards:
            return self.forwardSwitch.get()
        if switch == NEOBrushless.LimitSwitch.Backwards:
            return self.reverseSwitch.get()
        return False

    def setSmartCurrentLimit(self, limit: int = 25) -> None:
        self.motor.setSmartCurrentLimit(limit)
        # """25 amps"""
