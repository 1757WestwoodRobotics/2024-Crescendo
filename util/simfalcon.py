from enum import Enum, auto
from phoenix6.configs.config_groups import CurrentLimitsConfigs, MotorOutputConfigs
from phoenix6.signals.spn_enums import (
    ForwardLimitValue,
    InvertedValue,
    NeutralModeValue,
    ReverseLimitValue,
)
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState

from phoenix6.controls.neutral_out import NeutralOut
from phoenix6.controls.velocity_voltage import VelocityVoltage
from phoenix6.controls.position_voltage import PositionVoltage
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration


class Falcon:
    class ControlMode(Enum):
        Position = auto()
        """rotations"""
        Velocity = auto()
        """rotations/s"""
        Percent = auto()
        Amps = auto()

    class NeutralMode(Enum):
        Brake = auto()
        Coast = auto()

    class LimitSwitch(Enum):
        Forwards = auto()
        Backwards = auto()

    def __init__(
        self,
        canID: int,
        pGain: float = 1,
        iGain: float = 0,
        dGain: float = 0,
        isReversed: bool = False,
        canbus: str = "",
    ) -> None:
        self.motor = TalonFX(canID, canbus)

        conf = TalonFXConfiguration()
        conf.slot0.k_p = pGain
        conf.slot0.k_i = iGain
        conf.slot0.k_d = dGain
        conf.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if isReversed
            else InvertedValue.CLOCKWISE_POSITIVE
        )

        self.motor.configurator.apply(conf)

        self.velControl = VelocityVoltage(0)
        self.posControl = PositionVoltage(0)
        self.perControl = DutyCycleOut(0)

    def set(self, controlMode: ControlMode, demand: float, ff: float = 0) -> None:
        if controlMode == Falcon.ControlMode.Position:
            self.motor.set_control(
                self.posControl.with_velocity(demand).with_feed_forward(ff)
            )
        elif controlMode == Falcon.ControlMode.Velocity:
            self.motor.set_control(
                self.velControl.with_velocity(demand).with_feed_forward(ff)
            )
        elif controlMode == Falcon.ControlMode.Percent:
            self.motor.set_control(self.perControl.with_output(demand + ff / 12))
        elif controlMode == Falcon.ControlMode.Amps:
            raise NotImplementedError("AMP control is currently not implemented")

    def setCurrentLimit(self, lim: CurrentLimitsConfigs):
        self.motor.configurator.apply(lim)

    def neutralOutput(self):
        self.motor.set_control(NeutralOut())

    def setNeutralMode(self, mode: NeutralMode):
        conf = MotorOutputConfigs().with_neutral_mode(
            NeutralModeValue.COAST
            if mode == Falcon.NeutralMode.Coast
            else NeutralModeValue.BRAKE
        )
        self.motor.configurator.apply(conf)

    def getLimitSwitch(self, switch: LimitSwitch) -> bool:
        if switch == Falcon.LimitSwitch.Forwards:
            return (
                self.motor.get_forward_limit().value
                == ForwardLimitValue.CLOSED_TO_GROUND
            )
        elif switch == Falcon.LimitSwitch.Backwards:
            return (
                self.motor.get_reverse_limit().value
                == ReverseLimitValue.CLOSED_TO_GROUND
            )
        return False

    def get(self, controlMode: ControlMode) -> float:
        if controlMode == Falcon.ControlMode.Position:
            return self.motor.get_position().value
        elif controlMode == Falcon.ControlMode.Velocity:
            return self.motor.get_velocity().value
        elif controlMode == Falcon.ControlMode.Percent:
            return self.motor.get_motor_voltage().value / 12
        return 0

    def setEncoderPosition(self, rotations: float):
        self.motor.set_position(rotations)

    def getSimCollection(self) -> TalonFXSimState:
        return self.motor.sim_state
