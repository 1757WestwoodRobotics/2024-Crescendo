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
from phoenix6.controls.velocity_duty_cycle import VelocityDutyCycle
from phoenix6.controls.position_voltage import PositionVoltage
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.controls.follower import Follower
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode

from wpilib import RobotBase, SmartDashboard


class Talon:
    class ControlMode(Enum):
        Position = auto()
        """rotations"""
        Velocity = auto()
        """rotations/s"""
        Percent = auto()
        Amps = auto()
        MotionMagic = auto()

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
        pGain: float = 1,
        iGain: float = 0,
        dGain: float = 0,
        isReversed: bool = False,
        canbus: str = "",
        kV: float = 0,
        moMagicAccel: float = 0,
        moMagicVel: float = 0,
    ) -> None:
        print(f"Init TalonFX with port {canID} on {canbus} with name {name}")
        self.id = canID
        self.name = name
        self.motor = TalonFX(canID, canbus)

        self.isReversed = isReversed

        conf = TalonFXConfiguration()
        conf.slot0.k_p = pGain
        conf.slot0.k_i = iGain
        conf.slot0.k_d = dGain
        conf.slot0.k_v = kV
        conf.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if isReversed
            else InvertedValue.CLOCKWISE_POSITIVE
        )
        conf.motion_magic.motion_magic_acceleration = moMagicAccel
        conf.motion_magic.motion_magic_cruise_velocity = moMagicVel
        self._nettableidentifier = f"motors/{self.name}({self.id})"
        SmartDashboard.putNumber(f"{self._nettableidentifier}/gains/p", pGain)
        SmartDashboard.putNumber(f"{self._nettableidentifier}/gains/i", iGain)
        SmartDashboard.putNumber(f"{self._nettableidentifier}/gains/d", dGain)
        SmartDashboard.putNumber(f"{self._nettableidentifier}/gains/v", kV)
        SmartDashboard.putBoolean(f"{self._nettableidentifier}/inverted", isReversed)
        SmartDashboard.putString(f"{self._nettableidentifier}/canbus", canbus)

        self.motor.configurator.apply(conf)

        self.velControl = VelocityVoltage(0, 0, False, 0, 0, False, False, False)
        self.posControl = PositionVoltage(0, 0, False, 0, 0, False, False, False)
        self.perControl = DutyCycleOut(0, False, False, False, False)
        self.moMagic = MotionMagicVoltage(0)

        self.velControl.slot = 0
        self.posControl.slot = 0

        if RobotBase.isSimulation():
            self.motor.get_position().set_update_frequency(100)
            self.motor.get_velocity().set_update_frequency(100)
        print("...Done")

    def set(
        self,
        controlMode: ControlMode,
        demand: float,
        ff: float = 0,
        duty_cycle: bool = True,
    ) -> None:
        self.updateDashboard()
        SmartDashboard.putNumber(f"{self._nettableidentifier}/target", demand)
        if controlMode == Talon.ControlMode.Position:
            c = self.motor.set_control(
                self.posControl.with_position(demand).with_feed_forward(ff)
            )
        elif controlMode == Talon.ControlMode.Velocity:
            c = self.motor.set_control(
                self.velControl.with_velocity(demand).with_feed_forward(ff)
                if not duty_cycle
                else VelocityDutyCycle(demand, feed_forward=ff)
            )
        elif controlMode == Talon.ControlMode.Percent:
            c = self.motor.set_control(self.perControl.with_output(demand + ff / 12))
        elif controlMode == Talon.ControlMode.MotionMagic:
            c = self.motor.set_control(self.moMagic.with_position(demand))
        elif controlMode == Talon.ControlMode.Amps:
            raise NotImplementedError("AMP control is currently not implemented")

        if c != StatusCode.OK:
            print(
                f"ERROR: {c} \n ({controlMode}, {demand}, {ff}, {self.motor.device_id})"
            )

    def follow(self, other, opp_direction: bool = False):
        c = self.motor.set_control(Follower(other.id, opp_direction))
        if c != StatusCode.OK:
            print(f"ERROR: {c} \n {self.motor.device_id})")

    def updateDashboard(self):
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/position", self.motor.get_position().value
        )
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/velocity", self.motor.get_velocity().value
        )
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/acceleration",
            self.motor.get_acceleration().value,
        )
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/outvoltage",
            self.motor.get_motor_voltage().value,
        )
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/supplyvoltage",
            self.motor.get_supply_voltage().value,
        )
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/temp", self.motor.get_device_temp().value
        )
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/dutycycle", self.motor.get_duty_cycle().value
        )
        SmartDashboard.putNumber(
            f"{self._nettableidentifier}/current",
            self.motor.get_torque_current().value,
        )

    def setCurrentLimit(self, lim: CurrentLimitsConfigs):
        self.motor.configurator.apply(lim)

    def neutralOutput(self):
        self.motor.set_control(NeutralOut())

    def setNeutralMode(self, mode: NeutralMode):
        conf = (
            MotorOutputConfigs()
            .with_neutral_mode(
                NeutralModeValue.COAST
                if mode == Talon.NeutralMode.Coast
                else NeutralModeValue.BRAKE
            )
            .with_inverted(
                InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                if self.isReversed
                else InvertedValue.CLOCKWISE_POSITIVE
            )
        )
        self.motor.configurator.apply(conf)

    def getLimitSwitch(self, switch: LimitSwitch) -> bool:
        if switch == Talon.LimitSwitch.Forwards:
            return (
                self.motor.get_forward_limit().value
                == ForwardLimitValue.CLOSED_TO_GROUND
            )
        elif switch == Talon.LimitSwitch.Backwards:
            return (
                self.motor.get_reverse_limit().value
                == ReverseLimitValue.CLOSED_TO_GROUND
            )
        return False

    def get(self, controlMode: ControlMode) -> float:
        self.updateDashboard()
        if controlMode == Talon.ControlMode.Position:
            return self.motor.get_position().value
        elif controlMode == Talon.ControlMode.Velocity:
            return self.motor.get_velocity().value
        elif controlMode == Talon.ControlMode.Percent:
            return self.motor.get_motor_voltage().value / 12
        return 0

    def setEncoderPosition(self, rotations: float):
        self.motor.set_position(rotations)

    def getSimCollection(self) -> TalonFXSimState:
        return self.motor.sim_state
