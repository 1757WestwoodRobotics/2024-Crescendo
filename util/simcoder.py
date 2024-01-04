from phoenix6.hardware.cancoder import CANcoder
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.configs.config_groups import MagnetSensorConfigs
from phoenix6.signals.spn_enums import AbsoluteSensorRangeValue
from phoenix6.sim.cancoder_sim_state import CANcoderSimState
from wpimath.geometry import Rotation2d

import constants

class CTREEncoder:
    def __init__(self, canId: int, offset: float, canbus: str = "") -> None:
        self.encoder = CANcoder(canId, canbus)
        self.offset = offset

        config = CANcoderConfiguration().with_magnet_sensor(
            MagnetSensorConfigs()
            .with_absolute_sensor_range(AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF)
            .with_magnet_offset(-1 * self.offset / constants.kDegeersPerRevolution)
        )
        self.encoder.configurator.apply(config)

    def getDeviceNumber(self) -> int:
        return self.encoder.device_id

    def getPosition(self) -> Rotation2d:
        return Rotation2d(self.encoder.get_position().value * constants.kRadiansPerRevolution)

    def getSim(self) -> CANcoderSimState:
        return self.encoder.sim_state
