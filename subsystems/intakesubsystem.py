from enum import Enum, auto
from commands2 import SubsystemBase

from util.simtalon import Talon
from util.simneo import NEOBrushless
from util.simcoder import CTREEncoder
import constants


class IntakeSubsystem(SubsystemBase):
    class IntakeState(Enum):
        Intaking = (
            auto()
        )  # arm is down, motor is running inwards/holding depending on note
        Holding = auto()  # arm in feeding position, motor holding
        Feeding = auto()  # arm in feeding position, motor ejecting
        Staging = auto()  # arm is in position to score amp/trap, motor holding
        Amp = auto()  # arm in same staging position, motor ejects
        Trap = auto()  # arm pushes into trap, delay(?), motor ejects

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)  # basic subsystem boilerplate

        self.pivotEncoder = CTREEncoder(
            constants.kPivotEncoderID,
            constants.kIntakeAngleOffset.degrees(),
        )

        self.intakeMotor = NEOBrushless(
            constants.kIntakeCANID,
            constants.kIntakeName,
            constants.kIntakePIDSlot,
            constants.kIntakePGain,
            constants.kIntakeIGain,
            constants.kIntakeDGain,
            constants.kIntakeInverted,
        )
        self.pivotMotor = Talon(
            constants.kPivotCANID,
            constants.kPivotName,
            constants.kPivotPGain,
            constants.kPivotIGain,
            constants.kPivotDGain,
            constants.kPivotInverted,
        )

        # pivot motor spins 60 times per arm revolution
        # use absolute encoder to determine motor position
        pivotMotorPosition = (
            self.pivotEncoder.getPosition().degrees()
            / constants.kDegeersPerRevolution
            * 60
        )
        self.pivotMotor.setEncoderPosition(pivotMotorPosition)

        self.state = self.IntakeState.Holding

    def periodic(self) -> None:
        # get actual velocity values for intake motor later

        if self.state == self.IntakeState.Intaking:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kFloorPositionAngle.degrees()
                / constants.kDegeersPerRevolution,
            )
            if self.intakeMotor.getLimitSwitch(NEOBrushless.LimitSwitch.Forwards):
                self.intakeMotor.setNeutralOutput(NEOBrushless.NeutralMode.Brake)
            else:
                self.intakeMotor.set(NEOBrushless.ControlMode.Velocity, 100)

        elif self.state == self.IntakeState.Holding:
            self.pivotMotor.set(Talon.ControlMode.Position, 0)
            self.intakeMotor.setNeutralOutput(NEOBrushless.NeutralMode.Brake)

        elif self.state == self.IntakeState.Feeding:
            self.pivotMotor.set(Talon.ControlMode.Position, 0)
            self.intakeMotor.set(NEOBrushless.ControlMode.Velocity, -100)

        elif self.state == self.IntakeState.Staging:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kStagingPositionAngle.degrees()
                / constants.kDegeersPerRevolution,
            )
            self.intakeMotor.setNeutralMode(NEOBrushless.NeutralMode.Brake)

        elif self.state == self.IntakeState.Amp:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kStagingPositionAngle.degrees()
                / constants.kDegeersPerRevolution,
            )
            self.intakeMotor.set(NEOBrushless.ControlMode.Velocity, -100)

        elif self.state == self.IntakeState.Trap:
            # move with timeout
            self.pivotMotor.motor.set_position(
                constants.kAmpScoringPositionAngle / constants.kDegeersPerRevolution,
                1,
            )
            self.intakeMotor.set(NEOBrushless.ControlMode.Velocity, -100)

    # the following methods are simply state setting, all actual motor control is done in periodic
    def setIntaking(self) -> None:
        self.state = self.IntakeState.Intaking

    def setHolding(self) -> None:
        self.state = self.IntakeState.Holding

    def setFeeding(self) -> None:
        self.state = self.IntakeState.Feeding

    def setStaging(self) -> None:
        self.state = self.IntakeState.Staging

    def setAmp(self) -> None:
        self.state = self.IntakeState.Amp

    def setTrap(self) -> None:
        self.state = self.IntakeState.Trap
