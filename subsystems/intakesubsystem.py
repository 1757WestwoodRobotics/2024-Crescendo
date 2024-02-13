from enum import Enum, auto
from commands2 import Subsystem
from wpilib import SmartDashboard
from util.simtalon import Talon
from util.simneo import NEOBrushless
from util.simcoder import CTREEncoder
import constants


class IntakeSubsystem(Subsystem):
    class IntakeState(Enum):
        Intaking = auto()  # arm is down, motor intaking/holding
        Holding = auto()  # arm in feeding position, motor holding
        Feeding = auto()  # arm in feeding position, motor passing through
        Staging = auto()  # arm is in position to score amp/trap, motor holding
        Amp = auto()  # arm in same staging position, motor ejects
        Trap = auto()  # arm pushes into trap, delay(?), motor ejects
        Ejecting = auto()  # arm is down, motor ejects

    def __init__(self) -> None:
        Subsystem.__init__(self)
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
            self.pivotEncoder.getPosition().radians()
            / constants.kRadiansPerRevolution
            * constants.kPivotGearRatio
        )
        self.pivotMotor.setEncoderPosition(pivotMotorPosition)

        self.state = self.IntakeState.Holding

        self.frontSensor = NEOBrushless.LimitSwitch.Forwards
        self.backSensor = NEOBrushless.LimitSwitch.Backwards
        self.hasPosition = False
        self.heldPosition = 0

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kIntakeStateKey, self.state.name)
        # get actual velocity values for intake motor later

        frontLimitState = self.intakeMotor.getLimitSwitch(self.frontSensor)
        backLimitState = self.intakeMotor.getLimitSwitch(self.backSensor)
        self.hasPosition = backLimitState or frontLimitState
        if self.state == self.IntakeState.Intaking:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kFloorPositionAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
            )

            if self.hasPosition:
                self.intakeMotor.set(NEOBrushless.ControlMode.Velocity, 0)
            # If either sensor is covered, stop the motor

        elif self.state == self.IntakeState.Holding:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kHandoffAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
            )
            # none - intaking
            # only front - keep intaking
            # front and back - get position and hold
            # only back - go to held position from front and back
            if frontLimitState and backLimitState:
                if not self.hasPosition:
                    self.heldPosition = self.intakeMotor.get(
                        NEOBrushless.ControlMode.Position
                    )
                self.intakeMotor.set(
                    NEOBrushless.ControlMode.Position, self.heldPosition
                )
            elif (
                not frontLimitState and backLimitState
            ):  # Note: not only aplies to first part (this is for front off back on)
                self.intakeMotor.set(
                    NEOBrushless.ControlMode.Position, self.heldPosition
                )
            else:
                self.intakeMotor.set(
                    NEOBrushless.ControlMode.Velocity, constants.kIntakeSpeed
                )
            self.intakeMotor.set(NEOBrushless.ControlMode.Position, self.heldPosition)

        elif self.state == self.IntakeState.Feeding:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kHandoffAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
            )
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Velocity, constants.kIntakeSpeed
            )

        elif self.state == self.IntakeState.Staging:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kStagingPositionAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
            )
            self.intakeMotor.set(NEOBrushless.ControlMode.Position, self.heldPosition)

        elif self.state == self.IntakeState.Amp:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kStagingPositionAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
            )
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Velocity, constants.kIntakeSpeed * -1
            )

        elif self.state == self.IntakeState.Trap:
            # move with timeout
            self.pivotMotor.motor.set_position(
                constants.kAmpScoringPositionAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
                1,
            )
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Velocity, constants.kIntakeSpeed * -1
            )

        elif self.state == self.IntakeState.Ejecting:
            self.pivotMotor.set(
                Talon.ControlMode.Position,
                constants.kFloorPositionAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
            )
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Velocity, constants.kIntakeSpeed * -1
            )

        SmartDashboard.putNumber(
            constants.kPivotAngleKey, self.pivotEncoder.getPosition().radians()
        )
        SmartDashboard.putNumber(
            constants.kIntakeSpeedKey,
            self.intakeMotor.get(NEOBrushless.ControlMode.Velocity),
        )
        SmartDashboard.putBoolean(constants.kIntakeHasNoteKey, self.hasPosition)

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

    def setEjecting(self) -> None:
        self.state = self.IntakeState.Ejecting
