from enum import Enum, auto
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpilib._wpilib import RobotBase
from wpimath.geometry import Rotation2d
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
            constants.kPivotEncoderID, constants.kPivotEncoderOffset
        )

        self.intakeMotor = NEOBrushless(
            constants.kIntakeCANID,
            constants.kIntakeName,
            constants.kIntakePIDSlot,
            constants.kIntakePGain,
            constants.kIntakeIGain,
            constants.kIntakeDGain,
            constants.kIntakeInverted,
            enableLimitSwitches=False,
        )
        self.pivotMotor = Talon(
            constants.kPivotCANID,
            constants.kPivotName,
            constants.kPivotPGain,
            constants.kPivotIGain,
            constants.kPivotDGain,
            constants.kPivotInverted,
            moMagicAccel=constants.kPivotAccel,
            moMagicVel=constants.kPivotVel,
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
        self.heldPosition = self.intakeMotor.get(NEOBrushless.ControlMode.Position)
        self.putInPlace = False
        self.targetAngle = Rotation2d()
        self.holdSet = False
        self.holdPosition = 0
        self.canMoveNote = False
        self.shooterPosition = 0

    def resetPivot(self) -> None:
        pivotMotorPosition = (
            self.pivotEncoder.getPosition().radians()
            / constants.kRadiansPerRevolution
            * constants.kPivotGearRatio
        )
        self.pivotMotor.setEncoderPosition(pivotMotorPosition)
        self.holdPosition = 0
        self.holdSet = False

    def centerNote(self, frontLimitState, backLimitState) -> None:
        if self.hasPosition:
            self.intakeMotor.enableLimitSwitch(NEOBrushless.LimitSwitch.Forwards, False)
        else:
            self.intakeMotor.enableLimitSwitch(NEOBrushless.LimitSwitch.Forwards, True)
        # If either sensor is covered, stop the h

        if self.putInPlace:
            self.intakeMotor.set(NEOBrushless.ControlMode.Position, self.heldPosition)
        elif frontLimitState and backLimitState:
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Percent,
                constants.kIntakeFineControlVoltage,
            )
        elif not frontLimitState and backLimitState:
            self.heldPosition = (
                self.intakeMotor.get(NEOBrushless.ControlMode.Position)
                + constants.kIntakeSafetyPositionOffset
            )
            self.intakeMotor.set(NEOBrushless.ControlMode.Position, self.heldPosition)
            self.putInPlace = True
        else:
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Percent, constants.kIntakePercentageVoltage
            )

    def holdingState(self, backLimitState: bool) -> None:
        self.setPivotAngle(constants.kHandoffAngle)
        if (
            abs(self.getPivotAngle() - constants.kHandoffAngle.radians())
            < constants.kIntakePivotTolerance
        ):
            self.canMoveNote = True
        else:
            self.canMoveNote = False

        if self.hasPosition and self.canMoveNote:
            if backLimitState:
                self.intakeMotor.set(
                    NEOBrushless.ControlMode.Percent,
                    -constants.kIntakeFineControlVoltage,
                )
                self.holdSet = False
            else:
                if self.holdSet:
                    self.intakeMotor.set(
                        NEOBrushless.ControlMode.Position, self.shooterPosition
                    )
                else:
                    self.holdSet = True
                    self.shooterPosition = self.intakeMotor.get(
                        NEOBrushless.ControlMode.Position
                    )

        else:
            self.intakeMotor.set(NEOBrushless.ControlMode.Percent, 0)

    def periodic(self) -> None:
        SmartDashboard.putString(constants.kIntakeStateKey, self.state.name)
        # get actual velocity values for intake motor later

        frontLimitState = self.intakeMotor.getLimitSwitch(self.frontSensor)
        backLimitState = self.intakeMotor.getLimitSwitch(self.backSensor)
        self.hasPosition = backLimitState or frontLimitState
        if not self.hasPosition:
            self.putInPlace = False

        if self.state == self.IntakeState.Intaking:
            self.setPivotAngle(constants.kFloorPositionAngle)
            self.centerNote(frontLimitState, backLimitState)

        elif self.state == self.IntakeState.Holding:
            self.holdingState(backLimitState)
            # none - intaking
            # only front - keep intaking
            # front and back - get position and hold
            # only back - go to held position from front and back

            # put in place to stop when in a good spot
            # Only front to continue
            # On rising edge of back state (both, it's a known position), intake a bit more and lock

        elif self.state == self.IntakeState.Feeding:
            self.setPivotAngle(constants.kHandoffAngle)
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Percent, constants.kIntakePercentageVoltage
            )

        elif self.state == self.IntakeState.Staging:
            self.setPivotAngle(constants.kStagingPositionAngle)
            self.intakeMotor.set(NEOBrushless.ControlMode.Position, self.heldPosition)

        elif self.state == self.IntakeState.Amp:
            self.setPivotAngle(constants.kStagingPositionAngle)
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Percent,
                -constants.kIntakePercentageVoltage,
            )

        elif self.state == self.IntakeState.Trap:
            # move with timeout
            self.pivotMotor.motor.set_position(
                constants.kTrapPositionAngle.radians()
                / constants.kRadiansPerRevolution
                * constants.kPivotGearRatio,
                1,
            )
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Velocity,
                constants.kIntakePercentageVoltage * -1,
            )

        elif self.state == self.IntakeState.Ejecting:
            self.setPivotAngle(constants.kFloorPositionAngle)
            self.intakeMotor.set(
                NEOBrushless.ControlMode.Velocity,
                constants.kIntakePercentageVoltage * -1,
            )

        if RobotBase.isSimulation():
            SmartDashboard.putNumber(constants.kPivotAngleKey, self.getPivotAngle())
        else:
            SmartDashboard.putNumber(
                constants.kPivotAngleKey, self.pivotEncoder.getPosition().radians()
            )
        SmartDashboard.putNumber(
            constants.kIntakeSpeedKey,
            self.intakeMotor.get(NEOBrushless.ControlMode.Velocity),
        )
        SmartDashboard.putBoolean(constants.kIntakeHasNoteKey, self.hasPosition)
        SmartDashboard.putBoolean(
            constants.kIntakeAtPositionKey, self.intakeAtPosition()
        )

    def setPivotAngle(self, rotation: Rotation2d) -> None:
        self.targetAngle = rotation
        self.pivotMotor.set(
            Talon.ControlMode.MotionMagic,
            rotation.radians()
            / constants.kRadiansPerRevolution
            * constants.kPivotGearRatio,
        )

    def getPivotAngle(self) -> float:
        return (
            self.pivotMotor.get(Talon.ControlMode.Position)
            / constants.kPivotGearRatio
            * constants.kRadiansPerRevolution
        )

    # the following methods are simply state setting, all actual motor control is done in periodic
    def setIntaking(self) -> None:
        self.state = self.IntakeState.Intaking

    def setHolding(self) -> None:
        self.state = self.IntakeState.Holding

    def setStaging(self) -> None:
        self.state = self.IntakeState.Staging

    def setFeeding(self) -> None:
        self.state = self.IntakeState.Feeding

    def setAmp(self) -> None:
        self.state = self.IntakeState.Amp

    def setTrap(self) -> None:
        self.state = self.IntakeState.Trap

    def setEjecting(self) -> None:
        self.state = self.IntakeState.Ejecting

    def intakeAtPosition(self) -> bool:
        return (
            abs(self.targetAngle.radians() - self.getPivotAngle())
            < constants.kIntakePivotTolerance
        )
