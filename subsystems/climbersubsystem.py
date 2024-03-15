from wpilib import (DoubleSolenoid,
    PneumaticsModuleType,
    Solenoid, SmartDashboard)
from commands2 import Subsystem
from enum import Enum, auto
import constants
from util.simtalon import Talon


class ClimberSubsystem(Subsystem):
    class ClimberState(Enum):
        ClimberRetract = auto()
        ClimberExtend = auto()
        ClimberNeutral = auto()
        
    def __init__(self):
     Subsystem.__init__(self)
     self.setName(__class__.__name__)
    
     self.climberMotor = Talon(
            constants.kClimberCANID,
            constants.kClimberName,
            constants.kClimberPGain,
            constants.kClimberIGain,
            constants.kClimberDGain,
            constants.kClimberInverted 
        )
     
     self.climberMotor.setCurrentLimit(
         
     )
     self.state = ClimberSubsystem.ClimberState.ClimberNeutral


            
    def periodic(self) -> None:
         SmartDashboard.putString(constants.kClimberStateKey, str(self.state))

         if self.state == self.ClimberState.ClimberRetract:
              self.climberMotor.set(-constants.kClimberMotorPercent)
         elif self.state == self.ClimberState.ClimberExtend:
              self.climberMotor.set(constants.kClimberMotorPercent)
         elif self.state == self.ClimberState.ClimberNeutral:
              self.climberMotor.setNeutralMode


    def setClimberRetract(self) -> None:
         self.state = self.ClimberState.ClimberRetract

    def setClimberExtend(self) -> None:
         self.state = self.ClimberState.ClimberExtend
     
    def setClimberHold(self) -> None:
         self.state = self.ClimberState.ClimberNeutral
    