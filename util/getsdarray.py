from wpilib import SmartDashboard

from ntcore import NetworkTableInstance

inst = NetworkTableInstance.getDefault().getTable("SmartDashboard")

def getSDArray(key: str, defaultValue):
    """use only for number arrays"""
    return (
        SmartDashboard.getValue(key).getDoubleArray()
        if SmartDashboard.getValue(key).isValid()
        else defaultValue
    )

def putSDArray(key: str, value):
    """use only for number arrays"""
    inst.getEntry(key).setDoubleArray(value)
