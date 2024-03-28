from wpilib import SmartDashboard


def getSDArray(key: str, defaultValue):
    """use only for number arrays"""
    return (
        SmartDashboard.getValue(key).getDoubleArray()
        if SmartDashboard.getValue(key).isValid()
        else defaultValue
    )
