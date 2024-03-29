from ntcore import NetworkTableInstance

inst = NetworkTableInstance.getDefault().getTable("SmartDashboard")

def getSDArray(key: str, defaultValue):
    """use only for number arrays"""
    return (
        inst.getEntry(key).getDoubleArray(defaultValue)
        if inst.getEntry(key).getValue().isValid()
        else defaultValue
    )

def putSDArray(key: str, value):
    """use only for number arrays"""
    inst.getEntry(key).setDoubleArray(value)
