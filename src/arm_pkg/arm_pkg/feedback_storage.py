# struct for socket feedback
class Feedback:
    def __init__(self):
        self.bat_voltage = 0.0
        self.voltage_12 = 0.0
        self.voltage_5 = 0.0
        self.voltage_3 = 0.0   
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.joint_temps = [0.0, 0.0, 0.0, 0.0]
        self.joint_voltages = [0.0, 0.0, 0.0, 0.0]
        self.joint_currents = [0.0, 0.0, 0.0, 0.0]

    def updateBusVoltages(self, voltages):
        self.bat_voltage = voltages[0]
        self.voltage_12 = voltages[1]
        self.voltage_5 = voltages[2]
        self.voltage_3 = voltages[3]

    def updateJointVoltages(self, axis, voltage):
        self.joint_voltages[axis] = voltage

    def updateJointCurrents(self, axis, current):
        self.joint_currents[axis] = current

    def updateJointTemperatures(self, axis, temperature):
        self.joint_temps[axis] = temperature

    def updateJointAngles(self, angles):
        self.joint_angles = angles
