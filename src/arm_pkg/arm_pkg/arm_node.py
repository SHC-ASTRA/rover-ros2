import rclpy
from rclpy.node import Node
import serial
import sys
import threading
import glob
import time
import atexit
import signal
from std_msgs.msg import String
from ros2_interfaces_pkg.msg import ArmManual
from ros2_interfaces_pkg.msg import ArmIK
from ros2_interfaces_pkg.msg import SocketFeedback

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initialize node
        super().__init__("arm_node")

        # Get launch mode parameter
        self.declare_parameter('launch_mode', 'arm')
        self.launch_mode = self.get_parameter('launch_mode').value
        self.get_logger().info(f"arm launch_mode is: {self.launch_mode}")

        # Create publishers
        self.debug_pub = self.create_publisher(String, '/arm/feedback/debug', 10)
        self.socket_pub = self.create_publisher(SocketFeedback, '/arm/feedback/socket', 10)
        self.feedback_timer = self.create_timer(1.0, self.publish_feedback)

        # Create subscribers
        self.ik_sub = self.create_subscription(ArmIK, '/arm/control/ik', self.send_ik, 10) 
        self.man_sub = self.create_subscription(ArmManual, '/arm/control/manual', self.send_manual, 10)

        # Topics used in anchor mode
        if self.launch_mode == 'anchor':
            self.anchor_sub = self.create_subscription(String, '/anchor/arm/feedback', self.anchor_feedback, 10)
            self.anchor_pub = self.create_publisher(String, '/anchor/relay', 10)

        self.arm_feedback = SocketFeedback()

        # Search for ports IF in 'arm' (standalone) and not 'anchor' mode
        if self.launch_mode == 'arm':
            # Loop through all serial devices on the computer to check for the MCU
            self.port = None
            ports = SerialRelay.list_serial_ports()
            for i in range(4):
                for port in ports:
                    try:
                        # connect and send a ping command
                        ser = serial.Serial(port, 115200, timeout=1)
                        #print(f"Checking port {port}...")
                        ser.write(b"ping\n")
                        response = ser.read_until("\n")

                        # if pong is in response, then we are talking with the MCU
                        if b"pong" in response:
                            self.port = port
                            self.get_logger().info(f"Found MCU at {self.port}!")
                            break
                    except:
                        pass
                if self.port is not None:
                    break
            
            if self.port is None:
                self.get_logger().info("Unable to find MCU... please make sure it is connected.")
                time.sleep(1)
                sys.exit(1)
            
            self.ser = serial.Serial(self.port, 115200)
            atexit.register(self.cleanup)

    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()
        
        #if in arm mode, will need to read from the MCU 

        try:
            while rclpy.ok():
                if self.launch_mode == 'arm':
                    if self.ser.in_waiting:
                        self.read_mcu()
                    else:
                        time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()


    #Currently will just spit out all values over the /arm/feedback/debug topic as strings
    def read_mcu(self):
        try:
            output = str(self.ser.readline(), "utf8")
            if output:
                #self.get_logger().info(f"[MCU] {output}")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
        except serial.SerialException:
            self.get_logger().info("SerialException caught... closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
        except TypeError as e:
            self.get_logger().info(f"TypeError: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
        except Exception as e:
            print(f"Exception: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass

    def send_ik(self, msg):
        pass

    def send_manual(self, msg):
        axis0 = msg.axis0
        axis1 = -1 * msg.axis1
        axis2 = msg.axis2
        axis3 = msg.axis3

        #Send controls for arm
        command = "can_relay_tovic,arm,39," + str(axis0) + "," + str(axis1) + "," + str(axis2) + "," + str(axis3) + "\n"
        #self.send_cmd(command)
        
        #Send controls for end effector
        command += "can_relay_tovic,digit,35," + str(msg.effector_roll) + "\n"
        #self.send_cmd(command)
        
        command += "can_relay_tovic,digit,36,0," + str(msg.effector_yaw) + "\n"
        #self.send_cmd(command)

        command += "can_relay_tovic,digit,26," + str(msg.gripper) + "\n"
        #self.send_cmd(command)

        command += "can_relay_tovic,digit,28," + str(msg.laser) + "\n"
        self.send_cmd(command)
        
        
        
        #print(f"[Wrote] {command}", end="")

    #Not yet finished, needs embedded implementation for new commands
        # ef_roll = msg.effector_roll
        # ef_yaw = msg.effector_yaw
        # gripper = msg.gripper
        # actuator = msg.linear_actuator
        # laser = msg.laser
        # #Send controls for digit

        # command = "can_relay_tovic,digit," + str(ef_roll) + "," + str(ef_yaw) + "," + str(gripper) + "," + str(actuator) + "," + str(laser) + "\n"

        return
    
    def send_cmd(self, msg):
        if self.launch_mode == 'anchor': #if in anchor mode, send to anchor node to relay
            output = String()
            output.data = msg
            self.anchor_pub.publish(output)
        elif self.launch_mode == 'arm': #if in standalone mode, send to MCU directly
            self.get_logger().info(f"[Arm to MCU] {msg}")
            self.ser.write(bytes(msg, "utf8"))

    def anchor_feedback(self, msg):
        output = msg.data
        if output.startswith("can_relay_fromvic,arm,55"):
            #pass
            self.updateAngleFeedback(output)
        elif output.startswith("can_relay_fromvic,arm,54"):
            #pass
            self.updateBusVoltage(output)
        elif output.startswith("can_relay_fromvic,arm,53"):
            #pass
            self.updateMotorFeedback(output)
        else:
            return

    def publish_feedback(self):
        self.socket_pub.publish(self.arm_feedback)

    def updateAngleFeedback(self, msg):
                # Angle feedbacks,
        #split the msg.data by commas
        parts = msg.split(",")
        
        if len(parts) >= 7:
            # Extract the angles from the string
            angles_in = parts[3:7]
            # Convert the angles to floats divide by 10.0
            angles = [float(angle) / 10.0 for angle in angles_in]
            #angles[0] = 0.0 #override axis0 to zero
            #
            #
            #THIS NEEDS TO BE REMOVED LATER
            #PLACEHOLDER FOR WRIST VALUE 
            #
            #
            angles.append(0.0)#placeholder for wrist_continuous
            angles.append(0.0)#placeholder for wrist
            #
            #
            # # Update the arm's current angles
            #self.arm.update_angles(angles)
            self.arm_feedback.axis0_angle = angles[0]
            self.arm_feedback.axis1_angle = angles[1]
            self.arm_feedback.axis2_angle = angles[2]
            self.arm_feedback.axis3_angle = angles[3]
            # self.get_logger().info(f"Angles: {angles}")
            # #debug publish angles
            # tempMsg = String()
            # tempMsg.data = "Angles: " + str(angles)
            # #self.debug_pub.publish(tempMsg)
        else:
            self.get_logger().info("Invalid angle feedback input format")


    def updateBusVoltage(self, msg):
        # Bus Voltage feedbacks
        parts = msg.split(",")
        if len(parts) >= 7:
            # Extract the voltage from the string
            voltages_in = parts[3:7]
            # Convert the voltages to floats
            self.arm_feedback.bat_voltage = float(voltages_in[0]) / 100.0
            self.arm_feedback.voltage_12 = float(voltages_in[1]) / 100.0
            self.arm_feedback.voltage_5 = float(voltages_in[2]) / 100.0
            self.arm_feedback.voltage_3 = float(voltages_in[3]) / 100.0
        else:
            self.get_logger().info("Invalid voltage feedback input format")


    def updateMotorFeedback(self, msg):
        # Motor voltage/current/temperature feedback
        return
        # parts = msg.split(",")
        # if len(parts) >= 7:
        #     # Extract the voltage/current/temperature from the string
        #     values_in = parts[3:7]
        #     # Convert the voltages to floats
        #     for i in range(4):
        #         #update arm_feedback's axisX_temp for each axis0_temp, axis1_temp, etc...
        #         pass

        #         # self.arm_feedback.updateJointVoltages(i, float(values_in[i]) / 10.0)
        #         # self.arm_feedback.updateJointCurrents(i, float(values_in[i]) / 10.0)
        #         # self.arm_feedback.updateJointTemperatures(i, float(values_in[i]) / 10.0)
        # else:
        #     self.get_logger().info("Invalid motor feedback input format")


    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        #return glob.glob("/dev/tty[A-Za-z]*")

    def cleanup(self):
        print("Cleaning up...")
        if self.ser.is_open:
            self.ser.close()

def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)
    if serial_pub:
        serial_pub.cleanup()

def main(args=None):
    rclpy.init(args=args)
    sys.excepthook = myexcepthook

    global serial_pub
    serial_pub = SerialRelay()
    serial_pub.run()

if __name__ == '__main__':
    signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))  # Catch termination signals and exit cleanly
    main()
