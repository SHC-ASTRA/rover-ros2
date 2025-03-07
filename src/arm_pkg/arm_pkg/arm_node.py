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
        # Create subscribers
        self.ik_sub = self.create_subscription(ArmIK, '/arm/control/ik', self.send_ik, 10) 
        self.man_sub = self.create_subscription(ArmManual, '/arm/control/manual', self.send_manual, 10)

        # Topics used in anchor mode
        if self.launch_mode == 'anchor':
            self.anchor_sub = self.create_subscription(String, '/anchor/arm/feedback', self.anchor_feedback, 10)
            self.anchor_pub = self.create_publisher(String, '/anchor/relay', 10)


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
                self.get_logger().info(f"[MCU] {output}")
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
        axis1 = msg.axis1
        axis2 = msg.axis2
        axis3 = msg.axis3

        #Send controls for arm
        command = "can_relay_tovic,arm,39," + str(axis0) + "," + str(axis1) + "," + str(axis2) + "," + str(axis3) + "\n"
        self.send_cmd(command)
        
        #Send controls for end effector
        command = "can_relay_tovic,digit,35," + str(msg.effector_roll) + "\n"
        self.send_cmd(command)
        
        command = "can_relay_tovic,digit,36,0," + str(msg.effector_yaw) + "\n"
        self.send_cmd(command)

        command = "can_relay_tovic,digit,26," + str(msg.gripper) + "\n"
        self.send_cmd(command)

        command = "can_relay_tovic,digit,28," + str(msg.laser) + "\n"
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
        self.get_logger().info(f"[Arm Anchor] {msg.data}")
        #self.send_cmd(msg.data)


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
