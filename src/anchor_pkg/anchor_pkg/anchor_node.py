import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

import signal
import time
import atexit

import serial
import sys
import threading
import glob

from std_msgs.msg import String
from ros2_interfaces_pkg.msg import CoreFeedback
from ros2_interfaces_pkg.msg import CoreControl

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("anchor_node")#previously 'serial_publisher'


        # Create publishers 
        self.arm_pub = self.create_publisher(String, '/anchor/arm/feedback', 10)
        self.core_pub = self.create_publisher(String, '/anchor/core/feedback', 10)
        self.bio_pub = self.create_publisher(String, '/anchor/bio/feedback', 10)

        self.debug_pub = self.create_publisher(String, '/anchor/debug', 10)
        
        # Create a subscriber 
        self.relay_sub = self.create_subscription(String, '/anchor/relay', self.send_cmd, 10)

        # Loop through all serial devices on the computer to check for the MCU
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for i in range(4):
            for port in ports:
                try:
                    # connect and send a ping command
                    ser = serial.Serial(port, 115200, timeout=1)
                    #(f"Checking port {port}...")
                    ser.write(b"ping\n")
                    response = ser.read_until("\n")
                    ser.write(b"can_relay_mode,on\n")

                    # if pong is in response, then we are talking with the MCU
                    if b"pong" in response:
                        self.port = port
                        self.get_logger().info(f"Found MCU at {self.port}!")
                        self.get_logger().info(f"Enabling Relay Mode")
                        ser.write(b"can_relay_mode,on\n")
                        break
                except:
                    pass
            if self.port is not None:
                break
        
        if self.port is None:
            self.get_logger().info("Unable to find MCU...")
            time.sleep(1)
            sys.exit(1)
        
        self.ser = serial.Serial(self.port, 115200)
        atexit.register(self.cleanup)


    def run(self):
        # This thread makes all the update processes run in the background
        global thread
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        
        try:
            while rclpy.ok():
                self.read_MCU() # Check the MCU for updates
        except KeyboardInterrupt:
            sys.exit(0)

    def read_MCU(self):
        try:
            output = str(self.ser.readline(), "utf8")
            
            if output:
                # All output over debug temporarily
                #self.get_logger().info(f"[MCU] {output}")
                msg = String()
                msg.data = output
                if output.startswith("can_relay_fromvic,core"):
                    self.core_pub.publish(msg)
                elif output.startswith("can_relay_fromvic,arm") or output.startswith("can_relay_fromvic,digit"):  # digit for voltage readings
                    self.arm_pub.publish(msg)
                if output.startswith("can_relay_fromvic,citadel") or output.startswith("can_relay_fromvic,digit"):  # digit for SHT sensor
                    self.bio_pub.publish(msg)
                # msg = String()
                # msg.data = output
                # self.debug_pub.publish(msg)
                return
        except serial.SerialException as e:
            print(f"SerialException: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            self.exit(1)
        except TypeError as e:
            print(f"TypeError: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            self.exit(1)
        except Exception as e:
            print(f"Exception: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            self.exit(1)

    def send_cmd(self, msg):
        message = msg.data
        #self.get_logger().info(f"Sending command to MCU: {msg}")
        self.ser.write(bytes(message, "utf8"))

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    
    def cleanup(self):
        print("Cleaning up before terminating...")
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
