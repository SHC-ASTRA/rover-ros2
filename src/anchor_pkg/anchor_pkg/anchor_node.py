import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

import signal
import time
import atexit

import serial
import os
import sys
import threading
import glob

from std_msgs.msg import String, Header
from ros2_interfaces_pkg.msg import VicCAN

serial_pub = None
thread = None


"""
Publishers:
  * /anchor/from_vic/debug
    - Every string received from the MCU is published here for debugging
  * /anchor/from_vic/core
    - VicCAN messages for Core node
  * /anchor/from_vic/arm
    - VicCAN messages for Arm node
  * /anchor/from_vic/bio
    - VicCAN messages for Bio node

Subscribers:
  * /anchor/from_vic/mock_mcu
    - For testing without an actual MCU, publish strings here as if they came from an MCU
  * /anchor/to_vic/relay
    - Core, Arm, and Bio publish VicCAN messages to this topic to send to the MCU
  * /anchor/to_vic/relay_string
    - Publish raw strings to this topic to send directly to the MCU for debugging
"""
class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("anchor_node")#previously 'serial_publisher'

        # New pub/sub with VicCAN
        self.fromvic_debug_pub_ = self.create_publisher(String, '/anchor/from_vic/debug', 20)
        self.fromvic_core_pub_ = self.create_publisher(VicCAN, '/anchor/from_vic/core', 20)
        self.fromvic_arm_pub_ = self.create_publisher(VicCAN, '/anchor/from_vic/arm', 20)
        self.fromvic_bio_pub_ = self.create_publisher(VicCAN, '/anchor/from_vic/bio', 20)

        self.mock_mcu_sub_ = self.create_subscription(String, '/anchor/from_vic/mock_mcu', self.on_mock_fromvic, 20)
        self.tovic_sub_ = self.create_subscription(VicCAN, '/anchor/to_vic/relay', self.on_relay_tovic_viccan, 20)
        self.tovic_debug_sub_ = self.create_subscription(String, '/anchor/to_vic/relay_string', self.on_relay_tovic_string, 20)


        # Create publishers 
        self.arm_pub = self.create_publisher(String, '/anchor/arm/feedback', 10)
        self.core_pub = self.create_publisher(String, '/anchor/core/feedback', 10)
        self.bio_pub = self.create_publisher(String, '/anchor/bio/feedback', 10)

        self.debug_pub = self.create_publisher(String, '/anchor/debug', 10)

        # Create a subscriber 
        self.relay_sub = self.create_subscription(String, '/anchor/relay', self.on_relay_tovic_string, 10)

        # Loop through all serial devices on the computer to check for the MCU
        self.port = None
        # self.port = "/tmp/ttyACM9"  # Fake port, for debugging
        ports = SerialRelay.list_serial_ports()
        for i in range(4):
            if self.port is not None:
                break
            for port in ports:
                try:
                    # connect and send a ping command
                    ser = serial.Serial(port, 115200, timeout=1)
                    #(f"Checking port {port}...")
                    ser.write(b"ping\n")
                    response = ser.read_until(bytes("\n", "utf8"))

                    # if pong is in response, then we are talking with the MCU
                    if b"pong" in response:
                        self.port = port
                        self.get_logger().info(f"Found MCU at {self.port}!")
                        break
                except:
                    pass

        if self.port is None:
            self.get_logger().info("Unable to find MCU...")
            time.sleep(1)
            sys.exit(1)

        self.ser = serial.Serial(self.port, 115200)
        self.get_logger().info(f"Enabling Relay Mode")
        self.ser.write(b"can_relay_mode,on\n")
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
        """ Check the USB serial port for new data from the MCU, and publish string to appropriate topics """
        try:
            output = str(self.ser.readline(), "utf8")
            
            if output:
                self.relay_fromvic(output)
                # All output over debug temporarily
                #self.get_logger().info(f"[MCU] {output}")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
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
            exit(1)
        except TypeError as e:
            print(f"TypeError: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            exit(1)
        except Exception as e:
            print(f"Exception: {e}")
            # print("Closing serial port.")
            # if self.ser.is_open:
            #     self.ser.close()
            # exit(1)


    def on_mock_fromvic(self, msg: String):
        """ For testing without an actual MCU, publish strings here as if they came from an MCU """
        # self.get_logger().info(f"Got command from mock MCU: {msg}")
        self.relay_fromvic(msg.data)


    def on_relay_tovic_viccan(self, msg: VicCAN):
        """ Relay a VicCAN message to the MCU """
        output: str = f"can_relay_tovic,{msg.mcu_name},{msg.command_id}"
        for num in msg.data:
            output += f",{round(num, 7)}"  # limit to 7 decimal places
        output += "\n"
        # self.get_logger().info(f"VicCAN relay to MCU: {output}")
        self.ser.write(bytes(output, "utf8"))

    def relay_fromvic(self, msg: str):
        """ Relay a string message from the MCU to the appropriate VicCAN topic """
        self.fromvic_debug_pub_.publish(String(data=msg))
        parts = msg.strip().split(",")
        if len(parts) > 0 and parts[0] != "can_relay_fromvic":
            self.get_logger().debug(f"Ignoring non-VicCAN message: '{msg.strip()}'")
            return

        # String validation
        malformed: bool = False
        malformed_reason: str = ""
        if len(parts) < 3 or len(parts) > 7:
            malformed = True
            malformed_reason = f"invalid argument count (expected [3,7], got {len(parts)})"
        elif parts[1] not in ["core", "arm", "digit", "citadel", "broadcast"]:
            malformed = True
            malformed_reason = f"invalid mcu_name '{parts[1]}'"
        elif parts[2].isnumeric() is False or int(parts[2]) < 0:
            malformed = True
            malformed_reason = f"command_id '{parts[2]}' is not a non-negative integer"
        else:
            for x in parts[3:]:
                try:
                    float(x)
                except ValueError:
                    malformed = True
                    malformed_reason = f"data '{x}' is not a float"
                    break

        if malformed:
            self.get_logger().warning(f"Ignoring malformed from_vic message: '{msg.strip()}'; reason: {malformed_reason}")
            return

        # Have valid VicCAN message

        output = VicCAN()
        output.mcu_name = parts[1]
        output.command_id = int(parts[2])
        if len(parts) > 3:
            output.data = [float(x) for x in parts[3:]]
        output.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="from_vic")

        # self.get_logger().info(f"Relaying from MCU: {output}")
        if output.mcu_name == "core":
            self.fromvic_core_pub_.publish(output)
        elif output.mcu_name == "arm" or output.mcu_name == "digit":
            self.fromvic_arm_pub_.publish(output)
        elif output.mcu_name == "citadel" or output.mcu_name == "digit":
            self.fromvic_bio_pub_.publish(output)


    def on_relay_tovic_string(self, msg: String):
        """ Relay a raw string message to the MCU for debugging """
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
    #signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))  # Catch termination signals and exit cleanly
    main()
