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

from std_msgs.msg import String
from ros2_interfaces_pkg.msg import CoreFeedback
from ros2_interfaces_pkg.msg import CoreControl

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("core_node")#previously 'serial_publisher'

        # Get launch mode parameter
        self.declare_parameter('launch_mode', 'core')
        self.launch_mode = self.get_parameter('launch_mode').value
        self.get_logger().info(f"core launch_mode is: {self.launch_mode}")

        # Create publishers 
        self.debug_pub = self.create_publisher(String, '/core/debug', 10) 
        self.feedback_pub = self.create_publisher(CoreFeedback, '/core/feedback', 10)
        # Create a subscriber 
        self.control_sub = self.create_subscription(CoreControl, '/core/control', self.send_controls, 10)

        # Create a service server for pinging the rover
        self.ping_service = self.create_service(Empty, '/astra/core/ping', self.ping_callback)

        if self.launch_mode == 'anchor':
            self.anchor_sub = self.create_subscription(String, '/anchor/core/feedback', self.anchor_feedback, 10)
            self.anchor_pub = self.create_publisher(String, '/anchor/relay', 10)


        if self.launch_mode == 'core':
            # Loop through all serial devices on the computer to check for the MCU
            self.port = None
            ports = SerialRelay.list_serial_ports()
            for i in range(2):
                for port in ports:
                    try:
                        # connect and send a ping command
                        ser = serial.Serial(port, 115200, timeout=1)
                        #(f"Checking port {port}...")
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
                self.get_logger().info("Unable to find MCU...")
                #kill the node/process entirely
                os.kill(os.getpid(), signal.SIGKILL)
                sys.exit(0)
        
            self.ser = serial.Serial(self.port, 115200)
            atexit.register(self.cleanup)


    def run(self):
        # This thread makes all the update processes run in the background
        global thread
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        
    
        try:
            while rclpy.ok():
                if self.launch_mode == 'core':
                    self.read_MCU() # Check the MCU for updates
        except KeyboardInterrupt:
            sys.exit(0)

    def read_MCU(self):
        try:
            output = str(self.ser.readline(), "utf8")
            
            if output:
                # All output over debug temporarily
                print(f"[MCU] {output}")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
                return
                # Temporary

                # packet = output.strip().split(',')
                
                # if len(packet) >= 2 and packet[0] == "core" and packet[1] == "telemetry":
                #     feedback = CoreFeedback()
                #     feedback.gpslat = float(packet[2])
                #     feedback.gpslon = float(packet[3])
                #     feedback.gpssat = float(packet[4])
                #     feedback.bnogyr.x = float(packet[5])
                #     feedback.bnogyr.y = float(packet[6])
                #     feedback.bnogyr.z = float(packet[7])
                #     feedback.bnoacc.x = float(packet[8])
                #     feedback.bnoacc.y = float(packet[9])
                #     feedback.bnoacc.z = float(packet[10])
                #     feedback.orient = float(packet[11])
                #     feedback.bmptemp = float(packet[12])
                #     feedback.bmppres = float(packet[13])
                #     feedback.bmpalt = float(packet[14])

                #     self.telemetry_publisher.publish(feedback)
                # else:
                #     # print(f"[MCU] {output}", end="")
                #     # msg = String()
                #     # msg.data = output
                #     # self.debug_pub.publish(msg)
                #     return
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
                
    def scale_duty(self, value, max_speed):
        leftMin = -1
        leftMax = 1
        rightMin = -max_speed/100.0
        rightMax = max_speed/100.0


        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return str(rightMin + (valueScaled * rightSpan))

    def send_controls(self, msg):
        #can_relay_tovic,core,19, left_stick, right_stick 
        left_stick_neg = msg.left_stick * -1
        command = "can_relay_tovic,core,19," + self.scale_duty(left_stick_neg, msg.max_speed) + ',' + self.scale_duty(msg.right_stick, msg.max_speed) + '\n'
        #print(f"[Sys] {command}", end="")
        
        #self.ser.write(bytes(command, "utf8"))# Send command to MCU
        #self.get_logger().debug(f"wrote: {command}")
        
        self.send_cmd(command)
        
        #print(f"[Sys] Relaying: {command}")
    def send_cmd(self, msg):
        if self.launch_mode == 'anchor':
            #self.get_logger().info(f"[Core to Anchor Relay] {msg}")
            output = String()#Convert to std_msg string
            output.data = msg
            self.anchor_pub.publish(output)
        elif self.launch_mode == 'core':
            self.get_logger().info(f"[Core to MCU] {msg}")
            self.ser.write(bytes(msg, "utf8"))

    def anchor_feedback(self, msg):
        self.get_logger().info(f"[Core Anchor] {msg}")

    def ping_callback(self, request, response):
        return response


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

