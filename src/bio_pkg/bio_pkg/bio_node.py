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
from ros2_interfaces_pkg.msg import BioControl

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initialize node
        super().__init__("bio_node")

        # Get launch mode parameter
        self.declare_parameter('launch_mode', 'bio')
        self.launch_mode = self.get_parameter('launch_mode').value
        self.get_logger().info(f"bio launch_mode is: {self.launch_mode}")

        # Create publishers
        self.debug_pub = self.create_publisher(String, '/bio/feedback/debug', 10)
        #self.socket_pub = self.create_publisher(SocketFeedback, '/arm/feedback/socket', 10)


        # Create subscribers\
        self.control_sub = self.create_subscription(BioControl, '/bio/control', self.send_control, 10)

        # Topics used in anchor mode
        if self.launch_mode == 'anchor':
            self.anchor_sub = self.create_subscription(String, '/anchor/bio/feedback', self.anchor_feedback, 10)
            self.anchor_pub = self.create_publisher(String, '/anchor/relay', 10)


        # Search for ports IF in 'arm' (standalone) and not 'anchor' mode
        if self.launch_mode == 'bio':
            # Loop through all serial devices on the computer to check for the MCU
            self.port = None
            for i in range(2):
                try:
                    # connect and send a ping command
                    set_port = '/dev/ttyACM0' #MCU is controlled through GPIO pins on the PI
                    ser = serial.Serial(set_port, 115200, timeout=1)
                    #print(f"Checking port {port}...")
                    ser.write(b"ping\n")
                    response = ser.read_until("\n")

                    # if pong is in response, then we are talking with the MCU
                    if b"pong" in response:
                        self.port = set_port
                        self.get_logger().info(f"Found MCU at {set_port}!")
                        break
                except:
                    pass
            
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
                if self.launch_mode == 'bio':
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


    def send_control(self, msg):
        # CITADEL Control Commands
        ################


        # Chem Pumps, only send if not zero
        if msg.pumpID != 0:
            command = "can_relay_tovic,citadel,27," + str(msg.pumpID) + "," + str(msg.pumpAmount) + "\n"
            self.send_cmd(command)
        # Fans, only send if not zero
        if msg.fanID != 0:
            command = "can_relay_tovic,citadel,40," + str(msg.fanID) + "," + str(msg.fanDuration) + "\n"
            self.send_cmd(command)
        # Servos, only send if not zero
        if msg.servoID != 0:
            command = "can_relay_tovic,citadel,25," + str(msg.servoID) + "," + str(msg.servoPosition) + "\n"
            self.send_cmd(command)        
        

        # LSS
        command = "can_relay_tovic,citadel,24," + str(msg.lssDirection) + "\n"
        self.send_cmd(command)
        # Vibration Motor
        command = "can_relay_tovic,citadel,26," + str(msg.vibrationMotor) + "\n"
        self.send_cmd(command)
        

        # FAERIE Control Commands 
        ################
        
        # To be reviewed before use#

        # # Laser
        # command = "can_relay_tovic,faerie,28," + str(msg.laser) + "\n"
        # self.send_cmd(command)
        
        # # UV Light
        # command = "can_relay_tovic,faerie,38," + str(msg.uvLight) + "\n"
        # self.send_cmd(command)

        # # Drill
        # command = "can_relay_tovic,faerie,19," + str(msg.drillDuty) + "\n"
        # self.send_cmd(command)



    def send_cmd(self, msg):
        if self.launch_mode == 'anchor': #if in anchor mode, send to anchor node to relay
            output = String()
            output.data = msg
            self.anchor_pub.publish(output)
        elif self.launch_mode == 'bio': #if in standalone mode, send to MCU directly
            self.get_logger().info(f"[Bio to MCU] {msg}")
            self.ser.write(bytes(msg, "utf8"))

    def anchor_feedback(self, msg):
        self.get_logger().info(f"[Bio Anchor] {msg.data}")
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
