import rclpy
from rclpy.node import Node
import serial
import sys
import threading
import os
import glob
import time
import atexit
import signal
from std_msgs.msg import String
from astra_msgs.msg import BioControl
from astra_msgs.msg import BioFeedback

serial_pub = None
thread = None


class SerialRelay(Node):
    def __init__(self):
        # Initialize node
        super().__init__("bio_node")

        # Get launch mode parameter
        self.declare_parameter("launch_mode", "bio")
        self.launch_mode = self.get_parameter("launch_mode").value
        self.get_logger().info(f"bio launch_mode is: {self.launch_mode}")

        # Create publishers
        self.debug_pub = self.create_publisher(String, "/bio/feedback/debug", 10)
        self.feedback_pub = self.create_publisher(BioFeedback, "/bio/feedback", 10)

        # Create subscribers
        self.control_sub = self.create_subscription(
            BioControl, "/bio/control", self.send_control, 10
        )

        # Create a publisher for telemetry
        self.telemetry_pub_timer = self.create_timer(1.0, self.publish_feedback)

        # Topics used in anchor mode
        if self.launch_mode == "anchor":
            self.anchor_sub = self.create_subscription(
                String, "/anchor/bio/feedback", self.anchor_feedback, 10
            )
            self.anchor_pub = self.create_publisher(String, "/anchor/relay", 10)

        self.bio_feedback = BioFeedback()

        # Search for ports IF in 'arm' (standalone) and not 'anchor' mode
        if self.launch_mode == "bio":
            # Loop through all serial devices on the computer to check for the MCU
            self.port = None
            for i in range(2):
                try:
                    # connect and send a ping command
                    set_port = (
                        "/dev/ttyACM0"  # MCU is controlled through GPIO pins on the PI
                    )
                    ser = serial.Serial(set_port, 115200, timeout=1)
                    # print(f"Checking port {port}...")
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
                self.get_logger().info(
                    "Unable to find MCU... please make sure it is connected."
                )
                time.sleep(1)
                sys.exit(1)

            self.ser = serial.Serial(self.port, 115200)
            atexit.register(self.cleanup)

    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        # if in arm mode, will need to read from the MCU

        try:
            while rclpy.ok():
                if self.launch_mode == "bio":
                    if self.ser.in_waiting:
                        self.read_mcu()
                    else:
                        time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    # Currently will just spit out all values over the /arm/feedback/debug topic as strings
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

    def send_control(self, msg: BioControl):
        # CITADEL Control Commands
        ################

        # Chem Pumps, only send if not zero
        if msg.pump_id != 0:
            command = (
                "can_relay_tovic,citadel,27,"
                + str(msg.pump_id)
                + ","
                + str(msg.pump_amount)
                + "\n"
            )
            self.send_cmd(command)
        # Fans, only send if not zero
        if msg.fan_id != 0:
            command = (
                "can_relay_tovic,citadel,40,"
                + str(msg.fan_id)
                + ","
                + str(msg.fan_duration)
                + "\n"
            )
            self.send_cmd(command)
        # Servos, only send if not zero
        if msg.servo_id != 0:
            command = (
                "can_relay_tovic,citadel,25,"
                + str(msg.servo_id)
                + ","
                + str(int(msg.servo_state))
                + "\n"
            )
            self.send_cmd(command)

        # LSS (SCYTHE)
        command = "can_relay_tovic,citadel,24," + str(msg.bio_arm) + "\n"
        # self.send_cmd(command)

        # Vibration Motor
        command += "can_relay_tovic,citadel,26," + str(msg.vibration_motor) + "\n"
        # self.send_cmd(command)

        # FAERIE Control Commands
        ################

        # To be reviewed before use#

        # Laser
        command += "can_relay_tovic,digit,28," + str(msg.laser) + "\n"
        # self.send_cmd(command)

        # Drill (SCABBARD)
        command += f"can_relay_tovic,digit,19,{msg.drill:.2f}\n"
        # self.send_cmd(command)

        # Bio linear actuator
        command += "can_relay_tovic,digit,42," + str(msg.drill_arm) + "\n"
        self.send_cmd(command)

    def send_cmd(self, msg: str):
        if (
            self.launch_mode == "anchor"
        ):  # if in anchor mode, send to anchor node to relay
            output = String()
            output.data = msg
            self.anchor_pub.publish(output)
        elif self.launch_mode == "bio":  # if in standalone mode, send to MCU directly
            self.get_logger().info(f"[Bio to MCU] {msg}")
            self.ser.write(bytes(msg, "utf8"))

    def anchor_feedback(self, msg: String):
        output = msg.data
        parts = str(output.strip()).split(",")
        # self.get_logger().info(f"[Bio Anchor] {msg.data}")

        if output.startswith(
            "can_relay_fromvic,citadel,54"
        ):  # bat, 12, 5, Voltage readings * 100
            self.bio_feedback.bat_voltage = float(parts[3]) / 100.0
            self.bio_feedback.voltage_12 = float(parts[4]) / 100.0
            self.bio_feedback.voltage_5 = float(parts[5]) / 100.0
        elif output.startswith("can_relay_fromvic,digit,57"):
            self.bio_feedback.drill_temp = float(parts[3])
            self.bio_feedback.drill_humidity = float(parts[4])

    def publish_feedback(self):
        self.feedback_pub.publish(self.bio_feedback)

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        # return glob.glob("/dev/tty[A-Za-z]*")

    def cleanup(self):
        print("Cleaning up...")
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception as e:
            exit(0)


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


if __name__ == "__main__":
    # signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(
        signal.SIGTERM, lambda signum, frame: sys.exit(0)
    )  # Catch termination signals and exit cleanly
    main()
