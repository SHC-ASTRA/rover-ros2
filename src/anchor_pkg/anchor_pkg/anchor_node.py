import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Empty

import signal
import time
import atexit

import serial
import serial.tools.list_ports
import os
import sys
import threading
import glob

from std_msgs.msg import String, Header
from astra_msgs.msg import VicCAN

KNOWN_USBS = [
    (0x2E8A, 0x00C0),  # Raspberry Pi Pico
    (0x1A86, 0x55D4),  # Adafruit Feather ESP32 V2
    (0x10C4, 0xEA60),  # DOIT ESP32 Devkit V1
    (0x1A86, 0x55D3),  # ESP32 S3 Development Board
]


class Anchor(Node):
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

    def __init__(self):
        # Initalize node with name
        super().__init__("anchor_node")  # previously 'serial_publisher'

        self.serial_port: str | None = None  # e.g., "/dev/ttyUSB0"

        # Serial port override
        if port_override := os.getenv("PORT_OVERRIDE"):
            self.serial_port = port_override

        ##################################################
        # Serial MCU Discovery

        # If there was not a port override, look for a MCU over USB for Serial.
        if self.serial_port is None:
            comports = serial.tools.list_ports.comports()
            real_ports = list(
                filter(
                    lambda p: p.vid is not None
                    and p.pid is not None
                    and p.device is not None,
                    comports,
                )
            )
            recog_ports = list(filter(lambda p: (p.vid, p.pid) in KNOWN_USBS, comports))

            if len(recog_ports) == 1:  # Found singular recognized MCU
                found_port = recog_ports[0]
                self.get_logger().info(
                    f"Selecting MCU '{found_port.description}' at {found_port.device}."
                )
                self.serial_port = found_port.device
            elif len(recog_ports) > 1:  # Found multiple recognized MCUs
                # Kinda jank log message
                self.get_logger().error(
                    f"Found multiple recognized MCUs: {[p.device for p in recog_ports].__str__()}"
                )
                # time.sleep(1)
                # sys.exit(1)
            elif (
                len(recog_ports) == 0 and len(real_ports) > 0
            ):  # Found real ports but none recognized
                self.get_logger().error(
                    f"No recognized MCUs found; instead found {[p.device for p in real_ports].__str__()}."
                )
                # time.sleep(1)
                # sys.exit(1)
            else:  # Found jack shit
                self.get_logger().error("No valid Serial ports specified or found.")
                # time.sleep(1)
                # sys.exit(1)

        # We still don't have a serial port; fall back to legacy discovery (Areeb's code)
        # Loop through all serial devices on the computer to check for the MCU
        if self.serial_port is None:
            self.get_logger().warning("Falling back to legacy MCU discovery...")
            ports = Anchor.list_serial_ports()
            for _ in range(4):
                if self.serial_port is not None:
                    break
                for port in ports:
                    try:
                        # connect and send a ping command
                        ser = serial.Serial(port, 115200, timeout=1)
                        # (f"Checking port {port}...")
                        ser.write(b"ping\n")
                        response = ser.read_until(bytes("\n", "utf8"))

                        # if pong is in response, then we are talking with the MCU
                        if b"pong" in response:
                            self.serial_port = port
                            self.get_logger().info(f"Found MCU at {self.serial_port}!")
                            break
                    except:
                        pass

        # If port is still None then we ain't finding no mcu
        if self.serial_port is None:
            self.get_logger().error("Unable to find MCU. Exiting...")
            time.sleep(1)
            sys.exit(1)
        # Found a Serial port, try to open it; above code has not officially opened a Serial port
        else:
            self.get_logger().debug(
                f"Attempting to open Serial port '{self.serial_port}'..."
            )
            try:
                self.serial_interface = serial.Serial(
                    self.serial_port, 115200, timeout=1
                )

                # Attempt to get name of connected MCU
                self.serial_interface.write(
                    b"can_relay_mode,on\n"
                )  # can_relay_ready,[mcu]
                mcu_name: str = ""
                for _ in range(4):
                    response = self.serial_interface.read_until(bytes("\n", "utf8"))
                    if b"can_relay_ready" in response:
                        args: list[str] = response.decode("utf8").strip().split(",")
                        if len(args) == 2:
                            mcu_name = args[1]
                        break
                self.get_logger().info(
                    f"MCU '{mcu_name}' is ready at '{self.serial_port}'."
                )

            except serial.SerialException as e:
                self.get_logger().error(
                    f"Could not open Serial port '{self.serial_port}' for reason:"
                )
                self.get_logger().error(e.strerror)
                time.sleep(1)
                sys.exit(1)

        # Close serial port on exit
        atexit.register(self.cleanup)

        ##################################################
        # ROS2 Topic Setup

        # New pub/sub with VicCAN
        self.fromvic_debug_pub_ = self.create_publisher(
            String, "/anchor/from_vic/debug", 20
        )
        self.fromvic_core_pub_ = self.create_publisher(
            VicCAN, "/anchor/from_vic/core", 20
        )
        self.fromvic_arm_pub_ = self.create_publisher(
            VicCAN, "/anchor/from_vic/arm", 20
        )
        self.fromvic_bio_pub_ = self.create_publisher(
            VicCAN, "/anchor/from_vic/bio", 20
        )

        self.mock_mcu_sub_ = self.create_subscription(
            String, "/anchor/from_vic/mock_mcu", self.on_mock_fromvic, 20
        )
        self.tovic_sub_ = self.create_subscription(
            VicCAN, "/anchor/to_vic/relay", self.on_relay_tovic_viccan, 20
        )
        self.tovic_debug_sub_ = self.create_subscription(
            String, "/anchor/to_vic/relay_string", self.on_relay_tovic_string, 20
        )

        # Create publishers
        self.arm_pub = self.create_publisher(String, "/anchor/arm/feedback", 10)
        self.core_pub = self.create_publisher(String, "/anchor/core/feedback", 10)
        self.bio_pub = self.create_publisher(String, "/anchor/bio/feedback", 10)

        self.debug_pub = self.create_publisher(String, "/anchor/debug", 10)

        # Create a subscriber
        self.relay_sub = self.create_subscription(
            String, "/anchor/relay", self.on_relay_tovic_string, 10
        )

    def read_MCU(self):
        """Check the USB serial port for new data from the MCU, and publish string to appropriate topics"""
        try:
            output = str(self.serial_interface.readline(), "utf8")

            if output:
                self.relay_fromvic(output)
                # All output over debug temporarily
                # self.get_logger().info(f"[MCU] {output}")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
                if output.startswith("can_relay_fromvic,core"):
                    self.core_pub.publish(msg)
                elif output.startswith("can_relay_fromvic,arm") or output.startswith(
                    "can_relay_fromvic,digit"
                ):  # digit for voltage readings
                    self.arm_pub.publish(msg)
                if output.startswith("can_relay_fromvic,citadel") or output.startswith(
                    "can_relay_fromvic,digit"
                ):  # digit for SHT sensor
                    self.bio_pub.publish(msg)
                # msg = String()
                # msg.data = output
                # self.debug_pub.publish(msg)
                return
        except serial.SerialException as e:
            print(f"SerialException: {e}")
            print("Closing serial port.")
            try:
                if self.serial_interface.is_open:
                    self.serial_interface.close()
            except:
                pass
            exit(1)
        except TypeError as e:
            print(f"TypeError: {e}")
            print("Closing serial port.")
            try:
                if self.serial_interface.is_open:
                    self.serial_interface.close()
            except:
                pass
            exit(1)
        except Exception as e:
            print(f"Exception: {e}")
            # print("Closing serial port.")
            # if self.ser.is_open:
            #     self.ser.close()
            # exit(1)

    def on_mock_fromvic(self, msg: String):
        """For testing without an actual MCU, publish strings here as if they came from an MCU"""
        # self.get_logger().info(f"Got command from mock MCU: {msg}")
        self.relay_fromvic(msg.data)

    def on_relay_tovic_viccan(self, msg: VicCAN):
        """Relay a VicCAN message to the MCU"""
        output: str = f"can_relay_tovic,{msg.mcu_name},{msg.command_id}"
        for num in msg.data:
            output += f",{round(num, 7)}"  # limit to 7 decimal places
        output += "\n"
        # self.get_logger().info(f"VicCAN relay to MCU: {output}")
        self.serial_interface.write(bytes(output, "utf8"))

    def relay_fromvic(self, msg: str):
        """Relay a string message from the MCU to the appropriate VicCAN topic"""
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
            malformed_reason = (
                f"invalid argument count (expected [3,7], got {len(parts)})"
            )
        elif parts[1] not in ["core", "arm", "digit", "citadel", "broadcast"]:
            malformed = True
            malformed_reason = f"invalid mcu_name '{parts[1]}'"
        elif not (parts[2].isnumeric()) or int(parts[2]) < 0:
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
            self.get_logger().warning(
                f"Ignoring malformed from_vic message: '{msg.strip()}'; reason: {malformed_reason}"
            )
            return

        # Have valid VicCAN message

        output = VicCAN()
        output.mcu_name = parts[1]
        output.command_id = int(parts[2])
        if len(parts) > 3:
            output.data = [float(x) for x in parts[3:]]
        output.header = Header(
            stamp=self.get_clock().now().to_msg(), frame_id="from_vic"
        )

        # self.get_logger().info(f"Relaying from MCU: {output}")
        if output.mcu_name == "core":
            self.fromvic_core_pub_.publish(output)
        elif output.mcu_name == "arm" or output.mcu_name == "digit":
            self.fromvic_arm_pub_.publish(output)
        elif output.mcu_name == "citadel" or output.mcu_name == "digit":
            self.fromvic_bio_pub_.publish(output)

    def on_relay_tovic_string(self, msg: String):
        """Relay a raw string message to the MCU for debugging"""
        message = msg.data
        # self.get_logger().info(f"Sending command to MCU: {msg}")
        self.serial_interface.write(bytes(message, "utf8"))

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")

    def cleanup(self):
        print("Cleaning up before terminating...")
        if self.serial_interface.is_open:
            self.serial_interface.close()


def main(args=None):
    try:
        rclpy.init(args=args)
        anchor_node = Anchor()

        thread = threading.Thread(target=rclpy.spin, args=(anchor_node,), daemon=True)
        thread.start()

        rate = anchor_node.create_rate(100)  # 100 Hz -- arbitrary rate
        while rclpy.ok():
            anchor_node.read_MCU()  # Check the MCU for updates
            rate.sleep()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Caught shutdown signal, shutting down...")
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    signal.signal(
        signal.SIGTERM, lambda signum, frame: sys.exit(0)
    )  # Catch termination signals and exit cleanly
    main()
