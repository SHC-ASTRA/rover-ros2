import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import signal
import atexit

from connector import Connector, SerialConnector, CANConnector
import sys
import threading

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

    connector: Connector

    def __init__(self):
        # Initalize node with name
        super().__init__("anchor_node")  # previously 'serial_publisher'

        self.connector = SerialConnector(self.get_logger())

        # Close serial port on exit
        atexit.register(self.cleanup)

        ##################################################
        # ROS2 Topic Setup

        # Pub/sub with VicCAN
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

        self.tovic_sub_ = self.create_subscription(
            VicCAN, "/anchor/to_vic/relay", self.connector.write, 20
        )

    def cleanup(self):
        self.connector.cleanup()

    def read_MCU(self):
        """Check the USB serial port for new data from the MCU, and publish string to appropriate topics"""
        output = self.connector.read()

        if not output:
            return

        self.relay_fromvic(output)

    def relay_fromvic(self, msg: VicCAN):
        """Relay a string message from the MCU to the appropriate VicCAN topic"""
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="from_vic")

        if msg.mcu_name == "core":
            self.fromvic_core_pub_.publish(msg)
        elif msg.mcu_name == "arm" or msg.mcu_name == "digit":
            self.fromvic_arm_pub_.publish(msg)
        elif msg.mcu_name == "citadel" or msg.mcu_name == "digit":
            self.fromvic_bio_pub_.publish(msg)


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
