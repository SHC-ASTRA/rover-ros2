import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import signal
import atexit

from .connector import (
    Connector,
    MockConnector,
    SerialConnector,
    CANConnector,
    NoValidDeviceException,
    NoWorkingDeviceException,
)
from .convert import string_to_viccan
import sys
import threading

from std_msgs.msg import String, Header
from astra_msgs.msg import VicCAN


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
    """

    connector: Connector

    def __init__(self):
        super().__init__("anchor_node")

        logger = self.get_logger()

        self.declare_parameter(
            "connector",
            "auto",
            ParameterDescriptor(
                name="connector",
                description="Declares which MCU connector should be used. Defaults to 'auto'.",
                type=ParameterType.PARAMETER_STRING,
                additional_constraints="Must be 'serial', 'can', 'mock', or 'auto'.",
            ),
        )

        # Determine which connector to use. Options are Mock, Serial, and CAN
        connector_select = (
            self.get_parameter("connector").get_parameter_value().string_value
        )

        match connector_select:
            case "serial":
                logger.info("using serial connector")
                self.connector = SerialConnector(self.get_logger())
            case "can":
                logger.info("using CAN connector")
                self.connector = CANConnector(self.get_logger())
            case "mock":
                logger.info("using mock connector")
                self.connector = MockConnector(self.get_logger())
            case "auto":
                logger.info("automatically determining connector")
                try:
                    logger.info("trying CAN connector")
                    self.connector = CANConnector(self.get_logger())
                except (NoValidDeviceException, NoWorkingDeviceException, TypeError):
                    logger.info("CAN connector failed, trying serial connector")
                    self.connector = SerialConnector(self.get_logger())
            case _:
                self.get_logger().fatal(
                    f"invalid value for connector parameter: {connector_select}"
                )
                exit(1)

        # Close devices on exit
        atexit.register(self.cleanup)

        # ROS2 Topic Setup

        # Publishers
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

        # Subscribers
        self.tovic_sub_ = self.create_subscription(
            VicCAN, "/anchor/to_vic/relay", self.connector.write, 20
        )
        self.mock_mcu_sub_ = self.create_subscription(
            String, "/anchor/from_vic/mock_mcu", self.on_mock_fromvic, 20
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

    def on_mock_fromvic(self, msg: String):
        viccan = string_to_viccan(
            msg.data,
            "mock",
            self.get_logger(),
        )
        if viccan:
            self.relay_fromvic(viccan)


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
