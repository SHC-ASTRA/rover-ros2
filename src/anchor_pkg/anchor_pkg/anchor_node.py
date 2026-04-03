from warnings import deprecated
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

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
import threading

from astra_msgs.msg import VicCAN
from std_msgs.msg import String


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
    * /anchor/to_vic/debug
        - A string copy of the messages published to ./relay are published here

    Subscribers:
    * /anchor/from_vic/mock_mcu
        - For testing without an actual MCU, publish strings here as if they came from an MCU
    * /anchor/to_vic/relay
        - Core, Arm, and Bio publish VicCAN messages to this topic to send to the MCU
    * /anchor/to_vic/relay_string
        - Send raw strings to connectors. Does not work for connectors that require conversion (like CANConnector)
    * /anchor/relay
        - Legacy method for talking to connectors. Takes String as input, but does not send the raw strings to connectors.
          Instead, it converts them to VicCAN messages first.
    """

    connector: Connector

    def __init__(self):
        super().__init__("anchor_node")

        logger = self.get_logger()

        # ROS2 Parameter Setup

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

        self.declare_parameter(
            "can_override",
            "",
            ParameterDescriptor(
                name="can_override",
                description="Overrides which CAN channel will be used. Defaults to ''.",
                type=ParameterType.PARAMETER_STRING,
                additional_constraints="Must be a valid CAN network that shows up in `ip link show`.",
            ),
        )

        self.declare_parameter(
            "serial_override",
            "",
            ParameterDescriptor(
                name="serial_override",
                description="Overrides which serial port will be used. Defaults to ''.",
                type=ParameterType.PARAMETER_STRING,
                additional_constraints="Must be a valid path to a serial device file that shows up in `ls /dev/tty*`.",
            ),
        )

        # Determine which connector to use. Options are Mock, Serial, and CAN
        connector_select = (
            self.get_parameter("connector").get_parameter_value().string_value
        )
        can_override = (
            self.get_parameter("can_override").get_parameter_value().string_value
        )
        serial_override = (
            self.get_parameter("serial_override").get_parameter_value().string_value
        )
        match connector_select:
            case "serial":
                logger.info("using serial connector")
                self.connector = SerialConnector(
                    logger, self.get_clock(), serial_override
                )
            case "can":
                logger.info("using CAN connector")
                self.connector = CANConnector(logger, self.get_clock(), can_override)
            case "mock":
                logger.info("using mock connector")
                self.connector = MockConnector(logger, self.get_clock())
            case "auto":
                logger.info("automatically determining connector")
                try:
                    logger.info("trying CAN connector")
                    self.connector = CANConnector(
                        logger, self.get_clock(), can_override
                    )
                except (NoValidDeviceException, NoWorkingDeviceException, TypeError):
                    logger.info("CAN connector failed, trying serial connector")
                    self.connector = SerialConnector(
                        logger, self.get_clock(), serial_override
                    )
            case _:
                logger.fatal(
                    f"invalid value for connector parameter: {connector_select}"
                )
                exit(1)

        # ROS2 Topic Setup

        # Publishers
        self.fromvic_debug_pub_ = self.create_publisher(  # only used by serial
            String,
            "/anchor/from_vic/debug",
            20,
        )
        self.fromvic_core_pub_ = self.create_publisher(
            VicCAN,
            "/anchor/from_vic/core",
            20,
        )
        self.fromvic_arm_pub_ = self.create_publisher(
            VicCAN,
            "/anchor/from_vic/arm",
            20,
        )
        self.fromvic_bio_pub_ = self.create_publisher(
            VicCAN,
            "/anchor/from_vic/bio",
            20,
        )
        # Debug publisher
        self.tovic_debug_pub_ = self.create_publisher(
            VicCAN,
            "/anchor/to_vic/debug",
            20,
        )

        # Subscribers
        self.tovic_sub_ = self.create_subscription(
            VicCAN,
            "/anchor/to_vic/relay",
            self.write_connector,
            20,
        )
        self.tovic_sub_legacy_ = self.create_subscription(
            String,
            "/anchor/relay",
            self.write_connector_legacy,
            20,
        )
        self.mock_mcu_sub_ = self.create_subscription(
            String,
            "/anchor/from_vic/mock_mcu",
            self.on_mock_fromvic,
            20,
        )
        self.tovic_string_sub_ = self.create_subscription(
            String,
            "/anchor/to_vic/relay_string",
            self.connector.write_raw,
            20,
        )

        # Close devices on exit
        atexit.register(self.cleanup)

    def cleanup(self):
        self.connector.cleanup()

    def read_connector(self):
        """Check the connector for new data from the MCU, and publish string to appropriate topics"""
        viccan, raw = self.connector.read()

        if raw:
            self.fromvic_debug_pub_.publish(String(data=raw))

        if viccan:
            self.relay_fromvic(viccan)

    def write_connector(self, msg: VicCAN):
        """Write to the connector and send a copy to /anchor/to_vic/debug"""
        self.connector.write(msg)
        self.tovic_debug_pub_.publish(msg)

    @deprecated("Use /anchor/to_vic/relay or /anchor/to_vic/relay_string instead of /anchor/relay")
    def write_connector_legacy(self, msg: String):
        """Write to the connector by first attempting to convert String to VicCAN"""
        # please do not reference this code. ~riley
        for cmd in msg.data.split("\n"):
            viccan = string_to_viccan(
                cmd,
                "anchor",
                self.get_logger(),
                self.get_clock().now().to_msg(),
            )
            if viccan:
                self.write_connector(viccan)

    def relay_fromvic(self, msg: VicCAN):
        """Relay a message from the MCU to the appropriate VicCAN topic"""
        if msg.mcu_name == "core":
            self.fromvic_core_pub_.publish(msg)
        elif msg.mcu_name == "arm" or msg.mcu_name == "digit":
            self.fromvic_arm_pub_.publish(msg)
        elif msg.mcu_name == "citadel" or msg.mcu_name == "digit":
            self.fromvic_bio_pub_.publish(msg)

    def on_mock_fromvic(self, msg: String):
        """Relay a message as if it came from the MCU"""
        viccan = string_to_viccan(
            msg.data,
            "mock",
            self.get_logger(),
            self.get_clock().now().to_msg(),
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
            anchor_node.read_connector()  # Check the connector for updates
            rate.sleep()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Caught shutdown signal, shutting down...")
    finally:
        rclpy.try_shutdown()
