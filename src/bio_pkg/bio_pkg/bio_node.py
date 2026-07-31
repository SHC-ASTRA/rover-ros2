import signal
import sys
import time
from typing import Callable, TypeVar

import rclpy
from astra_msgs.action import BioVacuum
from astra_msgs.msg import NewBioFeedback, CitadelControl, LanceControl, VicCAN
from astra_msgs.srv import BioTestTube, FireLibs
from rclpy.action import ActionServer
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Header, String

##################################################
# Shared code -- candidate for unilib.
# Keep this section identical across all five node files (anchor, arm, bio, core, headless).

# NOTE: The commented-out QoS options can break other nodes and CLI commands if enabled.
# The values are left here for if they are desired in the future.
CONTROL_QOS = qos.QoSProfile(
    history=qos.QoSHistoryPolicy.KEEP_LAST,
    depth=2,
    reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,  # Best Effort subscribers are still compatible with Reliable publishers
    durability=qos.QoSDurabilityPolicy.VOLATILE,
    # deadline=Duration(seconds=1),
    # lifespan=Duration(nanoseconds=500_000_000),  # 500ms
    # liveliness=qos.QoSLivelinessPolicy.SYSTEM_DEFAULT,
    # liveliness_lease_duration=Duration(seconds=5),
)

# Template parameter type
T = TypeVar("T", bool, int, float, str)


def create_param(node: Node, name: str, default: T, silent: bool = False) -> T:
    """Declare a parameter on `node` and return its value, logging it unless silent.

    Example usage:

    ```
    self.use_ros2_control = create_param(self, "use_ros2_control", True)
    ```
    """
    try:
        node.declare_parameter(name, default)
    except InvalidParameterTypeException as e:
        node.get_logger().fatal(f"Invalid type: {e}")
        sys.exit(1)
    value: T = node.get_parameter(name).value  # type: ignore (trust me bro it's T)
    if not silent:
        node.get_logger().info(f"P: {name} = {value}")
    return value


def create_list_param(
    node: Node, name: str, default: list[T], silent: bool = False
) -> list[T]:
    """Declare a list parameter on `node` and return its value, logging it unless silent."""
    try:
        node.declare_parameter(name, default)
    except InvalidParameterTypeException as e:
        node.get_logger().fatal(f"Invalid type: {e}")
        sys.exit(1)
    value: list[T] = node.get_parameter(name).value  # type: ignore (trust me bro it's T)
    if not silent:
        node.get_logger().info(f"P: {name} = {value}")
    return value


def exit_handler(signum: int, frame):
    """Exit cleanly on a termination signal."""
    print(f"Caught {signal.Signals(signum).name}. Exiting...")
    rclpy.try_shutdown()
    sys.exit(0)


def run_node(node_factory: Callable[[], Node], args=None) -> None:
    """Init rclpy, construct the node, and spin until shutdown (Ctrl-C, SIGTERM/SIGHUP, or rclpy)."""
    node: Node | None = None
    try:
        rclpy.init(args=args)
        # Catch termination signals and exit cleanly
        # SIGTERM: systemd stop / launch shutdown. SIGHUP: SSH or terminal dropped
        for sig in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, exit_handler)
        node = node_factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Caught shutdown signal. Exiting...")
    finally:
        if node is not None:
            node.destroy_node()  # runs node-specific cleanup (e.g. Anchor's connector)
        rclpy.try_shutdown()


# End of shared code
##################################################


class SerialRelay(Node):
    def __init__(self):
        # Initialize node
        super().__init__("bio_node")

        # Anchor Topics
        self.anchor_fromvic_sub_ = self.create_subscription(
            VicCAN, "/anchor/from_vic/bio", self.relay_fromvic, 20
        )
        self.anchor_tovic_pub_ = self.create_publisher(
            VicCAN, "/anchor/to_vic/relay", 20
        )

        self.bio_feedback_pub = self.create_publisher(
            NewBioFeedback, "/anchor/bio/new_feedback", 10
        )
        self.anchor_pub = self.create_publisher(String, "/anchor/relay", 10)

        # Messages
        self.citadel_sub = self.create_subscription(
            CitadelControl,
            "/bio/citadel/control",
            self.citadel_callback,
            10,
        )

        self.lance_sub = self.create_subscription(
            LanceControl,
            "/bio/lance/control",
            self.lance_callback,
            10,
        )

        # Services
        self.test_tube_service = self.create_service(
            BioTestTube, "/bio/test_tube", self.test_tube_callback
        )
        self.libs_service = self.create_service(
            FireLibs, "bio/libs/fire", self.libs_fire_callback
        )

        # Actions
        self._action_server = ActionServer(
            self, BioVacuum, "/bio/vacuum", self.execute_vacuum
        )

        # Feedback state
        self.bio_feedback = NewBioFeedback()

        # Publish feedback periodically
        self.feedback_timer = self.create_timer(1.0, self.publish_bio_feedback)

    def send_cmd(self, msg: str):
        # send to anchor node to relay
        output = String()
        output.data = msg
        self.anchor_pub.publish(output)

    def relay_fromvic(self, msg: VicCAN):
        # self.get_logger().info(msg)
        if msg.mcu_name == "citadel":
            self.process_fromvic_citadel(msg)
        elif msg.mcu_name == "digit":
            self.process_fromvic_digit(msg)
        elif msg.mcu_name == "lance":
            self.process_fromvic_lance(msg)
        elif msg.mcu_name == "libs":
            self.process_fromvic_libs(msg)

    def process_fromvic_lance(self, msg: VicCAN):
        pass

    def process_fromvic_libs(self, msg: VicCAN):
        pass

    def process_fromvic_citadel(self, msg: VicCAN):
        # command 54
        if msg.command_id == 54 and len(msg.data) >= 3:
            self.bio_feedback.board_voltage.vbatt = msg.data[0] / 100.0
            self.bio_feedback.board_voltage.v12 = msg.data[1] / 100.0
            self.bio_feedback.board_voltage.v5 = msg.data[2] / 100.0

    def process_fromvic_digit(self, msg: VicCAN):
        # Command 57
        if msg.command_id == 57 and len(msg.data) >= 2:
            self.bio_feedback.drill_temp = msg.data[0]
            self.bio_feedback.drill_humidity = msg.data[1]

    def publish_bio_feedback(self):
        self.bio_feedback_pub.publish(self.bio_feedback)

    def citadel_callback(self, msg: CitadelControl):
        distributor_arr = list(msg.distributor_id)
        # Distributor Control
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="citadel",
            command_id=40,
            data=[
                float(clamp_short(distributor_arr[0])),
                float(clamp_short(distributor_arr[1])),
                float(clamp_short(distributor_arr[2])),
                0,
            ],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)
        # Move Scythe
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="digit",
            command_id=42,
            data=[float(msg.move_scythe)],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)

    def lance_callback(self, msg: LanceControl):
        # Move Lance
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="digit",
            command_id=42,
            data=[float(msg.move_lance)],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)
        # Drill Speed
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="digit",
            command_id=19,
            data=[
                float(msg.drill_speed * 100)
            ],  # change on embedded so we can go (-1,1)
        )
        self.anchor_tovic_pub_.publish(vic_cmd)
        # Drill Laser Control
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="digit",
            command_id=28,
            data=[float(msg.drill_laser)],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)

    def test_tube_callback(self, request, response):
        # Open Test Tube
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="citadel",
            command_id=40,
            data=[
                float(int(request.tube_id)),
                float(request.milliliters),
            ],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)
        return response

    def libs_fire_callback(self, request, response):
        print("todo")
        return response

    def execute_vacuum(self, goal_handle):
        valve_id = int(goal_handle.request.valve_id)
        duty = int(goal_handle.request.fan_duty_cycle_percent)
        total = goal_handle.request.fan_time_ms

        # open valve
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg()),
                mcu_name="citadel",
                command_id=40,
                data=[float(valve_id)],
            )
        )

        feedback = BioVacuum.Feedback()
        start = time.time()

        # This blocks every other callback of the Node
        while True:
            # set fan duty cycle
            self.anchor_tovic_pub_.publish(
                VicCAN(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    mcu_name="citadel",
                    command_id=19,
                    data=[float(duty)],
                )
            )

            elapsed = int((time.time() - start) * 1000)
            remaining = max(0, total - elapsed)

            feedback.fan_time_remaining_ms = remaining
            goal_handle.publish_feedback(feedback)

            if remaining == 0:
                break

            time.sleep(0.1)

        # stop fan
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg()),
                mcu_name="citadel",
                command_id=19,
                data=[0.0],
            )
        )

        # close valve
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg()),
                mcu_name="citadel",
                command_id=40,
                data=[-1.0],
            )
        )

        goal_handle.succeed()
        return BioVacuum.Result()


def clamp_short(x: int) -> int:
    return max(-32768, min(32767, x))


def main(args=None):
    run_node(SerialRelay, args)


if __name__ == "__main__":
    main()
