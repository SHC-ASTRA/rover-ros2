import signal
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Header, String

from astra_msgs.action import BioVacuum
from astra_msgs.msg import BioControl, BioDistributor, BioFeedback, VicCAN
from astra_msgs.srv import BioTestTube

serial_pub = None
thread = None

# used to verify the length of an incoming VicCAN feedback message
# key is VicCAN command_id, value is expected length of data list
viccan_citadel_msg_len_dict = {
    # empty because not expecting any VicCAN from citadel atm
}
viccan_lance_msg_len_dict = {
    # empty because not sure what VicCAN commands LANCE sends
}


class SerialRelay(Node):
    def __init__(self):
        # Initialize node
        super().__init__("bio_node")

        # Get launch mode parameter
        self.declare_parameter("launch_mode")
        self.launch_mode = self.get_parameter("launch_mode").value
        self.get_logger().info(f"bio launch_mode is: {self.launch_mode}")

        # Anchor Topics
        if self.launch_mode == "anchor":
            self.anchor_fromvic_sub_ = self.create_subscription(
                VicCAN, "/anchor/from_vic/bio", self.relay_fromvic, 20
            )
            self.anchor_tovic_pub_ = self.create_publisher(
                VicCAN, "/anchor/to_vic/relay", 20
            )

            self.anchor_sub = self.create_subscription(
                String, "/anchor/bio/feedback", self.anchor_feedback, 10
            )
            self.anchor_pub = self.create_publisher(String, "/anchor/relay", 10)

            self.distributor_sub = self.create_subscription(
                BioDistributor,
                "/bio/control/distributor",
                self.distributor_callback,
                10,
            )

        # Services

        self.test_tube_service = self.create_service(
            BioTestTube, "/bio/control/test_tube", self.test_tube_callback
        )

        # Actions
        self._action_server = ActionServer(
            self, BioVacuum, "/bio/actions/vacuum", self.execute_vacuum
        )

    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        try:
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    def send_control(self, msg: BioControl):
        print("todo")

    def send_cmd(self, msg: str):
        if (
            self.launch_mode == "anchor"
        ):  # if in anchor mode, send to anchor node to relay
            output = String()
            output.data = msg
            self.anchor_pub.publish(output)

    def relay_fromvic(self, msg: VicCAN):
        # self.get_logger().info(msg)
        if msg.mcu_name == "citadel":
            self.process_fromvic_citadel(msg)

    def process_fromvic_citadel(self, msg: VicCAN):
        if msg.mcu_name != "citadel":
            return

        # Check message len to prevent crashing on bad data
        if msg.command_id in viccan_citadel_msg_len_dict:
            expected_len = viccan_citadel_msg_len_dict[msg.command_id]
            if len(msg.data) != expected_len:
                self.get_logger().warning(
                    f"Ignoring VicCAN message with id {msg.command_id} due to unexpected data length (expected {expected_len}, got {len(msg.data)})"
                )
                return

    def anchor_feedback(self, msg: String):
        output = msg.data
        parts = str(output.strip()).split(",")
        self.get_logger().info(f"[Bio Anchor] {msg.data}")
        # no data planned to be received from citadel, not sure about lance

    def distributor_callback(self, msg: BioDistributor):
        distributor_arr = msg.distributor_id
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="citadel",
            command_id=40,
            data=[
                to_short(distributor_arr[0]),
                to_short(distributor_arr[1]),
                to_short(distributor_arr[2]),
                0,
            ],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)

    def test_tube_callback(self, request, response):
        tubes_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="citadel",
            command_id=40,
            data=[
                float(int.from_bytes(request.tube_id)),
                float(request.milliliters),
            ],
        )
        self.anchor_tovic_pub_.publish(tubes_cmd)
        return response

    def execute_vacuum(self, goal_handle):
        valve_id = int.from_bytes(goal_handle.request.valve_id)
        duty = int.from_bytes(goal_handle.request.fan_duty_cycle_percent)
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

        # set fan duty cycle
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg()),
                mcu_name="citadel",
                command_id=19,
                data=[float(duty)],
            )
        )

        feedback = BioVacuum.Feedback()
        start = time.time()

        while True:
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


def to_short(x: int) -> int:
    return max(-32768, min(32767, x))


def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)


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
