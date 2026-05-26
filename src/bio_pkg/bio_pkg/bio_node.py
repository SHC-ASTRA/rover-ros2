import signal
import sys
import threading
import time

import rclpy
from astra_msgs.action import BioVacuum
from astra_msgs.msg import NewBioFeedback, CitadelControl, LanceControl, VicCAN
from astra_msgs.srv import BioTestTube, FireLibs
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Header, String
from unilib import CanCmdId

serial_pub = None
thread = None


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
            NewBioFeedback, "/bio/feedback/main", 10
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

    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        try:
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    def send_cmd(self, msg: str):
        # send to anchor node to relay
        output = String()
        output.data = msg
        self.anchor_pub.publish(output)

    def relay_fromvic(self, msg: VicCAN):
        # self.get_logger().info(msg)
        if msg.mcu_name == "citadel":
            self.process_fromvic_citadel(msg)
        elif msg.mcu_name == "lance":
            self.process_fromvic_lance(msg)
        elif msg.mcu_name == "libs":
            self.process_fromvic_libs(msg)

    def process_fromvic_lance(self, msg: VicCAN):
        if msg.command_id == CanCmdId.CMD_SHT_TEMP_HUM.value and len(msg.data) >= 2:
            self.bio_feedback.drill_temp = msg.data[0]
            self.bio_feedback.drill_humidity = msg.data[1]

    def process_fromvic_libs(self, msg: VicCAN):
        pass

    def process_fromvic_citadel(self, msg: VicCAN):
        if msg.command_id == CanCmdId.CMD_POWER_VOLTAGE.value and len(msg.data) >= 3:
            self.bio_feedback.board_voltage.vbatt = msg.data[0] / 100.0
            self.bio_feedback.board_voltage.v12 = msg.data[1] / 100.0
            self.bio_feedback.board_voltage.v5 = msg.data[2] / 100.0

    def publish_bio_feedback(self):
        self.bio_feedback_pub.publish(self.bio_feedback)

    def citadel_callback(self, msg: CitadelControl):
        # Distributor Control
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg()),
                mcu_name="citadel",
                command_id=CanCmdId.CMD_CITADEL_FAN_CTRL.value,
                data=list(map(float, msg.distributors_ctrl)),
            )
        )

    def lance_callback(self, msg: LanceControl):
        # Move Lance (CMD_LANCE_LINEAR_AC: 2f32 linac_id, duty)
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="lance",
            command_id=CanCmdId.CMD_LANCE_LINEAR_AC.value,
            data=[1.0, float(msg.move_lance)],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)
        # Drill Speed (CMD_REV_SET_DUTY: 1f64 duty -1.0..1.0)
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="lance",
            command_id=CanCmdId.CMD_REV_SET_DUTY.value,
            data=[float(msg.drill_speed)],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)

    def test_tube_callback(self, request, response):
        # Open Test Tube
        vic_cmd = VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg()),
            mcu_name="citadel",
            command_id=CanCmdId.CMD_CITADEL_FAN_CTRL.value,
            data=[
                float(request.tube_id),
                float(int(request.extended)),  # msg has bool
            ],
        )
        self.anchor_tovic_pub_.publish(vic_cmd)
        return response

    def libs_fire_callback(self, request, response):
        print("todo")

    def execute_vacuum(self, goal_handle):
        valve_id = int(goal_handle.request.valve_id)
        duty = int(goal_handle.request.fan_duty_cycle_percent)
        total = goal_handle.request.fan_time_ms

        # open valve
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg()),
                mcu_name="citadel",
                command_id=CanCmdId.CMD_CITADEL_VALVES.value,
                data=[
                    1.0 if valve_id == 0 else 0.0,
                    1.0 if valve_id == 1 else 0.0,
                    1.0 if valve_id == 2 else 0.0,
                ],
            )
        )

        feedback = BioVacuum.Feedback()
        start = time.time()

        while True:
            # set fan duty cycle
            self.anchor_tovic_pub_.publish(
                VicCAN(
                    header=Header(stamp=self.get_clock().now().to_msg()),
                    mcu_name="citadel",
                    command_id=CanCmdId.CMD_REV_SET_DUTY.value,
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
                command_id=CanCmdId.CMD_REV_SET_DUTY.value,
                data=[0.0],
            )
        )

        # close valve
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg()),
                mcu_name="citadel",
                command_id=CanCmdId.CMD_CITADEL_VALVES.value,
                data=[0.0] * 3,
            )
        )

        goal_handle.succeed()
        return BioVacuum.Result()


def clamp_short(x: int) -> int:
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
