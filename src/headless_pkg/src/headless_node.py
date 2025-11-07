import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.duration import Duration

import signal
import time
import atexit

import os
import sys
import threading
import glob
from math import copysign

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ros2_interfaces_pkg.msg import CoreControl, ArmManual, BioControl
from ros2_interfaces_pkg.msg import CoreCtrlState

import pygame

os.environ["SDL_VIDEODRIVER"] = "dummy"  # Prevents pygame from trying to open a display
os.environ["SDL_AUDIODRIVER"] = (
    "dummy"  # Force pygame to use a dummy audio driver before pygame.init()
)


CORE_STOP_MSG = CoreControl()  # All zeros by default
CORE_STOP_TWIST_MSG = Twist()  # "
ARM_STOP_MSG = ArmManual()  # "
BIO_STOP_MSG = BioControl()  # "

control_qos = qos.QoSProfile(
    history=qos.QoSHistoryPolicy.KEEP_LAST,
    depth=2,
    reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
    durability=qos.QoSDurabilityPolicy.VOLATILE,
    deadline=Duration(seconds=1),
    lifespan=Duration(nanoseconds=500_000_000),  # 500ms
    liveliness=qos.QoSLivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration=Duration(seconds=5),
)

CORE_MODE = "twist"  # "twist" or "duty"


class Headless(Node):
    def __init__(self):
        # Initialize pygame first
        pygame.init()
        pygame.joystick.init()
        super().__init__("headless")

        # Wait for anchor to start
        pub_info = self.get_publishers_info_by_topic("/anchor/from_vic/debug")
        while len(pub_info) == 0:
            self.get_logger().info("Waiting for anchor to start...")
            time.sleep(1.0)
            pub_info = self.get_publishers_info_by_topic("/anchor/from_vic/debug")

        # Wait for a gamepad to be connected
        print("Waiting for gamepad connection...")
        while pygame.joystick.get_count() == 0:
            # Process any pygame events to keep it responsive
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit(0)
            time.sleep(1.0)  # Check every second
            print("No gamepad found. Waiting...")

        # Initialize the gamepad
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        print(f"Gamepad Found: {self.gamepad.get_name()}")

        self.create_timer(0.15, self.send_controls)

        self.core_publisher = self.create_publisher(CoreControl, "/core/control", 2)
        self.arm_publisher = self.create_publisher(ArmManual, "/arm/control/manual", 2)
        self.bio_publisher = self.create_publisher(BioControl, "/bio/control", 2)

        self.core_twist_pub_ = self.create_publisher(
            Twist, "/core/twist", qos_profile=control_qos
        )
        self.core_state_pub_ = self.create_publisher(
            CoreCtrlState, "/core/control/state", qos_profile=control_qos
        )

        self.ctrl_mode = "core"  # Start in core mode
        self.core_brake_mode = False
        self.core_max_duty = 0.5  # Default max duty cycle (walking speed)

        # Rumble when node is ready (returns False if rumble not supported)
        self.gamepad.rumble(0.7, 0.8, 150)

    def run(self):
        # This thread makes all the update processes run in the background
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()

        try:
            while rclpy.ok():
                self.send_controls()
                time.sleep(0.1)  # Small delay to avoid CPU hogging
        except KeyboardInterrupt:
            sys.exit(0)

    def send_controls(self):
        """Read the gamepad state and publish control messages"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        # Check if controller is still connected
        if pygame.joystick.get_count() == 0:
            print("Gamepad disconnected. Exiting...")
            # Send one last zero control message
            self.core_publisher.publish(CORE_STOP_MSG)
            self.arm_publisher.publish(ARM_STOP_MSG)
            self.bio_publisher.publish(BIO_STOP_MSG)
            self.get_logger().info("Final stop commands sent. Shutting down.")
            # Clean up
            pygame.quit()
            sys.exit(0)

        new_ctrl_mode = self.ctrl_mode  # if "" then inequality will always be true

        # Check for control mode change
        dpad_input = self.gamepad.get_hat(0)
        if dpad_input[1] == 1:
            new_ctrl_mode = "arm"
        elif dpad_input[1] == -1:
            new_ctrl_mode = "core"

        if new_ctrl_mode != self.ctrl_mode:
            self.gamepad.rumble(0.6, 0.7, 75)
            self.ctrl_mode = new_ctrl_mode
            self.get_logger().info(f"Switched to {self.ctrl_mode} control mode")

        # CORE
        if self.ctrl_mode == "core" and CORE_MODE == "duty":
            input = CoreControl()
            input.max_speed = 90

            # Collect controller state
            left_stick_y = deadzone(self.gamepad.get_axis(1))
            right_stick_y = deadzone(self.gamepad.get_axis(4))
            right_trigger = deadzone(self.gamepad.get_axis(5))

            # Right wheels
            input.right_stick = float(round(-1 * right_stick_y, 2))

            # Left wheels
            if right_trigger > 0:
                input.left_stick = input.right_stick
            else:
                input.left_stick = float(round(-1 * left_stick_y, 2))

            # Debug
            output = f"L: {input.left_stick}, R: {input.right_stick}"
            self.get_logger().info(f"[Ctrl] {output}")

            self.core_publisher.publish(input)
            self.arm_publisher.publish(ARM_STOP_MSG)
            # self.bio_publisher.publish(BIO_STOP_MSG)

        elif self.ctrl_mode == "core" and CORE_MODE == "twist":
            input = Twist()

            # Collect controller state
            left_stick_y = deadzone(self.gamepad.get_axis(1))
            right_stick_x = deadzone(self.gamepad.get_axis(3))
            button_a = self.gamepad.get_button(0)
            left_bumper = self.gamepad.get_button(4)
            right_bumper = self.gamepad.get_button(5)

            # Forward/back and Turn
            input.linear.x = -1.0 * left_stick_y
            input.angular.z = -1.0 * copysign(
                right_stick_x**2, right_stick_x
            )  # Exponent for finer control (curve)

            # Publish
            self.core_twist_pub_.publish(input)
            self.arm_publisher.publish(ARM_STOP_MSG)
            # self.bio_publisher.publish(BIO_STOP_MSG)
            self.get_logger().info(
                f"[Core Ctrl] Linear: {round(input.linear.x, 2)}, Angular: {round(input.angular.z, 2)}"
            )

            # Brake mode
            new_brake_mode = button_a
            # Max duty cycle
            if left_bumper:
                new_max_duty = 0.25
            elif right_bumper:
                new_max_duty = 0.9
            else:
                new_max_duty = 0.5

            # Only publish if needed
            if (
                new_brake_mode != self.core_brake_mode
                or new_max_duty != self.core_max_duty
            ):
                self.core_brake_mode = new_brake_mode
                self.core_max_duty = new_max_duty
                state_msg = CoreCtrlState()
                state_msg.brake_mode = bool(self.core_brake_mode)
                state_msg.max_duty = float(self.core_max_duty)
                self.core_state_pub_.publish(state_msg)
                self.get_logger().info(
                    f"[Core State] Brake: {self.core_brake_mode}, Max Duty: {self.core_max_duty}"
                )

        # ARM and BIO
        if self.ctrl_mode == "arm":
            arm_input = ArmManual()

            # Collect controller state
            left_stick_x = deadzone(self.gamepad.get_axis(0))
            left_stick_y = deadzone(self.gamepad.get_axis(1))
            left_trigger = deadzone(self.gamepad.get_axis(2))
            right_stick_x = deadzone(self.gamepad.get_axis(3))
            right_stick_y = deadzone(self.gamepad.get_axis(4))
            right_trigger = deadzone(self.gamepad.get_axis(5))
            right_bumper = self.gamepad.get_button(5)
            dpad_input = self.gamepad.get_hat(0)

            # EF Grippers
            if left_trigger > 0 and right_trigger > 0:
                arm_input.gripper = 0
            elif left_trigger > 0:
                arm_input.gripper = -1
            elif right_trigger > 0:
                arm_input.gripper = 1

            # Axis 0
            if dpad_input[0] == 1:
                arm_input.axis0 = 1
            elif dpad_input[0] == -1:
                arm_input.axis0 = -1

            if right_bumper:  # Control end effector

                # Effector yaw
                if left_stick_x > 0:
                    arm_input.effector_yaw = 1
                elif left_stick_x < 0:
                    arm_input.effector_yaw = -1

                # Effector roll
                if right_stick_x > 0:
                    arm_input.effector_roll = 1
                elif right_stick_x < 0:
                    arm_input.effector_roll = -1

            else:  # Control arm axis

                # Axis 1
                if abs(left_stick_x) > 0.15:
                    arm_input.axis1 = round(left_stick_x)

                # Axis 2
                if abs(left_stick_y) > 0.15:
                    arm_input.axis2 = -1 * round(left_stick_y)

                # Axis 3
                if abs(right_stick_y) > 0.15:
                    arm_input.axis3 = -1 * round(right_stick_y)

            # BIO
            bio_input = BioControl(
                bio_arm=int(left_stick_y * -100),
                drill_arm=int(round(right_stick_y) * -100),
            )

            # Drill motor (FAERIE)
            if deadzone(left_trigger) > 0 or deadzone(right_trigger) > 0:
                bio_input.drill = int(
                    30 * (right_trigger - left_trigger)
                )  # Max duty cycle 30%

            self.core_publisher.publish(CORE_STOP_MSG)
            self.arm_publisher.publish(arm_input)
            # self.bio_publisher.publish(bio_input)


def deadzone(value: float, threshold=0.05) -> float:
    """Apply a deadzone to a joystick input so the motors don't sound angry"""
    if abs(value) < threshold:
        return 0
    return value


def main(args=None):
    rclpy.init(args=args)
    node = Headless()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    signal.signal(
        signal.SIGTERM, lambda signum, frame: sys.exit(0)
    )  # Catch termination signals and exit cleanly
    main()
