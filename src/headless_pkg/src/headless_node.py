import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy import qos
from rclpy.duration import Duration

import signal
import time
import atexit

import os
import sys
import threading
import glob
import pwd
import grp
from math import copysign

from std_srvs.srv import Trigger
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from astra_msgs.msg import CoreControl, ArmManual, BioControl
from astra_msgs.msg import CoreCtrlState

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

arm_ik_qos = qos.QoSProfile(
    history=qos.QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
    durability=qos.QoSDurabilityPolicy.VOLATILE,
)

CORE_MODE = "twist"  # "twist" or "duty"

STICK_DEADZONE = float(os.getenv("STICK_DEADZONE", "0.05"))


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
        id = 0
        while True:
            self.num_gamepads = pygame.joystick.get_count()
            if id >= self.num_gamepads:
                self.get_logger().fatal("Ran out of controllers to try")
                sys.exit(1)

            try:
                self.gamepad = pygame.joystick.Joystick(id)
                self.gamepad.init()
            except Exception as e:
                self.get_logger().error("Error when initializing gamepad")
                self.get_logger().error(e)
                id += 1
                continue
            print(f"Gamepad Found: {self.gamepad.get_name()}")

            if self.gamepad.get_numhats() == 0 or self.gamepad.get_numaxes() < 5:
                self.get_logger().error("Controller not correctly initialized.")
                if not is_user_in_group("input"):
                    self.get_logger().warning(
                        "If using NixOS, you may need to add yourself to the 'input' group."
                    )
                if is_user_in_group("plugdev"):
                    self.get_logger().warning(
                        "If using NixOS, you may need to remove yourself from the 'plugdev' group."
                    )
            else:
                break
            id += 1

        self.create_timer(0.1, self.send_controls)

        self.core_publisher = self.create_publisher(CoreControl, "/core/control", 2)
        self.arm_publisher = self.create_publisher(ArmManual, "/arm/control/manual", 2)
        self.arm_ik_twist_publisher = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", arm_ik_qos
        )
        self.arm_ik_jointjog_publisher = self.create_publisher(
            JointJog, "/servo_node/delta_joint_cmds", arm_ik_qos
        )
        self.bio_publisher = self.create_publisher(BioControl, "/bio/control", 2)

        self.core_twist_pub_ = self.create_publisher(
            Twist, "/core/twist", qos_profile=control_qos
        )
        self.core_state_pub_ = self.create_publisher(
            CoreCtrlState, "/core/control/state", qos_profile=control_qos
        )

        self.declare_parameter("arm_mode", "manual")
        self.arm_mode = self.get_parameter("arm_mode").value

        self.declare_parameter("arm_manual_scheme", "old")
        self.arm_manual_scheme = self.get_parameter("arm_manual_scheme").value

        # Check parameter validity
        if self.arm_mode not in ["manual", "ik"]:
            self.get_logger().warn(
                f"Invalid value '{self.arm_mode}' for arm_mode parameter. Defaulting to 'manual' (per-axis control)."
            )
        if self.arm_manual_scheme not in ["old", "new"]:
            self.get_logger().warn(
                f"Invalid value '{self.arm_manual_scheme}' for arm_manual_scheme parameter. Defaulting to 'old' ('24 and '25 controls)."
            )

        # If using IK control, we have to "start" the servo node to enable it to accept commands
        self.servo_start_client = None
        if self.arm_mode == "ik":
            self.get_logger().info("Starting servo node for IK control...")
            self.servo_start_client = self.create_client(
                Trigger, "/servo_node/start_servo"
            )
            timeout_counter = 0
            while not self.servo_start_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for servo_node/start_servo service...")
                timeout_counter += 1
                if timeout_counter >= 10:
                    self.get_logger().error(
                        "Servo's start service not available. IK control will not work."
                    )
                    break
            if self.servo_start_client.service_is_ready():
                self.servo_start_client.call_async(Trigger.Request())

        self.ctrl_mode = "core"  # Start in core mode
        self.core_brake_mode = False
        self.core_max_duty = 0.5  # Default max duty cycle (walking speed)

        # Rumble when node is ready (returns False if rumble not supported)
        self.gamepad.rumble(0.7, 0.8, 150)

    def send_controls(self):
        """Read the gamepad state and publish control messages"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        # Check if controller is still connected
        if pygame.joystick.get_count() != self.num_gamepads:
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
            self.core_publisher.publish(CORE_STOP_MSG)
            self.arm_publisher.publish(ARM_STOP_MSG)
            self.bio_publisher.publish(BIO_STOP_MSG)
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
        if self.ctrl_mode == "arm" and self.arm_mode == "manual":
            arm_input = ArmManual()

            # Collect controller state
            left_stick_x = deadzone(self.gamepad.get_axis(0))
            left_stick_y = deadzone(self.gamepad.get_axis(1))
            left_trigger = deadzone(self.gamepad.get_axis(2))
            right_stick_x = deadzone(self.gamepad.get_axis(3))
            right_stick_y = deadzone(self.gamepad.get_axis(4))
            right_trigger = deadzone(self.gamepad.get_axis(5))
            button_a = self.gamepad.get_button(0)
            button_b = self.gamepad.get_button(1)
            button_x = self.gamepad.get_button(2)
            button_y = self.gamepad.get_button(3)
            left_bumper = self.gamepad.get_button(4)
            right_bumper = self.gamepad.get_button(5)
            dpad_input = self.gamepad.get_hat(0)

            if self.arm_manual_scheme == "old":
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

            if self.arm_manual_scheme == "new":
                # Right stick: EF yaw and axis 3
                # Left stick: axis 1 and 2
                # D-pad: axis 0 and _
                # Triggers: EF grippers
                # Bumpers: EF roll
                # A: brake
                # B: _
                # X: _
                # Y: linear actuator

                ARM_THRESHOLD = 0.2

                # Right stick: EF yaw and axis 3
                arm_input.effector_yaw = (
                    0 if abs(right_stick_x) < ARM_THRESHOLD else int(copysign(1, right_stick_x))
                )
                arm_input.axis3 = (
                    0 if abs(right_stick_y) < ARM_THRESHOLD else int(-1 * copysign(1, right_stick_y))
                )

                # Left stick: axis 1 and 2
                arm_input.axis1 = (
                    0 if abs(left_stick_x) < ARM_THRESHOLD else int(copysign(1, left_stick_x))
                )
                arm_input.axis2 = (
                    0 if abs(left_stick_y) < ARM_THRESHOLD else int(-1 * copysign(1, left_stick_y))
                )

                # D-pad: axis 0 and _
                arm_input.axis0 = (
                    0 if dpad_input[0] == 0 else int(copysign(1, dpad_input[0]))
                )

                # Triggers: EF Grippers
                if left_trigger > 0 and right_trigger > 0:
                    arm_input.gripper = 0
                elif left_trigger > 0:
                    arm_input.gripper = -1
                elif right_trigger > 0:
                    arm_input.gripper = 1

                # Bumpers: EF roll
                if left_bumper > 0 and right_bumper > 0:
                    arm_input.effector_roll = 0
                elif left_bumper > 0:
                    arm_input.effector_roll = -1
                elif right_bumper > 0:
                    arm_input.effector_roll = 1

                # A: brake
                if button_a:
                    arm_input.brake = True

                # Y: linear actuator
                if button_y:
                    arm_input.linear_actuator = 1

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

            self.arm_publisher.publish(arm_input)

        if self.ctrl_mode == "arm" and self.arm_mode == "ik":
            arm_twist = TwistStamped()
            arm_twist.header.frame_id = "base_link"
            arm_twist.header.stamp = self.get_clock().now().to_msg()
            arm_jointjog = JointJog()
            arm_jointjog.header.frame_id = "base_link"
            arm_jointjog.header.stamp = self.get_clock().now().to_msg()

            # Collect controller state
            left_stick_x = deadzone(self.gamepad.get_axis(0))
            left_stick_y = deadzone(self.gamepad.get_axis(1))
            left_trigger = deadzone(self.gamepad.get_axis(2))
            right_stick_x = deadzone(self.gamepad.get_axis(3))
            right_stick_y = deadzone(self.gamepad.get_axis(4))
            right_trigger = deadzone(self.gamepad.get_axis(5))
            button_a = self.gamepad.get_button(0)
            button_b = self.gamepad.get_button(1)
            button_x = self.gamepad.get_button(2)
            button_y = self.gamepad.get_button(3)
            left_bumper = self.gamepad.get_button(4)
            right_bumper = self.gamepad.get_button(5)
            dpad_input = self.gamepad.get_hat(0)

            # Right stick: linear y and linear x
            # Left stick: angular z and linear z
            # D-pad: angular y and _
            # Triggers: EF grippers
            # Bumpers: angular x
            # A: brake
            # B: IK mode
            # X: manual mode
            # Y: linear actuator

            # Right stick: linear y and linear x
            arm_twist.twist.linear.y = float(right_stick_x)
            arm_twist.twist.linear.x = float(right_stick_y)

            # Left stick: angular z and linear z
            arm_twist.twist.angular.z = float(-1 * left_stick_x)
            arm_twist.twist.linear.z = float(-1 * left_stick_y)
            # D-pad: angular y and _
            arm_twist.twist.angular.y = (
                float(0)
                if dpad_input[0] == 0
                else float(-1 * copysign(0.75, dpad_input[0]))
            )

            # Triggers: EF Grippers
            if left_trigger > 0 or right_trigger > 0:
                arm_jointjog.joint_names.append(
                    "Gripper_Slider_Left"  # TODO: Update joint name
                )
                arm_jointjog.velocities.append(float(right_trigger - left_trigger))

            # Bumpers: angular x
            if left_bumper > 0 and right_bumper > 0:
                arm_twist.twist.angular.x = float(0)
            elif left_bumper > 0:
                arm_twist.twist.angular.x = float(1)
            elif right_bumper > 0:
                arm_twist.twist.angular.x = float(-1)

            self.arm_ik_twist_publisher.publish(arm_twist)
            # self.arm_ik_jointjog_publisher.publish(arm_jointjog)  # TODO: Figure this shit out


def deadzone(value: float, threshold=STICK_DEADZONE) -> float:
    """Apply a deadzone to a joystick input so the motors don't sound angry"""
    if abs(value) < threshold:
        return 0
    return value


def is_user_in_group(group_name: str) -> bool:
    # Copied from https://zetcode.com/python/os-getgrouplist/
    try:
        username = os.getlogin()

        # Get group ID from name
        group_info = grp.getgrnam(group_name)
        target_gid = group_info.gr_gid

        # Get user's groups
        user_info = pwd.getpwnam(username)
        user_groups = os.getgrouplist(username, user_info.pw_gid)

        return target_gid in user_groups
    except KeyError:
        return False


def main(args=None):
    try:
        rclpy.init(args=args)

        node = Headless()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Caught shutdown signal. Exiting...")
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    signal.signal(
        signal.SIGTERM, lambda signum, frame: sys.exit(0)
    )  # Catch termination signals and exit cleanly
    main()
