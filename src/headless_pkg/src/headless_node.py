import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy import qos
from rclpy.duration import Duration

import signal
import time

import os
import sys
import pwd
import grp
from math import copysign

from std_srvs.srv import Trigger
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from astra_msgs.msg import CoreControl, ArmManual, BioControl
from astra_msgs.msg import CoreCtrlState

import warnings

# Literally headless
warnings.filterwarnings(
    "ignore",
    message="Your system is avx2 capable but pygame was not built with support for it.",
)

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
    # deadline=Duration(seconds=1),
    # lifespan=Duration(nanoseconds=500_000_000),  # 500ms
    # liveliness=qos.QoSLivelinessPolicy.SYSTEM_DEFAULT,
    # liveliness_lease_duration=Duration(seconds=5),
)


STICK_DEADZONE = float(os.getenv("STICK_DEADZONE", "0.05"))
ARM_DEADZONE = float(os.getenv("ARM_DEADZONE", "0.2"))


class Headless(Node):
    # Every non-fixed joint defined in Arm's URDF
    # Used for JointState and JointJog messsages
    all_joint_names = [
        "axis_0_joint",
        "axis_1_joint",
        "axis_2_joint",
        "axis_3_joint",
        "wrist_yaw_joint",
        "wrist_roll_joint",
        "ef_gripper_left_joint",
    ]

    def __init__(self):
        # Initialize pygame first
        pygame.init()
        pygame.joystick.init()
        super().__init__("headless_node")

        ##################################################
        # Preamble

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

        ##################################################
        # Parameters

        self.declare_parameter("use_old_topics", True)
        self.use_old_topics = (
            self.get_parameter("use_old_topics").get_parameter_value().bool_value
        )

        self.declare_parameter("use_cmd_vel", False)
        self.using_cmd_vel = self.get_parameter("use_cmd_vel").value

        self.declare_parameter("use_bio", False)
        self.use_bio = self.get_parameter("use_bio").get_parameter_value().bool_value

        self.declare_parameter("use_arm_ik", False)
        self.use_arm_ik = (
            self.get_parameter("use_arm_ik").get_parameter_value().bool_value
        )
        # NOTE: only applicable if use_old_topics == True
        self.declare_parameter("use_new_arm_manual_scheme", True)
        self.use_new_arm_manual_scheme = (
            self.get_parameter("use_new_arm_manual_scheme")
            .get_parameter_value()
            .bool_value
        )

        # Check parameter validity
        if self.using_cmd_vel:
            self.get_logger().info("Using cmd_vel for core control")
            global CORE_MODE
            CORE_MODE = "twist"
        else:
            self.get_logger().info("Using astra_msgs/CoreControl for core control")

        if self.use_arm_ik and self.use_old_topics:
            self.get_logger().fatal("Old topics do not support arm IK control.")
            sys.exit(1)
        if not self.use_new_arm_manual_scheme and not self.use_old_topics:
            self.get_logger().warn(
                f"New arm manual does not support old control scheme. Defaulting to new scheme."
            )

        self.ctrl_mode = "core"  # Start in core mode
        self.core_brake_mode = False
        self.core_max_duty = 0.5  # Default max duty cycle (walking speed)

        ##################################################
        # Old Topics

        if self.use_old_topics:
            self.core_publisher = self.create_publisher(CoreControl, "/core/control", 2)
            self.arm_publisher = self.create_publisher(
                ArmManual, "/arm/control/manual", 2
            )
            self.bio_publisher = self.create_publisher(BioControl, "/bio/control", 2)

        ##################################################
        # New Topics

        if not self.use_old_topics:
            self.core_twist_pub_ = self.create_publisher(
                Twist, "/core/twist", qos_profile=control_qos
            )
            self.core_cmd_vel_pub_ = self.create_publisher(
                TwistStamped, "/diff_controller/cmd_vel", qos_profile=control_qos
            )
            self.core_state_pub_ = self.create_publisher(
                CoreCtrlState, "/core/control/state", qos_profile=control_qos
            )

            self.arm_manual_pub_ = self.create_publisher(
                JointJog, "/arm/manual_new", qos_profile=control_qos
            )

            self.arm_ik_twist_publisher = self.create_publisher(
                TwistStamped, "/servo_node/delta_twist_cmds", qos_profile=control_qos
            )
            self.arm_ik_jointjog_publisher = self.create_publisher(
                JointJog, "/servo_node/delta_joint_cmds", qos_profile=control_qos
            )

            # TODO: add new bio topics

        ##################################################
        # Timers

        self.create_timer(0.1, self.send_controls)

        ##################################################
        # Services

        # If using IK control, we have to "start" the servo node to enable it to accept commands
        self.servo_start_client = None
        if self.use_arm_ik:
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

        # Rumble when node is ready (returns False if rumble not supported)
        self.gamepad.rumble(0.7, 0.8, 150)

        # Added so you can tell when it starts running after changing the constant logging to debug from info
        self.get_logger().info("Defaulting to Core mode. Ready.")

    def stop_all(self):
        if self.use_old_topics:
            self.core_publisher.publish(CORE_STOP_MSG)
            self.arm_publisher.publish(ARM_STOP_MSG)
            self.bio_publisher.publish(BIO_STOP_MSG)
        else:
            self.core_twist_pub_.publish(CORE_STOP_TWIST_MSG)
            if self.use_arm_ik:
                self.arm_ik_twist_publisher.publish(self.arm_ik_twist_stop_msg())
            else:
                self.arm_manual_pub_.publish(self.arm_manual_stop_msg())
            # TODO: add bio here after implementing new topics

    def send_controls(self):
        """Read the gamepad state and publish control messages"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        # Check if controller is still connected
        if pygame.joystick.get_count() != self.num_gamepads:
            print("Gamepad disconnected. Exiting...")
            # Stop the rover if controller disconnected
            self.stop_all()
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
            self.stop_all()
            self.gamepad.rumble(0.6, 0.7, 75)
            self.ctrl_mode = new_ctrl_mode
            self.get_logger().info(f"Switched to {self.ctrl_mode} control mode")
            if self.ctrl_mode == "arm" and self.use_bio:
                self.get_logger().warning("NOTE: Using bio instead of arm.")

        # Actually send the controls
        if self.ctrl_mode == "core":
            self.send_core()
            if self.use_old_topics:
                if self.use_bio:
                    self.bio_publisher.publish(BIO_STOP_MSG)
                else:
                    self.arm_publisher.publish(ARM_STOP_MSG)
            # New topics shouldn't need to constantly send zeroes imo
        else:
            if self.use_bio:
                self.send_bio()
            else:
                self.send_arm()
            if self.use_old_topics:
                self.core_publisher.publish(CORE_STOP_MSG)
            # Ditto

    def send_core(self):
        # Collect controller state
        left_stick_x = stick_deadzone(self.gamepad.get_axis(0))
        left_stick_y = stick_deadzone(self.gamepad.get_axis(1))
        left_trigger = stick_deadzone(self.gamepad.get_axis(2))
        right_stick_x = stick_deadzone(self.gamepad.get_axis(3))
        right_stick_y = stick_deadzone(self.gamepad.get_axis(4))
        right_trigger = stick_deadzone(self.gamepad.get_axis(5))
        button_a = self.gamepad.get_button(0)
        button_b = self.gamepad.get_button(1)
        button_x = self.gamepad.get_button(2)
        button_y = self.gamepad.get_button(3)
        left_bumper = self.gamepad.get_button(4)
        right_bumper = self.gamepad.get_button(5)
        dpad_input = self.gamepad.get_hat(0)

        if self.use_old_topics:
            input = CoreControl()
            input.max_speed = 90

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

        else:  # New topics
            twist = Twist()

            # Forward/back and Turn
            twist.linear.x = -1.0 * left_stick_y
            twist.angular.z = -1.0 * copysign(
                right_stick_x**2, right_stick_x
            )  # Exponent for finer control (curve)

            if self.using_cmd_vel:
                twist.linear.x *= 1.5
                twist.angular.z *= 0.5

            # Publish
            if not self.using_cmd_vel:
                self.core_twist_pub_.publish(twist)
            else:
                header = Header(stamp=self.get_clock().now().to_msg())
                self.core_cmd_vel_pub_.publish(TwistStamped(header=header, twist=twist))
            self.get_logger().debug(
                f"[Core Ctrl] Linear: {round(twist.linear.x, 2)}, Angular: {round(twist.angular.z, 2)}"
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

    def send_arm(self):
        # Collect controller state
        left_stick_x = stick_deadzone(self.gamepad.get_axis(0))
        left_stick_y = stick_deadzone(self.gamepad.get_axis(1))
        left_trigger = stick_deadzone(self.gamepad.get_axis(2))
        right_stick_x = stick_deadzone(self.gamepad.get_axis(3))
        right_stick_y = stick_deadzone(self.gamepad.get_axis(4))
        right_trigger = stick_deadzone(self.gamepad.get_axis(5))
        button_a = self.gamepad.get_button(0)
        button_b = self.gamepad.get_button(1)
        button_x = self.gamepad.get_button(2)
        button_y = self.gamepad.get_button(3)
        left_bumper = self.gamepad.get_button(4)
        right_bumper = self.gamepad.get_button(5)
        dpad_input = self.gamepad.get_hat(0)

        # OLD MANUAL
        # ==========

        if not self.use_arm_ik and self.use_old_topics:
            arm_input = ArmManual()

            # OLD ARM MANUAL CONTROL SCHEME
            if not self.use_new_arm_manual_scheme:
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

            # NEW ARM MANUAL CONTROL SCHEME
            if self.use_new_arm_manual_scheme:
                # Right stick: EF yaw and axis 3
                # Left stick: axis 1 and 2
                # D-pad: axis 0 and _
                # Triggers: EF grippers
                # Bumpers: EF roll
                # A: brake
                # B: linear actuator in
                # X: _
                # Y: linear actuator out

                # Right stick: EF yaw and axis 3
                arm_input.effector_yaw = stick_to_arm_direction(right_stick_x)
                arm_input.axis3 = -1 * stick_to_arm_direction(right_stick_y)

                # Left stick: axis 1 and 2
                arm_input.axis1 = stick_to_arm_direction(left_stick_x)
                arm_input.axis2 = -1 * stick_to_arm_direction(left_stick_y)

                # D-pad: axis 0 and _
                arm_input.axis0 = int(dpad_input[0])

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
                if button_y and not button_b:
                    arm_input.linear_actuator = 1
                elif button_b and not button_y:
                    arm_input.linear_actuator = -1
                else:
                    arm_input.linear_actuator = 0

            self.arm_publisher.publish(arm_input)

        # NEW MANUAL
        # ==========

        elif not self.use_arm_ik and not self.use_old_topics:
            arm_input = JointJog()
            arm_input.header.frame_id = "base_link"
            arm_input.header.stamp = self.get_clock().now().to_msg()
            arm_input.joint_names = self.all_joint_names
            arm_input.velocities = [0.0] * len(self.all_joint_names)

            # Right stick: EF yaw and axis 3
            # Left stick: axis 1 and 2
            # D-pad: axis 0 and _
            # Triggers: EF grippers
            # Bumpers: EF roll
            # A: brake
            # B: linear actuator in
            # X: _
            # Y: linear actuator out

            # Right stick: EF yaw and axis 3
            arm_input.velocities[self.all_joint_names.index("wrist_yaw_joint")] = float(
                stick_to_arm_direction(right_stick_x)
            )
            arm_input.velocities[self.all_joint_names.index("axis_3_joint")] = float(
                stick_to_arm_direction(right_stick_y)
            )

            # Left stick: axis 1 and 2
            arm_input.velocities[self.all_joint_names.index("axis_1_joint")] = float(
                stick_to_arm_direction(left_stick_x)
            )
            arm_input.velocities[self.all_joint_names.index("axis_2_joint")] = float(
                stick_to_arm_direction(left_stick_y)
            )

            # D-pad: axis 0 and _
            arm_input.velocities[self.all_joint_names.index("axis_0_joint")] = float(
                dpad_input[0]
            )

            # Triggers: EF Grippers
            if left_trigger > 0 and right_trigger > 0:
                arm_input.velocities[
                    self.all_joint_names.index("ef_gripper_left_joint")
                ] = 0.0
            elif left_trigger > 0:
                arm_input.velocities[
                    self.all_joint_names.index("ef_gripper_left_joint")
                ] = -1.0
            elif right_trigger > 0:
                arm_input.velocities[
                    self.all_joint_names.index("ef_gripper_left_joint")
                ] = 1.0

            # Bumpers: EF roll
            arm_input.velocities[self.all_joint_names.index("wrist_roll_joint")] = (
                right_bumper - left_bumper
            )

            # A: brake
            # TODO: Brake mode

            # Y: linear actuator
            # TODO: linear actuator

            self.arm_manual_pub_.publish(arm_input)

        # IK (ONLY NEW)
        # =============

        elif self.use_arm_ik:
            arm_twist = TwistStamped()
            arm_twist.header.frame_id = "base_link"
            arm_twist.header.stamp = self.get_clock().now().to_msg()
            arm_jointjog = JointJog()
            arm_jointjog.header.frame_id = "base_link"
            arm_jointjog.header.stamp = self.get_clock().now().to_msg()

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
                arm_jointjog.joint_names.append("ef_gripper_left_joint")  # type: ignore
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

    def send_bio(self):
        # Collect controller state
        left_stick_x = stick_deadzone(self.gamepad.get_axis(0))
        left_stick_y = stick_deadzone(self.gamepad.get_axis(1))
        left_trigger = stick_deadzone(self.gamepad.get_axis(2))
        right_stick_x = stick_deadzone(self.gamepad.get_axis(3))
        right_stick_y = stick_deadzone(self.gamepad.get_axis(4))
        right_trigger = stick_deadzone(self.gamepad.get_axis(5))
        button_a = self.gamepad.get_button(0)
        button_b = self.gamepad.get_button(1)
        button_x = self.gamepad.get_button(2)
        button_y = self.gamepad.get_button(3)
        left_bumper = self.gamepad.get_button(4)
        right_bumper = self.gamepad.get_button(5)
        dpad_input = self.gamepad.get_hat(0)

        if self.use_old_topics:
            bio_input = BioControl(
                bio_arm=int(left_stick_y * -100),
                drill_arm=int(round(right_stick_y) * -100),
            )

            # Drill motor (FAERIE)
            if left_trigger > 0 or right_trigger > 0:
                bio_input.drill = int(
                    30 * (right_trigger - left_trigger)
                )  # Max duty cycle 30%

            self.bio_publisher.publish(bio_input)

        else:
            pass  # TODO: implement new bio control topics

    def arm_manual_stop_msg(self):
        return JointJog(
            header=Header(frame_id="base_link", stamp=self.get_clock().now().to_msg()),
            joint_names=self.all_joint_names,
            velocities=[0.0] * len(self.all_joint_names),
        )

    def arm_ik_twist_stop_msg(self):
        return TwistStamped(
            header=Header(frame_id="base_link", stamp=self.get_clock().now().to_msg())
        )


def stick_deadzone(value: float, threshold=STICK_DEADZONE) -> float:
    """Apply a deadzone to a joystick input so the motors don't sound angry"""
    if abs(value) < threshold:
        return 0
    return value


def stick_to_arm_direction(value: float, threshold=ARM_DEADZONE) -> int:
    """Apply a larger deadzone to a stick input and make digital/binary instead of analog"""
    if abs(value) < threshold:
        return 0
    return int(copysign(1, value))


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


def exit_handler(signum, frame):
    print("Caught SIGTERM. Exiting...")
    rclpy.try_shutdown()
    sys.exit(0)


def main(args=None):
    try:
        rclpy.init(args=args)

        # Catch termination signals and exit cleanly
        signal.signal(signal.SIGTERM, exit_handler)

        node = Headless()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Caught shutdown signal. Exiting...")
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
