import rclpy
from rclpy import qos
from rclpy.duration import Duration
from rclpy.node import Node

import pygame

import time

import serial
import sys
import threading
import glob
import os

import importlib
from std_msgs.msg import String
from ros2_interfaces_pkg.msg import CoreControl, ArmManual
from geometry_msgs.msg import Twist


os.environ["SDL_VIDEODRIVER"] = "dummy"  # Prevents pygame from trying to open a display
os.environ["SDL_AUDIODRIVER"] = "dummy"  # Force pygame to use a dummy audio driver before pygame.init()


max_speed = 90 #Max speed as a duty cycle percentage (1-100)

core_stop_msg = CoreControl()
core_stop_msg.left_stick = 0.0
core_stop_msg.right_stick = 0.0
core_stop_msg.max_speed = 0

core_stop_twist_msg = Twist()
core_stop_twist_msg.linear.x = 0.0
core_stop_twist_msg.linear.y = 0.0
core_stop_twist_msg.linear.z = 0.0
core_stop_twist_msg.angular.x = 0.0
core_stop_twist_msg.angular.y = 0.0
core_stop_twist_msg.angular.z = 0.0

arm_stop_msg = ArmManual()
arm_stop_msg.axis0 = 0
arm_stop_msg.axis1 = 0
arm_stop_msg.axis2 = 0
arm_stop_msg.axis3 = 0
arm_stop_msg.effector_roll = 0
arm_stop_msg.effector_yaw = 0
arm_stop_msg.gripper = 0
arm_stop_msg.linear_actuator = 0
arm_stop_msg.laser = 0

ctrl_mode = "core"
CORE_MODE = "duty"  # "twist" or "duty"

control_qos = qos.QoSProfile(
    history=qos.QoSHistoryPolicy.KEEP_LAST,
    depth=2,
    reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
    durability=qos.QoSDurabilityPolicy.VOLATILE,
    deadline=Duration(seconds=1),
    lifespan=Duration(nanoseconds=500_000_000),  # 500ms
    liveliness=qos.QoSLivelinessPolicy.SYSTEM_DEFAULT,
    liveliness_lease_duration=Duration(seconds=5)
)


class Headless(Node):
    def __init__(self):
        # Initialize pygame first
        pygame.init()
        pygame.joystick.init()
        
        # Wait for a gamepad to be connected
        self.gamepad = None
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
        print(f'Gamepad Found: {self.gamepad.get_name()}')
        
        # Now initialize the ROS2 node
        super().__init__("headless")
        self.create_timer(0.15, self.send_controls)
        self.core_publisher = self.create_publisher(CoreControl, '/core/control', 10)
        self.arm_publisher = self.create_publisher(ArmManual, '/arm/control/manual', 10)
        self.core_twist_pub_ = self.create_publisher(Twist, '/core/twist', qos_profile=control_qos)

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
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
        
        # Check if controller is still connected
        if pygame.joystick.get_count() == 0:
            print("Gamepad disconnected. Exiting...")
            # Send one last zero control message
            self.core_publisher.publish(core_stop_msg)
            self.arm_publisher.publish(arm_stop_msg)
            self.get_logger().info("Final stop commands sent. Shutting down.")
            # Clean up
            pygame.quit()
            sys.exit(0)


        global ctrl_mode

        # Check for control mode change
        dpad_input = self.gamepad.get_hat(0)
        if dpad_input[1] == 1:
            ctrl_mode = "arm"
        elif dpad_input[1] == -1:
            ctrl_mode = "core"


        # CORE
        if ctrl_mode == "core" and CORE_MODE == "duty":
            input = CoreControl()
            input.max_speed = max_speed

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
            output = f'L: {input.left_stick}, R: {input.right_stick}, M: {max_speed}'
            self.get_logger().info(f"[Ctrl] {output}")

            self.core_publisher.publish(input)
            self.arm_publisher.publish(arm_stop_msg)
        
        elif ctrl_mode == "core" and CORE_MODE == "twist":
            input = Twist()

            # Collect controller state
            left_stick_y = deadzone(self.gamepad.get_axis(1))
            right_stick_x = deadzone(self.gamepad.get_axis(3))


            # Forward/back
            input.linear.x = float(round(-1 * left_stick_y, 2))

            # Turn
            input.angular.z = float(round(right_stick_x, 2))


            # Debug
            output = f'Lin X: {input.linear.x}, Ang Z: {input.angular.z}'
            self.get_logger().info(f"[Ctrl] {output}")

            self.core_twist_pub_.publish(input)
            self.arm_publisher.publish(arm_stop_msg)


        # ARM
        if ctrl_mode == "arm":
            input = ArmManual()
            
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
                input.gripper = 0
            elif left_trigger > 0:
                input.gripper = -1
            elif right_trigger > 0:
                input.gripper = 1

            # Axis 0
            if dpad_input[0] == 1:
                input.axis0 = 1
            elif dpad_input[0] == -1:
                input.axis0 = -1


            if right_bumper:  # Control end effector

                # Effector yaw
                if left_stick_x > 0:
                    input.effector_yaw = 1
                elif left_stick_x < 0:
                    input.effector_yaw = -1

                # Effector roll
                if right_stick_x > 0:
                    input.effector_roll = 1
                elif right_stick_x < 0:
                    input.effector_roll = -1

            else:   # Control arm axis

                # Axis 1
                if abs(left_stick_x) > .15:
                    input.axis1 = round(left_stick_x)

                # Axis 2
                if abs(left_stick_y) > .15:
                    input.axis2 = -1 * round(left_stick_y)

                # Axis 3
                if abs(right_stick_y) > .15:
                    input.axis3 = -1 * round(right_stick_y)

            self.core_publisher.publish(core_stop_msg)
            self.arm_publisher.publish(input)


def deadzone(value: float, threshold=0.05) -> float:
    if abs(value) < threshold:
        return 0
    return value


def main(args=None):
    rclpy.init(args=args)
    node = Headless()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
