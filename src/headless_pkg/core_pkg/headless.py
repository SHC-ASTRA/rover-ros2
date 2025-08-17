import rclpy
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


os.environ["SDL_VIDEODRIVER"] = "dummy"  # Prevents pygame from trying to open a display
os.environ["SDL_AUDIODRIVER"] = "dummy"  # Force pygame to use a dummy audio driver before pygame.init()


max_speed = 90 #Max speed as a duty cycle percentage (1-100)

core_stop_msg = CoreControl()
core_stop_msg.left_stick = 0
core_stop_msg.right_stick = 0
core_stop_msg.max_speed = 0

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
            self.get_logger().info("Final stop command sent. Shutting down.")
            # Clean up
            pygame.quit()
            sys.exit(0)


        global ctrl_mode

        dpad_input = self.gamepad.get_hat(0)
        if dpad_input[1] == 1:
            ctrl_mode = "arm"
        elif dpad_input[1] == -1:
            ctrl_mode = "core"


        if ctrl_mode == "core":
            input = CoreControl()
            input.max_speed = max_speed
            input.right_stick = -1 * round(self.gamepad.get_axis(4), 2)  # right y-axis
            if self.gamepad.get_axis(5) > 0:
                input.left_stick = input.right_stick
            else:
                input.left_stick = -1 * round(self.gamepad.get_axis(1), 2)  # left y-axis

            output = f'L: {input.left_stick}, R: {input.right_stick}, M: {max_speed}'
            self.get_logger().info(f"[Ctrl] {output}")
            self.core_publisher.publish(input)
            self.arm_publisher.publish(arm_stop_msg)

        if ctrl_mode == "arm":
            input = ArmManual()

            # Triggers for gripper control
            if self.gamepad.get_axis(2) > 0:#left trigger
                input.gripper = -1
            elif self.gamepad.get_axis(5) > 0:#right trigger
                input.gripper = 1

            if self.gamepad.get_button(5):#right bumper, control effector

                # Left stick X-axis for effector yaw
                if self.gamepad.get_axis(0) > 0:
                    input.effector_yaw = 1
                elif self.gamepad.get_axis(0) < 0:
                    input.effector_yaw = -1

                # Right stick X-axis for effector roll
                if self.gamepad.get_axis(3) > 0:
                    input.effector_roll = 1
                elif self.gamepad.get_axis(3) < 0:
                    input.effector_roll = -1

            else:   # Control arm axis
                dpad_input = self.gamepad.get_hat(0)
                input.axis0 = 0
                if dpad_input[0] == 1:
                    input.axis0 = 1
                elif dpad_input[0] == -1:
                    input.axis0 = -1

                if self.gamepad.get_axis(0) > .15 or self.gamepad.get_axis(0) < -.15:
                    input.axis1 = round(self.gamepad.get_axis(0))

                if self.gamepad.get_axis(1) > .15 or self.gamepad.get_axis(1) < -.15:
                    input.axis2 = -1 * round(self.gamepad.get_axis(1))

                if self.gamepad.get_axis(4) > .15 or self.gamepad.get_axis(4) < -.15:
                    input.axis3 = -1 * round(self.gamepad.get_axis(4))

            self.arm_publisher.publish(input)
            self.core_publisher.publish(core_stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Headless()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
