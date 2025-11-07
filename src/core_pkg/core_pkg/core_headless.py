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
from astra_msgs.msg import CoreControl


os.environ["SDL_VIDEODRIVER"] = "dummy"  # Prevents pygame from trying to open a display
os.environ["SDL_AUDIODRIVER"] = (
    "dummy"  # Force pygame to use a dummy audio driver before pygame.init()
)


max_speed = 90  # Max speed as a duty cycle percentage (1-100)


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
        print(f"Gamepad Found: {self.gamepad.get_name()}")

        # Now initialize the ROS2 node
        super().__init__("core_headless")
        self.create_timer(0.15, self.send_controls)
        self.publisher = self.create_publisher(CoreControl, "/core/control", 10)
        self.lastMsg = (
            String()
        )  # Used to ignore sending controls repeatedly when they do not change

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
            input = CoreControl()
            input.left_stick = 0
            input.right_stick = 0
            input.max_speed = 0
            self.publisher.publish(input)
            self.get_logger().info("Final stop command sent. Shutting down.")
            # Clean up
            pygame.quit()
            sys.exit(0)

        input = CoreControl()
        input.max_speed = max_speed
        input.right_stick = -1 * round(self.gamepad.get_axis(4), 2)  # right y-axis
        if self.gamepad.get_axis(5) > 0:
            input.left_stick = input.right_stick
        else:
            input.left_stick = -1 * round(self.gamepad.get_axis(1), 2)  # left y-axis

        output = f"L: {input.left_stick}, R: {input.right_stick}, M: {max_speed}"
        self.get_logger().info(f"[Ctrl] {output}")
        self.publisher.publish(input)


def main(args=None):
    rclpy.init(args=args)
    node = Headless()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
