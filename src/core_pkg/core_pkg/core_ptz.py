#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
from concurrent.futures import ThreadPoolExecutor
import signal
import sys
import threading
import time

from std_msgs.msg import String
from ros2_interfaces_pkg.msg import PtzControl, PtzFeedback

# Import the SIYI SDK
from core_pkg.siyi_sdk import (
    SiyiGimbalCamera,
    CommandID,
    DataStreamType,
    DataStreamFrequency,
    SingleAxis,
    AttitudeData
)

class PtzNode(Node):
    def __init__(self):
        # Initialize node with name
        super().__init__("core_ptz")

        # Declare parameters
        self.declare_parameter('camera_ip', '192.168.1.17')
        self.declare_parameter('camera_port', 37260)
        #self.declare_parameter('launch_mode', 'core')
        
        # Get parameters
        self.camera_ip = self.get_parameter('camera_ip').value
        self.camera_port = self.get_parameter('camera_port').value
        #self.launch_mode = self.get_parameter('launch_mode').value
        
        #self.get_logger().info(f"PTZ mode: {self.launch_mode}")
        self.get_logger().info(f"PTZ camera IP: {self.camera_ip} Port: {self.camera_port}")
        
        # Create a camera instance
        self.camera = None
        self.camera_connected = False
        self.loop = None
        self.executor = None
        
        # Create publishers
        self.feedback_pub = self.create_publisher(PtzFeedback, '/ptz/feedback', 10)
        self.debug_pub = self.create_publisher(String, '/ptz/debug', 10)
        
        # Create subscribers
        self.control_sub = self.create_subscription(
            PtzControl, '/ptz/control', self.handle_control_command, 10)
            
        # Create timers
        self.connection_timer = self.create_timer(5.0, self.check_camera_connection)
        
        # Create feedback message
        self.feedback_msg = PtzFeedback()
        self.feedback_msg.connected = False
        self.feedback_msg.error_msg = "Initializing"
        
        # Flags for async operations
        self.shutdown_requested = False
        
        # Set up asyncio event loop in a separate thread
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.loop = asyncio.new_event_loop()
        
        # Connect to camera on startup
        self.connect_task = self.executor.submit(
            self.run_async_func, self.connect_to_camera()
        )

    async def connect_to_camera(self):
        """Connect to the SIYI camera."""
        try:
            # Create a new camera instance
            self.camera = SiyiGimbalCamera(ip=self.camera_ip, port=self.camera_port)
            
            # Connect to the camera
            await self.camera.connect()
            
            # Set up data callback
            self.camera.set_data_callback(self.camera_data_callback)
            
            # Request attitude data stream
            await self.camera.send_data_stream_request(
                DataStreamType.ATTITUDE_DATA,
                DataStreamFrequency.HZ_10
            )
            
            # Update connection status
            self.camera_connected = True
            self.feedback_msg.connected = True
            self.feedback_msg.error_msg = ""
            
            self.get_logger().info("Connected to PTZ camera")
            self.publish_debug("Camera connected successfully")
            
        except Exception as e:
            self.camera_connected = False
            self.feedback_msg.connected = False
            self.feedback_msg.error_msg = f"Connection error: {str(e)}"
            self.get_logger().error(f"Failed to connect to camera: {e}")
            self.publish_debug(f"Camera connection failed: {str(e)}")

    def camera_data_callback(self, cmd_id, data):
        """Handle data received from the camera."""
        if cmd_id == CommandID.ATTITUDE_DATA_RESPONSE and isinstance(data, AttitudeData):
            # Update feedback message with attitude data
            self.feedback_msg.yaw = data.yaw
            self.feedback_msg.pitch = data.pitch
            self.feedback_msg.roll = data.roll
            self.feedback_msg.yaw_velocity = data.yaw_velocity
            self.feedback_msg.pitch_velocity = data.pitch_velocity
            self.feedback_msg.roll_velocity = data.roll_velocity
            
            # Publish feedback
            self.feedback_pub.publish(self.feedback_msg)
        else:
            # Log other data for debugging
            debug_str = f"Camera data: CMD_ID={cmd_id}, Data={data}"
            self.get_logger().debug(debug_str)

    def check_camera_connection(self):
        """Periodically check camera connection and attempt to reconnect if needed."""
        if not self.camera_connected and not self.shutdown_requested:
            self.get_logger().info("Attempting to reconnect to camera...")
            self.connect_task = self.executor.submit(
                self.run_async_func, self.connect_to_camera()
            )

    def handle_control_command(self, msg):
        """Handle incoming control commands."""
        if not self.camera_connected:
            self.get_logger().warning("Camera not connected, ignoring control command")
            return
            
        # Submit the control command to the async executor
        self.executor.submit(
            self.run_async_func,
            self.process_control_command(msg)
        )

    async def process_control_command(self, msg):
        """Process and send the control command to the camera."""
        try:
            # Check if reset command
            if msg.reset:
                self.get_logger().info("Resetting camera to center position")
                await self.camera.send_attitude_angles_command(0.0, 0.0)
                return
                
            # Process based on control mode
            if msg.control_mode == 0:  # Relative speed control
                # Clamp values between -100 and 100
                turn_yaw = max(-100, min(100, msg.turn_yaw))
                turn_pitch = max(-100, min(100, msg.turn_pitch))
                
                self.get_logger().debug(f"Sending rotation command: yaw={turn_yaw}, pitch={turn_pitch}")
                await self.camera.send_rotation_command(turn_yaw, turn_pitch)
                
            elif msg.control_mode == 1:  # Absolute angle control
                # Clamp angles to valid ranges
                yaw = max(-135.0, min(135.0, msg.yaw))
                pitch = max(-90.0, min(90.0, msg.pitch))
                
                self.get_logger().debug(f"Sending absolute angles: yaw={yaw}, pitch={pitch}")
                await self.camera.send_attitude_angles_command(yaw, pitch)
                
            elif msg.control_mode == 2:  # Single axis control
                axis = SingleAxis.YAW if msg.axis_id == 0 else SingleAxis.PITCH
                self.get_logger().debug(f"Sending single axis command: axis={axis.name}, angle={msg.angle}")
                await self.camera.send_single_axis_attitude_command(msg.angle, axis)
                
            # Handle data streaming configuration if requested
            if msg.stream_type > 0 and msg.stream_freq >= 0:
                try:
                    stream_type = DataStreamType(msg.stream_type)
                    stream_freq = DataStreamFrequency(msg.stream_freq)
                    
                    self.get_logger().info(
                        f"Setting data stream: type={stream_type.name}, freq={stream_freq.name}"
                    )
                    
                    await self.camera.send_data_stream_request(stream_type, stream_freq)
                    
                except ValueError:
                    self.get_logger().error("Invalid stream type or frequency values")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing control command: {e}")
            self.feedback_msg.error_msg = f"Control error: {str(e)}"
            self.feedback_pub.publish(self.feedback_msg)

    def publish_debug(self, message):
        """Publish debug message."""
        msg = String()
        msg.data = message
        self.debug_pub.publish(msg)

    def run_async_func(self, coro):
        """Run an async function in the event loop."""
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result()

    async def shutdown(self):
        """Perform clean shutdown."""
        self.shutdown_requested = True
        
        if self.camera and self.camera_connected:
            try:
                # Stop any data streams
                await self.camera.send_data_stream_request(
                    DataStreamType.ATTITUDE_DATA,
                    DataStreamFrequency.DISABLE
                )
                
                # Disconnect from camera
                await self.camera.disconnect()
                self.get_logger().info("Disconnected from camera")
                
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")

    def cleanup(self):
        """Clean up resources."""
        if self.loop and self.executor:
            # Schedule the shutdown coroutine
            try:
                self.run_async_func(self.shutdown())
            except Exception as e:
                self.get_logger().error(f"Error during cleanup: {e}")
            
            # Shut down the executor
            self.executor.shutdown()
            
            # Close the event loop
            self.loop.stop()
            self.loop.close()
            
            self.get_logger().info("PTZ node resources cleaned up")

def main(args=None):
    """Main function."""
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create the node
    ptz_node = PtzNode()
    
    # Create and start event loop for the camera
    def run_event_loop(loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()
    
    # Start the asyncio loop in a separate thread
    asyncio_thread = threading.Thread(
        target=run_event_loop, 
        args=(ptz_node.loop,),
        daemon=True
    )
    asyncio_thread.start()
    
    try:
        # Spin the node to process callbacks
        rclpy.spin(ptz_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        ptz_node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    # Register signal handler for clean shutdown
    signal.signal(signal.SIGINT, lambda signum, frame: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))
    main()

