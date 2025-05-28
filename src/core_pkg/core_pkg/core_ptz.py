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
        self.declare_parameter('camera_ip', '192.168.1.9')
        self.declare_parameter('camera_port', 37260)
        
        # Get parameters
        self.camera_ip = self.get_parameter('camera_ip').value
        self.camera_port = self.get_parameter('camera_port').value
        
        self.get_logger().info(f"PTZ camera IP: {self.camera_ip} Port: {self.camera_port}")
        
        # Create a camera instance
        self.camera = None
        self.camera_connected = False # This flag is still managed but not used to gate commands
        self.loop = None
        self.thread_pool = None
        
        # Track current zoom level and zoom state
        self.current_zoom_level = 1.0
        self.max_zoom_level = 6.0  # A8 mini max zoom
        self.zoom_step = 0.2       # Zoom step per command
        self.zooming = 0           # Current zoom direction: -1=out, 0=stop, 1=in
        self.zoom_timer = None
        
        # Create publishers
        self.feedback_pub = self.create_publisher(PtzFeedback, '/ptz/feedback', 10)
        self.debug_pub = self.create_publisher(String, '/ptz/debug', 10)
        
        # Create subscribers
        self.control_sub = self.create_subscription(
            PtzControl, '/ptz/control', self.handle_control_command, 10)
            
        # Create timers
        self.connection_timer = self.create_timer(5.0, self.check_camera_connection)
        self.last_data_time = time.time()
        self.health_check_timer = self.create_timer(2.0, self.check_camera_health)
        
        # Create feedback message
        self.feedback_msg = PtzFeedback()
        self.feedback_msg.connected = False # This will reflect the actual connection state
        self.feedback_msg.error_msg = "Initializing"
        
        # Flags for async operations
        self.shutdown_requested = False
        
        # Set up asyncio event loop in a separate thread
        self.thread_pool = ThreadPoolExecutor(max_workers=1)
        self.loop = asyncio.new_event_loop()
        
        # Connect to camera on startup
        self.connect_task = self.thread_pool.submit(
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
            
            self.publish_debug("Camera connected successfully")
            
        except Exception as e:
            self.camera_connected = False 
            self.feedback_msg.connected = False
            self.feedback_msg.error_msg = f"Connection error: {str(e)}"
            self.publish_debug(f"Camera connection failed: {str(e)}")

    def camera_data_callback(self, cmd_id, data):
        """Handle data received from the camera."""
        # Update last_data_time regardless of self.camera_connected,
        # as data might arrive during a brief reconnect window.
        self.last_data_time = time.time()
        if self.camera_connected: # Only process for feedback if we believe we are connected
            if cmd_id == CommandID.ATTITUDE_DATA_RESPONSE and isinstance(data, AttitudeData):
                self.feedback_msg.yaw = data.yaw
                self.feedback_msg.pitch = data.pitch
                self.feedback_msg.roll = data.roll
                self.feedback_msg.yaw_velocity = data.yaw_velocity
                self.feedback_msg.pitch_velocity = data.pitch_velocity
                self.feedback_msg.roll_velocity = data.roll_velocity
                self.feedback_pub.publish(self.feedback_msg)
            else:
                debug_str = ""
                if isinstance(cmd_id, CommandID):
                    debug_str = f"Camera data: CMD_ID={cmd_id.name}, Data="
                else:
                    debug_str = f"Camera data: CMD_ID={cmd_id}, Data="
                
                if isinstance(data, bytes):
                    debug_str += data.hex()
                else:
                    debug_str += str(data)
                self.get_logger().debug(debug_str)

    def check_camera_connection(self):
        """Periodically check camera connection and attempt to reconnect if needed."""
        if not self.camera_connected and not self.shutdown_requested:
            self.publish_debug("Attempting to reconnect to camera...")
            if self.camera:
                try:
                    if self.camera.is_connected: # SDK's internal connection state
                         self.run_async_func(self.camera.disconnect())
                except Exception as e:
                    self.get_logger().debug(f"Error during pre-reconnect disconnect: {e}")
                # self.camera = None # Don't nullify here, connect_to_camera will re-assign or create new
                
            self.connect_task = self.thread_pool.submit(
                self.run_async_func, self.connect_to_camera()
            )

    def check_camera_health(self):
        """Check if we're still receiving data from the camera"""
        if self.camera_connected: # Only check health if we think we are connected
            time_since_last_data = time.time() - self.last_data_time
            if time_since_last_data > 5.0:
                self.publish_debug(f"No camera data for {time_since_last_data:.1f}s, marking as disconnected.")
                self.camera_connected = False
                self.feedback_msg.connected = False
                self.feedback_msg.error_msg = "Connection stale (no data)"
                self.feedback_pub.publish(self.feedback_msg)

    def handle_control_command(self, msg):
        """Handle incoming control commands."""
        # Removed: if not self.camera_connected
        if not self.camera: # Still check if camera object exists
            self.get_logger().warning("Camera object not initialized, ignoring control command")
            return
            
        self.thread_pool.submit(
            self.run_async_func,
            self.process_control_command(msg)
        )

    async def process_control_command(self, msg):
        """Process and send the control command to the camera."""
        if not self.camera: 
            self.get_logger().error("Process control command called but camera object is None.")
            return
        try:
            # The SDK's send_... methods will raise RuntimeError if not connected.
            # This try-except block will catch those.
            if msg.reset:
                self.get_logger().info("Attempting to reset camera to center position")
                await self.camera.send_attitude_angles_command(0.0, 0.0)
                
                # Also reset zoom to 1x when resetting camera
                self.current_zoom_level = 1.0
                await self.camera.send_absolute_zoom_command(1.0)
                return
                
            if msg.control_mode == 0:
                turn_yaw = max(-100, min(100, int(msg.turn_yaw)))
                turn_pitch = max(-100, min(100, int(msg.turn_pitch)))
                self.get_logger().debug(f"Attempting rotation: yaw_speed={turn_yaw}, pitch_speed={turn_pitch}")
                await self.camera.send_rotation_command(turn_yaw, turn_pitch)
                
            elif msg.control_mode == 1:
                yaw = max(-135.0, min(135.0, msg.yaw))
                pitch = max(-90.0, min(90.0, msg.pitch))
                self.get_logger().debug(f"Attempting absolute angles: yaw={yaw}, pitch={pitch}")
                await self.camera.send_attitude_angles_command(yaw, pitch)
                
            elif msg.control_mode == 2:
                axis = SingleAxis.YAW if msg.axis_id == 0 else SingleAxis.PITCH
                angle = msg.angle
                self.get_logger().debug(f"Attempting single axis: axis={axis.name}, angle={angle}")
                await self.camera.send_single_axis_attitude_command(angle, axis)

            elif msg.control_mode == 3:
                # Instead of absolute zoom, interpret zoom_level as zoom direction
                zoom_direction = int(msg.zoom_level) if abs(msg.zoom_level) >= 0.5 else 0
                zoom_direction = max(-1, min(1, zoom_direction))  # Restrict to -1, 0, 1
                
                if zoom_direction != self.zooming:
                    self.zooming = zoom_direction
                    self.get_logger().debug(f"Zoom direction changed to {zoom_direction}")
                    
                    if zoom_direction == 0:
                        # Stop zooming
                        self.get_logger().debug(f"Stopping zoom at level {self.current_zoom_level:.1f}x")
                    else:
                        # Start zooming in the specified direction
                        await self.perform_zoom(zoom_direction)
                
            if hasattr(msg, 'stream_type') and hasattr(msg, 'stream_freq'):
                if msg.stream_type > 0 and msg.stream_freq >= 0: 
                    try:
                        stream_type = DataStreamType(msg.stream_type)
                        stream_freq = DataStreamFrequency(msg.stream_freq)
                        self.get_logger().info(
                            f"Attempting to set data stream: type={stream_type.name}, freq={stream_freq.name}"
                        )
                        await self.camera.send_data_stream_request(stream_type, stream_freq)
                    except ValueError:
                        self.get_logger().error("Invalid stream type or frequency values in control message")
            
        except RuntimeError as e: # Catch SDK's "not connected" errors
            self.get_logger().warning(f"SDK command failed (likely not connected): {e}")
            # self.camera_connected will be updated by health/connection checks
        except Exception as e:
            self.get_logger().error(f"Error processing control command: {e}")
            self.feedback_msg.error_msg = f"Control error: {str(e)}"
            self.feedback_pub.publish(self.feedback_msg) # Publish for other errors

    async def perform_zoom(self, direction):
        """Perform a zoom operation in the specified direction."""
        if not self.camera or not self.camera.is_connected:
            return
            
        if direction == 0:
            return
            
        # Calculate new zoom level
        new_zoom_level = self.current_zoom_level + (direction * self.zoom_step)
        
        # Clamp to valid range
        new_zoom_level = max(1.0, min(self.max_zoom_level, new_zoom_level))
        
        # Skip if no change (already at min/max)
        if new_zoom_level == self.current_zoom_level:
            if direction > 0:
                self.get_logger().debug(f"Already at maximum zoom level ({self.max_zoom_level:.1f}x)")
            else:
                self.get_logger().debug(f"Already at minimum zoom level (1.0x)")
            return
            
        # Set the new zoom level
        self.get_logger().debug(f"Zooming from {self.current_zoom_level:.1f}x to {new_zoom_level:.1f}x")
        self.current_zoom_level = new_zoom_level
        
        try:
            await self.camera.send_absolute_zoom_command(new_zoom_level)
            
            # If still zooming in same direction, schedule another zoom step
            if self.zooming == direction:
                # Schedule a new zoom step after a short delay if we're not at the limits
                if (direction > 0 and new_zoom_level < self.max_zoom_level) or \
                   (direction < 0 and new_zoom_level > 1.0):
                    # Use create_timer for a non-blocking delay
                    await asyncio.sleep(0.2)  # 200ms delay between zoom steps
                    if self.zooming == direction:  # Check if direction hasn't changed
                        await self.perform_zoom(direction)
        except Exception as e:
            self.get_logger().error(f"Error during zoom operation: {e}")

    def publish_debug(self, message_text):
        """Publish debug message."""
        msg = String()
        msg.data = f"[{self.get_clock().now().nanoseconds / 1e9:.2f}] PTZ Node: {message_text}"
        self.debug_pub.publish(msg)
        self.get_logger().info(message_text) 

    def run_async_func(self, coro):
        """Run an async function in the event loop."""
        if self.loop and self.loop.is_running():
            try:
                return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout=5.0) # Added timeout
            except asyncio.TimeoutError:
                self.get_logger().warning(f"Async function {coro.__name__} timed out.")
                return None
            except Exception as e:
                self.get_logger().error(f"Exception in run_async_func for {coro.__name__}: {e}")
                return None
        else:
            self.get_logger().warning("Asyncio loop not running, cannot execute coroutine.")
            return None

    async def shutdown_node_async(self):
        """Perform clean shutdown of camera connection."""
        self.shutdown_requested = True
        self.get_logger().info("Async shutdown initiated...")
        if self.camera and self.camera.is_connected: # Check SDK's connection state
            try:
                self.get_logger().info("Disabling data stream...")
                await self.camera.send_data_stream_request(
                    DataStreamType.ATTITUDE_DATA, 
                    DataStreamFrequency.DISABLE
                )
                await asyncio.sleep(0.1) 
                
                self.get_logger().info("Disconnecting from camera...")
                await self.camera.disconnect()
                self.get_logger().info("Disconnected from camera successfully.")
                
            except Exception as e:
                self.get_logger().error(f"Error during camera shutdown: {e}")
        self.camera_connected = False # Update node's flag
        self.feedback_msg.connected = False
        self.feedback_msg.error_msg = "Shutting down"

    def cleanup(self):
        """Clean up resources."""
        self.get_logger().info("PTZ node cleanup initiated.")
        self.shutdown_requested = True 

        if self.connection_timer:
            self.connection_timer.cancel()
        if self.health_check_timer:
            self.health_check_timer.cancel()

        if self.loop and self.thread_pool:
            if self.loop.is_running():
                try:
                    future = asyncio.run_coroutine_threadsafe(self.shutdown_node_async(), self.loop)
                    future.result(timeout=5) 
                except Exception as e:
                    self.get_logger().error(f"Error during async shutdown in cleanup: {e}")
            
            self.get_logger().info("Shutting down thread pool executor...")
            self.thread_pool.shutdown(wait=True)
            
            if self.loop.is_running():
                self.get_logger().info("Stopping asyncio event loop...")
                self.loop.call_soon_threadsafe(self.loop.stop)
            
            self.get_logger().info("PTZ node resources cleaned up.")
        else:
            self.get_logger().warning("Loop or thread_pool not initialized, skipping parts of cleanup.")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    ptz_node = PtzNode()
    
    asyncio_thread = None
    if ptz_node.loop: 
        def run_event_loop(loop):
            asyncio.set_event_loop(loop)
            try:
                loop.run_forever()
            finally:
                # This ensures the loop is closed when the thread running it exits.
                # This is important if run_forever() exits due to loop.stop()
                # or an unhandled exception within a task scheduled on the loop.
                if not loop.is_closed():
                    loop.close()
        
        asyncio_thread = threading.Thread(
            target=run_event_loop, 
            args=(ptz_node.loop,),
            daemon=True 
        )
        asyncio_thread.start()
    
    try:
        rclpy.spin(ptz_node)
    except KeyboardInterrupt:
        ptz_node.get_logger().info("KeyboardInterrupt received, shutting down...")
    except SystemExit:
        ptz_node.get_logger().info("SystemExit received, shutting down...")
    finally:
        ptz_node.get_logger().info("Initiating final cleanup...")
        ptz_node.cleanup() # This will stop the loop and shutdown the executor
        
        if asyncio_thread and asyncio_thread.is_alive():
             # The loop should have been stopped by cleanup. We just join the thread.
             ptz_node.get_logger().info("Waiting for asyncio thread to join...")
             asyncio_thread.join(timeout=5) 
             if asyncio_thread.is_alive():
                 ptz_node.get_logger().warning("Asyncio thread did not join cleanly.")
        
        rclpy.shutdown()
        ptz_node.get_logger().info("ROS shutdown complete.")


if __name__ == '__main__':
    main()
