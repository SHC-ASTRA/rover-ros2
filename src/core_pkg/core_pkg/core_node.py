import rclpy
from rclpy.node import Node
from rclpy import qos
from rclpy.duration import Duration
from std_srvs.srv import Empty

import signal
import time
import atexit

import serial
import os
import sys
import threading
import glob

from std_msgs.msg import String
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, Twist
from ros2_interfaces_pkg.msg import CoreControl, CoreFeedback
from ros2_interfaces_pkg.msg import NewCoreFeedback, CoreCtrlState


serial_pub = None
thread = None

CORE_WHEELBASE = 0.836  # meters
CORE_WHEEL_RADIUS = 0.171  # meters
CORE_GEAR_RATIO = 100  # Clucky: 100:1, Testbed: 64:1

WALKING_SPEED = 0.5  # duty cycle
FAST_SPEED = 0.9  # duty cycle
SLOW_SPEED = 0.25  # duty cycle

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


class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("core_node")

        # Launch mode -- anchor vs core
        self.declare_parameter('launch_mode', 'core')
        self.launch_mode = self.get_parameter('launch_mode').value
        self.get_logger().info(f"Core launch_mode is: {self.launch_mode}")

        # Anchor
        if self.launch_mode == 'anchor':
            self.anchor_sub = self.create_subscription(String, '/anchor/core/feedback', self.anchor_feedback, 10)
            self.anchor_pub = self.create_publisher(String, '/anchor/relay', 10)

        # Controls
        self.control_sub = self.create_subscription(CoreControl, '/core/control', self.send_controls, 10)
        self.cmd_vel_sub_ = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_callback, qos_profile=control_qos)
        self.twist_man_sub_ = self.create_subscription(Twist, '/core/twist', self.twist_man_callback, qos_profile=control_qos)
        self.control_state_sub_ = self.create_subscription(CoreCtrlState, '/core/control/state', self.control_state_callback, qos_profile=control_qos)
        self.twist_speed_mode = 0  # -1 = slow, 0 = walk, 1 = fast

        # Feedback
        self.feedback_pub = self.create_publisher(CoreFeedback, '/core/feedback', 10)
        self.core_feedback = CoreFeedback()
        self.feedback_new_pub_ = self.create_publisher(NewCoreFeedback, '/core/feedback_new', 10)
        self.feedback_new_state = NewCoreFeedback()
        self.feedback_new_state.fl_motor.id = 1
        self.feedback_new_state.bl_motor.id = 2
        self.feedback_new_state.fr_motor.id = 3
        self.feedback_new_state.br_motor.id = 4
        self.telemetry_pub_timer = self.create_timer(1.0, self.publish_feedback)
        self.imu_pub_ = self.create_publisher(Imu, '/imu/data', 10)
        self.imu_state = Imu()
        self.gps_pub_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.gps_state = NavSatFix()

        # Debug
        self.debug_pub = self.create_publisher(String, '/core/debug', 10)
        self.ping_service = self.create_service(Empty, '/astra/core/ping', self.ping_callback)


        # Core (non-anchor) specific
        if self.launch_mode == 'core':
            # Loop through all serial devices on the computer to check for the MCU
            self.port = None
            ports = SerialRelay.list_serial_ports()
            for i in range(2):
                for port in ports:
                    try:
                        # connect and send a ping command
                        ser = serial.Serial(port, 115200, timeout=1)
                        #(f"Checking port {port}...")
                        ser.write(b"ping\n")
                        response = ser.read_until("\n")  # type: ignore

                        # if pong is in response, then we are talking with the MCU
                        if b"pong" in response:
                            self.port = port
                            self.get_logger().info(f"Found MCU at {self.port}!")
                            self.get_logger().info(f"Enabling Relay Mode")
                            ser.write(b"can_relay_mode,on\n")
                            break
                    except:
                        pass
                if self.port is not None:
                    break
        
            if self.port is None:
                self.get_logger().info("Unable to find MCU...")
                time.sleep(1)
                sys.exit(1)
        
            self.ser = serial.Serial(self.port, 115200)
            atexit.register(self.cleanup)
        # end __init__()


    def run(self):
        # This thread makes all the update processes run in the background
        global thread
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()

        try:
            while rclpy.ok():
                if self.launch_mode == 'core':
                    self.read_MCU() # Check the MCU for updates
        except KeyboardInterrupt:
            sys.exit(0)

    def read_MCU(self):  # NON-ANCHOR SPECIFIC
        try:
            output = str(self.ser.readline(), "utf8")
            
            if output:
                # All output over debug temporarily
                print(f"[MCU] {output}")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
                return
        except serial.SerialException as e:
            print(f"SerialException: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            sys.exit(1)
        except TypeError as e:
            print(f"TypeError: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            sys.exit(1)
        except Exception as e:
            print(f"Exception: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            sys.exit(1)

    def scale_duty(self, value: float, max_speed: float):
        leftMin = -1
        leftMax = 1
        rightMin = -max_speed/100.0
        rightMax = max_speed/100.0

        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return str(rightMin + (valueScaled * rightSpan))

    def send_controls(self, msg: CoreControl):
        if(msg.turn_to_enable):
            command = "can_relay_tovic,core,41," + str(msg.turn_to) + ',' + str(msg.turn_to_timeout) + '\n' 
        else:
            command = "can_relay_tovic,core,19," + self.scale_duty(msg.left_stick, msg.max_speed) + ',' + self.scale_duty(msg.right_stick, msg.max_speed) + '\n'
        self.send_cmd(command)

        # Brake mode
        command = "can_relay_tovic,core,18," + str(int(msg.brake)) + '\n'
        self.send_cmd(command)

        #print(f"[Sys] Relaying: {command}")

    def cmd_vel_callback(self, msg: TwistStamped):
        linear = msg.twist.linear.x
        angular = -msg.twist.angular.z

        vel_left_rads = (linear - (angular * CORE_WHEELBASE / 2)) / CORE_WHEEL_RADIUS
        vel_right_rads = (linear + (angular * CORE_WHEELBASE / 2)) / CORE_WHEEL_RADIUS

        vel_left_rpm = round((vel_left_rads * 60) / (2 * 3.14159)) * CORE_GEAR_RATIO
        vel_right_rpm = round((vel_right_rads * 60) / (2 * 3.14159)) * CORE_GEAR_RATIO

        command = f"can_relay_tovic,core,20,{vel_left_rpm},{vel_right_rpm}\n"
        self.send_cmd(command)
    
    def twist_man_callback(self, msg: Twist):
        linear = msg.linear.x  # [-1 1] for forward/back from left stick y
        angular = msg.angular.z  # [-1 1] for left/right from right stick x
        
        if (linear < 0):  # reverse turning direction when going backwards (WIP)
            angular *= -1

        if abs(linear) > 1 or abs(angular) > 1:
            # if speed is greater than 1, then there is a problem
            # make it look like a problem and don't just run away lmao
            linear = 0.25 * (linear / abs(linear))
            angular = 0.25 * (angular / abs(angular))
        
        duty_left = linear - angular
        duty_right = linear + angular
        scale = max(1, abs(duty_left), abs(duty_right))
        duty_left /= scale
        duty_right /= scale
        
        # Apply speed mode
        if self.twist_speed_mode == -1:  # slow
            duty_left = map_range(duty_left, -1, 1, -SLOW_SPEED, SLOW_SPEED)
            duty_right = map_range(duty_right, -1, 1, -SLOW_SPEED, SLOW_SPEED)
        elif self.twist_speed_mode == 0:  # walk
            duty_left = map_range(duty_left, -1, 1, -WALKING_SPEED, WALKING_SPEED)
            duty_right = map_range(duty_right, -1, 1, -WALKING_SPEED, WALKING_SPEED)
        elif self.twist_speed_mode == 1:  # fast
            duty_left = map_range(duty_left, -1, 1, -FAST_SPEED, FAST_SPEED)
            duty_right = map_range(duty_right, -1, 1, -FAST_SPEED, FAST_SPEED)

        command = f"can_relay_tovic,core,19,{duty_left},{duty_right}\n"
        self.send_cmd(command)
    
    def control_state_callback(self, msg: CoreCtrlState):
        # Brake mode
        command = f"can_relay_tovic,core,18,{str(int(msg.brake_mode))}\n"
        self.send_cmd(command)

        # Speed mode
        self.twist_speed_mode = msg.speed_mode  # twist_man_callback will handle this

    def send_cmd(self, msg: str):
        if self.launch_mode == 'anchor':
            #self.get_logger().info(f"[Core to Anchor Relay] {msg}")
            output = String()#Convert to std_msg string
            output.data = msg
            self.anchor_pub.publish(output)
        elif self.launch_mode == 'core':
            self.get_logger().info(f"[Core to MCU] {msg}")
            self.ser.write(bytes(msg, "utf8"))


    def anchor_feedback(self, msg: String):
        output = msg.data
        parts = str(output.strip()).split(",")
        # GNSS Lattitude
        if output.startswith("can_relay_fromvic,core,48"):
            self.core_feedback.gps_lat = float(parts[3])
            self.gps_state.latitude = float(parts[3])  # new
        # GNSS Longitude
        elif output.startswith("can_relay_fromvic,core,49"):
            self.core_feedback.gps_long = float(parts[3])
            self.gps_state.longitude = float(parts[3])  # new
        # GNSS Satellite count and altitude
        elif output.startswith("can_relay_fromvic,core,50"):
            self.core_feedback.gps_sats = round(float(parts[3]))
            self.core_feedback.gps_alt = round(float(parts[4]), 2)
            self.gps_state.altitude = float(parts[3])  # new
            self.gps_state.altitude = round(float(parts[4]), 2)  # new
        # Gyro x, y, z, and imu calibration
        elif output.startswith("can_relay_fromvic,core,51"):
            self.core_feedback.bno_gyro.x = float(parts[3])
            self.core_feedback.bno_gyro.y = float(parts[4])
            self.core_feedback.bno_gyro.z = float(parts[5])
            self.core_feedback.imu_calib = round(float(parts[6]))
            self.imu_state.angular_velocity.x = float(parts[3])  # new
            self.imu_state.angular_velocity.y = float(parts[4])  # new
            self.imu_state.angular_velocity.z = float(parts[5])  # new
            self.feedback_new_state.imu_calib = round(float(parts[6]))  # new
        # Accel x, y, z, heading
        elif output.startswith("can_relay_fromvic,core,52"):
            self.core_feedback.bno_accel.x = float(parts[3])
            self.core_feedback.bno_accel.y = float(parts[4])
            self.core_feedback.bno_accel.z = float(parts[5])
            self.core_feedback.orientation = float(parts[6])
            self.imu_state.linear_acceleration.x = float(parts[3])  # new
            self.imu_state.linear_acceleration.y = float(parts[4])  # new
            self.imu_state.linear_acceleration.z = float(parts[5])  # new
            self.imu_state.orientation.z = float(parts[6])  # new  -- ik this is not correct stfu
        # REV Sparkmax feedback
        elif output.startswith("can_relay_fromvic,core,53"):
            motorId = round(float(parts[3]))
            temp = float(parts[4]) / 10.0
            voltage = float(parts[5]) / 10.0
            current = float(parts[6]) / 10.0
            if motorId == 1:
                self.core_feedback.fl_temp = temp
                self.core_feedback.fl_voltage = voltage
                self.core_feedback.fl_current = current
                self.feedback_new_state.fl_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.fl_motor.temperature = temp  # new
                self.feedback_new_state.fl_motor.voltage = voltage  # new
                self.feedback_new_state.fl_motor.current = current  # new
            elif motorId == 2:
                self.core_feedback.bl_temp = temp
                self.core_feedback.bl_voltage = voltage
                self.core_feedback.bl_current = current
                self.feedback_new_state.bl_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.bl_motor.temperature = temp  # new
                self.feedback_new_state.bl_motor.voltage = voltage  # new
                self.feedback_new_state.bl_motor.current = current  # new
            elif motorId == 3:
                self.core_feedback.fr_temp = temp
                self.core_feedback.fr_voltage = voltage
                self.core_feedback.fr_current = current
                self.feedback_new_state.fr_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.fr_motor.temperature = temp  # new
                self.feedback_new_state.fr_motor.voltage = voltage  # new
                self.feedback_new_state.fr_motor.current = current  # new
            elif motorId == 4:
                self.core_feedback.br_temp = temp
                self.core_feedback.br_voltage = voltage
                self.core_feedback.br_current = current
                self.feedback_new_state.br_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.br_motor.temperature = temp  # new
                self.feedback_new_state.br_motor.voltage = voltage  # new
                self.feedback_new_state.br_motor.current = current  # new
        # Voltages batt, 12, 5, 3, all * 100
        elif output.startswith("can_relay_fromvic,core,54"):
            self.core_feedback.bat_voltage = float(parts[3]) / 100.0
            self.core_feedback.voltage_12 = float(parts[4]) / 100.0
            self.core_feedback.voltage_5 = float(parts[5]) / 100.0
            self.core_feedback.voltage_3 = float(parts[6]) / 100.0
            self.feedback_new_state.voltage_batt = float(parts[3]) / 100.0  # new
            self.feedback_new_state.voltage_12 = float(parts[4]) / 100.0  # new
            self.feedback_new_state.voltage_5 = float(parts[5]) / 100.0  # new
            self.feedback_new_state.voltage_3 = float(parts[6]) / 100.0  # new
        # BMP temperature, altitude, pressure
        elif output.startswith("can_relay_fromvic,core,56"):
            self.core_feedback.bmp_temp = float(parts[3])
            self.core_feedback.bmp_alt = float(parts[4])
            self.core_feedback.bmp_pres = float(parts[5])
            self.feedback_new_state.bmp_temp = float(parts[3])  # new
            self.feedback_new_state.bmp_alt = float(parts[4])  # new
            self.feedback_new_state.bmp_pres = float(parts[5])  # new
        # REV position and velocity
        elif output.startswith("can_relay_fromvic,core,58"):
            motorId = round(float(parts[3]))
            position = float(parts[4])
            velocity = float(parts[5])
            if motorId == 1:
                self.feedback_new_state.fl_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.fl_motor.position = position  # new
                self.feedback_new_state.fl_motor.velocity = velocity  # new
            elif motorId == 2:
                self.feedback_new_state.bl_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.bl_motor.position = position  # new
                self.feedback_new_state.bl_motor.velocity = velocity  # new
            elif motorId == 3:
                self.feedback_new_state.fr_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.fr_motor.position = position  # new
                self.feedback_new_state.fr_motor.velocity = velocity  # new
            elif motorId == 4:
                self.feedback_new_state.br_motor.header.stamp = self.get_clock().now().to_msg()
                self.feedback_new_state.br_motor.position = position  # new
                self.feedback_new_state.br_motor.velocity = velocity  # new
        else:
            return
        self.feedback_new_state.header.stamp = self.get_clock().now().to_msg()
        self.feedback_new_pub_.publish(self.feedback_new_state)
        #self.get_logger().info(f"[Core Anchor] {msg}")

    def publish_feedback(self):
        #self.get_logger().info(f"[Core] {self.core_feedback}")
        self.feedback_pub.publish(self.core_feedback)

    def ping_callback(self, request, response):
        return response


    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    
    def cleanup(self):
        print("Cleaning up before terminating...")
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception as e:
            exit(0)


def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)
    if serial_pub:
        serial_pub.cleanup()

def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main(args=None):
   rclpy.init(args=args)
   sys.excepthook = myexcepthook

   global serial_pub

   serial_pub = SerialRelay()
   serial_pub.run()

if __name__ == '__main__':
    #signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))  # Catch termination signals and exit cleanly
    main()
