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
from scipy.spatial.transform import Rotation
from math import copysign, pi

from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, JointState
from geometry_msgs.msg import TwistStamped, Twist
from ros2_interfaces_pkg.msg import CoreControl, CoreFeedback, RevMotorState
from ros2_interfaces_pkg.msg import VicCAN, NewCoreFeedback, Barometer, CoreCtrlState


serial_pub = None
thread = None

CORE_WHEELBASE = 0.836  # meters
CORE_WHEEL_RADIUS = 0.171  # meters
CORE_GEAR_RATIO = 100.0  # Clucky: 100:1, Testbed: 64:1

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

# Used to verify the length of an incoming VicCAN feedback message
# Key is VicCAN command_id, value is expected length of data list
viccan_msg_len_dict = {
    48: 1,
    49: 1,
    50: 2,
    51: 4,
    52: 4,
    53: 4,
    54: 4,
    56: 4,  # really 3, but viccan
    58: 4   # ditto
}


class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("core_node")

        # Launch mode -- anchor vs core
        self.declare_parameter('launch_mode', 'core')
        self.launch_mode = self.get_parameter('launch_mode').value
        self.get_logger().info(f"Core launch_mode is: {self.launch_mode}")


        ##################################################
        # Topics

        # Anchor
        if self.launch_mode == 'anchor':
            self.anchor_fromvic_sub_ = self.create_subscription(VicCAN, '/anchor/from_vic/core', self.relay_fromvic, 20)
            self.anchor_tovic_pub_ = self.create_publisher(VicCAN, '/anchor/to_vic/relay', 20)

            self.anchor_sub = self.create_subscription(String, '/anchor/core/feedback', self.anchor_feedback, 10)
            self.anchor_pub = self.create_publisher(String, '/anchor/relay', 10)

        # Control

        # autonomy twist -- m/s and rad/s -- for autonomy, in particular Nav2
        self.cmd_vel_sub_ = self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_callback, 1)
        # manual twist -- [-1, 1] rather than real units
        self.twist_man_sub_ = self.create_subscription(Twist, '/core/twist', self.twist_man_callback, qos_profile=control_qos)
        # manual flags -- brake mode and max duty cycle
        self.control_state_sub_ = self.create_subscription(CoreCtrlState, '/core/control/state', self.control_state_callback, qos_profile=control_qos)
        self.twist_max_duty = 0.5  # max duty cycle for twist commands (0.0 - 1.0); walking speed is 0.5

        # Feedback

        # Consolidated and organized core feedback
        self.feedback_new_pub_ = self.create_publisher(NewCoreFeedback, '/core/feedback_new', qos_profile=qos.qos_profile_sensor_data)
        self.feedback_new_state = NewCoreFeedback()
        self.feedback_new_state.fl_motor.id = 1
        self.feedback_new_state.bl_motor.id = 2
        self.feedback_new_state.fr_motor.id = 3
        self.feedback_new_state.br_motor.id = 4
        self.telemetry_pub_timer = self.create_timer(1.0, self.publish_feedback)  # TODO: not sure about this
        # Joint states for topic-based controller
        self.joint_state_pub_ = self.create_publisher(JointState, '/core/joint_states', qos_profile=qos.qos_profile_sensor_data)
        # IMU (embedded BNO-055)
        self.imu_pub_ = self.create_publisher(Imu, '/core/imu', qos_profile=qos.qos_profile_sensor_data)
        self.imu_state = Imu()
        self.imu_state.header.frame_id = "core_bno055"
        # GPS (embedded u-blox M9N)
        self.gps_pub_ = self.create_publisher(NavSatFix, '/gps/fix', qos_profile=qos.qos_profile_sensor_data)
        self.gps_state = NavSatFix()
        self.gps_state.header.frame_id = "core_gps_antenna"
        self.gps_state.status.service = NavSatStatus.SERVICE_GPS
        self.gps_state.status.status = NavSatStatus.STATUS_NO_FIX
        self.gps_state.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        # Barometer (embedded BMP-388)
        self.baro_pub_ = self.create_publisher(Barometer, '/core/baro', qos_profile=qos.qos_profile_sensor_data)
        self.baro_state = Barometer()
        self.baro_state.header.frame_id = "core_bmp388"

        # Old
        
        # /core/control
        self.control_sub = self.create_subscription(CoreControl, '/core/control', self.send_controls, 10)  # old control method -- left_stick, right_stick, max_speed, brake, and some other random autonomy stuff
        # /core/feedback
        self.feedback_pub = self.create_publisher(CoreFeedback, '/core/feedback', 10)
        self.core_feedback = CoreFeedback()
        # Debug
        self.debug_pub = self.create_publisher(String, '/core/debug', 10)
        self.ping_service = self.create_service(Empty, '/astra/core/ping', self.ping_callback)


        ##################################################
        # Find microcontroller (Non-anchor only)

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
        
        self.send_viccan(20, [vel_left_rpm, vel_right_rpm])

    def twist_man_callback(self, msg: Twist):
        linear = msg.linear.x  # [-1 1] for forward/back from left stick y
        angular = msg.angular.z  # [-1 1] for left/right from right stick x

        if (linear < 0):  # reverse turning direction when going backwards (WIP)
            angular *= -1

        if abs(linear) > 1 or abs(angular) > 1:
            # if speed is greater than 1, then there is a problem
            # make it look like a problem and don't just run away lmao
            linear = copysign(0.25, linear)  # 0.25 duty cycle in direction of control (hopefully slow)
            angular = copysign(0.25, angular)

        duty_left = linear - angular
        duty_right = linear + angular
        scale = max(1, abs(duty_left), abs(duty_right))
        duty_left /= scale
        duty_right /= scale

        # Apply max duty cycle
        # Joysticks provide values [-1, 1] rather than real units
        duty_left = map_range(duty_left, -1, 1, -self.twist_max_duty, self.twist_max_duty)
        duty_right = map_range(duty_right, -1, 1, -self.twist_max_duty, self.twist_max_duty)

        self.send_viccan(19, [duty_left, duty_right])

    def control_state_callback(self, msg: CoreCtrlState):
        # Brake mode
        self.send_viccan(18, [msg.brake_mode])

        # Max duty cycle
        self.twist_max_duty = msg.max_duty  # twist_man_callback will handle this

    def send_cmd(self, msg: str):
        if self.launch_mode == 'anchor':
            #self.get_logger().info(f"[Core to Anchor Relay] {msg}")
            output = String()#Convert to std_msg string
            output.data = msg
            self.anchor_pub.publish(output)
        elif self.launch_mode == 'core':
            self.get_logger().info(f"[Core to MCU] {msg}")
            self.ser.write(bytes(msg, "utf8"))


    def send_viccan(self, cmd_id: int, data: list[float]):
        self.anchor_tovic_pub_.publish(VicCAN(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="to_vic"),
            mcu_name="core",
            command_id=cmd_id,
            data=data
        ))


    def anchor_feedback(self, msg: String):
        output = msg.data
        parts = str(output.strip()).split(",")
        # GNSS Latitude
        if output.startswith("can_relay_fromvic,core,48"):
            self.core_feedback.gps_lat = float(parts[3])
        # GNSS Longitude
        elif output.startswith("can_relay_fromvic,core,49"):
            self.core_feedback.gps_long = float(parts[3])
        # GNSS Satellite count and altitude
        elif output.startswith("can_relay_fromvic,core,50"):
            self.core_feedback.gps_sats = round(float(parts[3]))
            self.core_feedback.gps_alt = round(float(parts[4]), 2)
        # Gyro x, y, z, and imu calibration
        elif output.startswith("can_relay_fromvic,core,51"):
            self.core_feedback.bno_gyro.x = float(parts[3])
            self.core_feedback.bno_gyro.y = float(parts[4])
            self.core_feedback.bno_gyro.z = float(parts[5])
            self.core_feedback.imu_calib = round(float(parts[6]))
        # Accel x, y, z, heading
        elif output.startswith("can_relay_fromvic,core,52"):
            self.core_feedback.bno_accel.x = float(parts[3])
            self.core_feedback.bno_accel.y = float(parts[4])
            self.core_feedback.bno_accel.z = float(parts[5])
            self.core_feedback.orientation = float(parts[6])
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
            elif motorId == 2:
                self.core_feedback.bl_temp = temp
                self.core_feedback.bl_voltage = voltage
                self.core_feedback.bl_current = current
            elif motorId == 3:
                self.core_feedback.fr_temp = temp
                self.core_feedback.fr_voltage = voltage
                self.core_feedback.fr_current = current
            elif motorId == 4:
                self.core_feedback.br_temp = temp
                self.core_feedback.br_voltage = voltage
                self.core_feedback.br_current = current
        # Voltages batt, 12, 5, 3, all * 100
        elif output.startswith("can_relay_fromvic,core,54"):
            self.core_feedback.bat_voltage = float(parts[3]) / 100.0
            self.core_feedback.voltage_12 = float(parts[4]) / 100.0
            self.core_feedback.voltage_5 = float(parts[5]) / 100.0
            self.core_feedback.voltage_3 = float(parts[6]) / 100.0
        # BMP temperature, altitude, pressure
        elif output.startswith("can_relay_fromvic,core,56"):
            self.core_feedback.bmp_temp = float(parts[3])
            self.core_feedback.bmp_alt = float(parts[4])
            self.core_feedback.bmp_pres = float(parts[5])
        else:
            return
        self.feedback_new_state.header.stamp = self.get_clock().now().to_msg()
        self.feedback_new_pub_.publish(self.feedback_new_state)
        #self.get_logger().info(f"[Core Anchor] {msg}")

    def relay_fromvic(self, msg: VicCAN):
        # Assume that the message is coming from Core
        # skill diff if not

        # Check message len to prevent crashing on bad data
        if msg.command_id in viccan_msg_len_dict:
            expected_len = viccan_msg_len_dict[msg.command_id]
            if len(msg.data) != expected_len:
                self.get_logger().warning(f"Ignoring VicCAN message with id {msg.command_id} due to unexpected data length (expected {expected_len}, got {len(msg.data)})")
                return

        match msg.command_id:
            # GNSS
            case 48:  # GNSS Latitude
                self.gps_state.latitude = float(msg.data[0])
            case 49:  # GNSS Longitude
                self.gps_state.longitude = float(msg.data[0])
            case 50:  # GNSS Satellite count and altitude
                self.gps_state.status.status = NavSatStatus.STATUS_FIX if int(msg.data[0]) >= 3 else NavSatStatus.STATUS_NO_FIX
                self.gps_state.altitude = float(msg.data[1])
                self.gps_state.header.stamp = msg.header.stamp
                self.gps_pub_.publish(self.gps_state)
            # IMU
            case 51:  # Gyro x, y, z, and imu calibration
                self.feedback_new_state.imu_calib = round(float(msg.data[3]))
                self.imu_state.angular_velocity.x = float(msg.data[0])
                self.imu_state.angular_velocity.y = float(msg.data[1])
                self.imu_state.angular_velocity.z = float(msg.data[2])
                self.imu_state.header.stamp = msg.header.stamp
            case 52:  # Accel x, y, z, heading
                self.imu_state.linear_acceleration.x = float(msg.data[0])
                self.imu_state.linear_acceleration.y = float(msg.data[1])
                self.imu_state.linear_acceleration.z = float(msg.data[2])
                # Deal with quaternion
                r = Rotation.from_euler('z', float(msg.data[3]), degrees=True)
                q = r.as_quat()
                self.imu_state.orientation.x = q[0]
                self.imu_state.orientation.y = q[1]
                self.imu_state.orientation.z = q[2]
                self.imu_state.orientation.w = q[3]
                self.imu_state.header.stamp = msg.header.stamp
                self.imu_pub_.publish(self.imu_state)
            # REV Motors
            case 53:  # REV SPARK MAX feedback
                motorId = round(float(msg.data[0]))
                temp = float(msg.data[1]) / 10.0
                voltage = float(msg.data[2]) / 10.0
                current = float(msg.data[3]) / 10.0
                motor: RevMotorState | None = None
                match motorId:
                    case 1:
                        motor = self.feedback_new_state.fl_motor
                    case 2:
                        motor = self.feedback_new_state.bl_motor
                    case 3:
                        motor = self.feedback_new_state.fr_motor
                    case 4:
                        motor = self.feedback_new_state.br_motor

                if motor:
                    motor.temperature = temp
                    motor.voltage = voltage
                    motor.current = current
                    motor.header.stamp = msg.header.stamp

                self.feedback_new_pub_.publish(self.feedback_new_state)
            # Board voltage
            case 54:  # Voltages batt, 12, 5, 3, all * 100
                self.feedback_new_state.board_voltage.vbatt = float(msg.data[0]) / 100.0
                self.feedback_new_state.board_voltage.v12 = float(msg.data[1]) / 100.0
                self.feedback_new_state.board_voltage.v5 = float(msg.data[2]) / 100.0
                self.feedback_new_state.board_voltage.v3 = float(msg.data[3]) / 100.0
            # Baro
            case 56:  # BMP temperature, altitude, pressure
                self.baro_state.temperature = float(msg.data[0])
                self.baro_state.altitude = float(msg.data[1])
                self.baro_state.pressure = float(msg.data[2])
                self.baro_state.header.stamp = msg.header.stamp
                self.baro_pub_.publish(self.baro_state)
            # REV Motors (pos and vel)
            case 58:  # REV position and velocity
                motorId = round(float(msg.data[0]))
                position = float(msg.data[1])
                velocity = float(msg.data[2])
                joint_state_msg = JointState()  # TODO: not sure if all motors should be in each message or not
                joint_state_msg.position = [position * (2 * pi) / CORE_GEAR_RATIO]  # revolutions to radians
                joint_state_msg.velocity = [velocity * (2 * pi / 60.0) / CORE_GEAR_RATIO]  # RPM to rad/s

                motor: RevMotorState | None = None

                match motorId:
                    case 1:
                        motor = self.feedback_new_state.fl_motor
                        joint_state_msg.name = ["fl_motor_joint"]
                    case 2:
                        motor = self.feedback_new_state.bl_motor
                        joint_state_msg.name = ["bl_motor_joint"]
                    case 3:
                        motor = self.feedback_new_state.fr_motor
                        joint_state_msg.name = ["fr_motor_joint"]
                    case 4:
                        motor = self.feedback_new_state.br_motor
                        joint_state_msg.name = ["br_motor_joint"]

                joint_state_msg.header.stamp = msg.header.stamp
                self.joint_state_pub_.publish(joint_state_msg)
            case _:
                return


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
