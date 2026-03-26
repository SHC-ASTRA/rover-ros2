import sys
import signal
from scipy.spatial.transform import Rotation
from math import copysign, pi
from warnings import deprecated

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy import qos
from rclpy.duration import Duration

from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, JointState
from geometry_msgs.msg import TwistStamped, Twist
from astra_msgs.msg import CoreControl, CoreFeedback, RevMotorState
from astra_msgs.msg import VicCAN, NewCoreFeedback, Barometer, CoreCtrlState


CORE_WHEELBASE = 0.836  # meters
CORE_WHEEL_RADIUS = 0.171  # meters
CORE_GEAR_RATIO = 100.0  # Clucky: 100:1, Testbed: 64:1

control_qos = qos.QoSProfile(
    history=qos.QoSHistoryPolicy.KEEP_LAST,
    depth=2,
    reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,  # Best Effort subscribers are still compatible with Reliable publishers
    durability=qos.QoSDurabilityPolicy.VOLATILE,
    # deadline=Duration(seconds=1),
    # lifespan=Duration(nanoseconds=500_000_000),  # 500ms
    # liveliness=qos.QoSLivelinessPolicy.SYSTEM_DEFAULT,
    # liveliness_lease_duration=Duration(seconds=5),
)


class CoreNode(Node):
    """Relay between Anchor and Basestation/Headless/Moveit2 for Core related topics."""

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
        58: 4,  # ditto
    }

    def __init__(self):
        super().__init__("core_node")

        self.get_logger().info(f"core launch_mode is: anchor")

        ##################################################
        # Parameters

        self.declare_parameter("use_ros2_control", False)
        self.use_ros2_control = (
            self.get_parameter("use_ros2_control").get_parameter_value().bool_value
        )

        ##################################################
        # Old Topics

        self.anchor_sub = self.create_subscription(
            String, "/anchor/core/feedback", self.anchor_feedback, 10
        )
        self.anchor_pub = self.create_publisher(String, "/anchor/relay", 10)

        if not self.use_ros2_control:
            # /core/control
            self.control_sub = self.create_subscription(
                CoreControl, "/core/control", self.send_controls, 10
            )  # old control method -- left_stick, right_stick, max_speed, brake, and some other random autonomy stuff
        # /core/feedback
        self.feedback_pub = self.create_publisher(CoreFeedback, "/core/feedback", 10)
        self.core_feedback = CoreFeedback()

        self.telemetry_pub_timer = self.create_timer(
            1.0, self.publish_feedback
        )

        ##################################################
        # New Topics

        # Anchor

        self.anchor_fromvic_sub_ = self.create_subscription(
            VicCAN, "/anchor/from_vic/core", self.relay_fromvic, 20
        )
        self.anchor_tovic_pub_ = self.create_publisher(
            VicCAN, "/anchor/to_vic/relay", 20
        )

        # Control

        if self.use_ros2_control:
            # Joint state control for topic-based controller
            self.joint_command_sub_ = self.create_subscription(
                JointState, "/core/joint_commands", self.joint_command_callback, 2
            )
        else:
            # autonomy twist -- m/s and rad/s -- for autonomy, in particular Nav2
            self.cmd_vel_sub_ = self.create_subscription(
                TwistStamped, "/cmd_vel", self.cmd_vel_callback, 1
            )
            # manual twist -- [-1, 1] rather than real units
            # TODO: change topic to '/core/control/twist'
            self.twist_man_sub_ = self.create_subscription(
                Twist, "/core/twist", self.twist_man_callback, qos_profile=control_qos
            )
            # manual flags -- brake mode and max duty cycle
            self.control_state_sub_ = self.create_subscription(
                CoreCtrlState,
                "/core/control/state",
                self.control_state_callback,
                qos_profile=control_qos,
            )
            self.twist_max_duty = 0.5  # max duty cycle for twist commands (0.0 - 1.0); walking speed is 0.5

        # Feedback

        # Consolidated and organized main core feedback
        # TODO: change topic to something like '/core/feedback/main'
        self.feedback_new_pub_ = self.create_publisher(
            NewCoreFeedback,
            "/core/feedback_new",
            qos_profile=qos.qos_profile_sensor_data,
        )

        # Joint states for topic-based controller
        self.joint_state_pub_ = self.create_publisher(
            JointState, "/joint_states", qos_profile=qos.qos_profile_sensor_data
        )

        # IMU (embedded BNO-055)
        self.imu_pub_ = self.create_publisher(
            Imu, "/core/imu", qos_profile=qos.qos_profile_sensor_data
        )

        # GPS (embedded u-blox M9N)
        self.gps_pub_ = self.create_publisher(
            NavSatFix, "/gps/fix", qos_profile=qos.qos_profile_sensor_data
        )

        # Barometer (embedded BMP-388)
        self.baro_pub_ = self.create_publisher(
            Barometer, "/core/feedback/baro", qos_profile=qos.qos_profile_sensor_data
        )

        ###################################################
        # Saved state

        # Main Core feedback
        self.feedback_new_state = NewCoreFeedback()
        self.feedback_new_state.fl_motor.id = 1
        self.feedback_new_state.bl_motor.id = 2
        self.feedback_new_state.fr_motor.id = 3
        self.feedback_new_state.br_motor.id = 4

        # IMU
        self.imu_state = Imu()
        self.imu_state.header.frame_id = "core_bno055"

        # GPS
        self.gps_state = NavSatFix()
        self.gps_state.header.frame_id = "core_gps_antenna"
        self.gps_state.status.service = NavSatStatus.SERVICE_GPS
        self.gps_state.status.status = NavSatStatus.STATUS_NO_FIX
        self.gps_state.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        # Barometer
        self.baro_state = Barometer()
        self.baro_state.header.frame_id = "core_bmp388"

    @deprecated("Uses an old message type. Will be removed at some point.")
    def scale_duty(self, value: float, max_speed: float):
        leftMin = -1
        leftMax = 1
        rightMin = -max_speed / 100.0
        rightMax = max_speed / 100.0

        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return str(rightMin + (valueScaled * rightSpan))

    @deprecated("Uses an old message type. Will be removed at some point.")
    def send_controls(self, msg: CoreControl):
        if msg.turn_to_enable:
            command = (
                "can_relay_tovic,core,41,"
                + str(msg.turn_to)
                + ","
                + str(msg.turn_to_timeout)
                + "\n"
            )
        else:
            command = (
                "can_relay_tovic,core,19,"
                + self.scale_duty(msg.left_stick, msg.max_speed)
                + ","
                + self.scale_duty(msg.right_stick, msg.max_speed)
                + "\n"
            )
        self.send_cmd(command)

        # Brake mode
        command = "can_relay_tovic,core,18," + str(int(msg.brake)) + "\n"
        self.send_cmd(command)

        # print(f"[Sys] Relaying: {command}")

    def joint_command_callback(self, msg: JointState):
        # So... topic based control node publishes JointState messages over /joint_commands
        #  with len(msg.name) == 5 and len(msg.velocity) == 4... all 5 non-fixed joints
        #  are included in msg.name, but ig it is implied that msg.velocity only
        #  includes velocities for the commanded joints (ros__parameters.joints).
        # So, this will be much more hacky and less adaptable than I would like it to be.
        if len(msg.name) != 5 or len(msg.velocity) != 4 or len(msg.position) != 0:
            self.get_logger().warning(
                f"Received joint control message with unexpected number of joints. Ignoring."
            )
            return
        if msg.name != [
            "left_suspension_joint",
            "bl_wheel_joint",
            "br_wheel_joint",
            "fl_wheel_joint",
            "fr_wheel_joint",
        ]:
            self.get_logger().warning(
                f"Received joint control message with unexpected name[]. Ignoring."
            )
            return

        (bl_vel, br_vel, fl_vel, fr_vel) = msg.velocity

        bl_rpm = radps_to_rpm(bl_vel) * CORE_GEAR_RATIO
        br_rpm = radps_to_rpm(br_vel) * CORE_GEAR_RATIO
        fl_rpm = radps_to_rpm(fl_vel) * CORE_GEAR_RATIO
        fr_rpm = radps_to_rpm(fr_vel) * CORE_GEAR_RATIO

        self.send_viccan(
            20, [fl_rpm, bl_rpm, fr_rpm, br_rpm]
        )  # order expected by embedded

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

        if linear < 0:  # reverse turning direction when going backwards (WIP)
            angular *= -1

        if abs(linear) > 1 or abs(angular) > 1:
            # if speed is greater than 1, then there is a problem
            # make it look like a problem and don't just run away lmao
            linear = copysign(
                0.25, linear
            )  # 0.25 duty cycle in direction of control (hopefully slow)
            angular = copysign(0.25, angular)

        duty_left = linear - angular
        duty_right = linear + angular
        scale = max(1, abs(duty_left), abs(duty_right))
        duty_left /= scale
        duty_right /= scale

        # Apply max duty cycle
        # Joysticks provide values [-1, 1] rather than real units
        duty_left = map_range(
            duty_left, -1, 1, -self.twist_max_duty, self.twist_max_duty
        )
        duty_right = map_range(
            duty_right, -1, 1, -self.twist_max_duty, self.twist_max_duty
        )

        self.send_viccan(19, [duty_left, duty_right])

    def control_state_callback(self, msg: CoreCtrlState):
        # Brake mode
        self.send_viccan(18, [msg.brake_mode])

        # Max duty cycle
        self.twist_max_duty = msg.max_duty  # twist_man_callback will handle this

    @deprecated("Uses an old message type. Will be removed at some point.")
    def send_cmd(self, msg: str):
        self.anchor_pub.publish(String(data=msg))  # Publish to anchor for relay

    def send_viccan(self, cmd_id: int, data: list[float]):
        self.anchor_tovic_pub_.publish(
            VicCAN(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="to_vic"),
                mcu_name="core",
                command_id=cmd_id,
                data=data,
            )
        )

    @deprecated("Uses an old message type. Will be removed at some point.")
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
        # self.get_logger().info(f"[Core Anchor] {msg}")

    def relay_fromvic(self, msg: VicCAN):
        # Assume that the message is coming from Core
        # skill diff if not

        # Check message len to prevent crashing on bad data
        if msg.command_id in self.viccan_msg_len_dict:
            expected_len = self.viccan_msg_len_dict[msg.command_id]
            if len(msg.data) != expected_len:
                self.get_logger().warning(
                    f"Ignoring VicCAN message with id {msg.command_id} due to unexpected data length (expected {expected_len}, got {len(msg.data)})"
                )
                return

        self.feedback_new_state.header.stamp = msg.header.stamp

        match msg.command_id:
            # GNSS
            case 48:  # GNSS Latitude
                self.gps_state.latitude = float(msg.data[0])
            case 49:  # GNSS Longitude
                self.gps_state.longitude = float(msg.data[0])
            case 50:  # GNSS Satellite count and altitude
                self.gps_state.status.status = (
                    NavSatStatus.STATUS_FIX
                    if int(msg.data[0]) >= 3
                    else NavSatStatus.STATUS_NO_FIX
                )
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
                self.feedback_new_state.orientation = float(msg.data[3])
                # Deal with quaternion
                r = Rotation.from_euler("z", float(msg.data[3]), degrees=True)
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
                    case _:
                        self.get_logger().warning(
                            f"Ignoring REV motor feedback 53 with invalid motorId {motorId}"
                        )
                        return

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
                self.feedback_new_state.board_voltage.header.stamp = msg.header.stamp
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
                joint_state_msg = (
                    JointState()
                )
                joint_state_msg.position = [
                    position * (2 * pi) / CORE_GEAR_RATIO
                ]  # revolutions to radians
                joint_state_msg.velocity = [
                    velocity * (2 * pi / 60.0) / CORE_GEAR_RATIO
                ]  # RPM to rad/s

                motor: RevMotorState | None = None

                match motorId:
                    case 1:
                        motor = self.feedback_new_state.fl_motor
                        joint_state_msg.name = ["fl_wheel_joint"]
                    case 2:
                        motor = self.feedback_new_state.bl_motor
                        joint_state_msg.name = ["bl_wheel_joint"]
                    case 3:
                        motor = self.feedback_new_state.fr_motor
                        joint_state_msg.name = ["fr_wheel_joint"]
                    case 4:
                        motor = self.feedback_new_state.br_motor
                        joint_state_msg.name = ["br_wheel_joint"]
                    case _:
                        self.get_logger().warning(
                            f"Ignoring REV motor feedback 58 with invalid motorId {motorId}"
                        )
                        return
                
                if motor:
                    motor.position = position
                    motor.velocity = velocity

                # make the fucking shit work
                joint_state_msg.name.append("left_suspension_joint")
                joint_state_msg.position.append(0.0)
                joint_state_msg.velocity.append(0.0)

                joint_state_msg.header.stamp = msg.header.stamp
                self.joint_state_pub_.publish(joint_state_msg)
            case _:
                return

    @deprecated("Uses an old message type. Will be removed at some point.")
    def publish_feedback(self):
        # self.get_logger().info(f"[Core] {self.core_feedback}")
        self.feedback_pub.publish(self.core_feedback)


def map_range(
    value: float, in_min: float, in_max: float, out_min: float, out_max: float
):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def radps_to_rpm(radps: float):
    return radps * 60 / (2 * pi)


def exit_handler(signum, frame):
    print("Caught SIGTERM. Exiting...")
    rclpy.try_shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    # Catch termination signals and exit cleanly
    signal.signal(signal.SIGTERM, exit_handler)

    core_node = CoreNode()

    try:
        rclpy.spin(core_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
