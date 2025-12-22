import rclpy
from rclpy.node import Node
from rclpy import qos
import serial
import sys
import threading
import glob
import time
import atexit
import signal
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from astra_msgs.msg import SocketFeedback, DigitFeedback, ArmManual
from astra_msgs.msg import ArmFeedback, VicCAN, RevMotorState
import math

# control_qos = qos.QoSProfile(
#     history=qos.QoSHistoryPolicy.KEEP_LAST,
#     depth=1,
#     reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
#     durability=qos.QoSDurabilityPolicy.VOLATILE,
#     deadline=1000,
#     lifespan=500,
#     liveliness=qos.QoSLivelinessPolicy.SYSTEM_DEFAULT,
#     liveliness_lease_duration=5000
# )

serial_pub = None
thread = None

# Used to verify the length of an incoming VicCAN feedback message
# Key is VicCAN command_id, value is expected length of data list
viccan_socket_msg_len_dict = {
    53: 4,
    54: 4,
    55: 4,
    58: 4,
    59: 4,
}
viccan_digit_msg_len_dict = {
    54: 4,
    55: 2,
    59: 2,
}


class SerialRelay(Node):
    def __init__(self):
        super().__init__("arm_node")

        # Get launch mode -- anchor vs arm
        self.declare_parameter("launch_mode", "arm")
        self.launch_mode = self.get_parameter("launch_mode").value
        self.get_logger().info(f"arm launch_mode is: {self.launch_mode}")

        ##################################################
        # Topics

        # Anchor topics
        if self.launch_mode == "anchor":
            self.anchor_fromvic_sub_ = self.create_subscription(
                VicCAN, "/anchor/from_vic/arm", self.relay_fromvic, 20
            )
            self.anchor_tovic_pub_ = self.create_publisher(
                VicCAN, "/anchor/to_vic/relay", 20
            )

            self.anchor_sub = self.create_subscription(
                String, "/anchor/arm/feedback", self.anchor_feedback, 10
            )
            self.anchor_pub = self.create_publisher(String, "/anchor/relay", 10)

        # Control

        # Manual: who tf knows. Maybe JointJog?
        # IK: /joint_commands is published by JointTrajectoryController via topic_based_control
        self.joint_command_sub_ = self.create_subscription(
            JointState, "/joint_commands", self.joint_command_callback, 1
        )

        # Feedback

        self.arm_feedback_pub_ = self.create_publisher(
            ArmFeedback,
            "/arm/feedback/new_feedback",
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.arm_feedback_new = ArmFeedback()
        # IK: /joint_states is published from here to topic_based_control
        self.joint_state_pub_ = self.create_publisher(
            JointState, "joint_states", qos_profile=qos.qos_profile_sensor_data
        )
        self.saved_joint_state = JointState()
        self.saved_joint_state.name = [
            "Axis_0_Joint",
            "Axis_1_Joint",
            "Axis_2_Joint",
            "Axis_3_Joint",
            "Wrist_Differential_Joint",
            "Wrist-EF_Roll_Joint",
            "Gripper_Slider_Left",
        ]
        self.saved_joint_state.position = [0.0] * len(
            self.saved_joint_state.name
        )  # Initialize with zeros
        self.saved_joint_state.velocity = [0.0] * len(
            self.saved_joint_state.name
        )  # Initialize with zeros

        # Old

        # Create publishers
        self.debug_pub = self.create_publisher(String, "/arm/feedback/debug", 10)
        self.socket_pub = self.create_publisher(
            SocketFeedback, "/arm/feedback/socket", 10
        )
        self.arm_feedback = SocketFeedback()
        self.digit_pub = self.create_publisher(DigitFeedback, "/arm/feedback/digit", 10)
        self.digit_feedback = DigitFeedback()
        self.feedback_timer = self.create_timer(0.25, self.publish_feedback)

        # Create subscribers
        self.man_sub = self.create_subscription(
            ArmManual, "/arm/control/manual", self.send_manual, 10
        )

        ##################################################
        # Find microcontroller -- non-anchor only

        # Search for ports IF in 'arm' (standalone) and not 'anchor' mode
        if self.launch_mode == "arm":
            # Loop through all serial devices on the computer to check for the MCU
            self.port = None
            ports = SerialRelay.list_serial_ports()
            for _ in range(4):
                for port in ports:
                    try:
                        # connect and send a ping command
                        ser = serial.Serial(port, 115200, timeout=1)
                        # print(f"Checking port {port}...")
                        ser.write(b"ping\n")
                        response = ser.read_until("\n")  # type: ignore

                        # if pong is in response, then we are talking with the MCU
                        if b"pong" in response:
                            self.port = port
                            self.get_logger().info(f"Found MCU at {self.port}!")
                            break
                    except:
                        pass
                if self.port is not None:
                    break

            if self.port is None:
                self.get_logger().info(
                    "Unable to find MCU... please make sure it is connected."
                )
                time.sleep(1)
                sys.exit(1)

            self.ser = serial.Serial(self.port, 115200)
            atexit.register(self.cleanup)

    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        # if in arm mode, will need to read from the MCU

        try:
            while rclpy.ok():
                if self.launch_mode == "arm":
                    if self.ser.in_waiting:
                        self.read_mcu()
                    else:
                        time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    # Currently will just spit out all values over the /arm/feedback/debug topic as strings
    def read_mcu(self):
        try:
            output = str(self.ser.readline(), "utf8")
            if output:
                # self.get_logger().info(f"[MCU] {output}")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
        except serial.SerialException:
            self.get_logger().info("SerialException caught... closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
        except TypeError as e:
            self.get_logger().info(f"TypeError: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
        except Exception as e:
            print(f"Exception: {e}")
            print("Closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass

    def joint_command_callback(self, msg: JointState):
        if len(msg.position) < 7 and len(msg.velocity) < 7:
            return  # command needs either position or velocity for all 7 joints

        # Assumed order: Axis0, Axis1, Axis2, Axis3, Wrist_Yaw, Wrist_Roll, Gripper
        # TODO: formalize joint names in URDF, refactor here to depend on joint names
        # Embedded takes deg*10, ROS2 uses Radians
        velocities = [
            math.degrees(vel) * 10 if abs(vel) > 0.05 else 0.0 for vel in msg.velocity
        ]
        # Axis 2 & 3 URDF direction is inverted
        velocities[2] = -velocities[2]
        velocities[3] = -velocities[3]

        # Axis 0-3
        arm_cmd = VicCAN(mcu_name="arm", command_id=43, data=velocities[0:3])
        arm_cmd.header = Header(stamp=self.get_clock().now().to_msg())
        # Wrist yaw and roll, gripper included for future use when have adequate hardware
        digit_cmd = VicCAN(mcu_name="digit", command_id=43, data=velocities[4:6])
        digit_cmd.header = Header(stamp=self.get_clock().now().to_msg())

        self.anchor_tovic_pub_.publish(arm_cmd)
        self.anchor_tovic_pub_.publish(digit_cmd)

    def send_manual(self, msg: ArmManual):
        axis0 = msg.axis0
        axis1 = -1 * msg.axis1
        axis2 = msg.axis2
        axis3 = msg.axis3

        # Send controls for arm
        command = f"can_relay_tovic,arm,18,{int(msg.brake)}\n"
        command += f"can_relay_tovic,arm,39,{axis0},{axis1},{axis2},{axis3}\n"

        # Send controls for end effector

        command += f"can_relay_tovic,digit,39,{msg.effector_yaw},{msg.effector_roll}\n"

        # command += f"can_relay_tovic,digit,26,{msg.gripper}\n"  # no hardware rn

        command += f"can_relay_tovic,digit,28,{msg.laser}\n"

        command += f"can_relay_tovic,digit,34,{msg.linear_actuator}\n"

        self.send_cmd(command)

        return

    def send_cmd(self, msg: str):
        if (
            self.launch_mode == "anchor"
        ):  # if in anchor mode, send to anchor node to relay
            output = String()
            output.data = msg
            self.anchor_pub.publish(output)
        elif self.launch_mode == "arm":  # if in standalone mode, send to MCU directly
            self.get_logger().info(f"[Arm to MCU] {msg}")
            self.ser.write(bytes(msg, "utf8"))

    def anchor_feedback(self, msg: String):
        output = msg.data
        if output.startswith("can_relay_fromvic,arm,55"):
            self.updateAngleFeedback(output)
        elif output.startswith("can_relay_fromvic,arm,54"):
            self.updateBusVoltage(output)
        elif output.startswith("can_relay_fromvic,arm,53"):
            self.updateMotorFeedback(output)
        elif output.startswith("can_relay_fromvic,digit,54"):
            parts = msg.data.split(",")
            if len(parts) >= 7:
                # Extract the voltage from the string
                voltages_in = parts[3:7]
                # Convert the voltages to floats
                self.digit_feedback.bat_voltage = float(voltages_in[0]) / 100.0
                self.digit_feedback.voltage_12 = float(voltages_in[1]) / 100.0
                self.digit_feedback.voltage_5 = float(voltages_in[2]) / 100.0
        elif output.startswith("can_relay_fromvic,digit,55"):
            parts = msg.data.split(",")
            if len(parts) >= 4:
                self.digit_feedback.wrist_angle = float(parts[3])
                # self.digit_feedback.wrist_roll = float(parts[4])
        else:
            return

    def relay_fromvic(self, msg: VicCAN):
        # Code for socket and digit are broken out for cleaner code
        if msg.mcu_name == "arm":
            self.process_fromvic_arm(msg)
        elif msg.mcu_name == "digit":
            self.process_fromvic_digit(msg)

    def process_fromvic_arm(self, msg: VicCAN):
        if msg.mcu_name != "arm":
            return

        # Check message len to prevent crashing on bad data
        if msg.command_id in viccan_socket_msg_len_dict:
            expected_len = viccan_socket_msg_len_dict[msg.command_id]
            if len(msg.data) != expected_len:
                self.get_logger().warning(
                    f"Ignoring VicCAN message with id {msg.command_id} due to unexpected data length (expected {expected_len}, got {len(msg.data)})"
                )
                return

        match msg.command_id:
            case 53:  # REV SPARK MAX feedback
                motorId = round(msg.data[0])
                motor: RevMotorState | None = None
                match motorId:
                    case 1:
                        motor = self.arm_feedback_new.axis1_motor
                    case 2:
                        motor = self.arm_feedback_new.axis2_motor
                    case 3:
                        motor = self.arm_feedback_new.axis3_motor
                    case 4:
                        motor = self.arm_feedback_new.axis0_motor

                if motor:
                    motor.temperature = float(msg.data[1]) / 10.0
                    motor.voltage = float(msg.data[2]) / 10.0
                    motor.current = float(msg.data[3]) / 10.0
                    motor.header.stamp = msg.header.stamp

                self.arm_feedback_pub_.publish(self.arm_feedback_new)
            case 54:  # Board voltages
                self.arm_feedback_new.socket_voltage.vbatt = float(msg.data[0]) / 100.0
                self.arm_feedback_new.socket_voltage.v12 = float(msg.data[1]) / 100.0
                self.arm_feedback_new.socket_voltage.v5 = float(msg.data[2]) / 100.0
                self.arm_feedback_new.socket_voltage.v3 = float(msg.data[3]) / 100.0
            case 55:  # Arm joint positions
                angles = [angle / 10.0 for angle in msg.data]  # VicCAN sends deg*10
                # Joint state publisher for URDF visualization
                self.saved_joint_state.position[0] = math.radians(angles[0])  # Axis 0
                self.saved_joint_state.position[1] = math.radians(angles[1])  # Axis 1
                self.saved_joint_state.position[2] = math.radians(
                    -angles[2]
                )  # Axis 2 (inverted)
                self.saved_joint_state.position[3] = math.radians(
                    -angles[3]
                )  # Axis 3 (inverted)
                # Wrist is handled by digit feedback
                self.saved_joint_state.header.stamp = msg.header.stamp
                self.joint_state_pub_.publish(self.saved_joint_state)
            case 58:  # REV SPARK MAX position and velocity feedback
                motorId = round(msg.data[0])
                motor: RevMotorState | None = None
                match motorId:
                    case 1:
                        motor = self.arm_feedback_new.axis1_motor
                    case 2:
                        motor = self.arm_feedback_new.axis2_motor
                    case 3:
                        motor = self.arm_feedback_new.axis3_motor
                    case 4:
                        motor = self.arm_feedback_new.axis0_motor

                if motor:
                    motor.position = float(msg.data[1])
                    motor.velocity = float(msg.data[2])
                    motor.header.stamp = msg.header.stamp

                self.arm_feedback_pub_.publish(self.arm_feedback_new)
            case 59:  # Arm joint velocities
                velocities = [vel / 100.0 for vel in msg.data]  # VicCAN sends deg/s*100
                self.saved_joint_state.velocity[0] = math.radians(
                    velocities[0]
                )  # Axis 0
                self.saved_joint_state.velocity[1] = math.radians(
                    velocities[1]
                )  # Axis 1
                self.saved_joint_state.velocity[2] = math.radians(
                    -velocities[2]
                )  # Axis 2 (-)
                self.saved_joint_state.velocity[3] = math.radians(
                    -velocities[3]
                )  # Axis 3 (-)
                # Wrist is handled by digit feedback
                self.saved_joint_state.header.stamp = msg.header.stamp
                self.joint_state_pub_.publish(self.saved_joint_state)

    def process_fromvic_digit(self, msg: VicCAN):
        if msg.mcu_name != "digit":
            return

        # Check message len to prevent crashing on bad data
        if msg.command_id in viccan_digit_msg_len_dict:
            expected_len = viccan_digit_msg_len_dict[msg.command_id]
            if len(msg.data) != expected_len:
                self.get_logger().warning(
                    f"Ignoring VicCAN message with id {msg.command_id} due to unexpected data length (expected {expected_len}, got {len(msg.data)})"
                )
                return

        match msg.command_id:
            case 54:  # Board voltages
                self.arm_feedback_new.digit_voltage.vbatt = float(msg.data[0]) / 100.0
                self.arm_feedback_new.digit_voltage.v12 = float(msg.data[1]) / 100.0
                self.arm_feedback_new.digit_voltage.v5 = float(msg.data[2]) / 100.0
            case 55:  # Arm joint positions
                self.saved_joint_state.position[4] = math.radians(
                    msg.data[0]
                )  # Wrist roll
                self.saved_joint_state.position[5] = math.radians(
                    msg.data[1]
                )  # Wrist yaw

    def publish_feedback(self):
        self.socket_pub.publish(self.arm_feedback)
        self.digit_pub.publish(self.digit_feedback)

    def updateAngleFeedback(self, msg: str):
        # Angle feedbacks,
        # split the msg.data by commas
        parts = msg.split(",")

        if len(parts) >= 7:
            # Extract the angles from the string
            angles_in = parts[3:7]
            # Convert the angles to floats divide by 10.0
            angles = [float(angle) / 10.0 for angle in angles_in]

            self.arm_feedback.axis0_angle = angles[0]
            self.arm_feedback.axis1_angle = angles[1]
            self.arm_feedback.axis2_angle = angles[2]
            self.arm_feedback.axis3_angle = angles[3]
        else:
            self.get_logger().info("Invalid angle feedback input format")

    def updateBusVoltage(self, msg: str):
        # Bus Voltage feedbacks
        parts = msg.split(",")
        if len(parts) >= 7:
            # Extract the voltage from the string
            voltages_in = parts[3:7]
            # Convert the voltages to floats
            self.arm_feedback.bat_voltage = float(voltages_in[0]) / 100.0
            self.arm_feedback.voltage_12 = float(voltages_in[1]) / 100.0
            self.arm_feedback.voltage_5 = float(voltages_in[2]) / 100.0
            self.arm_feedback.voltage_3 = float(voltages_in[3]) / 100.0
        else:
            self.get_logger().info("Invalid voltage feedback input format")

    def updateMotorFeedback(self, msg: str):
        parts = str(msg.strip()).split(",")
        motorId = round(float(parts[3]))
        temp = float(parts[4]) / 10.0
        voltage = float(parts[5]) / 10.0
        current = float(parts[6]) / 10.0
        if motorId == 1:
            self.arm_feedback.axis1_temp = temp
            self.arm_feedback.axis1_voltage = voltage
            self.arm_feedback.axis1_current = current
        elif motorId == 2:
            self.arm_feedback.axis2_temp = temp
            self.arm_feedback.axis2_voltage = voltage
            self.arm_feedback.axis2_current = current
        elif motorId == 3:
            self.arm_feedback.axis3_temp = temp
            self.arm_feedback.axis3_voltage = voltage
            self.arm_feedback.axis3_current = current
        elif motorId == 4:
            self.arm_feedback.axis0_temp = temp
            self.arm_feedback.axis0_voltage = voltage
            self.arm_feedback.axis0_current = current

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        # return glob.glob("/dev/tty[A-Za-z]*")

    def cleanup(self):
        print("Cleaning up...")
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception as e:
            exit(0)


def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)
    if serial_pub:
        serial_pub.cleanup()


def main(args=None):
    rclpy.init(args=args)
    sys.excepthook = myexcepthook

    global serial_pub
    serial_pub = SerialRelay()
    serial_pub.run()


if __name__ == "__main__":
    # signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(
        signal.SIGTERM, lambda signum, frame: sys.exit(0)
    )  # Catch termination signals and exit cleanly
    main()
