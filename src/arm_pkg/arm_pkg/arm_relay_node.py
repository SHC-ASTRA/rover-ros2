import rclpy
from rclpy.node import Node
import serial
import sys
import threading
import glob
import time
import atexit
import signal
from std_msgs.msg import String
from ros2_interfaces_pkg.msg import ArmManual
from ros2_interfaces_pkg.msg import ArmIK
from ros2_interfaces_pkg.msg import SocketFeedback

serial_pub = None
thread = None

class SerialRelay(Node):
    def __init__(self):
        # Initialize node with name
        super().__init__("arm_relay")

        # Create publishers

        #Depricated, temporary for reference
        # self.output_publisher = self.create_publisher(String, '/astra/arm/feedback', 10)
        # self.state_publisher = self.create_publisher(ArmState, '/astra/arm/state', 10)
        # self.faerie_publisher = self.create_publisher(FaerieTelemetry, '/astra/arm/bio/feedback', 10)

        self.debug_pub = self.create_publisher(String, '/arm/feedback/debug', 10)
        self.socket_pub = self.create_publisher(SocketFeedback, '/arm/feedback/socket', 10)


        # Create subscribers

        #Depricated, temporary for reference
        #self.control_subscriber = self.create_subscription(ControllerState, '/astra/arm/control', self.send_controls, 10)
        
        self.ik_sub = self.create_subscription(ArmIK, '/arm/control/ik', self.send_ik, 10) 
        self.man_sub = self.create_subscription(ArmManual, '/arm/control/manual', self.send_manual, 10)

        #self.command_subscriber = self.create_subscription(String, '/astra/arm/command', self.send_command, 10)
        #self.faerie_subscriber = self.create_subscription(String, "/astra/arm/bio/control", self.send_faerie_controls, 10)

        # Loop through all serial devices on the computer to check for the MCU
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for i in range(2):
            for port in ports:
                try:
                    # connect and send a ping command
                    ser = serial.Serial(port, 115200, timeout=1)
                    print(f"Checking port {port}...")
                    ser.write(b"ping\n")
                    response = ser.read_until("\n")

                    # if pong is in response, then we are talking with the MCU
                    if b"pong" in response:
                        self.port = port
                        print(f"Found MCU at {self.port}!")
                        break
                except:
                    pass
            if self.port is not None:
                break
        
        if self.port is None:
            print("Unable to find MCU... please make sure it is connected.")
            time.sleep(1)
            sys.exit(1)
        
        self.ser = serial.Serial(self.port, 115200)
        atexit.register(self.cleanup)

    def run(self):
        global thread
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()
        
        try:
            while rclpy.ok():
                if self.ser.in_waiting:
                    self.read_mcu()
                else:
                    time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()


    #Currently will just spit out all values over the /arm/feedback/debug topic as strings
    def read_mcu(self):
        try:
            output = str(self.ser.readline(), "utf8")
            if output:
                print(f"[MCU] {output}", end="")
                msg = String()
                msg.data = output
                self.debug_pub.publish(msg)
        except serial.SerialException:
            print("SerialException caught... closing serial port.")
            if self.ser.is_open:
                self.ser.close()
            pass
        except TypeError as e:
            print(f"TypeError: {e}")
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

    def send_ik(self, msg):
        pass

    def send_manual(self, msg):
        axis0 = msg.axis0
        axis1 = msg.axis1
        axis2 = msg.axis2
        axis3 = msg.axis3
        #Send controls for arm
        command = "can_relay_tovic,arm,40," + str(axis0) + "," + str(axis1) + "," + str(axis2) + "," + str(axis3) + "\n"
        self.ser.write(bytes(command, "utf8"))
        print(f"[Wrote] {command}", end="")

    #Not yet finished, needs embedded implementation for new commands
        # ef_roll = msg.effector_roll
        # ef_yaw = msg.effector_yaw
        # gripper = msg.gripper
        # actuator = msg.linear_actuator
        # laser = msg.laser
        # #Send controls for digit

        # command = "can_relay_tovic,digit," + str(ef_roll) + "," + str(ef_yaw) + "," + str(gripper) + "," + str(actuator) + "," + str(laser) + "\n"

        return
    

    # Depricated functions, kept temporarily for reference

    # def send_controls(self, msg):
    #     command = ""
    #     ef_cmd = ""
    #     if(msg.b):
    #         command = "arm,stop\n"
    #         self.ser.write(bytes(command, "utf8"))
    #         print(f"[Wrote] {command}", end="")
    #         return 
        
    #     if(msg.a):
    #         command = "arm,endEffect,act,0"#retract actuator out
    #         self.ser.write(bytes(command, "utf8"))
    #         print(f"[Wrote] {command}", end="")
    #     elif(msg.x):
    #         command = "arm,endEffect,act,1"#extend actuator in
    #         self.ser.write(bytes(command, "utf8"))
    #         print(f"[Wrote] {command}", end="")

    #     if(msg.plus):
    #         command = "arm,endEffect,laser,1\n"
    #         self.ser.write(bytes(command, "utf8"))
    #         print(f"[Wrote] {command}", end="")
    #     elif(msg.minus):
    #         command = "arm,endEffect,laser,0\n"
    #         self.ser.write(bytes(command, "utf8"))
    #         print(f"[Wrote] {command}", end="")
        
    #     if(msg.rb):
    #         ef_cmd = "arm,endEffect,ctrl,"
    #         if(msg.lt >= 0.5):
    #             ef_cmd += "-1,"
    #         elif(msg.rt >= 0.5):
    #             ef_cmd += "1,"
    #         else:
    #             ef_cmd += "0,"

    #         if(msg.rs_x < 0):
    #             ef_cmd += "-1,"
    #         elif(msg.rs_x > 0):
    #             ef_cmd += "1,"
    #         else:
    #             ef_cmd += "0,"
    #         if(msg.ls_x < 0):
    #             ef_cmd += "-1"
    #         elif(msg.ls_x > 0):
    #             ef_cmd += "1"
    #         else:
    #             ef_cmd += "0"

    #         command = ef_cmd + "\n"
    #         self.ser.write(bytes(command, "utf8"))
    #         print(f"[Wrote] {command}", end="")
    #         return
    #     else:
    #         ef_cmd = "arm,endEffect,ctrl,"
    #         if(msg.lt >= 0.5):
    #             ef_cmd += "-1,0,0"
    #         elif(msg.rt >= 0.5):
    #             ef_cmd += "1,0,0"
    #         else:
    #             ef_cmd += "0,0,0"
    #         command = ef_cmd + "\n"
    #         self.ser.write(bytes(command, "utf8"))
    #         print(f"[Wrote] {command}", end="")
        
    #     if(msg.lb):
    #         #self.ser.write(bytes("arm,setMode,manual\n", "utf8"))
    #         command = "arm,man,0.25,"
    #     else:
    #         #self.ser.write(bytes("arm,setMode,manual\n", "utf8"))
    #         command = "arm,man,0.15,"

    #     if(msg.d_left):
    #         command += "-1,"
    #     elif(msg.d_right):
    #         command += "1,"
    #     else:
    #         command += "0,"
    #     if(msg.ls_x < -0.4):
    #         command += "1,"
    #     elif(msg.ls_x > 0.4):
    #         command += "-1,"
    #     else:
    #         command += "0,"
    #     if(msg.ls_y < -0.4):
    #         command += "1,"
    #     elif(msg.ls_y > 0.4):
    #         command += "-1,"
    #     else:
    #         command += "0,"
    #     if(msg.rs_y < -0.4):
    #         command += "1"
    #     elif(msg.rs_y > 0.4):
    #         command += "-1"
    #     else:
    #         command += "0"

    #     command += "\n"
    #     self.ser.write(bytes(command, "utf8"))
    #     print(f"[Wrote] {command}", end="")
    #     return

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        #return glob.glob("/dev/tty[A-Za-z]*")

    def cleanup(self):
        print("Cleaning up...")
        if self.ser.is_open:
            self.ser.close()

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

if __name__ == '__main__':
    signal.signal(signal.SIGTSTP, lambda signum, frame: sys.exit(0))  # Catch Ctrl+Z and exit cleanly
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))  # Catch termination signals and exit cleanly
    main()
