import rclpy
from rclpy.node import Node

import serial
import sys
import threading
import glob
import time


from std_msgs.msg import String
from interfaces_pkg.msg import ControllerState
from interfaces_pkg.msg import ArmState



serial_pub = None
thread = None


class SerialRelay(Node):
    def __init__(self):
        # Initalize node with name
        super().__init__("arm_relay")

        # Create a publisher to publish any output the mcu sends
        self.output_publisher = self.create_publisher(String, '/astra/arm/feedback', 10)#Random String feedback data
        self.state_publisher = self.create_publisher(ArmState, '/astra/arm/state', 10)#Standard state feedback data

        # Create a subscriber to listen to any commands sent for the mcu
        self.subscriber = self.create_subscription(ControllerState, '/astra/arm/control', self.send_controls, 10)

        # Loop through all serial devices on the computer to check for the mcu
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for port in ports:
            try:
                # connect and send a ping command
                ser = serial.Serial(port, timeout=1)
                ser.write(b"arm,ping\n")
                response = ser.read_until("\n")

                # if pong is in response, then we are talking with the mcu
                if b"pong" in response:
                    self.port = port
                    print(f"Found MCU at {self.port}!")
                    break
            except:
                pass
        
        if self.port is None:
           print("Unable to find MCU... please make sure it is connected.")
           #sys.exit(1) TEMPORARY TEST, UNCOMMENT THIS LINE IN THE FUTURE
        
        self.ser = serial.Serial(self.port, 115200)

        #self.mutex = threading.Lock()

    def run(self):
        # This thread makes all the update processes run in the background
        global thread
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon = True)
        thread.start()
        
        try:
            while rclpy.ok():
                # Check the mcu for updates
                #rclpy.spin_once(self, timeout_sec=0.1)
                #self.mutex.acquire()
                if self.ser.in_waiting:
                    #self.mutex.release()
                    self.read_mcu()
                else:
                    time.sleep(0.1)

        except KeyboardInterrupt:
            #self.mutex.release()
            serial_pub.ser.close()
            sys.exit(0)
        

    def read_mcu(self):
        try:
            #self.mutex.acquire()
            output = str(self.ser.readline(), "utf8")
            if output:
                print(f"[MCU] {output}", end="")
                # Create a string message object
                msg = String()

                # Set message data
                msg.data = output

                # Publish data
                self.output_publisher.publish(msg)
                #print(f"[MCU] Publishing: {msg}")

        except serial.SerialException:
            #self.mutex.release()
            pass
        #finally:
            #self.mutex.release()

    def send_controls(self, msg):
        command = ""
        ef_cmd = "" #end effector command to be apended
        #self.mutex.acquire()
        if(msg.b):#If B button: send ESTOP command
            command = "arm,stop\n"
            self.ser.write(bytes(command, "utf8"))#Send command to MCU
            print(f"[Wrote] {command}", end="")#Echo command to console

            #self.mutex.release()#Release mutex lock
            return 
        
        if(msg.plus):#Turn EF laser on
            command = "arm,endEffect,laser,1\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
        elif(msg.minus):#Turn EF laser off
            command = "arm,endEffect,laser,0\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
        
        if(msg.rb):#If RB button: End Effector control mode
            ef_cmd = "arm,endEffect,ctrl,"
            
            if(msg.lt >= 0.5):#If LT button: Open end effector
                ef_cmd += "-1,"
            elif(msg.rt >= 0.5):#If RT button: Close end effector
                ef_cmd += "1,"
            else:
                ef_cmd += "0,"

            if(msg.rs_x < 0): #Check left/stop/right for tilt movement
                ef_cmd += "-1,"
            elif(msg.rs_x > 0):
                ef_cmd += "1,"
            else:
                ef_cmd += "0,"
            pass
            
            if(msg.ls_x < 0): #Check left/stop/right for revolve movement
                ef_cmd += "-1"
            elif(msg.ls_x > 0):
                ef_cmd += "1"
            else:
                ef_cmd += "0"

            command = ef_cmd + "\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")         
            #self.mutex.release()   
            return
        else:
            ef_cmd = "arm,endEffect,ctrl,"
            if(msg.lt >= 0.5):
                ef_cmd += "-1,0,0"
            elif(msg.rt >= 0.5):
                ef_cmd += "1,0,0"
            else:
                ef_cmd += "0,0,0"
            command = ef_cmd + "\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
        
        if(msg.lb):#If LB button: Manual control mode
            #First, ensure control mode is set to manual on the MCU
            self.ser.write(bytes("arm,setMode,manual\n", "utf8"))
            #print(f"[Wrote] arm,setMode,manual", end="")
            
            command = "arm,man,0.15,"#Set manual control duty cycle statically to 15% for now
            if(msg.d_left):#If D-Pad left set axis_1 to -1
                command += "-1,"
            elif(msg.d_right):#If D-Pad right set axis_0 to 1
                command += "1,"
            else:
                command += "0,"

            #Axis_1
            if(msg.ls_x < -0.5):#If LS left at least 50% set axis_1 to -1
                command += "-1,"
            elif(msg.ls_x > 0.5):#If LS right at least 50% set axis_1 to 1
                command += "1,"
            else:
                command += "0,"

            #Axis_2 
            if(msg.ls_y < -0.5):#If LS up at least 50% set axis_2 to 1
                command += "1,"
            elif(msg.ls_y > 0.5):#If LS down at least 50% set axis_2 to -1
                command += "-1,"
            else:
                command += "0,"

            #Axis_3
            if(msg.rs_y < -0.5):#If RS up at least 50% set axis_3 to 1
                command += "1"
            elif(msg.rs_y > 0.5):#If RS down at least 50% set axis_3 to -1
                command += "-1"
            else:
                command += "0"

            command += "\n"
            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")

            #self.mutex.release()
            return
        else:#Else normal (IK) control mode
            #First, ensure control mode is set to IK on the MCU
            self.ser.write(bytes("arm,setMode,ik\n", "utf8"))
            #print(f"[Wrote] arm,setMode,ik", end="")

            command = "arm,ik,"
            if(msg.d_left):#If D-Pad left set axis zero to -1
                command += "-1,"
            elif(msg.d_right):#If D-Pad right set axis zero to 1
                command += "1,"
            else:
                command += "0," 

            #start with input from controller
            coord_x = -1*msg.ls_y #out/in
            coord_y = -1*msg.rs_y #up/down

            #convert units for ik coordinate offsets
            coord_x = round(coord_x * 20.4 * 2, 1) #20.4mm (1 in.) per 1 unit of input. Times 2 for two inches
            coord_y = round(coord_y * 20.4 * 2, 1) #

            command += f"{coord_x},{coord_y}\n"

            self.ser.write(bytes(command, "utf8"))
            print(f"[Wrote] {command}", end="")
            #self.mutex.release()
            return
        

    @staticmethod
    def list_serial_ports():
        return glob.glob("/dev/tty[A-Za-z]*")
        

def myexcepthook(type, value, tb):
    print("Uncaught exception:", type, value)
    serial_pub.ser.close()


def main(args=None):
    rclpy.init(args=args)
    sys.excepthook = myexcepthook

    global serial_pub
    serial_pub = SerialRelay()
    serial_pub.run()


if __name__ == '__main__':
    main()