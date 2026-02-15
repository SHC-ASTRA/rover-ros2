from abc import ABC, abstractmethod
import serial
import serial.tools.list_ports
from astra_msgs.msg import VicCAN
from rclpy.impl.rcutils_logger import RcutilsLogger
from .convert import string_to_viccan

KNOWN_USBS = [
    (0x2E8A, 0x00C0),  # Raspberry Pi Pico
    (0x1A86, 0x55D4),  # Adafruit Feather ESP32 V2
    (0x10C4, 0xEA60),  # DOIT ESP32 Devkit V1
    (0x1A86, 0x55D3),  # ESP32 S3 Development Board
]
BAUD_RATE = 115200


class NoValidDeviceException(Exception):
    pass


class NoWorkingDeviceException(Exception):
    pass


class MultipleValidDevicesException(Exception):
    pass


class DeviceClosedException(Exception):
    pass


class Connector(ABC):
    logger: RcutilsLogger

    @abstractmethod
    def read(self) -> VicCAN | None:
        pass

    @abstractmethod
    def write(self, msg: VicCAN):
        pass

    def cleanup(self):
        pass


class SerialConnector(Connector):
    port: str
    mcu_name: str
    serial_interface: serial.Serial
    override: bool

    def _get_name(self, port: str) -> str | None:
        """
        Get the name of the MCU (if it works)

        returns: str name of the MCU, None if it doesn't work
        """
        # attempt to open the serial port
        serial_interface: serial.Serial
        try:
            self.logger.info(f"asking {port} for its name")
            serial_interface = serial.Serial(port, BAUD_RATE, timeout=1)

            for i in range(4):
                self.logger.debug(f"attempt {i + 1} of 4 asking {port} for its name")
                response = serial_interface.read_until(bytes("\n", "utf8"))
                try:
                    if b"can_relay_ready" in response:
                        args = response.decode("utf8").strip().split(",")
                        if len(args) == 2:
                            return args[1]
                        break
                except UnicodeDecodeError as e:
                    self.logger.debug(
                        f"ignoring UnicodeDecodeError when asking for MCU name: {e}"
                    )

            if serial_interface.is_open:
                serial_interface.close()
        except serial.SerialException as e:
            self.logger.error(f"SerialException when asking for MCU name: {e}")

        return None

    def _find_ports(self) -> list[str]:
        """
        Finds all valid ports but does not test them

        returns: all valid ports
        """
        comports = serial.tools.list_ports.comports()
        valid_ports = list(
            map(  # get just device strings
                lambda p: p.device,
                filter(  # make sure we have a known device
                    lambda p: (p.vid, p.pid, p.device) in KNOWN_USBS
                    and p.device is not None,
                    comports,
                ),
            )
        )
        self.logger.debug(f"found valid MCU ports: [ {', '.join(valid_ports)} ]")
        return valid_ports

    def cleanup(self):
        self.logger.info(f"closing serial port if open {self.port}")
        try:
            if self.serial_interface.is_open:
                self.serial_interface.close()
        except Exception as e:
            self.logger.error(e)

    def __init__(self, logger: RcutilsLogger):
        self.logger = logger

        ports = self._find_ports()

        if len(ports) <= 0:
            raise NoValidDeviceException("no valid serial device found")
        if (l := len(ports)) > 1:
            raise MultipleValidDevicesException(
                f"too many ({l}) valid serial devices found"
            )

        # check each of our ports to make sure one of them is responding
        port = ports[0]
        mcu_name = "mock" if self.override else self._get_name(port)
        if not mcu_name:
            raise NoWorkingDeviceException(
                f"found {port}, but it did not respond with its name"
            )

        self.port = port
        self.mcu_name = mcu_name

        # if we fail at this point, it should crash because we've already tested the port
        self.serial_interface = serial.Serial(self.port, BAUD_RATE, timeout=1)

    def read(self) -> VicCAN | None:
        try:
            raw = str(self.serial_interface.readline(), "utf8")

            if not raw:
                return None

            return string_to_viccan(raw, self.mcu_name, self.logger)
        except serial.SerialException as e:
            self.logger.error(f"SerialException: {e}")
            raise DeviceClosedException(f"serial port {self.port} closed unexpectedly")
        except TypeError as e:
            self.logger.error(f"TypeError: {e}")
            raise DeviceClosedException(f"serial port {self.port} closed unexpectedly")
        except Exception:
            pass  # pretty much no other error matters

    def write(self, msg: VicCAN):
        # go from [ w, x, y, z ] -> "w,x,y,z" & round to 7 digits max
        data = ",".join([str(round(x, 7)) for x in msg.data])
        output = f"can_relay_tovic,{msg.mcu_name},{msg.command_id},{data}\n"
        self.serial_interface.write(bytes(output, "utf8"))


class CANConnector(Connector):
    def __init__(self, logger: RcutilsLogger):
        pass


class MockConnector(Connector):
    def __init__(self, _: RcutilsLogger):
        pass

    def read(self) -> VicCAN | None:
        return None

    def write(self, msg: VicCAN):
        print(msg)
