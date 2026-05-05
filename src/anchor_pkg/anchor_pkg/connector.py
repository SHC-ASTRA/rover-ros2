from abc import ABC, abstractmethod
from time import monotonic
from typing import TYPE_CHECKING

from std_msgs.msg import String, Header
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger
from astra_msgs.msg import VicCAN
from .convert import string_to_viccan as _string_to_viccan, viccan_to_string

# CAN
import can
import can.interfaces.socketcan
import struct

# Serial
import serial
import serial.tools.list_ports

KNOWN_USBS = [
    (0x2E8A, 0x00C0),  # Raspberry Pi Pico
    (0x1A86, 0x55D4),  # Adafruit Feather ESP32 V2
    (0x10C4, 0xEA60),  # DOIT ESP32 Devkit V1
    (0x1A86, 0x55D3),  # ESP32 S3 Development Board
]

BAUD_RATE = 115200

SERIAL_READ_TIMEOUT = 0.5  # seconds

MCU_IDS = {
    1: "broadcast",
    2: "core",
    3: "arm",
    4: "digit",
    5: "faerie",
    6: "citadel",
    7: "libs",
}


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
    clock: Clock

    def string_to_viccan(self, msg: str, mcu_name: str):
        """function currying so that we do not need to pass logger and clock every time"""
        return _string_to_viccan(
            msg,
            mcu_name,
            self.logger,
            self.clock.now().to_msg(),
        )

    def __init__(self, logger: RcutilsLogger, clock: Clock):
        self.logger = logger
        self.clock = clock

    @abstractmethod
    def read(self) -> tuple[VicCAN | None, str | None]:
        """
        Must return a tuple of (VicCAN, debug message or string repr of VicCAN)
        """
        pass

    @abstractmethod
    def write(self, msg: VicCAN):
        pass

    @abstractmethod
    def write_raw(self, msg: String):
        pass

    def cleanup(self):
        pass


class SerialConnector(Connector):
    port: str
    mcu_name: str
    serial_interface: serial.Serial

    def __init__(self, logger: RcutilsLogger, clock: Clock, serial_override: str = ""):
        super().__init__(logger, clock)

        ports = self._find_ports()
        mcu_name: str | None = None

        # Serial buffering
        self._serial_buffer: bytes = b""
        self._last_read_time = monotonic()

        if serial_override:
            logger.warn(
                f"using serial_override: `{serial_override}`! this will bypass several checks."
            )
            ports = [serial_override]
            mcu_name = "override"

        if len(ports) <= 0:
            raise NoValidDeviceException("no valid serial device found")
        if (l := len(ports)) > 1:
            raise MultipleValidDevicesException(
                f"too many ({l}) valid serial devices found"
            )

        # check each of our ports to make sure one of them is responding
        port = ports[0]
        # we might already have a name by now if we overrode earlier
        mcu_name = mcu_name or self._get_name(port)
        if not mcu_name:
            raise NoWorkingDeviceException(
                f"found {port}, but it did not respond with its name"
            )

        self.port = port
        self.mcu_name = mcu_name

        # if we fail at this point, it should crash because we've already tested the port
        self.serial_interface = serial.Serial(self.port, BAUD_RATE, timeout=0)

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
                    lambda p: (p.vid, p.pid) in KNOWN_USBS and p.device is not None,
                    comports,
                ),
            )
        )
        self.logger.info(f"found valid MCU ports: [ {', '.join(valid_ports)} ]")
        return valid_ports

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

            serial_interface.write(b"can_relay_mode,on\n")

            for i in range(4):
                self.logger.debug(f"attempt {i + 1} of 4 asking {port} for its name")
                response = serial_interface.read_until(bytes("\n", "utf8"))
                try:
                    if b"can_relay_ready" in response:
                        args: list[str] = response.decode("utf8").strip().split(",")
                        if len(args) == 2:
                            self.logger.info(f"we are talking to {args[1]}")
                            return args[1]
                        break
                except UnicodeDecodeError as e:
                    self.logger.info(
                        f"ignoring UnicodeDecodeError when asking for MCU name: {e}"
                    )

            if serial_interface.is_open:
                # turn relay mode off if it failed to respond with its name
                serial_interface.write(b"can_relay_mode,off\n")
                serial_interface.close()
        except serial.SerialException as e:
            self.logger.error(f"SerialException when asking for MCU name: {e}")

        return None

    def _try_readline(self) -> str | None:
        """Attempts to read a full string from the MCU without blocking.

        When pyserial is used with 'timeout=0', reads are performed non-blocking.
        When used with interface.readline(), this breaks the assumption that a returned
        string will be a completed attempt by the MCU to send a string; it may be
        cut off between the start of the string and the newline, removing information
        and rendering the string(s) useless; thus, to get around the downside of readline()
        not waiting for a newline while still not blocking, this function manually
        implements a serial input buffer and newline timeout.

        If readline() returns a non-empty string, send it if it ends with a newline
        (readline() will not read past any newline); otherwise, save the read string.
        This buffered string should be pre-pended to the next readline() result.

        If readline() does not receive a non-empty string after the last non-newline-
        terminated readline() result within the manual timeout, send the contents of the
        buffer as if it ended with a newline, and clear the buffer.

        Returns:
            str: A hopefully-complete string read from the MCU via the serial interface.
        """
        if TYPE_CHECKING:
            assert type(self.serial_interface) == serial.Serial

        # Warn on buffer timeout, as the only scenarios that would trigger this are
        # a microcontroller output that isn't newline-terminated (bad), or the MCU is
        # hanging (also bad).
        if (
            self._serial_buffer
            and (monotonic() - self._last_read_time) > SERIAL_READ_TIMEOUT
        ):
            self.logger.warn(
                f"Serial buffer timeout, last received '{self._serial_buffer}'."
            )
            result = self._serial_buffer
            self._serial_buffer = b""
            self._last_read_time = monotonic()
            return str(result, "utf8").strip()

        # No try-except here so caller catches it instead.
        raw = self.serial_interface.readline()

        # Empty or whitespace-only string
        if not raw or not raw.strip():
            return None

        # Add to buffer or send finished buffer
        if not (raw.endswith(b"\n") or raw.endswith(b"\r")):  # unfinished string
            self._serial_buffer += raw
            self._last_read_time = monotonic()
            return None
        else:
            result = self._serial_buffer + raw
            self._serial_buffer = b""
            return str(result, "utf8").strip()

    def read(self) -> tuple[VicCAN | None, str | None]:
        try:
            raw = self._try_readline()

            if not raw:
                return (None, None)

            return (
                self.string_to_viccan(raw, self.mcu_name),
                raw,
            )
        except serial.SerialException as e:
            self.logger.error(f"SerialException: {e}")
            raise DeviceClosedException(f"serial port {self.port} closed unexpectedly")
        except Exception:
            return (None, None)  # pretty much no other error matters

    def write(self, msg: VicCAN):
        self.write_raw(String(data=viccan_to_string(msg)))

    def write_raw(self, msg: String):
        self.serial_interface.write(bytes(msg.data, "utf8"))

    def cleanup(self):
        self.logger.info(f"closing serial port if open {self.port}")
        try:
            if self.serial_interface.is_open:
                self.serial_interface.close()
        except Exception as e:
            self.logger.error(e)


class CANConnector(Connector):
    def __init__(self, logger: RcutilsLogger, clock: Clock, can_override: str):
        super().__init__(logger, clock)

        self.can_channel: str | None = None
        self.can_bus: can.BusABC | None = None

        avail = can.interfaces.socketcan.SocketcanBus._detect_available_configs()

        if len(avail) == 0:
            raise NoValidDeviceException("no CAN interfaces found")

        # filter to busses whose channel matches the can_override
        if can_override:
            self.logger.info(f"overrode can interface with {can_override}")
            avail = list(
                filter(
                    lambda b: b.get("channel") == can_override,
                    avail,
                )
            )

        if (l := len(avail)) > 1:
            channels = ", ".join(str(b.get("channel")) for b in avail)
            raise MultipleValidDevicesException(
                f"too many ({l}) CAN interfaces found: [{channels}]"
            )

        bus = avail[0]
        self.can_channel = str(bus.get("channel"))
        self.logger.info(f"found CAN interface '{self.can_channel}'")

        try:
            self.can_bus = can.Bus(
                interface="socketcan",
                channel=self.can_channel,
                bitrate=1_000_000,
            )
        except can.CanError as e:
            raise NoWorkingDeviceException(
                f"could not open CAN channel '{self.can_channel}': {e}"
            )

        if self.can_channel and self.can_channel.startswith("v"):
            self.logger.warn("CAN interface is likely virtual")

    def read(self) -> tuple[VicCAN | None, str | None]:
        if not self.can_bus:
            raise DeviceClosedException("CAN bus not initialized")

        try:
            message = self.can_bus.recv(timeout=0.0)
        except can.CanError as e:
            self.logger.error(f"CAN error while receiving: {e}")
            raise DeviceClosedException("CAN bus closed unexpectedly")

        if message is None:
            return (None, None)

        arbitration_id = message.arbitration_id & 0x7FF
        data_bytes = bytes(message.data)

        mcu_key = (arbitration_id >> 8) & 0b111
        data_type_key = (arbitration_id >> 6) & 0b11
        command = arbitration_id & 0x3F

        try:
            mcu_name = MCU_IDS[mcu_key]
        except IndexError:
            self.logger.warn(
                f"received CAN frame with unknown MCU key {mcu_key}; id=0x{arbitration_id:X}"
            )
            return (None, None)

        data: list[float] = []

        try:
            if data_type_key == 3:
                data = []
            elif data_type_key == 0:
                if len(data_bytes) < 8:
                    self.logger.warn(
                        f"received double payload with insufficient length {len(data_bytes)}; dropping frame"
                    )
                    return (None, None)
                (value,) = struct.unpack(">d", data_bytes[:8])
                data = [float(value)]
            elif data_type_key == 1:
                if len(data_bytes) < 8:
                    self.logger.warn(
                        f"received float32x2 payload with insufficient length {len(data_bytes)}; dropping frame"
                    )
                    return (None, None)
                v1, v2 = struct.unpack(">ff", data_bytes[:8])
                data = [float(v1), float(v2)]
            elif data_type_key == 2:
                if len(data_bytes) < 8:
                    self.logger.warn(
                        f"received int16x4 payload with insufficient length {len(data_bytes)}; dropping frame"
                    )
                    return (None, None)
                i1, i2, i3, i4 = struct.unpack(">hhhh", data_bytes[:8])
                data = [float(i1), float(i2), float(i3), float(i4)]
            else:
                self.logger.warn(
                    f"received CAN frame with unknown data_type_key {data_type_key}; id=0x{arbitration_id:X}"
                )
                return (None, None)
        except struct.error as e:
            self.logger.error(f"error unpacking CAN payload: {e}")
            return (None, None)

        viccan = VicCAN(
            header=Header(
                stamp=self.clock.now().to_msg(),
                frame_id="from_vic",
            ),
            mcu_name=mcu_name,
            command_id=int(command),
            data=data,
        )

        self.logger.debug(
            f"received CAN frame id=0x{message.arbitration_id:X}, "
            f"decoded as VicCAN(mcu_name={viccan.mcu_name}, command_id={viccan.command_id}, data={viccan.data})"
        )

        return (
            viccan,
            f"{viccan.mcu_name},{viccan.command_id},"
            + ",".join(map(str, list(viccan.data))),
        )

    def write(self, msg: VicCAN):
        if not self.can_bus:
            raise DeviceClosedException("CAN bus not initialized")

        # build 11-bit arbitration ID according to VicCAN spec:
        # bits 10..8: targeted MCU key
        # bits 7..6:  data type key
        # bits 5..0:  command

        # map MCU name to 3-bit key.
        try:
            # convert string MCU name to MCU ID
            mcu_id = next(key for key, name in MCU_IDS.items() if name == msg.mcu_name.lower())
        except ValueError:
            self.logger.error(
                f"unknown VicCAN mcu_name '{msg.mcu_name}' for CAN frame; dropping message"
            )
            return

        # determine data type from length:
        # 0: double x1, 1: float32 x2, 2: int16 x4, 3: empty
        match data_len := len(msg.data):
            case 0:
                data_type = 3
                data = bytes()
            case 1:
                data_type = 0
                data = struct.pack(">d", *msg.data)
            case 2:
                data_type = 1
                data = struct.pack(">ff", *msg.data)
            case 3 | 4:  # 3 gets treated as 4
                data_type = 2
                if data_len == 3:
                    msg.data.append(0)
                data = struct.pack(">hhhh", *[int(x) for x in msg.data])
            case _:
                self.logger.error(
                    f"unexpected VicCAN data length: {data_len}; dropping message"
                )
                return

        # command is limited to 6 bits.
        command = int(msg.command_id)
        if command < 0 or command > 0x3F:
            self.logger.error(
                f"invalid command_id for CAN frame: {command}; dropping message"
            )
            return

        try:
            can_message = can.Message(
                arbitration_id=(mcu_id << 8) | (data_type << 6) | command,
                data=data,
                is_extended_id=False,
            )
        except Exception as e:
            self.logger.error(f"failed to construct CAN message: {e}")
            return

        try:
            self.can_bus.send(can_message)
            self.logger.debug(
                f"sent CAN frame id=0x{can_message.arbitration_id:X}, "
                f"data={list(can_message.data)}"
            )
        except can.CanError as e:
            self.logger.error(f"CAN error while sending: {e}")
            raise DeviceClosedException("CAN bus closed unexpectedly")

    def write_raw(self, msg: String):
        self.logger.warn(
            f"write_raw is not supported for CANConnector. msg: {msg.data}"
        )

    def cleanup(self):
        try:
            if self.can_bus is not None:
                self.logger.info("shutting down CAN bus")
                self.can_bus.shutdown()
        except Exception as e:
            self.logger.error(e)


class MockConnector(Connector):
    def __init__(self, logger: RcutilsLogger, clock: Clock):
        super().__init__(logger, clock)
        # No hardware interface for MockConnector. Publish to `/anchor/from_vic/mock_mcu` instead.

    def read(self) -> tuple[VicCAN | None, str | None]:
        return (None, None)

    def write(self, msg: VicCAN):
        pass

    def write_raw(self, msg: String):
        pass
