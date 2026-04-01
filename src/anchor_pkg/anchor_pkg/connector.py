from abc import ABC, abstractmethod
from astra_msgs.msg import VicCAN
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger
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
    def write_raw(self, msg: str):
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
        self.serial_interface = serial.Serial(self.port, BAUD_RATE, timeout=1)

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

    def read(self) -> tuple[VicCAN | None, str | None]:
        try:
            raw = str(self.serial_interface.readline(), "utf8")

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
        self.write_raw(viccan_to_string(msg))

    def write_raw(self, msg: str):
        self.serial_interface.write(bytes(msg, "utf8"))

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
            self.logger.warn("likely using virtual CAN interface")

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

        key_to_mcu: dict[int, str] = {
            1: "broadcast",
            2: "core",
            3: "arm",
            4: "digit",
            5: "faerie",
            6: "citadel",
        }

        mcu_name = key_to_mcu.get(mcu_key)
        if mcu_name is None:
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
                (value,) = struct.unpack("<d", data_bytes[:8])
                data = [float(value)]
            elif data_type_key == 1:
                if len(data_bytes) < 8:
                    self.logger.warn(
                        f"received float32x2 payload with insufficient length {len(data_bytes)}; dropping frame"
                    )
                    return (None, None)
                v1, v2 = struct.unpack("<ff", data_bytes[:8])
                data = [float(v1), float(v2)]
            elif data_type_key == 2:
                if len(data_bytes) < 8:
                    self.logger.warn(
                        f"received int16x4 payload with insufficient length {len(data_bytes)}; dropping frame"
                    )
                    return (None, None)
                i1, i2, i3, i4 = struct.unpack("<hhhh", data_bytes[:8])
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

        # Build 11-bit arbitration ID according to the VicCAN scheme:
        # bits 10..8: targeted MCU key
        # bits 7..6:  data type key
        # bits 5..0:  command

        # Map MCU name to 3-bit key.
        mcu_name = (msg.mcu_name or "").lower()
        mcu_key_map: dict[str, int] = {
            "broadcast": 1,
            "core": 2,
            "arm": 3,
            "digit": 4,
            "faerie": 5,
            "citadel": 6,
        }

        if mcu_name not in mcu_key_map:
            self.logger.error(
                f"unknown VicCAN mcu_name '{msg.mcu_name}' for CAN frame; dropping message"
            )
            return

        mcu_key = mcu_key_map[mcu_name] & 0b111

        # Infer data type key from payload length according to the table:
        # 0: double x1, 1: float32 x2, 2: int16 x4, 3: empty
        data_len = len(msg.data)
        if data_len == 0:
            data_type_key = 3
        elif data_len == 1:
            data_type_key = 0
        elif data_len == 2:
            data_type_key = 1
        elif data_len == 4:
            data_type_key = 2
        else:
            # Fallback: treat any other non-zero length as float32 x2
            self.logger.warn(
                f"unexpected VicCAN data length {data_len}; encoding as float32 x2 (key=1) and truncating/padding as needed"
            )
            data_type_key = 1

        # Command is limited to 6 bits.
        command = int(msg.command_id)
        if command < 0:
            self.logger.error(f"invalid negative command_id for CAN frame: {command}")
            return
        if command > 0x3F:
            self.logger.warn(
                f"command_id 0x{command:X} exceeds 6-bit range; truncating to lower 6 bits"
            )
            command &= 0x3F

        arbitration_id = (
            ((mcu_key & 0b111) << 8) | ((data_type_key & 0b11) << 6) | (command & 0x3F)
        )

        # Map VicCAN.data (floats) to up to 8 CAN data bytes.
        raw_bytes: list[int] = []
        for value in msg.data:
            try:
                b = int(round(value))
            except (TypeError, ValueError):
                self.logger.error(
                    f"non-numeric VicCAN data value: {value}; dropping message"
                )
                return

            if b < 0 or b > 255:
                self.logger.warn(
                    f"VicCAN data value {value} out of byte range; clamping into [0, 255]"
                )
                b = max(0, min(255, b))

            raw_bytes.append(b)

        if len(raw_bytes) > 8:
            self.logger.warn(
                f"VicCAN data too long for single CAN frame ({len(raw_bytes)} > 8); truncating"
            )
            raw_bytes = raw_bytes[:8]

        try:
            can_message = can.Message(
                arbitration_id=arbitration_id,
                data=raw_bytes,
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

    def write_raw(self, msg: str):
        self.logger.warn(f"write_raw is not supported for CANConnector. msg: {msg}")

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

    def write_raw(self, msg: str):
        pass
