import asyncio
import socket
import struct
import logging
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Callable, Any
from crccheck.crc import Crc16

# Set up logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)  # You can adjust the logging level as needed
handler = logging.StreamHandler()  # Output to console
formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)

# Global constant for the heartbeat packet (used in TCP keep-alive)
HEARTBEAT_PACKET = bytes.fromhex("55 66 01 01 00 00 00 00 00 59 8B")


class DataStreamType(Enum):
    """
    Enumeration for data stream types in the 0x25 command.

    Allows users to select which type of data to stream from the gimbal camera.
    """

    ATTITUDE_DATA = 1
    LASER_RANGE_DATA = 2
    MAGNETIC_ENCODER_ANGLE_DATA = 3
    MOTOR_VOLTAGE_DATA = 4


class DataStreamFrequency(Enum):
    """
    Enumeration for data stream frequencies in the 0x25 command.

    Allows the user to select the output (streaming) frequency.
    """

    DISABLE = 0
    HZ_2 = 1
    HZ_4 = 2
    HZ_5 = 3
    HZ_10 = 4
    HZ_20 = 5
    HZ_50 = 6
    HZ_100 = 7


class SingleAxis(Enum):
    """
    Enumeration for specifying the controlled axis in the 0x41 single-axis control command.

    YAW corresponds to 0 and PITCH corresponds to 1.
    """

    YAW = 0
    PITCH = 1


class CommandID(Enum):
    """
    Enumeration for command identifiers used in the protocol.

    These identifiers are sent as the CMD_ID in a packet and help in matching
    responses or processing incoming packets.
    """

    ROTATION_CONTROL = 0x07
    ATTITUDE_ANGLES = 0x0E
    SINGLE_AXIS_CONTROL = 0x41
    DATA_STREAM_REQUEST = 0x25
    ATTITUDE_DATA_RESPONSE = 0x0D  # Expected response packet for attitude data


@dataclass
class AttitudeData:
    """
    Class representing the parsed attitude data from the gimbal.

    Angles (yaw, pitch, roll) are in degrees and are scaled using one decimal point precision.
    Angular velocities are raw int16 values.
    """

    yaw: float
    pitch: float
    roll: float
    yaw_velocity: int
    pitch_velocity: int
    roll_velocity: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "AttitudeData":
        """Create an AttitudeData instance from a byte string."""
        if len(data) != 12:
            raise ValueError("Attitude data should be exactly 12 bytes.")
        # Unpack six little-endian int16 values
        values = struct.unpack("<hhhhhh", data)
        return cls(
            yaw=values[0] / 10.0,
            pitch=values[1] / 10.0,
            roll=values[2] / 10.0,
            yaw_velocity=values[3],
            pitch_velocity=values[4],
            roll_velocity=values[5],
        )


class SiyiGimbalCamera:
    """
    Class to interface with the SIYI Gimbal Camera over TCP.

    This class handles:
      - Establishing a TCP connection.
      - Sending periodic heartbeat packets.
      - Building and sending commands (e.g. for rotation, setting attitudes,
        single-axis control, and data stream requests).
      - Continuously reading incoming packets in a background task.

    Command identifiers and axis flags are represented as enums, and the parsed
    attitude data is returned as an AttitudeData object.

    The data stream listener is started as a background task when a connection is
    established. Users can provide or update the callback used for processing data
    stream packets by calling set_data_callback().
    """

    def __init__(self, ip: str, port: int = 37260, *, heartbeat_interval: int = 2):
        """
        Initialize a new SiyiGimbalCamera instance.

        :param ip: The IP address (as a string) of the SIYI gimbal camera.
        :param port: The TCP port (as an integer) used for the connection (default: 37260).
        :param heartbeat_interval: Interval in seconds (int) between heartbeat packets.
        """
        self.ip = ip
        self.port = port
        self.heartbeat_interval = heartbeat_interval
        # These will be assigned after successfully connecting.
        self.reader: asyncio.StreamReader
        self.writer: asyncio.StreamWriter
        self.is_connected = False
        # Sequence counter (integer in the range 0 to 65535) for outgoing packets.
        self.seq: int = 0
        self._data_callback: Optional[Callable[[CommandID, Any], None]] = None

    async def connect(self) -> None:
        """Establish a TCP connection with the gimbal camera."""
        try:
            self.reader, self.writer = await asyncio.open_connection(
                self.ip, self.port
            )
            self.is_connected = True
            logger.info(f"Connected to {self.ip}:{self.port}")
            asyncio.create_task(self.heartbeat_loop())
            # Start the data stream listener in the background.
            asyncio.create_task(self._data_stream_listener())
        except (socket.gaierror, ConnectionRefusedError) as e:
            self.is_connected = False
            logger.error(f"Could not connect: {e}")
            raise

    async def disconnect(self) -> None:
        """Gracefully disconnect from the gimbal camera."""
        if self.is_connected:
            self.is_connected = False
            self.writer.close()
            await self.writer.wait_closed()
            logger.info("Disconnected")
        else:
            logger.warning("Not connected, cannot disconnect.")

    async def heartbeat_loop(self) -> None:
        """Periodically send heartbeat packets to maintain the TCP connection."""
        if not self.is_connected:
            logger.warning("Heartbeat loop started before connection was established.")
            return

        try:
            while self.is_connected:
                try:
                    self.writer.write(HEARTBEAT_PACKET)
                    await self.writer.drain()
                    logger.debug("Sent heartbeat packet")
                    await asyncio.sleep(self.heartbeat_interval)
                except (socket.error, BrokenPipeError) as e:
                    logger.error(f"Connection error in heartbeat loop: {e}")
                    break
        finally:
            logger.info("Heartbeat loop stopped.")
            if self.is_connected:
                await self.disconnect()

    def _build_rotation_packet(self, turn_yaw: int, turn_pitch: int) -> bytes:
        """Build a full packet for the 0x07 Gimbal Rotation Control command."""
        packet = bytearray()
        packet.extend(b"\x55\x66")               # STX bytes
        packet.append(0x01)                      # CTRL (request ACK)
        packet.extend(struct.pack("<H", 2))      # Data length: 2 bytes
        packet.extend(struct.pack("<H", self.seq))  # Sequence number
        packet.append(CommandID.ROTATION_CONTROL.value)  # CMD_ID for rotation control
        packet.extend(struct.pack("bb", turn_yaw, turn_pitch))  # Data: two int8 values
        crc_value = Crc16.calc(packet)
        packet.extend(struct.pack("<H", crc_value))
        self.seq = (self.seq + 1) % 0x10000
        return bytes(packet)

    async def send_rotation_command(self, turn_yaw: int, turn_pitch: int) -> None:
        """Send a rotation control command (0x07) with the given yaw and pitch values."""
        if not self.is_connected:
            raise RuntimeError("Socket is not connected, cannot send rotation command.")
        packet = self._build_rotation_packet(turn_yaw, turn_pitch)
        self.writer.write(packet)
        await self.writer.drain()
        logger.debug(f"Sent rotation command with yaw {turn_yaw} and pitch {turn_pitch}")

    def _build_attitude_angles_packet(self, yaw: float, pitch: float) -> bytes:
        """Build a full packet for the 0x0E 'Set Gimbal Attitude Angles' command."""
        packet = bytearray()
        packet.extend(b"\x55\x66")
        packet.append(0x01)  # CTRL (request ACK)
        packet.extend(struct.pack("<H", 4))  # Data length: 4 bytes (2 for each angle)
        packet.extend(struct.pack("<H", self.seq))
        packet.append(CommandID.ATTITUDE_ANGLES.value)  # CMD_ID for set gimbal attitude angles
        yaw_int = int(round(yaw * 10))
        pitch_int = int(round(pitch * 10))
        packet.extend(struct.pack("<hh", yaw_int, pitch_int))
        crc_value = Crc16.calc(packet)
        packet.extend(struct.pack("<H", crc_value))
        self.seq = (self.seq + 1) % 0x10000
        return bytes(packet)

    async def send_attitude_angles_command(self, yaw: float, pitch: float) -> None:
        """Send the 0x0E 'Set Gimbal Attitude Angles' command to set absolute angles."""
        if not self.is_connected:
            raise RuntimeError("Socket is not connected, cannot send attitude angles command.")
        packet = self._build_attitude_angles_packet(yaw, pitch)
        self.writer.write(packet)
        await self.writer.drain()
        logger.debug(f"Sent attitude angles command with yaw {yaw}° and pitch {pitch}°")

    def _build_single_axis_attitude_packet(self, angle: float, axis: SingleAxis) -> bytes:
        """Build a full packet for the 0x41 'Single-Axis Attitude Control' command."""
        packet = bytearray()
        packet.extend(b"\x55\x66")
        packet.append(0x01)  # CTRL
        packet.extend(struct.pack("<H", 3))  # Data length: 3 bytes
        packet.extend(struct.pack("<H", self.seq))
        packet.append(CommandID.SINGLE_AXIS_CONTROL.value)  # CMD_ID for single-axis control
        angle_int = int(round(angle * 10))
        packet.extend(struct.pack("<hB", angle_int, axis.value))
        crc_value = Crc16.calc(packet)
        packet.extend(struct.pack("<H", crc_value))
        self.seq = (self.seq + 1) % 0x10000
        return bytes(packet)

    async def send_single_axis_attitude_command(self, angle: float, axis: SingleAxis) -> None:
        """Send the 0x41 single-axis attitude control command."""
        if not self.is_connected:
            raise RuntimeError("Socket is not connected, cannot send single-axis attitude command.")
        packet = self._build_single_axis_attitude_packet(angle, axis)
        self.writer.write(packet)
        await self.writer.drain()
        logger.debug(
            f"Sent single-axis attitude command for {axis.name.lower()} with angle {angle}°"
        )

    def _build_data_stream_packet(self, data_type: DataStreamType, data_freq: DataStreamFrequency) -> bytes:
        """Build a packet for the 0x25 'Request Gimbal to Send Data Stream' command."""
        packet = bytearray()
        packet.extend(b"\x55\x66")
        packet.append(0x01)  # CTRL, request ACK
        packet.extend(struct.pack("<H", 2))  # Data length: 2 bytes
        packet.extend(struct.pack("<H", self.seq))
        packet.append(CommandID.DATA_STREAM_REQUEST.value)  # CMD_ID for data stream request
        packet.append(data_type.value)           # Data type (uint8)
        packet.append(data_freq.value)           # Data frequency (uint8)
        crc_value = Crc16.calc(packet)
        packet.extend(struct.pack("<H", crc_value))
        self.seq = (self.seq + 1) % 0x10000
        return bytes(packet)

    async def send_data_stream_request(self, data_type: DataStreamType, data_freq: DataStreamFrequency) -> None:
        """Send the 0x25 command to request a continuous data stream from the gimbal."""
        if not self.is_connected:
            raise RuntimeError("Socket is not connected, cannot request data stream.")
        packet = self._build_data_stream_packet(data_type, data_freq)
        self.writer.write(packet)
        await self.writer.drain()
        logger.info(f"Sent data stream request: data_type {data_type}, data_freq {data_freq}")

    async def _read_packet(self):
        """Read a full packet from the TCP stream following the protocol format."""
        stx = await self.reader.readexactly(2)
        if stx != b"\x55\x66":
            raise ValueError("Invalid packet start bytes.")
        ctrl = await self.reader.readexactly(1)
        data_len_bytes = await self.reader.readexactly(2)
        data_len = struct.unpack("<H", data_len_bytes)[0]
        seq = await self.reader.readexactly(2)
        cmd_id_bytes = await self.reader.readexactly(1)
        cmd_id = cmd_id_bytes[0]
        data = await self.reader.readexactly(data_len)
        crc_bytes = await self.reader.readexactly(2)
        received_crc = struct.unpack("<H", crc_bytes)[0]
        packet_without_crc = stx + ctrl + data_len_bytes + seq + cmd_id_bytes + data
        computed_crc = Crc16.calc(packet_without_crc)
        if computed_crc != received_crc:
            raise ValueError("CRC check failed for received packet.")
        return cmd_id, data

    @staticmethod
    def parse_attitude_data(data: bytes) -> AttitudeData:
        """Parse the attitude data (12 bytes) into human-readable values."""
        if len(data) != 12:
            raise ValueError("Attitude data should be exactly 12 bytes.")
        return AttitudeData.from_bytes(data)

    async def _data_stream_listener(self) -> None:
        """
        Continuously listen for incoming packets from the gimbal and process them.

        If a data callback is set via set_data_callback(), the callback is called
        with the CommandID and the corresponding data (parsed or raw).
        """
        while self.is_connected:
            try:
                cmd_id_int, data = await self._read_packet()
                cmd_id = CommandID(cmd_id_int)
            except Exception as e:
                logger.error(f"Error reading packet: {e}")
                continue

            # Process attitude data packets if applicable.
            if cmd_id == CommandID.ATTITUDE_DATA_RESPONSE and len(data) == 12:
                try:
                    parsed = AttitudeData.from_bytes(data)
                except Exception as e:
                    logger.exception(f"Failed to parse attitude data: {e}")
                    parsed = None
                if self._data_callback:
                    self._data_callback(cmd_id, parsed)
                else:
                    logger.info(f"Received attitude data: {parsed}")
            else:
                if self._data_callback:
                    self._data_callback(cmd_id, data)
                else:
                    logger.info(f"Received packet with CMD_ID {cmd_id}: {data}")

    def set_data_callback(self, callback: Callable[[CommandID, Any], None]) -> None:
        """
        Change the callback function used by the data stream listener.

        The callback should be a function that accepts two arguments:
          - A CommandID indicating the type of packet received.
          - The parsed data (for attitude data, an AttitudeData instance) or raw data.

        :param callback: A callable to be used as the data callback.
        """
        self._data_callback = callback
