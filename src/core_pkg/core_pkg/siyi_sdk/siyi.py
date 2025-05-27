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
    ABSOLUTE_ZOOM = 0x08
    ATTITUDE_ANGLES = 0x0E
    SINGLE_AXIS_CONTROL = 0x41
    DATA_STREAM_REQUEST = 0x25
    ATTITUDE_DATA_RESPONSE = 0x0D


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
    """

    MAX_A8_MINI_ZOOM = 6.0  # Maximum zoom for A8 mini

    def __init__(
        self, ip: str, port: int = 37260, *, heartbeat_interval: int = 2
    ):
        self.ip = ip
        self.port = port
        self.heartbeat_interval = heartbeat_interval
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.is_connected = False
        self.seq: int = 0
        self._data_callback: Optional[Callable[[CommandID | int, Any], None]] = None

    async def connect(self) -> None:
        try:
            self.reader, self.writer = await asyncio.open_connection(
                self.ip, self.port
            )
            self.is_connected = True
            asyncio.create_task(self.heartbeat_loop())
            asyncio.create_task(self._data_stream_listener())
        except (socket.gaierror, ConnectionRefusedError) as e:
            self.is_connected = False
            logger.error(f"Could not connect: {e}")
            raise

    async def disconnect(self) -> None:
        if self.is_connected and self.writer:
            self.is_connected = False
            self.writer.close()
            await self.writer.wait_closed()
        else:
            pass

    async def heartbeat_loop(self) -> None:
        if not self.is_connected or not self.writer:
            return
        try:
            while self.is_connected:
                try:
                    self.writer.write(HEARTBEAT_PACKET)
                    await self.writer.drain()
                    await asyncio.sleep(self.heartbeat_interval)
                except (socket.error, BrokenPipeError) as e:
                    break
        finally:
            if self.is_connected:
                await self.disconnect()

    def _build_packet_header(
        self, cmd_id: CommandID, data_len: int
    ) -> bytearray:
        """Helper to build the common packet header."""
        packet = bytearray()
        packet.extend(b"\x55\x66")  # STX
        packet.append(0x01)  # CTRL (request ACK)
        packet.extend(struct.pack("<H", data_len))
        packet.extend(struct.pack("<H", self.seq))
        packet.append(cmd_id.value)
        return packet

    def _finalize_packet(self, packet: bytearray) -> bytes:
        """Helper to add CRC and increment sequence number."""
        crc_value = Crc16.calc(packet)
        packet.extend(struct.pack("<H", crc_value))
        self.seq = (self.seq + 1) % 0x10000
        return bytes(packet)

    def _build_rotation_packet(self, turn_yaw: int, turn_pitch: int) -> bytes:
        data_len = 2
        packet = self._build_packet_header(
            CommandID.ROTATION_CONTROL, data_len
        )
        packet.extend(struct.pack("bb", turn_yaw, turn_pitch))
        return self._finalize_packet(packet)

    async def send_rotation_command(
        self, turn_yaw: int, turn_pitch: int
    ) -> None:
        if not self.is_connected or not self.writer:
            raise RuntimeError(
                "Socket is not connected or writer is None, cannot send rotation command."
            )
        packet = self._build_rotation_packet(turn_yaw, turn_pitch)
        self.writer.write(packet)
        await self.writer.drain()
        logger.debug(
            f"Sent rotation command with yaw_speed {turn_yaw} and pitch_speed {turn_pitch}"
        )

    def _build_attitude_angles_packet(
        self, yaw: float, pitch: float
    ) -> bytes:
        data_len = 4
        packet = self._build_packet_header(
            CommandID.ATTITUDE_ANGLES, data_len
        )
        yaw_int = int(round(yaw * 10))
        pitch_int = int(round(pitch * 10))
        packet.extend(struct.pack("<hh", yaw_int, pitch_int))
        return self._finalize_packet(packet)

    async def send_attitude_angles_command(
        self, yaw: float, pitch: float
    ) -> None:
        if not self.is_connected or not self.writer:
            raise RuntimeError(
                "Socket is not connected or writer is None, cannot send attitude angles command."
            )
        packet = self._build_attitude_angles_packet(yaw, pitch)
        self.writer.write(packet)
        await self.writer.drain()
        logger.debug(
            f"Sent attitude angles command with yaw {yaw}° and pitch {pitch}°"
        )

    def _build_single_axis_attitude_packet(
        self, angle: float, axis: SingleAxis
    ) -> bytes:
        data_len = 3
        packet = self._build_packet_header(
            CommandID.SINGLE_AXIS_CONTROL, data_len
        )
        angle_int = int(round(angle * 10))
        packet.extend(struct.pack("<hB", angle_int, axis.value))
        return self._finalize_packet(packet)

    async def send_single_axis_attitude_command(
        self, angle: float, axis: SingleAxis
    ) -> None:
        if not self.is_connected or not self.writer:
            raise RuntimeError(
                "Socket is not connected or writer is None, cannot send single-axis attitude command."
            )
        packet = self._build_single_axis_attitude_packet(angle, axis)
        self.writer.write(packet)
        await self.writer.drain()
        logger.debug(
            f"Sent single-axis attitude command for {axis.name.lower()} with angle {angle}°"
        )

    def _build_data_stream_packet(
        self, data_type: DataStreamType, data_freq: DataStreamFrequency
    ) -> bytes:
        data_len = 2
        packet = self._build_packet_header(
            CommandID.DATA_STREAM_REQUEST, data_len
        )
        packet.append(data_type.value)
        packet.append(data_freq.value)
        return self._finalize_packet(packet)

    async def send_data_stream_request(
        self, data_type: DataStreamType, data_freq: DataStreamFrequency
    ) -> None:
        if not self.is_connected or not self.writer:
            raise RuntimeError(
                "Socket is not connected or writer is None, cannot request data stream."
            )
        packet = self._build_data_stream_packet(data_type, data_freq)
        self.writer.write(packet)
        await self.writer.drain()
        logger.info(
            f"Sent data stream request: data_type {data_type.name}, data_freq {data_freq.name}"
        )

    def _build_absolute_zoom_packet(self, zoom_level: float) -> bytes:
        data_len = 2
        packet = self._build_packet_header(CommandID.ABSOLUTE_ZOOM, data_len)
        zoom_packet_value = int(round(zoom_level * 10))
        if not (0 <= zoom_packet_value <= 65535): # Should be caught by clamping earlier
            raise ValueError(
                "Zoom packet value out of uint16_t range after conversion."
            )
        packet.extend(struct.pack("<H", zoom_packet_value))
        return self._finalize_packet(packet)

    async def send_absolute_zoom_command(self, zoom_level: float) -> None:
        """
        Send an absolute zoom command (0x08) to the gimbal.

        :param zoom_level: The desired zoom level as a float (e.g., 1.0 for 1x,
                           2.5 for 2.5x). For A8 mini, this will be clamped
                           between 1.0 and MAX_A8_MINI_ZOOM (6.0).
        """
        if not self.is_connected or not self.writer:
            raise RuntimeError(
                "Socket is not connected or writer is None, cannot send absolute zoom command."
            )

        original_requested_zoom = zoom_level  # For logging clarity

        if zoom_level < 1.0:
            logger.warning(
                f"Requested zoom level {original_requested_zoom:.1f} is less than 1.0. Clamping to 1.0x."
            )
            zoom_level = 1.0
        elif zoom_level > self.MAX_A8_MINI_ZOOM:  # Check against max zoom
            logger.warning(
                f"Requested zoom level {original_requested_zoom:.1f} exceeds maximum {self.MAX_A8_MINI_ZOOM:.1f}x for A8 mini. "
                f"Clamping to {self.MAX_A8_MINI_ZOOM:.1f}x."
            )
            zoom_level = self.MAX_A8_MINI_ZOOM

        packet = self._build_absolute_zoom_packet(zoom_level)
        self.writer.write(packet)
        await self.writer.drain()
        logger.debug(
            f"Sent absolute zoom command with level {zoom_level:.1f}x (original request: {original_requested_zoom:.1f}x)"
        )

    async def _read_packet(self):
        if not self.reader:
            raise RuntimeError("Reader is not initialized.")
        stx = await self.reader.readexactly(2)
        if stx != b"\x55\x66":
            raise ValueError(f"Invalid packet start bytes: {stx.hex()}")
        ctrl = await self.reader.readexactly(1)
        data_len_bytes = await self.reader.readexactly(2)
        data_len = struct.unpack("<H", data_len_bytes)[0]
        seq_bytes = await self.reader.readexactly(2) # Renamed for clarity
        # seq_val = struct.unpack("<H", seq_bytes)[0] # If you need the sequence value
        cmd_id_bytes = await self.reader.readexactly(1)
        cmd_id_val = cmd_id_bytes[0] # Renamed for clarity
        
        # Protect against excessively large data_len
        if data_len > 2048: # Arbitrary reasonable limit
            raise ValueError(f"Excessive data length received: {data_len}")

        data = await self.reader.readexactly(data_len)
        crc_bytes = await self.reader.readexactly(2)
        received_crc = struct.unpack("<H", crc_bytes)[0]
        
        packet_without_crc = (
            stx + ctrl + data_len_bytes + seq_bytes + cmd_id_bytes + data
        )
        computed_crc = Crc16.calc(packet_without_crc)
        
        if computed_crc != received_crc:
            raise ValueError(
                f"CRC check failed. Expected {computed_crc:04X}, got {received_crc:04X}. "
                f"Packet: {packet_without_crc.hex()}"
            )
        return cmd_id_val, data

    @staticmethod
    def parse_attitude_data(data: bytes) -> AttitudeData:
        if len(data) != 12:
            raise ValueError("Attitude data should be exactly 12 bytes.")
        return AttitudeData.from_bytes(data)

    async def _data_stream_listener(self) -> None:
        while self.is_connected:
            try:
                cmd_id_int, data = await self._read_packet()
                try:
                    cmd_id_enum = CommandID(cmd_id_int)
                except ValueError:
                    logger.warning(
                        f"Received unknown CMD_ID {cmd_id_int:02X}, data: {data.hex()}"
                    )
                    if self._data_callback:
                        self._data_callback(cmd_id_int, data)
                    continue

                if (
                    cmd_id_enum == CommandID.ATTITUDE_DATA_RESPONSE
                    and len(data) == 12
                ):
                    try:
                        parsed = AttitudeData.from_bytes(data)
                        if self._data_callback:
                            self._data_callback(cmd_id_enum, parsed)
                        else:
                            logger.info(f"Received attitude data: {parsed}")
                    except Exception as e:
                        logger.exception(
                            f"Failed to parse attitude data: {e}"
                        )
                        if self._data_callback:
                            self._data_callback(cmd_id_enum, data)
                else:
                    if self._data_callback:
                        self._data_callback(cmd_id_enum, data)
                    else:
                        logger.info(
                            f"Received packet with CMD_ID {cmd_id_enum.name} ({cmd_id_enum.value:02X}): {data.hex()}"
                        )

            except (
                ConnectionResetError,
                BrokenPipeError,
                ConnectionError,
                asyncio.IncompleteReadError,
            ) as e:
                logger.error(f"Connection error in listener: {e}")
                self.is_connected = False
                break
            except ValueError as e:
                logger.error(f"Packet error in listener: {e}")
                # Consider adding a small delay or a mechanism to resync if this happens frequently
                await asyncio.sleep(0.1) # Small delay before trying to read again
                continue
            except Exception as e:
                logger.exception(f"Unexpected error in data stream listener: {e}")
                # Depending on the error, you might want to break or continue
                await asyncio.sleep(0.1) # Small delay
                continue

    def set_data_callback(
        self, callback: Callable[[CommandID | int, Any], None]
    ) -> None:
        self._data_callback = callback


async def main_sdk_test(): # Renamed to avoid conflict if this file is imported
    gimbal_ip = "192.168.144.25"
    gimbal = SiyiGimbalCamera(gimbal_ip)

    def my_data_handler(cmd_id, data):
        if cmd_id == CommandID.ATTITUDE_DATA_RESPONSE and isinstance(data, AttitudeData):
            print(
                f"Attitude: Yaw={data.yaw:.1f}, Pitch={data.pitch:.1f}, Roll={data.roll:.1f}"
            )
        elif isinstance(cmd_id, CommandID):
            print(
                f"Received {cmd_id.name}: {data.hex() if isinstance(data, bytes) else data}"
            )
        else:
            print(
                f"Received Unknown CMD_ID {cmd_id:02X}: {data.hex() if isinstance(data, bytes) else data}"
            )

    gimbal.set_data_callback(my_data_handler)

    try:
        await gimbal.connect()
        if gimbal.is_connected:
            print("SDK Test: Successfully connected to the gimbal.")

            print("SDK Test: Setting zoom to 1.0x")
            await gimbal.send_absolute_zoom_command(1.0)
            await asyncio.sleep(2)

            print("SDK Test: Setting zoom to 3.5x")
            await gimbal.send_absolute_zoom_command(3.5)
            await asyncio.sleep(2)

            print("SDK Test: Setting zoom to 6.0x (A8 mini max)")
            await gimbal.send_absolute_zoom_command(6.0)
            await asyncio.sleep(2)
            
            print("SDK Test: Attempting zoom to 7.0x (should be clamped to 6.0x)")
            await gimbal.send_absolute_zoom_command(7.0)
            await asyncio.sleep(2)

            print("SDK Test: Attempting zoom to 0.5x (should be clamped to 1.0x)")
            await gimbal.send_absolute_zoom_command(0.5)
            await asyncio.sleep(2)

            print("SDK Test: Requesting attitude data stream at 5Hz...")
            await gimbal.send_data_stream_request(DataStreamType.ATTITUDE_DATA, DataStreamFrequency.HZ_5)
            
            print("SDK Test: Listening for data for 10 seconds...")
            await asyncio.sleep(10)

            print("SDK Test: Disabling attitude data stream...")
            await gimbal.send_data_stream_request(DataStreamType.ATTITUDE_DATA, DataStreamFrequency.DISABLE)
            await asyncio.sleep(1)


    except ConnectionRefusedError:
        print(
            f"SDK Test: Connection to {gimbal_ip} was refused. Is the gimbal on and accessible?"
        )
    except Exception as e:
        print(f"SDK Test: An error occurred: {e}")
    finally:
        if gimbal.is_connected:
            print("SDK Test: Disconnecting...")
            await gimbal.disconnect()
        print("SDK Test: Program finished.")


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )
    # To see debug logs from this SDK specifically:
    # logger.setLevel(logging.DEBUG)
    asyncio.run(main_sdk_test())
