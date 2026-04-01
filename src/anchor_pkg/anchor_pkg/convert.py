from astra_msgs.msg import VicCAN
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.impl.rcutils_logger import RcutilsLogger


def string_to_viccan(
    msg: str, mcu_name: str, logger: RcutilsLogger, time: Time
) -> VicCAN | None:
    """
    Converts the serial string VicCAN format to a ROS2 VicCAN message.
    Does not fill out the Header of the message.
    On a failure, it will log at a debug level why it failed and return None.
    """

    parts: list[str] = msg.strip().split(",")

    # don't need an extra check because len of .split output is always >= 1
    if not parts[0].startswith("can_relay_"):
        logger.debug(f"got non-CAN data from {mcu_name}: {msg}")
        return None
    elif len(parts) < 3:
        logger.debug(f"got garbage (not enough parts) CAN data from {mcu_name}: {msg}")
        return None
    elif len(parts) > 7:
        logger.debug(f"got garbage (too many parts) CAN data from {mcu_name}: {msg}")
        return None

    try:
        command_id = int(parts[2])
    except ValueError:
        logger.debug(
            f"got garbage (non-integer command id) CAN data from {mcu_name}: {msg}"
        )
        return None

    if command_id not in range(64):
        logger.debug(
            f"got garbage (wrong command id {command_id}) CAN data from {mcu_name}: {msg}"
        )
        return None

    try:
        return VicCAN(
            header=Header(
                stamp=time,
                frame_id="from_vic",
            ),
            mcu_name=parts[1],
            command_id=command_id,
            data=[float(x) for x in parts[3:]],
        )
    except ValueError:
        logger.debug(f"got garbage (non-numerical) CAN data from {mcu_name}: {msg}")
        return None


def viccan_to_string(viccan: VicCAN) -> str:
    """Converts a ROS2 VicCAN message to the serial string VicCAN format."""
    # go from [ w, x, y, z ] -> ",w,x,y,z" & round to 7 digits max
    data = "".join([f",{round(val,7)}" for val in viccan.data])
    return f"can_relay_tovic,{viccan.mcu_name},{viccan.command_id}{data}\n"
