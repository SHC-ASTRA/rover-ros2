from astra_msgs.msg import VicCAN
from rclpy.impl.rcutils_logger import RcutilsLogger

def string_to_viccan(msg: str, mcu_name: str, logger: RcutilsLogger):
    """
    Converts the serial string VicCAN format to a ROS2 VicCAN message.
    Does not fill out the Header of the message.
    On a failure, it will log at a debug level why it failed.

    Parameters:
      * msg: str
        - The message in serial VicCAN format
      * mcu_name: str
        - The name of the MCU (e.g. core, citadel, arm)
      * logger: RcutilsLogger
        - A logger retrieved from node.get_logger()

    Returns:
      * VicCAN | None
        - The VicCAN message on a success or None on a failure
    """

    parts: list[str] = msg.split(",")

    # don't need an extra check because len of .split output is always >= 1
    if parts[0] != "can_relay_fromvic":
        logger.debug(f"got non-CAN data from {mcu_name}: {msg}")
        return None
    elif len(parts) < 3:
        logger.debug(
            f"got garbage (not enough parts) CAN data from {mcu_name}: {msg}"
        )
        return None
    elif len(parts) > 7:
        logger.debug(f"got garbage (too many parts) CAN data from {mcu_name}: {msg}")
        return None

    return VicCAN(
        mcu_name=parts[1],
        command_id=int(parts[2]),
        data=[float(x) for x in parts[3:]],
    )
