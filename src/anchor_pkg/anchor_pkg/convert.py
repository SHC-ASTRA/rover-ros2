from astra_msgs.msg import VicCAN
from rclpy.impl.rcutils_logger import RcutilsLogger

def string_to_viccan(msg: str, mcu_name, logger: RcutilsLogger):
    parts: list[str] = msg.split(",")

    # don't need an extra check because len of .split output is always >= 1
    if parts[0] != "can_relay_fromvic":
        logger.info(f"got non-CAN data from {mcu_name}: {msg}")
        return None
    elif len(parts) < 3:
        logger.info(
            f"got garbage (not enough parts) CAN data from {mcu_name}: {msg}"
        )
        return None
    elif len(parts) > 7:
        logger.info(f"got garbage (too many parts) CAN data from {mcu_name}: {msg}")
        return None

    return VicCAN(
        mcu_name=parts[1],
        command_id=int(parts[2]),
        data=[float(x) for x in parts[3:]],
    )
