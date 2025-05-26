import asyncio
import logging

from siyi import (
    SiyiGimbalCamera,
    CommandID,
    DataStreamType,
    DataStreamFrequency,
    SingleAxis,
)

# Set up basic logging for this test script
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def main():
    # Setup: Define your camera instance and connection parameters
    camera = SiyiGimbalCamera(ip="192.168.1.17", port=37260)

    try:
        await camera.connect()
        # Initial Connection Established: Wait briefly before sending the commands
        await asyncio.sleep(2)

        # Command 1: Move all the way to the right (using set angles)
        logger.info("Command 1: Move all the way to the right (using absolute angle control)")
        await camera.send_attitude_angles_command(135.0, 0.0)
        await asyncio.sleep(5)

        # Command 2: Look down (using relative control)
        logger.info("Command 2: Start looking down (relative speed control)")
        #   The numbers used are example numbers and may be different from the numbers you use.
        await camera.send_rotation_command(0, 50)
        await asyncio.sleep(5)

        # Command 3: Stop looking down, then look up (with the single axis)
        logger.info("Command 3: Stop looking down and start looking up (single axis control)")
        await camera.send_rotation_command(0, 0)
        await camera.send_single_axis_attitude_command(135, SingleAxis.PITCH)
        await asyncio.sleep(5)

        # Command 4: Reset and move all the way to the left (Absolute value).
        logger.info("Command 4: Move back to the center, and start moving all the way left")
        await camera.send_attitude_angles_command(-135.0, 0.0)
        await asyncio.sleep(5)

        # Command 5: Final rotation to the center (using set angles)
        logger.info("Command 5: Moving back to the center.")
        await camera.send_attitude_angles_command(0, 0)
        await asyncio.sleep(5)
    except Exception as e:
        logger.exception(f"An exception occurred: {e}")
    finally:
        # Disconnect and cleanup after commands have run.
        await camera.disconnect()
        logger.info("Test script completed and shutdown")


if __name__ == "__main__":
    asyncio.run(main())
