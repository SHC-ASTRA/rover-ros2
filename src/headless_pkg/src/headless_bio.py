from rclpy.node import Node
from rclpy.publisher import Publisher
from pygame.joystick import JoystickType
from os import getenv
from math import sqrt
from unilib import CanCmdId

# messages
from astra_msgs.msg import VicCAN

viccan_publisher: Publisher

STICK_DEADZONE = float(getenv("STICK_DEADZONE", "0.05"))


def mcu(mcu_name: str, cmd_id: CanCmdId, data: list[float]):
    viccan_publisher.publish(
        VicCAN(mcu_name=mcu_name, command_id=cmd_id.value, data=data)
    )


def citadel(cmd_id: CanCmdId, data: list[float]):
    mcu("citadel", cmd_id, data)


def lance(cmd_id: CanCmdId, data: list[float]):
    mcu("lance", cmd_id, data)


def stick_deadzone(value: float, threshold=STICK_DEADZONE) -> float:
    """Apply a deadzone to a joystick input so the motors don't sound angry"""
    if abs(value) < threshold:
        return 0.0
    return value


def setup_bio(node: Node):
    global viccan_publisher
    viccan_publisher = node.create_publisher(VicCAN, "/anchor/to_vic/relay", 2)


def loop_bio(gamepad: JoystickType):
    # left_stick_x = stick_deadzone(gamepad.get_axis(0))
    left_stick_y = stick_deadzone(gamepad.get_axis(1))
    # right_stick_x = stick_deadzone(gamepad.get_axis(3))
    right_stick_y = stick_deadzone(gamepad.get_axis(4))
    left_trigger = max(0.0, gamepad.get_axis(2))
    right_trigger = max(0.0, gamepad.get_axis(5))
    # button_a = gamepad.get_button(0)
    button_b = gamepad.get_button(1)
    button_x = gamepad.get_button(2)
    button_y = gamepad.get_button(3)
    left_bumper = gamepad.get_button(4)
    # right_bumper = gamepad.get_button(5)
    dpad_x, _ = gamepad.get_hat(0)

    # /bio/citadel/control (sike not actually it's all VicCAN)
    target_id = 0 if button_x else 1 if button_y else 2 if button_b else -1
    target_list = [float(x == target_id) for x in range(4)]

    # here is valve (only if no dpad)
    citadel(CanCmdId.CMD_CITADEL_VALVES, target_list if dpad_x == 0 else [0.0] * 4)
    # here is fan (only if no dpad and yes face button)
    citadel(
        CanCmdId.CMD_REV_SET_DUTY,
        [sqrt(right_trigger) * 100.0 if dpad_x == 0 and target_id != -1 else 0.0],
    )
    # distibutor (only if dpad left
    citadel(
        CanCmdId.CMD_CITADEL_FAN_CTRL,
        target_list if dpad_x < 0 else [0.0] * 4,
    )
    # pump (only if dpad right)
    for i in range(3):
        citadel(
            CanCmdId.CMD_LSS_TURNBY_DEG,
            [
                float(i),
                (
                    float(i == target_id and dpad_x > 0)
                    if not (left_bumper and dpad_x > 0)
                    else -1.0
                ),
            ],
        )

    # /bio/lance/control (sike again it's still VicCAN)
    lance(
        CanCmdId.CMD_REV_SET_DUTY,
        [right_trigger - left_trigger if target_id == -1 else 0.0],
    )
    lance(
        CanCmdId.CMD_LANCE_LINEAR_AC,
        [1.0, left_stick_y],
    )
    lance(
        CanCmdId.CMD_LANCE_LINEAR_AC,
        [2.0, right_stick_y],
    )


def stop_bio():
    # stop lance
    lance(
        CanCmdId.CMD_REV_SET_DUTY,
        [0.0],
    )
    lance(
        CanCmdId.CMD_LANCE_LINEAR_AC,
        [1.0, 0.0],
    )
    lance(
        CanCmdId.CMD_LANCE_LINEAR_AC,
        [2.0, 0.0],
    )

    # stop citadel
    # valve
    citadel(
        CanCmdId.CMD_CITADEL_VALVES,
        [0.0] * 4,
    )
    # fan
    citadel(
        CanCmdId.CMD_REV_SET_DUTY,
        [0.0],
    )
    # distib
    citadel(
        CanCmdId.CMD_CITADEL_FAN_CTRL,
        [0.0] * 4,
    )
    # pumps
    for i in range(3):
        citadel(
            CanCmdId.CMD_LSS_TURNBY_DEG,
            [float(i), 0.0],
        )
