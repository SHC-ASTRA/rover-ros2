from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connector = LaunchConfiguration("connector")
    serial_override = LaunchConfiguration("serial_override")
    can_override = LaunchConfiguration("can_override")
    use_ptz = LaunchConfiguration("use_ptz")

    ld = LaunchDescription()

    # arguments
    ld.add_action(
        DeclareLaunchArgument(
            "connector",
            default_value="auto",
            description="Connector parameter for anchor node (default: 'auto')",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "serial_override",
            default_value="",
            description="Serial port override parameter for anchor node (default: '')",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "can_override",
            default_value="auto",
            description="CAN network override parameter for anchor node (default: '')",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_ptz",
            default_value="true",  # must be string for launch system
            description="Whether to launch PTZ node (default: true)",
        )
    )

    # nodes
    ld.add_action(
        Node(
            package="arm_pkg",
            executable="arm",
            name="arm",
            output="both",
            parameters=[{"launch_mode": "anchor"}],
            on_exit=Shutdown(),
        )
    )

    ld.add_action(
        Node(
            package="core_pkg",
            executable="core",
            name="core",
            output="both",
            parameters=[{"launch_mode": "anchor"}],
            on_exit=Shutdown(),
        )
    )

    ld.add_action(
        Node(
            package="core_pkg",
            executable="ptz",
            name="ptz",
            output="both",
            condition=IfCondition(use_ptz),
        )
    )

    ld.add_action(
        Node(
            package="bio_pkg",
            executable="bio",
            name="bio",
            output="both",
            parameters=[{"launch_mode": "anchor"}],
            on_exit=Shutdown(),
        )
    )

    ld.add_action(
        Node(
            package="anchor_pkg",
            executable="anchor",
            name="anchor",
            output="both",
            parameters=[
                {
                    "launch_mode": "anchor",
                    "connector": connector,
                    "serial_override": serial_override,
                    "can_override": can_override,
                }
            ],
            on_exit=Shutdown(),
        )
    )

    return ld
