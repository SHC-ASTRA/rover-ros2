#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    Shutdown,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Prevent making __pycache__ directories
from sys import dont_write_bytecode

dont_write_bytecode = True


def launch_setup(context, *args, **kwargs):
    # Retrieve the resolved value of the launch argument 'mode'
    mode = LaunchConfiguration("mode").perform(context)
    nodes = []

    if mode == "anchor":
        # Launch every node and pass "anchor" as the parameter

        nodes.append(
            Node(
                package="arm_pkg",
                executable="arm",  # change as needed
                name="arm",
                output="both",
                parameters=[{"launch_mode": mode}],
                on_exit=Shutdown(),
            )
        )
        nodes.append(
            Node(
                package="core_pkg",
                executable="core",  # change as needed
                name="core",
                output="both",
                parameters=[
                    {"launch_mode": mode},
                    {"use_ros2_control": LaunchConfiguration("use_ros2_control", default=False)},
                ],
                on_exit=Shutdown(),
            )
        )
        nodes.append(
            Node(
                package="core_pkg",
                executable="ptz",  # change as needed
                name="ptz",
                output="both",
                # Currently don't shutdown all nodes if the PTZ node fails, as it is not critical
                # on_exit=Shutdown()  # Uncomment if you want to shutdown on PTZ failure
            )
        )
        nodes.append(
            Node(
                package="bio_pkg",
                executable="bio",  # change as needed
                name="bio",
                output="both",
                parameters=[{"launch_mode": mode}],
                on_exit=Shutdown(),
            )
        )
        nodes.append(
            Node(
                package="anchor_pkg",
                executable="anchor",  # change as needed
                name="anchor",
                output="both",
                parameters=[{"launch_mode": mode}],
                on_exit=Shutdown(),
            )
        )
    elif mode in ["arm", "core", "bio", "ptz"]:
        # Only launch the node corresponding to the provided mode.
        if mode == "arm":
            nodes.append(
                Node(
                    package="arm_pkg",
                    executable="arm",
                    name="arm",
                    output="both",
                    parameters=[{"launch_mode": mode}],
                    on_exit=Shutdown(),
                )
            )
        elif mode == "core":
            nodes.append(
                Node(
                    package="core_pkg",
                    executable="core",
                    name="core",
                    output="both",
                    parameters=[{"launch_mode": mode}],
                    on_exit=Shutdown(),
                )
            )
        elif mode == "bio":
            nodes.append(
                Node(
                    package="bio_pkg",
                    executable="bio",
                    name="bio",
                    output="both",
                    parameters=[{"launch_mode": mode}],
                    on_exit=Shutdown(),
                )
            )
        elif mode == "ptz":
            nodes.append(
                Node(
                    package="core_pkg",
                    executable="ptz",
                    name="ptz",
                    output="both",
                    on_exit=Shutdown(),  # on fail, shutdown if this was the only node to be launched
                )
            )
    else:
        # If an invalid mode is provided, print an error.
        print("Invalid mode provided. Choose one of: arm, core, bio, anchor, ptz.")

    return nodes


def generate_launch_description():
    declare_arg = DeclareLaunchArgument(
        "mode",
        default_value="anchor",
        description="Launch mode: arm, core, bio, anchor, or ptz",
    )

    ros2_control_arg = DeclareLaunchArgument(
        "use_ros2_control",
        default_value="false",
        description="Whether to use DiffDriveController for driving instead of direct Twist",
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("core_description"),
                    "launch",
                    "robot_state_publisher.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
        launch_arguments={("hardware_mode", "physical")},
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("core_description"),
                    "launch",
                    "spawn_controllers.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_ros2_control")),
        launch_arguments={("hardware_mode", "physical")},
    )

    return LaunchDescription(
        [
            declare_arg,
            ros2_control_arg,
            rsp,
            controllers,
            OpaqueFunction(function=launch_setup),
        ]
    )
