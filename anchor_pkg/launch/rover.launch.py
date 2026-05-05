from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
            default_value="",
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

    ld.add_action(
        DeclareLaunchArgument(
            "use_ros2_control",
            default_value="false",
            description="Whether to use DiffDriveController for driving instead of direct Twist",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "rover_platform_override",
            default_value="",
            description="Override the rover platform (either clucky or testbed). If unset, hostname is used; defaults to clucky without hostname.",
            choices=["clucky", "testbed", ""],
        )
    )

    # nodes
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
            package="core_pkg",
            executable="core",
            name="core",
            output="both",
            parameters=[
                {"launch_mode": "anchor"},
                {
                    "use_ros2_control": LaunchConfiguration(
                        "use_ros2_control", default=False
                    )
                },
                {
                    "rover_platform_override": LaunchConfiguration(
                        "rover_platform_override", default="auto"
                    )
                },
            ],
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
        IncludeLaunchDescription(
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
    )

    ld.add_action(
        IncludeLaunchDescription(
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
    )

    return ld
