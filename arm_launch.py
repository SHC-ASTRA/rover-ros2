#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition



#Prevent making __pycache__ directories
from sys import dont_write_bytecode
dont_write_bytecode = True

def launch_setup(context, *args, **kwargs):
    # Retrieve the resolved value of the launch argument 'mode'
    mode = LaunchConfiguration('mode').perform(context)
    nodes = []

    # Arm
    # package_dir = get_package_share_directory('arm_pkg')
    package_dir = "/home/david/repos/rover-ros2/src/arm_pkg"  # TODO: copy files to share and point there
    urdf_file_name = 'arm12.urdf'
    urdf = os.path.join(package_dir, "urdf", urdf_file_name)
    # urdf = "/home/david/repos/rover-ros2/src/arm_pkg/urdf/" + urdf_file_name
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_path = os.path.join(package_dir, 'viz.rviz')

    if mode == 'anchor':
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='both',
                parameters=[{'robot_description': robot_desc}],
                arguments=[urdf],
                on_exit=Shutdown()
            )
        )
        nodes.append(
            Node(
                package='arm_pkg',
                executable='arm',  # change as needed
                name='arm',
                output='both',
                parameters=[{'launch_mode': 'anchor'}],
                on_exit=Shutdown()
            )
        )
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='both',
                arguments=['-d', rviz_config_path],
                on_exit=Shutdown()
            )
        )
        # nodes.append(
        #     Node(
        #         package='anchor_pkg',
        #         executable='anchor',  # change as needed
        #         name='anchor',
        #         output='both',
        #         parameters=[{'launch_mode': 'anchor'}],
        #         on_exit=Shutdown()
        #     )
        # )
    else:
        # If an invalid mode is provided, print an error.
        print("Invalid mode provided. Choose one of: arm, core, bio, anchor, ptz.")

    return nodes

def generate_launch_description():
    declare_arg = DeclareLaunchArgument(
        'mode',
        default_value='anchor',
        description='Launch mode: arm, core, bio, anchor, or ptz'
    )

    return LaunchDescription([
        declare_arg,
        OpaqueFunction(function=launch_setup)
    ])
