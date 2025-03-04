#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



#Prevent making __pycache__ directories
from sys import dont_write_bytecode
dont_write_bytecode = True

def launch_setup(context, *args, **kwargs):
    # Retrieve the resolved value of the launch argument 'mode'
    mode = LaunchConfiguration('mode').perform(context)
    nodes = []

    if mode == 'anchor':
        # Launch every node and pass "anchor" as the parameter
        nodes.append(
            Node(
                package='arm_pkg',
                executable='arm',  # change as needed
                name='arm',
                output='both',
                parameters=[{'launch_mode': mode}]
            )
        )
        nodes.append(
            Node(
                package='core_pkg',
                executable='core',  # change as needed
                name='core',
                output='both',
                parameters=[{'launch_mode': mode}]
            )
        )
        nodes.append(
            Node(
                package='bio_pkg',
                executable='bio',  # change as needed
                name='bio',
                output='both',
                parameters=[{'launch_mode': mode}]
            )
        )
        nodes.append(
            Node(
                package='anchor_pkg',
                executable='anchor',  # change as needed
                name='anchor',
                output='both',
                parameters=[{'launch_mode': mode}]
            )
        )
    elif mode in ['arm', 'core', 'bio']:
        # Only launch the node corresponding to the provided mode.
        if mode == 'arm':
            nodes.append(
                Node(
                    package='arm_pkg',
                    executable='arm',
                    name='arm',
                    output='both',
                    parameters=[{'launch_mode': mode}]
                )
            )
        elif mode == 'core':
            nodes.append(
                Node(
                    package='core_pkg',
                    executable='core',
                    name='core',
                    output='both',
                    parameters=[{'launch_mode': mode}]
                )
            )
        elif mode == 'bio':
            nodes.append(
                Node(
                    package='bio_pkg',
                    executable='bio',
                    name='bio',
                    output='both',
                    parameters=[{'launch_mode': mode}]
                )
            )
    else:
        # If an invalid mode is provided, print an error.
        # (You might want to raise an exception or handle it differently.)
        print("Invalid mode provided. Choose one of: arm, core, bio, anchor.")

    return nodes

def generate_launch_description():
    declare_arg = DeclareLaunchArgument(
        'mode',
        default_value='anchor',
        description='Launch mode: arm, core, bio, or anchor'
    )

    return LaunchDescription([
        declare_arg,
        OpaqueFunction(function=launch_setup)
    ])
