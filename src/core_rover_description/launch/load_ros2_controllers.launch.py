#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the rover.

This script creates a launch description that starts the necessary controllers
for operating the rover's differential drive system in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Diff Drive Controller: Controls the rover's differential drive movement

Launch Sequence:
    1. Wait 5 seconds for Gazebo and controller manager to initialize
    2. Load Joint State Broadcaster (unconfigured state)
    3. Configure Joint State Broadcaster (unconfigured -> inactive)
    4. Activate Joint State Broadcaster (inactive -> active)
    5. Load Diff Drive Controller (unconfigured state)
    6. Configure Diff Drive Controller (unconfigured -> inactive)
    7. Activate Diff Drive Controller (inactive -> active)

Note: ROS2 controllers follow the state machine: unconfigured -> inactive -> active.
Each state transition must be done explicitly.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    """Generate a launch description for loading, configuring and activating robot controllers.

    Controllers follow the ROS2 control state machine: unconfigured -> inactive -> active.
    Each transition must be done explicitly in sequence.

    Returns:
        LaunchDescription: Launch description containing sequenced controller state transitions
    """

    # Load joint state broadcaster first
    load_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_broadcaster'],
        output='screen')

    # Configure joint state broadcaster (unconfigured -> inactive)
    configure_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'joint_broadcaster', 'inactive'],
        output='screen')

    # Activate joint state broadcaster (inactive -> active)
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'joint_broadcaster', 'active'],
        output='screen')

    # Load diff drive controller
    load_diff_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'diff_controller'],
        output='screen')

    # Configure diff drive controller (unconfigured -> inactive)
    configure_diff_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'diff_controller', 'inactive'],
        output='screen')

    # Activate diff drive controller (inactive -> active)
    start_diff_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'set_controller_state', 'diff_controller', 'active'],
        output='screen')

    # Add delay to allow controller manager to fully initialize
    delayed_start = TimerAction(
        period=5.0,
        actions=[load_joint_state_broadcaster_cmd]
    )

    # Register event handlers for sequencing
    # Configure joint state broadcaster after loading it (unconfigured -> inactive)
    configure_joint_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster_cmd,
            on_exit=[configure_joint_state_broadcaster_cmd]))

    # Activate joint state broadcaster after configuring it (inactive -> active)
    activate_joint_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_joint_state_broadcaster_cmd,
            on_exit=[start_joint_state_broadcaster_cmd]))

    # Load diff controller after joint state broadcaster is activated
    load_diff_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[load_diff_drive_controller_cmd]))

    # Configure diff controller after loading it (unconfigured -> inactive)
    configure_diff_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_diff_drive_controller_cmd,
            on_exit=[configure_diff_drive_controller_cmd]))

    # Activate diff controller after configuring it (inactive -> active)
    activate_diff_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_diff_drive_controller_cmd,
            on_exit=[start_diff_drive_controller_cmd]))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description in sequence
    ld.add_action(delayed_start)
    ld.add_action(configure_joint_broadcaster_event)
    ld.add_action(activate_joint_broadcaster_event)
    ld.add_action(load_diff_controller_event)
    ld.add_action(configure_diff_controller_event)
    ld.add_action(activate_diff_controller_event)

    return ld
