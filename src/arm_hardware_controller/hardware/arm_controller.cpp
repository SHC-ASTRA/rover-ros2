// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arm_hardware_controller/arm_controller.hpp"
#include <string>
#include <vector>

#include <iostream>

namespace astra_arm
{

CallbackReturn ArmSystem::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn ArmSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // reset values always when configuring hardware
    for (const auto & [name, descr] : joint_state_interfaces_)
    {
        set_state(name, 0.0);
    }
    for (const auto & [name, descr] : joint_command_interfaces_)
    {
        set_command(name, 0.0);
    }
    for (const auto & [name, descr] : sensor_state_interfaces_)
    {
        set_state(name, 0.0);
    }

    return CallbackReturn::SUCCESS;
}

return_type ArmSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    // TODO(pac48) set sensor_states_ values from subscriber

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
        const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
        const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
        set_state(name_vel, get_command(name_vel));
        set_state(name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds());
    }
    return return_type::OK;
}

return_type ArmSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    return return_type::OK;
}

void ArmSystem::state_sub_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for (std::size_t i = 0; i < msg->name.size(); i++)
    {
        if (joint_state_interfaces_.find(msg->name[i]) != joint_state_interfaces_.end())
        {
            set_state(msg->name[i] + "/" + hardware_interface::HW_IF_POSITION, msg->position[i]);
            if (!msg->position.empty())
                set_state(msg->name[i] + "/" + hardware_interface::HW_IF_VELOCITY, msg->velocity[i]);
        }
    }
    last_state_time_ = std::chrono::steady_clock::now();
}

}  // namespace astra_arm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  astra_arm::ArmSystem, hardware_interface::SystemInterface)
