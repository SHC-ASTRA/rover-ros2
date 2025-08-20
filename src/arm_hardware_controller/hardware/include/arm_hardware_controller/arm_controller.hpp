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

#ifndef ASTRA_ARM__HARDWARE_HPP_
#define ASTRA_ARM__HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "ros2_interfaces_pkg/msg/arm_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using hardware_interface::return_type;

namespace astra_arm
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArmSystem : public hardware_interface::SystemInterface
{
public:
    // Boilerplate
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
private:
    // Comms with arm_pkg
    rclcpp::Publisher<ros2_interfaces_pkg::msg::ArmIK>::SharedPtr command_pub_;
    void state_sub_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;

    std::vector<double> cmd_;
    std::vector<double> pos_, vel_;
    std::chrono::steady_clock::time_point last_state_time_;

protected:
};

}  // namespace astra_arm

#endif  // ASTRA_ARM__HARDWARE_HPP_
