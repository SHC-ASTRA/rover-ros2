#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <locale>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;


/**
 * `input` into `args` separated by `delim`; equivalent to Python's `.split`;
 * Example:  "ctrl,led,on" => `{"ctrl","led","on"}`
 * @param input String to be separated
 * @param delim char which separates parts of input
 * @author David Sharpe, for ASTRA
 * @deprecated Use function without delim parameter
 */
std::vector<std::string> split(const std::string& input, const char delim = ',');


class LatencyTester : public rclcpp::Node
{
public:
    LatencyTester()
    : Node("latency_tester"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/anchor/relay", 10);
        timer_ = this->create_wall_timer(
        1000ms, std::bind(&LatencyTester::timer_callback, this));
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/anchor/debug",
            10,
            std::bind(&LatencyTester::response_callback, this, std::placeholders::_1));

        target_mcu_ = this->declare_parameter<std::string>("target_mcu", "core");
    }

private:
    void timer_callback()  // Send ping to embedded at 1 Hz
    {
        auto message = std_msgs::msg::String();
        message.data = "can_relay_tovic," + target_mcu_ + ",1," + std::to_string(count_++) + '\n';
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Sending ping #%ld to %s", count_, target_mcu_.c_str());
        last_send_stamp_ = this->get_clock()->now();
    }

    void response_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto now = this->now();
        std::vector<std::string> args = split(msg->data, ',');
        if (args.size() < 3 || args[0] != "can_relay_fromvic" || args[2] != "1")
            return;

        if (args[1] != target_mcu_) {
            RCLCPP_INFO(this->get_logger(), "Received pong from different MCU: %s", args[1].c_str());
            return;
        }

        // TODO: add topic for this so we can plot with MATLAB while using core/arm
        RCLCPP_INFO(this->get_logger(), "Received pong from %s after %lf ms", target_mcu_.c_str(),
                    (now - last_send_stamp_).nanoseconds() / 1000000.0);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    rclcpp::Time last_send_stamp_;
    std::string target_mcu_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatencyTester>());
    rclcpp::shutdown();
    return 0;
}


std::vector<std::string> split(const std::string& input, const char delim) {
    // Modified from
    // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c

    std::vector<std::string> args;

    // if empty input for some reason, don't do anything
    if (input.length() == 0)
        return args;

    size_t last = 0;
    size_t next = 0;
    while ((next = input.find(delim, last)) != std::string::npos)
    {
        args.push_back(input.substr(last, next-last));
        last = next + 1;
    }
    args.push_back(input.substr(last));

    return args;
}
