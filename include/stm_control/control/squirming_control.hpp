#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class SquirmingControl: public rclcpp::Node {
    public:
        SquirmingControl();
    private:
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time past_callback;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_joint_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_publisher_;
        double counter_;
};