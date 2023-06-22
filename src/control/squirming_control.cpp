#include "stm_control/control/squirming_control.hpp"

SquirmingControl::SquirmingControl() : Node("squirming_control") {
    steering_joint_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("steering_joint_controller/commands", 10);
    wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_base_controller/commands", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&SquirmingControl::timer_callback, this));
    counter_ = 0;
}

void SquirmingControl::timer_callback() {
    rclcpp::Time time_begin = this->now();

    auto wheel_speeds = std_msgs::msg::Float64MultiArray();
    wheel_speeds.data = {1, 1, 1, 1};


    auto steering_joint_position = std_msgs::msg::Float64MultiArray();
    steering_joint_position.data = {std::sin(counter_)};
    counter_ += M_PI/80;

    steering_joint_publisher_->publish(steering_joint_position);
    wheel_speed_publisher_->publish(wheel_speeds);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquirmingControl>());
    rclcpp::shutdown();
    return 0;
}