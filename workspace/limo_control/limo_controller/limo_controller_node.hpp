#ifndef LIMO_CONTROLLER_NODE_HPP
#define LIMO_CONTROLLER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <random>

class LimoController : public rclcpp::Node
{
public:
    LimoController();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();
    void generateNewGoal();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double current_x_, current_y_, current_theta_;
    double goal_x_, goal_y_;

    std::mt19937 gen_;
    std::uniform_real_distribution<> dist_;

    const double k_v_ = 0.5;  // Linear velocity gain
    const double k_w_ = 1.0;  // Angular velocity gain
    const double max_v_ = 0.5;  // Maximum linear velocity
    const double max_w_ = 1.0;  // Maximum angular velocity
    const double DISTANCE_THRESHOLD = 0.1;  // 10 centimeters
};

#endif // LIMO_CONTROLLER_NODE_HPP