#include "limo_controller_node.hpp"
#include <cmath>

LimoController::LimoController() 
    : Node("limo_controller"), 
      gen_(std::random_device{}()),
      dist_(-5.0, 5.0)  // Random distribution for x and y between -5 and 5
{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&LimoController::odomCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LimoController::controlLoop, this));

    generateNewGoal();
}

void LimoController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_theta_ = yaw;
}

void LimoController::controlLoop()
{
    double dx = goal_x_ - current_x_;
    double dy = goal_y_ - current_y_;
    double distance_error = std::sqrt(dx*dx + dy*dy);
    
    double desired_theta = std::atan2(dy, dx);
    double theta_error = desired_theta - current_theta_;
    
    // Normalize theta_error to [-pi, pi]
    while (theta_error > M_PI) theta_error -= 2*M_PI;
    while (theta_error < -M_PI) theta_error += 2*M_PI;
    
    double v = k_v_ * distance_error;
    double w = k_w_ * theta_error;
    
    // Limit maximum velocities
    v = std::min(std::max(v, -max_v_), max_v_);
    w = std::min(std::max(w, -max_w_), max_w_);
    
    // Check if we're close enough to the goal
    if (distance_error < DISTANCE_THRESHOLD) {
        generateNewGoal();
        RCLCPP_INFO(this->get_logger(), "Goal reached! New goal: (%.2f, %.2f)", goal_x_, goal_y_);
    }

    // Create and publish Twist message
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = v;
    twist_msg.angular.z = w;
    cmd_vel_pub_->publish(twist_msg);
    
    // Debug output
    RCLCPP_INFO(this->get_logger(), "Current: (%.2f, %.2f, %.2f), Goal: (%.2f, %.2f)",
                current_x_, current_y_, current_theta_, goal_x_, goal_y_);
    RCLCPP_INFO(this->get_logger(), "Control: (v: %.2f, w: %.2f)", v, w);
}

void LimoController::generateNewGoal()
{
    goal_x_ = dist_(gen_);
    goal_y_ = dist_(gen_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LimoController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}