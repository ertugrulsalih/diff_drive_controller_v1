#ifndef DIFF_DRIVE_CONTROLLER_V1_HPP_
#define DIFF_DRIVE_CONTROLLER_V1_HPP_

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class DiffDriveController : public rclcpp::Node
{
public:
    DiffDriveController();
    ~DiffDriveController();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publishOdometry();
    
    // Robot state variables
    double left_wheel_position_;
    double right_wheel_position_;
    double x_pos_;
    double y_pos_;
    double theta_;
    double left_wheel_velocity_;
    double right_wheel_velocity_;
    double prev_time_;
    double wheel_radius_;
    double wheel_separation_;

    // ROS components
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // DIFF_DRIVE_CONTROLLER_V1_HPP_
