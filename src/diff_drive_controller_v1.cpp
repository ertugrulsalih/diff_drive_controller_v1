#include "diff_drive_controller_v1/diff_drive_controller_v1.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

DiffDriveController::DiffDriveController()
: Node("diff_drive_controller_v1_node"),
  left_wheel_position_(0.0), right_wheel_position_(0.0),
  x_pos_(0.0), y_pos_(0.0), theta_(0.0), prev_time_(0.0)
{
    // Parameters
    this->declare_parameter("wheel_radius", 0.165);
    this->declare_parameter("wheel_separation", 0.6);

    // Get parameter values
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    // Subscriber: /cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

    // Publisher: /odom and joint state
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer for publishing odometry and joint state
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // Update rate (10 Hz)
        std::bind(&DiffDriveController::publishOdometry, this));
}

DiffDriveController::~DiffDriveController() {}

void DiffDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Process the incoming cmd_vel message
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    // Calculate wheel velocities
    left_wheel_velocity_ = (2 * linear_velocity - angular_velocity * wheel_separation_) / (2 * wheel_radius_);
    right_wheel_velocity_ = (2 * linear_velocity + angular_velocity * wheel_separation_) / (2 * wheel_radius_);
}

void DiffDriveController::publishOdometry()
{
    // Time difference
    double current_time = this->now().seconds();
    double delta_time = current_time - prev_time_;
    prev_time_ = current_time;

    // Update robot position and orientation
    double delta_x = (left_wheel_velocity_ + right_wheel_velocity_) / 2.0 * delta_time;
    double delta_theta = (right_wheel_velocity_ - left_wheel_velocity_) / wheel_separation_ * delta_time;

    x_pos_ += delta_x * cos(theta_ + delta_theta / 2.0);
    y_pos_ += delta_x * sin(theta_ + delta_theta / 2.0);
    theta_ += delta_theta;

    // Create odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";

    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Publish odometry
    odom_pub_->publish(odom_msg);

    // TF Transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = x_pos_;
    transform.transform.translation.y = y_pos_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // Publish TF
    tf_broadcaster_->sendTransform(transform);

    // Joint State message
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name = {"left_wheel_joint", "right_wheel_joint"};

    // Calculate wheel positions
    left_wheel_position_ += left_wheel_velocity_ * delta_time;
    right_wheel_position_ += right_wheel_velocity_ * delta_time;

    // Add position and velocity information
    joint_state_msg.position = {left_wheel_position_, right_wheel_position_};
    joint_state_msg.velocity = {left_wheel_velocity_, right_wheel_velocity_};

    // Publish joint state
    joint_state_pub_->publish(joint_state_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveController>());
    rclcpp::shutdown();
    return 0;
}
