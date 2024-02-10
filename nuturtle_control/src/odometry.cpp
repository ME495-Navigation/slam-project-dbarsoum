#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance.h"
#include "geometry_msgs/msg/twist_with_covariance.h"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("Odometry")
  {
    /// publishes odometry messages and the odometry transform
    // RCLCPP_INFO(this->get_logger(), "odometry has been started.");

    declare_parameter("wheel_radius", 0.0);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    // RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius_);
    if (wheel_radius_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_radius error");
      rclcpp::shutdown();
    }

    declare_parameter("track_width", 0.0);
    double track_width_ = get_parameter("track_width").as_double();
    if (track_width_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "track_width error");
      rclcpp::shutdown();
    }

    /// parameters
    declare_parameter("body_id", std::string(" "));
    body_id_ = get_parameter("body_id").as_string();
    if (body_id_ == " ") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "body_id error");
      rclcpp::shutdown();
    }

    declare_parameter("odom_id", std::string("odom"));
    odom_id_ = get_parameter("odom_id").as_string();
    if (odom_id_ == " ") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "odom_id error");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_left", std::string(" "));
    wheel_left_ = get_parameter("wheel_left").as_string();
    if (wheel_left_ == " ") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_left error");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_right", std::string(" "));
    wheel_right_ = get_parameter("wheel_right").as_string();
    if (wheel_right_ == " ") {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_right error");
      rclcpp::shutdown();
    }

    /// subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));

    /// publish odometry messages
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    /// services
    service_InitPose_ = this->create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose",
      std::bind(
        &Odometry::initial_pose_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    /// Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);
    /// timers
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 100.0), std::bind(&Odometry::timer_callback, this));

  }

private:
  /// timer callback
  void timer_callback()
  {
    /// need to publish the odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = odom_id_;
    odom.child_frame_id = body_id_;
    odom.pose.pose.position.x = q_diff_.x_;
    odom.pose.pose.position.y = q_diff_.y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, q_diff_.theta_);
    // q.normalize();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odometry_publisher_->publish(odom);

    /// need to publish the odometry transform
    t_.header.stamp = this->get_clock()->now();
    t_.header.frame_id = odom_id_;
    t_.child_frame_id = body_id_;

    t_.transform.translation.x = q_diff_.x_;
    t_.transform.translation.y = q_diff_.y_;
    t_.transform.rotation.x = q.x();
    t_.transform.rotation.y = q.y();
    t_.transform.rotation.z = q.z();
    t_.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t_);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "joint_state_callback");

    // turtlelib::WheelPositions_phi wheel_positions;
    wheel_positions_.phi_left = msg->position.at(0);
    wheel_positions_.phi_right = msg->position.at(1);

    diff_drive_.update_configuration(wheel_positions_);
    q_diff_ = diff_drive_.get_configuration();

    /// need to publish the odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = odom_id_;
    odom.child_frame_id = body_id_;
    odom.pose.pose.position.x = q_diff_.x_;
    odom.pose.pose.position.y = q_diff_.y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, q_diff_.theta_);
    // q.normalize();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    // odometry_publisher_->publish(odom);

    /// need to publish the odometry transform
    t_.header.stamp = this->get_clock()->now();
    t_.header.frame_id = odom_id_;
    t_.child_frame_id = body_id_;

    t_.transform.translation.x = q_diff_.x_;
    t_.transform.translation.y = q_diff_.y_;
    t_.transform.rotation.x = q.x();
    t_.transform.rotation.y = q.y();
    t_.transform.rotation.z = q.z();
    t_.transform.rotation.w = q.w();

    // tf_broadcaster_->sendTransform(t_);
  }

  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> req,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "initial_pose_callback");

    // turtlelib::Configuration_q q_diff;
    q_diff_.x_ = req->x;
    q_diff_.y_ = req->y;
    q_diff_.theta_ = req->theta;

    diff_drive_.set_configuration(q_diff_);

    // res->msg_feedback = "Initial pose set";
    /// need to publish the odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = odom_id_;
    odom.child_frame_id = body_id_;
    odom.pose.pose.position.x = q_diff_.x_;
    odom.pose.pose.position.y = q_diff_.y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, q_diff_.theta_);
    // q.normalize();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odometry_publisher_->publish(odom);

    /// need to publish the odometry transform
    t_.header.stamp = this->get_clock()->now();
    t_.header.frame_id = odom_id_;
    t_.child_frame_id = body_id_;

    t_.transform.translation.x = q_diff_.x_;
    t_.transform.translation.y = q_diff_.y_;
    t_.transform.rotation.x = q.x();
    t_.transform.rotation.y = q.y();
    t_.transform.rotation.z = q.z();
    t_.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t_);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr service_InitPose_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped t_;
  std::string body_id_;
  std::string odom_id_;
  double wheel_radius_;
  double track_width_;
  std::string wheel_left_;
  std::string wheel_right_;

  turtlelib::DiffDrive diff_drive_;
  turtlelib::Configuration_q q_diff_;
  turtlelib::WheelPositions_phi wheel_positions_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
