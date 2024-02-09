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
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    RCLCPP_INFO(this->get_logger(), "circle has been started.");

    /// parameters
    this->declare_parameter("frequency", 100);
    frequency_ = get_parameter("frequency").as_int();

    /// publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    /// services
    control_service_ = this->create_service<nuturtle_control::srv::Control>(
      "control", std::bind(
        &Circle::control_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    reverse_service_ = this->create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));

    stop_service_ = this->create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

    /// timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / frequency_),
      std::bind(&Circle::timer_callback, this));

  }

private:
  void timer_callback()
  {
    // RCLCPP_INFO(this->get_logger(), "circle timer callback");

    /// publish the velocity when the robot is not stopped
    if (turtlelib::almost_equal(velocity_, 0.0, 1e-6)) {
      cmd_vel_twist_.linear.x = 0.0;
      cmd_vel_twist_.angular.z = 0.0;
    } else {
      cmd_vel_twist_.linear.x = velocity_;
      cmd_vel_twist_.angular.z = velocity_ / radius_;
    }
    cmd_vel_pub_->publish(cmd_vel_twist_);
  }

  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> req,
    const std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    // RCLCPP_INFO(this->get_logger(), "circle control callback");
    velocity_ = req->velocity;  // angular velocity
    radius_ = req->radius;

  }

  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "circle reverse callback");
    /// reverse the direction of the circle
    velocity_ = -velocity_;
  }

  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    /// stop the motion of the robot
    RCLCPP_INFO(this->get_logger(), "circle stop callback");
    velocity_ = 0.0;
    cmd_vel_twist_.linear.x = velocity_;
    cmd_vel_twist_.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_twist_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  geometry_msgs::msg::Twist cmd_vel_twist_;

  int frequency_;
  float velocity_;
  float radius_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
