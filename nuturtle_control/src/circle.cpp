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

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
  RCLCPP_INFO(this->get_logger(), "circle has been started.");

  /// timer
  timer_ = this->create_wall_timer(
      100ms, std::bind(&Circle::timer_callback, this));
      
  }
private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "circle timer callback");
  }
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}