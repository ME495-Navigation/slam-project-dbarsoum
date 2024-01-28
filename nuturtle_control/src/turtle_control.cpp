#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2_ros/transform_broadcaster.h"

// #include "nuturtle_description/config/"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    RCLCPP_INFO(this->get_logger(), "turtle_control has been started.");

    declare_parameter("wheel_radius", -1.0);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    // RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius_);
    if (wheel_radius_ < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_radius error");
      rclcpp::shutdown();
    }

    declare_parameter("track_width", -1.0);
    double track_width_ = get_parameter("track_width").as_double();
    if (track_width_ < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "track_width error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_speed", -1.0);
    double motor_cmd_speed_ = get_parameter("motor_cmd_speed").as_double();
    if (motor_cmd_speed_ < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_speed error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    double motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_per_rad_sec error");
      rclcpp::shutdown();
    }

    declare_parameter("encoder_ticks_per_rad", -1.0);
    double encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();  
    if (encoder_ticks_per_rad_ < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "encoder_ticks_per_rad error");
      rclcpp::shutdown();
    }

    declare_parameter("collision_radius", -1.0);
    double collision_radius_ = get_parameter("collision_radius").as_double();
    if (collision_radius_ < 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "collision_radius error");
      rclcpp::shutdown();
    }

    /// timers
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), std::bind(&TurtleControl::timer_callback, this));

  }
private:
  void timer_callback()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "timer_callback");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  double rate_;
  double wheel_radius_;
  double track_width_;
  double motor_cmd_speed_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}