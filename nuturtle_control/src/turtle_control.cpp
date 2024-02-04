#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

    /// subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    sensor_data_sub_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &TurtleControl::sensor_data_callback, this,
        std::placeholders::_1));

    /// publishers
    wheel_cmd_pub_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    diff_drive_ = turtlelib::DiffDrive(wheel_radius_, track_width_);

    /// timers
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 100.0), std::bind(&TurtleControl::timer_callback, this));

  }

private:
  /// timer callback
  void timer_callback()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "timer_callback");
  }

  /// subscriber callbacks
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "cmd_vel_callback");

    turtlelib::Twist2D twist;
    twist.x = msg->linear.x;
    twist.y = msg->linear.y;
    twist.omega = msg->angular.z;
    turtlelib::WheelPositions_phi wheel_positions = diff_drive_.compute_wheel_velocities(twist);

    nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
    wheel_cmd.left_velocity = wheel_positions.phi_left;
    wheel_cmd.right_velocity = wheel_positions.phi_right;

    wheel_cmd_pub_->publish(wheel_cmd);
  }

  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "sensor_data_callback");

    // convert encoder ticks to radians
    double left_encoder = msg->left_encoder / encoder_ticks_per_rad_;
    double right_encoder = msg->right_encoder / encoder_ticks_per_rad_;

    turtlelib::WheelPositions_phi wheel_positions;
    wheel_positions.phi_left = left_encoder;
    wheel_positions.phi_right = right_encoder;
    // diff_drive_.update_configuration(wheel_positions);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.effort.resize(2);

    joint_state.name[0] = "left_wheel_joint";
    joint_state.position[0] = wheel_positions.phi_left;
    joint_state.name[1] = "right_wheel_joint";
    joint_state.position[1] = wheel_positions.phi_right;

    joint_state_pub_->publish(joint_state);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  double wheel_radius_;
  double track_width_;
  double motor_cmd_speed_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double collision_radius_;

  turtlelib::DiffDrive diff_drive_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
