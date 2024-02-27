/// \file
/// \brief turtle control node
/// PARAMETERS

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <algorithm>

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

    declare_parameter("wheel_radius", 0.0);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    // RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius_);
    if (wheel_radius_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_radius error");
      rclcpp::shutdown();
    }

    declare_parameter("track_width", 0.0);
    track_width_ = get_parameter("track_width").as_double();
    if (track_width_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "track_width error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_max", 0.0);
    motor_cmd_speed_ = get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_speed_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_speed error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_per_rad_sec error");
      rclcpp::shutdown();
    }

    declare_parameter("encoder_ticks_per_rad", 0.0);
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    if (encoder_ticks_per_rad_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "encoder_ticks_per_rad error");
      rclcpp::shutdown();
    }

    declare_parameter("collision_radius", 0.0);
    collision_radius_ = get_parameter("collision_radius").as_double();
    if (collision_radius_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "collision_radius error");
      rclcpp::shutdown();
    }

    /// subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    sensor_data_sub_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "/red/sensor_data", 10, std::bind(
        &TurtleControl::sensor_data_callback, this,
        std::placeholders::_1));

    /// publishers
    wheel_cmd_pub_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);

    // time
    previous_ = 0.0;
  }

private:
  /// subscriber callbacks
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "cmd_vel_callback");
    /// need to go from rad/s to mcu
    turtlelib::Twist2D twist;
    twist.x = msg->linear.x;
    twist.y = msg->linear.y;
    twist.omega = msg->angular.z;
    turtlelib::WheelPositions_phi wheel_positions = diff_drive_.compute_wheel_velocities(twist); // rad/s

    // RCLCPP_INFO_STREAM(this->get_logger(), "left_velocity: " << wheel_positions.phi_left << " right_velocity: " << wheel_positions.phi_right);

    wheel_positions.phi_left /= motor_cmd_per_rad_sec_;
    wheel_positions.phi_right /= motor_cmd_per_rad_sec_;
    // RCLCPP_INFO_STREAM(this->get_logger(), "motor_cmd_per_rad_sec_: " << motor_cmd_per_rad_sec_);

    /// saturation
    wheel_positions.phi_left = std::clamp(
      wheel_positions.phi_left, -motor_cmd_speed_,
      motor_cmd_speed_);
    wheel_positions.phi_right = std::clamp(
      wheel_positions.phi_right, -motor_cmd_speed_,
      motor_cmd_speed_);

    nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
    wheel_cmd.left_velocity = wheel_positions.phi_left; // mcu
    wheel_cmd.right_velocity = wheel_positions.phi_right; // mcu

    // RCLCPP_INFO_STREAM(this->get_logger(), "left_velocity_mcu: " << wheel_cmd.left_velocity << " right_velocity_mcu: " << wheel_cmd.right_velocity);

    wheel_cmd_pub_->publish(wheel_cmd);
  }

  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "sensor_data_callback");
    auto current_time = msg->stamp.sec + msg->stamp.nanosec * 1e-9;

    // RCLCPP_INFO_STREAM(this->get_logger(), "left_encoder: " << msg->left_encoder);
    // RCLCPP_INFO_STREAM(this->get_logger(), "right_encoder: " << msg->right_encoder);

    turtlelib::WheelPositions_phi wheel_positions;
    wheel_positions.phi_left = double(msg->left_encoder) / encoder_ticks_per_rad_;
    wheel_positions.phi_right = double(msg->right_encoder) / encoder_ticks_per_rad_;
    // diff_drive_.update_configuration(wheel_positions);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state.position = {wheel_positions.phi_left, wheel_positions.phi_right};
    // previous_ = current_time;
    double dt = current_time - previous_;
    previous_ = current_time;
    joint_state.velocity = {wheel_positions.phi_left / dt, wheel_positions.phi_right / dt};

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
  double previous_;

  turtlelib::DiffDrive diff_drive_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
