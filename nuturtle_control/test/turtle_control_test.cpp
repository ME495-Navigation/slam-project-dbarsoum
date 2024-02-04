#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;

nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
sensor_msgs::msg::JointState joint_state;

TEST_CASE("cmd_vel", "[cmd_vel]") {
  // Initialize ROS
  // rclcpp::init(0, nullptr);

  // Init test node
  auto node = rclcpp::Node::make_shared("turtle_control_test_node");

  // Create publishers
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto sensor_data_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
    "sensor_data", 10);

  // Create subscribers
  auto cmd_vel_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "cmd_vel", 10, [](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      // save the message
      wheel_cmd = *msg;
    });

  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // save the message
      joint_state = *msg;
    });

  // pure translation
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.1;
  twist.angular.z = 0.0;

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_pub->publish(twist);
    rclcpp::spin_some(node);
  }

  Twist2D twist2d;
  twist2d.x = twist.linear.x;
  twist2d.y = twist.linear.y;
  twist2d.omega = twist.angular.z;

  turtlelib::DiffDrive diff_drive;
  turtlelib::WheelPositions_phi wheel_positions = diff_drive.compute_wheel_velocities(twist2d);

  REQUIRE_THAT(
    wheel_cmd.left_velocity,
    Catch::Matchers::WithinAbs(wheel_positions.phi_left, 1.0e-6));
  REQUIRE_THAT(
    wheel_cmd.right_velocity,
    Catch::Matchers::WithinAbs(wheel_positions.phi_right, 1.0e-6));

  // pure rotation
  auto twist_rot = geometry_msgs::msg::Twist();
  twist_rot.linear.x = 0.0;
  twist_rot.angular.z = 0.1;

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_pub->publish(twist_rot);
    rclcpp::spin_some(node);
  }

  Twist2D twist2d_rot;
  twist2d_rot.x = 0.0;
  twist2d_rot.y = 0.0;
  twist2d_rot.omega = 0.1;

  turtlelib::WheelPositions_phi wheel_positions_rot = diff_drive.compute_wheel_velocities(
    twist2d_rot);

  REQUIRE_THAT(
    wheel_cmd.left_velocity,
    Catch::Matchers::WithinAbs(wheel_positions_rot.phi_left, 1.0e-6));
  REQUIRE_THAT(
    wheel_cmd.right_velocity,
    Catch::Matchers::WithinAbs(wheel_positions_rot.phi_right, 1.0e-6));

  // verifies that encoder data on sensors is converted to joint_states properly
  nuturtlebot_msgs::msg::SensorData sensor_data;
  sensor_data.left_encoder = 1;
  sensor_data.right_encoder = 1;

  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    sensor_data_pub->publish(sensor_data);
    rclcpp::spin_some(node);
  }

  REQUIRE(joint_state.position[0] == 1);
  REQUIRE(joint_state.position[1] == 1);

  REQUIRE(0 == 1);
}
