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
sensor_msgs::msg::JointState joint_state_msg;

auto left_wheel_vel{0.0}, right_wheel_vel{0.0}, left_joint_pos{0.0}, right_joint_pos{0.0},
left_joint_vel{0.0}, right_joint_vel{0.0};

auto sub_found{false};

TEST_CASE("Test turtle_control node", "[turtle_control]") {
  // Init test node
  auto node = rclcpp::Node::make_shared("turtle_control_test");

  // Create publishers
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto sensor_data_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
    "sensor_data", 10);

  // Create subscribers
  auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, [](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      // save the message
      wheel_cmd = *msg;
    });

  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [&node](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // save the message
      // RCLCPP_INFO_STREAM(node->get_logger(), "joint state: " <<joint_state.position[0] <<" " <<joint_state.position[1]);
      joint_state_msg = *msg;
    });


  // pure translation
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.1;
  twist.angular.z = 0.0;

  // test sensor_data_callback
  nuturtlebot_msgs::msg::SensorData sensor_data_msg;
  sensor_data_msg.left_encoder = 100;
  sensor_data_msg.right_encoder = 100;

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "sensor data: " << sensor_data_msg.left_encoder << " " << sensor_data_msg.right_encoder);

  double encoder_ticks_per_rad = 651.898646904;

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_pub->publish(twist);
    // sensor_data_pub->publish(sensor_data_msg); // not working here!!
    rclcpp::spin_some(node);
  }

  Twist2D twist2d;
  twist2d.x = twist.linear.x;
  twist2d.y = twist.linear.y;
  twist2d.omega = twist.angular.z;

  REQUIRE_THAT(
    wheel_cmd.left_velocity,
    Catch::Matchers::WithinAbs(126.0, 1.0e-6));
  REQUIRE_THAT(
    wheel_cmd.right_velocity,
    Catch::Matchers::WithinAbs(126.0, 1.0e-6));

  auto joint_left = sensor_data_msg.left_encoder / encoder_ticks_per_rad;
  auto joint_right = sensor_data_msg.right_encoder / encoder_ticks_per_rad;

  // pure rotation
  twist.linear.x = 0.0;
  twist.angular.z = 0.25;

  start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_pub->publish(twist);
    rclcpp::spin_some(node);
  }

  Twist2D twist2d_rot;
  twist2d_rot.x = twist.linear.x;
  twist2d_rot.y = twist.linear.y;
  twist2d_rot.omega = twist.angular.z;

  // RCLCPP_INFO(
  //   node->get_logger(), "wheel cmd: %i, %i", wheel_cmd.left_velocity, wheel_cmd.right_velocity);

  REQUIRE_THAT(
    wheel_cmd.left_velocity,
    Catch::Matchers::WithinAbs(-25.0, 1.0e-6));
  REQUIRE_THAT(
    wheel_cmd.right_velocity,
    Catch::Matchers::WithinAbs(25.0, 1.0e-6));

  // REQUIRE(0 == 1);
}

/// ################################# Citation ################################# ///
/// Nader helped me with this part. ///
/// ############################################################################ ///
void joint_states_callback(const sensor_msgs::msg::JointState msg)
{
  left_joint_pos = msg.position[0];
  right_joint_pos = msg.position[1];
  left_joint_vel = msg.velocity[0];
  right_joint_vel = msg.velocity[1];
}

TEST_CASE("Ticks to radians", "[controller]")
{
  sub_found = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  rclcpp::Time start_time = rclcpp::Clock().now();

  nuturtlebot_msgs::msg::SensorData sensor_data_msg;
  sensor_data_msg.left_encoder = 100;
  sensor_data_msg.right_encoder = 100;

  auto sensor_data_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
    "sensor_data", 10);
  auto joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, &joint_states_callback);

  while (rclcpp::ok() && !(sub_found) &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2)))
  {
    sensor_data_pub->publish(sensor_data_msg);
    rclcpp::spin_some(node);
  }
  REQUIRE_THAT(left_joint_pos, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(right_joint_pos, Catch::Matchers::WithinAbs(0.0, 1e-5));
}
