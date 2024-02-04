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

TEST_CASE("cmd_vel", "[cmd_vel]") {
  // Initialize ROS
  rclcpp::init(0, nullptr);

  // Init test node
  auto node = rclcpp::Node::make_shared("turtle_control_test_node");

  // Create publisher to publish cmd_vel messages
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // create a subscriber to check if the cmd_vel message was published
  auto cmd_vel_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "cmd_vel", 10, [](const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
      // save the message
      wheel_cmd = *msg;
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


  REQUIRE(0 == 1);
}
