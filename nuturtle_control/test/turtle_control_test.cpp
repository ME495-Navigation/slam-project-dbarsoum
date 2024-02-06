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
      joint_state = *msg;
    });


  // pure translation
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.1;
  twist.angular.z = 0.0;

  // test sensor_data_callback
  nuturtlebot_msgs::msg::SensorData sensor_data;
  sensor_data.left_encoder = 100;
  sensor_data.right_encoder = 100;
  

  RCLCPP_INFO_STREAM(node->get_logger(), "sensor data: " <<sensor_data.left_encoder <<" " <<sensor_data.right_encoder);

  double encoder_ticks_per_rad = 651.898646904;

  // RCLCPP_INFO(node->get_logger(), "joint state: %f, %f", joint_state.position[0], joint_state.position[1]);

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    cmd_vel_pub->publish(twist);
    sensor_data_pub->publish(sensor_data); // not working here!!
    rclcpp::spin_some(node);
  }

  Twist2D twist2d;
  twist2d.x = twist.linear.x;
  twist2d.y = twist.linear.y;
  twist2d.omega = twist.angular.z;

  // turtlelib::DiffDrive diff_drive(wheel_track_, wheel_radius_);
  DiffDrive diff_drive(0.160, 0.033);
  WheelPositions_phi wheel_positions = diff_drive.compute_wheel_velocities(twist2d);
  RCLCPP_INFO(
  node->get_logger(), "Wheel velocities: %f, %f", wheel_positions.phi_left,
  wheel_positions.phi_right);

  RCLCPP_INFO(node->get_logger(), "wheel cmd: %i, %i", wheel_cmd.left_velocity, wheel_cmd.right_velocity);
  REQUIRE_THAT(
    wheel_cmd.left_velocity,
    Catch::Matchers::WithinAbs(int(wheel_positions.phi_left), 1.0e-6));
  REQUIRE_THAT(
    wheel_cmd.right_velocity,
    Catch::Matchers::WithinAbs(int(wheel_positions.phi_right), 1.0e-6));


  // RCLCPP_INFO_STREAM(node->get_logger(), "joint state: " <<joint_state.position[0] <<" " <<joint_state.position[1]);
  // RCLCPP_INFO(node->get_logger(), "sensor data: %d, %d", sensor_data.left_encoder, sensor_data.right_encoder);
  REQUIRE_THAT(
    joint_state.position.at(0),
    Catch::Matchers::WithinAbs(sensor_data.left_encoder / encoder_ticks_per_rad, 1.0e-6));
  REQUIRE_THAT(
    joint_state.position.at(1),
    Catch::Matchers::WithinAbs(sensor_data.right_encoder / encoder_ticks_per_rad, 1.0e-6));

//   // pure rotation
//   twist.linear.x = 0.0;
//   twist.angular.z = 0.1;

//   // auto twist_rot = geometry_msgs::msg::Twist();
//   // twist_rot.linear.x = 0.0;
//   // twist_rot.angular.z = 0.1;
//   start_time = rclcpp::Clock().now();
//   while (
//     rclcpp::ok() &&
//     ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
//   )
//   {
//     cmd_vel_pub->publish(twist);
//     rclcpp::spin_some(node);
//   }

//   Twist2D twist2d_rot;
//   twist2d_rot.x = twist.linear.x;
//   twist2d_rot.y = twist.linear.y;
//   twist2d_rot.omega = twist.angular.z;

//   WheelPositions_phi wheel_positions_rot = diff_drive.compute_wheel_velocities(
//     twist2d_rot);

//   RCLCPP_INFO(
//   node->get_logger(), "Wheel velocities: %f, %f", wheel_positions_rot.phi_left,
//   wheel_positions_rot.phi_right);

//   RCLCPP_INFO(node->get_logger(), "wheel cmd: %i, %i", wheel_cmd.left_velocity, wheel_cmd.right_velocity);

//   REQUIRE_THAT(
//     wheel_cmd.left_velocity,
//     Catch::Matchers::WithinAbs(int(wheel_positions_rot.phi_left), 1.0e-6));
//   REQUIRE_THAT(
//     wheel_cmd.right_velocity,
//     Catch::Matchers::WithinAbs(int(wheel_positions_rot.phi_right), 1.0e-6));

//   // // test sensor_data_callback
//   // nuturtlebot_msgs::msg::SensorData sensor_data;
//   // sensor_data.left_encoder = 100;
//   // sensor_data.right_encoder = 100;

//   // double encoder_ticks_per_rad = 651.898646904;

//   // start_time = rclcpp::Clock().now();
//   // while (
//   //   rclcpp::ok() &&
//   //   ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
//   // )
//   // {
//   //   sensor_data_pub->publish(sensor_data);
//   //   rclcpp::spin_some(node);
//   // }

//   // RCLCPP_INFO(node->get_logger(), "joint state: %f, %f", joint_state.position[0], joint_state.position[1]);
//   // RCLCPP_INFO(node->get_logger(), "sensor data: %d, %d", sensor_data.left_encoder, sensor_data.right_encoder);
//   // REQUIRE_THAT(
//   //   joint_state.position[0],
//   //   Catch::Matchers::WithinAbs(sensor_data.left_encoder / encoder_ticks_per_rad, 1.0e-6));
//   // REQUIRE_THAT(
//   //   joint_state.position[1],
//   //   Catch::Matchers::WithinAbs(sensor_data.right_encoder / encoder_ticks_per_rad, 1.0e-6));

  // REQUIRE(0 == 1);
// }
}
