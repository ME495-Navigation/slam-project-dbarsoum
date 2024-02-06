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
#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;

double counter = 0.0;

TEST_CASE("Test circle_test_node", "[circle_node]") {
  auto node = rclcpp::Node::make_shared("turtle_circle_test");

  // subscriber
  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](const geometry_msgs::msg::Twist::SharedPtr) {
      counter += 1.0;
    });

  // service client
  auto control_srv_client = node->create_client<nuturtle_control::srv::Control>("control");

  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    if (control_srv_client->wait_for_service(0s)) {
      break;
    }
    rclcpp::spin_some(node);
  }

  auto request = std::make_shared<nuturtle_control::srv::Control::Request>();
  request->velocity = 1.0;
  request->radius = 0.1;

  auto result = control_srv_client->async_send_request(request);

  while (rclcpp::spin_until_future_complete(node, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
  }

  start_time = rclcpp::Clock().now();
  rclcpp::Time end_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
  )
  {
    end_time = rclcpp::Clock().now();
    rclcpp::spin_some(node);
  }

  // check the frequency of the cmd_vel topic
  RCLCPP_INFO(node->get_logger(), "counter: %f", counter / (end_time - start_time).seconds());
  REQUIRE_THAT(counter / (end_time - start_time).seconds(), Catch::Matchers::WithinAbs(100.0, 5.0));

  REQUIRE(1.0 == 0.0);  // force test to fail (to see the output of the counter value
}
