// #include "catch_ros2/catch_ros2.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "std_srvs/srv/empty.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "nuturtlebot_msgs/msg/wheel_commands.hpp"
// #include "nuturtlebot_msgs/msg/sensor_data.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "turtlelib/diff_drive.hpp"
// #include "turtlelib/geometry2d.hpp"
// #include "turtlelib/se2d.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "tf2_ros/transform_broadcaster.h"
// #include "tf2/LinearMath/Quaternion.h"
// #include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals;
// using namespace turtlelib;

// TEST_CASE("odom_test_init", "[odom_test_init]") {
//   // test case for initial pose service

//   auto node = rclcpp::Node::make_shared("turtle_odom_test");

//   // Declare a parameter on node
//   node->declare_parameter<float>("wheel_radius", 0.033);
//   node->declare_parameter<float>("track_width", 0.160);
//   node->declare_parameter<std::string>("body_id", "base_link");
//   node->declare_parameter<std::string>("odom_id", "odom");
//   node->declare_parameter<std::string>("wheel_left", "left_wheel");
//   node->declare_parameter<std::string>("wheel_right", "right_wheel");

//   // get value of parameters
//   const auto WHEEL_RADIUS = node->get_parameter("wheel_radius").get_parameter_value().get<float>();
//   const auto TRACK_WIDTH = node->get_parameter("track_width").get_parameter_value().get<float>();
//   const auto BODY_ID = node->get_parameter("body_id").get_parameter_value().get<std::string>();
//   const auto ODOM_ID = node->get_parameter("odom_id").get_parameter_value().get<std::string>();
//   const auto WHEEL_LEFT =
//     node->get_parameter("wheel_left").get_parameter_value().get<std::string>();
//   const auto WHEEL_RIGHT =
//     node->get_parameter("wheel_right").get_parameter_value().get<std::string>();

//   // create client for initial pose service
//   auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

//   rclcpp::Time start_time = rclcpp::Clock().now();

//   bool initial_pose_srv_found = false;

//   while (
//     rclcpp::ok() &&
//     ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(2))
//   )
//   {
//     // wait for service to be available (1 second or 0 second??)
//     if (client->wait_for_service(0s)) {
//       initial_pose_srv_found = true;
//       break;
//     }

//     rclcpp::spin_some(node);
//   }

//   // test assertions
//   CHECK(initial_pose_srv_found);
// }

// TEST_CASE("Test odom_test_tf", "[odom_test_tf]") {
//   // test case for odometry transform
//   // uses a tf2_listener to verify that a transform from odom to a base_footprint is being published
//   // transform should be the identity transformation, the joint states will not change throughout the test

//   auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

//   // Declare a parameter on node
//   node->declare_parameter<float>("wheel_radius", 0.033);
//   node->declare_parameter<float>("track_width", 0.160);
//   node->declare_parameter<std::string>("body_id", "base_link");
//   node->declare_parameter<std::string>("odom_id", "odom");
//   node->declare_parameter<std::string>("wheel_left", "left_wheel");
//   node->declare_parameter<std::string>("wheel_right", "right_wheel");

//   // get value of parameters
//   const auto WHEEL_RADIUS = node->get_parameter("wheel_radius").get_parameter_value().get<float>();
//   const auto TRACK_WIDTH = node->get_parameter("track_width").get_parameter_value().get<float>();
//   const auto BODY_ID = node->get_parameter("body_id").get_parameter_value().get<std::string>();
//   const auto ODOM_ID = node->get_parameter("odom_id").get_parameter_value().get<std::string>();
//   const auto WHEEL_LEFT =
//     node->get_parameter("wheel_left").get_parameter_value().get<std::string>();
//   const auto WHEEL_RIGHT =
//     node->get_parameter("wheel_right").get_parameter_value().get<std::string>();

//   // create transform broadcaster
//   auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

//   // create odometry message
//   auto odom_msg = nav_msgs::msg::Odometry();
//   odom_msg.header.frame_id = ODOM_ID;
//   odom_msg.child_frame_id = BODY_ID;
//   // odom_msg.pose.pose.position.x = 0.0;
//   // odom_msg.pose.pose.position.y = 0.0;
//   // odom_msg.pose.pose.position.z = 0.0;
//   // odom_msg.pose.pose.orientation.x = 0.0;
//   // odom_msg.pose.pose.orientation.y = 0.0;
//   // odom_msg.pose.pose.orientation.z = 0.0;
//   // odom_msg.pose.pose.orientation.w = 1.0;

//   // create odometry transform
//   auto odom_tf = geometry_msgs::msg::TransformStamped();
//   odom_tf.header.frame_id = ODOM_ID;
//   odom_tf.child_frame_id = BODY_ID;

//   odom_tf.transform.translation.x = 0.0;


// }
