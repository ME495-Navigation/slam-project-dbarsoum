#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

class SLAM : public rclcpp::Node
{
public:
  SLAM()
  : Node("slam")
  {
  // declare_parameter("rate", 200.0)
  // rate_ = get_parameter("rate").as_double();
  declare_parameter("obstacles/x", std::vector<double>{});
  declare_parameter("obstacles/y", std::vector<double>{});
  declare_parameter("obstacles/r", std::vector<double>{});

  obstacles_x_ = get_parameter("obstacles/x").as_double_array();
  obstacles_y_ = get_parameter("obstacles/y").as_double_array();
  obstacles_r_ = get_parameter("obstacles/r").as_double_array();
  if (obstacles_x_.size() != obstacles_y_.size() || obstacles_x_.size() != obstacles_r_.size()) {
    RCLCPP_ERROR(get_logger(), "Obstacles are not the same length.");
    rclcpp::shutdown();
  }

  // declare_parameter("wheel_radius", 0.0);
  //   wheel_radius_ = get_parameter("wheel_radius").as_double();
  //   // RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius_);
  //   if (wheel_radius_ == 0.0) {
  //     RCLCPP_ERROR_STREAM(get_logger(), "wheel_radius error");
  //     rclcpp::shutdown();
  //   }

  //   declare_parameter("track_width", 0.0);
  //   track_width_ = get_parameter("track_width").as_double();
  //   if (track_width_ == 0.0) {
  //     RCLCPP_ERROR_STREAM(get_logger(), "track_width error");
  //     rclcpp::shutdown();
  //   }

  
  // broacasters
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // publishers
  green_path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
  // rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
  green_obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
  timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
  // joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10); // for blue robot

  // subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SLAM::odom_callback, this, std::placeholders::_1));
  fake_sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>("/fake_data", 10, std::bind(&SLAM::fake_sensor_data_callback, this, std::placeholders::_1));

  // timer
  timer_ = create_wall_timer(
    (1s / 100), std::bind(&SLAM::timer_callback, this));

  // // previous pose
  // prev_green_pose_.header.stamp = get_clock()->now();
  // prev_green_pose_.header.frame_id = "green/odom"; // check this
  prev_green_pose_.position.x = 0.0;
  prev_green_pose_.position.y = 0.0;
  prev_green_pose_.position.z = 0.0;

  // diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);
}

private:
  // callback function for the timer
  void timer_callback()
  {

    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++ * (1 / 200.0) * 1e3;
    timestep_pub_->publish(message);
    // add green obstacles
    add_green_obstacles();

    // // calculate T_mo
    // T_mo_ = q_diff_ * T_or_.inv();

    // // updateTransform("map", "green/odom");
    // tf_broadcaster_->sendTransform(T_mo_);

    // // updateTransform("green/odom", "green/base_footprint", )
    // tf_broadcaster_->sendTransform(T_or_);

    // addPath();
    // green_path_pub_->publish(msg_path_);
  
  }
  // prediction step
  void predict(double dx, double dy, double dtheta)
  {
    // calculate the new state of the robot (mu) 
    // state_pred = state + pred
    mu_bar_.at(0) = mu_.at(0) + dx;
    mu_bar_.at(1) = mu_.at(1) + dy;
    mu_bar_.at(2) = mu_.at(2) + dtheta;

    // mu_bar_.at(3) = mu_.at(3);
    // mu_bar_.at(4) = mu_.at(4);
    // mu_bar_.at(5) = mu_.at(5);
    // mu_bar_.at(6) = mu_.at(6);
    // mu_bar_.at(7) = mu_.at(7);
    // mu_bar_.at(8) = mu_.at(8);

    // A_r matrix
    A_r_ = arma::zeros(9, 9);
    A_r_.at(2, 0) = -dy;
    A_r_.at(3, 0) = dx;

    // A_t matrix = identity matrix + A_r
    A_t_ = arma::eye(9, 9) + A_r_; // check if this is correct

    // sigma_bar matrix
    Q_.at(0, 0) = 0.01;
    Q_.at(1, 1) = 0.01;
    Q_.at(2, 2) = 0.01;
    Q_bar_.at(0, 0) = Q_.at(0, 0);
    Q_bar_.at(1, 1) = Q_.at(1, 1);
    Q_bar_.at(2, 2) = Q_.at(2, 2);
    sigma_bar_ = A_t_ * sigma_ * A_t_.t() + Q_bar_; 
  }


  double get_yaw(geometry_msgs::msg::Quaternion q)
  {
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return yaw;
  }

  // callback function for the odometry
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // calculate T_or
    turtlelib::Vector2D v;
    v.x = msg->pose.pose.position.x;
    v.y = msg->pose.pose.position.y;
    T_or_ = turtlelib::Transform2D(v, get_yaw(msg->pose.pose.orientation));

    // calculate the change in position and orientation
    double dx = msg->pose.pose.position.x - prev_green_pose_.position.x;
    double dy = msg->pose.pose.position.y - prev_green_pose_.position.y;
    double dtheta = get_yaw(msg->pose.pose.orientation) - get_yaw(prev_green_pose_.orientation);

    // update the previous pose to current pose
    prev_green_pose_ = msg->pose.pose;

    // range and bearing
    double r = sqrt(pow(dx, 2) + pow(dy, 2));
    double phi = atan2(dy, dx);
    
    // prediction step
    predict(dx, dy, dtheta);

    // update the state and covariance
    mu_ = mu_bar_;

    sigma_ = sigma_bar_;

  }

  void update(std::string id, double x, double y)
  {
    // calculate the estimated range and bearing
    auto delta_x = prev_green_pose_.position.x * mu_.at(0);
    auto delta_y = prev_green_pose_.position.y * mu_.at(1);
    double r_hat = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    double phi_hat = atan2(delta_y, delta_x);

    auto d = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

    // calculate the jacobian
    arma::mat H_t = arma::zeros(2, 9);


    // update the state and covariance
  }
  // callback function for the fake sensor data
  void fake_sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    // get id of obstacles
    // for ()
    rclcpp::Time time = get_clock()->now(); // filler
  }

  // update odom msgs
  void update_odom_msgs()
  {
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.header.frame_id = "green/odom";
    odom_msg_.child_frame_id = "green/base_footprint";
    odom_msg_.transform.translation.x = T_or_.translation().x;
    odom_msg_.transform.translation.y = T_or_.translation().y;
    odom_msg_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, T_or_.rotation());
    odom_msg_.transform.rotation.x = q.x();
    odom_msg_.transform.rotation.y = q.y();
    odom_msg_.transform.rotation.z = q.z();
    odom_msg_.transform.rotation.w = q.w();

  }

  // void update_map_msgs()
  // {
  //   map_msg_.header.stamp = get_clock()->now();
  //   map_msg_.header.frame_id = "map";
  //   map_msg_.child_frame_id = "green/odom";
  //   map_msg_.pose.pose.position.x = q_diff_.translation().x;
  //   map_msg_.pose.pose.position.y = q_diff_.translation().y;
  //   map_msg_.pose.pose.position.z = 0.0;
  //   tf2::Quaternion q;
  //   q.setRPY(0.0, 0.0, q_diff_.rotation());
  //   map_msg_.pose.pose.orientation.x = q.x();
  //   map_msg_.pose.pose.orientation.y = q.y();
  //   map_msg_.pose.pose.orientation.z = q.z();
  //   map_msg_.pose.pose.orientation.w = q.w();
  // }

  void add_green_obstacles()
  {
    int num_obs = 3;
      if (timestep_ % int(200.0 / 5.0) == 0.0) {
        visualization_msgs::msg::MarkerArray green_obstacles_array;
        visualization_msgs::msg::Marker green_obstacle;
        green_obstacles_array.markers.clear();
        for (int i = 0; i < int(num_obs); i++) {
          green_obstacle.header.stamp = get_clock()->now();
          green_obstacle.header.frame_id = "nusim/world";
          green_obstacle.ns = "green_obstacles";
          green_obstacle.id = i;
          green_obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
          green_obstacle.action = visualization_msgs::msg::Marker::ADD;
          // green_obstacle.pose.position.x = mu_.at(3 + 2 * i);
          green_obstacle.pose.position.x = obstacles_x_.at(i);
          // green_obstacle.pose.position.y = mu_.at(4 + 2 * i);
          green_obstacle.pose.position.y = obstacles_y_.at(i);
          green_obstacle.pose.position.z = 0.0;
          green_obstacle.pose.orientation.x = 0.0;
          green_obstacle.pose.orientation.y = 0.0;
          green_obstacle.pose.orientation.z = 0.0;
          green_obstacle.pose.orientation.w = 1.0;
          green_obstacle.scale.x = obstacles_r_.at(i) * 2.0;
          green_obstacle.scale.y = obstacles_r_.at(i) * 2.0;
          green_obstacle.scale.z = 0.25;
          green_obstacle.color.r = 0.0;
          green_obstacle.color.g = 1.0;
          green_obstacle.color.b = 0.0;
          green_obstacle.color.a = 1.0;
          green_obstacles_array.markers.push_back(green_obstacle);
        }
        green_obstacles_pub_->publish(green_obstacles_array);
      }
  }

  // update the transform
  void updateTransform(std::string frame_id, std::string child_frame_id)
  {
    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = frame_id;
    t_.child_frame_id = child_frame_id;
  }

  void updateTransform(std::string frame_id, std::string child_frame_id, double x, double y, double theta)
  {
    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = frame_id;
    t_.child_frame_id = child_frame_id;
    t_.transform.translation.x = x;
    t_.transform.translation.y = y;
    t_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t_.transform.rotation.x = q.x();
    t_.transform.rotation.y = q.y();
    t_.transform.rotation.z = q.z();
    t_.transform.rotation.w = q.w();
  }

  // // add path
  //  void addPath()
  // {
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.stamp = get_clock()->now();
  //   pose.header.frame_id = "nusim/world";
  //   pose.pose.position.x = t_.transform.translation.x;
  //   pose.pose.position.y = t_.transform.translation.y;
  //   pose.pose.position.z = 0.0;
  //   pose.pose.orientation.x = t_.transform.rotation.x;
  //   pose.pose.orientation.y = t_.transform.rotation.y;
  //   pose.pose.orientation.z = t_.transform.rotation.z;
  //   pose.pose.orientation.w = t_.transform.rotation.w;

  //   msg_path_.header.stamp = get_clock()->now();
  //   msg_path_.header.frame_id = "nusim/world";
  //   msg_path_.poses.push_back(pose);
  // }

  //
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr green_obstacles_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr fake_sensor_data_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;

  // double rate_;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  std::vector<double> obstacles_r_;
  geometry_msgs::msg::TransformStamped odom_msg_;

  int timestep_ = 0;
  geometry_msgs::msg::Pose prev_green_pose_;
  geometry_msgs::msg::TransformStamped t_;
  nav_msgs::msg::Path green_path_;
  turtlelib::DiffDrive diff_drive_;
  turtlelib::Transform2D T_or_;
  // turtlelib::Transform2D T_mr_;
  turtlelib::Transform2D T_mo_;

  arma::vec mu_{0.0, 0.0, 0.0, -0.5, -0.7, 0.8, -0.8, 0.4, 0.8};
  arma::vec mu_bar_{0.0, 0.0, 0.0, -0.5, -0.7, 0.8, -0.8, 0.4, 0.8};
  arma::mat A_r_{9, 9, arma::fill::zeros};
  arma::mat A_t_{9, 9, arma::fill::zeros};
  arma::mat sigma_{9, 9, arma::fill::zeros};
  arma::mat sigma_bar_{9, 9, arma::fill::zeros};
  arma::mat Q_{3, 3, arma::fill::zeros}; // turnable
  arma::mat Q_bar_{9, 9, arma::fill::zeros};
  arma::mat R_{2, 2, arma::fill::eye}; // turnable -- should be identity matrix I think
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAM>());
  rclcpp::shutdown();
  return 0;
}