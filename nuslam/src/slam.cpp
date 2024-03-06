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
  // broacasters
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // publishers
  green_path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
  green_obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

  // subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SLAM::odom_callback, this, std::placeholders::_1));
  fake_sensor_data_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/fake_sensor", 10, std::bind(&SLAM::fake_sensor_data_callback, this, std::placeholders::_1));

  // timer
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / 5.0), std::bind(&SLAM::timer_callback, this));

}

private:
  // callback function for the timer
  void timer_callback()
  {
    //update the tranform
    turtlelib::Vector2D v;
    v.x = mu_.at(1);
    v.y = mu_.at(2);
    T_mr_ = turtlelib::Transform2D(v, mu_.at(0));

    // RCLCPP_INFO_STREAM(get_logger(), "T_mr_ translation: " << T_mr_.translation().x << " " << T_mr_.translation().y);

    T_mo_ = T_mr_ * T_or_.inv();
    // T_mo_ = turtlelib::Transform2D();

    // RCLCPP_INFO_STREAM(get_logger(), "T_mo_ translation: " << T_mo_.translation().x << " " << T_mo_.translation().y);

    // update the odometry messages
    update_odom_msgs();
    update_map_msgs();

    // publish the odometry messages
    tf_broadcaster_->sendTransform(odom_msg_);

    // publish the map messages
    tf_broadcaster_->sendTransform(map_msg_);

    // publish the path messages
    green_path_pub_->publish(path_msg_);

    // add green obstacles
    add_green_obstacles();
    green_obstacles_pub_->publish(green_obstacles_array_);
    
  
  }
  // prediction step
  void predict(double dx, double dy, double dtheta)
  {
    // arma::vec odom_update = arma::zeros(9);

    // calculate the new state of the robot (mu) 
    // state_pred = state + pred
    mu_bar_.at(0) = mu_.at(0) + dtheta;
    mu_bar_.at(1) = mu_.at(1) + dx;
    mu_bar_.at(2) = mu_.at(2) + dy;

    mu_bar_.at(3) = mu_.at(3);
    mu_bar_.at(4) = mu_.at(4);
    mu_bar_.at(5) = mu_.at(5);
    mu_bar_.at(6) = mu_.at(6);
    mu_bar_.at(7) = mu_.at(7);
    mu_bar_.at(8) = mu_.at(8);

    // A_r matrix
    A_r_ = arma::zeros(9, 9);
    A_r_.at(1, 0) = -dy;
    A_r_.at(2, 0) = dx;

    // A_t_ = arma::zeros(9, 9);

    // A_t matrix = identity matrix + A_r
    A_t_ = arma::eye(9, 9) + A_r_; // check if this is correct

    // sigma_bar matrix
    Q_.at(0, 0) = 0.01;
    Q_.at(1, 1) = 0.01;
    Q_.at(2, 2) = 0.01;
    Q_bar_.at(0, 0) = Q_.at(0, 0);
    Q_bar_.at(1, 1) = Q_.at(1, 1);
    Q_bar_.at(2, 2) = Q_.at(2, 2);

    // W matrix
    double w_t_ = 0.02;
    std::normal_distribution<double> W_{0.0, w_t_};
    arma::mat W = arma::zeros(9, 9);
    for (int i = 0; i < 9; i++) {
      W.at(i, i) = W_(generator_);
    }

    sigma_bar_ = A_t_ * sigma_ * A_t_.t() + W * Q_bar_ * W.t(); 
  }


  double get_yaw(geometry_msgs::msg::Quaternion q)
  {
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return yaw;
  }

  // callback function for the odometry
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "BEGINNING POSE" << prev_green_pose_.position.x << " " << prev_green_pose_.position.y);
    // calculate T_or
    turtlelib::Vector2D v;
    v.x = msg->pose.pose.position.x;
    v.y = msg->pose.pose.position.y;
    T_or_ = turtlelib::Transform2D(v, get_yaw(msg->pose.pose.orientation));

    // RCLCPP_INFO_STREAM(get_logger(), "T_or_ translation: " << T_or_.translation().x << " " << T_or_.translation().y);

    // RCLCPP_INFO_STREAM(get_logger(), "current x: " << msg->pose.pose.position.x << " current y: " << msg->pose.pose.position.y);

    // calculate the change in position and orientation
    double dx = msg->pose.pose.position.x - prev_green_pose_.position.x;
    double dy = msg->pose.pose.position.y - prev_green_pose_.position.y;
    double dtheta = turtlelib::normalize_angle(get_yaw(msg->pose.pose.orientation) - get_yaw(prev_green_pose_.orientation));

    // update the previous pose to current pose
    prev_green_pose_ = msg->pose.pose;

    // RCLCPP_INFO_STREAM(get_logger(), "ENDING POSE" << prev_green_pose_.position.x << " " << prev_green_pose_.position.y);

    // range and bearing
    // double r = sqrt(pow(dx, 2) + pow(dy, 2));
    // double phi = atan2(dy, dx);
    
    // prediction step
    predict(dx, dy, dtheta);

    // update the state and covariance
    // mu_ = mu_bar_;

    // sigma_ = sigma_bar_;
 
  }

  void update(int id, double x, double y)
  {
    // calculate the estimated range and bearing
    auto delta_x = mu_bar_.at(3 + 2 * id) - mu_bar_.at(1); 
    auto delta_y = mu_bar_.at(4 + 2 * id) - mu_bar_.at(2);
    auto d = pow(delta_x, 2) + pow(delta_y, 2);

    // calculate the jacobian
    arma::mat H_t = arma::zeros(2, 9);
    H_t.at(0, 1) = -delta_x / sqrt(d);
    H_t.at(0, 2) = -delta_y / sqrt(d);
    H_t.at(1, 0) = -1.0;
    H_t.at(1, 1) = delta_y / d;
    H_t.at(1, 2) = -delta_x / d;

    H_t.at(0, 3 + 2 * id) = delta_x / sqrt(d);
    H_t.at(0, 4 + 2 * id) = delta_y / sqrt(d);
    H_t.at(1, 3 + 2 * id) = -delta_y / d;
    H_t.at(1, 4 + 2 * id) = delta_x / d;

    // V -- random guassian distrubtion
    double v_t_ = 0.02;
    std::normal_distribution<double> V_{0.0, v_t_};
    arma::mat V = arma::zeros(2, 2);
    for (int i = 0; i < 2; i++) {
      V.at(i, i) = V_(generator_);
    }

    // calculate the kalman gain
    arma::mat K_t = arma::zeros(9, 2);
    K_t = sigma_bar_ * H_t.t() * arma::inv(H_t * sigma_bar_ * H_t.t() + V * R_ * V.t());

    // calculate the measurement
    arma::vec z_hat = calculate_r_phi(delta_x, delta_y);
    z_hat.at(1) = turtlelib::normalize_angle(z_hat.at(1) - mu_bar_.at(0));

    arma::vec z_t = calculate_r_phi(x, y);
    arma::vec z_diff = arma::zeros(2);
    z_diff.at(0) = z_t.at(0) - z_hat.at(0);
    z_diff.at(1) = turtlelib::normalize_angle(z_t.at(1) - z_hat.at(1));

    // update the state and covariance
    mu_ = mu_ + K_t * z_diff;
    sigma_ = (arma::eye(9, 9) - K_t * H_t) * sigma_bar_;

    mu_bar_ = mu_;
    sigma_bar_ = sigma_;

  }

  arma::vec calculate_r_phi(double x, double y)
  {
    auto r = sqrt(pow(x, 2) + pow(y, 2));
    auto phi = atan2(y, x);
    arma::vec z = {r, phi};
    return z;
  }

  // callback function for the fake sensor data
  void fake_sensor_data_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // neighbor_ids_ = {};
    // get id of obstacles
    for (int i = 0; i < int(msg->markers.size()); i++) {
      auto id = msg->markers[i].id; // int
      double x = msg->markers[i].pose.position.x;
      double y = msg->markers[i].pose.position.y;
      if (msg->markers[i].action == visualization_msgs::msg::Marker::ADD) {
        // if id is in neighbor_ids, update
        if (check_neighbors(id)) {
          update(id, x, y);
        }
        else {
          // if id is not in neighbor_ids, add to neighbor_ids
          neighbor_ids_.push_back(id);

          // set mu
          mu_bar_.at(3 + 2 * id) = mu_bar_.at(1) + x;
          mu_bar_.at(4 + 2 * id) = mu_bar_.at(2) + y;

          mu_.at(3 + 2 * id) = mu_bar_.at(3 + 2 * id);
          mu_.at(4 + 2 * id) = mu_bar_.at(4 + 2 * id);
        }
      }
    }
  }

  bool check_neighbors(int id)
  {
    for (int i = 0; i < int(neighbor_ids_.size()); i++) {
      if (neighbor_ids_.at(i) == id) {
        return true;
      }
    }
    return false;
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

    path_msg_.header.stamp = get_clock()->now();
    path_msg_.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = T_mr_.translation().x;
    pose.pose.position.y = T_mr_.translation().y;
    pose.pose.position.z = 0.0;

    path_msg_.poses.push_back(pose);
  }

  void update_map_msgs()
  {
    map_msg_.header.stamp = get_clock()->now();
    map_msg_.header.frame_id = "map";
    map_msg_.child_frame_id = "green/odom";
    map_msg_.transform.translation.x = T_mo_.translation().x;
    map_msg_.transform.translation.y = T_mo_.translation().y;
    map_msg_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, T_mo_.rotation());
    map_msg_.transform.rotation.x = q.x();
    map_msg_.transform.rotation.y = q.y();
    map_msg_.transform.rotation.z = q.z();
    map_msg_.transform.rotation.w = q.w();
  }

  void add_green_obstacles()
  {
    int num_obs = 3;
    visualization_msgs::msg::Marker green_obstacle;
    green_obstacles_array_.markers.clear();
    // for (int i = 0; i < int(num_obs); i++) {
    // RCLCPP_INFO_STREAM(get_logger(), "Neighbors size = " << neighbor_ids_.size());
    for (int i = 0; i < int(neighbor_ids_.size()); i++) {
      green_obstacle.header.stamp = get_clock()->now();
      green_obstacle.header.frame_id = "map";
      green_obstacle.ns = "green_obstacles";
      green_obstacle.id = neighbor_ids_.at(i);
      green_obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
      green_obstacle.action = visualization_msgs::msg::Marker::ADD;
      green_obstacle.pose.position.x = mu_.at(3 + 2 * neighbor_ids_.at(i));
      green_obstacle.pose.position.y = mu_.at(4 + 2 * neighbor_ids_.at(i));
      green_obstacle.pose.position.z = 0.25 / 2.0;
      green_obstacle.pose.orientation.x = 0.0;
      green_obstacle.pose.orientation.y = 0.0;
      green_obstacle.pose.orientation.z = 0.0;
      green_obstacle.pose.orientation.w = 1.0;
      green_obstacle.scale.x = 0.1;
      green_obstacle.scale.y = 0.1;
      green_obstacle.scale.z = 0.25;
      green_obstacle.color.r = 0.0;
      green_obstacle.color.g = 1.0;
      green_obstacle.color.b = 0.0;
      green_obstacle.color.a = 1.0;
      green_obstacles_array_.markers.push_back(green_obstacle);
    }
  }
  //
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr green_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr green_obstacles_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_data_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;

  // double rate_;
  geometry_msgs::msg::TransformStamped odom_msg_;
  geometry_msgs::msg::TransformStamped map_msg_;
  nav_msgs::msg::Path path_msg_;
  visualization_msgs::msg::MarkerArray green_obstacles_array_;

  geometry_msgs::msg::Pose prev_green_pose_;
  turtlelib::Transform2D T_or_;
  turtlelib::Transform2D T_mr_;
  turtlelib::Transform2D T_mo_;

  // arma::vec mu_ = {0.0, 0.0, 0.0, -0.5, -0.7, 0.8, -0.8, 0.4, 0.8};
  // arma::vec mu_bar_= {0.0, 0.0, 0.0, -0.5, -0.7, 0.8, -0.8, 0.4, 0.8};
  arma::mat mu_ = arma::zeros(9, 1);
  arma::mat mu_bar_ = arma::zeros(9, 1);
  arma::mat A_r_ = arma::zeros(9, 9);
  arma::mat A_t_ = arma::zeros(9, 9);
  arma::mat sigma_ = arma::eye(9, 9);
  arma::mat sigma_bar_ = arma::eye(9, 9);
  arma::mat Q_ = arma::zeros(3, 3);
  arma::mat Q_bar_ = arma::zeros(9, 9);
  arma::mat R_ = arma::eye(2, 2);

  std::default_random_engine generator_;

  std::vector<int> neighbor_ids_ = {};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAM>());
  rclcpp::shutdown();
  return 0;
}