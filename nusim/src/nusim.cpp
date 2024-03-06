/// \file
/// \brief nusim node -- simulator in rviz
///
/// PARAMETERS:
///     \param rate (double): Timer callback frequency [Hz]
///     \param x0_ (double): Initial x coordinate of the robot [m]
///     \param y0_ (double): Initial y coordinate of the robot [m]
///     \param theta0_ (double): Initial theta angle of the robot [radians]
///     \param arena_x_length_ (double): Length of arena in x direction [m]
///     \param arena_y_length_ (double): Length of arena in y direction [m]
///     \param obstacles_x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles_y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
///     \param wheel_radius (double): The radius of the wheels [m]
///     \param track_width (double): The distance between the wheels [m]
///     \param motor_cmd_max (double): Maximum motor command unit value
///     \param motor_cmd_per_rad_sec (double): Motor command unit per rad/s conversion factor
///     \param encoder_ticks_per_rad (double): Encoder ticks per 1 radian turn
///     \param collision_radius (double): Robot collision radius [m]
///
/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): Simulation timestep
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): Marker for the obstacles in the arena
///     \param ~/walls (visualization_msgs::msg::MarkerArray): Marker for the arena
///     \param red/sensor_data (nuturtlebot_msgs::msg::SensorData): Current simulated sensor
///            readings
///
/// SUBSCRIBES:
///     \param red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Reads twist to update robot
///
/// SERVERS:
///     \param ~/reset (std_srvs::srv::Empty): Resets simulation to initial state
///     \param ~/teleport (nusim::srv::Teleport): Teleports robot to a specific pose
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts the robot

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
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {
    /// \brief function initializes takes in wheel commands and
    ///        integrates to ticks to send sensor data to turtle control
    /// \param rate (double): Timer callback frequency [Hz]
    /// \param x0_ (double): Initial x coordinate of the robot [m]
    /// \param y0_ (double): Initial y coordinate of the robot [m]
    /// \param theta0_ (double): Initial theta angle of the robot [radians]
    /// \param arena_x_length_ (double): Length of arena in x direction [m]
    /// \param arena_y_length_ (double): Length of arena in y direction [m]
    /// \param obstacles_x_ (std::vector<double>): Vector of x coordinates for each obstacle [m]
    /// \param obstacles_y_ (std::vector<double>): Vector of y coordinates for each obstacle [m]
    /// \param obstacles_r_ (double): Radius of cylindrical obstacles [m]
    /// \param wheel_radius (double): The radius of the wheels [m]
    /// \param track_width (double): The distance between the wheels [m]
    /// \param motor_cmd_max (double): Maximum motor command unit value
    /// \param motor_cmd_per_rad_sec (double): Motor command unit per rad/s conversion factor
    /// \param encoder_ticks_per_rad (double): Encoder ticks per 1 radian turn
    /// \param collision_radius (double): Robot collision radius [m]

    /// declares a parameter
    declare_parameter("rate", 200.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_x_length", 5.0);
    declare_parameter("arena_y_length", 5.0);
    declare_parameter("obstacles/x", std::vector<double>{});
    declare_parameter("obstacles/y", std::vector<double>{});
    declare_parameter("obstacles/r", std::vector<double>{});
    declare_parameter("input_noise", 5.0);
    declare_parameter("slip_fraction", 0.5);
    declare_parameter("basic_sensor_variance", 0.01);
    declare_parameter("max_range", 3.0);
    declare_parameter("collision_radius", 0.11);
    declare_parameter("max_range_lidar", 3.5);
    declare_parameter("min_range", 0.12);
    declare_parameter("angle_increment", 0.01745);
    declare_parameter("number_of_samples", 360);
    declare_parameter("resolution", 0.01745);
    declare_parameter("noise_level", 0.01);


    /// gets the value of the parameter
    rate_ = get_parameter("rate").as_double();
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    theta0_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    input_noise_ = get_parameter("input_noise").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").as_double();
    max_range_ = get_parameter("max_range").as_double();
    collision_radius_ = get_parameter("collision_radius").as_double();
    max_range_lidar_ = get_parameter("max_range_lidar").as_double();
    // max_range_lidar_ = 10.0;
    min_range_ = get_parameter("min_range").as_double();
    angle_increment_ = get_parameter("angle_increment").as_double();
    number_of_samples_ = get_parameter("number_of_samples").as_int();
    resolution_ = get_parameter("resolution").as_double();
    noise_level_ = get_parameter("noise_level").as_double();

    double obstacles_height = 0.25;
    double wall_height = 0.25;

    /// check to make sure that the obstacles are the same length
    obstacles_x_ = get_parameter("obstacles/x").as_double_array();
    obstacles_y_ = get_parameter("obstacles/y").as_double_array();
    obstacles_r_ = get_parameter("obstacles/r").as_double_array();
    if (obstacles_x_.size() != obstacles_y_.size() || obstacles_x_.size() != obstacles_r_.size()) {
      RCLCPP_ERROR(get_logger(), "Obstacles are not the same length.");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_radius", 0.0);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    // RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius_);
    if (wheel_radius_ == 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_radius error");
      rclcpp::shutdown();
    }

    declare_parameter("track_width", 0.0);
    track_width_ = get_parameter("track_width").as_double();
    if (track_width_ == 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "track_width error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_max", 0.0);
    motor_cmd_speed_ = get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_speed_ == 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_speed error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ == 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "motor_cmd_per_rad_sec error");
      rclcpp::shutdown();
    }

    declare_parameter("encoder_ticks_per_rad", 0.0);
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    if (encoder_ticks_per_rad_ == 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "encoder_ticks_per_rad error");
      rclcpp::shutdown();
    }

    /// Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// creates a publisher
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_policy);
    obstacle_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      qos_policy);

    sensor_data_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data", 10);

    fake_sensor_data_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", qos_policy);

    laser_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "laser_scan", 10);

    red_path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);

    /// subscribers
    wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));
    /// wheel_cmd gets remapped in launch file

    /// services
    service_ =
      create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_teleport_ =
      create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));

    diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);
    /// timers
    timer_ = create_wall_timer(
      (1s / rate_), std::bind(&Nusim::timer_callback, this));

    // Publish the markers
    double wall_thickness = 0.1;
    visualization_msgs::msg::MarkerArray marker_array;
    addWall(marker_array, arena_x_length_ / 2.0 + wall_thickness / 2.0, 0.0, "top_wall", 1);
    addWall(marker_array, 0.0, arena_y_length_ / 2.0 + wall_thickness / 2.0, "right_wall", 2);
    addWall(marker_array, -arena_x_length_ / 2.0 - wall_thickness / 2.0, 0.0, "bottom_wall", 3);
    addWall(marker_array, 0.0, -arena_y_length_ / 2.0 - wall_thickness / 2.0, "left_wall", 4);
    marker_pub_->publish(marker_array);

    // Publish the obstacles
    visualization_msgs::msg::MarkerArray obstacle_array;
    for (int i = 0; i < int(obstacles_x_.size()); i++) {
      addObstacle(obstacle_array, obstacles_x_.at(i), obstacles_y_.at(i), obstacles_r_.at(i), "obstacle", i);
    }
    obstacle_pub_->publish(obstacle_array);
  }

private:
  /// \brief timer_callback that publishes the arena, obstacles, sensor data, and broadcasts the transform
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    /// need the timestep in ms
    message.data = timestep_++ *(1 / rate_) * 1e3;
    // RCLCPP_INFO(get_logger(), "Publishing: '%ld'", message.data);
    timestep_publisher_->publish(message);

    publish_sensor_data();

    updateTransform();
    tf_broadcaster_->sendTransform(t_);

    addPath();
    red_path_pub_->publish(msg_path_);

    // Fake Obstacles: Publish the fake sensor data at 5Hz -- c.10 (basic sensor)
    // std::normal_distribution<double> noise_gauss_fake_{0.0, basic_sensor_variance_};
    std::normal_distribution<double> noise_gauss_fake_{0.0, 0.0};
    if (timestep_ % int(rate_ / 5.0)  == 0.0) {
      // TODO: fix the placement of the lidar markers--showing above the walls
      // RCLCPP_INFO(get_logger(), "Publishing fake sensor data", timestep_);
      visualization_msgs::msg::MarkerArray fake_sensor_array;
      visualization_msgs::msg::Marker fake_sensor;
      fake_sensor_array.markers.clear(); // clear the array
      for (int i = 0; i < int(obstacles_x_.size()); i++) {
        fake_sensor.header.frame_id = "red/base_footprint";
        // fake_sensor.header.frame_id = "nusim/world";
        fake_sensor.header.stamp = get_clock()->now();
        fake_sensor.ns = "fake_sensor";
        fake_sensor.id = i;
        fake_sensor.type = visualization_msgs::msg::Marker::CYLINDER;
        double x_diff_ = obstacles_x_.at(i) - diff_drive_.get_configuration().x_;
        double y_diff_ = obstacles_y_.at(i) - diff_drive_.get_configuration().y_;
        double theta = diff_drive_.get_configuration().theta_;
        fake_sensor.pose.position.x = x_diff_ * cos(theta) + y_diff_ * sin(theta) + noise_gauss_fake_(generator_);
        fake_sensor.pose.position.y = -x_diff_ * sin(theta) + y_diff_ * cos(theta) + noise_gauss_fake_(generator_);
        fake_sensor.pose.position.z = 0.25/2.0;
        fake_sensor.pose.orientation.x = 0.0;
        fake_sensor.pose.orientation.y = 0.0;
        fake_sensor.pose.orientation.z = 0.0;
        fake_sensor.pose.orientation.w = 1.0;
        fake_sensor.color.r = 1.0;
        fake_sensor.color.g = 1.0;
        fake_sensor.color.b = 0.0;
        fake_sensor.color.a = 1.0;
        fake_sensor.scale.x = obstacles_r_.at(i) * 2.0; // diameter
        fake_sensor.scale.y = obstacles_r_.at(i) * 2.0; // diameter
        fake_sensor.scale.z = 0.25;

        // check if within range
        auto dist_ = std::sqrt(std::pow(obstacles_x_.at(i) - t_.transform.translation.x, 2) +
                      std::pow(obstacles_y_.at(i) - t_.transform.translation.y, 2));

        if (dist_ < max_range_) {
          fake_sensor.action = visualization_msgs::msg::Marker::ADD;
        } else {
          fake_sensor.action = visualization_msgs::msg::Marker::DELETE;
        }

        fake_sensor_array.markers.push_back(fake_sensor);
      }
      // publish the fake sensor data of the obstacles
      fake_sensor_data_pub_->publish(fake_sensor_array);

      // Laser Scan
      updateLaserScan();
      laser_scan_pub_->publish(laser_scan_);
    }
  }

  /// \brief reset_callback brings robot back to origin
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    x0_ = 0.0;
    y0_ = 0.0;
    theta0_ = 0.0;

    // RCLCPP_INFO(get_logger(), "Resetting timestep to 0.");
  }
  /// \brief teleports the robot to specified location
  /// \param request x, y, and theta location to teleoport
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x0_ = request->x;
    y0_ = request->y;
    theta0_ = request->theta;

    // Read message content and assign it to
    // corresponding tf variables
    updateTransform(x0_, y0_, theta0_);
    tf_broadcaster_->sendTransform(t_);
    
  }

  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    std::normal_distribution<double> noise_gauss_{0.0, input_noise_};
    std::uniform_real_distribution<double> noise_uniform_{-slip_fraction_, slip_fraction_};
    /// need to convert the wheel commands (mcu) to rad/s
    wheel_velocity_left_ = msg->left_velocity * motor_cmd_per_rad_sec_;  // need rad/s so need to multiply by motor_cmd_per_rad_sec_
    wheel_velocity_right_ = msg->right_velocity * motor_cmd_per_rad_sec_;
    

    if (turtlelib::almost_equal(wheel_velocity_left_, 0.0)) {
      wheel_velocity_left_ = 0.0;
    }
    else {
      wheel_velocity_left_ += noise_gauss_(generator_); //* (1 + noise_uniform_(generator_));
    }
    if (turtlelib::almost_equal(wheel_velocity_right_, 0.0)) {
      wheel_velocity_right_ = 0.0;
    }
    else {
      wheel_velocity_right_ += noise_gauss_(generator_); //+ (1 + noise_uniform_(generator_));
    }

    sensor_data_.left_encoder = sensor_data_.left_encoder + (wheel_velocity_left_) * (1 / rate_) * encoder_ticks_per_rad_;
    sensor_data_.right_encoder = sensor_data_.right_encoder + wheel_velocity_right_ * (1 / rate_) * encoder_ticks_per_rad_;

    // 
    wheel_velocity_left_ = wheel_velocity_left_ * (1 + noise_uniform_(generator_));
    wheel_velocity_right_ = wheel_velocity_right_ * (1 + noise_uniform_(generator_));

    wheel_positions_.phi_left = wheel_positions_.phi_left + wheel_velocity_left_ * (1 / rate_);  // rad
    wheel_positions_.phi_right = wheel_positions_.phi_right + wheel_velocity_right_ * (1 / rate_);

    diff_drive_.update_configuration(wheel_positions_);  // fk
    q_diff_ = diff_drive_.get_configuration();

    // collision detection
    for (int i = 0; i < int(obstacles_x_.size()); i++) {
      auto x_obs_loc = obstacles_x_.at(i);
      auto y_obs_loc = obstacles_y_.at(i);
      auto r_obs = obstacles_r_.at(i);
      auto dist_robot_obstacle = std::sqrt(std::pow(x_obs_loc - q_diff_.x_, 2) +
                      std::pow(y_obs_loc - q_diff_.y_, 2));
      if (dist_robot_obstacle < (collision_radius_ + r_obs)) {
        RCLCPP_INFO(get_logger(), "Collision detected");
        // move robot's center along line between robot center and obstacle center so collision circles are tangent
        turtlelib::Vector2D center_to_obstacle;
        center_to_obstacle.x = x_obs_loc - q_diff_.x_;
        center_to_obstacle.y = y_obs_loc - q_diff_.y_;
        
        turtlelib::Vector2D norm_center_line;
        auto mag_vect = std::sqrt(std::pow(center_to_obstacle.x, 2) + std::pow(center_to_obstacle.y, 2));
        norm_center_line.x = center_to_obstacle.x / mag_vect;
        norm_center_line.y = center_to_obstacle.y / mag_vect;

        q_diff_.x_ = x_obs_loc - (collision_radius_ + r_obs) * norm_center_line.x;
        q_diff_.y_ = y_obs_loc - (collision_radius_ + r_obs) * norm_center_line.y;

        // set configuration
        diff_drive_.set_configuration(q_diff_);
        q_diff_ = diff_drive_.get_configuration();
      }
    }
    // // update transformation
    updateTransform(q_diff_.x_, q_diff_.y_, q_diff_.theta_);
  }

  void publish_sensor_data()
  {
    sensor_data_pub_->publish(sensor_data_);
  }

  void addWall(
    visualization_msgs::msg::MarkerArray & marker_array, double x, double y,
    const std::string & name, int id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.25/2.0;
    if (name == "right_wall" || name == "left_wall") {
      // green -- vertical walls -- no rotation needed
      marker.pose.orientation.x = 1.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.scale.x = arena_x_length_;
      marker.scale.y = 0.1;
      marker.scale.z = 0.25;
    } else {
      // blue -- top and bottom walls -- needed rotation
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = arena_y_length_;
      marker.scale.z = 0.25;
    }

    marker_array.markers.push_back(marker);
  }

  // create cylindrical obstacles are given locations
  void addObstacle(
    visualization_msgs::msg::MarkerArray & marker_array, double x, double y,
    double r, const std::string & name, int id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.25/2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = r * 2.0;
    marker.scale.y = r * 2.0;
    marker.scale.z = 0.25;

    marker_array.markers.push_back(marker);
  }

  void updateTransform()
  {
    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
  }

  void updateTransform(double x, double y, double theta)
  {
    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = "nusim/world";
    t_.child_frame_id = "red/base_footprint";
    t_.transform.translation.x = x;
    t_.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t_.transform.rotation.x = q.x();
    t_.transform.rotation.y = q.y();
    t_.transform.rotation.z = q.z();
    t_.transform.rotation.w = q.w();
  }

  void addPath()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = "nusim/world";
    pose.pose.position.x = t_.transform.translation.x;
    pose.pose.position.y = t_.transform.translation.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = t_.transform.rotation.x;
    pose.pose.orientation.y = t_.transform.rotation.y;
    pose.pose.orientation.z = t_.transform.rotation.z;
    pose.pose.orientation.w = t_.transform.rotation.w;

    msg_path_.header.stamp = get_clock()->now();
    msg_path_.header.frame_id = "nusim/world";
    msg_path_.poses.push_back(pose);
  }

  void updateLaserScan()
  {
    laser_scan_.ranges.clear();

    laser_scan_.header.stamp = get_clock()->now();
    laser_scan_.header.frame_id = "red/base_scan";
    laser_scan_.angle_min = 0.0;
    laser_scan_.angle_max = 2 * turtlelib::PI;
    laser_scan_.angle_increment = angle_increment_;
    laser_scan_.range_min = min_range_;
    laser_scan_.range_max = max_range_lidar_;

    for (int i = 0; i < number_of_samples_; i++) {
      double gamma = i * resolution_;
      double scan_value = 3.0 * max_range_lidar_;

      turtlelib::Vector2D current_robot_location;
      current_robot_location.x = diff_drive_.get_configuration().x_;
      current_robot_location.y = diff_drive_.get_configuration().y_;
      double current_robot_theta = diff_drive_.get_configuration().theta_;
      turtlelib::Transform2D T_wrobot(current_robot_location, current_robot_theta);

      turtlelib::Transform2D T_robotlaser(gamma);

      turtlelib::Transform2D T_wlaser = T_wrobot * T_robotlaser;
      turtlelib::Transform2D T_laserw = T_wlaser.inv();

      // wall top right
      turtlelib::Point2D wall_top_right;
      wall_top_right.x = arena_x_length_ / 2.0;
      wall_top_right.y = arena_y_length_ / 2.0;

      // wall top left
      turtlelib::Point2D wall_top_left;
      wall_top_left.x = -arena_x_length_ / 2.0;
      wall_top_left.y = arena_y_length_ / 2.0;

      // wall bottom left
      turtlelib::Point2D wall_bottom_left;
      wall_bottom_left.x = -arena_x_length_ / 2.0;
      wall_bottom_left.y = -arena_y_length_ / 2.0;

      // wall bottom right
      turtlelib::Point2D wall_bottom_right;
      wall_bottom_right.x = arena_x_length_ / 2.0;
      wall_bottom_right.y = -arena_y_length_ / 2.0;

      turtlelib::Point2D wall_top_right_lidar = T_laserw(wall_top_right);
      turtlelib::Point2D wall_top_left_lidar = T_laserw(wall_top_left);
      turtlelib::Point2D wall_bottom_left_lidar = T_laserw(wall_bottom_left);
      turtlelib::Point2D wall_bottom_right_lidar = T_laserw(wall_bottom_right);

      double slope = (wall_top_right_lidar.y - wall_top_left_lidar.y) / (wall_top_right_lidar.x - wall_top_left_lidar.x);
      double intercept = wall_top_right_lidar.y - slope * wall_top_right_lidar.x;

      // intersection of lidar beam with wall
      double x_intersection = -intercept / slope;

      if (x_intersection >= 0) {
        scan_value = x_intersection;
      }


      slope = (wall_top_right_lidar.y - wall_bottom_right_lidar.y) / (wall_top_right_lidar.x - wall_bottom_right_lidar.x);
      intercept = wall_top_right_lidar.y - slope * wall_top_right_lidar.x;

      // intersection of lidar beam with wall
      x_intersection = -intercept / slope;

      if (x_intersection >= 0) {
        if (x_intersection < scan_value) {
          scan_value = x_intersection;
        }
      }


      slope = (wall_bottom_left_lidar.y - wall_top_left_lidar.y) / (wall_bottom_left_lidar.x - wall_top_left_lidar.x);
      intercept = wall_bottom_left_lidar.y - slope * wall_bottom_left_lidar.x;

      // intersection of lidar beam with wall
      x_intersection = -intercept / slope;

      if (x_intersection >= 0) {
        if (x_intersection < scan_value) {
          scan_value = x_intersection;
        }
      }


      slope = (wall_bottom_left_lidar.y - wall_bottom_right_lidar.y) / (wall_bottom_left_lidar.x - wall_bottom_right_lidar.x);
      intercept = wall_bottom_left_lidar.y - slope * wall_bottom_left_lidar.x;

      // intersection of lidar beam with wall
      x_intersection = -intercept / slope;

      if (x_intersection >= 0) {
        if (x_intersection < scan_value) {
          scan_value = x_intersection;
        }
      }

      for (int j = 0; j < int(obstacles_x_.size()); j++) {
        turtlelib::Point2D obstacle;
        obstacle.x = obstacles_x_.at(j);
        obstacle.y = obstacles_y_.at(j);

        turtlelib::Point2D obstacle_lidar = T_laserw(obstacle);

        double obstacle_radius = obstacles_r_.at(j);

        double x_intersection_1 = std::sqrt(
          std::pow(obstacle_radius, 2) - std::pow(-obstacle_lidar.y, 2)) + obstacle_lidar.x;

        double x_intersection_2 = -std::sqrt(
          std::pow(obstacle_radius, 2) - std::pow(-obstacle_lidar.y, 2)) + obstacle_lidar.x;

        if(x_intersection_1 >= 0.0){
          if (x_intersection_1 < scan_value) {
            scan_value = x_intersection_1;
          }
        }

        if(x_intersection_2 >= 0.0){
          if (x_intersection_2 < scan_value) {
            scan_value = x_intersection_2;
          }
        }
      }

      // add noise
      std::normal_distribution<double> noise_gauss_{0.0, noise_level_};
      scan_value += noise_gauss_(generator_);

      if (scan_value > max_range_lidar_) {
        scan_value = max_range_lidar_;
      
      } else if (scan_value < min_range_) {
        scan_value = min_range_;
      }

      laser_scan_.ranges.push_back(scan_value);

    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr service_teleport_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped t_;
  nav_msgs::msg::Path msg_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr red_path_pub_;
  size_t count_;
  double rate_;
  int timestep_;
  double x0_;
  double y0_;
  double theta0_;
  double arena_x_length_;
  double arena_y_length_;
  double wheel_radius_;
  double track_width_;
  double motor_cmd_speed_;
  double motor_cmd_per_rad_sec_;
  double encoder_ticks_per_rad_;
  double input_noise_;
  double slip_fraction_;
  double basic_sensor_variance_;
  double max_range_;
  double collision_radius_;
  double min_range_;
  double max_range_lidar_;
  double angle_increment_;
  int number_of_samples_;
  double resolution_;
  double noise_level_;
  double wall_height;
  double obstacles_height;
  std::vector<double> obstacles_x_;
  std::vector<double> obstacles_y_;
  std::vector<double> obstacles_r_;
  double wheel_velocity_left_;
  double wheel_velocity_right_;
  nuturtlebot_msgs::msg::SensorData sensor_data_;
  sensor_msgs::msg::JointState joint_state_;
  turtlelib::DiffDrive diff_drive_;
  turtlelib::Configuration_q q_diff_;
  turtlelib::WheelPositions_phi wheel_positions_;
  // std::normal_distribution<double> noise_gauss_{0.0, input_noise_};  // do not think this is right
  // std::uniform_real_distribution<double> noise_uniform_{-slip_fraction_, slip_fraction_};
  std::default_random_engine generator_;

  sensor_msgs::msg::LaserScan laser_scan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
