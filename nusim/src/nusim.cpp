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
// #include <cmath>
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
    declare_parameter("arena_y_length", 10.0);
    declare_parameter("obstacles/x", std::vector<double>{});
    declare_parameter("obstacles/y", std::vector<double>{});
    declare_parameter("obstacles/r", std::vector<double>{});
    declare_parameter("input_noise", 5.0);
    declare_parameter("slip_fraction", 0.5);

    /// gets the value of the parameter
    rate_ = get_parameter("rate").as_double();
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    theta0_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    input_noise_ = get_parameter("input_noise").as_double();
    slip_fraction_ = get_parameter("slip_fraction").as_double();

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
    RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "track width: " << track_width_);
    RCLCPP_INFO_STREAM(get_logger(), "motor cmd speed: " << motor_cmd_speed_);
    RCLCPP_INFO_STREAM(get_logger(), "motor cmd per rad sec: " << motor_cmd_per_rad_sec_);
    RCLCPP_INFO_STREAM(get_logger(), "encoder ticks per rad: " << encoder_ticks_per_rad_);


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
      "red/sensor_data", 10); // sensor data gets remapped in launch file

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
    visualization_msgs::msg::MarkerArray marker_array;
    addWall(marker_array, arena_x_length_ / 2, 0.0, "top_wall", 1);
    addWall(marker_array, 0.0, arena_y_length_ / 2, "right_wall", 2);
    addWall(marker_array, -arena_x_length_ / 2, 0.0, "bottom_wall", 3);
    addWall(marker_array, 0.0, -arena_y_length_ / 2, "left_wall", 4);
    marker_pub_->publish(marker_array);

    // Publish the obstacles
    visualization_msgs::msg::MarkerArray obstacle_array;
    for (int i = 0; i < int(obstacles_x_.size()); i++) {
      addObstacle(obstacle_array, obstacles_x_[i], obstacles_y_[i], obstacles_r_[i], "obstacle", i);
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
    // RCLCPP_INFO(
    //   get_logger(), "Teleporting turtle to (%f, %f, %f).", request->x, request->y,
    //   request->theta);
    // geometry_msgs::msg::TransformStamped t_;
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
  
    // update transformation
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
    marker.pose.position.z = 0.0;
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
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = r;
    marker.scale.y = r;
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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
