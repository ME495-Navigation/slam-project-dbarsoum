#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

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


using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim")
  {
    // RCLCPP_INFO(this->get_logger(), "Nusim has been started.");

    /// declares a parameter
    this->declare_parameter("rate", 200.0);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);
    this->declare_parameter("arena_x_length", 5.0);
    this->declare_parameter("arena_y_length", 10.0);
    this->declare_parameter("obstacles/x", std::vector<double>{});
    this->declare_parameter("obstacles/y", std::vector<double>{});
    this->declare_parameter("obstacles/r", std::vector<double>{});

    /// gets the value of the parameter
    rate_ = this->get_parameter("rate").as_double();
    x0_ = this->get_parameter("x0").as_double();
    y0_ = this->get_parameter("y0").as_double();
    theta0_ = this->get_parameter("theta0").as_double();
    arena_x_length_ = this->get_parameter("arena_x_length").as_double();
    arena_y_length_ = this->get_parameter("arena_y_length").as_double();

    /// check to make sure that the obstacles are the same length
    obstacles_x_ = this->get_parameter("obstacles/x").as_double_array();
    obstacles_y_ = this->get_parameter("obstacles/y").as_double_array();
    obstacles_r_ = this->get_parameter("obstacles/r").as_double_array();
    if (obstacles_x_.size() != obstacles_y_.size() || obstacles_x_.size() != obstacles_r_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Obstacles are not the same length.");
      rclcpp::shutdown();
    }

    declare_parameter("wheel_radius", 0.0);
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    // RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius_);
    if (wheel_radius_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "wheel_radius error");
      rclcpp::shutdown();
    }

    declare_parameter("track_width", 0.0);
    track_width_ = get_parameter("track_width").as_double();
    if (track_width_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "track_width error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_max", 0.0);
    motor_cmd_speed_ = get_parameter("motor_cmd_max").as_double();
    if (motor_cmd_speed_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_speed error");
      rclcpp::shutdown();
    }

    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    if (motor_cmd_per_rad_sec_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "motor_cmd_per_rad_sec error");
      rclcpp::shutdown();
    }

    declare_parameter("encoder_ticks_per_rad", 0.0);
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").as_double();
    if (encoder_ticks_per_rad_ == 0.0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "encoder_ticks_per_rad error");
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
    timestep_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_policy);
    obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      qos_policy);

    sensor_data_pub_ = this->create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data", 10); // sensor data gets remapped in launch file

    /// subscribers
    wheel_cmd_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));
    /// wheel_cmd gets remapped in launch file

    /// services
    service_ =
      this->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_teleport_ =
      this->create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    diff_drive_ = turtlelib::DiffDrive(track_width_, wheel_radius_);
    /// timers
    timer_ = this->create_wall_timer(
      (1s / rate_), std::bind(&Nusim::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    /// need the timestep in ms
    message.data = timestep_++ *(1 / rate_) * 1e3;
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
    timestep_publisher_->publish(message);

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

    publish_sensor_data();

    tf_broadcaster_->sendTransform(t_);

  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    x0_ = 0.0;
    y0_ = 0.0;
    theta0_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Resetting timestep to 0.");
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    RCLCPP_INFO(
      this->get_logger(), "Teleporting turtle to (%f, %f, %f).", request->x, request->y,
      request->theta);
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
    /// need to convert the wheel commands (mcu) to rad/s
    wheel_velocity_left_ = msg->left_velocity * motor_cmd_per_rad_sec_;  // need rad/s so need to multiply by motor_cmd_per_rad_sec_
    wheel_velocity_right_ = msg->right_velocity * motor_cmd_per_rad_sec_;
    // RCLCPP_INFO_STREAM(this->get_logger(), "wheel_cmd_callback: " << wheel_velocity_left_ << " " << wheel_velocity_right_);

    diff_drive_.update_configuration(wheel_positions_);  // fk
    q_diff_ = diff_drive_.get_configuration();
    wheel_positions_ = diff_drive_.get_wheel_positions();

    RCLCPP_INFO_STREAM(this->get_logger(), "LOOK HERE: " << wheel_velocity_left_*(1/rate_) << " AND HERE: " << wheel_velocity_right_*(1/rate_));
    wheel_positions_.phi_left = wheel_positions_.phi_left + wheel_velocity_left_ * (1 / rate_);  // rad
    wheel_positions_.phi_right = wheel_positions_.phi_right + wheel_velocity_right_ * (1 / rate_);

    // update transformation
    updateTransform(q_diff_.x_, q_diff_.y_, q_diff_.theta_);
  }

  void publish_sensor_data()
  {

    RCLCPP_INFO_STREAM(this->get_logger(), "publish_sensor_data");
    sensor_data_.left_encoder += (wheel_velocity_left_) * (1 / rate_) * encoder_ticks_per_rad_;
    sensor_data_.right_encoder += wheel_velocity_right_ * (1 / rate_) * encoder_ticks_per_rad_;
    RCLCPP_INFO_STREAM(this->get_logger(), "left_encoder: " << sensor_data_.left_encoder << " right_encoder: " << sensor_data_.right_encoder);
    sensor_data_pub_->publish(sensor_data_);
  }

  void addWall(
    visualization_msgs::msg::MarkerArray & marker_array, double x, double y,
    const std::string & name, int id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = this->get_clock()->now();
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
    marker.header.stamp = this->get_clock()->now();
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

  void updateTransform(double x, double y, double theta)
  {
    t_.header.stamp = this->get_clock()->now();
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
