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

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node{
    public:
        Nusim()
        : Node("nusim"){
            RCLCPP_INFO(this->get_logger(), "Nusim has been started.");
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

            RCLCPP_INFO(this->get_logger(), "Teleporting turtle to (%f, %f, %f).", x0_ , y0_, theta0_);

            /// Initialize the transform broadcaster
            tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            /// creates a publisher
            publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
            rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_policy);
            obstacle_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos_policy);

            /// services
            service_ = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
            service_teleport_ = this->create_service<nusim::srv::Teleport>("~/teleport", std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
            
            /// timers
            timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/rate_), std::bind(&Nusim::timer_callback, this));
        }
    private:
        void timer_callback(){
            /// publishes hello world at given rate
            auto message = std_msgs::msg::UInt64();
            /// need the timestep in ms
            message.data = timestep_++ * (1/rate_)*1e3;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
            publisher_->publish(message);

            // Publish the transformation
            t_.header.stamp = this->get_clock()->now();
            t_.header.frame_id = "nusim/world";
            t_.child_frame_id = "red/base_footprint";

            t_.transform.translation.x = x0_;
            t_.transform.translation.y = y0_;
            tf2::Quaternion q;
            q.setRPY(0, 0, theta0_);
            t_.transform.rotation.x = q.x();
            t_.transform.rotation.y = q.y();
            t_.transform.rotation.z = q.z();
            t_.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(t_);

            // Publish the markers
            visualization_msgs::msg::MarkerArray marker_array;
            addWall(marker_array, arena_x_length_/2, 0.0, "top_wall", 1);
            addWall(marker_array, 0.0, arena_y_length_/2, "right_wall", 2);
            addWall(marker_array, -arena_x_length_/2, 0.0,"bottom_wall", 3);
            addWall(marker_array, 0.0, -arena_y_length_/2, "left_wall", 4);
            marker_pub_->publish(marker_array);

            // Publish the obstacles
            visualization_msgs::msg::MarkerArray obstacle_array;
            for (int i = 0; i < int(obstacles_x_.size()) ; i++) {
                addObstacle(obstacle_array, obstacles_x_[i], obstacles_y_[i], obstacles_r_[i], "obstacle", i);
            }
            obstacle_pub_->publish(obstacle_array);
        }

        void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>){
            timestep_ = 0;
            x0_ = 0.0;
            y0_ = 0.0;
            theta0_ = 0.0;

            RCLCPP_INFO(this->get_logger(), "Resetting timestep to 0.");
        }

        void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                               std::shared_ptr<nusim::srv::Teleport::Response>){
            RCLCPP_INFO(this->get_logger(), "Teleporting turtle to (%f, %f, %f).", request->x, request->y, request->theta);
            // geometry_msgs::msg::TransformStamped t_;
            x0_ = request->x;
            y0_ = request->y;
            theta0_ = request->theta;

            // Read message content and assign it to
            // corresponding tf variables
            t_.header.stamp = this->get_clock()->now();
            t_.header.frame_id = "nusim/world";
            t_.child_frame_id = "red/base_footprint";

            t_.transform.translation.x = x0_;
            t_.transform.translation.y = y0_;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta0_);
            t_.transform.rotation.x = q.x();
            t_.transform.rotation.y = q.y();
            t_.transform.rotation.z = q.z();
            t_.transform.rotation.w = q.w();

            // Send the transformation
            tf_broadcaster_->sendTransform(t_);
        }

        void addWall(visualization_msgs::msg::MarkerArray &marker_array, double x, double y, const std::string &name, int id){
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
            if (name == "right_wall" || name == "left_wall"){
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
                } 
            else {
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
        void addObstacle(visualization_msgs::msg::MarkerArray &marker_array, double x, double y, double r, const std::string &name, int id){
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
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub_;
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
        std::vector<double> obstacles_x_;
        std::vector<double> obstacles_y_;
        std::vector<double> obstacles_r_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}