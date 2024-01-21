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


using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
    public:
        Nusim()
        : Node("nusim")
        {
            RCLCPP_INFO(this->get_logger(), "Nusim has been started.");
            /// declares a parameter
            this->declare_parameter("rate", 200.0);
            this->declare_parameter("x0", 0.0);
            this->declare_parameter("y0", 0.0);
            this->declare_parameter("theta0", 0.0);

            /// gets the value of the parameter
            rate_ = this->get_parameter("rate").as_double();
            x0_ = this->get_parameter("x0").as_double();
            y0_ = this->get_parameter("y0").as_double();
            theta0_ = this->get_parameter("theta0").as_double();

            RCLCPP_INFO(this->get_logger(), "Teleporting turtle to (%f, %f, %f).", x0_ , y0_, theta0_);

            /// Initialize the transform broadcaster
            tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            /// creates a publisher
            publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

            /// services
            service_ = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
            service_teleport_ = this->create_service<nusim::srv::Teleport>("~/teleport", std::bind(&Nusim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2));
            
            /// timers
            timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/rate_), std::bind(&Nusim::timer_callback, this));
        }
    private:
        void timer_callback()
        {
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
        }

        void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                            std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
            timestep_ = 0;
            x0_ = 0.0;
            y0_ = 0.0;
            theta0_ = 0.0;

            RCLCPP_INFO(this->get_logger(), "Resetting timestep to 0.");
        }

        void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                               std::shared_ptr<nusim::srv::Teleport::Response>)
        {
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

            // Turtle only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t_.transform.translation.x = x0_;
            t_.transform.translation.y = y0_;

            // For the same reason, turtle can only rotate around one axis
            // and this why we set rotation in x and y to 0 and obtain
            // rotation in z axis from the message
            tf2::Quaternion q;
            q.setRPY(0, 0, theta0_);
            t_.transform.rotation.x = q.x();
            t_.transform.rotation.y = q.y();
            t_.transform.rotation.z = q.z();
            t_.transform.rotation.w = q.w();

            // Send the transformation
            tf_broadcaster_->sendTransform(t_);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}