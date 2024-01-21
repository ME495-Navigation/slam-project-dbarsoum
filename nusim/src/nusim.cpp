#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
// #include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
// #include "turtlesim/msg/pose.hpp"
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

            /// gets the value of the parameter
            rate_ = this->get_parameter("rate").as_double();

            /// Initialize the transform broadcaster
            tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            /// creates a publisher
            publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

            /// creates a subscriber
            // subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            //     "turtle/pose", 10,
            //     std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));

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

            // time_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            publisher_->publish(message);

            // Publish the transformation
            t_.header.stamp = this->get_clock()->now();
            t_.header.frame_id = "nusim/world";
            t_.child_frame_id = "red/base_footprint";

            tf_broadcaster_->sendTransform(t_);


        }

        void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            timestep_ = 0;
            RCLCPP_INFO(this->get_logger(), "Resetting timestep to 0.");
        }

        void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                               std::shared_ptr<nusim::srv::Teleport::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Teleporting turtle to (%f, %f, %f).", request->x, request->y, request->theta);
            // geometry_msgs::msg::TransformStamped t_;

            // Read message content and assign it to
            // corresponding tf variables
            t_.header.stamp = this->get_clock()->now();
            t_.header.frame_id = "nusim/world";
            t_.child_frame_id = "red/base_footprint";

            // Turtle only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t_.transform.translation.x = request->x;
            t_.transform.translation.y = request->y;
            t_.transform.translation.z = 0.0;

            // For the same reason, turtle can only rotate around one axis
            // and this why we set rotation in x and y to 0 and obtain
            // rotation in z axis from the message
            tf2::Quaternion q;
            q.setRPY(0, 0, request->theta);
            t_.transform.rotation.x = q.x();
            t_.transform.rotation.y = q.y();
            t_.transform.rotation.z = q.z();
            t_.transform.rotation.w = q.w();

            // Send the transformation
            tf_broadcaster_->sendTransform(t_);
        }

        // void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
        // {
        //     geometry_msgs::msg::TransformStamped t;

        //     // Read message content and assign it to
        //     // corresponding tf variables
        //     t.header.stamp = this->get_clock()->now();
        //     t.header.frame_id = "nusim/world";
        //     t.child_frame_id = "red/base_footprint";

        //     // Turtle only exists in 2D, thus we get x and y translation
        //     // coordinates from the message and set the z coordinate to 0
        //     t.transform.translation.x = msg->x;
        //     t.transform.translation.y = msg->y;
        //     t.transform.translation.z = 0.0;

        //     // For the same reason, turtle can only rotate around one axis
        //     // and this why we set rotation in x and y to 0 and obtain
        //     // rotation in z axis from the message
        //     tf2::Quaternion q;
        //     q.setRPY(0, 0, msg->theta);
        //     t.transform.rotation.x = q.x();
        //     t.transform.rotation.y = q.y();
        //     t.transform.rotation.z = q.z();
        //     t.transform.rotation.w = q.w();

        //     // Send the transformation
        //     tf_broadcaster_->sendTransform(t);
        // }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        // rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
        rclcpp::Service<nusim::srv::Teleport>::SharedPtr service_teleport_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        geometry_msgs::msg::TransformStamped t_;
        size_t count_;
        double rate_;
        int timestep_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}