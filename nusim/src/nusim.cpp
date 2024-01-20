#include <chrono>
#include <functional>
#include <memory>
#include <string>
// #include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"


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

            /// creates a publisher
            publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

            /// create an empty service
            service_ = this->create_service<std_srvs::srv::Empty>("~/reset", std::bind(&Nusim::reset_callback, this, std::placeholders::_1, std::placeholders::_2));

            /// need to use auto to init variable when declaring it
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

        }

        void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response)
        {
            timestep_ = 0;
            RCLCPP_INFO(this->get_logger(), "Resetting timestep to 0.");
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
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