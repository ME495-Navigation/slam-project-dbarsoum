#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
    public:
        Nusim()
        : Node("nusim")
        {
            RCLCPP_INFO(this->get_logger(), "Nusim has been started.");
            /// declares a parameter
            this->declare_parameter("frequency", 200.0);

            /// gets the value of the parameter
            double frequency = this->get_parameter("frequency").as_double();

            /// creates a publisher
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

            /// creates a timer using the parameter
            /// need to use auto to init variable when declaring it
            auto fre_ms = std::chrono::duration<double>(1.0/frequency);
            timer_ = this->create_wall_timer(
            fre_ms, std::bind(&Nusim::timer_callback, this));
        }
    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hello, world! " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}