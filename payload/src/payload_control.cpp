#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>

using namespace std::chrono_literals; // enable the usage of 1s, 1ms, 1us, etc

class PayloadControl : public rclcpp::Node
{
public:
  PayloadControl()
  : Node("payload_control")
  {
    // Subscribers
    

    // Timers
    timer_ =
      this->create_wall_timer(0.01s, std::bind(&PayloadControl::timer_callback, this));
  }

private:

  void timer_callback()
  {
    // log a simple message for testing
    RCLCPP_INFO(this->get_logger(), "Hello, world!");
  }


  // Declare timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Declare subscribers




};





int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadControl>());
  rclcpp::shutdown();
  return 0;
}