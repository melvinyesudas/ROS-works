

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

using std::chrono::milliseconds;

/**
 * This example class is used to demonstrate how to get the system time
 */
class ExampleTime : public rclcpp::Node
{
public:
  explicit ExampleTime(const rclcpp::NodeOptions &options)
      : Node("ExampleTime", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting %s.", this->get_name());

    // Binds Timer method in this instance.
    auto callback = std::bind(&ExampleTime::Timer, this);

    // Creates a timer with a delay of X milliseconds with the callback.
    timer_ = this->create_wall_timer(milliseconds(500), callback);
  }

private:
  // The timer callback function, we use a timer to provide continuous test data.
  void Timer()
  {
    rclcpp::Time time = this->now();
    RCLCPP_INFO(this->get_logger(), "Time: %f", time.seconds());
    std::flush(std::cout);
  }

  // Class Variables
  rclcpp::TimerBase::SharedPtr timer_;

  // Prevents this Node from being copied
  RCLCPP_DISABLE_COPY(ExampleTime)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleTime)
