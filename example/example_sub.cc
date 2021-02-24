
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "example_msgs/msg/example_data.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

// rclcpp (ROS Client Library for C++)
class ExampleSub : public rclcpp::Node
{
public:
    explicit ExampleSub(const rclcpp::NodeOptions &options) : Node("sub", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting %s.", this->get_name());

        // Binds the ExampleCallback method in this instance. The _1 is a
        // placeholder, which is required for the example message in the callback.
        auto callback = std::bind(&ExampleSub::ExampleCallback, this, _1);

        // Creates the subscriber to the topic "example" with a history depth of 10
        subscriber_ =
            this->create_subscription<example_msgs::msg::ExampleData>("example", 10, callback);
    }

private:
    // The example callback with a shared pointer to the message
    void ExampleCallback(const example_msgs::msg::ExampleData::SharedPtr msg)
    {
        // Prints the message to the log and flushes it.
        RCLCPP_INFO(this->get_logger(), "Received MSG %x %x", msg->number_a, msg->number_b);
        std::flush(std::cout);
    }

    // Class Variables
    rclcpp::Subscription<example_msgs::msg::ExampleData>::SharedPtr subscriber_;

    // Prevents this Node from being copied
    RCLCPP_DISABLE_COPY(ExampleSub)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleSub)
