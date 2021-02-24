
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "example_msgs/msg/example_data.hpp"
#include "rclcpp/rclcpp.hpp"

using std::chrono::milliseconds;

/**
 * This example class is used to show how to establish and publish data to a ros
 * 2 topic.
 */
class ExamplePub : public rclcpp::Node
{
public:
    explicit ExamplePub(const rclcpp::NodeOptions &options) : Node("pub", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting %s.", this->get_name());

        // Creates the publisher, to topic "example" with a history depth of 10
        publisher_ = this->create_publisher<example_msgs::msg::ExampleData>("example", 10);

        // Binds Timer method in this instance.
        auto callback = std::bind(&ExamplePub::Timer, this);

        // Creates a timer with a delay of 50 milliseconds with the callback.
        timer_ = this->create_wall_timer(milliseconds(500), callback);

        counter_ = 0;
    }

private:
    // The timer callback function, we use a timer to provide continuous test data.
    void Timer()
    {
        // Create a example data mesage
        auto msg = example_msgs::msg::ExampleData();

        // Assign sample data to the message
        ++counter_;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "none";
        msg.number_a = counter_ * 0xF;
        msg.number_b = counter_;

        // Publishes the message to the topic
        publisher_->publish(msg);

        // Prints the message to the log and flushes the data for immediate display.
        RCLCPP_INFO(get_logger(), "Sent MSG %x %x", msg.number_a, msg.number_b);
        std::flush(std::cout);
    }

    // Class Variables
    uint32_t counter_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_msgs::msg::ExampleData>::SharedPtr publisher_;

    // Prevents this Node from being copied
    RCLCPP_DISABLE_COPY(ExamplePub)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExamplePub)
