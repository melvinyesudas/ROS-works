
#include <cstdint>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

/**
 * This Example Class is used to show how to read parameters stored in the
 * params.yaml.
 */
class ExampleConfig : public rclcpp::Node
{
public:
    explicit ExampleConfig(const rclcpp::NodeOptions &options)
        // The node name `config` is used in the params.yaml file.
        : Node("config", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting %s.", this->get_name());

        // Parameters

        // example int (meters)
        // This is a simple example of an integer parameter
        int example_int = declare_parameter("example_int", 42);

        // example float (hz)
        // This is a simple example of an float parameter
        double example_float = declare_parameter("example_float", 23.0);

        // example string
        // This is a simple example of an string parameter
        std::string example_string =
            declare_parameter("example_string", std::string("Default Value"));

        // Log the parameter values as an example
        RCLCPP_INFO(this->get_logger(), "example_int: %d", example_int);
        RCLCPP_INFO(this->get_logger(), "example_float: %f", example_float);
        RCLCPP_INFO(this->get_logger(), "example_string: %s", example_string.c_str());

        // flushes values from the buffer to the active log.
        std::flush(std::cout);
    }

private:
    // Prevents this Node from being copied
    RCLCPP_DISABLE_COPY(ExampleConfig)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleConfig)
