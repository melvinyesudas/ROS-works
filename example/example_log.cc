
#include <cstdint>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

/**
 * This example shows how to use ROS2 logging functions:
 * http://docs.ros2.org/dashing/api/rclcpp/logging_8hpp.html
 */
class ExampleLog : public rclcpp::Node
{
public:
    explicit ExampleLog(const rclcpp::NodeOptions &options) : Node("log", options)
    {
        // Every module reports when starts
        RCLCPP_INFO(get_logger(), "Starting %s.", get_name());

        // Integers
        int example_int = 42;
        RCLCPP_INFO(get_logger(), "example_int: %d", example_int);

        // Floats
        double example_float = 23.0;
        RCLCPP_INFO(get_logger(), "example_float: %f", example_float);

        // Strings
        std::string example_string = "American Haval";
        RCLCPP_INFO(get_logger(), "example_string: %s", example_string.c_str());

        // Log Levels
        // Set in the launch file via __log_level:=debug
        RCLCPP_DEBUG(get_logger(), " >>> DEBUG <<< ");
        RCLCPP_INFO(get_logger(), " >>> INFO <<< ");
        RCLCPP_WARN(get_logger(), " >>> WARN <<< ");
        RCLCPP_ERROR(get_logger(), " >>> ERROR <<< ");
        RCLCPP_FATAL(get_logger(), " >>> FATAL <<< ");

        // Conditional Logging (works for all levels)
        RCLCPP_INFO_EXPRESSION(get_logger(), example_int > 32, " conditional expression (true) ");
        RCLCPP_INFO_EXPRESSION(
            get_logger(), example_int < 32,
            " conditional expression (false) "); // You'll never see this in the log

        // CSV Example
        // You can use grep to filter only these lines, and copy and paste into a spreadsheet:
        // ./node.sh start | grep '###'
        for (int i = 0; i < 50; ++i)
        {
            RCLCPP_INFO(get_logger(), "###,%d,%d,%d,%d", i, i * 2, i % 13, i * i);
        }

        // flushes values from the buffer immediately
        std::flush(std::cout);
    }

private:
    // Prevents this Node from being copied
    RCLCPP_DISABLE_COPY(ExampleLog)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleLog)
