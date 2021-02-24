#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "example_msgs/srv/test_service.hpp"
#include "rclcpp/rclcpp.hpp"

using std::chrono::milliseconds;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Use `using` to use the short message name the same as any c++ class
using example_msgs::srv::TestService;

/**
 * This example class is used to show how to establish and publish data to a ros
 * 2 topic.
 */
class ExampleService : public rclcpp::Node
{
public:
    explicit ExampleService(const rclcpp::NodeOptions &options) : Node("service", options)
    {
        RCLCPP_INFO(get_logger(), "Starting %s.", get_name());

        // flushes values from the buffer to the active log.
        std::flush(std::cout);

        // Binds the service callback
        auto callback = std::bind(&ExampleService::ServiceCallback, this, _1, _2, _3);

        // Creates the service
        service_ = create_service<TestService>("/example/srv/test", callback);
    }

private:
    void ServiceCallback(const std::shared_ptr<rmw_request_id_t> header,
                         const std::shared_ptr<TestService::Request> request,
                         const std::shared_ptr<TestService::Response> response)
    {
        (void)header; // prevents warning

        // reads the request from the
        std::string rstr = request->req;

        // prints the request string
        RCLCPP_INFO(get_logger(), "Request: %d", rstr);
        std::flush(std::cout);

        // Sets the response string, this is automatically published
        // when after the end of this function.
        response->header.stamp = now();
        response->header.frame_id = "none";
        response->res = "Echo: " + rstr;
    }

    rclcpp::Service<TestService>::SharedPtr service_;

    // Prevents this Node from being copied
    RCLCPP_DISABLE_COPY(ExampleService)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleService)
