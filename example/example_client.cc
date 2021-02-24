
#include "example_msgs/srv/test_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

using std::chrono::seconds;
using std::placeholders::_1;

// Use `using` to use the short message name the same as any c++ class
using example_msgs::srv::TestService;

/**
 * This example class is used to show how to establish and publish data to a ros
 * 2 topic.
 */
class ExampleClient : public rclcpp::Node
{
public:
  explicit ExampleClient(const rclcpp::NodeOptions &options) : Node("client", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting %s.", this->get_name());

    // flushes values from the buffer to the active log.
    std::flush(std::cout);

    // setup a timer se we may repeatedly call the service
    auto callback = std::bind(&ExampleClient::TimerCallback, this);
    timer_ = this->create_wall_timer(seconds(1), callback);

    // create the client for calling the service
    client_ = this->create_client<TestService>("/example/srv/test");
  }

private:
  void ResponseCallback(rclcpp::Client<TestService>::SharedFuture future)
  {
    // here we get the result from the future
    auto response = future.get();
    auto result = response->res;

    RCLCPP_INFO(this->get_logger(), "Response: %s.", result);
    std::flush(std::cout);
  }

  void TimerCallback()
  {
    // wait for the service to be ready
    if (client_->service_is_ready())
    {
      auto request = std::make_shared<TestService::Request>();
      request->req = "Test Request";

      // create a callback to recieve the response from the test
      auto callback = std::bind(&ExampleClient::ResponseCallback, this, _1);
      client_->async_send_request(request, callback);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<TestService>::SharedPtr client_;

  // Prevents this Node from being copied
  RCLCPP_DISABLE_COPY(ExampleClient)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleClient)
