
#include "rclcpp/rclcpp.hpp"
#include "status.hh"
#include <cstdint>
#include <iostream>

/**
 * This Example Class is used to show how to read parameters stored in the
 * params.yaml.
 */
class ExampleStatus : public rclcpp::Node
{
public:
  explicit ExampleStatus(const rclcpp::NodeOptions &options)
      : Node("status", options),
        /* We initialize the status with this node and an status invertal of 500 milliseconds.  */
        status_(this, 500)
  {
    RCLCPP_INFO(this->get_logger(), "Starting %s.", this->get_name());

    //////////////////////////////////////////////////////////////
    // README
    //////////////////////////////////////////////////////////////
    //
    // Note, that the module status will only be sent at the
    // interval designated in the constructor, changing it in
    // quick succession as in this example will most likely
    // cause only the message set last to be sent during the
    // interval.
    //
    // However this is not guaranteed and should not be relied
    // upon. The status message should only be set if the actual
    // status of the module has changed.
    //
    //////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////
    // Sets the status message to display a NOMINAL message.
    status_.Nominal("This is an example of a Normal Message");

    // Check if we have a nominal message.
    if (status_.IsNominal())
    {
      RCLCPP_INFO(this->get_logger(), "Nominal Message OK");
    }

    //////////////////////////////////////////////////////////////
    // Sets the status message to display a WARNING message.
    status_.Warning("This is an example of a Warning Message");

    // Check if we have a warning message.
    if (status_.IsWarning())
    {
      RCLCPP_INFO(this->get_logger(), "Warning Message OK");
    }

    //////////////////////////////////////////////////////////////
    // Sets the status message to display a FATAL message.
    status_.Fatal("This is an example of a Fatal Message");

    // Check if we have a fatal message.
    if (status_.IsFatal())
    {
      RCLCPP_INFO(this->get_logger(), "Fatal Message OK");
    }

    // This is a convience method to determine if we have a non-fatal state.
    if (status_.IsNotFatal())
    {
      RCLCPP_INFO(this->get_logger(), "You should not see this message.");
    }

    // flushes values from the buffer to the active log.
    std::flush(std::cout);
  }

private:
  Status status_;

  // Prevents this Node from being copied
  RCLCPP_DISABLE_COPY(ExampleStatus)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleStatus)
