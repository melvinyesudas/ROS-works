
#pragma once

// Use the proper guard format to prevent cpplint warnings
// Generally '<Module Name>__<FILENAME>_'
#ifndef EXAMPLE__STATUS_HH_
#define EXAMPLE__STATUS_HH_

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

using diagnostic_msgs::msg::DiagnosticStatus;
using std::chrono::milliseconds;
using std::chrono::seconds;

/**
 * The status class is used to automatically report the module
 * status to the rest of the modules. This file should not be modified.
 */
class Status
{
public:
  /**
   * Explicit constructor with a default 500 millisecond update time.
   * @param node Node to send message for.
   */
  explicit Status(rclcpp::Node *node) : Status(node, 500) {}

  /**
   * Standard constructor
   * @param node Node to send message for.
   * @param interval Interval (milliseconds) in which to send status message.
   */
  Status(rclcpp::Node *node, int interval)
  {
    // Sets up the status inital state, including the name of the
    DiagnosticStatus status;
    status_.name = node->get_name();
    status_.hardware_id = "";
    status_.message = "Starting.";
    status_.level = DiagnosticStatus::WARN;

    timer_ = node->create_wall_timer(milliseconds(interval), std::bind(&Status::Timer, this));
    diag_pub_ = node->create_publisher<DiagnosticStatus>("/status", 10);
  }
  /**
   * Used to report a non auto recoverable error has occured in the module.
   * While this can be overwritten with a non-fatal state, that is not it's intended use.
   * @param msg the message to report
   */
  inline void Fatal(std::string msg)
  {
    status_.level = DiagnosticStatus::ERROR;
    status_.message = msg;
  }
  /**
   * Used to report normal operation.
   * @param msg the message to report
   */
  inline void Nominal(std::string msg)
  {
    status_.level = DiagnosticStatus::OK;
    status_.message = msg;
  }
  /**
   * Used to report a temporary or auto recoverable issue in the system, such as being unable
   * to proceed until more information is provided.
   * @param msg the message to report
   */
  inline void Warning(std::string msg)
  {
    status_.level = DiagnosticStatus::WARN;
    status_.message = msg;
  }
  /**
   * @return true if this status is reporting a fatal state, false otherwise.
   */
  inline bool IsFatal() { return status_.level == DiagnosticStatus::ERROR; }
  /**
   * @return true if this status is reporting a normal state, false otherwise.
   */
  inline bool IsNominal() { return status_.level == DiagnosticStatus::OK; }
  /**
   * @return true if this status is reporting a warning state, false otherwise.
   */
  inline bool IsWarning() { return status_.level == DiagnosticStatus::WARN; }
  /**
   * @return true if this status is reporting a non-fatal state, false otherwise.
   */
  inline bool IsNotFatal() { return !IsFatal(); }

private:
  inline void Timer()
  {
    diag_pub_->publish(status_);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr diag_pub_;
  DiagnosticStatus status_;
};

#endif // EXAMPLE__STATUS_HH_
