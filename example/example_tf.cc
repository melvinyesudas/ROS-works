
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

class ExampleTF : public rclcpp::Node
{
public:
    explicit ExampleTF(const rclcpp::NodeOptions &options) : Node("example_tf_node", options)
    {
        RCLCPP_INFO(get_logger(), "Starting %s.", this->get_name());
        std::flush(std::cout);

        // Parameters
        parent_frame_ = declare_parameter("parent_frame_", std::string("parent"));
        child_frame_ = declare_parameter("child_frame_", std::string("child"));

        // TF Buffer & Listener
        tfBuffer_ = new tf2_ros::Buffer(get_clock());
        tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);

        // create a timer to poll for a valid TF
        tf_timer_ = this->create_wall_timer(std::chrono::seconds(3),
                                            std::bind(&ExampleTF::static_tf_timer__callback, this));
    }

    /**
     * @brief Retrieve the latest transformation between specified frames
     */
    void static_tf_timer__callback()
    {
        //
        // Get the latest transform example
        // Replace tf2::TimePointZero to get a transform at a particular time point
        //
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            auto timeout = std::chrono::milliseconds(500);
            transformStamped = tfBuffer_->lookupTransform(parent_frame_, child_frame_,
                                                          tf2::TimePointZero, timeout);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "TF Lookup Failed: %s", ex.what());
            std::flush(std::cout);
            return;
        }

        auto T = transformStamped.transform.translation;
        auto Q = transformStamped.transform.rotation;
        RCLCPP_INFO(get_logger(),
                    "TF obtained from %s to %s:\nX:%05.2f Y:%05.2f Z:%05.2f\nX:%05.2f Y:%05.2f "
                    "Z:%05.2f W:%05.2f",
                    parent_frame_.c_str(), child_frame_.c_str(), T.x, T.y, T.z, Q.x, Q.y, Q.z, Q.w);
        std::flush(std::cout);

        //
        // Point Transform Example (Eigen)
        //
        {
            Eigen::Vector3d in(0, 1, 0);
            Eigen::Vector3d out;
            tf2::doTransform(in, out, transformStamped);

            RCLCPP_INFO(get_logger(), "Point(Eigen): X:%05.2f Y:%05.2f Z:%05.2f", out.x(), out.y(),
                        out.z());
            std::flush(std::cout);
        }

        //
        // Pose Transform Example
        //
        {
            geometry_msgs::msg::PoseStamped ps;
            geometry_msgs::msg::PoseStamped pt;
            ps.pose.position.x = 0;
            ps.pose.position.y = 0;
            ps.pose.position.z = 1;
            ps.pose.orientation.x = 0;
            ps.pose.orientation.y = 0;
            ps.pose.orientation.z = 0;
            ps.pose.orientation.w = 1;
            tf2::doTransform(ps, pt, transformStamped);

            RCLCPP_INFO(get_logger(),
                        "PoseStamped [%s]:\nX:%05.2f Y:%05.2f Z:%05.2f\nX:%05.2f Y:%05.2f "
                        "Z:%05.2f W:%05.2f",
                        pt.header.frame_id.c_str(), pt.pose.position.x, pt.pose.position.y,
                        pt.pose.position.z, pt.pose.orientation.x, pt.pose.orientation.y,
                        pt.pose.orientation.z, pt.pose.orientation.w);
            std::flush(std::cout);
        }

        // There are dozens of doTransform implementations for various data types and you can
        // implement your own for custom data types as well. Here's a good place to look:
        // https://github.com/ros2/geometry2/search?q=void+doTransform%28
    }

private:
    /// example parent frame
    std::string parent_frame_;

    /// example child frame
    std::string child_frame_;

    // create a buffer for lookup
    tf2_ros::Buffer *tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    // Timers
    rclcpp::TimerBase::SharedPtr tf_timer_;
};
// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleTF)
