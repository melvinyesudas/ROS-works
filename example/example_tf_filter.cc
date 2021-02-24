

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

class ExampleTF_filter : public rclcpp::Node
{
public:
    explicit ExampleTF_filter(const rclcpp::NodeOptions &options)
        : Node("example_tf_filter_node", options)
    {
        RCLCPP_INFO(get_logger(), "Starting %s.", get_name());
        std::flush(std::cout);

        // Parameters
        parent_frame_ = declare_parameter("parent_frame_", std::string("parent"));
        child_frame_ = declare_parameter("child_frame_", std::string("child"));

        // Example point data publisher
        pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/example_points", 10);

        // TF Buffer & Listener
        tfBuffer_ = new tf2_ros::Buffer(get_clock());
        tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);

        // Example point data subscriber
        sub_.subscribe(this, "/example_points");
        filter_ = new tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>(
            sub_, *tfBuffer_, parent_frame_, 10, get_node_logging_interface(),
            get_node_clock_interface());
        filter_->registerCallback(&ExampleTF_filter::sub_callback, this);

        // Timers
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&ExampleTF_filter::timer_callback, this));
    }

    /// Example Point Data Publisher
    void timer_callback()
    {
        // Simple loop from -1.0 ~ 0.1
        static double last = 0.0;
        last += 0.1;
        if (last > 1.0)
            last = -1.0;

        geometry_msgs::msg::PointStamped p;
        p.header.stamp = now();
        p.header.frame_id = child_frame_;
        p.point.x = last;
        p.point.y = 0;
        p.point.z = 0;
        pub_->publish(p);
    }

    /// Example Point Data Filter Callback
    void sub_callback(const geometry_msgs::msg::PointStamped &msg)
    {
        // Transform the point data
        geometry_msgs::msg::PointStamped out;
        out = tfBuffer_->transform(msg, parent_frame_);

        RCLCPP_INFO(get_logger(), "PointStamped [%s]:\nX:%05.2f Y:%05.2f Z:%05.2f",
                    out.header.frame_id.c_str(), out.point.x, out.point.y, out.point.z);
        std::flush(std::cout);
    }

private:
    /// Parameters
    std::string parent_frame_;
    std::string child_frame_;

    /// Example point data Publisher
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;

    // create a buffer for lookup
    tf2_ros::Buffer *tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    /// Example point data Subscriber
    message_filters::Subscriber<geometry_msgs::msg::PointStamped> sub_;
    tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> *filter_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
};

// Used to register this node as a ROS2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ExampleTF_filter)
