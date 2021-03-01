
#include <math.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "trackedobj_msgs/msg/tracked_object_array.hpp"
#include "trackedobj_msgs/msg/tracks.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::chrono::milliseconds;
using std::placeholders::_1;

/*

Changes to support testing the behavior:
1) Sub to /localization/pose and then pub a patched '/vehicle_patched' frame (i.e., pub the
    relationship between "vehicle_patched" frame and "map" frame)

2) Pub a static tf between navatel and vehicle. (can be added to the launch file)

*/

class VisualizationMarker : public rclcpp::Node
{
public:
    explicit VisualizationMarker(const rclcpp::NodeOptions &options)
        : Node("visualization_marker", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting %s.", this->get_name());
        counter = 0;
        log_as_csv = false;
        auto callback = std::bind(&VisualizationMarker::MarkerCallback, this, _1);

        // Setting a subscriber
        marker_sub_ = this->create_subscription<trackedobj_msgs::msg::Tracks>("/tracking/tracks",
                                                                              10, callback);

        auto localization_callback =
            std::bind(&VisualizationMarker::localization_sub_callback, this, _1);
        localization_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/pose", 10, localization_callback);
        patched_localizationTF_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

        // Setting a publisher for the marker array
        markerArray_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracking/track_viz", 10);
    }

private:
    void MarkerCallback(const trackedobj_msgs::msg::Tracks::SharedPtr TracksListAry)
    {
        static int32_t n = 0;
        std::cout << "tracks msg received by vis, count " << n++ << "\n"
                  << std::flush;
        visualization_msgs::msg::MarkerArray markerArray;
        visualization_msgs::msg::Marker marker;
        std::string writter;
        if (!log_as_csv)
        {
            writter = "----------  Tracked Frame " + std::to_string(counter) + " ----------\n";
        }

        for (trackedobj_msgs::msg::Track &track : TracksListAry->tracks)
        {
            uint32_t marker_id_temp = track.track_id;
            // for (trackedobj_msgs::msg::TrackedObjectNoID& trackObjNoId : trackObj.track_history)
            // {
            auto &trackObjNoId = track.track_history.back();
            rclcpp::Time ros_time = this->now();
            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            marker.header.frame_id = "velodyne64"; // "velodyne32fl";
            marker.header.stamp = ros_time;

            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            marker.ns = "basic_shapes";
            marker.id = marker_id_temp;
            std::string object_class;
            std::string ros_marker_color;

            // Set the marker type.  Initially this is CUBE, and cycles between that and
            // SPHERE, ARROW, and CYLINDER
            // Set the color -- be sure to set alpha to something non-zero!
            if (trackObjNoId.obj_class == "car" || trackObjNoId.obj_class == "truck" ||
                trackObjNoId.obj_class == "trailer")
            {
                // Setting marker for Cars
                marker.type = visualization_msgs::msg::Marker::CUBE;
                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = 1.0 * trackObjNoId.size.x;
                marker.scale.y = 1.0 * trackObjNoId.size.y;
                marker.scale.z = 1.0 * trackObjNoId.size.z;
                object_class = trackObjNoId.obj_class;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.5f;
                marker.color.a = 0.3;
                marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(500));
                ros_marker_color = "purple";
            }
            else if (trackObjNoId.obj_class == "pedestrian")
            {
                // Setting marker for Pedestrian
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                // Set the scale of the marker
                marker.scale.x = 1.0 * trackObjNoId.size.x;
                marker.scale.y = 1.0 * trackObjNoId.size.y;
                marker.scale.z = 1.0 * trackObjNoId.size.z;
                object_class = "Pedestrian";
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.2;
                marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(1000));
                ros_marker_color = "red";
            }
            else
            {
                // Setting marker for unknown objects
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                // Set the scale of the marker
                marker.scale.x = 1.0 * trackObjNoId.size.x;
                marker.scale.y = 1.0 * trackObjNoId.size.y;
                marker.scale.z = 1.0 * trackObjNoId.size.z;
                object_class = trackObjNoId.obj_class;
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 0.2;
                marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(500));
                ros_marker_color = "blue";
            }
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the
            // frame/time specified in the header
            marker.pose.position.x = trackObjNoId.pose.position.x;
            marker.pose.position.y = trackObjNoId.pose.position.y;
            marker.pose.position.z = trackObjNoId.pose.position.z;
            marker.pose.orientation.x = trackObjNoId.pose.orientation.x;
            marker.pose.orientation.y = trackObjNoId.pose.orientation.y;
            marker.pose.orientation.z = trackObjNoId.pose.orientation.z;
            marker.pose.orientation.w = trackObjNoId.pose.orientation.w;

            marker.lifetime = rclcpp::Duration(std::chrono::milliseconds(500));
            markerArray.markers.emplace_back(marker);
            writter += std::to_string(marker.id) + "," + std::to_string(marker.id) + "," +
                       ros_marker_color + "," + object_class + "\n";

            // Adding overlay Text to the markers
            if (marker.action == visualization_msgs::msg::Marker::ADD)
            {
                marker.ns = "track_text";
                marker.id = marker_id_temp;
                marker.pose.position.z = 5.0 + trackObjNoId.pose.position.z;

                marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

                std::stringstream speed_string;
                auto vx = trackObjNoId.vel.linear.x;
                auto vy = trackObjNoId.vel.linear.y;
                auto vz = trackObjNoId.vel.linear.z;
                auto speed = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
                if (speed < 1.0)
                {
                    // if spd < threshold, show the text in green
                    marker.color.r = 0.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 0.5;
                }
                else
                {
                    marker.color.r = 1.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 1.0f;
                    marker.color.a = 1.0;
                }
                speed_string << std::setprecision(3) << "speed: " << speed << "\n"
                             << "vx: " << vx << "\n"
                             << "vy: " << vy << "\n"
                             << "vz: " << vz << "\n";
                marker.text = speed_string.str() + object_class + "\n " + std::to_string(marker.id);
                markerArray.markers.emplace_back(marker);
            }

            markerArray_pub_->publish(markerArray);
            // Turn off the logging to file for better performance and only turn this on
            // when debugging needed
            // if(!WriteToFile("marker_log.txt", writter)) {
            //     std::cout << "Logging visualization info to file failed." << std::endl;
            // }
            // }
        }
    }

    void localization_sub_callback(const nav_msgs::msg::Odometry::SharedPtr vehicle_pose)
    {
        static uint8_t localization_msg_cntr = 0;
        localization_msg_cntr++;

        tf2_msgs::msg::TFMessage tf_series;
        geometry_msgs::msg::TransformStamped tf_single;

        tf_single.header.set__frame_id("map");
        tf_single.header.set__stamp(vehicle_pose->header.stamp);
        tf_single.set__child_frame_id("vehicle");
        tf_single.transform.translation.x = vehicle_pose->pose.pose.position.x;
        tf_single.transform.translation.y = vehicle_pose->pose.pose.position.y;
        tf_single.transform.translation.z = vehicle_pose->pose.pose.position.z;
        Eigen::Quaternion<float_t> q_unpatched(
            vehicle_pose->pose.pose.orientation.w, vehicle_pose->pose.pose.orientation.x,
            vehicle_pose->pose.pose.orientation.y, vehicle_pose->pose.pose.orientation.z);
        auto theta_unpatched = 2 * std::asin(q_unpatched.z());
        auto theta_patched = M_PI_2 - theta_unpatched;
        tf_single.transform.rotation.set__w(std::cos(theta_patched / 2));
        tf_single.transform.rotation.set__x(vehicle_pose->pose.pose.orientation.x);
        tf_single.transform.rotation.set__y(vehicle_pose->pose.pose.orientation.y);
        tf_single.transform.rotation.set__z(std::sin(theta_patched / 2));

        if (localization_msg_cntr >= 10)
        {
            localization_msg_cntr = 0;
            std::stringstream ss;
            ss << "\nraw localization received as "
                  "\ntranslation(x,y,z): "
               << vehicle_pose->pose.pose.position.x << " " << vehicle_pose->pose.pose.position.y
               << " " << vehicle_pose->pose.pose.position.z
               << "\nquaternion(x,y,z,w): " << vehicle_pose->pose.pose.orientation.x << " "
               << vehicle_pose->pose.pose.orientation.y << " "
               << vehicle_pose->pose.pose.orientation.z << " "
               << vehicle_pose->pose.pose.orientation.w
               << " \nand equivalent heading :" << theta_unpatched / M_PI * 180 << " degree";
            RCLCPP_INFO(this->get_logger(), ss.str());

            ss.str("");
            ss << "\n---------\npatched localization is \ntranslation(x,y,z): "
               << tf_single.transform.translation.x << " " << tf_single.transform.translation.y
               << " " << tf_single.transform.translation.z
               << "\nquaternion(x,y,z,w): " << tf_single.transform.rotation.x << " "
               << tf_single.transform.rotation.y << " " << tf_single.transform.rotation.z << " "
               << tf_single.transform.rotation.w
               << " \nand equivalent heading :" << theta_patched / M_PI * 180 << " degree"
               << "\n\n-----------------------------------\n";
            RCLCPP_INFO(this->get_logger(), ss.str());
        }

        tf_series.transforms.emplace_back(tf_single);
        patched_localizationTF_pub_->publish(tf_series);

        return;
    }

    bool WriteToFile(std::string file_name, std::string writter)
    {
        if (counter == 0)
        {
            std::remove("marker_log.txt"); // delete file
        }
        std::ofstream file;
        // Open a file to log
        file.open(file_name, std::ios_base::app);
        if (counter == 0)
        {
            file << "Tracked object ID"
                 << ","
                 << "ROS Marker ID"
                 << ","
                 << "ROS Marker Color"
                 << ","
                 << "Object Class" << std::endl;
        }
        // file << trackedobj_id_ << "," << ros_marker_id_ << "," << ros_marker_color_ <<
        // ","
        //      << object_class_ << std::endl;
        file << writter;
        file.close();
        counter++;

        return true;
    }

    // Class Variables
    rclcpp::Subscription<trackedobj_msgs::msg::Tracks>::SharedPtr marker_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerArray_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr patched_localizationTF_pub_;
    int counter;
    bool log_as_csv;
    // Prevents this Node from being copied
    RCLCPP_DISABLE_COPY(VisualizationMarker)
};

// Used to register this node as a ros 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(VisualizationMarker)
