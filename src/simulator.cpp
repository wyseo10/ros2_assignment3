#include "simulator.h"

using namespace std::chrono_literals;

Simulator::Simulator() : Node("simulator")
{
    // ROS publisher
    pub_poses = this->create_publisher<visualization_msgs::msg::MarkerArray>("/poses", 10);

    // ROS subscriber
    subs_pose.resize(number_of_robots);
    for (size_t id = 0; id < number_of_robots; id++)
    {
        // This allows the callback function to get argument
        std::function<void(const turtlesim::msg::Pose msg)> fcn =
            std::bind(&Simulator::topic_callback, this, std::placeholders::_1, id);
        subs_pose[id] = this->create_subscription<turtlesim::msg::Pose>(
            "/turtlesim" + std::to_string(id) + "/turtle1/pose", 10, fcn);
    }

    // ROS tf publisher
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ROS timer
    timer_ = this->create_wall_timer(
        10ms, std::bind(&Simulator::timer_callback, this));

    // Poses array resize
    poses.resize(number_of_robots);
}

void Simulator::topic_callback(const turtlesim::msg::Pose &msg, size_t id)
{
    poses[id].position.x = msg.x - ori + (double)id * offset;
    poses[id].position.y = msg.y - ori;
    poses[id].position.z = 0;

    yaw = msg.theta;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    poses[id].orientation.w = cr * cp * cy + sr * sp * sy; 
    poses[id].orientation.x = sr * cp * cy - cr * sp * sy; 
    poses[id].orientation.y = cr * sp * sy + sr * cp * sy;
    poses[id].orientation.z = cr * cp * sy - sr * sp * cy;
    //RCLCPP_INFO(this->get_logger(), "[ID: %d] [ORIENT w : %lf] ", id, poses[id].orientation.w);
}

void Simulator::timer_callback()
{
    visualization_msgs::msg::MarkerArray msg;

    for (size_t id = 0; id < number_of_robots; id++)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "/turtlesim" + std::to_string(id);
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = poses[id];

        marker.scale.x = robot_radius;
        marker.scale.y = robot_radius;
        marker.scale.z = robot_radius;

        // Set color
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        msg.markers.push_back(marker);
    }

    pub_poses->publish(msg);
}