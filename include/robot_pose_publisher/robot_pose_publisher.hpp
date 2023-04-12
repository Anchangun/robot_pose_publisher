#ifndef ROBOT_POSE_PUBLISHER
#define ROBOT_POSE_PUBLISHER

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "robot_pose_publisher/df_robot_pose_publisher.hpp"


using namespace std::chrono_literals;

class RobotPosePublisher : public rclcpp::Node{
    public:
        RobotPosePublisher();
    private:
        rclcpp::TimerBase::SharedPtr m_timer;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_publisher_stamp;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_publisher;
        bool m_is_stamped;
        std::string m_base_frame;
        std::string m_map_frame;
        const std::string m_tp_robot_pose;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        void fn_timer_callback();
};
#endif