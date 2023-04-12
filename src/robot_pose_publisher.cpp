#include "robot_pose_publisher/robot_pose_publisher.hpp"

RobotPosePublisher::RobotPosePublisher() : Node("robot_pose_publisher"), m_is_stamped(false), m_base_frame(F_BASE_LINK), m_map_frame(F_MAP),m_tp_robot_pose(TP_ROBOT_POSE){
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_buffer->canTransform(m_map_frame, m_base_frame, tf2_ros::fromMsg(this->now()), std::chrono::seconds(1));

    this->declare_parameter<std::string>("map_frame","map");
    this->declare_parameter<std::string>("base_frame","base_link");
    this->declare_parameter<bool>("is_stamped",false);
    this->get_parameter("map_frame", m_map_frame);
    this->get_parameter("base_frame", m_base_frame);
    this->get_parameter("is_stamped", m_is_stamped);
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    if (m_is_stamped){
        m_publisher_stamp = this->create_publisher<geometry_msgs::msg::PoseStamped>(m_tp_robot_pose, qos_profile);
    }
    else{
        m_publisher = this->create_publisher<geometry_msgs::msg::Pose>(m_tp_robot_pose, qos_profile);
    }
    m_timer = this->create_wall_timer(10ms, std::bind(&RobotPosePublisher::fn_timer_callback, this));
}


void RobotPosePublisher::fn_timer_callback(){
  geometry_msgs::msg::TransformStamped transformStamped;
  try{
    transformStamped = tf_buffer->lookupTransform(m_map_frame, m_base_frame, tf2::TimePointZero);    
  }
  catch (tf2::TransformException &ex){
    RCLCPP_INFO(this->get_logger(),"fn_timer_callback : %s","tf2_transform_exception");
    return;
  }
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = m_map_frame;
  pose_stamped.header.stamp = this->now();

  pose_stamped.pose.orientation.x = transformStamped.transform.rotation.x;
  pose_stamped.pose.orientation.y = transformStamped.transform.rotation.y;
  pose_stamped.pose.orientation.z = transformStamped.transform.rotation.z;
  pose_stamped.pose.orientation.w = transformStamped.transform.rotation.w;

  pose_stamped.pose.position.x = transformStamped.transform.translation.x;
  pose_stamped.pose.position.y = transformStamped.transform.translation.y;
  pose_stamped.pose.position.z = transformStamped.transform.translation.z;

  if (m_is_stamped){
    m_publisher_stamp->publish(pose_stamped);
  }
  else{
    m_publisher->publish(pose_stamped.pose);
  }
}

int main(int argc, char *argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotPosePublisher>());
	rclcpp::shutdown();
	return 0;
}