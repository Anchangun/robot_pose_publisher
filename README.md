# RobotPosePublisher
robot pose(almost real time) publisher node.
robot_pose_publisher publishes the transform between the /base_link frame and the /map frame.

### 개발환경
* ![Dev badge](https://img.shields.io/badge/ROS2-Foxy-orange?style=flat&logo=ROS&logoColor=white)
* ![Dev_badge](https://img.shields.io/badge/Ubuntu-20.04-brightgreen?style=flat&logo=Ubuntu&logoColor=white)
* ![Dev_badge](https://img.shields.io/badge/HardWare-MSI.AMD-lightgrey?style=flat&logo=MSI&logoColor=white) ![Dev_badge](https://img.shields.io/badge/HardWare-NUC12-lightgrey?style=flat&logo=INTEL&logoColor=white)
* ![Dev_badge](https://img.shields.io/badge/Robot-JARA-blue) ![Dev_badge](https://img.shields.io/badge/Robot-Wabot3-blue) 
* ![Dev_badge](https://img.shields.io/badge/Test-GAZEBO-navy)
<br/>

### Source Directory Structure
```
│─ include
│    │─ df_robot_pose_publisher.hpp
│    └─ robot_pose_publisher.hpp
└─ src
     └─ robot_pose_publisher.cpp
```

### Published Topics
* robot_pose(geometry_msgs/PoseStamped)
* robot_pose(geometry_msgs/Pose)
<br/>

### Parameter
* map_frame(Default:map)
* base_grame(Default:base_link)
* is_stamped (Default:false)
<br/>

### Compile 
```shell
colcon build --packages-select robot_pose_publisher
```
<br/>

### Start up
```shell
ros2 run robot_pose_publisher robot_pose_publisher_node
```

### Reference
http://wiki.ros.org/robot_pose_publisher
