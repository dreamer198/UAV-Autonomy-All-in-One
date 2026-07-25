cmake_minimum_required(VERSION 3.0.2)
project(sim2real_deployment)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  livox_ros_driver2
  nav_msgs
  rospy
  sensor_msgs
  tf2_ros
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/cloud_relay.py
  scripts/livox_custom_to_pointcloud2.py
  scripts/odom_to_base.py
  scripts/odom_to_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
