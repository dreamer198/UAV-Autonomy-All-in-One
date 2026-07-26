cmake_minimum_required(VERSION 3.0.2)
project(sim2real_simulation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf
  tf2_ros
  tf2_sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pointcloud_to_world.py
  scripts/reconstruct_bag_world.py
  scripts/rviz_2d_goal_bridge.py
  scripts/sim_odometry_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_sim_adapter_safety.py)
endif()
