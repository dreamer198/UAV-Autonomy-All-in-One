cmake_minimum_required(VERSION 3.0.2)
project(sim2real_diff_adapter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  sim2real_planning_msgs
  std_msgs
  traj_utils
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/diff_backend_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_diff_backend_adapter.py)
  catkin_add_nosetests(test/test_planner_config.py)
endif()
