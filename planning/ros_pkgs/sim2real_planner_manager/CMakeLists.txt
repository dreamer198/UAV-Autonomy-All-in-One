cmake_minimum_required(VERSION 3.0.2)
project(sim2real_planner_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  sim2real_planning_msgs
  std_srvs
  trajectory_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  sim2real_planning_msgs
  std_srvs
  trajectory_msgs
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/planner_backend_runner.py
  scripts/planner_gateway.py
  scripts/planner_plugins.py
  scripts/planner_visualization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_command_gate.py)
  catkin_add_nosetests(test/test_gateway_status.py)
  catkin_add_nosetests(test/test_manifest.py)
  catkin_add_nosetests(test/test_rate_monitor.py)
  catkin_add_nosetests(test/test_validation.py)
  catkin_add_nosetests(test/test_visualization.py)
endif()
