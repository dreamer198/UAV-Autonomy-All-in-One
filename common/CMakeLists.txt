cmake_minimum_required(VERSION 3.0.2)
project(sim2real_common)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sim2real_planning_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/arm_executor.py
  scripts/goal_executor.py
  scripts/localization_guard.py
  scripts/mission_executor.py
  scripts/rviz_goal_to_diff_planner.py
  scripts/waypoint_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_arm_executor.py)
  catkin_add_nosetests(test/test_controller_gateway_contract.py)
  catkin_add_nosetests(test/test_goal_executor.py)
  catkin_add_nosetests(test/test_localization_guard.py)
  catkin_add_nosetests(test/test_mission_executor.py)
  catkin_add_nosetests(test/test_shared_arm_entrypoints.py)
  catkin_add_nosetests(test/test_shared_goal_entrypoints.py)
  catkin_add_nosetests(test/test_shared_localization_guard_entrypoints.py)
  catkin_add_nosetests(test/test_shared_mission_entrypoints.py)
  catkin_add_nosetests(test/test_shared_rosbag_lifecycle.py)
  catkin_add_nosetests(test/test_shell_safety_contracts.py)
  catkin_add_nosetests(test/test_sim_build_retry.py)
  catkin_add_nosetests(test/test_ros_adapter_safety.py)
  catkin_add_nosetests(test/test_waypoint_mission.py)
endif()
