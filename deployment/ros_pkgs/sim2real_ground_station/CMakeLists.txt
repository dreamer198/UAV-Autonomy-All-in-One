cmake_minimum_required(VERSION 3.0.2)
project(sim2real_ground_station)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  mavros_msgs
  nav_msgs
  pluginlib
  python_qt_binding
  roscpp
  rospy
  rviz
  sensor_msgs
  sim2real_planning_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    mavros_msgs
    nav_msgs
    pluginlib
    python_qt_binding
    roscpp
    rospy
    rviz
    sensor_msgs
    sim2real_planning_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/sim2real_ground_station/interactive_goal_panel.hpp
  src/interactive_goal_panel.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

catkin_install_python(PROGRAMS
  scripts/embedded_rviz.py
  scripts/ground_station_telemetry.py
  scripts/interactive_goal_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_non_modal_status.py)
endif()
