cmake_minimum_required(VERSION 3.0.2)
project(sim2real_fast_adapter)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(catkin REQUIRED COMPONENTS
  bspline
  geometry_msgs
  nav_msgs
  plan_manage
  roscpp
  sensor_msgs
  sim2real_planning_msgs
  std_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    bspline geometry_msgs nav_msgs plan_manage roscpp sensor_msgs
    sim2real_planning_msgs std_msgs trajectory_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(fast_backend_adapter_node src/fast_backend_adapter_node.cpp)
add_dependencies(fast_backend_adapter_node
  ${catkin_EXPORTED_TARGETS}
  ${plan_manage_EXPORTED_TARGETS}
  ${sim2real_planning_msgs_EXPORTED_TARGETS}
)
target_link_libraries(fast_backend_adapter_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_adapter_validation test/test_adapter_validation.cpp)
  if(TARGET test_adapter_validation)
    target_link_libraries(test_adapter_validation ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_fast_config.py)
endif()

install(TARGETS fast_backend_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
