cmake_minimum_required(VERSION 3.0.2)
project(sim2real_super_adapter)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  sensor_msgs
  sim2real_planning_msgs
  std_msgs
  std_srvs
  super_planner
  trajectory_msgs
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    quadrotor_msgs
    roscpp
    sensor_msgs
    sim2real_planning_msgs
    std_msgs
    std_srvs
    super_planner
    trajectory_msgs
    visualization_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(super_backend_adapter_node
  src/super_backend_adapter_node.cpp
)
add_dependencies(super_backend_adapter_node
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(super_backend_adapter_node PRIVATE cxx_std_14)
target_link_libraries(super_backend_adapter_node
  ${catkin_LIBRARIES}
)

install(TARGETS super_backend_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_super_validation
    test/test_super_validation.cpp
  )
  if(TARGET test_super_validation)
    target_link_libraries(test_super_validation ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_super_config.py)
endif()
