cmake_minimum_required(VERSION 3.0.2)
project(se3_controller)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS "YES")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/tune.cfg
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES se3_controller
 CATKIN_DEPENDS geometry_msgs nav_msgs mavros_msgs roscpp rospy std_msgs std_srvs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(se3_controller_node
  src/se3_controller_node.cpp
  src/se3_ctrl.cpp
)

# GCC 9 intermittently crashes in its Release optimizer
# passes while optimizing the large dynamic_reconfigure-generated config
# constructor included by se3_ctrl.cpp.  Keep Release semantics but compile
# this small controller target without GIMPLE optimization so clean and incremental simulation
# starts are deterministic.
target_compile_options(se3_controller_node PRIVATE -O0)

add_dependencies(se3_controller_node ${PROJECT_NAME}_gencfg)

target_link_libraries(se3_controller_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_se3_safety
    test/test_se3_safety.cpp
  )
  if(TARGET test_se3_safety)
    target_link_libraries(test_se3_safety
      ${catkin_LIBRARIES}
    )
  endif()
endif()
