cmake_minimum_required(VERSION 2.8.3)
project(bspline_opt)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# Ubuntu and multi-architecture images install NLopt through the system
# package manager. Prefer its CMake target and fall back to pkg-config for
# older distributions; never assume /usr/local or an architecture-specific
# library filename.
find_package(NLopt CONFIG QUIET)
if(TARGET NLopt::nlopt)
  set(FAST_PLANNER_NLOPT_TARGET NLopt::nlopt)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(NLOPT REQUIRED nlopt)
  set(FAST_PLANNER_NLOPT_TARGET ${NLOPT_LIBRARIES})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  plan_env
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline_opt
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIRS}
)

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    ${FAST_PLANNER_NLOPT_TARGET}
    )
