cmake_minimum_required(VERSION 3.0.2)
project(sim2real_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
  trajectory_msgs
)

add_message_files(
  FILES
  PlannerGoal.msg
  PlannerCommand.msg
  PlannerStatus.msg
  PlannerCapabilities.msg
)

add_service_files(
  FILES
  ValidateGoal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
  trajectory_msgs
)
