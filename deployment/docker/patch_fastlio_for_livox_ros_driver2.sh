#!/usr/bin/env bash
set -euo pipefail

FAST_LIO_ROOT="${1:?usage: patch_fastlio_for_livox_ros_driver2.sh <FAST_LIO_ROOT>}"

sed -i \
  -e 's#<build_depend>livox_ros_driver</build_depend>#<build_depend>livox_ros_driver2</build_depend>#g' \
  -e 's#<run_depend>livox_ros_driver</run_depend>#<run_depend>livox_ros_driver2</run_depend>#g' \
  "$FAST_LIO_ROOT/package.xml"

sed -i \
  -e 's/^[[:space:]]*livox_ros_driver[[:space:]]*$/  livox_ros_driver2/g' \
  "$FAST_LIO_ROOT/CMakeLists.txt"

sed -i \
  -e 's#<livox_ros_driver/CustomMsg.h>#<livox_ros_driver2/CustomMsg.h>#g' \
  -e 's#livox_ros_driver::CustomMsg#livox_ros_driver2::CustomMsg#g' \
  "$FAST_LIO_ROOT/src/laserMapping.cpp" \
  "$FAST_LIO_ROOT/src/preprocess.h" \
  "$FAST_LIO_ROOT/src/preprocess.cpp"

# FAST-LIO keeps velocity in the inertial/world frame but its published
# nav_msgs/Odometry historically leaves twist at zero. Publish the equivalent
# child/body-frame linear velocity so the message follows the standard odom
# contract and can be normalized by the shared adapter/Planner path.
if ! grep -q 'sim2real_velocity_body' "$FAST_LIO_ROOT/src/laserMapping.cpp"; then
  sed -i '/set_posestamp(odomAftMapped.pose);/a\
    const V3D sim2real_velocity_body = state_point.rot.toRotationMatrix().transpose() * state_point.vel;\
    odomAftMapped.twist.twist.linear.x = sim2real_velocity_body(0);\
    odomAftMapped.twist.twist.linear.y = sim2real_velocity_body(1);\
    odomAftMapped.twist.twist.linear.z = sim2real_velocity_body(2);' \
    "$FAST_LIO_ROOT/src/laserMapping.cpp"
fi

# The upstream MID360 profile saves every scan into one unbounded in-memory
# cloud when interval=-1. Disable that deployment-hostile default; flight bags
# already provide bounded diagnostics and PCD export can be enabled explicitly.
sed -i -E \
  's/^([[:space:]]*pcd_save_en:)[[:space:]]*true([[:space:]]*)$/\1 false\2/' \
  "$FAST_LIO_ROOT/config/mid360.yaml"
grep -Eq '^[[:space:]]*pcd_save_en:[[:space:]]*false[[:space:]]*$' \
  "$FAST_LIO_ROOT/config/mid360.yaml"

if ! grep -q 'add_dependencies(fastlio_mapping' "$FAST_LIO_ROOT/CMakeLists.txt"; then
  sed -i '/add_executable(fastlio_mapping/a add_dependencies(fastlio_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})' \
    "$FAST_LIO_ROOT/CMakeLists.txt"
fi
