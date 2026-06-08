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

if ! grep -q 'add_dependencies(fastlio_mapping' "$FAST_LIO_ROOT/CMakeLists.txt"; then
  sed -i '/add_executable(fastlio_mapping/a add_dependencies(fastlio_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})' \
    "$FAST_LIO_ROOT/CMakeLists.txt"
fi
