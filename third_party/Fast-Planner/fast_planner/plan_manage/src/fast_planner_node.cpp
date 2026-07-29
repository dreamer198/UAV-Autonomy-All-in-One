/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * 阅读定位：这是规划进程的最薄入口，本文件本身不执行搜索。
 * launch 把所有参数写到节点私有命名空间后，main() 只负责选择一种 FSM 并进入 ROS 事件循环；
 * 地图更新、周期重规划和安全检查都由所选 FSM 在回调/定时器中驱动。
 */

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <atomic>
#include <thread>

#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/topo_replan_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
// 注册进程级信号处理器；崩溃时打印调用栈，便于定位规划器异常，不参与规划逻辑。
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  // 私有 NodeHandle 使 "fsm/..." 等相对参数解析到 /fast_planner_node/fsm/...；
  // FSM 内使用的绝对 topic（例如 /odom_world）仍由 launch 的 remap 改接实际数据源。
  ros::NodeHandle nh("~");

  // 规划主入口：根据 launch 中的 planner_node/planner 参数选择具体重规划策略。
  // 1 表示 kinodynamic A* + B-spline 优化，2 表示拓扑路径引导优化。
  int planner;
  nh.param("planner_node/planner", planner, -1);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;

  // 两个对象都会构造，但只初始化所选分支；init() 会进一步创建地图、搜索器、优化器和定时器。
  if (planner == 1) {
    kino_replan.init(nh);
  } else if (planner == 2) {
    topo_replan.init(nh);
  } else {
    ROS_FATAL_STREAM("planner_node/planner must be 1 (kinodynamic) or 2 (topological), got "
                     << planner);
    return 2;
  }

  // 心跳表示整个 Fast 进程仍存活，必须独立于同步搜索/优化回调。规划结果另由
  // adapter 的 planning_timeout 约束，避免一次正常的较长搜索被误判为进程退出。
  ros::Publisher heartbeat_pub =
      nh.advertise<std_msgs::Empty>("/planning/heartbeat", 1);
  std::atomic<bool> heartbeat_running(true);
  std::thread heartbeat_thread([&heartbeat_pub, &heartbeat_running]() {
    ros::WallRate rate(20.0);
    while (heartbeat_running.load() && ros::ok()) {
      heartbeat_pub.publish(std_msgs::Empty());
      rate.sleep();
    }
  });

  // 规划、地图和安全回调仍沿用上游单线程执行顺序。
  ros::Duration(1.0).sleep();
  ros::spin();

  heartbeat_running.store(false);
  heartbeat_thread.join();
  return 0;
}
