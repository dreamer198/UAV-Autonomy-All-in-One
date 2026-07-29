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



// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <path_searching/topology_safety.h>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace fast_planner {

namespace {

double clearanceEscapeTolerance(double required_clearance) {
  return std::min(0.03, 0.10 * required_clearance);
}

double maximumClearanceEscapeDistance(double required_clearance) {
  return std::max(0.50, 2.0 * required_clearance);
}

}  // namespace

/*
 * 阅读导航：FastPlannerManager 是算法模块的装配层，不负责 ROS 状态机，也不直接发送控制命令。
 * 它接收 FSM 整理好的起终端状态，调用搜索器和优化器，并把最终轨迹写入 local_data_。
 *
 * 本文件包含两条相对独立的规划链：
 *
 *   kinodynamicReplan()
 *     Kinodynamic A* -> 路径采样 -> 三阶 B-spline 参数化 -> 控制点优化 -> 时间可行化
 *
 *   planGlobalTraj() / topoReplan()
 *     全局 min-snap 参考 -> 截取局部 B-spline -> 碰撞区间 -> 多拓扑路径
 *     -> 每条路径两阶段优化 -> 选择 jerk 最小的候选 -> 再优化和时间调整
 *
 * 位置轨迹生成后，updateTrajInfo() 统一派生速度/加速度；planYaw() 再独立生成一维
 * yaw B-spline。调用者随后从 local_data_ 取出这些结果并发布。
 *
 * 读到几个 data 容器时可按生命周期区分：pp_ 是配置和耗时统计；local_data_ 是当前
 * 最终可执行轨迹；global_data_ 只服务 topo 模式，维护全局参考与局部替换段的时间映射；
 * plan_data_ 保存搜索路径、拓扑候选、引导点等中间结果，主要供后续阶段和可视化使用。
 */

// SECTION interfaces for setup and query

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() { std::cout << "des manager" << std::endl; }

void FastPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /*
   * 读取规划器公共参数并按 feature switch 装配模块。
   * manager/use_* 来自 kino_algorithm.xml 或 topo_algorithm.xml；因此同一个 Manager
   * 能服务两套 FSM，但未启用的模块保持空指针，调用流程必须与 launch 配置匹配。
   */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
  nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh.param("manager/use_topo_path", use_topo_path, false);
  nh.param("manager/use_optimization", use_optimization, false);

  if (pp_.dynamic_) {
    ROS_FATAL("manager/dynamic_environment is not supported by the sim2real planner API v1");
    throw std::invalid_argument("dynamic obstacle mode is disabled");
  }

  local_data_.traj_id_ = 0;
  // SDFMap 持有占据栅格和 ESDF；EDTEnvironment 在其上提供距离/梯度查询。
  // 所有前端和后端共享同一地图实例，所以传感器更新会直接反映到下一次规划中。
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    // 普通几何 A*，通常用于全局或辅助路径搜索。
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    // kinodynamic A*：直接在位置-速度状态空间中搜索动态可行粗轨迹。
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    // 预创建多个 B-spline 优化器。kino 流程只用下标 0（yaw 用下标 1）；
    // topo 流程可能并行优化多条候选，每个线程独占一个实例，避免共享 NLopt 状态。
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    // 拓扑 PRM 用于寻找多条不同 homotopy class 的绕障路径。
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  // 仅 topo 流程使用：FSM 先写入整组全局航点，planGlobalTraj() 再把当前起点插到最前面。
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {

  /*
   * 执行期的轻量前视安全检查：从当前轨迹时刻起每 0.02 s 采样一次，最多检查到
   * 与当前位置直线距离 6 m 的位置或轨迹末端。这里传入 time=-1，只检查静态 ESDF。
   * 返回 false 时，distance 是碰撞采样点相对当前位置的直线距离，FSM 用它判断
   * 障碍是否已经近到需要立即重规划。
   */
  const double raw_t_now =
      (ros::Time::now() - local_data_.start_time_).toSec();
  if (!std::isfinite(raw_t_now) ||
      !std::isfinite(local_data_.duration_) ||
      local_data_.duration_ < 0.0 ||
      !std::isfinite(pp_.clearance_) || pp_.clearance_ <= 0.0) {
    distance = 0.0;
    return false;
  }
  const double t_now =
      std::max(0.0, std::min(raw_t_now, local_data_.duration_));

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);
  if (!cur_pt.allFinite()) {
    distance = 0.0;
    return false;
  }

  const double initial_distance =
      edt_environment_->evaluateCoarseEDT(cur_pt, -1.0);
  InitialClearanceEscape clearance_escape(
      initial_distance, pp_.clearance_,
      clearanceEscapeTolerance(pp_.clearance_),
      maximumClearanceEscapeDistance(pp_.clearance_));
  if (!clearance_escape.accept(initial_distance, 0.0)) {
    distance = 0.0;
    return false;
  }

  double          radius = 0.0;
  double          travelled_distance = 0.0;
  Eigen::Vector3d previous_pt = cur_pt;
  const vector<double> sample_times =
      sampleTimesIncludingEnd(t_now, local_data_.duration_, 0.02);
  for (size_t index = 1U;
       index < sample_times.size() && radius < 6.0; ++index) {
    Eigen::Vector3d fut_pt =
        local_data_.position_traj_.evaluateDeBoor(tm + sample_times[index]);
    if (!fut_pt.allFinite()) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    travelled_distance += (fut_pt - previous_pt).norm();
    previous_pt = fut_pt;

    const double clearance =
        edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    /*
     * Keep the configured clearance for normal execution. If a map update
     * places only the current start slightly inside that margin, permit a
     * short prefix solely when it continuously escapes and regains the full
     * margin. This avoids the impossible state where every safe exit is
     * rejected at its first sample.
     */
    if (!clearance_escape.accept(clearance, travelled_distance)) {
      distance = radius;
      return false;
    }
  }

  if (!clearance_escape.complete()) {
    distance = radius;
    return false;
  }
  return true;
}

// !SECTION

// SECTION kinodynamic replanning

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  /*
   * Kino 模式的唯一核心入口。输入是 FSM 选定的边界状态：首次规划通常来自 odom，
   * 在线重规划通常来自旧轨迹在未来时刻的预测状态。函数成功返回后，local_data_
   * 中已有新的位置、速度和加速度轨迹，但 yaw 要由 FSM 另行调用 planYaw() 生成。
   */

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  // 新轨迹的时间原点在搜索前确定；发布的 Bspline 消息和 traj_server 都以它为基准。
  local_data_.start_time_ = ros::Time::now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // 1. 前端搜索：状态为 [位置, 速度]，控制为分段常加速度；搜索结果是 motion
  // primitives 拼成的动态可行粗轨迹，而不是最终平滑轨迹。

  t1 = ros::Time::now();

  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // init=true 时搜索第一层固定沿用 start_acc，保证加速度连续；失败后用 init=false
    // 重试，第一层也允许采样其他加速度，以连续性换取更大的可达集合。
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  t_search = (ros::Time::now() - t1).toSec();

  // 2. 轨迹参数化：先按 ts 对粗轨迹等时间采样，再解线性方程得到三阶 B-spline
  // 控制点。ts 由期望控制点空间距离 / 最大速度估计，端点速度、加速度作为约束。

  double                  ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  // 3. 后端优化：NORMAL_PHASE 组合平滑、ESDF 避障和动力学可行性代价。
  // 若搜索只到达局部 horizon/终点附近而没有 shot 到终点，还额外约束末端位置。

  t1 = ros::Time::now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  t_opt = (ros::Time::now() - t1).toSec();

  // 4. 时间调整：控制点优化后几何形状已确定；若导数超限，只拉长 knot 时间间隔，
  // 最多迭代 3 次。这样降低速度/加速度而尽量不改变空间路径。

  t1                    = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {

    feasible = pos.reallocateTime();

    if (++iter_num >= 3) break;
  }

  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;

  double tn = pos.getTimeSum();

  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();

  // 5. 保存规划结果。updateTrajInfo() 会从位置 B-spline 派生导数轨迹并递增 traj_id。

  local_data_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  return true;
}

// !SECTION

// SECTION topological replanning

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  /*
   * Topo 模式的首次全局规划。它只根据起点和预设航点生成一条 min-snap 时间轨迹，
   * 不在这里做局部避障；后续 topoReplan() 会滚动截取其中一段，并在检测到碰撞时
   * 用拓扑路径重塑该局部段。GlobalTrajData 负责维护全局时间轴与被替换的局部段。
   */

  vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.size() == 0) std::cout << "no global waypoints!" << std::endl;

  points.insert(points.begin(), start_pos);

  // 相邻航点过远时每约 4 m 插入一个中间点，避免 min-snap 多项式在长段上过度偏离。
  vector<Eigen::Vector3d> inter_points;
  const double            dist_thresh = 4.0;

  for (int i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = floor(dist / dist_thresh) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // minSnapTraj 的每一行是一个三维航点；段时间按距离 / 最大速度初始化。
  int             pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  // 首尾段至少给 1 s 且加倍，给静止边界条件留出更平缓的加减速时间。
  time(0) *= 2.0;
  time(0) = max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = max(1.0, time(time.rows() - 1));

  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  auto time_now = ros::Time::now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // 立即从全局参考的 t=0 截取第一个局部段，转换成统一的三阶 B-spline 表示。

  double            dt, duration;
  Eigen::MatrixXd   ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_    = time_now;
  ROS_INFO("global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  ros::Time t1, t2;

  /*
   * Topo 模式的滚动局部重规划入口。t_now 属于“全局参考时间轴”；每次先从该时刻
   * 截取固定空间半径的局部段。collide 由 FSM 的安全检查给出：安全段只 refinement，
   * 碰撞段则搜索不同 homotopy class 的绕障路径并并行优化。
   */
  ros::Time time_now = ros::Time::now();
  double    t_now    = (time_now - global_data_.global_start_time_).toSec();
  double    local_traj_dt, local_traj_duration;
  double    time_inc = 0.0;

  double            segment_radius = pp_.local_traj_len_;
  Eigen::MatrixXd   ctrl_pts =
      reparamLocalTraj(t_now, local_traj_dt, local_traj_duration, segment_radius);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    // 当前局部段安全时，只做轻量 refinement 后继续执行。
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    // 保存未绕障的局部段，findCollisionRange() 将沿它定位首次进入和最后离开障碍的位置。
    vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    CollisionRangeState collision_range = CollisionRangeState::INVALID;
    int expansion_count = 0;
    while (true) {
      plan_data_.initial_local_segment_ = init_traj;
      colli_start.clear();
      colli_end.clear();
      start_pts.clear();
      end_pts.clear();
      findCollisionRange(colli_start, colli_end, start_pts, end_pts);
      collision_range =
          classifyCollisionRange(colli_start.size(), colli_end.size());

      if (collision_range != CollisionRangeState::ENDS_IN_OBSTACLE) break;

      const bool reaches_global_end =
          t_now + local_traj_duration >= global_data_.global_duration_ - 1.0e-2;
      if (reaches_global_end) break;

      /*
       * TopologyPRM needs a safe point after the last collision interval.
       * A fixed local segment may end inside a second/long wall and upstream
       * then skips PRM entirely. Expand only this exceptional search segment;
       * normal rolling replans retain the configured local horizon.
       */
      const double previous_duration = local_traj_duration;
      segment_radius += pp_.local_traj_len_;
      ctrl_pts = reparamLocalTraj(
          t_now, local_traj_dt, local_traj_duration, segment_radius);
      init_traj = NonUniformBspline(ctrl_pts, 3, local_traj_dt);
      ++expansion_count;
      ROS_WARN(
          "Topological collision has no exit within %.1f m; expanded the "
          "one-shot search segment to %.1f m.",
          segment_radius - pp_.local_traj_len_, segment_radius);

      if (expansion_count > 16 ||
          local_traj_duration <= previous_duration + 1.0e-3) {
        ROS_ERROR(
            "Unable to extend the topological reference to a safe collision "
            "exit.");
        return false;
      }
    }

    if (collision_range == CollisionRangeState::CLEAR) {
      // The map may update between the safety timer checking the currently
      // optimized trajectory and this callback rebuilding the global
      // reference segment. In that case there is no interval to pass to PRM;
      // use the now collision-free reference instead of dereferencing empty
      // colli_start/colli_end vectors.
      ROS_INFO("Collision interval disappeared; using the clear reference segment.");
      refineTraj(init_traj, time_inc);
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(
          init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

    } else if (collision_range == CollisionRangeState::ENDS_IN_OBSTACLE) {
      ROS_WARN(
          "Topological reference reaches the goal while still in an "
          "obstacle; refusing the trajectory.");
      return false;

    } else if (collision_range == CollisionRangeState::INVALID) {
      ROS_ERROR_STREAM(
          "Invalid collision transition sequence: starts=" << colli_start.size()
                                                            << ", ends="
                                                            << colli_end.size());
      return false;

    } else {
      NonUniformBspline best_traj;

      // colli_start.front() 到 colli_end.back() 是需要改道的整体区间；两端的安全采样点
      // 帮助 PRM 把候选绕障路径平滑地接回原参考轨迹。
      /* search topological distinctive paths */
      ROS_INFO("[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr>            graph;
      vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      const Eigen::Vector3d collision_start = colli_start.front();
      const Eigen::Vector3d collision_end = colli_end.back();
      if (!collision_start.allFinite() || !collision_end.allFinite() ||
          (collision_end - collision_start).norm() < 1.0e-6) {
        ROS_ERROR("Invalid or degenerate bounded collision interval.");
        return false;
      }
      topo_prm_->findTopoPaths(collision_start, collision_end, start_pts, end_pts,
                               graph, raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        ROS_WARN("No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      ROS_INFO("[Optimize]: ---------");
      t1 = ros::Time::now();

      // 每个候选对应一个独立优化器和结果槽。join() 后才能统一比较所有候选的 jerk。
      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      vector<thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (ros::Time::now() - t1).toSec();
      cout << "[planner]: optimization time: " << t_opt << endl;
      if (!selectBestTraj(best_traj)) {
        ROS_WARN("All optimized topological candidates remain in collision.");
        return false;
      }
      NonUniformBspline safe_optimized_traj = best_traj;
      refineTraj(best_traj, time_inc);
      double refined_collision_distance = 0.0;
      if (!trajectoryCollisionFree(
              best_traj, pp_.clearance_, refined_collision_distance)) {
        /*
         * The final unconstrained refinement may smooth away from the guide
         * path and cut back through an obstacle. Retain the already checked
         * topological result instead of publishing an unsafe trajectory or
         * retrying the same deterministic failure until timeout.
         */
        ROS_WARN(
            "Final refinement reintroduced a collision in %.3f m; using the "
            "safe optimized topological candidate.",
            refined_collision_distance);
        best_traj = safe_optimized_traj;
        time_inc = 0.0;
      }

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

bool FastPlannerManager::trajectoryCollisionFree(
    NonUniformBspline& traj, double clearance, double& first_collision_distance) {
  const double duration = traj.getTimeSum();
  if (!std::isfinite(duration) || duration <= 0.0 ||
      !std::isfinite(clearance) || clearance <= 0.0 ||
      !traj.getControlPoint().allFinite()) {
    first_collision_distance = 0.0;
    return false;
  }

  Eigen::Vector3d start = traj.evaluateDeBoorT(0.0);
  if (!start.allFinite()) {
    first_collision_distance = 0.0;
    return false;
  }

  const double initial_clearance =
      edt_environment_->evaluateCoarseEDT(start, -1.0);
  InitialClearanceEscape clearance_escape(
      initial_clearance, clearance,
      clearanceEscapeTolerance(clearance),
      maximumClearanceEscapeDistance(clearance));
  if (!clearance_escape.accept(initial_clearance, 0.0)) {
    first_collision_distance = 0.0;
    return false;
  }

  // Include both endpoints while sampling at least as densely as the 20 ms
  // execution-time collision monitor.
  const int sample_count =
      std::max(1, static_cast<int>(std::ceil(duration / 0.02)));
  double travelled_distance = 0.0;
  Eigen::Vector3d previous = start;
  for (int i = 1; i <= sample_count; ++i) {
    const double t = duration * static_cast<double>(i) /
        static_cast<double>(sample_count);
    Eigen::Vector3d point = traj.evaluateDeBoorT(t);
    if (!point.allFinite()) {
      first_collision_distance = 0.0;
      return false;
    }
    travelled_distance += (point - previous).norm();
    previous = point;
    const double point_clearance =
        edt_environment_->evaluateCoarseEDT(point, -1.0);
    if (!clearance_escape.accept(point_clearance, travelled_distance)) {
      first_collision_distance =
          (point - start).norm();
      return false;
    }
  }
  if (!clearance_escape.complete()) {
    first_collision_distance = (previous - start).norm();
    return false;
  }
  return true;
}

bool FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  /*
   * Jerk is only a tie-breaker among safe candidates. NORMAL_PHASE can cut a
   * corner despite its soft ESDF cost, so test every phase-2 result first. If
   * none survives, fall back to a collision-free GUIDE_PHASE result, which
   * remains in the PRM-selected topological channel.
   */
  auto jerk_order = [&](NonUniformBspline& first, NonUniformBspline& second) {
    return first.getJerk() < second.getJerk();
  };
  auto choose_safe = [&](vector<NonUniformBspline>& candidates,
                         const char* stage) {
    sort(candidates.begin(), candidates.end(), jerk_order);
    for (size_t index = 0; index < candidates.size(); ++index) {
      double collision_distance = 0.0;
      if (trajectoryCollisionFree(
              candidates[index], pp_.clearance_, collision_distance)) {
        traj = candidates[index];
        ROS_INFO(
            "Selected collision-free %s topological candidate %zu/%zu.",
            stage, index + 1U, candidates.size());
        return true;
      }
      ROS_WARN_THROTTLE(
          1.0,
          "%s topological candidate %zu/%zu violates the %.3f m clearance "
          "%.3f m from its start.",
          stage, index + 1U, candidates.size(), pp_.clearance_,
          collision_distance);
    }
    return false;
  };

  if (choose_safe(plan_data_.topo_traj_pos2_, "NORMAL_PHASE")) return true;
  if (choose_safe(plan_data_.topo_traj_pos1_, "GUIDE_PHASE")) {
    ROS_WARN(
        "NORMAL_PHASE left every candidate in collision; falling back to a "
        "safe GUIDE_PHASE topological trajectory.");
    return true;
  }
  return false;
}

void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  /*
   * 候选选定后的统一收尾：先根据速度/加速度比例缩放时间并重新参数化，再做一次
   * NORMAL_PHASE 优化。time_inc 记录局部段相对全局参考的时长变化，GlobalTrajData
   * 用它修正后续全局轨迹的时间映射。动力学已经可行时不再主动缩短时间。
   */
  ros::Time t1 = ros::Time::now();
  time_inc     = 0.0;
  double    dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts      = best_traj.getControlPoint();
  int             cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts  = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  ROS_WARN_STREAM("[Refine]: cost " << (ros::Time::now() - t1).toSec()
                                    << " seconds, time change is: " << time_inc);
}

void FastPlannerManager::updateTrajInfo() {
  // 每次生成新位置轨迹后，同步更新速度、加速度、起点、总时长和轨迹编号。
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  /*
   * 按 ratio 缩放 B-spline 内部时间后重新均匀采样并求控制点，使新曲线仍近似原空间
   * 轨迹且保留端点导数约束。只允许为恢复动力学可行性而拉长时间；ratio<=1
   * 表示原轨迹已经满足限制，不应为了“用满速度”而缩短时间并扭曲安全路径。
   */
  int    prev_num    = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int    seg_num     = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  ratio = min(1.01, max(1.0, ratio));
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt              = duration / double(seg_num);
  time_inc        = duration - time_origin;

  vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // ROS_WARN("prev: %d, new: %d", prev_num, ctrl_pts.rows());
}

void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             vector<Eigen::Vector3d> guide_path, int traj_id) {
  /*
   * 一个候选拓扑路径对应一次本函数调用，通常运行在独立线程中。traj_id 同时选择
   * 独占的优化器和结果数组槽，因此不同候选之间没有可变优化状态竞争。
   * 两阶段设计先把 B-spline 拉入目标拓扑通道，再解除引导约束做常规质量优化。
   */
  ros::Time t1;
  double    tm1, tm2, tm3;

  t1 = ros::Time::now();

  // guide path 越长，分配的 B-spline 段越多，使相邻控制点距离接近 ctrl_pt_dist。
  int             seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double          dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // std::cout << "ctrl pt num: " << ctrl_pts.rows() << std::endl;

  // 将折线路径重采样成内部控制点的一一对应引导点；删掉两端点是因为三阶
  // B-spline 的边界控制点承担端点状态约束，不参与 guide cost。
  vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, int(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // std::cout << "guide pt num: " << guide_pt.size() << std::endl;
  if (guide_pt.size() != int(ctrl_pts.rows()) - 6) ROS_WARN("what guide");

  tm1 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // 第一阶段 GUIDE_PHASE：平滑 + 可行性 + guide cost，先锁定候选的拓扑类别。

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (ros::Time::now() - t1).toSec();
  t1  = ros::Time::now();

  // 第二阶段 NORMAL_PHASE：改用 ESDF 距离代价，在保持通道的同时进一步避障、平滑。

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (ros::Time::now() - t1).toSec();
  ROS_INFO("optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(
    double start_t, double& dt, double& duration, double desired_radius) {
  /*
   * 按空间范围截取全局参考：从 start_t 向前推进，直到离起点达到
   * local_traj_len_ 或全局轨迹结束。采样密度由 ctrl_pt_dist 决定，返回实际 dt/duration。
   */

  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  const double radius =
      desired_radius > 0.0 ? desired_radius : pp_.local_traj_len_;
  global_data_.getTrajByRadius(start_t, radius, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  // 用采样点以及截取段两端的一、二阶导数，求出保持边界连续性的 B-spline 控制点。

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  // 固定时间范围和段数的重参数化版本，供 topo 候选按 guide path 长度调整控制点数量。
  vector<Eigen::Vector3d> point_set;
  vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // cout << "ctrl pts:" << ctrl_pts.rows() << endl;

  return ctrl_pts;
}

void FastPlannerManager::findCollisionRange(vector<Eigen::Vector3d>& colli_start,
                                            vector<Eigen::Vector3d>& colli_end,
                                            vector<Eigen::Vector3d>& start_pts,
                                            vector<Eigen::Vector3d>& end_pts) {
  /*
   * 以 0.05 s 沿未优化局部段扫描 ESDF，并检测 safe 状态的跳变：safe->occupied
   * 记录碰撞入口，occupied->safe 记录出口。调用方用第一个入口和最后一个出口包住
   * 全部碰撞区间；start_pts/end_pts 则是区间两侧的安全轨迹采样，供 TopologyPRM 接续。
   */
  bool               last_safe = true, safe;
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e = t_mp;
  const vector<double> sample_times =
      sampleTimesIncludingEnd(t_m, t_mp, 0.05);
  double previous_time = t_m;
  for (size_t index = 0; index < sample_times.size(); ++index) {
    const double tc = sample_times[index];

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      const double entry_time = index == 0U ? t_m : previous_time;
      colli_start.push_back(initial_traj->evaluateDeBoor(entry_time));
      if (t_s < 0.0) t_s = entry_time;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
    previous_time = tc;
  }

  if (colli_start.size() == 0) return;

  if (colli_start.size() == 1 && colli_end.size() == 0) return;

  // 按原 B-spline knot 间隔附近的密度，分别采样碰撞区间之前和之后的安全轨迹段。
  const double guide_step = initial_traj->getInterval();
  const vector<double> start_times =
      sampleTimesIncludingEnd(t_m, t_s, guide_step);
  for (double tc : start_times) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  const vector<double> end_times =
      sampleTimesIncludingEnd(t_e, t_mp, guide_step);
  for (double tc : end_times) {
    end_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }
}

// !SECTION

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw, bool constrain_end_yaw,
                                 double requested_end_yaw) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  /*
   * yaw 与位置分开规划。输入 start_yaw=[yaw, yaw_rate, yaw_acc]；每个采样时刻用
   * “当前位置指向 2 s 后位置”的方位角作为 waypoint，再优化一条一维三阶 B-spline。
   * 输出 yaw/yawdot/yawdotdot 与位置轨迹使用相同起始时刻，traj_server 会同步采样。
   */

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = max(1, int(ceil(duration / dt_yaw)));
  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // waypoint 约束覆盖各段采样时刻；calcNextYaw 把周期角展开到连续实数域，供样条优化。

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else if (!waypts.empty()) {
      waypt = waypts.back();
    } else {
      waypt << start_yaw(0), 0.0, 0.0;
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // states2pts 将 [角度, 角速度, 角加速度] 转成三阶均匀 B-spline 的前三/后三个控制点。

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v =
      local_data_.velocity_traj_.evaluateDeBoorT(max(0.0, duration - 0.1));
  Eigen::Vector3d end_yaw(
      constrain_end_yaw ? requested_end_yaw : atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // 只优化平滑度和 waypoint 跟踪；yaw 不需要 ESDF 避障项。
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // 与位置轨迹相同，通过样条求导得到角速度和角加速度轨迹。
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // yaw 与 yaw +/- 2*pi 表示同一朝向；选择离参考 last_yaw 最近的等价值，避免
  // 优化器把 +pi 到 -pi 误认为一次接近 2*pi 的大幅旋转。

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
