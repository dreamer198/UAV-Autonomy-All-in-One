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




#include <plan_manage/topo_replan_fsm.h>
#include <path_searching/topology_safety.h>
#include <cmath>
#include <stdexcept>

namespace fast_planner {

void TopoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  trigger_     = false;
  have_target_ = false;
  have_odom_   = false;
  collide_     = false;
  emergency_recovery_ = false;
  emergency_stop_settled_since_ = ros::Time();
  next_planning_attempt_ = ros::Time();
  goal_yaw_ = 0.0;
  goal_yaw_constrained_ = false;
  active_goal_id_ = 0;

  /* 读取拓扑重规划 FSM 参数：目标来源、重规划阈值、航点和是否启用主动建图。 */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_time_threshold_, -1.0);
  nh.param("fsm/thresh_no_replan", replan_distance_threshold_, -1.0);
  nh.param("fsm/emergency_stop_velocity", emergency_stop_velocity_threshold_, 0.15);
  nh.param("fsm/emergency_stop_settle_time", emergency_stop_settle_time_, 0.20);
  nh.param("fsm/max_tracking_error", max_tracking_error_, 0.50);
  nh.param("fsm/planning_retry_interval", planning_retry_interval_, 0.10);
  nh.param("manager/clearance_threshold", goal_clearance_, -1.0);
  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  nh.param("fsm/act_map", act_map_, false);
  if (!std::isfinite(emergency_stop_velocity_threshold_) ||
      !std::isfinite(emergency_stop_settle_time_) ||
      !std::isfinite(max_tracking_error_) ||
      !std::isfinite(planning_retry_interval_) ||
      !std::isfinite(goal_clearance_) ||
      emergency_stop_velocity_threshold_ <= 0.0 ||
      emergency_stop_settle_time_ < 0.0 ||
      max_tracking_error_ <= 0.0 || planning_retry_interval_ <= 0.0 ||
      goal_clearance_ <= 0.0) {
    ROS_FATAL(
        "invalid emergency recovery parameters: velocity=%f settle_time=%f "
        "max_tracking_error=%f retry_interval=%f goal_clearance=%f",
        emergency_stop_velocity_threshold_, emergency_stop_settle_time_,
        max_tracking_error_, planning_retry_interval_, goal_clearance_);
    throw std::runtime_error("invalid Fast-Topo emergency recovery parameters");
  }
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* 初始化核心规划管理器和 RViz 可视化工具。 */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* FSM 主循环和安全检查分开计时：主循环负责状态跳转，安全检查负责碰撞触发。 */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &TopoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &TopoReplanFSM::checkCollisionCallback, this);

  /* 输入：目标点和里程计；输出：新轨迹通知、重规划通知和 B-spline 轨迹。 */
  waypoint_sub_ =
      nh.subscribe("/planning/fast_goal", 1, &TopoReplanFSM::goalCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &TopoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 20);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 20);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 20);
  progress_pub_ = nh.advertise<std_msgs::UInt8>("/planning/progress", 10);
}

void TopoReplanFSM::goalCallback(const plan_manage::FastPlannerGoalConstPtr& msg) {
  if (msg->action == plan_manage::FastPlannerGoal::CANCEL) {
    if (msg->goal_id != 0 && active_goal_id_ != 0 && msg->goal_id != active_goal_id_) {
      ROS_WARN_STREAM("ignore cancel for stale goal " << msg->goal_id);
      return;
    }
    have_target_ = false;
    emergency_recovery_ = false;
    emergency_stop_settled_since_ = ros::Time();
    resetPlanningRetry();
    std_msgs::Empty stop;
    replan_pub_.publish(stop);
    if (exec_state_ != INIT) changeFSMExecState(WAIT_TARGET, "CANCEL");
    return;
  }
  if (msg->action != plan_manage::FastPlannerGoal::PLAN) {
    ROS_ERROR_STREAM("reject unknown FastPlannerGoal action " << int(msg->action));
    return;
  }
  const auto& goal = msg->goal;
  const auto& p = goal.pose.position;
  const auto& q = goal.pose.orientation;
  if (goal.header.frame_id != "world" ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    ROS_ERROR("reject invalid goal: frame must be world and position must be finite");
    return;
  }
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (msg->constrain_yaw &&
      (!std::isfinite(q_norm) || q_norm < 1e-6 || !std::isfinite(q.x) ||
       !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))) {
    ROS_ERROR("reject invalid constrained-yaw quaternion");
    return;
  }
  cout << "Triggered!" << endl;
  active_goal_id_ = msg->goal_id;
  emergency_recovery_ = false;
  emergency_stop_settled_since_ = ros::Time();
  resetPlanningRetry();

  // topo 模式先生成一条全局参考轨迹；这里把目标或预设航点整理成 global waypoints。
  vector<Eigen::Vector3d> global_wp;
  if (target_type_ == TARGET_TYPE::REFENCE_PATH) {
    // REFENCE_PATH 模式使用 launch 中配置的整条参考路径。
    for (int i = 0; i < waypoint_num_; ++i) {
      Eigen::Vector3d pt;
      pt(0) = waypoints_[i][0];
      pt(1) = waypoints_[i][1];
      pt(2) = waypoints_[i][2];
      global_wp.push_back(pt);
    }
  } else {

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
      // 插件模式完整保留请求的三维终点。
      target_point_(0) = p.x;
      target_point_(1) = p.y;
      target_point_(2) = p.z;
      std::cout << "manual: " << target_point_.transpose() << std::endl;

    } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
      if (waypoint_num_ <= 0 || waypoint_num_ > 50) {
        ROS_ERROR("invalid preset waypoint count");
        return;
      }
      // 预设模式按配置航点循环，每次触发切换到下一个目标。
      target_point_(0) = waypoints_[current_wp_][0];
      target_point_(1) = waypoints_[current_wp_][1];
      target_point_(2) = waypoints_[current_wp_][2];

      current_wp_ = (current_wp_ + 1) % waypoint_num_;
      std::cout << "preset: " << target_point_.transpose() << std::endl;
    } else {
      ROS_ERROR_STREAM("unsupported fsm/flight_type " << target_type_);
      return;
    }

    global_wp.push_back(target_point_);
    visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  }
  goal_yaw_constrained_ = msg->constrain_yaw;
  if (goal_yaw_constrained_) {
    Eigen::Quaterniond goal_q(q.w / q_norm, q.x / q_norm, q.y / q_norm, q.z / q_norm);
    const Eigen::Vector3d goal_x = goal_q.toRotationMatrix().col(0);
    goal_yaw_ = std::atan2(goal_x.y(), goal_x.x());
  }

  // 将全局航点交给 PlannerManager，后续 planGlobalTraj/topoReplan 都会使用它。
  planner_manager_->setGlobalWaypoints(global_wp);
  end_vel_.setZero();
  have_target_ = true;
  trigger_     = true;

  if (exec_state_ == WAIT_TARGET) {
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  } else if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
    replan_pub_.publish(std_msgs::Empty());
    new_pub_.publish(std_msgs::Empty());
    // Preemption stops the old public command immediately, so regenerate the
    // global/local trajectory from current odometry rather than old samples.
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  }
}

void TopoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  const auto& p = msg->pose.pose.position;
  const auto& v = msg->twist.twist.linear;
  const auto& q = msg->pose.pose.orientation;
  if (msg->header.frame_id != "world" || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.z) || !std::isfinite(v.x) || !std::isfinite(v.y) ||
      !std::isfinite(v.z) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w)) {
    ROS_ERROR_THROTTLE(1.0, "reject invalid world-frame odometry");
    have_odom_ = false;
    return;
  }
  // 缓存当前无人机位置、速度和姿态，生成新轨迹时会作为起始状态。
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void TopoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[6] = { "INIT",
                          "WAIT_TARGET",
                          "GEN_NEW_TRAJ",
                          "REPLAN_TRAJ",
                          "EXEC_TRAJ",
                          "REPLAN_"
                          "NEW" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void TopoReplanFSM::printFSMExecState() {
  string state_str[6] = { "INIT",
                          "WAIT_TARGET",
                          "GEN_NEW_TRAJ",
                          "REPLAN_TRAJ",
                          "EXEC_TRAJ",
                          "REPLAN_"
                          "NEW" };
  cout << "state: " + state_str[int(exec_state_)] << endl;
}

bool TopoReplanFSM::planningRetryPending() const {
  return !next_planning_attempt_.isZero() &&
      ros::Time::now() < next_planning_attempt_;
}

void TopoReplanFSM::deferPlanningRetry() {
  next_planning_attempt_ =
      ros::Time::now() + ros::Duration(planning_retry_interval_);
}

void TopoReplanFSM::resetPlanningRetry() {
  next_planning_attempt_ = ros::Time();
}

void TopoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  std_msgs::UInt8 progress;
  progress.data = static_cast<uint8_t>(exec_state_);
  progress_pub_.publish(progress);
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "no trigger_." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      // 初始化阶段等待 odom 和外部触发，避免状态不完整时规划。
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");

      break;
    }

    case WAIT_TARGET: {
      // 系统已就绪，等待 waypointCallback 写入目标/参考路径。
      if (!have_target_) {
        emergency_recovery_ = false;
        emergency_stop_settled_since_ = ros::Time();
        return;
      }

      if (emergency_recovery_) {
        /*
         * The public adapter has already truncated the unsafe trajectory.
         * Wait for the measured vehicle velocity to settle before rebuilding
         * from odometry; immediately sampling the old trajectory would retain
         * its velocity towards the newly observed obstacle.
         */
        if (odom_vel_.norm() > emergency_stop_velocity_threshold_) {
          emergency_stop_settled_since_ = ros::Time();
          return;
        }
        const ros::Time now = ros::Time::now();
        if (emergency_stop_settled_since_.isZero()) {
          emergency_stop_settled_since_ = now;
          return;
        }
        if ((now - emergency_stop_settled_since_).toSec() <
            emergency_stop_settle_time_) {
          return;
        }
        ROS_WARN(
            "Emergency hold settled; rebuilding the topological trajectory "
            "from measured odometry while retaining goal %lu",
            static_cast<unsigned long>(active_goal_id_));
        emergency_recovery_ = false;
        emergency_stop_settled_since_ = ros::Time();
      }

      changeFSMExecState(GEN_NEW_TRAJ, "FSM");

      break;
    }

    case GEN_NEW_TRAJ: {
      if (planningRetryPending()) return;

      // 生成全新 topo 轨迹：先以当前 odom 作为全局轨迹起点。
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      new_pub_.publish(std_msgs::Empty());
      /* step=1：先生成全局参考轨迹，并截取第一段局部 B-spline 发布执行。 */
      bool success = callTopologicalTraj(1);
      if (success) {
        resetPlanningRetry();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        deferPlanningRetry();
        ROS_WARN_THROTTLE(
            1.0,
            "Fast-Topo initial planning failed; retrying every %.2f s.",
            planning_retry_interval_);
      }
      break;
    }

    case EXEC_TRAJ: {
      /* 执行阶段根据全局轨迹进度、局部轨迹时长和剩余距离判断是否重规划。 */

      GlobalTrajData* global_data = &planner_manager_->global_data_;
      ros::Time       time_now    = ros::Time::now();
      double          t_cur       = (time_now - global_data->global_start_time_).toSec();

      if (t_cur > global_data->global_duration_ - 1e-2) {
        // 全局参考轨迹已经执行完，回到等待目标状态。
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else {
        LocalTrajData*  info      = &planner_manager_->local_data_;
        t_cur                     = (time_now - info->start_time_).toSec();
        const double sample_time =
            std::max(0.0, std::min(t_cur, info->duration_));
        const Eigen::Vector3d planned_position =
            info->position_traj_.evaluateDeBoorT(sample_time);
        const double tracking_error = (odom_pos_ - planned_position).norm();
        if (!planned_position.allFinite() || !std::isfinite(tracking_error) ||
            tracking_error > max_tracking_error_) {
          /*
           * The upstream demo replans from the ideal B-spline state. With a
           * real PX4/SE3 loop that reference can get ahead after the vehicle
           * slows near a wall. Continuing to sample it then commands the
           * vehicle through the obstacle. Truncate at the measured pose and
           * reuse the existing settled emergency-recovery path, which starts
           * the next global/topological plan from odometry.
           */
          ROS_ERROR(
              "trajectory tracking error %.3f m exceeds %.3f m; holding at "
              "measured pose before replanning",
              tracking_error, max_tracking_error_);
          replan_pub_.publish(std_msgs::Empty());
          emergency_recovery_ = true;
          emergency_stop_settled_since_ = ros::Time();
          collide_ = true;
          changeFSMExecState(WAIT_TARGET, "TRACKING");
          return;
        }

        if (t_cur > replan_time_threshold_) {

          if (!global_data->localTrajReachTarget()) {
            // 局部段还没有覆盖到全局目标，周期性截取下一段局部轨迹。
            changeFSMExecState(REPLAN_TRAJ, "FSM");

          } else {
            // 局部段已经到全局终点附近时，只在离局部终点还较远时继续重规划。
            Eigen::Vector3d cur_pos = info->position_traj_.evaluateDeBoorT(t_cur);
            Eigen::Vector3d end_pos = info->position_traj_.evaluateDeBoorT(info->duration_);
            if ((cur_pos - end_pos).norm() > replan_distance_threshold_)
              changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
      }
      break;
    }

    case REPLAN_TRAJ: {
      if (planningRetryPending()) return;

      // 从当前正在执行的局部 B-spline 上取状态作为重规划起点，保证轨迹连续。
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      // 在同步搜索/优化前先截断旧轨迹；若规划器卡住，adapter 只会保持
      // 截断点，不会继续回放一条已失去碰撞监测的旧轨迹。
      replan_pub_.publish(std_msgs::Empty());
      bool success = callTopologicalTraj(2);
      if (success) {
        resetPlanningRetry();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        deferPlanningRetry();
        ROS_WARN_THROTTLE(
            1.0, "Fast-Topo replanning failed; retrying every %.2f s.",
            planning_retry_interval_);
      }

      break;
    }
    case REPLAN_NEW: {
      if (planningRetryPending()) return;

      // 目标点被安全检查修改后，重新生成一条新的全局参考轨迹。
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      // This state can be entered because the current target became unsafe.
      // Truncate the old trajectory before the synchronous topological search
      // so a slow search cannot keep streaming motion toward that target.
      replan_pub_.publish(std_msgs::Empty());
      new_pub_.publish(std_msgs::Empty());

      // bool success = callSearchAndOptimization();
      // step=1 表示重新走“全局轨迹生成 + 第一段局部轨迹”流程。
      bool success = callTopologicalTraj(1);
      if (success) {
        resetPlanningRetry();
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        deferPlanningRetry();
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      break;
    }
  }
}

void TopoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  /* ---------- check goal safety ---------- */
  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(target_point_, /* time to program start */ info->duration_) :
        edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (violatesRequiredClearance(dist, goal_clearance_)) {
      /* try to find a max distance goal around */
      const double dr = 0.5, dtheta = 30, dz = 0.3;

      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal = target_point_;
      bool            fallback_found = false;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        double          ring_max_dist = -1.0;
        Eigen::Vector3d ring_goal = target_point_;
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (int z_index = 0; z_index < 3; ++z_index) {
            const double nz =
                z_index == 0 ? 0.0 : (z_index == 1 ? dz : -dz);

            new_x = target_point_(0) + r * cos(theta / 57.3);
            new_y = target_point_(1) + r * sin(theta / 57.3);
            new_z = target_point_(2) + nz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (isObservedClearance(dist) && dist > ring_max_dist) {
              ring_goal = new_pt;
              ring_max_dist = dist;
            }
          }
        }
        /*
         * Select the best candidate on the nearest safe ring. The upstream
         * global maximum tends to choose cells outside the current ESDF box,
         * where SDFMap reports its 10000 sentinel, and can move the goal into
         * an obstacle that the next cloud reveals.
         */
        if (!violatesRequiredClearance(ring_max_dist, goal_clearance_)) {
          goal = ring_goal;
          max_dist = ring_max_dist;
          fallback_found = true;
          break;
        }
      }

      if (fallback_found) {
        ROS_WARN(
            "Topological goal clearance fell below %.3f m; replacing it with "
            "nearby safe goal (%.3f, %.3f, %.3f), clearance %.3f m.",
            goal_clearance_, goal.x(), goal.y(), goal.z(), max_dist);
        target_point_ = goal;
        have_target_  = true;
        end_vel_.setZero();
        vector<Eigen::Vector3d> global_wp(1, target_point_);
        planner_manager_->setGlobalWaypoints(global_wp);

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_NEW, "SAFETY");
        }

        visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        ROS_ERROR_THROTTLE(
            1.0,
            "Topological goal is below the required %.3f m clearance and no "
            "nearby safe fallback was found.",
            goal_clearance_);
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
    /*
     * Any newly unsafe trajectory is truncated at the measured vehicle pose.
     * Replanning from a future sample of the old B-spline can leave the new
     * trajectory disconnected from an airframe that already slowed down.
     */
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN(
          "current trajectory becomes unsafe in %.3f m; holding at measured "
          "pose before rebuilding it",
          dist);
      replan_pub_.publish(std_msgs::Empty());
      /*
       * Stop the public command immediately, but do not discard the active
       * goal. Once measured velocity has settled, WAIT_TARGET regenerates a
       * map-aware topological trajectory from odometry.
       */
      emergency_recovery_ = true;
      emergency_stop_settled_since_ = ros::Time();
      collide_ = true;
      changeFSMExecState(WAIT_TARGET, "SAFETY");
    } else {
      collide_ = false;
    }
  }
}

bool TopoReplanFSM::callSearchAndOptimization() { return false; }

bool TopoReplanFSM::callTopologicalTraj(int step) {
  bool plan_success;

  if (step == 1) {
    // 第一次规划：先根据 global waypoints 生成全局参考轨迹和初始局部段。
    plan_success = planner_manager_->planGlobalTraj(start_pt_);
    if (plan_success) {
      double collision_distance = 0.0;
      if (!planner_manager_->checkTrajCollision(collision_distance)) {
        /*
         * A straight global reference is only an initialization for Topo. Do
         * not publish it when the current map already shows a collision;
         * search a topologically distinct local segment first.
         */
        ROS_WARN(
            "Initial topological reference collides in %.3f m; searching "
            "before publication.",
            collision_distance);
        plan_success = planner_manager_->topoReplan(true);
      }
    }
  } else {
    // 后续重规划：根据当前局部段是否碰撞，决定是否调用拓扑路径搜索绕障。
    plan_success = planner_manager_->topoReplan(collide_);
  }

  if (plan_success) {
    double collision_distance = 0.0;
    if (!planner_manager_->checkTrajCollision(collision_distance)) {
      /*
       * NLopt may return a finite but still colliding candidate (including
       * after an exception). Keep the adapter's hold active and retry instead
       * of exposing that trajectory to the command gateway.
       */
      ROS_WARN(
          "Rejecting colliding Fast-Topo candidate before publication "
          "(collision in %.3f m).",
          collision_distance);
      // A normal rolling refinement can expose a newly mapped obstacle on the
      // global reference. Make the next retry enter the topological branch
      // instead of deterministically repeating the same unsafe refinement.
      collide_ = true;
      return false;
    }

    // 位置轨迹确定后，再规划 yaw，并将位置/yaw B-spline 一起发给 traj_server。
    planner_manager_->planYaw(start_yaw_, goal_yaw_constrained_, goal_yaw_);

    LocalTrajData* locdat = &planner_manager_->local_data_;

    /* 发布最新局部轨迹给 traj_server 执行。 */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = locdat->start_time_;
    bspline.traj_id    = locdat->traj_id_;
    bspline.goal_id    = active_goal_id_;
    bspline.active_goal.x = target_point_.x();
    bspline.active_goal.y = target_point_.y();
    bspline.active_goal.z = target_point_.z();

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = locdat->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);

    /* 可视化：全局参考轨迹、当前局部 B-spline、topo 候选轨迹和 yaw 轨迹。 */

    MidPlanData* plan_data = &planner_manager_->plan_data_;
    visualization_->drawPolynomialTraj(planner_manager_->global_data_.global_traj_, 0.05,
                                       Eigen::Vector4d(0, 0, 0, 1), 0);
    visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), false,
                                0.15, Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99, 99);
    visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos2_, 0.075);
    visualization_->drawYawTraj(locdat->position_traj_, locdat->yaw_traj_, plan_data->dt_yaw_);

    return true;
  } else {
    return false;
  }
}
// TopoReplanFSM::
}  // namespace fast_planner
