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
 * 阅读定位：本文件是 kinodynamic 模式的运行时调度层，不实现搜索/优化数学细节。
 * 它把“目标、里程计、当前轨迹和安全检查”组织成如下滚动规划闭环：
 *
 *   INIT --已有 odom 和目标触发--> WAIT_TARGET
 *   WAIT_TARGET --已有目标--> GEN_NEW_TRAJ
 *   GEN_NEW_TRAJ --成功--> EXEC_TRAJ；失败则留在 GEN_NEW_TRAJ 重试
 *   EXEC_TRAJ --执行结束--> WAIT_TARGET
 *   EXEC_TRAJ --达到滚动阈值/新目标/碰撞--> REPLAN_TRAJ
 *   REPLAN_TRAJ --成功--> EXEC_TRAJ；失败则退回 GEN_NEW_TRAJ
 *
 * 首次规划从最新 odom 状态出发；滚动重规划从旧 B-spline 在当前执行时钟下的预测状态出发，
 * 以保持位置、速度和加速度连续。成功结果被序列化为 Bspline，交给 traj_server 按时间执行。
 */

#include <plan_manage/kino_replan_fsm.h>
#include <cmath>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  // FSM 对象只保存调度状态；轨迹和算法数据由 planner_manager_ 统一持有。
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  trigger_     = false;
  have_target_ = false;
  have_odom_   = false;
  goal_yaw_ = 0.0;
  goal_yaw_constrained_ = false;
  active_goal_id_ = 0;

  // trigger_ 会在第一个 waypoint 回调中置 true，用于 INIT 的启动门控。
  // 注意：原实现没有在这里显式初始化 trigger_，阅读/移植时不要把它误认为已在构造器中清零。

  /*
   * target_type_：1=手动目标，2=预设航点；replan_thresh_ 是离本段起点足够远后才重规划的阈值，
   * no_replan_thresh_ 是距最终目标足够近后停止滚动重规划的阈值。二者共同避免刚发布或即将结束时抖动。
   */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  // 参数名由下标拼接而成，launch 中的 waypoint0_x/y/z 等在此读入固定数组。
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /*
   * Manager 是算法总入口：其 initPlanModules() 创建 SDFMap/EDT、kinodynamic A* 和 B-spline
   * 优化器；FSM 之后只传入边界状态并读取 local_data_ 中的最终轨迹。
   */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /*
   * 100 Hz exec_timer_ 推进状态机，20 Hz safety_timer_ 独立检查目标与当前轨迹。
   * 默认 ros::spin() 是单线程的，因此这些回调与 odom/目标回调不会并发修改成员。
   */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  /*
   * ROS 协议：waypoint_generator 发布目标 Path，odom 提供首次规划状态；/planning/replan
   * 先通知 traj_server 截短旧轨迹，/planning/bspline 再携带新轨迹。/planning/new 在 kino FSM
   * 中只保留了发布接口、没有实际 publish，主要由 topo FSM 用于清空执行轨迹可视化。
   */
  waypoint_sub_ =
      nh.subscribe("/planning/fast_goal", 1, &KinoReplanFSM::goalCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  progress_pub_ = nh.advertise<std_msgs::UInt8>("/planning/progress", 10);
}

void KinoReplanFSM::goalCallback(const plan_manage::FastPlannerGoalConstPtr& msg) {
  if (msg->action == plan_manage::FastPlannerGoal::CANCEL) {
    if (msg->goal_id != 0 && active_goal_id_ != 0 && msg->goal_id != active_goal_id_) {
      ROS_WARN_STREAM("ignore cancel for stale goal " << msg->goal_id);
      return;
    }
    have_target_ = false;
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
  trigger_ = true;
  active_goal_id_ = msg->goal_id;

  // 插件模式完整保留目标高度；预设模式仍供上游兼容配置使用。
  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << p.x, p.y, p.z;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    if (waypoint_num_ <= 0 || waypoint_num_ > 50) {
      ROS_ERROR("invalid preset waypoint count");
      return;
    }
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  } else {
    ROS_ERROR_STREAM("unsupported fsm/flight_type " << target_type_);
    return;
  }
  goal_yaw_constrained_ = msg->constrain_yaw;
  if (goal_yaw_constrained_) {
    Eigen::Quaterniond goal_q(q.w / q_norm, q.x / q_norm, q.y / q_norm, q.z / q_norm);
    const Eigen::Vector3d goal_x = goal_q.toRotationMatrix().col(0);
    goal_yaw_ = std::atan2(goal_x.y(), goal_x.x());
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  // 目标速度固定为零，要求生成的轨迹在最终目标处停下。
  end_vel_.setZero();
  have_target_ = true;

  // 新目标到来时：空闲状态从 odom 生成全新轨迹；执行状态从旧轨迹当前时刻平滑切换。
  // 若此时仍在 INIT，目标标志会被保存，定时器完成 INIT -> WAIT_TARGET -> GEN_NEW_TRAJ。
  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ) {
    replan_pub_.publish(std_msgs::Empty());
    // The public gateway has already closed the old command gate. Rebuild
    // from the latest measured odom instead of sampling the now-stopped path.
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  }
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
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
  // 缓存 world 系位置、线速度和姿态。GEN_NEW_TRAJ 使用这些量；REPLAN_TRAJ 则改用轨迹采样值。
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

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  // pos_call 标明转换来自正常 FSM、外部目标 TRIG 还是 SAFETY 检查，日志可直接还原转换原因。
  string state_str[6] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ",
                          "REPLAN_NEW" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[6] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ",
                          "REPLAN_NEW" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  std_msgs::UInt8 progress;
  progress.data = static_cast<uint8_t>(exec_state_);
  progress_pub_.publish(progress);
  // 每 100 次（约 1 s）输出一次状态和缺失条件，避免 100 Hz 定时器刷屏。
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      // 启动门控要求既收到 odom 又收到过目标触发。目标本身已经由 waypointCallback 缓存。
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
      // 空闲态不做规划；执行完一条轨迹后会清掉 have_target_ 并回到这里等待下一目标。
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      // 首次规划或重规划降级：起点直接取最新 odom，因 odom 不含加速度而将其置零。
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // 用机体系 x 轴在 world 的投影恢复当前 yaw；yaw 角速度/角加速度初值均设为零。
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        // 保持 GEN_NEW_TRAJ，下一次 10 ms 定时器继续用更新后的 odom 重试。
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /*
       * 执行态不直接给控制器发命令，只根据 Manager 保存的同一条轨迹判断何时滚动重规划；
       * 真正的 100 Hz 采样在独立 traj_server 进程中进行。
       */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      // 判断逻辑只需 [0,duration] 内的位置，故轨迹时间在末端截断。
      t_cur                   = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* 三个分支的优先级：执行完毕 > 靠近最终目标 > 尚未离开本段起点 > 触发滚动重规划。 */
      if (t_cur > info->duration_ - 1e-2) {
        // 当前轨迹已经执行完，回到等待目标状态。
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // 离最终目标小于 no_replan_thresh_ 时不再更新轨迹，让当前轨迹稳定收敛到终点。
        // cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // 相对本段 start_pos_ 尚未飞过 replan_thresh_ 时保持当前轨迹，限制重规划频率。
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      // t_cur 是旧轨迹从 start_time_ 到当前 wall time 已执行的时间；在旧位置/yaw B-spline 上
      // 同时采样 0/1/2 阶预测状态作为新边界条件。相对直接使用有噪声的 odom，这相当于从
      // 命令轨迹上的“当前前瞻状态”续接，更容易保证二阶连续；代码没有再额外增加前视时长。
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      // 先发 Empty 通知：traj_server 将旧轨迹终点截到“当前时刻+10 ms”，避免继续沿失效轨迹飞行。
      // 随后的规划若成功，新 Bspline 会覆盖执行器缓存；失败则降级到 odom 起点的 GEN_NEW_TRAJ。
      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  // 安全定时器只改变 FSM/目标，不生成控制命令；实际新轨迹仍由 execFSMCallback 调度。
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    // 静态场景（默认）用 time=-1 查询当前 ESDF。动态分支直接把轨迹 duration 当预测时间传入；
    // 源码旁注表明原意还应加“程序已运行时间”，当前 Manager 也未接入 ObjPredictor，故保持 dynamic=0。
    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /*
       * 原目标安全距离不足时，在半径 0.5~2.5 m、方位一圈、垂向 ±0.3 m 的离散候选中
       * 选择 ESDF 距离最大的点。这里优化的是“离障碍最远”，不是离原目标最近。
       */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        // 找到满足同一 0.3 m 门槛的替代目标；若正在执行，立即转入重规划。
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        // 周围也没有安全候选：保留目标并持续重试，同时通知执行器尽快停止旧轨迹。
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* 目标检查之后，再检查已经发布、正在执行的局部轨迹。 */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    // Manager 从当前时刻起以 0.02 s 采样未来轨迹，最多向前看约 6 m；ESDF 过小即返回 false。
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  // 核心算法调用封装：Manager 内依次做 kinodynamic A*、控制点参数化、B-spline 优化与时间调整。
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    // 位置轨迹确定后单独规划一维 yaw B-spline，起始 yaw/角速度/角加速度来自当前边界状态。
    planner_manager_->planYaw(start_yaw_, goal_yaw_constrained_, goal_yaw_);

    auto info = &planner_manager_->local_data_;

    /*
     * 发布协议：位置 B-spline 发送阶数、控制点和完整 knot；yaw 只发送控制点与均匀间隔 yaw_dt。
     * start_time 在搜索开始前设置，是所有采样的统一时间原点；traj_id 每次成功后递增。
     * traj_server 会转发二者，但不会据 traj_id 拒绝乱序消息。
     */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;
    bspline.goal_id    = active_goal_id_;
    bspline.active_goal.x = end_pt_.x();
    bspline.active_goal.y = end_pt_.y();
    bspline.active_goal.z = end_pt_.z();

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    // geometry_msgs/Point[] 按控制点行顺序序列化三维位置。
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // 显式发送非均匀 knot，traj_server 才能精确重建经过时间调整后的位置轨迹。
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    // yaw 轨迹是一维三次均匀 B-spline，下游用相同 order 和 yaw_dt 重建。
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);

    /* 发布不依赖 RViz；以下只显示 kinodynamic 粗路径和最终 B-spline/控制点。 */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
