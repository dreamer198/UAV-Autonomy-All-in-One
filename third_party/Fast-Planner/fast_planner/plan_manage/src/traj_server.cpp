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
 * 阅读定位：traj_server 是规划与控制之间的时间参数化执行器。
 * 它不修改轨迹，也不做碰撞检查；收到 plan_manage/Bspline 后重建位置/yaw 曲线，在 100 Hz
 * 定时器中按 (now-start_time) 采样位置、速度、加速度、yaw 和 yaw_dot，再发布 PositionCommand。
 *
 * 消息链路：KinoReplanFSM --planning/bspline--> traj_server --/position_cmd--> SO3Control。
 * /planning/replan 用于提前截停旧轨迹，/planning/new 只清理 RViz 轨迹历史。
 */

#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

// 三类输出分别是当前 yaw 箭头、控制器命令和累计命令轨迹的 RViz Marker。
ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;

nav_msgs::Odometry odom;

// PositionCommand 在进程生命周期内复用；main() 一次性写入控制增益，定时器更新期望状态。
quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
// traj_ 的固定索引：0=位置，1=速度，2=加速度，3=yaw，4=yaw_dot。
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw 控制遗留变量：当前有效 yaw 直接来自轨迹；time_forward_ 被读取但未用于现有采样逻辑。
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  // 先 DELETE 同 id Marker，再以 SPHERE_LIST 整体重发，保证缩短/清空后的轨迹不会残留旧点。
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  // 用从 pos 指向 pos+vec 的 ARROW 显示命令方向；当前调用者传入的是水平 yaw 朝向。
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void bsplineCallback(plan_manage::BsplineConstPtr msg) {
  /*
   * 每条新消息会整体替换当前缓存。默认单线程 ros::spin() 保证本回调不会与 cmdCallback
   * 同时读写 traj_。消息未校验控制点尺寸、knot 单调性、start_time 合法性，也没有要求
   * traj_id 单调递增；因此延迟到达的旧消息也会覆盖新轨迹，发布端必须保证顺序和协议正确。
   */

  // 位置曲线：先恢复控制点矩阵和 knot 向量。构造器中的 0.1 只是临时间隔，随后 setKnot 覆盖。

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // yaw 消息没有携带 knot 向量，接收端用 yaw_dt 构造同阶均匀 B-spline。

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  // traj_id 由 Manager 每成功生成一条位置轨迹后递增；服务端仅复制并随 PositionCommand 转发，
  // 不用它拒绝重复/过期轨迹。start_time 则是规划开始时刻，而非本回调的接收时刻。
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  // 预先构造导数曲线，使 100 Hz 回调只做 De Boor 求值，不在实时路径上重复求导。
  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  // 规划计算和消息传输已经消耗的时间会自然体现在首次 t_cur 中，接收端会跳到对应轨迹时刻，
  // 而不是把整条轨迹延后执行；这里不检查 start_time 是否过旧或位于未来。
  receive_traj_ = true;
}

void replanCallback(std_msgs::Empty msg) {
  /*
   * FSM 在计算替代轨迹前先发此通知。把旧 duration 截到 now+10 ms 后，执行器最多再沿旧曲线
   * 运行一个控制周期；若新轨迹尚未到达，cmdCallback 会落入“末端悬停”分支。
   */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
  // 只清空累计的命令/实飞轨迹可视化，不会清空 traj_ 或停止命令发布。
  // kino FSM 当前不发布 /planning/new；topo FSM 会使用该协议标记一段全新任务。
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  // 仿真中的特殊 child_frame_id "X"/"O" 被视为无效标记，不纳入真实轨迹记录。
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  // odom 不参与控制命令采样，只记录实际飞行轨迹供调试；当前 visCallback 未启用其显示。
  odom = msg;

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  // 分块丢弃旧点而非每次 pop_front，降低长时间运行时 vector 搬移开销。
  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void visCallback(const ros::TimerEvent& e) {
  // 4 Hz 重绘累计命令位置；真实轨迹绘制保留在注释代码中，默认不发布。
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);

  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void cmdCallback(const ros::TimerEvent& e) {
  /* 第一个 Bspline 到达前保持静默，控制器不会收到未初始化的期望状态。 */
  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();
  // 与规划消息携带的 start_time_ 对齐，而不是从“消息收到时刻”重新计时。
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos, vel, acc, pos_f;
  double yaw, yawdot;

  if (t_cur < traj_duration_ && t_cur >= 0.0) {
    // 正常执行阶段：位置的导数曲线提供 vel/acc，独立 yaw 曲线提供角度及角速度前馈。
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];

    double tf = min(traj_duration_, t_cur + 2.0);
    // pos_f 是硬编码 2 s 前视点，仅供下方被禁用的“朝向路径切线”方案计算 pos_err；
    // 当前发布的 yaw 完全采用 yaw B-spline，launch 中 time_forward_ 参数因此不生效。
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /*
     * 轨迹结束后持续发布末端位置，平动速度/加速度清零；yaw 保持末端值，但 yaw_dot 仍取
     * yaw 导数曲线的末端采样值而非强制清零。这也是重规划消息与新轨迹之间的保底状态。
     */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;

  } else {
    // start_time 位于未来时进入。当前代码只打印错误但仍继续组装命令，未初始化的采样变量会被读取；
    // 正常同机规划流程中 start_time 在搜索前生成，因此发布到达时通常已经不晚于 now。
    cout << "[Traj server]: invalid time." << endl;
  }

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  // PositionCommand 同时包含期望平动状态、偏航状态、轨迹编号以及下游控制器使用的 kx/kv。
  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  // 以下 pos_err/last_yaw_ 对应另一种由前视位置计算 yaw 的实验方案；核心代码已被注释停用。
  auto pos_err = pos_f - pos;
  // if (pos_err.norm() > 1e-3) {
  //   cmd.yaw = atan2(pos_err(1), pos_err(0));
  // } else {
  //   cmd.yaw = last_yaw_;
  // }
  // cmd.yaw_dot = 1.0;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);

  // 控制输出之后发布一个长度为 2 m 的 yaw 箭头；速度/加速度箭头可按需恢复。

  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  traj_cmd_.push_back(pos);
  // 保存期望而非实测位置，用 4 Hz Marker 观察控制命令轨迹；同样分块限制内存增长。
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  // node 解析普通/绝对 topic；私有 nh 只用于读取 /traj_server/traj_server/time_forward 参数。
  // launch 中节点内的 param 名同样带 traj_server/ 前缀，因此能与这里的私有查找匹配。
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  /*
   * 输入 topic：轨迹、截停通知、任务重置通知和 odom；输出 topic：PositionCommand 与两类 Marker。
   * /position_cmd、/odom_world 是绝对名，可在 launch remap；planning/... 是相对全局命名空间。
   */
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  /* kx/kv 随每条 PositionCommand 一同下发给 SO3 控制器；它们不是 B-spline 优化权重。 */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  // 当前实现保留了参数读取和 last_yaw_ 状态，但有效 yaw 路径没有使用二者，见 cmdCallback。
  last_yaw_ = 0.0;

  // 延迟一秒等待控制器、仿真和 topic 连接建立；此后由单线程 spin 串行执行所有回调。
  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
