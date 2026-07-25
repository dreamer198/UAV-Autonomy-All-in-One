#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

#include <cmath>

using namespace Eigen;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
double yaw_dot_max_per_sec_, yaw_dot_dot_max_per_sec_;
double yaw_custom_;
bool receive_yaw_ = false;
ros::Time receive_yaw_time_(0);
bool have_goal_yaw_ = false;
bool goal_yaw_active_ = false;
double goal_yaw_ = 0.0;
double goal_yaw_switch_distance_ = 0.5;
Eigen::Vector3d goal_position_(Eigen::Vector3d::Zero());

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void yawCallback(const quadrotor_msgs::PositionCommandPtr msg)
{
  receive_yaw_ = true;
  receive_yaw_time_ = ros::Time::now();
  yaw_custom_ = msg->yaw;
  // std::cout << "Received yaw:  " << yaw_custom_ << std::endl;
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  const bool valid_goal_yaw = msg->has_goal_yaw &&
                              std::isfinite(msg->goal_yaw) &&
                              std::isfinite(msg->goal_position[0]) &&
                              std::isfinite(msg->goal_position[1]) &&
                              std::isfinite(msg->goal_position[2]);
  have_goal_yaw_ = valid_goal_yaw;
  goal_yaw_active_ = false;
  if (have_goal_yaw_)
  {
    goal_yaw_ = std::atan2(std::sin(msg->goal_yaw), std::cos(msg->goal_yaw));
    goal_position_ << msg->goal_position[0], msg->goal_position[1], msg->goal_position[2];
  }
  else if (msg->has_goal_yaw)
  {
    ROS_WARN("[traj_server] Ignoring non-finite goal yaw metadata.");
  }

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  const double YAW_DOT_MAX_PER_SEC = yaw_dot_max_per_sec_;
  const double YAW_DOT_DOT_MAX_PER_SEC = yaw_dot_dot_max_per_sec_;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;

  if (have_goal_yaw_ && (goal_position_ - pos).norm() <= goal_yaw_switch_distance_)
  {
    yaw_temp = goal_yaw_;
    if (!goal_yaw_active_)
    {
      ROS_INFO("[traj_server] Tracking final goal yaw %.1f deg within %.2f m of the goal.",
               goal_yaw_ * 180.0 / M_PI,
               goal_yaw_switch_distance_);
      goal_yaw_active_ = true;
    }
  }

  if (receive_yaw_ && yaw_custom_ > -100.0)
  { 
    if ((ros::Time::now() - receive_yaw_time_).toSec() < 0.5)
    {
      yaw_temp = yaw_custom_;
    }
    else
    {
      receive_yaw_ = false;
    }
  }

  const double yaw_error = std::atan2(std::sin(yaw_temp - last_yaw_),
                                      std::cos(yaw_temp - last_yaw_));

  // Generate a discrete trapezoidal yaw profile. The stopping-speed term makes
  // the command decelerate before the target, while the rate-delta clamp keeps
  // both yaw rate and yaw acceleration within their configured limits.
  double desired_yawdot = 0.0;
  if (std::fabs(yaw_error) > 1e-9)
  {
    const double direction = yaw_error > 0.0 ? 1.0 : -1.0;
    const double stopping_rate = std::sqrt(2.0 * YAW_DOT_DOT_MAX_PER_SEC * std::fabs(yaw_error));
    desired_yawdot = direction * std::min(YAW_DOT_MAX_PER_SEC, stopping_rate);
  }

  const double max_rate_delta = YAW_DOT_DOT_MAX_PER_SEC * dt;
  const double rate_delta = std::max(-max_rate_delta,
                                     std::min(max_rate_delta, desired_yawdot - last_yawdot_));
  const double yawdot = last_yawdot_ + rate_delta;
  const double yaw_step = 0.5 * (last_yawdot_ + yawdot) * dt;
  const double yaw = std::atan2(std::sin(last_yaw_ + yaw_step),
                                std::cos(last_yaw_ + yaw_step));
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    receive_traj_ = false;
    last_yawdot_ = 0.0;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
    return;
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, 0.01);
    /*** calculate yaw ***/

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if !FLIP_YAW_AT_END && !TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    // Keep publishing the final position so the controller can finish and hold
    // the requested goal yaw after translational motion has ended.
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();
    yaw_yawdot = calculate_yaw(traj_duration_, pos, 0.01);
    last_pos_ = pos;
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber yaw_sub = nh.subscribe("planning/yaw", 10, yawCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  double yaw_dot_max_deg_s = 360.0;
  double yaw_dot_dot_max_deg_s2 = 900.0;
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/yaw_dot_max_deg_s", yaw_dot_max_deg_s, yaw_dot_max_deg_s);
  nh.param("traj_server/yaw_dot_dot_max_deg_s2", yaw_dot_dot_max_deg_s2, yaw_dot_dot_max_deg_s2);
  nh.param("traj_server/goal_yaw_switch_distance", goal_yaw_switch_distance_, 0.5);
  if (!std::isfinite(yaw_dot_max_deg_s) || yaw_dot_max_deg_s <= 0.0)
  {
    ROS_WARN("traj_server/yaw_dot_max_deg_s must be positive; using 360 deg/s.");
    yaw_dot_max_deg_s = 360.0;
  }
  if (!std::isfinite(yaw_dot_dot_max_deg_s2) || yaw_dot_dot_max_deg_s2 <= 0.0)
  {
    ROS_WARN("traj_server/yaw_dot_dot_max_deg_s2 must be positive; using 900 deg/s^2.");
    yaw_dot_dot_max_deg_s2 = 900.0;
  }
  if (!std::isfinite(goal_yaw_switch_distance_) || goal_yaw_switch_distance_ < 0.0)
  {
    ROS_WARN("traj_server/goal_yaw_switch_distance must be non-negative; using 0.5 m.");
    goal_yaw_switch_distance_ = 0.5;
  }
  yaw_dot_max_per_sec_ = yaw_dot_max_deg_s * M_PI / 180.0;
  yaw_dot_dot_max_per_sec_ = yaw_dot_dot_max_deg_s2 * M_PI / 180.0;
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready. yaw_dot_max=%.1f deg/s yaw_dot_dot_max=%.1f deg/s^2 goal_yaw_switch_distance=%.2f m",
           yaw_dot_max_deg_s, yaw_dot_dot_max_deg_s2, goal_yaw_switch_distance_);

  ros::spin();

  return 0;
}
