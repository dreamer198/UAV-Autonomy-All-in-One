#include <sim2real_super_adapter/validation.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sim2real_planning_msgs/PlannerCapabilities.h>
#include <sim2real_planning_msgs/PlannerCommand.h>
#include <sim2real_planning_msgs/PlannerGoal.h>
#include <sim2real_planning_msgs/PlannerStatus.h>
#include <sim2real_planning_msgs/ValidateGoal.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Trigger.h>
#include <super_planner/ValidateNativeGoal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using sim2real_super_adapter::bodyVectorToWorld;
using sim2real_super_adapter::classifyNativeCommand;
using sim2real_super_adapter::classifyNativeTrajectoryId;
using sim2real_super_adapter::finite;
using sim2real_super_adapter::finiteGoalPose;
using sim2real_super_adapter::finitePoint;
using sim2real_super_adapter::finiteVector;
using sim2real_super_adapter::measuredStateIsSettled;
using sim2real_super_adapter::measuredStateSatisfiesGoal;
using sim2real_super_adapter::measurementStampIsCurrent;
using sim2real_super_adapter::NativeCommandDecision;
using sim2real_super_adapter::nativeGoalCorrelationStamp;
using sim2real_super_adapter::nativeOnlineGoalHandoffAllowed;
using sim2real_super_adapter::nativeProgressFinishesCloseGoalWithoutTrajectory;
using sim2real_super_adapter::nativeProgressFinishesReplanHold;
using sim2real_super_adapter::nativeProgressIndicatesFinished;
using sim2real_super_adapter::nativeProgressStartsReplanHold;
using sim2real_super_adapter::nativeReplanHoldPermitsMissingCommand;
using sim2real_super_adapter::nativeReplanHoldTimedOut;
using sim2real_super_adapter::NativeTrajectoryIdDecision;
using sim2real_super_adapter::nextPublicTrajectoryId;
using sim2real_super_adapter::pointInsideBodyExclusionCylinder;
using sim2real_super_adapter::quaternionNormSquared;
using sim2real_super_adapter::yawFromQuaternion;

constexpr uint8_t kSuperInit = 0U;
constexpr uint8_t kSuperWaitGoal = 1U;
constexpr uint8_t kSuperYawing = 2U;
constexpr uint8_t kSuperGenerateTrajectory = 3U;
constexpr uint8_t kSuperFollowTrajectory = 4U;
constexpr uint8_t kSuperEmergencyStop = 5U;

struct StampedBodyPose {
  ros::Time stamp;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond body_in_world = Eigen::Quaterniond::Identity();
};

struct CloudPoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  float intensity = 0.0F;
};

bool finiteNativeCommand(const quadrotor_msgs::PositionCommand &command) {
  return command.header.frame_id == "world" && !command.header.stamp.isZero() &&
         finitePoint(command.position) && finiteVector(command.velocity) &&
         finiteVector(command.acceleration) && finite(command.yaw) &&
         finite(command.yaw_dot);
}

class SuperBackendAdapter {
public:
  SuperBackendAdapter() : nh_(), pnh_("~") {
    loadParameters();
    validateParameters();

    native_goal_pub_ =
        nh_.advertise<geometry_msgs::PoseStamped>("native/goal", 2);
    native_odom_pub_ =
        nh_.advertise<nav_msgs::Odometry>("native/odom_world", 10);
    native_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("native/cloud_world", 2);
    command_pub_ =
        nh_.advertise<sim2real_planning_msgs::PlannerCommand>("command", 10);
    status_pub_ = nh_.advertise<sim2real_planning_msgs::PlannerStatus>(
        "status", 10, true);
    capabilities_pub_ =
        nh_.advertise<sim2real_planning_msgs::PlannerCapabilities>(
            "capabilities", 1, true);
    trajectory_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/planning/viz/backend/trajectory", 1000);

    goal_sub_ =
        nh_.subscribe("goal", 2, &SuperBackendAdapter::goalCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10,
                              &SuperBackendAdapter::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2,
                               &SuperBackendAdapter::cloudCallback, this);
    native_command_sub_ =
        nh_.subscribe("native/position_command", 20,
                      &SuperBackendAdapter::nativeCommandCallback, this);
    native_trajectory_sub_ =
        nh_.subscribe("native/polynomial_trajectory", 5,
                      &SuperBackendAdapter::nativeTrajectoryCallback, this);
    native_heartbeat_sub_ =
        nh_.subscribe("native/heartbeat", 10,
                      &SuperBackendAdapter::nativeHeartbeatCallback, this);
    native_progress_sub_ =
        nh_.subscribe("native/progress", 10,
                      &SuperBackendAdapter::nativeProgressCallback, this);
    native_effective_goal_sub_ =
        nh_.subscribe("native/effective_goal", 2,
                      &SuperBackendAdapter::nativeEffectiveGoalCallback, this);
    native_trajectory_viz_sub_ =
        nh_.subscribe("native/committed_trajectory_viz", 5,
                      &SuperBackendAdapter::nativeTrajectoryVizCallback, this);

    reset_client_ = nh_.serviceClient<std_srvs::Trigger>("native/reset", false);
    native_validate_client_ =
        nh_.serviceClient<super_planner::ValidateNativeGoal>(
            "native/validate_goal", false);
    validate_server_ = nh_.advertiseService(
        "validate_goal", &SuperBackendAdapter::validateGoalCallback, this);

    command_timer_ = nh_.createTimer(ros::Duration(1.0 / command_rate_),
                                     &SuperBackendAdapter::commandTimer, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_rate_),
                                    &SuperBackendAdapter::statusTimer, this);

    status_.backend_id = backend_id_;
    status_.state = sim2real_planning_msgs::PlannerStatus::STARTING;
    status_.armable = false;
    status_.reason =
        "waiting for odometry, point cloud, SUPER heartbeat, and native links";
    publishCapabilities();
    publishStatus();
  }

private:
  void loadParameters() {
    pnh_.param<std::string>("backend_id", backend_id_, "super");
    pnh_.param<std::string>("backend_namespace", backend_namespace_, "super");
    pnh_.param<std::string>("profile", profile_, "local");
    pnh_.param<std::string>("runtime_mode", runtime_mode_, "");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/localization/odom");
    pnh_.param<std::string>("cloud_topic", cloud_topic_,
                            "/localization/cloud_registered");

    pnh_.param("adapter/odom_timeout", odom_timeout_, 0.5);
    pnh_.param("adapter/cloud_timeout", cloud_timeout_, 1.0);
    pnh_.param("adapter/map_settle_time", map_settle_time_, 0.2);
    pnh_.param("adapter/heartbeat_timeout", heartbeat_timeout_, 0.25);
    pnh_.param("adapter/progress_timeout", progress_timeout_, 0.25);
    pnh_.param("adapter/native_command_timeout", native_command_timeout_, 0.08);
    pnh_.param("adapter/planning_timeout", planning_timeout_, 10.0);
    pnh_.param("adapter/settle_timeout", settle_timeout_, 10.0);
    pnh_.param("adapter/reset_timeout", reset_timeout_, 0.5);
    pnh_.param("adapter/command_rate", command_rate_, 100.0);
    pnh_.param("adapter/status_rate", status_rate_, 20.0);
    pnh_.param("adapter/settle_velocity_tolerance", settle_velocity_tolerance_,
               0.2);

    double settle_angular_velocity_tolerance_deg_s = 10.0;
    double reached_yaw_tolerance_deg = 5.0;
    double reached_yaw_rate_tolerance_deg_s = 10.0;
    pnh_.param("adapter/settle_angular_velocity_tolerance_deg_s",
               settle_angular_velocity_tolerance_deg_s, 10.0);
    pnh_.param("adapter/settle_hold_time", settle_hold_time_, 0.5);
    pnh_.param("adapter/goal_position_tolerance", goal_position_tolerance_,
               0.35);
    pnh_.param("adapter/reached_velocity_tolerance",
               reached_velocity_tolerance_, 0.2);
    pnh_.param("adapter/reached_yaw_tolerance_deg", reached_yaw_tolerance_deg,
               5.0);
    pnh_.param("adapter/reached_yaw_rate_tolerance_deg_s",
               reached_yaw_rate_tolerance_deg_s, 10.0);
    pnh_.param("adapter/reached_hold_time", reached_hold_time_, 0.5);
    pnh_.param("adapter/max_effective_goal_shift", max_effective_goal_shift_,
               3.0);
    pnh_.param("adapter/max_velocity", max_velocity_, 2.4);
    pnh_.param("adapter/max_acceleration", max_acceleration_, 3.0);
    pnh_.param("adapter/self_filter_enabled", self_filter_enabled_, true);
    pnh_.param("adapter/self_filter_radius", self_filter_radius_, 0.35);
    pnh_.param("adapter/self_filter_min_z", self_filter_min_z_, -0.20);
    pnh_.param("adapter/self_filter_max_z", self_filter_max_z_, 0.20);
    pnh_.param("adapter/self_filter_pose_tolerance",
               self_filter_pose_tolerance_, 0.10);

    const double radians_per_degree = std::acos(-1.0) / 180.0;
    settle_angular_velocity_tolerance_ =
        settle_angular_velocity_tolerance_deg_s * radians_per_degree;
    reached_yaw_tolerance_ = reached_yaw_tolerance_deg * radians_per_degree;
    reached_yaw_rate_tolerance_ =
        reached_yaw_rate_tolerance_deg_s * radians_per_degree;
  }

  void validateParameters() const {
    const char *environment_value = std::getenv("SIM2REAL_RUNTIME_MODE");
    const std::string environment_mode =
        environment_value == nullptr ? "" : environment_value;
    const std::vector<double> positive_values = {
        odom_timeout_,
        cloud_timeout_,
        heartbeat_timeout_,
        progress_timeout_,
        native_command_timeout_,
        planning_timeout_,
        settle_timeout_,
        reset_timeout_,
        command_rate_,
        status_rate_,
        settle_velocity_tolerance_,
        settle_angular_velocity_tolerance_,
        settle_hold_time_,
        goal_position_tolerance_,
        reached_velocity_tolerance_,
        reached_yaw_tolerance_,
        reached_yaw_rate_tolerance_,
        reached_hold_time_,
        max_effective_goal_shift_,
        max_velocity_,
        max_acceleration_,
        self_filter_pose_tolerance_,
    };
    const bool positive_parameters_valid =
        std::all_of(positive_values.begin(), positive_values.end(),
                    [](double value) { return finite(value) && value > 0.0; });
    if (backend_id_ != "super" || backend_namespace_ != "super" ||
        profile_ != "local" ||
        (runtime_mode_ != "simulation" && runtime_mode_ != "real") ||
        environment_mode != runtime_mode_ || odom_topic_.empty() ||
        cloud_topic_.empty() || !positive_parameters_valid ||
        command_rate_ < 80.0 || status_rate_ < 5.0 ||
        !finite(map_settle_time_) || map_settle_time_ < 0.0 ||
        (self_filter_enabled_ &&
         (!finite(self_filter_radius_) || self_filter_radius_ <= 0.0 ||
          !finite(self_filter_min_z_) || !finite(self_filter_max_z_) ||
          self_filter_min_z_ >= self_filter_max_z_))) {
      throw std::invalid_argument(
          "invalid SUPER adapter identity, runtime mode, or safety parameters");
    }
  }

  void publishCapabilities() {
    sim2real_planning_msgs::PlannerCapabilities capabilities;
    capabilities.header.stamp = ros::Time::now();
    capabilities.header.frame_id = "world";
    capabilities.api_version = "sim2real.planner/v1";
    capabilities.backend_id = backend_id_;
    capabilities.variant = "super";
    capabilities.simulation = true;
    capabilities.yaw = true;
    capabilities.cancel = true;
    capabilities.goal_validation = true;
    capabilities.rviz = true;
    capabilities.max_velocity = max_velocity_;
    capabilities.max_acceleration = max_acceleration_;
    capabilities.has_fixed_map_bounds = false;
    capabilities.map_min.x = 0.0;
    capabilities.map_min.y = 0.0;
    capabilities.map_min.z = 0.0;
    capabilities.map_max.x = 0.0;
    capabilities.map_max.y = 0.0;
    capabilities.map_max.z = 0.0;
    capabilities_pub_.publish(capabilities);
  }

  bool validateGoalContract(const sim2real_planning_msgs::PlannerGoal &goal,
                            std::string *reason) const {
    if (goal.action == sim2real_planning_msgs::PlannerGoal::CANCEL) {
      return true;
    }
    if (goal.action != sim2real_planning_msgs::PlannerGoal::PLAN) {
      if (reason != nullptr)
        *reason = "unknown goal action";
      return false;
    }
    if (goal.session_id.empty() || goal.goal_id == 0 ||
        goal.header.stamp.isZero()) {
      if (reason != nullptr) {
        *reason =
            "PLAN requires a session, non-zero goal ID, and request stamp";
      }
      return false;
    }
    return finiteGoalPose(goal.goal, goal.constrain_yaw, reason);
  }

  bool callNativeValidation(const sim2real_planning_msgs::PlannerGoal &goal,
                            geometry_msgs::PoseStamped *effective_goal,
                            std::string *reason) {
    if (!native_validate_client_.exists()) {
      if (reason != nullptr) {
        *reason = "SUPER native goal-validation service is unavailable";
      }
      return false;
    }
    super_planner::ValidateNativeGoal service;
    service.request.goal = goal.goal;
    if (!native_validate_client_.call(service)) {
      if (reason != nullptr) {
        *reason = "SUPER native goal-validation call failed";
      }
      return false;
    }
    if (!service.response.valid) {
      if (reason != nullptr) {
        *reason = service.response.reason.empty() ? "SUPER rejected the goal"
                                                  : service.response.reason;
      }
      return false;
    }
    std::string pose_reason;
    if (!finiteGoalPose(service.response.effective_goal, goal.constrain_yaw,
                        &pose_reason)) {
      if (reason != nullptr) {
        *reason = "SUPER returned an invalid effective goal: " + pose_reason;
      }
      return false;
    }
    if (service.response.effective_goal.header.stamp !=
        goal.goal.header.stamp) {
      if (reason != nullptr) {
        *reason =
            "SUPER validation response did not echo the original goal stamp";
      }
      return false;
    }
    const Eigen::Vector3d requested(goal.goal.pose.position.x,
                                    goal.goal.pose.position.y,
                                    goal.goal.pose.position.z);
    const Eigen::Vector3d effective(
        service.response.effective_goal.pose.position.x,
        service.response.effective_goal.pose.position.y,
        service.response.effective_goal.pose.position.z);
    if ((effective - requested).norm() > max_effective_goal_shift_ + 1.0e-6) {
      if (reason != nullptr) {
        *reason = "SUPER effective goal exceeds the permitted adjustment";
      }
      return false;
    }
    if (effective_goal != nullptr) {
      *effective_goal = service.response.effective_goal;
    }
    if (reason != nullptr) {
      *reason = service.response.reason.empty()
                    ? "goal is valid for the SUPER rolling map"
                    : service.response.reason;
    }
    return true;
  }

  bool validateGoalCallback(
      sim2real_planning_msgs::ValidateGoal::Request &request,
      sim2real_planning_msgs::ValidateGoal::Response &response) {
    std::string reason;
    if (!validateGoalContract(request.goal, &reason)) {
      response.valid = false;
      response.reason = reason;
      return true;
    }
    updateReadiness();
    if (!status_.odom_ready || !status_.map_ready ||
        !nativeCoreReady(ros::Time::now())) {
      response.valid = false;
      response.reason = "SUPER odometry, map, or native heartbeat is not ready";
      return true;
    }
    response.valid =
        callNativeValidation(request.goal, nullptr, &response.reason);
    return true;
  }

  bool callNativeReset(std::string *reason) {
    if (!reset_client_.waitForExistence(ros::Duration(reset_timeout_))) {
      if (reason != nullptr)
        *reason = "SUPER reset service is unavailable";
      return false;
    }
    std_srvs::Trigger service;
    if (!reset_client_.call(service)) {
      if (reason != nullptr)
        *reason = "SUPER reset service call failed";
      return false;
    }
    if (!service.response.success) {
      if (reason != nullptr) {
        *reason = service.response.message.empty() ? "SUPER reset was rejected"
                                                   : service.response.message;
      }
      return false;
    }
    return true;
  }

  void clearCurrentTrajectory() {
    pending_goal_ = false;
    goal_dispatched_ = false;
    effective_goal_acked_ = false;
    pending_native_trajectory_valid_ = false;
    accepted_native_trajectory_id_ = 0;
    accepted_public_trajectory_id_ = 0;
    trajectory_committed_ = false;
    goal_has_normal_command_ = false;
    latest_native_command_valid_ = false;
    latest_native_command_is_backup_ = false;
    native_finished_ = false;
    native_replan_hold_ = false;
    online_goal_handoff_ = false;
    close_goal_completion_pending_ = false;
    synthetic_close_goal_active_ = false;
    reached_ = false;
    planning_started_at_ = ros::Time();
    native_replan_started_at_ = ros::Time();
    accepted_trajectory_at_ = ros::Time();
    last_native_command_receipt_ = ros::Time();
    last_native_command_stamp_ = ros::Time();
    settled_since_ = ros::Time();
    reached_candidate_since_ = ros::Time();
    native_trajectory_floor_ = highest_native_trajectory_id_;
  }

  bool onlineGoalHandoffReady(const ros::Time &now) {
    if (!nativeOnlineGoalHandoffAllowed(
            has_goal_, goal_dispatched_, trajectory_committed_,
            accepted_native_trajectory_id_, goal_has_normal_command_,
            latest_native_command_valid_, native_finished_, native_replan_hold_,
            synthetic_close_goal_active_, native_progress_state_)) {
      return false;
    }
    if (last_native_command_receipt_.isZero() ||
        (now - last_native_command_receipt_).toSec() >
            native_command_timeout_) {
      return false;
    }
    updateReadiness();
    return runtimeReady(now);
  }

  void publishGoalToNative(const ros::Time &now,
                           const std::string &status_reason) {
    geometry_msgs::PoseStamped native_goal = current_goal_.goal;
    native_goal.header.frame_id = "world";
    native_goal.header.stamp = native_goal_stamp_;
    if (!current_goal_.constrain_yaw) {
      native_goal.pose.orientation = geometry_msgs::Quaternion();
    }
    native_goal_pub_.publish(native_goal);
    pending_goal_ = false;
    goal_dispatched_ = true;
    planning_started_at_ = now;
    status_.reason = status_reason;
    publishStatus();
  }

  void
  goalCallback(const sim2real_planning_msgs::PlannerGoalConstPtr &message) {
    std::string reason;
    if (!validateGoalContract(*message, &reason)) {
      setFault("rejected goal: " + reason, true);
      return;
    }

    if (message->action == sim2real_planning_msgs::PlannerGoal::CANCEL) {
      if (has_goal_ && (message->session_id != current_goal_.session_id ||
                        (message->goal_id != 0 &&
                         message->goal_id != current_goal_.goal_id))) {
        ROS_WARN("ignoring stale SUPER cancel request");
        return;
      }
      if (!callNativeReset(&reason)) {
        setFault("failed to cancel SUPER: " + reason, false);
        return;
      }
      clearCurrentTrajectory();
      has_goal_ = false;
      status_.session_id = message->session_id;
      status_.goal_id = message->goal_id;
      status_.trajectory_id = 0;
      status_.state = sim2real_planning_msgs::PlannerStatus::HOLDING;
      status_.armable = false;
      status_.reason = "goal cancelled and SUPER state reset";
      publishStatus();
      return;
    }

    if (!callNativeValidation(*message, nullptr, &reason)) {
      setFault("rejected goal: " + reason, true);
      return;
    }
    const ros::Time now = ros::Time::now();
    const bool use_online_handoff = onlineGoalHandoffReady(now);
    if (!use_online_handoff && !callNativeReset(&reason)) {
      setFault("failed to reset SUPER before planning: " + reason, false);
      return;
    }

    clearCurrentTrajectory();
    current_goal_ = *message;
    has_goal_ = true;
    pending_goal_ = !use_online_handoff;
    online_goal_handoff_ = use_online_handoff;
    native_goal_stamp_ = nativeGoalCorrelationStamp(*message);
    goal_received_at_ = now;
    status_.session_id = message->session_id;
    status_.goal_id = message->goal_id;
    status_.trajectory_id = 0;
    status_.active_goal = message->goal;
    status_.state = sim2real_planning_msgs::PlannerStatus::PLANNING;
    status_.armable = false;
    if (use_online_handoff) {
      publishGoalToNative(
          now, "SUPER accepted an in-flight goal handoff; waiting for a newer "
               "continuous trajectory");
    } else {
      status_.reason =
          "SUPER reset complete; waiting for measured vehicle settle";
      publishStatus();
      dispatchPendingGoalIfReady();
    }
  }

  void dispatchPendingGoalIfReady() {
    if (!pending_goal_ || !has_goal_)
      return;
    const ros::Time now = ros::Time::now();
    updateReadiness();
    if (!runtimeReady(now) || !latest_odom_valid_) {
      settled_since_ = ros::Time();
      return;
    }
    if (!measuredStateIsSettled(
            latest_odom_world_velocity_, latest_odom_world_angular_velocity_,
            settle_velocity_tolerance_, settle_angular_velocity_tolerance_)) {
      settled_since_ = ros::Time();
      status_.reason =
          "waiting for measured linear and angular velocity to settle";
      return;
    }
    if (settled_since_.isZero() || now < settled_since_) {
      settled_since_ = now;
      status_.reason = "measured vehicle settle is stabilizing";
      return;
    }
    if ((now - settled_since_).toSec() < settle_hold_time_)
      return;

    publishGoalToNative(
        now,
        "SUPER is computing a trajectory and effective-goal acknowledgement");
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &message) {
    const auto &position = message->pose.pose.position;
    const auto &orientation = message->pose.pose.orientation;
    const auto &linear = message->twist.twist.linear;
    const auto &angular = message->twist.twist.angular;
    const ros::Time now = ros::Time::now();
    const double norm_squared = quaternionNormSquared(orientation);
    if (!measurementStampIsCurrent(message->header.stamp, now, odom_timeout_,
                                   last_odom_measurement_) ||
        message->header.frame_id != "world" ||
        message->child_frame_id != "base_link" || !finitePoint(position) ||
        !finiteVector(linear) || !finiteVector(angular) ||
        !finite(norm_squared) || norm_squared < 0.9801 ||
        norm_squared > 1.0201) {
      latest_odom_valid_ = false;
      last_odom_receipt_ = ros::Time();
      odom_history_.clear();
      setFault("SUPER odometry contract violation", true);
      return;
    }

    const double norm = std::sqrt(norm_squared);
    const Eigen::Quaterniond body_in_world(
        orientation.w / norm, orientation.x / norm, orientation.y / norm,
        orientation.z / norm);
    const Eigen::Vector3d world_linear = bodyVectorToWorld(
        body_in_world, Eigen::Vector3d(linear.x, linear.y, linear.z));
    const Eigen::Vector3d world_angular = bodyVectorToWorld(
        body_in_world, Eigen::Vector3d(angular.x, angular.y, angular.z));

    nav_msgs::Odometry native = *message;
    native.child_frame_id = "world";
    native.pose.pose.orientation.w = body_in_world.w();
    native.pose.pose.orientation.x = body_in_world.x();
    native.pose.pose.orientation.y = body_in_world.y();
    native.pose.pose.orientation.z = body_in_world.z();
    native.twist.twist.linear.x = world_linear.x();
    native.twist.twist.linear.y = world_linear.y();
    native.twist.twist.linear.z = world_linear.z();
    native.twist.twist.angular.x = world_angular.x();
    native.twist.twist.angular.y = world_angular.y();
    native.twist.twist.angular.z = world_angular.z();
    native_odom_pub_.publish(native);

    latest_odom_position_ = Eigen::Vector3d(position.x, position.y, position.z);
    latest_odom_body_in_world_ = body_in_world;
    latest_odom_world_velocity_ = world_linear;
    latest_odom_world_angular_velocity_ = world_angular;
    latest_odom_valid_ = true;
    last_odom_measurement_ = message->header.stamp;
    last_odom_receipt_ = now;

    StampedBodyPose body_pose;
    body_pose.stamp = message->header.stamp;
    body_pose.position = latest_odom_position_;
    body_pose.body_in_world = latest_odom_body_in_world_;
    odom_history_.push_back(body_pose);
    const double history_duration = std::max(odom_timeout_, cloud_timeout_) +
                                    self_filter_pose_tolerance_ + 0.1;
    while (!odom_history_.empty() &&
           (message->header.stamp - odom_history_.front().stamp).toSec() >
               history_duration) {
      odom_history_.pop_front();
    }
  }

  bool bodyPoseAtMeasurement(const ros::Time &stamp, Eigen::Vector3d *position,
                             Eigen::Quaterniond *body_in_world) const {
    if (position == nullptr || body_in_world == nullptr || stamp.isZero() ||
        odom_history_.empty()) {
      return false;
    }
    const StampedBodyPose *nearest = nullptr;
    double nearest_delta = std::numeric_limits<double>::infinity();
    for (const StampedBodyPose &pose : odom_history_) {
      const double delta = std::fabs((pose.stamp - stamp).toSec());
      if (delta < nearest_delta) {
        nearest = &pose;
        nearest_delta = delta;
      }
    }
    if (nearest == nullptr || !finite(nearest_delta) ||
        nearest_delta > self_filter_pose_tolerance_) {
      return false;
    }
    *position = nearest->position;
    *body_in_world = nearest->body_in_world;
    return true;
  }

  bool extractCloud(const sensor_msgs::PointCloud2 &message,
                    const Eigen::Vector3d &body_position,
                    const Eigen::Quaterniond &body_in_world,
                    std::vector<CloudPoint> *points,
                    size_t *removed_self_points, std::string *reason) const {
    if (points == nullptr || removed_self_points == nullptr)
      return false;
    const uint64_t count = static_cast<uint64_t>(message.width) *
                           static_cast<uint64_t>(message.height);
    if (count > static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
      if (reason != nullptr) {
        *reason = "SUPER input point count exceeds address space";
      }
      return false;
    }

    bool float_intensity = false;
    for (const sensor_msgs::PointField &field : message.fields) {
      if (field.name == "intensity") {
        float_intensity = field.datatype == sensor_msgs::PointField::FLOAT32 &&
                          field.count == 1U;
      }
    }

    points->clear();
    points->reserve(static_cast<size_t>(count));
    *removed_self_points = 0U;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> input_x(message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_y(message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_z(message, "z");
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>>
          input_intensity;
      if (float_intensity) {
        input_intensity.reset(new sensor_msgs::PointCloud2ConstIterator<float>(
            message, "intensity"));
      }
      for (uint64_t i = 0; i < count; ++i, ++input_x, ++input_y, ++input_z) {
        const Eigen::Vector3d position(*input_x, *input_y, *input_z);
        float intensity = 0.0F;
        if (input_intensity != nullptr) {
          intensity = **input_intensity;
          ++(*input_intensity);
        }
        if (!position.allFinite() || !std::isfinite(intensity))
          continue;
        if (self_filter_enabled_ &&
            pointInsideBodyExclusionCylinder(
                position, body_position, body_in_world, self_filter_radius_,
                self_filter_min_z_, self_filter_max_z_)) {
          ++(*removed_self_points);
          continue;
        }
        CloudPoint point;
        point.position = position;
        point.intensity = intensity;
        points->push_back(point);
      }
    } catch (const std::runtime_error &error) {
      if (reason != nullptr) {
        *reason = std::string("SUPER point cloud lacks float32 xyz fields: ") +
                  error.what();
      }
      return false;
    }
    if (points->empty()) {
      if (reason != nullptr) {
        *reason = "SUPER point cloud contains no finite external points";
      }
      return false;
    }
    return true;
  }

  sensor_msgs::PointCloud2
  makeXyziCloud(const std_msgs::Header &header,
                const std::vector<CloudPoint> &points) const {
    sensor_msgs::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                  1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1,
                                  sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> output_intensity(output,
                                                             "intensity");
    for (const CloudPoint &point : points) {
      *output_x = static_cast<float>(point.position.x());
      *output_y = static_cast<float>(point.position.y());
      *output_z = static_cast<float>(point.position.z());
      *output_intensity = point.intensity;
      ++output_x;
      ++output_y;
      ++output_z;
      ++output_intensity;
    }
    output.is_dense = true;
    return output;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &message) {
    const ros::Time now = ros::Time::now();
    const uint64_t minimum_size = static_cast<uint64_t>(message->row_step) *
                                  static_cast<uint64_t>(message->height);
    if (!measurementStampIsCurrent(message->header.stamp, now, cloud_timeout_,
                                   last_cloud_measurement_) ||
        message->header.frame_id != "world" || message->width == 0U ||
        message->height == 0U || message->point_step == 0U ||
        message->row_step == 0U || minimum_size > message->data.size()) {
      latest_cloud_valid_ = false;
      last_cloud_receipt_ = ros::Time();
      setFault("SUPER point-cloud contract violation", true);
      return;
    }
    if (!latest_odom_valid_) {
      latest_cloud_valid_ = false;
      last_cloud_receipt_ = ros::Time();
      setFault("cannot adapt SUPER point cloud without valid odometry", true);
      return;
    }

    Eigen::Vector3d body_position;
    Eigen::Quaterniond body_in_world;
    if (!bodyPoseAtMeasurement(message->header.stamp, &body_position,
                               &body_in_world)) {
      latest_cloud_valid_ = false;
      last_cloud_receipt_ = ros::Time();
      setFault("cannot time-align SUPER self-filter with odometry", true);
      return;
    }

    std::vector<CloudPoint> points;
    size_t removed_self_points = 0U;
    std::string reason;
    if (!extractCloud(*message, body_position, body_in_world, &points,
                      &removed_self_points, &reason)) {
      latest_cloud_valid_ = false;
      last_cloud_receipt_ = ros::Time();
      setFault(reason, true);
      return;
    }
    if (removed_self_points > 0U) {
      ROS_WARN_THROTTLE(5.0, "SUPER airframe self-filter removed %zu point(s)",
                        removed_self_points);
    }

    native_cloud_pub_.publish(makeXyziCloud(message->header, points));
    latest_cloud_valid_ = true;
    last_cloud_measurement_ = message->header.stamp;
    last_cloud_receipt_ = now;
    if (first_cloud_receipt_.isZero())
      first_cloud_receipt_ = now;
  }

  void nativeHeartbeatCallback(const std_msgs::EmptyConstPtr &) {
    last_heartbeat_receipt_ = ros::Time::now();
  }

  void nativeTrajectoryVizCallback(
      const visualization_msgs::MarkerArrayConstPtr &message) {
    for (visualization_msgs::Marker marker : message->markers) {
      if (marker.header.frame_id.empty())
        marker.header.frame_id = "world";
      trajectory_viz_pub_.publish(marker);
    }
  }

  void nativeProgressCallback(const std_msgs::UInt8ConstPtr &message) {
    if (message->data > kSuperEmergencyStop) {
      setFault("SUPER reported an invalid FSM progress state", true);
      return;
    }
    const uint8_t previous = native_progress_state_;
    native_progress_state_ = message->data;
    last_progress_receipt_ = ros::Time::now();

    const bool busy = native_progress_state_ == kSuperYawing ||
                      native_progress_state_ == kSuperGenerateTrajectory;
    if (busy && busy_progress_started_at_.isZero()) {
      busy_progress_started_at_ = last_progress_receipt_;
    } else if (!busy) {
      busy_progress_started_at_ = ros::Time();
    }

    const bool follow_to_generate =
        previous == kSuperFollowTrajectory &&
        native_progress_state_ == kSuperGenerateTrajectory;
    if (follow_to_generate) {
      if (online_goal_handoff_ && goal_dispatched_ && !trajectory_committed_) {
        status_.state = sim2real_planning_msgs::PlannerStatus::PLANNING;
        status_.armable = false;
        status_.reason =
            "SUPER in-flight goal handoff reached a native trajectory "
            "boundary; waiting for its replacement";
        publishStatus();
      } else if (!nativeProgressStartsReplanHold(
                     previous, native_progress_state_, trajectory_committed_,
                     goal_has_normal_command_, latest_native_command_valid_)) {
        setFault("SUPER entered replanning without a safe normal-command "
                 "endpoint",
                 true);
        return;
      } else {
        native_replan_hold_ = true;
        native_replan_started_at_ = last_progress_receipt_;
        hold_position_ = Eigen::Vector3d(latest_native_command_.position.x,
                                         latest_native_command_.position.y,
                                         latest_native_command_.position.z);
        hold_yaw_ = latest_native_command_.yaw;
        reached_candidate_since_ = ros::Time();
        status_.state = sim2real_planning_msgs::PlannerStatus::HOLDING;
        status_.armable = false;
        status_.reason =
            "SUPER trajectory endpoint is holding during bounded replanning";
        publishStatus();
      }
    }

    if (nativeProgressFinishesReplanHold(previous, native_progress_state_,
                                         native_replan_hold_)) {
      const Eigen::Vector3d effective_goal_position(
          effective_goal_.pose.position.x, effective_goal_.pose.position.y,
          effective_goal_.pose.position.z);
      if (!latest_odom_valid_ || !effective_goal_acked_ ||
          (latest_odom_position_ - effective_goal_position).norm() >
              goal_position_tolerance_ + 1.0e-6) {
        setFault("SUPER replanning ended without a newer trajectory while "
                 "outside the endpoint tolerance",
                 true);
        return;
      }
      native_replan_hold_ = false;
      native_replan_started_at_ = ros::Time();
      native_finished_ = true;
      status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
      status_.armable = true;
      status_.reason =
          "SUPER replanning reached the endpoint; its authorized hold is "
          "converging to measured arrival";
      publishStatus();
    }

    if (nativeProgressFinishesCloseGoalWithoutTrajectory(
            previous, native_progress_state_, goal_dispatched_,
            trajectory_committed_)) {
      close_goal_completion_pending_ = true;
      acceptCloseGoalWithoutNativeTrajectoryIfReady();
      if (!has_goal_)
        return;
    }

    if (nativeProgressIndicatesFinished(
            previous, native_progress_state_, trajectory_committed_,
            goal_has_normal_command_, latest_native_command_valid_)) {
      native_finished_ = true;
      hold_position_ = Eigen::Vector3d(latest_native_command_.position.x,
                                       latest_native_command_.position.y,
                                       latest_native_command_.position.z);
      hold_yaw_ = latest_native_command_.yaw;
      status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
      status_.armable = true;
      status_.reason =
          "SUPER trajectory finished; its authorized endpoint hold is "
          "converging to measured arrival";
      publishStatus();
    }
  }

  bool validateNativeEffectiveGoal(const geometry_msgs::PoseStamped &goal,
                                   std::string *reason) const {
    if (!finiteGoalPose(goal, current_goal_.constrain_yaw, reason)) {
      return false;
    }
    if (goal.header.stamp != native_goal_stamp_) {
      if (reason != nullptr) {
        *reason = "effective-goal request stamp does not match";
      }
      return false;
    }
    const Eigen::Vector3d requested(current_goal_.goal.pose.position.x,
                                    current_goal_.goal.pose.position.y,
                                    current_goal_.goal.pose.position.z);
    const Eigen::Vector3d effective(goal.pose.position.x, goal.pose.position.y,
                                    goal.pose.position.z);
    if ((effective - requested).norm() > max_effective_goal_shift_ + 1.0e-6) {
      if (reason != nullptr) {
        *reason = "effective goal exceeds the permitted adjustment";
      }
      return false;
    }
    return true;
  }

  void nativeEffectiveGoalCallback(
      const geometry_msgs::PoseStampedConstPtr &message) {
    if (!has_goal_ || !goal_dispatched_) {
      ROS_WARN("dropping SUPER effective goal without a dispatched goal");
      return;
    }
    if (message->header.stamp != native_goal_stamp_) {
      ROS_WARN("dropping stale SUPER effective goal acknowledgement");
      return;
    }
    std::string reason;
    if (!validateNativeEffectiveGoal(*message, &reason)) {
      setFault("rejected SUPER effective goal: " + reason, true);
      return;
    }
    effective_goal_ = *message;
    effective_goal_acked_ = true;
    status_.active_goal = effective_goal_;
    status_.reason =
        "SUPER effective goal acknowledged; waiting for committed trajectory";
    publishStatus();

    if (pending_native_trajectory_valid_) {
      const quadrotor_msgs::PolynomialTrajectory pending =
          pending_native_trajectory_;
      pending_native_trajectory_valid_ = false;
      if (!validateNativeTrajectory(pending, &reason)) {
        setFault("stale pending SUPER trajectory after goal acknowledgement: " +
                     reason,
                 true);
        return;
      }
      acceptNativeTrajectory(pending);
    }
    acceptCloseGoalWithoutNativeTrajectoryIfReady();
  }

  uint64_t allocatePublicTrajectoryId(uint64_t preferred_native_id) {
    // A close-enough goal has no native polynomial or native ID. Keep the
    // public sequence independent so that this synthetic stationary command
    // cannot collide with SUPER's next real native trajectory.
    const uint64_t public_id = nextPublicTrajectoryId(
        highest_public_trajectory_id_, preferred_native_id);
    if (public_id == 0U)
      return 0U;
    highest_public_trajectory_id_ = public_id;
    accepted_public_trajectory_id_ = public_id;
    return public_id;
  }

  void acceptCloseGoalWithoutNativeTrajectoryIfReady() {
    if (!close_goal_completion_pending_ || !has_goal_ || !goal_dispatched_ ||
        !effective_goal_acked_ || trajectory_committed_) {
      return;
    }
    if (!latest_odom_valid_) {
      setFault("SUPER completed a close goal without valid measured state",
               true);
      return;
    }
    const Eigen::Vector3d effective_goal_position(
        effective_goal_.pose.position.x, effective_goal_.pose.position.y,
        effective_goal_.pose.position.z);
    if ((latest_odom_position_ - effective_goal_position).norm() >
        goal_position_tolerance_ + 1.0e-6) {
      setFault("SUPER completed without a trajectory while outside the "
               "measured goal tolerance",
               true);
      return;
    }
    const uint64_t public_id = allocatePublicTrajectoryId(0U);
    if (public_id == 0U) {
      setFault("SUPER public trajectory ID space is exhausted", true);
      return;
    }

    hold_position_ = Eigen::Vector3d(effective_goal_.pose.position.x,
                                     effective_goal_.pose.position.y,
                                     effective_goal_.pose.position.z);
    if (current_goal_.constrain_yaw) {
      const auto &q = effective_goal_.pose.orientation;
      hold_yaw_ = yawFromQuaternion(Eigen::Quaterniond(q.w, q.x, q.y, q.z));
    } else {
      hold_yaw_ = yawFromQuaternion(latest_odom_body_in_world_);
    }

    accepted_native_trajectory_id_ = 0U;
    trajectory_committed_ = true;
    close_goal_completion_pending_ = false;
    synthetic_close_goal_active_ = true;
    latest_native_command_valid_ = false;
    latest_native_command_is_backup_ = false;
    native_finished_ = true;
    native_replan_hold_ = false;
    online_goal_handoff_ = false;
    reached_ = false;
    reached_candidate_since_ = ros::Time();
    planning_started_at_ = ros::Time();
    native_replan_started_at_ = ros::Time();
    accepted_trajectory_at_ = ros::Time::now();
    last_native_command_receipt_ = ros::Time();
    last_native_command_stamp_ = ros::Time();

    status_.trajectory_id = public_id;
    status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
    status_.armable = true;
    status_.reason =
        "SUPER goal is already within its close threshold; stabilizing at "
        "the effective goal";
    publishStatus();
  }

  bool validateNativeTrajectory(
      const quadrotor_msgs::PolynomialTrajectory &trajectory,
      std::string *reason) const {
    const uint32_t supported_types =
        quadrotor_msgs::PolynomialTrajectory::HEART_BEAT |
        quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ |
        quadrotor_msgs::PolynomialTrajectory::YAW_TRAJ;
    const double header_age =
        (ros::Time::now() - trajectory.header.stamp).toSec();
    if (trajectory.header.frame_id != "world" ||
        trajectory.header.stamp.isZero() || trajectory.trajectory_id == 0U ||
        !finite(header_age) || std::fabs(header_age) > progress_timeout_ ||
        (trajectory.type & ~supported_types) != 0U ||
        !(trajectory.type &
          quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ) ||
        trajectory.start_WT_pos.isZero() || trajectory.piece_num_pos == 0U ||
        trajectory.order_pos != 7U ||
        trajectory.time_pos.size() != trajectory.piece_num_pos) {
      if (reason != nullptr) {
        *reason = "invalid frame, ID, type, timing, or position dimensions";
      }
      return false;
    }
    const size_t position_coefficients =
        static_cast<size_t>(trajectory.piece_num_pos) *
        (static_cast<size_t>(trajectory.order_pos) + 1U);
    if (trajectory.coef_pos_x.size() != position_coefficients ||
        trajectory.coef_pos_y.size() != position_coefficients ||
        trajectory.coef_pos_z.size() != position_coefficients) {
      if (reason != nullptr) {
        *reason = "invalid position polynomial coefficient dimensions";
      }
      return false;
    }
    for (double duration : trajectory.time_pos) {
      if (!finite(duration) || duration <= 0.0) {
        if (reason != nullptr) {
          *reason = "position polynomial duration must be finite and positive";
        }
        return false;
      }
    }
    const auto coefficients_are_finite =
        [](const std::vector<double> &coefficients) {
          return std::all_of(coefficients.begin(), coefficients.end(),
                             [](double value) { return finite(value); });
        };
    if (!coefficients_are_finite(trajectory.coef_pos_x) ||
        !coefficients_are_finite(trajectory.coef_pos_y) ||
        !coefficients_are_finite(trajectory.coef_pos_z)) {
      if (reason != nullptr) {
        *reason = "position polynomial contains non-finite coefficients";
      }
      return false;
    }
    if (trajectory.type & quadrotor_msgs::PolynomialTrajectory::YAW_TRAJ) {
      if (trajectory.start_WT_yaw.isZero() || trajectory.piece_num_yaw == 0U ||
          trajectory.order_yaw != 7U ||
          trajectory.time_yaw.size() != trajectory.piece_num_yaw ||
          trajectory.coef_yaw.size() !=
              static_cast<size_t>(trajectory.piece_num_yaw) *
                  (static_cast<size_t>(trajectory.order_yaw) + 1U) ||
          !coefficients_are_finite(trajectory.coef_yaw)) {
        if (reason != nullptr) {
          *reason = "invalid yaw polynomial dimensions or coefficients";
        }
        return false;
      }
      for (double duration : trajectory.time_yaw) {
        if (!finite(duration) || duration <= 0.0) {
          if (reason != nullptr) {
            *reason = "yaw polynomial duration must be finite and positive";
          }
          return false;
        }
      }
    }
    return true;
  }

  void nativeTrajectoryCallback(
      const quadrotor_msgs::PolynomialTrajectoryConstPtr &message) {
    if (!(message->type &
          quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ)) {
      return;
    }
    if (!has_goal_ || !goal_dispatched_) {
      ROS_WARN("dropping SUPER trajectory without a dispatched goal");
      return;
    }
    std::string reason;
    if (!validateNativeTrajectory(*message, &reason)) {
      setFault("rejected SUPER polynomial trajectory: " + reason, true);
      return;
    }
    const uint64_t native_id = message->trajectory_id;
    if (!trajectory_committed_ && native_id <= native_trajectory_floor_) {
      ROS_WARN_THROTTLE(
          1.0,
          "dropping queued SUPER trajectory from the preceding public goal");
      return;
    }
    const NativeTrajectoryIdDecision id_decision =
        classifyNativeTrajectoryId(native_id, highest_native_trajectory_id_,
                                   accepted_native_trajectory_id_);
    if (id_decision == NativeTrajectoryIdDecision::FAULT_BACKWARDS_OR_REPLAY) {
      setFault("SUPER native trajectory ID moved backwards or replayed", true);
      return;
    }
    if (id_decision == NativeTrajectoryIdDecision::IGNORE_DUPLICATE) {
      // SUPER may republish the same committed trajectory when ReplanOnce
      // returns NO_NEED. It is a duplicate, not a replacement trajectory.
      return;
    }
    if (!effective_goal_acked_) {
      pending_native_trajectory_ = *message;
      pending_native_trajectory_valid_ = true;
      return;
    }
    acceptNativeTrajectory(*message);
  }

  void acceptNativeTrajectory(
      const quadrotor_msgs::PolynomialTrajectory &trajectory) {
    const uint64_t native_id = trajectory.trajectory_id;
    if (native_id <= highest_native_trajectory_id_) {
      setFault("SUPER trajectory was not newer when acknowledgement completed",
               true);
      return;
    }
    const uint64_t public_id = allocatePublicTrajectoryId(native_id);
    if (public_id == 0U) {
      setFault("SUPER public trajectory ID space is exhausted", true);
      return;
    }
    highest_native_trajectory_id_ = native_id;
    accepted_native_trajectory_id_ = native_id;
    trajectory_committed_ = true;
    close_goal_completion_pending_ = false;
    synthetic_close_goal_active_ = false;
    // NORMAL authorization is scoped to the goal, not one rolling replan.
    // SUPER may commit a replacement whose first sample is already on its
    // backup segment. That is a valid BRAKE only after an earlier NORMAL
    // command from the same goal; clearCurrentTrajectory() still rejects a
    // backup as the first command of a new or reset goal.
    latest_native_command_valid_ = false;
    latest_native_command_is_backup_ = false;
    native_finished_ = false;
    native_replan_hold_ = false;
    online_goal_handoff_ = false;
    reached_ = false;
    reached_candidate_since_ = ros::Time();
    planning_started_at_ = ros::Time();
    native_replan_started_at_ = ros::Time();
    accepted_trajectory_at_ = ros::Time::now();
    last_native_command_receipt_ = ros::Time();
    last_native_command_stamp_ = ros::Time();

    status_.trajectory_id = public_id;
    status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
    status_.armable = true;
    status_.reason =
        "SUPER committed trajectory; waiting for its first normal command";
    publishStatus();
  }

  void nativeCommandCallback(
      const quadrotor_msgs::PositionCommandConstPtr &message) {
    if (!trajectory_committed_ || !has_goal_)
      return;
    if (synthetic_close_goal_active_ || accepted_native_trajectory_id_ == 0U) {
      return;
    }
    if (message->trajectory_id != accepted_native_trajectory_id_) {
      ROS_WARN_THROTTLE(
          1.0, "dropping SUPER command whose trajectory ID is not committed");
      return;
    }
    const ros::Time now = ros::Time::now();
    const double age = (now - message->header.stamp).toSec();
    if (!finiteNativeCommand(*message)) {
      setFault("SUPER native position command has an invalid frame, stamp, or "
               "non-finite field",
               true);
      return;
    }
    if (!finite(age) || std::fabs(age) > native_command_timeout_) {
      setFault("SUPER native position-command timestamp is stale or too far in "
               "the future",
               true);
      return;
    }
    if (!last_native_command_stamp_.isZero() &&
        message->header.stamp <= last_native_command_stamp_) {
      setFault("SUPER native position-command timestamp did not increase",
               true);
      return;
    }
    const NativeCommandDecision command_decision = classifyNativeCommand(
        message->trajectory_flag, goal_has_normal_command_);
    if (command_decision ==
        NativeCommandDecision::REJECT_INVALID_OR_UNAUTHORIZED) {
      setFault("SUPER native command flag is invalid or backup arrived before "
               "NORMAL",
               true);
      return;
    }
    if (native_replan_hold_) {
      // The progress transition closed this native trajectory. A queued
      // command from that ID may still arrive after the transition, but it
      // must never reactivate NORMAL output. Only a newer committed
      // polynomial trajectory may leave the replan HOLD.
      return;
    }

    const bool backup = command_decision == NativeCommandDecision::BRAKE;
    if (!backup)
      goal_has_normal_command_ = true;

    const bool remain_finished =
        native_finished_ && native_progress_state_ == kSuperWaitGoal && !backup;
    latest_native_command_ = *message;
    latest_native_command_valid_ = true;
    latest_native_command_is_backup_ = backup;
    native_finished_ = remain_finished;
    last_native_command_stamp_ = message->header.stamp;
    last_native_command_receipt_ = now;

    if (remain_finished) {
      hold_position_ = Eigen::Vector3d(message->position.x, message->position.y,
                                       message->position.z);
      hold_yaw_ = message->yaw;
      status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
      status_.armable = true;
      status_.reason =
          "SUPER trajectory finished; its authorized endpoint hold is "
          "converging to measured arrival";
    } else {
      // A backup segment is an authorized part of the already-open motion
      // trajectory and SUPER continues replanning it automatically. Reporting
      // it as a stopped planner causes mission clients to resubmit the goal,
      // close the gateway, and interrupt the safe brake. Keep the lifecycle
      // ACTIVE while preserving BRAKE as the command mode.
      status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
      status_.armable = true;
      status_.reason = backup
                           ? "SUPER backup trajectory is safely braking while "
                             "automatic replanning remains active"
                           : "SUPER trajectory active";
    }
  }

  bool nativeCoreReady(const ros::Time &now) const {
    return !last_heartbeat_receipt_.isZero() &&
           !last_progress_receipt_.isZero() &&
           (now - last_heartbeat_receipt_).toSec() <= heartbeat_timeout_ &&
           (now - last_progress_receipt_).toSec() <= progress_timeout_;
  }

  bool nativeLinksReady() {
    return native_goal_pub_.getNumSubscribers() > 0U &&
           native_command_sub_.getNumPublishers() > 0U &&
           native_trajectory_sub_.getNumPublishers() > 0U &&
           native_heartbeat_sub_.getNumPublishers() > 0U &&
           native_progress_sub_.getNumPublishers() > 0U &&
           native_effective_goal_sub_.getNumPublishers() > 0U &&
           reset_client_.exists() && native_validate_client_.exists();
  }

  void updateReadiness() {
    const ros::Time now = ros::Time::now();
    status_.odom_ready = latest_odom_valid_ && !last_odom_receipt_.isZero() &&
                         (now - last_odom_receipt_).toSec() <= odom_timeout_;
    status_.map_ready =
        latest_cloud_valid_ && !last_cloud_receipt_.isZero() &&
        !first_cloud_receipt_.isZero() &&
        (now - last_cloud_receipt_).toSec() <= cloud_timeout_ &&
        (now - first_cloud_receipt_).toSec() >= map_settle_time_;
  }

  bool runtimeReady(const ros::Time &now) {
    return status_.odom_ready && status_.map_ready && nativeCoreReady(now) &&
           nativeLinksReady();
  }

  void commandTimer(const ros::TimerEvent &) {
    if (!has_goal_ || !trajectory_committed_ || !effective_goal_acked_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    updateReadiness();
    if (!status_.odom_ready || !status_.map_ready || !nativeCoreReady(now)) {
      return;
    }

    const bool synthetic_close_goal = synthetic_close_goal_active_;
    bool publish_hold = (native_finished_ && !synthetic_close_goal) ||
                        native_replan_hold_ || reached_;
    if (!publish_hold && !synthetic_close_goal) {
      if (!latest_native_command_valid_ ||
          last_native_command_receipt_.isZero()) {
        if (!accepted_trajectory_at_.isZero() &&
            (now - accepted_trajectory_at_).toSec() > native_command_timeout_) {
          setFault("SUPER committed trajectory produced no timely command",
                   true);
        }
        return;
      }
      if ((now - last_native_command_receipt_).toSec() >
          native_command_timeout_) {
        setFault("SUPER native position-command stream became stale", true);
        return;
      }
    }

    Eigen::Vector3d position;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_rate = 0.0;
    uint8_t mode = sim2real_planning_msgs::PlannerCommand::HOLD;

    if (synthetic_close_goal) {
      // The gateway intentionally refuses to open a new goal with HOLD.
      // Publish a stationary NORMAL setpoint until measured arrival is stable;
      // subsequent samples switch to HOLD under the same public trajectory ID.
      position = hold_position_;
      yaw = hold_yaw_;
      mode = sim2real_planning_msgs::PlannerCommand::NORMAL;
    } else if (publish_hold) {
      position = hold_position_;
      yaw = hold_yaw_;
    } else {
      position = Eigen::Vector3d(latest_native_command_.position.x,
                                 latest_native_command_.position.y,
                                 latest_native_command_.position.z);
      velocity = Eigen::Vector3d(latest_native_command_.velocity.x,
                                 latest_native_command_.velocity.y,
                                 latest_native_command_.velocity.z);
      acceleration = Eigen::Vector3d(latest_native_command_.acceleration.x,
                                     latest_native_command_.acceleration.y,
                                     latest_native_command_.acceleration.z);
      yaw = latest_native_command_.yaw;
      yaw_rate = latest_native_command_.yaw_dot;
      mode = latest_native_command_is_backup_
                 ? sim2real_planning_msgs::PlannerCommand::BRAKE
                 : sim2real_planning_msgs::PlannerCommand::NORMAL;
    }
    if (!position.allFinite() || !velocity.allFinite() ||
        !acceleration.allFinite() || !finite(yaw) || !finite(yaw_rate)) {
      setFault("SUPER command conversion produced non-finite values", true);
      return;
    }

    sim2real_planning_msgs::PlannerCommand command;
    command.header.stamp = now;
    command.header.frame_id = "world";
    command.session_id = current_goal_.session_id;
    command.backend_id = backend_id_;
    command.goal_id = current_goal_.goal_id;
    command.trajectory_id = accepted_public_trajectory_id_;
    command.mode = mode;

    geometry_msgs::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform.rotation.z = std::sin(0.5 * yaw);
    transform.rotation.w = std::cos(0.5 * yaw);
    command.point.transforms.push_back(transform);

    geometry_msgs::Twist velocity_message;
    velocity_message.linear.x = velocity.x();
    velocity_message.linear.y = velocity.y();
    velocity_message.linear.z = velocity.z();
    velocity_message.angular.z = yaw_rate;
    command.point.velocities.push_back(velocity_message);

    geometry_msgs::Twist acceleration_message;
    acceleration_message.linear.x = acceleration.x();
    acceleration_message.linear.y = acceleration.y();
    acceleration_message.linear.z = acceleration.z();
    command.point.accelerations.push_back(acceleration_message);
    command.point.time_from_start =
        ros::Duration(std::max(0.0, (now - accepted_trajectory_at_).toSec()));
    command_pub_.publish(command);

    if (native_finished_ && !reached_) {
      const bool arrived =
          latest_odom_valid_ &&
          measuredStateSatisfiesGoal(
              latest_odom_position_, latest_odom_world_velocity_,
              latest_odom_body_in_world_, latest_odom_world_angular_velocity_,
              effective_goal_, current_goal_.constrain_yaw,
              goal_position_tolerance_, reached_velocity_tolerance_,
              reached_yaw_tolerance_, reached_yaw_rate_tolerance_);
      if (!arrived) {
        reached_candidate_since_ = ros::Time();
        if (synthetic_close_goal_active_) {
          status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
          status_.armable = true;
          status_.reason =
              "SUPER close-goal setpoint is converging to measured arrival";
        } else {
          status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
          status_.armable = true;
          status_.reason =
              "SUPER authorized endpoint hold is converging to measured "
              "arrival";
        }
      } else {
        if (reached_candidate_since_.isZero() ||
            now < reached_candidate_since_) {
          reached_candidate_since_ = now;
        }
        if ((now - reached_candidate_since_).toSec() >= reached_hold_time_) {
          synthetic_close_goal_active_ = false;
          reached_ = true;
          status_.state = sim2real_planning_msgs::PlannerStatus::REACHED;
          status_.armable = false;
          status_.reason =
              "measured vehicle state reached the SUPER effective goal";
          publishStatus();
        } else {
          if (synthetic_close_goal_active_) {
            status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
            status_.armable = true;
            status_.reason = "SUPER close-goal arrival is stabilizing";
          } else {
            status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
            status_.armable = true;
            status_.reason = "SUPER measured endpoint arrival is stabilizing";
          }
        }
      }
    }
  }

  void statusTimer(const ros::TimerEvent &) {
    const ros::Time now = ros::Time::now();
    updateReadiness();

    if (has_goal_ && pending_goal_ && !goal_received_at_.isZero() &&
        (now - goal_received_at_).toSec() > settle_timeout_) {
      setFault("vehicle did not settle before SUPER planning timeout", true);
      return;
    }
    if (has_goal_ && goal_dispatched_ && !trajectory_committed_ &&
        !planning_started_at_.isZero() &&
        (now - planning_started_at_).toSec() > planning_timeout_) {
      setFault("SUPER did not acknowledge and commit a trajectory in time",
               true);
      return;
    }
    if (!native_replan_started_at_.isZero() &&
        nativeReplanHoldTimedOut(native_replan_hold_,
                                 (now - native_replan_started_at_).toSec(),
                                 planning_timeout_)) {
      setFault("SUPER did not commit a replacement trajectory before the "
               "bounded replanning timeout",
               true);
      return;
    }
    if (!busy_progress_started_at_.isZero() &&
        (now - busy_progress_started_at_).toSec() > planning_timeout_) {
      setFault("SUPER FSM remained busy beyond planning timeout", true);
      return;
    }
    const bool replan_hold_authorized = nativeReplanHoldPermitsMissingCommand(
        native_replan_hold_, native_progress_state_);
    if (trajectory_committed_ && !native_finished_ && !replan_hold_authorized &&
        !accepted_trajectory_at_.isZero()) {
      const ros::Time reference = last_native_command_receipt_.isZero()
                                      ? accepted_trajectory_at_
                                      : last_native_command_receipt_;
      if ((now - reference).toSec() > native_command_timeout_) {
        setFault("SUPER native position-command watchdog expired", true);
        return;
      }
    }

    const bool was_runtime_ready = runtime_ready_;
    const bool ready = runtimeReady(now);
    runtime_ready_ = ready;
    if (ready)
      ever_runtime_ready_ = true;

    if (!ready && (was_runtime_ready || has_goal_)) {
      setFault("SUPER input, heartbeat, progress, service, or native link "
               "became unavailable",
               true);
      return;
    }
    if (!ready && !has_goal_) {
      if (!ever_runtime_ready_) {
        status_.state = sim2real_planning_msgs::PlannerStatus::STARTING;
        status_.armable = false;
        status_.reason = "waiting for odometry, point cloud, SUPER heartbeat, "
                         "and native links";
      } else if (status_.state !=
                 sim2real_planning_msgs::PlannerStatus::FAULT) {
        status_.state = sim2real_planning_msgs::PlannerStatus::FAULT;
        status_.armable = false;
        status_.reason = "SUPER runtime is not ready";
      }
    } else if (ready && !has_goal_) {
      status_.state = sim2real_planning_msgs::PlannerStatus::READY;
      status_.armable = false;
      status_.reason = "ready";
    }

    dispatchPendingGoalIfReady();
    publishStatus();
  }

  void setFault(const std::string &reason, bool reset_native) {
    const bool was_fault =
        status_.state == sim2real_planning_msgs::PlannerStatus::FAULT;
    const bool repeated = was_fault && status_.reason == reason;
    const bool had_active_state =
        has_goal_ || pending_goal_ || goal_dispatched_ || trajectory_committed_;
    const bool should_reset_native =
        reset_native && (!was_fault || had_active_state);
    clearCurrentTrajectory();
    has_goal_ = false;
    status_.state = sim2real_planning_msgs::PlannerStatus::FAULT;
    status_.armable = false;
    status_.reason = reason;
    if (!repeated)
      ROS_ERROR_STREAM(reason);
    publishStatus();

    const bool native_reset_reachable =
        reset_client_.exists() && nativeCoreReady(ros::Time::now());
    if (should_reset_native && native_reset_reachable) {
      std::string reset_reason;
      if (!callNativeReset(&reset_reason)) {
        ROS_ERROR_STREAM("SUPER fault reset failed: " << reset_reason);
      }
    } else if (should_reset_native) {
      ROS_WARN_STREAM(
          "skipping SUPER fault reset because the native runtime is not "
          "responsive");
    }
  }

  void publishStatus() {
    status_.header.stamp = ros::Time::now();
    status_.header.frame_id = "world";
    status_pub_.publish(status_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher native_goal_pub_;
  ros::Publisher native_odom_pub_;
  ros::Publisher native_cloud_pub_;
  ros::Publisher command_pub_;
  ros::Publisher status_pub_;
  ros::Publisher capabilities_pub_;
  ros::Publisher trajectory_viz_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber native_command_sub_;
  ros::Subscriber native_trajectory_sub_;
  ros::Subscriber native_heartbeat_sub_;
  ros::Subscriber native_progress_sub_;
  ros::Subscriber native_effective_goal_sub_;
  ros::Subscriber native_trajectory_viz_sub_;
  ros::ServiceClient reset_client_;
  ros::ServiceClient native_validate_client_;
  ros::ServiceServer validate_server_;
  ros::Timer command_timer_;
  ros::Timer status_timer_;

  std::string backend_id_;
  std::string backend_namespace_;
  std::string profile_;
  std::string runtime_mode_;
  std::string odom_topic_;
  std::string cloud_topic_;

  double odom_timeout_ = 0.5;
  double cloud_timeout_ = 1.0;
  double map_settle_time_ = 0.2;
  double heartbeat_timeout_ = 0.25;
  double progress_timeout_ = 0.25;
  double native_command_timeout_ = 0.08;
  double planning_timeout_ = 10.0;
  double settle_timeout_ = 10.0;
  double reset_timeout_ = 0.5;
  double command_rate_ = 100.0;
  double status_rate_ = 20.0;
  double settle_velocity_tolerance_ = 0.2;
  double settle_angular_velocity_tolerance_ = 10.0 * std::acos(-1.0) / 180.0;
  double settle_hold_time_ = 0.5;
  double goal_position_tolerance_ = 0.35;
  double reached_velocity_tolerance_ = 0.2;
  double reached_yaw_tolerance_ = 5.0 * std::acos(-1.0) / 180.0;
  double reached_yaw_rate_tolerance_ = 10.0 * std::acos(-1.0) / 180.0;
  double reached_hold_time_ = 0.5;
  double max_effective_goal_shift_ = 3.0;
  double max_velocity_ = 2.4;
  double max_acceleration_ = 3.0;
  bool self_filter_enabled_ = true;
  double self_filter_radius_ = 0.35;
  double self_filter_min_z_ = -0.20;
  double self_filter_max_z_ = 0.20;
  double self_filter_pose_tolerance_ = 0.10;

  sim2real_planning_msgs::PlannerGoal current_goal_;
  sim2real_planning_msgs::PlannerStatus status_;
  geometry_msgs::PoseStamped effective_goal_;
  ros::Time native_goal_stamp_;
  bool has_goal_ = false;
  bool pending_goal_ = false;
  bool goal_dispatched_ = false;
  bool effective_goal_acked_ = false;
  bool trajectory_committed_ = false;
  bool goal_has_normal_command_ = false;
  bool latest_native_command_valid_ = false;
  bool latest_native_command_is_backup_ = false;
  bool pending_native_trajectory_valid_ = false;
  bool native_finished_ = false;
  bool native_replan_hold_ = false;
  bool online_goal_handoff_ = false;
  bool close_goal_completion_pending_ = false;
  bool synthetic_close_goal_active_ = false;
  bool reached_ = false;
  bool latest_odom_valid_ = false;
  bool latest_cloud_valid_ = false;
  bool runtime_ready_ = false;
  bool ever_runtime_ready_ = false;
  uint8_t native_progress_state_ = kSuperInit;
  uint64_t accepted_native_trajectory_id_ = 0;
  uint64_t highest_native_trajectory_id_ = 0;
  uint64_t native_trajectory_floor_ = 0;
  uint64_t accepted_public_trajectory_id_ = 0;
  uint64_t highest_public_trajectory_id_ = 0;

  ros::Time last_odom_receipt_;
  ros::Time last_cloud_receipt_;
  ros::Time first_cloud_receipt_;
  ros::Time last_odom_measurement_;
  ros::Time last_cloud_measurement_;
  ros::Time last_heartbeat_receipt_;
  ros::Time last_progress_receipt_;
  ros::Time busy_progress_started_at_;
  ros::Time native_replan_started_at_;
  ros::Time goal_received_at_;
  ros::Time planning_started_at_;
  ros::Time accepted_trajectory_at_;
  ros::Time last_native_command_receipt_;
  ros::Time last_native_command_stamp_;
  ros::Time settled_since_;
  ros::Time reached_candidate_since_;

  Eigen::Vector3d latest_odom_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond latest_odom_body_in_world_ =
      Eigen::Quaterniond::Identity();
  Eigen::Vector3d latest_odom_world_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_odom_world_angular_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d hold_position_ = Eigen::Vector3d::Zero();
  double hold_yaw_ = 0.0;
  std::deque<StampedBodyPose> odom_history_;
  quadrotor_msgs::PositionCommand latest_native_command_;
  quadrotor_msgs::PolynomialTrajectory pending_native_trajectory_;
};

} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "super_backend_adapter");
  try {
    SuperBackendAdapter adapter;
    ros::spin();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM(
        "SUPER planner adapter failed to initialize: " << error.what());
    return 2;
  }
  return 0;
}
