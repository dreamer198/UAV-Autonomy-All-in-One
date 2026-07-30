#include <sim2real_fast_adapter/ground_plane_filter.h>
#include <sim2real_fast_adapter/validation.h>
#include <sim2real_fast_adapter/visualization_map.h>

#include <bspline/non_uniform_bspline.h>
#include <nav_msgs/Odometry.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/FastPlannerGoal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sim2real_planning_msgs/PlannerCapabilities.h>
#include <sim2real_planning_msgs/PlannerCommand.h>
#include <sim2real_planning_msgs/PlannerGoal.h>
#include <sim2real_planning_msgs/PlannerStatus.h>
#include <sim2real_planning_msgs/ValidateGoal.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using fast_planner::NonUniformBspline;
using sim2real_fast_adapter::bodyVelocityToWorld;
using sim2real_fast_adapter::buildVirtualFloor;
using sim2real_fast_adapter::finite;
using sim2real_fast_adapter::finitePoint;
using sim2real_fast_adapter::finitePose;
using sim2real_fast_adapter::finiteTrajectorySetpoint;
using sim2real_fast_adapter::fitDominantGroundPlane;
using sim2real_fast_adapter::goalAboveVirtualFloor;
using sim2real_fast_adapter::goalInsideMapBounds;
using sim2real_fast_adapter::measurementStampIsCurrent;
using sim2real_fast_adapter::measuredStateSatisfiesGoal;
using sim2real_fast_adapter::pointInsideBodyExclusionCylinder;
using sim2real_fast_adapter::RealObstacleVisualizationMap;
using sim2real_fast_adapter::removeGroundPlanePoints;

bool finiteVector(const geometry_msgs::Vector3& value) {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

struct StampedBodyPose {
  ros::Time stamp;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond body_in_world = Eigen::Quaterniond::Identity();
};

class FastBackendAdapter {
 public:
  FastBackendAdapter() : nh_(), pnh_("~") {
    pnh_.param<std::string>("backend_id", backend_id_, "fast-kino");
    pnh_.param<std::string>("backend_namespace", backend_namespace_, "fast_kino");
    pnh_.param<std::string>("profile", profile_, "local");
    pnh_.param<std::string>("runtime_mode", runtime_mode_, "");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/localization/odom");
    pnh_.param<std::string>("cloud_topic", cloud_topic_, "/localization/cloud_registered");
    pnh_.param("planner", planner_, 1);
    pnh_.param("adapter/odom_timeout", odom_timeout_, 0.5);
    pnh_.param("adapter/cloud_timeout", cloud_timeout_, 1.0);
    pnh_.param("adapter/self_filter_enabled", self_filter_enabled_, true);
    pnh_.param("adapter/self_filter_radius", self_filter_radius_, 0.35);
    pnh_.param("adapter/self_filter_min_z", self_filter_min_z_, -0.20);
    pnh_.param("adapter/self_filter_max_z", self_filter_max_z_, 0.20);
    pnh_.param("adapter/self_filter_pose_tolerance",
               self_filter_pose_tolerance_, 0.10);
    pnh_.param("adapter/hide_observed_ground_plane",
               hide_observed_ground_plane_, true);
    pnh_.param("adapter/ground_plane_distance_tolerance",
               ground_plane_distance_tolerance_, 0.03);
    double ground_plane_max_tilt_deg = 15.0;
    pnh_.param("adapter/ground_plane_max_tilt_deg",
               ground_plane_max_tilt_deg, 15.0);
    pnh_.param("adapter/ground_plane_min_points",
               ground_plane_min_points_, 200);
    pnh_.param("adapter/ground_plane_min_xy_span",
               ground_plane_min_xy_span_, 2.0);
    pnh_.param("adapter/ground_plane_ransac_iterations",
               ground_plane_ransac_iterations_, 96);
    pnh_.param("adapter/heartbeat_timeout", heartbeat_timeout_, 0.25);
    pnh_.param("adapter/progress_timeout", progress_timeout_, 0.25);
    pnh_.param("adapter/planning_timeout", planning_timeout_, 10.0);
    pnh_.param("adapter/map_settle_time", map_settle_time_, 0.1);
    pnh_.param("adapter/command_rate", command_rate_, 100.0);
    pnh_.param("adapter/status_rate", status_rate_, 20.0);
    pnh_.param("adapter/goal_position_tolerance", goal_position_tolerance_, 0.35);
    pnh_.param("adapter/reached_velocity_tolerance",
               reached_velocity_tolerance_, 0.2);
    double reached_yaw_tolerance_deg = 5.0;
    double reached_yaw_rate_tolerance_deg_s = 10.0;
    pnh_.param("adapter/reached_yaw_tolerance_deg",
               reached_yaw_tolerance_deg, 5.0);
    pnh_.param("adapter/reached_yaw_rate_tolerance_deg_s",
               reached_yaw_rate_tolerance_deg_s, 10.0);
    pnh_.param("adapter/reached_hold_time", reached_hold_time_, 0.5);
    pnh_.param("adapter/inject_virtual_floor", inject_virtual_floor_, false);
    pnh_.param("adapter/virtual_floor_height", virtual_floor_height_, 0.0);
    const double radians_per_degree = std::acos(-1.0) / 180.0;
    ground_plane_max_tilt_ =
        ground_plane_max_tilt_deg * radians_per_degree;
    reached_yaw_tolerance_ = reached_yaw_tolerance_deg * radians_per_degree;
    reached_yaw_rate_tolerance_ =
        reached_yaw_rate_tolerance_deg_s * radians_per_degree;
    pnh_.param("manager/max_vel", max_velocity_, 0.5);
    pnh_.param("manager/max_acc", max_acceleration_, 0.8);
    pnh_.param("manager/clearance_threshold",
               manager_clearance_threshold_, 0.25);
    pnh_.param("topo_prm/clearance", topo_clearance_, 0.3);
    goal_clearance_ = manager_clearance_threshold_;
    pnh_.param("sdf_map/origin_x", map_origin_.x(), 0.0);
    pnh_.param("sdf_map/origin_y", map_origin_.y(), 0.0);
    pnh_.param("sdf_map/origin_z", map_origin_.z(), 0.0);
    pnh_.param("sdf_map/map_size_x", map_size_.x(), -1.0);
    pnh_.param("sdf_map/map_size_y", map_size_.y(), -1.0);
    pnh_.param("sdf_map/map_size_z", map_size_.z(), -1.0);
    pnh_.param("sdf_map/local_update_range_x", local_update_range_.x(), -1.0);
    pnh_.param("sdf_map/local_update_range_y", local_update_range_.y(), -1.0);
    pnh_.param("sdf_map/local_update_range_z", local_update_range_.z(), -1.0);
    pnh_.param("sdf_map/resolution", map_resolution_, -1.0);
    pnh_.param("sdf_map/obstacles_inflation", inflation_, 0.1);
    pnh_.param("sdf_map/obstacles_inflation_z", inflation_z_, map_resolution_);
    pnh_.param("sdf_map/accumulate_cloud", accumulate_cloud_, false);

    const char* environment_mode_value = std::getenv("SIM2REAL_RUNTIME_MODE");
    const std::string environment_mode =
        environment_mode_value == nullptr ? "" : environment_mode_value;
    if ((backend_id_ != "fast-kino" && backend_id_ != "fast-topo") ||
        (planner_ != 1 && planner_ != 2) ||
        (backend_id_ == "fast-kino" && planner_ != 1) ||
        (backend_id_ == "fast-topo" && planner_ != 2) ||
        profile_ != "local" ||
        (runtime_mode_ != "simulation" && runtime_mode_ != "real") ||
        environment_mode != runtime_mode_ ||
        backend_namespace_.empty() ||
        backend_namespace_.find('-') != std::string::npos ||
        !finite(command_rate_) || !finite(status_rate_) ||
        !finite(odom_timeout_) || !finite(cloud_timeout_) ||
        !finite(self_filter_radius_) || !finite(self_filter_min_z_) ||
        !finite(self_filter_max_z_) ||
        !finite(self_filter_pose_tolerance_) ||
        !finite(ground_plane_distance_tolerance_) ||
        !finite(ground_plane_max_tilt_) ||
        !finite(ground_plane_min_xy_span_) ||
        !finite(map_settle_time_) || !finite(heartbeat_timeout_) ||
        !finite(progress_timeout_) ||
        !finite(planning_timeout_) ||
        !finite(goal_position_tolerance_) ||
        !finite(reached_velocity_tolerance_) ||
        !finite(reached_yaw_tolerance_) ||
        !finite(reached_yaw_rate_tolerance_) ||
        !finite(reached_hold_time_) ||
        !finite(virtual_floor_height_) ||
        !finite(max_velocity_) ||
        !finite(max_acceleration_) ||
        !finite(manager_clearance_threshold_) ||
        !finite(topo_clearance_) || !finite(goal_clearance_) ||
        !finite(inflation_) ||
        !finite(inflation_z_) ||
        command_rate_ <= 0.0 || status_rate_ <= 0.0 ||
        odom_timeout_ <= 0.0 || cloud_timeout_ <= 0.0 ||
        (self_filter_enabled_ &&
         (self_filter_radius_ <= 0.0 ||
          self_filter_min_z_ >= self_filter_max_z_ ||
          self_filter_pose_tolerance_ <= 0.0)) ||
        (hide_observed_ground_plane_ &&
         (ground_plane_distance_tolerance_ <= 0.0 ||
          ground_plane_max_tilt_ <= 0.0 ||
          ground_plane_max_tilt_ >= 0.5 * std::acos(-1.0) ||
          ground_plane_min_points_ < 3 ||
          ground_plane_min_xy_span_ <= 0.0 ||
          ground_plane_ransac_iterations_ <= 0)) ||
        heartbeat_timeout_ <= 0.0 || map_settle_time_ < 0.0 ||
        progress_timeout_ <= 0.0 ||
        planning_timeout_ <= 0.0 ||
        goal_position_tolerance_ <= 0.0 ||
        reached_velocity_tolerance_ <= 0.0 ||
        reached_yaw_tolerance_ <= 0.0 ||
        reached_yaw_rate_tolerance_ <= 0.0 ||
        reached_hold_time_ <= 0.0 ||
        max_velocity_ <= 0.0 ||
        max_acceleration_ <= 0.0 ||
        manager_clearance_threshold_ <= 0.0 ||
        topo_clearance_ <= 0.0 || goal_clearance_ <= 0.0 ||
        inflation_ < 0.0 ||
        inflation_z_ < 0.0 ||
        !finite(map_resolution_) || map_resolution_ <= 0.0 ||
        !map_origin_.allFinite() || !map_size_.allFinite() ||
        !local_update_range_.allFinite() || (map_size_.array() <= 0.0).any() ||
        (local_update_range_.array() < 0.0).any() ||
        (inject_virtual_floor_ &&
         (virtual_floor_height_ <= map_origin_.z() ||
          virtual_floor_height_ + map_resolution_ >=
              map_origin_.z() + map_size_.z()))) {
      throw std::invalid_argument("invalid Fast planner adapter identity or timing parameters");
    }

    visualization_map_.reset(new RealObstacleVisualizationMap(
        map_origin_, map_size_, local_update_range_, map_resolution_,
        inflation_, inflation_z_, accumulate_cloud_));

    native_goal_pub_ = nh_.advertise<plan_manage::FastPlannerGoal>("native/goal", 2);
    native_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("native/odom_world", 10);
    native_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("native/cloud_world", 2);
    visualization_occupancy_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("viz/occupancy", 1);
    visualization_inflated_occupancy_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            "viz/inflated_occupancy", 1);
    command_pub_ = nh_.advertise<sim2real_planning_msgs::PlannerCommand>("command", 10);
    status_pub_ = nh_.advertise<sim2real_planning_msgs::PlannerStatus>("status", 10, true);
    capabilities_pub_ =
        nh_.advertise<sim2real_planning_msgs::PlannerCapabilities>("capabilities", 1, true);

    goal_sub_ = nh_.subscribe("goal", 2, &FastBackendAdapter::goalCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &FastBackendAdapter::odomCallback, this);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &FastBackendAdapter::cloudCallback, this);
    bspline_sub_ = nh_.subscribe("native/bspline", 2, &FastBackendAdapter::bsplineCallback, this);
    replan_sub_ = nh_.subscribe("native/replan", 10, &FastBackendAdapter::replanCallback, this);
    heartbeat_sub_ =
        nh_.subscribe("native/heartbeat", 10, &FastBackendAdapter::heartbeatCallback, this);
    progress_sub_ =
        nh_.subscribe("native/progress", 10, &FastBackendAdapter::progressCallback, this);
    validate_server_ =
        nh_.advertiseService("validate_goal", &FastBackendAdapter::validateGoalCallback, this);

    command_timer_ = nh_.createTimer(ros::Duration(1.0 / command_rate_),
                                     &FastBackendAdapter::commandTimer, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_rate_),
                                    &FastBackendAdapter::statusTimer, this);

    status_.backend_id = backend_id_;
    status_.state = sim2real_planning_msgs::PlannerStatus::STARTING;
    status_.reason = "waiting for valid odometry and point cloud";
    publishCapabilities();
    publishStatus();
  }

 private:
  void publishCapabilities() {
    sim2real_planning_msgs::PlannerCapabilities capabilities;
    capabilities.header.stamp = ros::Time::now();
    capabilities.header.frame_id = "world";
    capabilities.api_version = "sim2real.planner/v1";
    capabilities.backend_id = backend_id_;
    capabilities.variant = planner_ == 1 ? "kinodynamic" : "topological";
    capabilities.simulation = true;
    capabilities.yaw = true;
    capabilities.cancel = true;
    capabilities.goal_validation = true;
    capabilities.rviz = true;
    // Fast-Planner's feasibility limits are per Cartesian axis. The common
    // capability is a Euclidean norm, whose tight enclosing bound is sqrt(3).
    capabilities.max_velocity = std::sqrt(3.0) * max_velocity_;
    capabilities.max_acceleration = std::sqrt(3.0) * max_acceleration_;
    capabilities.has_fixed_map_bounds = true;
    capabilities.map_min.x = map_origin_.x();
    capabilities.map_min.y = map_origin_.y();
    capabilities.map_min.z = map_origin_.z();
    const Eigen::Vector3d map_max = map_origin_ + map_size_;
    capabilities.map_max.x = map_max.x();
    capabilities.map_max.y = map_max.y();
    capabilities.map_max.z = map_max.z();
    capabilities_pub_.publish(capabilities);
  }

  bool validateGoal(const sim2real_planning_msgs::PlannerGoal& goal,
                    std::string* reason) const {
    if (goal.action == sim2real_planning_msgs::PlannerGoal::CANCEL) return true;
    if (goal.action != sim2real_planning_msgs::PlannerGoal::PLAN) {
      if (reason) *reason = "unknown goal action";
      return false;
    }
    if (goal.session_id.empty() || goal.goal_id == 0 || goal.goal.header.stamp.isZero()) {
      if (reason) *reason = "PLAN requires a session, non-zero goal ID, and measurement timestamp";
      return false;
    }
    if (!finitePose(goal.goal, goal.constrain_yaw, reason)) return false;
    if (!goalInsideMapBounds(goal.goal.pose.position, map_origin_, map_size_,
                             inflation_, reason)) {
      return false;
    }
    if (inject_virtual_floor_ &&
        !goalAboveVirtualFloor(
            goal.goal.pose.position.z, virtual_floor_height_, inflation_z_,
            goal_clearance_, map_resolution_, reason)) {
      return false;
    }
    if (!obstacle_validation_ready_ || visualization_map_ == nullptr) {
      if (reason) *reason = "Fast obstacle map is not ready for goal validation";
      return false;
    }
    const Eigen::Vector3d goal_position(
        goal.goal.pose.position.x, goal.goal.pose.position.y,
        goal.goal.pose.position.z);
    double nearest_obstacle = std::numeric_limits<double>::infinity();
    if (visualization_map_->inflatedObstacleWithin(
            goal_position, goal_clearance_, &nearest_obstacle)) {
      if (reason) {
        *reason =
            "goal is inside the required clearance around an observed Fast obstacle";
      }
      return false;
    }
    return true;
  }

  bool validateGoalCallback(sim2real_planning_msgs::ValidateGoal::Request& request,
                            sim2real_planning_msgs::ValidateGoal::Response& response) {
    response.valid = validateGoal(request.goal, &response.reason);
    if (response.valid) response.reason = "goal is valid for the Fast fixed map";
    return true;
  }

  void goalCallback(const sim2real_planning_msgs::PlannerGoalConstPtr& message) {
    std::string reason;
    if (!validateGoal(*message, &reason)) {
      setFault("rejected goal: " + reason);
      return;
    }

    if (message->action == sim2real_planning_msgs::PlannerGoal::CANCEL) {
      if (has_goal_ &&
          (message->session_id != current_goal_.session_id ||
           (message->goal_id != 0 && message->goal_id != current_goal_.goal_id))) {
        ROS_WARN("ignoring stale Fast planner cancel request");
        return;
      }
      plan_manage::FastPlannerGoal native;
      native.header = message->header;
      native.goal_id = message->goal_id;
      native.action = plan_manage::FastPlannerGoal::CANCEL;
      native.goal = message->goal;
      native.constrain_yaw = message->constrain_yaw;
      native_goal_pub_.publish(native);
      pending_goal_ = false;
      has_goal_ = false;
      trajectory_valid_ = false;
      braking_hold_valid_ = false;
      reached_candidate_since_ = ros::Time();
      planning_started_at_ = ros::Time();
      status_.session_id = message->session_id;
      status_.goal_id = message->goal_id;
      status_.trajectory_id = 0;
      status_.state = sim2real_planning_msgs::PlannerStatus::HOLDING;
      status_.armable = false;
      status_.reason = "goal cancelled";
      publishStatus();
      return;
    }

    current_goal_ = *message;
    has_goal_ = true;
    pending_goal_ = true;
    planning_started_at_ = ros::Time();
    trajectory_valid_ = false;
    braking_ = false;
    braking_hold_valid_ = false;
    reached_candidate_since_ = ros::Time();
    last_trajectory_id_ = 0;
    status_.session_id = message->session_id;
    status_.goal_id = message->goal_id;
    status_.trajectory_id = 0;
    status_.active_goal = message->goal;
    status_.state = sim2real_planning_msgs::PlannerStatus::PLANNING;
    status_.armable = false;
    status_.reason = "waiting for sensor readiness before planning";
    publishStatus();
    dispatchPendingGoalIfReady();
  }

  void dispatchPendingGoalIfReady() {
    updateReadiness();
    if (!pending_goal_ || !status_.odom_ready || !status_.map_ready ||
        !coreReady(ros::Time::now())) {
      return;
    }

    plan_manage::FastPlannerGoal native;
    native.header = current_goal_.header;
    native.goal_id = current_goal_.goal_id;
    native.action = plan_manage::FastPlannerGoal::PLAN;
    native.goal = current_goal_.goal;
    native.constrain_yaw = current_goal_.constrain_yaw;
    native_goal_pub_.publish(native);
    pending_goal_ = false;
    planning_started_at_ = ros::Time::now();
    status_.reason = "Fast-Planner is computing a trajectory";
    publishStatus();
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    const auto& p = message->pose.pose.position;
    const auto& q = message->pose.pose.orientation;
    const auto& linear = message->twist.twist.linear;
    const auto& angular = message->twist.twist.angular;
    const double q_norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const Eigen::Vector3d odom_position(p.x, p.y, p.z);
    const Eigen::Vector3d map_max = map_origin_ + map_size_;
    const ros::Time now = ros::Time::now();
    if (!measurementStampIsCurrent(message->header.stamp, now, odom_timeout_,
                                   last_odom_measurement_) ||
        message->header.frame_id != "world" ||
        message->child_frame_id != "base_link" || !finitePoint(p) ||
        !finiteVector(linear) || !finiteVector(angular) || !finite(q_norm2) ||
        q_norm2 < 0.9801 || q_norm2 > 1.0201 ||
        (odom_position.array() < map_origin_.array()).any() ||
        (odom_position.array() >= map_max.array()).any()) {
      last_odom_receipt_ = ros::Time();
      latest_odom_valid_ = false;
      odom_history_.clear();
      setFault("odometry contract violation");
      return;
    }

    const double q_norm = std::sqrt(q_norm2);
    const Eigen::Quaterniond body_in_world(q.w / q_norm, q.x / q_norm, q.y / q_norm, q.z / q_norm);
    const Eigen::Vector3d world_linear =
        bodyVelocityToWorld(body_in_world, Eigen::Vector3d(linear.x, linear.y, linear.z));
    const Eigen::Vector3d world_angular =
        bodyVelocityToWorld(body_in_world, Eigen::Vector3d(angular.x, angular.y, angular.z));

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
    latest_odom_position_ = odom_position;
    latest_odom_world_velocity_ = world_linear;
    latest_odom_body_in_world_ = body_in_world;
    latest_odom_world_angular_velocity_ = world_angular;
    latest_odom_valid_ = true;
    last_odom_measurement_ = message->header.stamp;
    last_odom_receipt_ = now;
    StampedBodyPose body_pose;
    body_pose.stamp = message->header.stamp;
    body_pose.position = odom_position;
    body_pose.body_in_world = body_in_world;
    odom_history_.push_back(body_pose);
    const double history_duration =
        std::max(odom_timeout_, cloud_timeout_) +
        self_filter_pose_tolerance_ + 0.1;
    while (!odom_history_.empty() &&
           (message->header.stamp - odom_history_.front().stamp).toSec() >
               history_duration) {
      odom_history_.pop_front();
    }
  }

  bool bodyPoseAtMeasurement(
      const ros::Time& stamp, Eigen::Vector3d* position,
      Eigen::Quaterniond* body_in_world) const {
    if (position == nullptr || body_in_world == nullptr ||
        stamp.isZero() || odom_history_.empty()) {
      return false;
    }
    const StampedBodyPose* nearest = nullptr;
    double nearest_delta = std::numeric_limits<double>::infinity();
    for (const StampedBodyPose& pose : odom_history_) {
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

  bool extractPlanningCloudPoints(
      const sensor_msgs::PointCloud2ConstPtr& message,
      const Eigen::Vector3d& body_position,
      const Eigen::Quaterniond& body_in_world,
      std::vector<Eigen::Vector3d>* points, size_t* removed_self_points,
      std::string* reason) const {
    if (points == nullptr || removed_self_points == nullptr) {
      if (reason) *reason = "null output passed to Fast cloud filter";
      return false;
    }
    const uint64_t input_count =
        static_cast<uint64_t>(message->width) *
        static_cast<uint64_t>(message->height);
    if (input_count >
        static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
      if (reason) *reason = "Fast input point count exceeds address space";
      return false;
    }
    points->clear();
    points->reserve(static_cast<size_t>(input_count));
    *removed_self_points = 0U;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> input_x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> input_y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> input_z(*message, "z");
      for (uint64_t i = 0; i < input_count;
           ++i, ++input_x, ++input_y, ++input_z) {
        const Eigen::Vector3d point(*input_x, *input_y, *input_z);
        if (!point.allFinite()) continue;
        if (self_filter_enabled_ &&
            pointInsideBodyExclusionCylinder(
                point, body_position, body_in_world, self_filter_radius_,
                self_filter_min_z_, self_filter_max_z_)) {
          ++(*removed_self_points);
          continue;
        }
        points->push_back(point);
      }
    } catch (const std::runtime_error& error) {
      if (reason) {
        *reason = std::string("Fast point cloud lacks float32 xyz fields: ") +
            error.what();
      }
      return false;
    }
    return true;
  }

  sensor_msgs::PointCloud2 makeXyzCloud(
      const std_msgs::Header& header,
      const std::vector<Eigen::Vector3d>& points) const {
    sensor_msgs::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> output_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> output_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> output_z(output, "z");
    for (const Eigen::Vector3d& point : points) {
      *output_x = static_cast<float>(point.x());
      *output_y = static_cast<float>(point.y());
      *output_z = static_cast<float>(point.z());
      ++output_x;
      ++output_y;
      ++output_z;
    }
    output.is_dense = true;
    return output;
  }

  void publishVisualizationMaps(const std_msgs::Header& header) {
    if (!latest_odom_valid_ || visualization_map_ == nullptr) return;
    if (visualization_occupancy_pub_.getNumSubscribers() > 0) {
      visualization_occupancy_pub_.publish(makeXyzCloud(
          header,
          visualization_map_->occupancyPoints(latest_odom_position_)));
    }
    if (visualization_inflated_occupancy_pub_.getNumSubscribers() > 0) {
      visualization_inflated_occupancy_pub_.publish(makeXyzCloud(
          header,
          visualization_map_->inflatedOccupancyPoints(
              latest_odom_position_)));
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    const uint64_t expected_minimum =
        static_cast<uint64_t>(message->row_step) * static_cast<uint64_t>(message->height);
    const bool empty = message->width == 0 || message->height == 0;
    const ros::Time now = ros::Time::now();
    if (!measurementStampIsCurrent(message->header.stamp, now, cloud_timeout_,
                                   last_cloud_measurement_) ||
        message->header.frame_id != "world" ||
        empty || message->point_step == 0 || message->row_step == 0 ||
        expected_minimum > message->data.size()) {
      last_cloud_receipt_ = ros::Time();
      setFault("point-cloud contract violation");
      return;
    }
    if (!latest_odom_valid_) {
      last_cloud_receipt_ = ros::Time();
      setFault("cannot adapt Fast point cloud without valid odometry");
      return;
    }

    Eigen::Vector3d cloud_body_position = latest_odom_position_;
    Eigen::Quaterniond cloud_body_in_world = latest_odom_body_in_world_;
    if (self_filter_enabled_ &&
        !bodyPoseAtMeasurement(message->header.stamp, &cloud_body_position,
                               &cloud_body_in_world)) {
      last_cloud_receipt_ = ros::Time();
      setFault("cannot time-align Fast self-filter with odometry");
      return;
    }

    std::vector<Eigen::Vector3d> planning_points;
    size_t removed_self_points = 0U;
    std::string filter_reason;
    if (!extractPlanningCloudPoints(
            message, cloud_body_position, cloud_body_in_world,
            &planning_points, &removed_self_points, &filter_reason)) {
      last_cloud_receipt_ = ros::Time();
      setFault(filter_reason);
      return;
    }
    if (removed_self_points > 0U) {
      ROS_WARN_THROTTLE(
          5.0,
          "Fast airframe self-filter removed %zu point(s) from the current scan",
          removed_self_points);
    }

    // Build both public map layers exclusively from external sensor points.
    // The simulated ground is recognized as a geometric plane instead of
    // deleting every point below a fixed z threshold, so low obstacles remain
    // visible. The private virtual floor is appended only afterwards.
    std::vector<Eigen::Vector3d> visualization_points = planning_points;
    bool visualization_points_ready = true;
    if (hide_observed_ground_plane_) {
      Eigen::Vector4d fitted_plane;
      const double candidate_minimum_z = std::max(
          map_origin_.z(), virtual_floor_height_ - 0.8);
      const double candidate_maximum_z = std::min(
          map_origin_.z() + map_size_.z(),
          virtual_floor_height_ + map_resolution_);
      if (fitDominantGroundPlane(
              planning_points, candidate_minimum_z, candidate_maximum_z,
              ground_plane_distance_tolerance_,
              ground_plane_max_tilt_,
              static_cast<size_t>(ground_plane_min_points_),
              ground_plane_min_xy_span_,
              ground_plane_ransac_iterations_, &fitted_plane)) {
        visualization_ground_plane_ = fitted_plane;
        visualization_ground_plane_valid_ = true;
      }
      if (visualization_ground_plane_valid_) {
        size_t removed_ground_points = 0U;
        visualization_points = removeGroundPlanePoints(
            planning_points, visualization_ground_plane_,
            ground_plane_distance_tolerance_, &removed_ground_points);
        ROS_INFO_THROTTLE(
            5.0,
            "Fast RViz ground-plane classifier removed %zu of %zu "
            "external point(s)",
            removed_ground_points, planning_points.size());
      } else {
        visualization_points_ready = false;
        ROS_WARN_THROTTLE(
            5.0,
            "Fast RViz map is waiting for a reliable observed ground plane");
      }
    }
    if (visualization_points_ready) {
      std::string visualization_reason;
      if (!visualization_map_->update(
              visualization_points, latest_odom_position_,
              &visualization_reason)) {
        last_cloud_receipt_ = ros::Time();
        setFault(
            "failed to update Fast real-obstacle visualization: " +
            visualization_reason);
        return;
      }
      obstacle_validation_ready_ = true;
      publishVisualizationMaps(message->header);
    }
    if (inject_virtual_floor_) {
      std::vector<Eigen::Vector3d> floor_points;
      std::string floor_reason;
      if (!buildVirtualFloor(
              latest_odom_position_, map_origin_, map_size_, local_update_range_,
              map_resolution_, virtual_floor_height_, &floor_points,
              &floor_reason)) {
        last_cloud_receipt_ = ros::Time();
        setFault("failed to build Fast virtual floor: " + floor_reason);
        return;
      }
      if (planning_points.size() >
          std::numeric_limits<size_t>::max() - floor_points.size()) {
        last_cloud_receipt_ = ros::Time();
        setFault("Fast private point cloud size overflow");
        return;
      }
      planning_points.insert(
          planning_points.end(), floor_points.begin(), floor_points.end());
    }
    native_cloud_pub_.publish(
        makeXyzCloud(message->header, planning_points));
    last_cloud_measurement_ = message->header.stamp;
    last_cloud_receipt_ = now;
    if (first_cloud_receipt_.isZero()) first_cloud_receipt_ = last_cloud_receipt_;
  }

  void heartbeatCallback(const std_msgs::EmptyConstPtr&) {
    last_heartbeat_receipt_ = ros::Time::now();
  }

  void progressCallback(const std_msgs::UInt8ConstPtr& message) {
    if (message->data > 5U) {
      last_progress_receipt_ = ros::Time();
      setFault("Fast planner reported an invalid FSM progress state");
      return;
    }
    const bool was_busy = core_planning_busy_;
    core_planning_busy_ =
        message->data == 2U || message->data == 3U || message->data == 5U;
    last_progress_receipt_ = ros::Time::now();
    if (core_planning_busy_ && !was_busy) {
      busy_progress_started_at_ = last_progress_receipt_;
    } else if (!core_planning_busy_) {
      busy_progress_started_at_ = ros::Time();
    }
  }

  bool coreReady(const ros::Time& now) const {
    return !last_heartbeat_receipt_.isZero() &&
        (now - last_heartbeat_receipt_).toSec() <= heartbeat_timeout_;
  }

  bool coreProgressReady(const ros::Time& now) const {
    if (last_progress_receipt_.isZero()) return false;
    if (core_planning_busy_) {
      return !busy_progress_started_at_.isZero() &&
          (now - busy_progress_started_at_).toSec() <= planning_timeout_;
    }
    return (now - last_progress_receipt_).toSec() <= progress_timeout_;
  }

  bool validateBspline(const plan_manage::Bspline& message, std::string* reason) const {
    if (message.order != 3 || message.traj_id <= 0 ||
        message.goal_id != current_goal_.goal_id || message.start_time.isZero()) {
      if (reason) *reason = "invalid order, IDs, or start_time";
      return false;
    }
    const size_t expected_knots =
        message.pos_pts.size() + static_cast<size_t>(message.order) + 1U;
    if (message.pos_pts.size() < 4U || message.knots.size() != expected_knots ||
        message.yaw_pts.size() < 4U || !finite(message.yaw_dt) ||
        message.yaw_dt <= 0.0 || !finitePoint(message.active_goal)) {
      if (reason) *reason = "invalid control-point, knot, yaw, or active-goal dimensions";
      return false;
    }
    for (const auto& point : message.pos_pts) {
      if (!finitePoint(point)) {
        if (reason) *reason = "non-finite position control point";
        return false;
      }
    }
    for (size_t i = 0; i < message.knots.size(); ++i) {
      if (!finite(message.knots[i]) ||
          (i > 0 && message.knots[i] <= message.knots[i - 1])) {
        if (reason) *reason = "knots must be finite and strictly increasing";
        return false;
      }
    }
    for (double yaw : message.yaw_pts) {
      if (!finite(yaw)) {
        if (reason) *reason = "non-finite yaw control point";
        return false;
      }
    }
    return true;
  }

  void bsplineCallback(const plan_manage::BsplineConstPtr& message) {
    if (!has_goal_ || message->goal_id != current_goal_.goal_id) {
      ROS_WARN("dropping stale Fast-Planner B-spline");
      return;
    }
    std::string reason;
    if (!validateBspline(*message, &reason)) {
      setFault("rejected native B-spline: " + reason);
      return;
    }
    const uint64_t trajectory_id = static_cast<uint64_t>(message->traj_id);
    if (trajectory_id <= last_trajectory_id_) {
      setFault("rejected non-monotonic native trajectory ID");
      return;
    }

    Eigen::MatrixXd position_points(message->pos_pts.size(), 3);
    for (size_t i = 0; i < message->pos_pts.size(); ++i) {
      position_points(static_cast<Eigen::Index>(i), 0) = message->pos_pts[i].x;
      position_points(static_cast<Eigen::Index>(i), 1) = message->pos_pts[i].y;
      position_points(static_cast<Eigen::Index>(i), 2) = message->pos_pts[i].z;
    }
    Eigen::VectorXd knots(message->knots.size());
    for (size_t i = 0; i < message->knots.size(); ++i) {
      knots(static_cast<Eigen::Index>(i)) = message->knots[i];
    }
    Eigen::MatrixXd yaw_points(message->yaw_pts.size(), 1);
    for (size_t i = 0; i < message->yaw_pts.size(); ++i) {
      yaw_points(static_cast<Eigen::Index>(i), 0) = message->yaw_pts[i];
    }

    NonUniformBspline position(position_points, message->order, 0.1);
    position.setKnot(knots);
    NonUniformBspline yaw(yaw_points, message->order, message->yaw_dt);
    const double position_duration = position.getTimeSum();
    const double yaw_duration = yaw.getTimeSum();
    const double elapsed = (ros::Time::now() - message->start_time).toSec();
    if (!finite(position_duration) || position_duration <= 0.0 ||
        !finite(yaw_duration) || std::fabs(position_duration - yaw_duration) > 0.05 ||
        elapsed > position_duration + 0.1 ||
        (message->start_time - ros::Time::now()).toSec() > 0.1) {
      setFault("rejected stale or inconsistent native B-spline timing");
      return;
    }

    trajectory_.clear();
    trajectory_.push_back(position);
    trajectory_.push_back(position.getDerivative());
    trajectory_.push_back(trajectory_[1].getDerivative());
    trajectory_.push_back(yaw);
    trajectory_.push_back(yaw.getDerivative());
    start_time_ = message->start_time;
    full_duration_ = position_duration;
    effective_duration_ = full_duration_;
    last_trajectory_id_ = trajectory_id;
    trajectory_valid_ = true;
    planning_started_at_ = ros::Time();
    braking_ = false;
    braking_hold_valid_ = false;
    reached_candidate_since_ = ros::Time();

    status_.trajectory_id = trajectory_id;
    status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
    status_.armable = true;
    status_.reason = "trajectory active";
    status_.active_goal.pose.position = message->active_goal;
    status_.active_goal.header.frame_id = "world";
    status_.active_goal.header.stamp = current_goal_.goal.header.stamp;
    publishStatus();
  }

  void replanCallback(const std_msgs::EmptyConstPtr&) {
    if (!trajectory_valid_) return;
    planning_started_at_ = ros::Time::now();
    const double elapsed = std::max(0.0, (ros::Time::now() - start_time_).toSec());
    effective_duration_ = std::min(full_duration_, elapsed + 0.01);
    braking_ = true;
    /*
     * A truncated native B-spline may already be ahead of the airframe. Hold
     * the measured pose instead of its sampled truncation point; otherwise a
     * failed/retried replan keeps pulling the vehicle toward the obstacle.
     */
    braking_hold_valid_ = latest_odom_valid_;
    if (braking_hold_valid_) {
      braking_hold_position_ = latest_odom_position_;
      const Eigen::Vector3d body_x =
          latest_odom_body_in_world_.toRotationMatrix().col(0);
      braking_hold_yaw_ = std::atan2(body_x.y(), body_x.x());
    }
    reached_candidate_since_ = ros::Time();
    status_.state = sim2real_planning_msgs::PlannerStatus::HOLDING;
    status_.armable = false;
    status_.reason = "native planner requested trajectory truncation";
    publishStatus();
  }

  void commandTimer(const ros::TimerEvent&) {
    updateReadiness();
    if (!trajectory_valid_ || !has_goal_ || !status_.odom_ready ||
        !status_.map_ready || !coreReady(ros::Time::now()) ||
        !coreProgressReady(ros::Time::now()) ||
        trajectory_.size() != 5U) {
      return;
    }
    const ros::Time now = ros::Time::now();
    const double elapsed = (now - start_time_).toSec();
    if (!finite(elapsed) || elapsed < 0.0) return;
    const double sample_time = std::min(elapsed, effective_duration_);

    Eigen::Vector3d position = trajectory_[0].evaluateDeBoorT(sample_time);
    Eigen::Vector3d velocity = trajectory_[1].evaluateDeBoorT(sample_time);
    Eigen::Vector3d acceleration = trajectory_[2].evaluateDeBoorT(sample_time);
    double yaw = trajectory_[3].evaluateDeBoorT(sample_time)[0];
    double yaw_rate = trajectory_[4].evaluateDeBoorT(sample_time)[0];
    if (braking_ && braking_hold_valid_) {
      position = braking_hold_position_;
      yaw = braking_hold_yaw_;
      velocity.setZero();
      acceleration.setZero();
      yaw_rate = 0.0;
    } else if (elapsed >= effective_duration_) {
      velocity.setZero();
      acceleration.setZero();
      yaw_rate = 0.0;
    }
    if (!finiteTrajectorySetpoint(
            position, velocity, acceleration, yaw, yaw_rate)) {
      setFault("native trajectory evaluated to a non-finite setpoint");
      return;
    }
    if (inject_virtual_floor_ &&
        !goalAboveVirtualFloor(
            position.z(), virtual_floor_height_, inflation_z_, 0.0,
            map_resolution_, nullptr)) {
      setFault("native trajectory crossed the Fast virtual floor");
      plan_manage::FastPlannerGoal cancel;
      cancel.header.stamp = now;
      cancel.goal_id = current_goal_.goal_id;
      cancel.action = plan_manage::FastPlannerGoal::CANCEL;
      native_goal_pub_.publish(cancel);
      return;
    }
    sim2real_planning_msgs::PlannerCommand command;
    command.header.stamp = now;
    command.header.frame_id = "world";
    command.session_id = current_goal_.session_id;
    command.backend_id = backend_id_;
    command.goal_id = current_goal_.goal_id;
    command.trajectory_id = last_trajectory_id_;
    command.mode = braking_ ? sim2real_planning_msgs::PlannerCommand::BRAKE :
        (elapsed >= full_duration_ ? sim2real_planning_msgs::PlannerCommand::HOLD :
                                     sim2real_planning_msgs::PlannerCommand::NORMAL);

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
    command.point.time_from_start = ros::Duration(std::max(0.0, elapsed));
    command_pub_.publish(command);

    if (!braking_ && elapsed >= full_duration_ &&
        status_.state != sim2real_planning_msgs::PlannerStatus::REACHED) {
      const bool measured_arrival =
          latest_odom_valid_ &&
          measuredStateSatisfiesGoal(
              latest_odom_position_, latest_odom_world_velocity_,
              latest_odom_body_in_world_, latest_odom_world_angular_velocity_,
              status_.active_goal, current_goal_.constrain_yaw,
              goal_position_tolerance_, reached_velocity_tolerance_,
              reached_yaw_tolerance_, reached_yaw_rate_tolerance_);
      if (!measured_arrival) {
        reached_candidate_since_ = ros::Time();
        status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
        status_.armable = true;
        status_.reason =
            "trajectory endpoint is holding; waiting for measured arrival";
      } else {
        if (reached_candidate_since_.isZero() || now < reached_candidate_since_) {
          reached_candidate_since_ = now;
        }
        if ((now - reached_candidate_since_).toSec() >= reached_hold_time_) {
          status_.state = sim2real_planning_msgs::PlannerStatus::REACHED;
          status_.armable = true;
          status_.reason = "measured vehicle state reached the active goal";
          publishStatus();
        } else {
          status_.state = sim2real_planning_msgs::PlannerStatus::ACTIVE;
          status_.armable = true;
          status_.reason = "measured arrival is stabilizing";
        }
      }
    } else if (elapsed < full_duration_) {
      reached_candidate_since_ = ros::Time();
    }
  }

  void updateReadiness() {
    const ros::Time now = ros::Time::now();
    status_.odom_ready =
        !last_odom_receipt_.isZero() && (now - last_odom_receipt_).toSec() <= odom_timeout_;
    status_.map_ready = !last_cloud_receipt_.isZero() && !first_cloud_receipt_.isZero() &&
        obstacle_validation_ready_ &&
        (now - last_cloud_receipt_).toSec() <= cloud_timeout_ &&
        (now - first_cloud_receipt_).toSec() >= map_settle_time_;
  }

  void statusTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (!planning_started_at_.isZero() &&
        (now - planning_started_at_).toSec() > planning_timeout_) {
      planning_started_at_ = ros::Time();
      setFault("Fast planner did not produce a trajectory before planning timeout");
      plan_manage::FastPlannerGoal cancel;
      cancel.header.stamp = now;
      cancel.goal_id = current_goal_.goal_id;
      cancel.action = plan_manage::FastPlannerGoal::CANCEL;
      native_goal_pub_.publish(cancel);
      return;
    }
    const bool previously_ready = runtime_ready_;
    updateReadiness();
    const bool core_ready = coreReady(now);
    const bool progress_ready = coreProgressReady(now);
    const bool goal_link_ready = native_goal_pub_.getNumSubscribers() > 0;
    const bool output_links_ready =
        bspline_sub_.getNumPublishers() > 0 && replan_sub_.getNumPublishers() > 0;
    const bool ready = status_.odom_ready && status_.map_ready && core_ready &&
                       progress_ready && goal_link_ready && output_links_ready;
    runtime_ready_ = ready;
    if (ready) ever_runtime_ready_ = true;
    if (!ready && previously_ready && trajectory_valid_) {
      trajectory_valid_ = false;
      braking_ = false;
      braking_hold_valid_ = false;
      status_.state = sim2real_planning_msgs::PlannerStatus::FAULT;
      status_.armable = false;
      status_.reason =
          "sensor stream or Fast core heartbeat became stale; old trajectory invalidated";
      plan_manage::FastPlannerGoal cancel;
      cancel.header.stamp = ros::Time::now();
      cancel.goal_id = current_goal_.goal_id;
      cancel.action = plan_manage::FastPlannerGoal::CANCEL;
      native_goal_pub_.publish(cancel);
    } else if ((!core_ready || !progress_ready || !goal_link_ready ||
                !output_links_ready) &&
               ever_runtime_ready_) {
      status_.state = sim2real_planning_msgs::PlannerStatus::FAULT;
      status_.armable = false;
      status_.reason =
          "Fast planner liveness, FSM progress, or native link is stale";
    } else if (ready && !has_goal_) {
      status_.state = sim2real_planning_msgs::PlannerStatus::READY;
      status_.armable = false;
      status_.reason = "ready";
    } else if (!ready && !has_goal_) {
      status_.state = ever_runtime_ready_
          ? sim2real_planning_msgs::PlannerStatus::FAULT
          : sim2real_planning_msgs::PlannerStatus::STARTING;
      status_.armable = false;
      status_.reason =
          "waiting for fresh odometry, point cloud, planner heartbeat, and native links";
    }
    dispatchPendingGoalIfReady();
    publishStatus();
  }

  void setFault(const std::string& reason) {
    trajectory_valid_ = false;
    braking_ = false;
    braking_hold_valid_ = false;
    reached_candidate_since_ = ros::Time();
    status_.state = sim2real_planning_msgs::PlannerStatus::FAULT;
    status_.armable = false;
    status_.reason = reason;
    ROS_ERROR_STREAM(reason);
    publishStatus();
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
  ros::Publisher visualization_occupancy_pub_;
  ros::Publisher visualization_inflated_occupancy_pub_;
  ros::Publisher command_pub_;
  ros::Publisher status_pub_;
  ros::Publisher capabilities_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber bspline_sub_;
  ros::Subscriber replan_sub_;
  ros::Subscriber heartbeat_sub_;
  ros::Subscriber progress_sub_;
  ros::ServiceServer validate_server_;
  ros::Timer command_timer_;
  ros::Timer status_timer_;

  std::string backend_id_;
  std::string backend_namespace_;
  std::string profile_;
  std::string runtime_mode_;
  std::string odom_topic_;
  std::string cloud_topic_;
  int planner_ = 1;
  double odom_timeout_ = 0.5;
  double cloud_timeout_ = 1.0;
  bool self_filter_enabled_ = true;
  double self_filter_radius_ = 0.35;
  double self_filter_min_z_ = -0.20;
  double self_filter_max_z_ = 0.20;
  double self_filter_pose_tolerance_ = 0.10;
  bool hide_observed_ground_plane_ = true;
  double ground_plane_distance_tolerance_ = 0.03;
  double ground_plane_max_tilt_ = 15.0 * std::acos(-1.0) / 180.0;
  int ground_plane_min_points_ = 200;
  double ground_plane_min_xy_span_ = 2.0;
  int ground_plane_ransac_iterations_ = 96;
  double heartbeat_timeout_ = 0.25;
  double progress_timeout_ = 0.25;
  double planning_timeout_ = 10.0;
  double map_settle_time_ = 0.1;
  double command_rate_ = 100.0;
  double status_rate_ = 20.0;
  double goal_position_tolerance_ = 0.35;
  double reached_velocity_tolerance_ = 0.2;
  double reached_yaw_tolerance_ = 5.0 * std::acos(-1.0) / 180.0;
  double reached_yaw_rate_tolerance_ = 10.0 * std::acos(-1.0) / 180.0;
  double reached_hold_time_ = 0.5;
  bool inject_virtual_floor_ = false;
  double virtual_floor_height_ = 0.0;
  double max_velocity_ = 0.5;
  double max_acceleration_ = 0.8;
  double manager_clearance_threshold_ = 0.25;
  double topo_clearance_ = 0.3;
  double goal_clearance_ = 0.25;
  double inflation_ = 0.1;
  double inflation_z_ = 0.1;
  bool accumulate_cloud_ = false;
  double map_resolution_ = -1.0;
  Eigen::Vector3d map_origin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_size_ = Eigen::Vector3d::Constant(-1.0);
  Eigen::Vector3d local_update_range_ = Eigen::Vector3d::Constant(-1.0);

  sim2real_planning_msgs::PlannerGoal current_goal_;
  sim2real_planning_msgs::PlannerStatus status_;
  bool has_goal_ = false;
  bool pending_goal_ = false;
  bool trajectory_valid_ = false;
  bool braking_ = false;
  bool braking_hold_valid_ = false;
  uint64_t last_trajectory_id_ = 0;
  ros::Time last_odom_receipt_;
  ros::Time last_cloud_receipt_;
  ros::Time last_odom_measurement_;
  ros::Time last_cloud_measurement_;
  ros::Time first_cloud_receipt_;
  ros::Time last_heartbeat_receipt_;
  ros::Time last_progress_receipt_;
  ros::Time busy_progress_started_at_;
  ros::Time planning_started_at_;
  ros::Time reached_candidate_since_;
  ros::Time start_time_;
  double full_duration_ = 0.0;
  double effective_duration_ = 0.0;
  bool ever_runtime_ready_ = false;
  bool runtime_ready_ = false;
  bool core_planning_busy_ = false;
  bool latest_odom_valid_ = false;
  bool obstacle_validation_ready_ = false;
  Eigen::Vector3d latest_odom_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_odom_world_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond latest_odom_body_in_world_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d latest_odom_world_angular_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d braking_hold_position_ = Eigen::Vector3d::Zero();
  double braking_hold_yaw_ = 0.0;
  bool visualization_ground_plane_valid_ = false;
  Eigen::Vector4d visualization_ground_plane_ =
      Eigen::Vector4d::Zero();
  std::deque<StampedBodyPose> odom_history_;
  std::vector<NonUniformBspline> trajectory_;
  std::unique_ptr<RealObstacleVisualizationMap> visualization_map_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_backend_adapter");
  try {
    FastBackendAdapter adapter;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("Fast planner adapter failed to initialize: " << error.what());
    return 2;
  }
  return 0;
}
