/**
 * ref: se3_example.cpp
 * @author tfly
 */

#include "se3_controller/se3_ctrl.h"

#include <cmath>
#include <stdexcept>

constexpr double SE3_CONTROLLER::kCtrlDt_;

se3Ctrl::se3Ctrl(const ros::NodeHandle &nh):nh_(nh)
{
    cmd_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    desire_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/desire_odom_pub", 10);
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_service_ = nh_.advertiseService("/land", &se3Ctrl::landCallback, this);

    nh_.param<std::string>(
        "odometry_topic", odometry_topic_, "/localization/odom");
    nh_.param<std::string>(
        "local_odometry_topic", local_odometry_topic_,
        "/mavros/local_position/odom");
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        odometry_topic_, 10, &se3Ctrl::OdomCallback, this);
    local_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        local_odometry_topic_, 10, &se3Ctrl::LocalOdomCallback, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &se3Ctrl::IMUCallback, this);
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &se3Ctrl::StateCallback, this);
    multiDOFJoint_sub_ = nh_.subscribe("/command/trajectory", 10, &se3Ctrl::multiDOFJointCallback, this);

    exec_timer_ = nh_.createTimer(ros::Duration(0.01), &se3Ctrl::execFSMCallback, this);

    nh_.param<bool>("enable_sim", sim_enable_, false);
    nh_.param<bool>("auto_request_offboard", auto_request_offboard_, false);
    nh_.param<bool>("auto_request_arm", auto_request_arm_, false);
    nh_.param<bool>("auto_land_on_geofence", auto_land_on_geofence_, false);
    nh_.param<bool>("enable_thrust_estimation", enable_thrust_estimation_, false);
    nh_.param<bool>("use_acceleration_feedforward", use_acceleration_feedforward_, true);
    nh_.param<bool>("use_yaw_rate_feedforward", use_yaw_rate_feedforward_, true);
    nh_.param<bool>(
        "align_attitude_with_imu", align_attitude_with_imu_, true);
    nh_.param<double>("max_feedforward_acc", max_feedforward_acc_, 2.0);
    nh_.param<double>("odom_timeout", odom_timeout_, 0.2);
    nh_.param<double>("imu_timeout", imu_timeout_, 0.2);
    nh_.param<double>("state_timeout", state_timeout_, 2.0);
    nh_.param<double>(
        "trajectory_command_timeout", trajectory_command_timeout_, 0.08);
    nh_.param<double>(
        "attitude_handoff_duration", attitude_handoff_duration_, 1.5);
    nh_.param<double>(
        "max_attitude_alignment_error_deg",
        max_attitude_alignment_error_deg_, 10.0);
    nh_.param<std::string>(
        "command_publisher_node", command_publisher_node_,
        "/planner_gateway");
    nh_.param<double>("land_retry_interval", land_retry_interval_, 1.0);
    nh_.param<double>(
        "safety_hold_retry_interval", safety_hold_retry_interval_, 1.0);
    nh_.param<double>("hover_percent", hover_percent_, 0.45);
    nh_.param<double>("max_hover_percent", max_hover_percent_, 0.75);
    nh_.param<double>("min_output_thrust", min_output_thrust_, 0.20);
    nh_.param<double>("max_output_thrust", max_output_thrust_, 0.85);
    nh_.param<double>("geo_fence/x", geo_fence_[0], 10.0);
    nh_.param<double>("geo_fence/y", geo_fence_[1], 10.0);
    nh_.param<double>("geo_fence/z", geo_fence_[2], 4.0);
    nh_.param<double>("ki_pz", ki_pz_, 0.0);
    nh_.param<double>("int_limit_z", int_limit_z_, 5.0);

    if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0) {
        ROS_WARN("[se3_controller] Invalid odom_timeout=%.3f; using 0.2 s.", odom_timeout_);
        odom_timeout_ = 0.2;
    }
    if (!std::isfinite(imu_timeout_) || imu_timeout_ <= 0.0) {
        ROS_WARN("[se3_controller] Invalid imu_timeout=%.3f; using 0.2 s.", imu_timeout_);
        imu_timeout_ = 0.2;
    }
    if (!std::isfinite(state_timeout_) || state_timeout_ <= 0.0) {
        ROS_WARN("[se3_controller] Invalid state_timeout=%.3f; using 2.0 s.", state_timeout_);
        state_timeout_ = 2.0;
    }
    if (!std::isfinite(trajectory_command_timeout_) ||
        trajectory_command_timeout_ <= 0.0) {
        ROS_WARN(
            "[se3_controller] Invalid trajectory_command_timeout=%.3f; "
            "using 0.08 s.",
            trajectory_command_timeout_);
        trajectory_command_timeout_ = 0.08;
    }
    if (!std::isfinite(attitude_handoff_duration_) ||
        attitude_handoff_duration_ < 0.5) {
        ROS_WARN(
            "[se3_controller] Invalid attitude_handoff_duration=%.3f; "
            "using 1.5 s.",
            attitude_handoff_duration_);
        attitude_handoff_duration_ = 1.5;
    }
    if (!std::isfinite(max_attitude_alignment_error_deg_) ||
        max_attitude_alignment_error_deg_ < 1.0 ||
        max_attitude_alignment_error_deg_ > 45.0) {
        ROS_WARN(
            "[se3_controller] Invalid max_attitude_alignment_error_deg=%.3f; "
            "using 10.0 deg.",
            max_attitude_alignment_error_deg_);
        max_attitude_alignment_error_deg_ = 10.0;
    }
    if (command_publisher_node_.empty() ||
        command_publisher_node_.front() != '/') {
        ROS_WARN(
            "[se3_controller] Invalid command_publisher_node='%s'; "
            "using /planner_gateway.",
            command_publisher_node_.c_str());
        command_publisher_node_ = "/planner_gateway";
    }
    if (!std::isfinite(land_retry_interval_) || land_retry_interval_ <= 0.0) {
        ROS_WARN("[se3_controller] Invalid land_retry_interval=%.3f; using 1.0 s.",
                 land_retry_interval_);
        land_retry_interval_ = 1.0;
    }
    if (!std::isfinite(safety_hold_retry_interval_) ||
        safety_hold_retry_interval_ <= 0.0) {
        ROS_WARN("[se3_controller] Invalid safety_hold_retry_interval=%.3f; using 1.0 s.",
                 safety_hold_retry_interval_);
        safety_hold_retry_interval_ = 1.0;
    }
    if (!std::isfinite(max_feedforward_acc_) || max_feedforward_acc_ < 0.0) {
        ROS_WARN("[se3_controller] Invalid max_feedforward_acc=%.3f; using 2.0 m/s^2.",
                 max_feedforward_acc_);
        max_feedforward_acc_ = 2.0;
    }
    if (!std::isfinite(hover_percent_) || hover_percent_ <= 0.0 ||
        hover_percent_ > 1.0) {
        ROS_WARN("[se3_controller] Invalid hover_percent=%.3f; using 0.45.",
                 hover_percent_);
        hover_percent_ = 0.45;
    }
    if (!std::isfinite(max_hover_percent_) ||
        max_hover_percent_ < hover_percent_ || max_hover_percent_ > 1.0) {
        ROS_WARN("[se3_controller] Invalid max_hover_percent=%.3f; using %.3f.",
                 max_hover_percent_, std::max(hover_percent_, 0.75));
        max_hover_percent_ = std::max(hover_percent_, 0.75);
    }
    if (!std::isfinite(min_output_thrust_) || min_output_thrust_ < 0.0 ||
        min_output_thrust_ >= 1.0) {
        ROS_WARN("[se3_controller] Invalid min_output_thrust=%.3f; using 0.20.",
                 min_output_thrust_);
        min_output_thrust_ = 0.20;
    }
    if (!std::isfinite(max_output_thrust_) ||
        max_output_thrust_ <= min_output_thrust_ ||
        max_output_thrust_ > 1.0) {
        const double safe_max =
            std::max(0.85, 0.5 * (1.0 + min_output_thrust_));
        ROS_WARN("[se3_controller] Invalid max_output_thrust=%.3f; using %.3f.",
                 max_output_thrust_, safe_max);
        max_output_thrust_ = safe_max;
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(geo_fence_[axis]) || geo_fence_[axis] <= 0.0) {
            ROS_WARN("[se3_controller] Invalid geofence axis %d value %.3f; using 10.0 m.",
                     axis, geo_fence_[axis]);
            geo_fence_[axis] = 10.0;
        }
    }
    if (!std::isfinite(ki_pz_)) {
        ROS_WARN("[se3_controller] Invalid ki_pz; disabling integral gain.");
        ki_pz_ = 0.0;
    }
    if (!std::isfinite(int_limit_z_) || int_limit_z_ < 0.0) {
        ROS_WARN("[se3_controller] Invalid int_limit_z; using 0.0.");
        int_limit_z_ = 0.0;
    }
    attitude_handoff_start_thrust_ = hover_percent_;

    enu_frame_ = true;
    vel_in_body_ = true;

    node_state_ = WAITING_FOR_CONNECTED;

    kp_p_ << 0.85, 0.85, 1.5;
    kp_v_ << 1.5, 1.5, 1.5;
    kp_a_ << 1.5, 1.5, 1.5;
    kp_q_ << 5.5, 5.5, 0.1;
    kp_w_ << 1.5, 1.5, 0.1;

    kd_p_ << 0.1, 0.1, 0.0;
    kd_v_ << 0.0, 0.0, 0.0;
    kd_a_ << 0.0, 0.0, 0.0;
    kd_q_ << 0.0, 0.0, 0.0;
    kd_w_ << 0.0, 0.0, 0.0;

    limit_err_p_ = 3.0;
    limit_err_v_ = 2.0;
    limit_err_a_ = 1.0;
    limit_d_err_p_ = 3.5;
    limit_d_err_v_ = 1.0;
    limit_d_err_a_ = 1.0;

    ROS_INFO("[se3_controller] thrust params: hover_percent=%.3f max_hover_percent=%.3f min_output_thrust=%.3f max_output_thrust=%.3f",
             hover_percent_, max_hover_percent_, min_output_thrust_, max_output_thrust_);
    ROS_INFO("[se3_controller] enable_thrust_estimation=%s", enable_thrust_estimation_ ? "true" : "false");
    ROS_INFO("[se3_controller] feedforward: acceleration=%s yaw_rate=%s max_acc=%.3f",
             use_acceleration_feedforward_ ? "true" : "false",
             use_yaw_rate_feedforward_ ? "true" : "false",
             max_feedforward_acc_);
    ROS_INFO(
        "[se3_controller] attitude reference: %s",
        align_attitude_with_imu_
            ? "latched IMU-to-external-world alignment"
            : "direct desired attitude (already in the FCU frame)");
    ROS_INFO(
        "[se3_controller] input timeouts: state=%.3f odom=%.3f imu=%.3f "
        "trajectory=%.3f s",
        state_timeout_, odom_timeout_, imu_timeout_,
        trajectory_command_timeout_);
    ROS_INFO(
        "[se3_controller] frames: control odometry=%s, PX4 local hold=%s, "
        "attitude handoff=%.2f s, alignment limit=%.1f deg",
        odometry_topic_.c_str(), local_odometry_topic_.c_str(),
        attitude_handoff_duration_, max_attitude_alignment_error_deg_);

    se3_controller_.init(hover_percent_, max_hover_percent_, min_output_thrust_, max_output_thrust_, enu_frame_, vel_in_body_);
    if (!se3_controller_.setup(kp_p_, kp_v_, kp_a_, kp_q_, kp_w_,
                               kd_p_, kd_v_, kd_a_, kd_q_, kd_w_,
                               limit_err_p_, limit_err_v_, limit_err_a_,
                               limit_d_err_p_, limit_d_err_v_, limit_d_err_a_)) {
        ROS_FATAL("[se3_controller] Internal default gains are invalid.");
        throw std::runtime_error("invalid SE3 controller defaults");
    }
    se3_controller_.setIntegral(Eigen::Vector3d(0.0, 0.0, ki_pz_), int_limit_z_);

    // Register dynamic reconfigure only after the controller has a complete,
    // validated initial state. setCallback invokes the callback immediately.
    dynamic_tune_cb_type_ =
        boost::bind(&se3Ctrl::DynamicTuneCallback, this, _1, _2);
    dynamic_tune_server_.setCallback(dynamic_tune_cb_type_);
}


void se3Ctrl::execFSMCallback(const ros::TimerEvent &e){
    switch (node_state_)
    {
    case WAITING_FOR_CONNECTED:{
        if (!hasFreshState()) {
            ROS_INFO_THROTTLE(
                2.0,
                "[se3_controller] Waiting for a fresh connected MAVROS state.");
            break;
        }

        ROS_INFO("MAVROS connected.");
        node_state_ = WAITING_FOR_OFFBOARD;
        break;
    }
    case WAITING_FOR_OFFBOARD:{
        if (!hasFreshState()) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] MAVROS state is stale; not requesting OFFBOARD/arm.");
            break;
        }
        if (currState_.armed &&
            currState_.mode == mavros_msgs::State::MODE_PX4_LAND) {
            node_state_ = LANDED;
            break;
        }
        if (!hasFreshOdom() || !hasFreshLocalOdom() || !hasFreshImu()) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] No fresh valid world odometry, PX4 local odometry, or IMU; not publishing an OFFBOARD warmup setpoint or requesting OFFBOARD/arm.");
            break;
        }
        local_hold_position_ = local_odom_data_.p;
        local_hold_orientation_ = local_odom_data_.q.normalized();
        has_local_hold_position_ = true;
        pubLocalPose(local_hold_position_, local_hold_orientation_);
        setDesiredStateToCurrentOdom();
        trigger_offboard();
        trigger_arm();
        if(currState_.mode == "OFFBOARD" && currState_.armed){
            has_trajectory_after_offboard_ = false;
            captureLocalHoldPosition();
            resetAttitudeHandoff();
            setDesiredStateToCurrentOdom();
            ROS_INFO(
                "OFFBOARD entered in PX4 local-position hold. Raw SE3 "
                "attitude control remains disabled until a fresh planner "
                "trajectory is received.");
            node_state_ = MISSION_EXECUTION;
            // last_ = ros::Time::now();
        }
        break;
    } 
    case MISSION_EXECUTION:{
        if (!hasFreshState()) {
            has_trajectory_after_offboard_ = false;
            resetAttitudeHandoff();
            se3_controller_.resetIntegral();
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] MAVROS state is stale; suppressing attitude/thrust output and relying on PX4 OFFBOARD-loss failsafe.");
            return;
        }

        if (has_trajectory_after_offboard_ &&
            !last_trajectory_command_wall_time_.isZero() &&
            (ros::WallTime::now() - last_trajectory_command_wall_time_).toSec()
                > trajectory_command_timeout_) {
            has_trajectory_after_offboard_ = false;
            resetAttitudeHandoff();
            captureLocalHoldPosition();
            setDesiredStateToCurrentOdom();
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] Planner command stream timed out; replacing "
                "raw attitude control with a PX4 local-position hold at the "
                "current pose. "
                "A later fresh gateway command may resume OFFBOARD.");
        }
        if(currState_.mode != "OFFBOARD" || !currState_.armed){
            has_trajectory_after_offboard_ = false;
            resetAttitudeHandoff();
            se3_controller_.resetIntegral();
            if (hasFreshOdom() && hasFreshLocalOdom() && hasFreshImu()) {
                local_hold_position_ = local_odom_data_.p;
                local_hold_orientation_ = local_odom_data_.q.normalized();
                has_local_hold_position_ = true;
                pubLocalPose(local_hold_position_, local_hold_orientation_);
                setDesiredStateToCurrentOdom();
            } else {
                ROS_ERROR_THROTTLE(1.0, "[se3_controller] No fresh world odometry, PX4 local odometry, or IMU; not publishing an OFFBOARD warmup setpoint.");
            }
            return;
        }

        if (!hasFreshOdom() || !hasFreshLocalOdom() || !hasFreshImu()) {
            has_trajectory_after_offboard_ = false;
            resetAttitudeHandoff();
            se3_controller_.resetIntegral();
            if (hasFreshOdom()) {
                setDesiredStateToCurrentOdom();
            }
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] World odometry, PX4 local odometry, or IMU is stale/invalid; suppressing control output and requesting AUTO.LOITER.");
            requestSafetyHold("stale or invalid world/local odometry or IMU");
            return;
        }

        if (!has_trajectory_after_offboard_) {
            if (!has_local_hold_position_ && !captureLocalHoldPosition()) {
                requestSafetyHold("PX4 local hold position is unavailable");
                return;
            }
            // Keep PX4's own position loop active while the vehicle is merely
            // waiting for a goal. This avoids switching from AUTO.TAKEOFF to a
            // raw attitude target and makes the Takeoff action bumpless.
            pubLocalPose(local_hold_position_, local_hold_orientation_);
            setDesiredStateToCurrentOdom();
            desire_odom_pub_.publish(desire_odom_);
            return;
        }

        if (!attitudeAlignmentIsStable()) {
            has_trajectory_after_offboard_ = false;
            captureLocalHoldPosition();
            resetAttitudeHandoff();
            setDesiredStateToCurrentOdom();
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] IMU-to-world attitude alignment changed "
                "beyond the configured limit; refusing raw attitude output.");
            requestSafetyHold("external-odometry attitude alignment changed");
            return;
        }

        double control_dt = (e.current_real - e.last_real).toSec();
        if (!std::isfinite(control_dt) || control_dt <= 0.0 ||
            control_dt > 0.2) {
            control_dt = 0.01;
        }
        Controller_Output_t output;
        if(se3_controller_.calControl(
               odom_data_, imu_data_, desired_state_, output,
               odom_timeout_, imu_timeout_, control_dt,
               align_attitude_with_imu_, attitude_alignment_q_)){
            applyAttitudeHandoff(output);
            if (!send_cmd(output, true)) {
                requestSafetyHold("non-finite controller output");
                return;
            }
            desire_odom_pub_.publish(desire_odom_);
            if (enable_thrust_estimation_) {
                se3_controller_.estimateTa(imu_data_.a);
            }
        } else {
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] Control input/output validation failed; suppressing attitude/thrust output and requesting AUTO.LOITER.");
            se3_controller_.resetIntegral();
            requestSafetyHold("control validation failure");
        }
        break;
    }
    case LANDING: {
        if (!hasFreshState()) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] Waiting for fresh MAVROS state before retrying AUTO.LAND.");
            break;
        }
        if (!currState_.armed) {
            ROS_INFO("Vehicle is disarmed; landing request is complete.");
            resetForDisarmedState();
            node_state_ = WAITING_FOR_OFFBOARD;
            break;
        }
        if (currState_.mode == mavros_msgs::State::MODE_PX4_LAND) {
            if (!land_mode_request_accepted_) {
                ROS_INFO("AUTO.LAND is active.");
            }
            land_mode_request_accepted_ = true;
            node_state_ = LANDED;
            break;
        }

        const ros::Time now = ros::Time::now();
        if (!last_land_mode_request_.isZero() &&
            (now - last_land_mode_request_).toSec() < land_retry_interval_) {
            break;
        }
        last_land_mode_request_ = now;
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = mavros_msgs::State::MODE_PX4_LAND;
        if(set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent){
            land_mode_request_accepted_ = true;
            ROS_INFO("PX4 accepted AUTO.LAND; waiting for MAVROS state confirmation.");
        } else {
            land_mode_request_accepted_ = false;
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] PX4 rejected AUTO.LAND; retrying every %.1f s.",
                land_retry_interval_);
        }
        break;
    }
    case LANDED:
        if (!hasFreshState()) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] MAVROS state became stale while landing; no control output is being published.");
            break;
        }
        if(!currState_.armed){
            resetForDisarmedState();
            ROS_INFO("Landed and disarmed. SE3 controller is ready for a new takeoff cycle.");
            node_state_ = WAITING_FOR_OFFBOARD;
        } else if (currState_.mode != mavros_msgs::State::MODE_PX4_LAND) {
            ROS_WARN(
                "[se3_controller] Vehicle is still armed but left AUTO.LAND; requesting AUTO.LAND again.");
            land_mode_request_accepted_ = false;
            last_land_mode_request_ = ros::Time(0);
            node_state_ = LANDING;
        }
        break;
    default:
        break;
    }
}

bool se3Ctrl::send_cmd(const Controller_Output_t &output, bool angle){
    if (!se3_safety::isQuaternionValid(output.q) ||
        !output.bodyrates.allFinite() ||
        !std::isfinite(output.thrust) ||
        output.thrust < 0.0 || output.thrust > 1.0) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Refusing to publish a non-finite/out-of-range attitude target.");
        return false;
    }

    mavros_msgs::AttitudeTarget cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.body_rate.x = output.bodyrates(0);
    cmd.body_rate.y = output.bodyrates(1);
    cmd.body_rate.z = output.bodyrates(2);
    cmd.orientation.w = output.q.w();
    cmd.orientation.x = output.q.x();
    cmd.orientation.y = output.q.y();
    cmd.orientation.z = output.q.z();
    cmd.thrust = output.thrust;
    if(angle){
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE + 
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE + 
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    }else{
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    }
    cmd_pub_.publish(cmd);
    return true;
}

void se3Ctrl::pubLocalPose(
    const Eigen::Vector3d &pose,
    const Eigen::Quaterniond &orientation)
{
    if (!pose.allFinite() || !se3_safety::isQuaternionValid(orientation)) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Refusing to publish an invalid PX4 local hold pose.");
        return;
    }
    const Eigen::Quaterniond normalized_orientation = orientation.normalized();
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    msg.pose.position.z = pose[2];
    msg.pose.orientation.w = normalized_orientation.w();
    msg.pose.orientation.x = normalized_orientation.x();
    msg.pose.orientation.y = normalized_orientation.y();
    msg.pose.orientation.z = normalized_orientation.z();

    local_pos_pub_.publish(msg);
}

bool se3Ctrl::hasFreshOdom() const
{
    return has_odom_ && odom_data_.isFresh(odom_timeout_);
}

bool se3Ctrl::hasFreshLocalOdom() const
{
    return has_local_odom_ && local_odom_data_.isFresh(odom_timeout_);
}

bool se3Ctrl::hasFreshImu() const
{
    return has_imu_ && imu_data_.isFresh(imu_timeout_);
}

bool se3Ctrl::hasFreshState() const
{
    return has_state_ && currState_.connected &&
           se3_safety::isFreshAt(
               state_rcv_stamp_, state_msg_stamp_,
               ros::Time::now(), state_timeout_);
}

void se3Ctrl::requestSafetyHold(const char *reason)
{
    if (!hasFreshState() || !currState_.armed ||
        currState_.mode != "OFFBOARD") {
        return;
    }

    const ros::Time now = ros::Time::now();
    if (!last_safety_hold_request_.isZero() &&
        (now - last_safety_hold_request_).toSec() <
            safety_hold_retry_interval_) {
        return;
    }
    last_safety_hold_request_ = now;

    mavros_msgs::SetMode hold_mode;
    hold_mode.request.custom_mode = "AUTO.LOITER";
    if (set_mode_client_.call(hold_mode) && hold_mode.response.mode_sent) {
        ROS_WARN("[se3_controller] PX4 accepted AUTO.LOITER after %s.", reason);
    } else {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] PX4 did not accept AUTO.LOITER after %s; attitude/thrust remains suppressed so PX4 OFFBOARD-loss failsafe can act.",
            reason);
    }
}

void se3Ctrl::resetForDisarmedState()
{
    has_trajectory_after_offboard_ = false;
    has_local_hold_position_ = false;
    resetAttitudeHandoff();
    land_mode_request_accepted_ = false;
    last_land_mode_request_ = ros::Time(0);
    last_safety_hold_request_ = ros::Time(0);
    last_offboard_request_ = ros::Time(0);
    last_arm_request_ = ros::Time(0);
    last_trajectory_command_wall_time_ = ros::WallTime();
    se3_controller_.resetIntegral();
    if (hasFreshOdom()) {
        setDesiredStateToCurrentOdom();
    }
}

bool se3Ctrl::captureLocalHoldPosition()
{
    if (!hasFreshLocalOdom() || !local_odom_data_.p.allFinite()) {
        return false;
    }
    local_hold_position_ = local_odom_data_.p;
    local_hold_orientation_ = local_odom_data_.q.normalized();
    has_local_hold_position_ = true;
    return true;
}

void se3Ctrl::resetAttitudeHandoff()
{
    attitude_handoff_active_ = false;
    attitude_alignment_valid_ = false;
    attitude_handoff_started_at_ = ros::WallTime();
    attitude_handoff_start_q_ = Eigen::Quaterniond::Identity();
    attitude_alignment_q_ = Eigen::Quaterniond::Identity();
    attitude_handoff_start_thrust_ = hover_percent_;
}

void se3Ctrl::startAttitudeHandoff()
{
    if (!hasFreshImu() || !se3_safety::isQuaternionValid(imu_data_.q)) {
        resetAttitudeHandoff();
        return;
    }
    attitude_handoff_start_q_ = imu_data_.q.normalized();
    attitude_handoff_start_thrust_ = hover_percent_;
    attitude_alignment_q_ = align_attitude_with_imu_
        ? (imu_data_.q * odom_data_.q.inverse()).normalized()
        : Eigen::Quaterniond::Identity();
    attitude_alignment_valid_ =
        se3_safety::isQuaternionValid(attitude_alignment_q_);
    if (!attitude_alignment_valid_) {
        resetAttitudeHandoff();
        return;
    }
    attitude_handoff_started_at_ = ros::WallTime::now();
    attitude_handoff_active_ = true;
}

void se3Ctrl::applyAttitudeHandoff(Controller_Output_t &output)
{
    if (!attitude_handoff_active_) {
        return;
    }
    const double elapsed =
        (ros::WallTime::now() - attitude_handoff_started_at_).toSec();
    const double progress = elapsed / attitude_handoff_duration_;
    output.q = se3_safety::interpolateAttitude(
        attitude_handoff_start_q_, output.q, progress);
    output.thrust = se3_safety::interpolateScalar(
        attitude_handoff_start_thrust_, output.thrust, progress);
    if (progress >= 1.0) {
        attitude_handoff_active_ = false;
        ROS_INFO(
            "[se3_controller] Bumpless PX4-position to SE3-attitude "
            "handoff completed in %.2f s.",
            elapsed);
    }
}

bool se3Ctrl::attitudeAlignmentIsStable() const
{
    if (!attitude_alignment_valid_ || !hasFreshOdom() || !hasFreshImu()) {
        return false;
    }
    if (!align_attitude_with_imu_) {
        return true;
    }
    const Eigen::Quaterniond live_alignment =
        (imu_data_.q * odom_data_.q.inverse()).normalized();
    const double error_rad = se3_safety::quaternionAngularDistance(
        attitude_alignment_q_, live_alignment);
    return std::isfinite(error_rad) &&
        error_rad <= max_attitude_alignment_error_deg_ * M_PI / 180.0;
}

void se3Ctrl::setDesiredStateToCurrentOdom()
{
    if (!hasFreshOdom() || !odom_data_.isValid()) {
        return;
    }
    se3_controller_.resetIntegral();
    desired_state_.p = odom_data_.p;
    desired_state_.v = Eigen::Vector3d::Zero();
    desired_state_.a.setZero();
    desired_state_.j.setZero();
    desired_state_.q = odom_data_.q.normalized();
    desired_state_.yaw = utils::fromQuaternion2yaw(desired_state_.q);
    desired_state_.yaw_rate = 0.0;

    syncDesiredOdomMessage(ros::Time::now());
}

void se3Ctrl::syncDesiredOdomMessage(const ros::Time &stamp)
{
    // /desire_odom_pub is diagnostic output for RViz and rosbag. Keep it in
    // sync with the exact Desired_State_t consumed by the controller instead
    // of leaving the last pre-OFFBOARD hold pose in the published cache.
    desire_odom_ = nav_msgs::Odometry();
    desire_odom_.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    desire_odom_.header.frame_id = "world";
    desire_odom_.child_frame_id = "base_link";
    desire_odom_.pose.pose.position.x = desired_state_.p(0);
    desire_odom_.pose.pose.position.y = desired_state_.p(1);
    desire_odom_.pose.pose.position.z = desired_state_.p(2);
    desire_odom_.pose.pose.orientation.w = desired_state_.q.w();
    desire_odom_.pose.pose.orientation.x = desired_state_.q.x();
    desire_odom_.pose.pose.orientation.y = desired_state_.q.y();
    desire_odom_.pose.pose.orientation.z = desired_state_.q.z();
    desire_odom_.twist.twist.linear.x = desired_state_.v(0);
    desire_odom_.twist.twist.linear.y = desired_state_.v(1);
    desire_odom_.twist.twist.linear.z = desired_state_.v(2);
    desire_odom_.twist.twist.angular.z = desired_state_.yaw_rate;
}

bool se3Ctrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
    if (!request.data) {
        response.success = false;
        response.message =
            "Landing was not requested; call /land with data=true.";
        return true;
    }
    if (!hasFreshState()) {
        response.success = false;
        response.message =
            "Cannot queue landing without a fresh connected MAVROS state.";
        return true;
    }
    if (!currState_.armed) {
        resetForDisarmedState();
        node_state_ = WAITING_FOR_OFFBOARD;
        response.success = true;
        response.message = "Vehicle is already disarmed.";
        return true;
    }
    if (currState_.mode == mavros_msgs::State::MODE_PX4_LAND) {
        land_mode_request_accepted_ = true;
        node_state_ = LANDED;
        response.success = true;
        response.message = "AUTO.LAND is already active.";
        return true;
    }

    ROS_WARN("Landing requested; attitude/thrust output will stop while AUTO.LAND is requested.");
    land_mode_request_accepted_ = false;
    last_land_mode_request_ = ros::Time(0);
    node_state_ = LANDING;
    response.success = true;
    response.message =
        "Landing request queued; waiting for PX4 AUTO.LAND confirmation.";
    return true;
}

void se3Ctrl::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    if (!msg || msg->header.frame_id != "world" ||
        msg->child_frame_id != "base_link") {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring control odometry outside the required "
            "world -> base_link contract.");
        return;
    }
    const bool recovered_from_stale_odom = has_odom_ && !odom_data_.isFresh(odom_timeout_);
    if (!odom_data_.feed(msg, enu_frame_, vel_in_body_)) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring odometry with non-finite values or an invalid quaternion.");
        return;
    }
    has_odom_ = true;
    if (!hasFreshOdom()) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Received odometry with a stale/future source timestamp.");
        return;
    }
    if (recovered_from_stale_odom) {
        has_trajectory_after_offboard_ = false;
        resetAttitudeHandoff();
        captureLocalHoldPosition();
        setDesiredStateToCurrentOdom();
        ROS_WARN("[se3_controller] World odometry recovered after a timeout. Holding with PX4 local-position control until a fresh trajectory is received.");
    }
    bool judge_x = ((odom_data_.p(0) >= geo_fence_[0]) || (odom_data_.p(0) <= -geo_fence_[0]));
    bool judge_y = ((odom_data_.p(1) >= geo_fence_[1]) || (odom_data_.p(1) <= -geo_fence_[1]));
    bool judge_z = (odom_data_.p(2) >= geo_fence_[2]);
    bool judge = (judge_x || judge_y || judge_z);
    if(judge && !auto_land_on_geofence_){
        ROS_WARN_THROTTLE(1.0, "[se3_controller] Geofence exceeded, but auto_land_on_geofence is disabled. Please take over manually if needed.");
    }
    if(judge && auto_land_on_geofence_ && currState_.mode != mavros_msgs::State::MODE_PX4_LAND){
        if (hasFreshState() && currState_.armed) {
            land_mode_request_accepted_ = false;
            last_land_mode_request_ = ros::Time(0);
            node_state_ = LANDING;
            ROS_WARN_THROTTLE(
                1.0,
                "[se3_controller] Geofence exceeded; requesting AUTO.LAND.");
        }
    }
}

void se3Ctrl::LocalOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    if (!msg || msg->header.frame_id != "map" ||
        msg->child_frame_id != "base_link") {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring PX4 local odometry outside the required "
            "map -> base_link contract.");
        return;
    }
    const bool recovered_from_stale_local_odom =
        has_local_odom_ && !local_odom_data_.isFresh(odom_timeout_);
    if (!local_odom_data_.feed(msg, enu_frame_, vel_in_body_)) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring PX4 local odometry with non-finite values or an invalid quaternion.");
        return;
    }
    has_local_odom_ = true;
    if (!hasFreshLocalOdom()) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Received PX4 local odometry with a stale/future source timestamp.");
        return;
    }
    if (recovered_from_stale_local_odom && !has_trajectory_after_offboard_) {
        captureLocalHoldPosition();
        ROS_WARN(
            "[se3_controller] PX4 local odometry recovered. Captured a new "
            "local-position hold setpoint.");
    }
}

void se3Ctrl::IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
    const bool recovered_from_stale_imu =
        has_imu_ && !imu_data_.isFresh(imu_timeout_);
    if (!imu_data_.feed(msg, enu_frame_)) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring IMU data with non-finite values or an invalid quaternion.");
        return;
    }
    has_imu_ = true;
    if (!hasFreshImu()) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Received IMU data with a stale/future source timestamp.");
        return;
    }
    if (recovered_from_stale_imu) {
        has_trajectory_after_offboard_ = false;
        resetAttitudeHandoff();
        captureLocalHoldPosition();
        if (hasFreshOdom()) {
            setDesiredStateToCurrentOdom();
        } else {
            se3_controller_.resetIntegral();
        }
        ROS_WARN(
            "[se3_controller] IMU recovered after a timeout. Holding the current pose until a fresh trajectory is received.");
    }
}

void se3Ctrl::StateCallback(const mavros_msgs::State::ConstPtr &msg){
    if (!msg) {
        return;
    }
    const bool recovered_from_stale_state =
        has_state_ && !hasFreshState();
    currState_ = *msg;
    state_rcv_stamp_ = ros::Time::now();
    state_msg_stamp_ = msg->header.stamp;
    has_state_ = true;
    if (recovered_from_stale_state && hasFreshState()) {
        has_trajectory_after_offboard_ = false;
        resetAttitudeHandoff();
        captureLocalHoldPosition();
        if (hasFreshOdom()) {
            setDesiredStateToCurrentOdom();
        } else {
            se3_controller_.resetIntegral();
        }
        ROS_WARN(
            "[se3_controller] MAVROS state stream recovered. Holding the current pose until a fresh trajectory is received.");
    }
}

void se3Ctrl::multiDOFJointCallback(
    const ros::MessageEvent<
        trajectory_msgs::MultiDOFJointTrajectory const> &event)
{
    const auto connection_header = event.getConnectionHeaderPtr();
    if (!connection_header) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Rejecting /command/trajectory from an "
            "unauthorized ROS publisher.");
        return;
    }
    const auto caller = connection_header->find("callerid");
    if (caller == connection_header->end() ||
        caller->second != command_publisher_node_) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Rejecting /command/trajectory from ROS node "
            "'%s'; expected '%s'.",
            caller == connection_header->end() ? "<unknown>"
                                                : caller->second.c_str(),
            command_publisher_node_.c_str());
        return;
    }
    const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr message =
        event.getMessage();
    if (!message) {
        return;
    }
    const trajectory_msgs::MultiDOFJointTrajectory &msg = *message;

    if (!hasFreshState() ||
        currState_.mode != "OFFBOARD" || !currState_.armed) {
        ROS_WARN_THROTTLE(1.0, "[se3_controller] Ignoring trajectory until vehicle is armed and in OFFBOARD.");
        return;
    }

    if (!hasFreshOdom() || !hasFreshLocalOdom() || !hasFreshImu()) {
        ROS_WARN_THROTTLE(
            1.0,
            "[se3_controller] Ignoring trajectory because world odometry, PX4 local odometry, or IMU is unavailable, invalid, or stale.");
        return;
    }

    if (msg.header.frame_id != "world") {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring trajectory in frame '%s'; expected 'world'.",
            msg.header.frame_id.c_str());
        return;
    }

    if (msg.points.empty() || msg.points[0].transforms.empty() || msg.points[0].velocities.empty()) {
        ROS_WARN_THROTTLE(1.0, "[se3_controller] Ignoring empty trajectory command.");
        return;
    }

    // command/trajectory
    trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
    Desired_State_t candidate;
    candidate.p = Eigen::Vector3d(
        pt.transforms[0].translation.x,
        pt.transforms[0].translation.y,
        pt.transforms[0].translation.z);
    candidate.v = Eigen::Vector3d(
        pt.velocities[0].linear.x,
        pt.velocities[0].linear.y,
        pt.velocities[0].linear.z);

    if (use_acceleration_feedforward_ && !pt.accelerations.empty()) {
        candidate.a(0) = pt.accelerations[0].linear.x;
        candidate.a(1) = pt.accelerations[0].linear.y;
        candidate.a(2) = pt.accelerations[0].linear.z;
        for (int i = 0; i < 3; ++i) {
            candidate.a(i) = std::max(
                std::min(candidate.a(i), max_feedforward_acc_),
                -max_feedforward_acc_);
        }
    } else {
        candidate.a.setZero();
    }
    candidate.j.setZero();
    candidate.q = Eigen::Quaterniond(
        pt.transforms[0].rotation.w,
        pt.transforms[0].rotation.x,
        pt.transforms[0].rotation.y,
        pt.transforms[0].rotation.z);
    if (!candidate.p.allFinite() || !candidate.v.allFinite() ||
        !candidate.a.allFinite() ||
        !se3_safety::isQuaternionValid(candidate.q)) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring trajectory containing non-finite values or an invalid quaternion.");
        return;
    }
    candidate.q.normalize();
    candidate.yaw = utils::fromQuaternion2yaw(candidate.q);
    candidate.yaw_rate =
        use_yaw_rate_feedforward_ ? pt.velocities[0].angular.z : 0.0;
    if (!candidate.isValid()) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring invalid trajectory command.");
        return;
    }

    desired_state_ = candidate;
    last_trajectory_command_wall_time_ = ros::WallTime::now();
    if (!has_trajectory_after_offboard_) {
        startAttitudeHandoff();
        if (!attitude_handoff_active_) {
            ROS_ERROR(
                "[se3_controller] Fresh trajectory cannot start because the "
                "current IMU attitude is unavailable; remaining in PX4 "
                "local-position hold.");
            return;
        }
        ROS_INFO(
            "[se3_controller] Fresh world-frame trajectory accepted after "
            "OFFBOARD; starting a %.2f s bumpless attitude handoff.",
            attitude_handoff_duration_);
        has_trajectory_after_offboard_ = true;
    }

    syncDesiredOdomMessage(msg.header.stamp);
}
