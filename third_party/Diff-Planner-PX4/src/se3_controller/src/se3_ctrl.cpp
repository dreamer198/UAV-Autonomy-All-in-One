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

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &se3Ctrl::OdomCallback, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &se3Ctrl::IMUCallback, this);
    state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &se3Ctrl::StateCallback, this);
    desire_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/desire_odom", 10, &se3Ctrl::DesireOdomCallback, this);
    multiDOFJoint_sub_ = nh_.subscribe("/command/trajectory", 10, &se3Ctrl::multiDOFJointCallback, this);

    exec_timer_ = nh_.createTimer(ros::Duration(0.01), &se3Ctrl::execFSMCallback, this);

    nh_.param<bool>("enable_sim", sim_enable_, false);
    nh_.param<bool>("auto_request_offboard", auto_request_offboard_, false);
    nh_.param<bool>("auto_request_arm", auto_request_arm_, false);
    nh_.param<bool>("auto_land_on_geofence", auto_land_on_geofence_, false);
    nh_.param<bool>("enable_thrust_estimation", enable_thrust_estimation_, false);
    nh_.param<bool>("use_acceleration_feedforward", use_acceleration_feedforward_, true);
    nh_.param<bool>("use_yaw_rate_feedforward", use_yaw_rate_feedforward_, true);
    nh_.param<double>("max_feedforward_acc", max_feedforward_acc_, 2.0);
    nh_.param<double>("odom_timeout", odom_timeout_, 0.2);
    nh_.param<double>("imu_timeout", imu_timeout_, 0.2);
    nh_.param<double>("state_timeout", state_timeout_, 2.0);
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
    ROS_INFO("[se3_controller] input timeouts: state=%.3f odom=%.3f imu=%.3f s",
             state_timeout_, odom_timeout_, imu_timeout_);

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
        if (!hasFreshOdom() || !hasFreshImu()) {
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] No fresh valid odometry/IMU; not publishing an OFFBOARD warmup setpoint or requesting OFFBOARD/arm.");
            break;
        }
        pubLocalPose(odom_data_.p);
        setDesiredStateToCurrentOdom();
        trigger_offboard();
        trigger_arm();
        if(currState_.mode == "OFFBOARD" && currState_.armed){
            has_trajectory_after_offboard_ = false;
            setDesiredStateToCurrentOdom();
            ROS_INFO("OFFBOARD entered. Holding current pose until a fresh trajectory is received.");
            node_state_ = MISSION_EXECUTION;
            // last_ = ros::Time::now();
        }
        break;
    } 
    case MISSION_EXECUTION:{
        if (!hasFreshState()) {
            has_trajectory_after_offboard_ = false;
            se3_controller_.resetIntegral();
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] MAVROS state is stale; suppressing attitude/thrust output and relying on PX4 OFFBOARD-loss failsafe.");
            return;
        }
        if(currState_.mode != "OFFBOARD" || !currState_.armed){
            has_trajectory_after_offboard_ = false;
            se3_controller_.resetIntegral();
            if (hasFreshOdom() && hasFreshImu()) {
                pubLocalPose(odom_data_.p);
                setDesiredStateToCurrentOdom();
            } else {
                ROS_ERROR_THROTTLE(1.0, "[se3_controller] No fresh odometry/IMU; not publishing an OFFBOARD warmup setpoint.");
            }
            return;
        }

        if (!hasFreshOdom() || !hasFreshImu()) {
            has_trajectory_after_offboard_ = false;
            se3_controller_.resetIntegral();
            if (hasFreshOdom()) {
                setDesiredStateToCurrentOdom();
            }
            ROS_ERROR_THROTTLE(
                1.0,
                "[se3_controller] Odometry or IMU is stale/invalid; suppressing attitude/thrust output and requesting AUTO.LOITER.");
            requestSafetyHold("stale or invalid odometry/IMU");
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
               odom_timeout_, imu_timeout_, control_dt)){
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

void se3Ctrl::pubLocalPose(const Eigen::Vector3d &pose) 
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    msg.pose.position.z = pose[2];
    msg.pose.orientation.w = 1.0;

    local_pos_pub_.publish(msg);
}

bool se3Ctrl::hasFreshOdom() const
{
    return has_odom_ && odom_data_.isFresh(odom_timeout_);
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
    land_mode_request_accepted_ = false;
    last_land_mode_request_ = ros::Time(0);
    last_safety_hold_request_ = ros::Time(0);
    last_offboard_request_ = ros::Time(0);
    last_arm_request_ = ros::Time(0);
    se3_controller_.resetIntegral();
    if (hasFreshOdom()) {
        setDesiredStateToCurrentOdom();
    }
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
    desire_odom_.header.frame_id = "map";
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
        setDesiredStateToCurrentOdom();
        ROS_WARN("[se3_controller] Odometry recovered after a timeout. Holding the recovered pose until a fresh trajectory is received.");
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
        if (hasFreshOdom()) {
            setDesiredStateToCurrentOdom();
        } else {
            se3_controller_.resetIntegral();
        }
        ROS_WARN(
            "[se3_controller] MAVROS state stream recovered. Holding the current pose until a fresh trajectory is received.");
    }
}

void se3Ctrl::DesireOdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
    if (!hasFreshState() || currState_.mode != "OFFBOARD" ||
        !currState_.armed || !hasFreshOdom() || !hasFreshImu()) {
        ROS_WARN_THROTTLE(
            1.0,
            "[se3_controller] Ignoring desired odometry because the vehicle/input safety gate is not ready.");
        return;
    }
    if (!msg) {
        return;
    }

    Desired_State_t candidate;
    candidate.p = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    candidate.v = Eigen::Vector3d(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
    candidate.a.setZero();
    candidate.j.setZero();
    candidate.q = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    if (!candidate.p.allFinite() || !candidate.v.allFinite() ||
        !se3_safety::isQuaternionValid(candidate.q)) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring non-finite desired odometry or invalid orientation.");
        return;
    }
    candidate.q.normalize();
    candidate.yaw = utils::fromQuaternion2yaw(candidate.q);
    candidate.yaw_rate = 0.0;
    if (!candidate.isValid()) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[se3_controller] Ignoring invalid desired odometry.");
        return;
    }

    desired_state_ = candidate;
    has_trajectory_after_offboard_ = true;
    syncDesiredOdomMessage(msg->header.stamp);
}

void se3Ctrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) 
{
    if (!hasFreshState() ||
        currState_.mode != "OFFBOARD" || !currState_.armed) {
        ROS_WARN_THROTTLE(1.0, "[se3_controller] Ignoring trajectory until vehicle is armed and in OFFBOARD.");
        return;
    }

    if (!hasFreshOdom() || !hasFreshImu()) {
        ROS_WARN_THROTTLE(
            1.0,
            "[se3_controller] Ignoring trajectory because odometry/IMU is unavailable, invalid, or stale.");
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
    if (!has_trajectory_after_offboard_) {
        ROS_INFO("[se3_controller] Fresh trajectory accepted after OFFBOARD.");
        has_trajectory_after_offboard_ = true;
    }

    syncDesiredOdomMessage(msg.header.stamp);
}
