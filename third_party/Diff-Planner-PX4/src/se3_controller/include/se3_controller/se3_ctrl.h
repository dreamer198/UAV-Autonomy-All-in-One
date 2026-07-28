/**
 * ref: se3_example.cpp
 * @author tfly
 */

#ifndef SE3_CTRL_H
#define SE3_CTRL_H
#include <string>

#include <ros/message_event.h>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "se3_controller/se3_controller.hpp"
#include "se3_controller/se3_dynamic_tuneConfig.h"
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

using namespace std;

class se3Ctrl{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_, desire_odom_pub_, local_pos_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_;
    ros::Subscriber desire_angle_sub_, multiDOFJoint_sub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceServer land_service_;
    ros::Timer exec_timer_;

    mavros_msgs::State currState_;
    mavros_msgs::CommandBool arm_cmd;
    nav_msgs::Odometry desire_odom_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Desired_State_t desired_state_;
    SE3_CONTROLLER se3_controller_;

    bool sim_enable_, auto_request_offboard_{false}, auto_request_arm_{false}, auto_land_on_geofence_{false};
    bool enable_thrust_estimation_{false};
    bool use_acceleration_feedforward_{true}, use_yaw_rate_feedforward_{true};
    bool has_odom_{false}, has_imu_{false}, has_state_{false};
    bool has_trajectory_after_offboard_{false};
    bool land_mode_request_accepted_{false};
    double max_feedforward_acc_, odom_timeout_{0.2};
    double imu_timeout_{0.2}, state_timeout_{2.0};
    double trajectory_command_timeout_{0.08};
    std::string command_publisher_node_{"/planner_gateway"};
    double land_retry_interval_{1.0}, safety_hold_retry_interval_{1.0};
    double ki_pz_{0.0}, int_limit_z_{5.0};
    Eigen::Vector3d geo_fence_;
    ros::Time state_rcv_stamp_, state_msg_stamp_;
    ros::Time last_land_mode_request_, last_safety_hold_request_;
    ros::Time last_offboard_request_, last_arm_request_;
    ros::WallTime last_trajectory_command_wall_time_;

    Eigen::Vector3d kp_p_, kp_v_, kp_a_, kp_q_, kp_w_, kd_p_, kd_v_, kd_a_, kd_q_, kd_w_;
    double limit_err_p_, limit_err_v_, limit_err_a_, limit_d_err_p_, limit_d_err_v_, limit_d_err_a_;
    double hover_percent_, max_hover_percent_, min_output_thrust_, max_output_thrust_;
    bool enu_frame_, vel_in_body_;

    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig> dynamic_tune_server_;
    dynamic_reconfigure::Server<se3_controller::se3_dynamic_tuneConfig>::CallbackType dynamic_tune_cb_type_;

    enum FlightState { WAITING_FOR_CONNECTED, WAITING_FOR_OFFBOARD, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED } node_state_;

    void execFSMCallback(const ros::TimerEvent &e);

    bool send_cmd(const Controller_Output_t &output, bool angle);
    void pubLocalPose(const Eigen::Vector3d &pose); 
    bool hasFreshOdom() const;
    bool hasFreshImu() const;
    bool hasFreshState() const;
    void setDesiredStateToCurrentOdom();
    void syncDesiredOdomMessage(const ros::Time &stamp);
    void requestSafetyHold(const char *reason);
    void resetForDisarmedState();

    bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void StateCallback(const mavros_msgs::State::ConstPtr &msg);
    void multiDOFJointCallback(
        const ros::MessageEvent<
            trajectory_msgs::MultiDOFJointTrajectory const> &event);


    void DynamicTuneCallback(se3_controller::se3_dynamic_tuneConfig &config, uint32_t level){
        ROS_INFO("kp_p: %f %f %f", config.kp_px, config.kp_py, config.kp_pz);
        ROS_INFO("kp_v: %f %f %f", config.kp_vx, config.kp_vy, config.kp_vz);
        ROS_INFO("kp_a: %f %f %f", config.kp_ax, config.kp_ay, config.kp_az);
        ROS_INFO("kp_q: %f %f %f", config.kp_qx, config.kp_qy, config.kp_qz);
        ROS_INFO("kp_w: %f %f %f", config.kp_wx, config.kp_wy, config.kp_wz);

        ROS_INFO("kd_p: %f %f %f", config.kd_px, config.kd_py, config.kd_pz);
        ROS_INFO("kd_v: %f %f %f", config.kd_vx, config.kd_vy, config.kd_vz);
        ROS_INFO("kd_a: %f %f %f", config.kd_ax, config.kd_ay, config.kd_az);
        ROS_INFO("kd_q: %f %f %f", config.kd_qx, config.kd_qy, config.kd_qz);
        ROS_INFO("kd_w: %f %f %f", config.kd_wx, config.kd_wy, config.kd_wz);

        ROS_INFO("limit err   p v a: %f %f %f", config.limit_err_p, config.limit_err_v, config.limit_err_a);
        ROS_INFO("limit d err p v a: %f %f %f", config.limit_d_err_p, config.limit_d_err_v, config.limit_d_err_a);

        Eigen::Vector3d kp_p(config.kp_px, config.kp_py, config.kp_pz);
        Eigen::Vector3d kp_v(config.kp_vx, config.kp_vy, config.kp_vz);
        Eigen::Vector3d kp_a(config.kp_ax, config.kp_ay, config.kp_az);
        Eigen::Vector3d kp_q(config.kp_qx, config.kp_qy, config.kp_qz);
        Eigen::Vector3d kp_w(config.kp_wx, config.kp_wy, config.kp_wz);
        Eigen::Vector3d kd_p(config.kd_px, config.kd_py, config.kd_pz);
        Eigen::Vector3d kd_v(config.kd_vx, config.kd_vy, config.kd_vz);
        Eigen::Vector3d kd_a(config.kd_ax, config.kd_ay, config.kd_az);
        Eigen::Vector3d kd_q(config.kd_qx, config.kd_qy, config.kd_qz);
        Eigen::Vector3d kd_w(config.kd_wx, config.kd_wy, config.kd_wz);

        if (!se3_controller_.setup(
                kp_p, kp_v, kp_a, kp_q, kp_w,
                kd_p, kd_v, kd_a, kd_q, kd_w,
                config.limit_err_p, config.limit_err_v, config.limit_err_a,
                config.limit_d_err_p, config.limit_d_err_v,
                config.limit_d_err_a)) {
            ROS_ERROR("[se3_controller] Ignoring non-finite or non-positive dynamic controller parameters.");
            return;
        }

        kp_p_ = kp_p;
        kp_v_ = kp_v;
        kp_a_ = kp_a;
        kp_q_ = kp_q;
        kp_w_ = kp_w;
        kd_p_ = kd_p;
        kd_v_ = kd_v;
        kd_a_ = kd_a;
        kd_q_ = kd_q;
        kd_w_ = kd_w;
        limit_err_p_ = config.limit_err_p;
        limit_err_v_ = config.limit_err_v;
        limit_err_a_ = config.limit_err_a;
        limit_d_err_p_ = config.limit_d_err_p;
        limit_d_err_v_ = config.limit_d_err_v;
        limit_d_err_a_ = config.limit_d_err_a;
        
        ki_pz_ = std::isfinite(config.ki_pz) ? config.ki_pz : 0.0;
        int_limit_z_ =
            std::isfinite(config.int_limit_z) && config.int_limit_z >= 0.0
                ? config.int_limit_z
                : 0.0;
        se3_controller_.setIntegral(Eigen::Vector3d(0.0, 0.0, ki_pz_), int_limit_z_);
        se3_controller_.resetIntegral();
        ROS_INFO("integral: ki_pz=%f int_limit_z=%f", ki_pz_, int_limit_z_);

        printf("\n");
    }

    


public:
    se3Ctrl(const ros::NodeHandle &nh);
    ~se3Ctrl(){};

    void trigger_offboard()
    {
        if (!(sim_enable_ && auto_request_offboard_) || !hasFreshState()) {
            return;
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        const ros::Time now = ros::Time::now();
        if (currState_.mode != "OFFBOARD" &&
            (last_offboard_request_.isZero() ||
             (now - last_offboard_request_).toSec() >= 1.0)) {
            last_offboard_request_ = now;
            if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
        }
    }

    void trigger_arm()
    {
        if (!(sim_enable_ && auto_request_arm_) || !hasFreshState()) {
            return;
        }

        arm_cmd.request.value = true;
        if( currState_.mode == "OFFBOARD"){
            const ros::Time now = ros::Time::now();
            if(!currState_.armed &&
               (last_arm_request_.isZero() ||
                (now - last_arm_request_).toSec() >= 1.0)){
                last_arm_request_ = now;
                if( arming_client_.call(arm_cmd) &&arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            }
        }
    }
};

#endif
