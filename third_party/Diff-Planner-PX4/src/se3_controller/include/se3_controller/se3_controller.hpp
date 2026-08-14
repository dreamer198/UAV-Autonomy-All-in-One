#ifndef SE3_CONTROLLER_HPP
#define SE3_CONTROLLER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "se3_controller/utils.hpp"

#define VEL_IN_BODY /* cancel the comment if the velocity in odom topic is relative to current body frame, not to world frame.*/
// #define AIRSIM

namespace se3_safety
{
inline bool isQuaternionValid(const Eigen::Quaterniond &q)
{
	return q.coeffs().allFinite() && q.squaredNorm() > 1e-12;
}

inline bool isFreshAt(const ros::Time &receive_stamp,
					  const ros::Time &message_stamp,
					  const ros::Time &now,
					  const double timeout_sec)
{
	if (receive_stamp.isZero() || now.isZero() ||
		!std::isfinite(timeout_sec) || timeout_sec <= 0.0)
	{
		return false;
	}

	const double receive_age = (now - receive_stamp).toSec();
	if (!std::isfinite(receive_age) || receive_age < 0.0 ||
		receive_age > timeout_sec)
	{
		return false;
	}

	// Some MAVROS messages legitimately have a zero header stamp. When a
	// source stamp exists, check it as well so repeatedly replaying an old
	// measurement cannot make it fresh merely by invoking the callback.
	if (!message_stamp.isZero())
	{
		const double message_age = (now - message_stamp).toSec();
		constexpr double kAllowedFutureSkew = 0.05;
		if (!std::isfinite(message_age) ||
			message_age < -kAllowedFutureSkew ||
			message_age > timeout_sec)
		{
			return false;
		}
	}
	return true;
}

inline double smoothStep(const double progress)
{
	if (!std::isfinite(progress) || progress <= 0.0)
	{
		return 0.0;
	}
	if (progress >= 1.0)
	{
		return 1.0;
	}
	return progress * progress * (3.0 - 2.0 * progress);
}

inline Eigen::Quaterniond interpolateAttitude(
	const Eigen::Quaterniond &start,
	const Eigen::Quaterniond &target,
	const double progress)
{
	if (!isQuaternionValid(start) || !isQuaternionValid(target))
	{
		return Eigen::Quaterniond::Identity();
	}

	Eigen::Quaterniond normalized_start = start.normalized();
	Eigen::Quaterniond normalized_target = target.normalized();
	// q and -q describe the same rotation. Select the nearer representation so
	// the OFFBOARD handoff can never take the long way around.
	if (normalized_start.dot(normalized_target) < 0.0)
	{
		normalized_target.coeffs() *= -1.0;
	}
	Eigen::Quaterniond result = normalized_start.slerp(
		smoothStep(progress), normalized_target);
	result.normalize();
	return result;
}

inline double quaternionAngularDistance(
	const Eigen::Quaterniond &first,
	const Eigen::Quaterniond &second)
{
	if (!isQuaternionValid(first) || !isQuaternionValid(second))
	{
		return std::numeric_limits<double>::infinity();
	}
	const double dot = std::abs(first.normalized().dot(second.normalized()));
	const double clamped_dot = std::max(0.0, std::min(1.0, dot));
	return 2.0 * std::acos(clamped_dot);
}

inline double interpolateScalar(
	const double start,
	const double target,
	const double progress)
{
	if (!std::isfinite(start) || !std::isfinite(target))
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	const double blend = smoothStep(progress);
	return start + (target - start) * blend;
}
} // namespace se3_safety

struct Odom_Data_t{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Quaterniond q;
	Eigen::Vector3d w;

	nav_msgs::Odometry msg;
	ros::Time rcv_stamp;
	bool recv_new_msg;

	Odom_Data_t()
		: p(Eigen::Vector3d::Zero()),
		  v(Eigen::Vector3d::Zero()),
		  q(Eigen::Quaterniond::Identity()),
		  w(Eigen::Vector3d::Zero()),
		  recv_new_msg(false) {}
	bool isFresh(double timeout_sec = 0.2) const {
		return isFreshAt(ros::Time::now(), timeout_sec);
	}
	bool isFreshAt(const ros::Time &now, double timeout_sec = 0.2) const {
		return recv_new_msg &&
			se3_safety::isFreshAt(
				rcv_stamp, msg.header.stamp, now, timeout_sec) &&
			isValid();
	}
	bool isValid() const {
		return p.allFinite() && v.allFinite() && w.allFinite() &&
			se3_safety::isQuaternionValid(q);
	}
	bool feed(nav_msgs::OdometryConstPtr pMsg, bool enu_frame, bool vel_in_body){
		if (!pMsg)
			return false;

		Eigen::Vector3d next_p(
			pMsg->pose.pose.position.x,
			pMsg->pose.pose.position.y,
			pMsg->pose.pose.position.z);
		Eigen::Vector3d next_v(
			pMsg->twist.twist.linear.x,
			pMsg->twist.twist.linear.y,
			pMsg->twist.twist.linear.z);
		Eigen::Quaterniond next_q(
			pMsg->pose.pose.orientation.w,
			pMsg->pose.pose.orientation.x,
			pMsg->pose.pose.orientation.y,
			pMsg->pose.pose.orientation.z);
		Eigen::Vector3d next_w(
			pMsg->twist.twist.angular.x,
			pMsg->twist.twist.angular.y,
			pMsg->twist.twist.angular.z);
		if (!next_p.allFinite() || !next_v.allFinite() ||
			!next_w.allFinite() ||
			!se3_safety::isQuaternionValid(next_q))
		{
			return false;
		}
		next_q.normalize();

		if(!enu_frame){
			Eigen::Matrix3d R_mid;
			R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
			Eigen::Quaterniond q_mid(R_mid);

			next_p = q_mid.toRotationMatrix() * next_p;
			next_v = q_mid.toRotationMatrix() * next_v;
			next_q = q_mid * next_q * q_mid;
			next_q.normalize();
			next_w = q_mid.toRotationMatrix() * next_w;
		}

		if(vel_in_body)
			next_v = next_q.toRotationMatrix() * next_v;

		p = next_p;
		v = next_v;
		q = next_q;
		w = next_w;
		msg = *pMsg;
		rcv_stamp = ros::Time::now();
		recv_new_msg = true;
		return isValid();
	}
};

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){
		p = Eigen::Vector3d::Zero();
		v = Eigen::Vector3d::Zero();
		a = Eigen::Vector3d::Zero();
		j = Eigen::Vector3d::Zero();
		q.w() = 1;
		q.x() = 0;
		q.y() = 0;
		q.z() = 0;
		yaw = 0;
		yaw_rate = 0;
	}

	Desired_State_t(Odom_Data_t odom){
		p = odom.p;
		v = Eigen::Vector3d::Zero();
		a = Eigen::Vector3d::Zero();
		j = Eigen::Vector3d::Zero();
		q = odom.q.normalized();
		yaw = utils::fromQuaternion2yaw(q);
		yaw_rate = 0;
	}

	bool isValid() const {
		return p.allFinite() && v.allFinite() && a.allFinite() &&
			j.allFinite() && se3_safety::isQuaternionValid(q) &&
			std::isfinite(yaw) && std::isfinite(yaw_rate);
	}
};

struct Controller_Output_t
{
	// Eigen::Vector3d v;

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;
};

struct Imu_Data_t{
	Eigen::Quaterniond q;
	Eigen::Vector3d w;
	Eigen::Vector3d a;

	sensor_msgs::Imu msg;
	ros::Time rcv_stamp;
	bool recv_new_msg;

	Imu_Data_t()
		: q(Eigen::Quaterniond::Identity()),
		  w(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  recv_new_msg(false) {}
	bool isFresh(double timeout_sec = 0.2) const {
		return isFreshAt(ros::Time::now(), timeout_sec);
	}
	bool isFreshAt(const ros::Time &now, double timeout_sec = 0.2) const {
		return recv_new_msg &&
			se3_safety::isFreshAt(
				rcv_stamp, msg.header.stamp, now, timeout_sec) &&
			isValid();
	}
	bool isValid() const {
		return a.allFinite() && w.allFinite() &&
			se3_safety::isQuaternionValid(q);
	}
	bool feed(sensor_msgs::ImuConstPtr pMsg, bool enu_frame){
		if (!pMsg)
			return false;

		Eigen::Vector3d next_a(
			pMsg->linear_acceleration.x,
			pMsg->linear_acceleration.y,
			pMsg->linear_acceleration.z);
		Eigen::Quaterniond next_q(
			pMsg->orientation.w,
			pMsg->orientation.x,
			pMsg->orientation.y,
			pMsg->orientation.z);
		Eigen::Vector3d next_w(
			pMsg->angular_velocity.x,
			pMsg->angular_velocity.y,
			pMsg->angular_velocity.z);
		if (!next_a.allFinite() || !next_w.allFinite() ||
			!se3_safety::isQuaternionValid(next_q))
		{
			return false;
		}
		next_q.normalize();

		if(!enu_frame){
			Eigen::Matrix3d R_mid;
			R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
			Eigen::Quaterniond q_mid(R_mid);

			next_a = q_mid.toRotationMatrix() * next_a;
			next_q = q_mid * next_q * q_mid;
			next_q.normalize();
			next_w = q_mid.toRotationMatrix() * next_w;
		}

		a = next_a;
		q = next_q;
		w = next_w;
		msg = *pMsg;
		rcv_stamp = ros::Time::now();
		recv_new_msg = true;
		return isValid();
	}
};

class SE3_CONTROLLER
{
private:
	Eigen::Vector3d Kp_p_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kp_v_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kp_a_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kp_q_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kp_w_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kd_p_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kd_v_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kd_a_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kd_q_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d Kd_w_{Eigen::Vector3d::Zero()};
	double limit_err_p_{1.0}, limit_err_v_{1.0}, limit_err_a_{1.0};
	double limit_d_err_p_{1.0}, limit_d_err_v_{1.0}, limit_d_err_a_{1.0};
	bool have_last_err_{false}, enu_frame_{true}, vel_in_body_{true};

	double hover_percent_{0.45}, max_hover_percent_{0.75};
	double min_output_thrust_{0.20}, max_output_thrust_{0.85};
	double T_a_{9.81 / 0.45}; // normalization constant
	double P_ = 1e6;
	const double rho_ = 0.998; // confidence
	const double gravity_ = 9.81;
	static constexpr double kAlmostZeroValueThreshold_ = 0.001;
	Eigen::Vector3d grav_vec_{Eigen::Vector3d(0.0, 0.0, 9.81)};
	Eigen::Vector3d last_err_p_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d last_err_v_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d last_err_a_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d last_err_q_{Eigen::Vector3d::Zero()};
	Eigen::Vector3d last_err_w_{Eigen::Vector3d::Zero()};
	std::queue<std::pair<ros::Time, double>> timed_thrust_;

	// 竖直(z)位置积分项：补偿随机重/电压变化的悬停推力，消除稳态高度误差
	Eigen::Vector3d Ki_p_ = Eigen::Vector3d::Zero();      // 积分增益（仅 z 非零）
	Eigen::Vector3d int_err_p_ = Eigen::Vector3d::Zero(); // 位置误差积分累加器 [m·s]
	double int_limit_ = 0.0;                              // 抗饱和逐轴钳位 [m·s]
	bool out_saturated_ = false;                          // 上一拍推力是否饱和
	static constexpr double kCtrlDt_ = 0.01;              // 100Hz 默认步长 [s]

	bool computeFlatInput_Hopf_Fibration(Desired_State_t desired_state, Odom_Data_t &desired_odom){
		const double acceleration_norm = desired_state.a.norm();
		if (!desired_state.isValid() || !std::isfinite(acceleration_norm) ||
			acceleration_norm <= kAlmostZeroValueThreshold_)
		{
			return false;
		}

		Eigen::Vector3d abc = desired_state.a / acceleration_norm;
		double a = abc(0), b = abc(1), c = abc(2);
		c = std::max(-1.0, std::min(1.0, c));
		Eigen::Vector3d abc_dot =
			(Eigen::Matrix3d::Identity() - abc * abc.transpose()) *
			desired_state.j / acceleration_norm;
		double a_dot = abc_dot(0), b_dot = abc_dot(1), c_dot = abc_dot(2);
		double yaw = desired_state.yaw;
		double yaw_dot = desired_state.yaw_rate;
		double syaw = sin(yaw), cyaw = cos(yaw);

		if(c > 0){
			double norm = sqrt(2 * (1 + c));
			Eigen::Quaterniond q((1 + c) / norm, -b / norm, a / norm, 0);
			Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
			desired_odom.q = q * q_yaw;
			desired_odom.w(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1);
			desired_odom.w(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1);
			desired_odom.w(2) = (b * a_dot - a * b_dot) / (1 + c) + yaw_dot;
		}else{
			double norm = sqrt(2 * (1 - c));
			Eigen::Quaterniond q(-b / norm, (1 - c) / norm, 0, a / norm);
			yaw += 2 * atan2(a, b);
			Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
			desired_odom.q = q * q_yaw;
			syaw = sin(yaw);
			cyaw = cos(yaw);
			desired_odom.w(0) = syaw * a_dot + cyaw * b_dot - (a * syaw + b * cyaw) * c_dot / (c - 1);
			desired_odom.w(1) = cyaw * a_dot - syaw * b_dot - (a * cyaw - b * syaw) * c_dot / (c - 1);
			desired_odom.w(2) = (b * a_dot - a * b_dot) / (c - 1) + yaw_dot;
		}
		if (!se3_safety::isQuaternionValid(desired_odom.q) ||
			!desired_odom.w.allFinite())
		{
			return false;
		}
		desired_odom.q.normalize();
		return true;
	}

	void limitErr(Eigen::Vector3d &err, double low, double upper){
		err(0) = std::max(std::min(err(0), upper), low);
		err(1) = std::max(std::min(err(1), upper), low);
		err(2) = std::max(std::min(err(2), upper), low);
	}

	double limitYaw(double yaw_curr, double yaw_des, double yaw_limit){
		double err = yaw_des - yaw_curr;
		yaw_limit = std::abs(yaw_limit);
		if(err < -yaw_limit){
			return yaw_curr - yaw_limit;
		}else if (err > yaw_limit){
			return yaw_curr + yaw_limit;
		}else{
			return yaw_des;
		}
	}

public:
	SE3_CONTROLLER(){};
    ~SE3_CONTROLLER(){};

	// 设置/清零竖直积分项（由 se3_ctrl 在构造与 dynamic_reconfigure 回调中调用）
	void setIntegral(const Eigen::Vector3d &ki, double int_limit){
		Ki_p_ = ki.allFinite() ? ki : Eigen::Vector3d::Zero();
		int_limit_ =
			std::isfinite(int_limit) && int_limit >= 0.0 ? int_limit : 0.0;
	}
	void resetIntegral(){
		int_err_p_.setZero();
		out_saturated_ = false;
		have_last_err_ = false;
		last_err_p_.setZero();
		last_err_v_.setZero();
		last_err_a_.setZero();
		last_err_q_.setZero();
		last_err_w_.setZero();
	}

	void init(double hover_percent, double max_hover_percent, double min_output_thrust, double max_output_thrust, bool enu_frame, bool vel_in_body){
		if (!std::isfinite(hover_percent) || hover_percent <= 0.0 ||
			hover_percent > 1.0)
			hover_percent = 0.45;
		if (!std::isfinite(max_hover_percent) ||
			max_hover_percent < hover_percent || max_hover_percent > 1.0)
			max_hover_percent = std::max(hover_percent, 0.75);
		if (!std::isfinite(min_output_thrust) ||
			min_output_thrust < 0.0 || min_output_thrust >= 1.0)
			min_output_thrust = 0.20;
		if (!std::isfinite(max_output_thrust) ||
			max_output_thrust <= min_output_thrust ||
			max_output_thrust > 1.0)
			max_output_thrust =
				std::max(0.85, 0.5 * (1.0 + min_output_thrust));

		hover_percent_ = hover_percent;
		max_hover_percent_ = max_hover_percent;
		min_output_thrust_ = min_output_thrust;
		max_output_thrust_ = max_output_thrust;
		enu_frame_ = enu_frame;
		vel_in_body_ = vel_in_body;
		T_a_ = gravity_ / hover_percent_;
		grav_vec_ << 0.0, 0.0, gravity_;

		last_err_p_ = Eigen::Vector3d::Zero();
		last_err_v_ = Eigen::Vector3d::Zero();
		last_err_a_ = Eigen::Vector3d::Zero();
		last_err_q_ = Eigen::Vector3d::Zero();
		last_err_w_ = Eigen::Vector3d::Zero();

		have_last_err_ = false;
		int_err_p_.setZero();
		out_saturated_ = false;
		P_ = 1e6;
		while (!timed_thrust_.empty())
			timed_thrust_.pop();
	}

	bool setup(Eigen::Vector3d kp_p, Eigen::Vector3d kp_v, Eigen::Vector3d kp_a, Eigen::Vector3d kp_q, Eigen::Vector3d kp_w,
				Eigen::Vector3d kd_p, Eigen::Vector3d kd_v, Eigen::Vector3d kd_a, Eigen::Vector3d kd_q, Eigen::Vector3d kd_w,
				double limit_err_p, double limit_err_v, double limit_err_a, 
				double limit_d_err_p, double limit_d_err_v, double limit_d_err_a){
		if (!kp_p.allFinite() || !kp_v.allFinite() || !kp_a.allFinite() ||
			!kp_q.allFinite() || !kp_w.allFinite() ||
			!kd_p.allFinite() || !kd_v.allFinite() || !kd_a.allFinite() ||
			!kd_q.allFinite() || !kd_w.allFinite() ||
			!std::isfinite(limit_err_p) || limit_err_p <= 0.0 ||
			!std::isfinite(limit_err_v) || limit_err_v <= 0.0 ||
			!std::isfinite(limit_err_a) || limit_err_a <= 0.0 ||
			!std::isfinite(limit_d_err_p) || limit_d_err_p <= 0.0 ||
			!std::isfinite(limit_d_err_v) || limit_d_err_v <= 0.0 ||
			!std::isfinite(limit_d_err_a) || limit_d_err_a <= 0.0)
		{
			return false;
		}
		Kp_p_ = kp_p;
		Kp_v_ = kp_v;
		Kp_a_ = kp_a;
		Kp_q_ = kp_q;
		Kp_w_ = kp_w;
		Kd_p_ = kd_p;
		Kd_v_ = kd_v;
		Kd_a_ = kd_a;
		Kd_q_ = kd_q;
		Kd_w_ = kd_w;
		limit_err_p_ = limit_err_p;
		limit_err_v_ = limit_err_v;
		limit_err_a_ = limit_err_a;
		limit_d_err_p_ = limit_d_err_p;
		limit_d_err_v_ = limit_d_err_v;
		limit_d_err_a_ = limit_d_err_a;
		return true;
	}

	bool calControl(Odom_Data_t odom_data, Imu_Data_t imu_data, Desired_State_t desired_state,
					Controller_Output_t &output,
					double odom_timeout_sec = 0.2,
					double imu_timeout_sec = 0.2,
					double control_dt_sec = kCtrlDt_,
					bool apply_attitude_alignment = true,
					const Eigen::Quaterniond &attitude_alignment =
						Eigen::Quaterniond::Identity()){
		if(!odom_data.isFresh(odom_timeout_sec) ||
		   !imu_data.isFresh(imu_timeout_sec) ||
		   !desired_state.isValid() ||
		   !std::isfinite(control_dt_sec) ||
		   control_dt_sec <= 0.0 || control_dt_sec > 0.2 ||
		   !std::isfinite(T_a_) ||
		   T_a_ <= kAlmostZeroValueThreshold_){
			return false;
		}
		// desired_state.yaw = limitYaw(fromQuaternion2yaw(odom_data.q), desired_state.yaw, M_PI_2 / 3);
		Eigen::Vector3d err_p = odom_data.p - desired_state.p;
		limitErr(err_p, -limit_err_p_, limit_err_p_);
		// 竖直积分项累加（抗饱和：上一拍推力饱和则冻结累加，逐轴钳位）
		if(!out_saturated_){
			int_err_p_ += err_p * control_dt_sec;
			for(int i = 0; i < 3; ++i)
				int_err_p_(i) = std::max(std::min(int_err_p_(i), int_limit_), -int_limit_);
		}
		Eigen::Vector3d d_err_p = Eigen::Vector3d::Zero();
		if (have_last_err_)
			d_err_p = (err_p - last_err_p_) / control_dt_sec;
		limitErr(d_err_p, -limit_d_err_p_, limit_d_err_p_);
		desired_state.v = desired_state.v - Kp_p_.asDiagonal() * err_p - Kd_p_.asDiagonal() * d_err_p;
		Eigen::Vector3d err_v = odom_data.v - desired_state.v;
		limitErr(err_v, -limit_err_v_, limit_err_v_);
		Eigen::Vector3d d_err_v = Eigen::Vector3d::Zero();
		if (have_last_err_)
			d_err_v = (err_v - last_err_v_) / control_dt_sec;
		limitErr(d_err_v, -limit_d_err_v_, limit_d_err_v_);
		desired_state.a = desired_state.a - Kp_v_.asDiagonal() * err_v - Kd_v_.asDiagonal() * d_err_v + grav_vec_;
		// 积分配平：err_p(2)<0(低于目标)→ 减去 Ki*负 → a 增大 → 推力上升 → 消除稳态下垂
		desired_state.a -= Ki_p_.asDiagonal() * int_err_p_;
		// std::cout << "err_p: " << err_p.transpose() << std::endl;
		// std::cout << "err_v: " << err_v.transpose() << std::endl;
		// std::cout << "imu_data.a: " << imu_data.a.transpose() << std::endl;
		// std::cout << "odom_data.v: " << odom_data.v.transpose() << std::endl;
		Eigen::Vector3d a_world = odom_data.q.toRotationMatrix() * imu_data.a;
		Eigen::Vector3d err_a = a_world - desired_state.a;
		limitErr(err_a, -limit_err_a_, limit_err_a_);
		Eigen::Vector3d d_err_a = Eigen::Vector3d::Zero();
		if (have_last_err_)
			d_err_a = (err_a - last_err_a_) / control_dt_sec;
		limitErr(d_err_a, -limit_d_err_a_, limit_d_err_a_);
		desired_state.j = desired_state.j - Kp_a_.asDiagonal() * err_a - Kd_a_.asDiagonal() * d_err_a;

		last_err_p_ = err_p;
		last_err_v_ = err_v;
		last_err_a_ = err_a;
		have_last_err_ = true;
		
		double thr = desired_state.a.transpose() * (odom_data.q * Eigen::Vector3d::UnitZ());
		double raw_thrust = thr / T_a_;
		if (!std::isfinite(raw_thrust))
		{
			resetIntegral();
			return false;
		}
		out_saturated_ = (raw_thrust > max_output_thrust_) || (raw_thrust < min_output_thrust_);
		output.thrust = std::max(std::min(raw_thrust, max_output_thrust_), min_output_thrust_);
		// std::cout << std::endl << "desired_state.a: " << desired_state.a.transpose() << std::endl;
		// std::cout << "odom_v: " << odom_data.v.transpose() << std::endl;
		
		Odom_Data_t desired_odom;
		if (!computeFlatInput_Hopf_Fibration(desired_state, desired_odom))
		{
			resetIntegral();
			return false;
		}
			// The node latches the transform from the external odometry world to
			// the FCU attitude frame while the vehicle is in PX4 position hold.
			// Never recompute it from asynchronous latest samples in this control
			// loop: doing so injects odom/IMU timing jitter directly into the
			// commanded attitude.
			if (apply_attitude_alignment &&
				!se3_safety::isQuaternionValid(attitude_alignment))
			{
				resetIntegral();
				return false;
			}
			output.q = apply_attitude_alignment
				? attitude_alignment.normalized() * desired_odom.q
				: desired_odom.q;
		if (!se3_safety::isQuaternionValid(output.q))
		{
			resetIntegral();
			return false;
		}
		output.q.normalize();
		
		// printf("desired q: (%lf,%lf,%lf,%lf)\n", desired_odom.q.w(), desired_odom.q.x(), desired_odom.q.y(), desired_odom.q.z());
		// std::cout << "desired q: " << desired_state.a.transpose() << std::endl;

		Eigen::Quaterniond err_q = odom_data.q.inverse() * desired_odom.q;
		Eigen::Vector3d err_br;
		if (err_q.w() >= 0){
			err_br.x() = Kp_q_(0) * err_q.x();
			err_br.y() = Kp_q_(1) * err_q.y();
			err_br.z() = Kp_q_(2) * err_q.z();
		}
		else{
			err_br.x() = -Kp_q_(0) * err_q.x();
			err_br.y() = -Kp_q_(1) * err_q.y();
			err_br.z() = -Kp_q_(2) * err_q.z();
		}

		output.bodyrates = desired_odom.w + err_br;

		if(!enu_frame_){
			Eigen::Matrix3d R_mid;
			R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
			Eigen::Quaterniond q_mid(R_mid.inverse());
			output.q = q_mid * output.q * q_mid;
			output.bodyrates = q_mid * output.bodyrates;
		}
		if (!se3_safety::isQuaternionValid(output.q) ||
			!output.bodyrates.allFinite() ||
			!std::isfinite(output.thrust))
		{
			resetIntegral();
			return false;
		}
		output.q.normalize();
	
		// Eigen::Quaterniond err_q = odom_data.q * (desired_odom.q.inverse());
		// // limitErr(err_q, -1.0, 1.0);
		// // Eigen::Vector3d err_w = odom_data.w - odom_data.q.matrix().transpose() * desired_odom.q.matrix() * desired_odom.w;
		// Eigen::Vector3d err_w = odom_data.w - desired_odom.w;
		// limitErr(err_w, -1.0, 1.0);
		// if(have_last_err_ == false){
		// 	have_last_err_ = true;
		// 	last_err_q_ = err_q.vec();
		// 	last_err_w_ = err_w;
		// }
		// Eigen::Vector3d d_err_q = err_q.vec() - last_err_q_;
		// limitErr(d_err_q, -1.0, 1.0);
		// Eigen::Vector3d d_err_w = err_w - last_err_w_;
		// limitErr(d_err_w, -1.0, 1.0);
		// if (err_q.w() >= 0){
		// 	output.bodyrates = desired_odom.w - Kp_q_.asDiagonal() * err_q.vec() - Kp_w_.asDiagonal() * err_w - Kd_q_.asDiagonal() * d_err_q - Kd_w_.asDiagonal() * d_err_w;
		// 	// err_br.x() = Kp_q_(0) * err_q.x();
		// 	// err_br.y() = Kp_q_(1) * err_q.y();
		// 	// err_br.z() = Kp_q_(2) * err_q.z();
		// }
		// else{
		// 	output.bodyrates = desired_odom.w + Kp_q_.asDiagonal() * err_q.vec() - Kp_w_.asDiagonal() * err_w + Kd_q_.asDiagonal() * d_err_q - Kd_w_.asDiagonal() * d_err_w;
		// 	// err_br.x() = -Kp_q_(0) * err_q.x();
		// 	// err_br.y() = -Kp_q_(1) * err_q.y();
		// 	// err_br.z() = -Kp_q_(2) * err_q.z();
		// }
		// // std::cout << "thrust: " << output.thrust << std::endl;
		// // std::cout << "bodyrates: " << output.bodyrates.transpose() << std::endl;
		// last_err_q_ = err_q.vec();
		// last_err_w_ = err_w;

		timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), output.thrust));
		while (timed_thrust_.size() > 100)
			timed_thrust_.pop();
		return true;
	}

	bool estimateTa(const Eigen::Vector3d &est_a){
		if (!est_a.allFinite() || !std::isfinite(T_a_) ||
			T_a_ <= kAlmostZeroValueThreshold_)
		{
			return false;
		}
		ros::Time t_now = ros::Time::now();
		while (timed_thrust_.size() >= 1)
		{
			// Choose data before 35~45ms ago
			std::pair<ros::Time, double> t_t = timed_thrust_.front();
			double time_passed = (t_now - t_t.first).toSec();
			if (time_passed > 0.045){ // 45ms
				timed_thrust_.pop();
				continue;
			}
			if (time_passed < 0.0){ // ROS clock rewound
				while (!timed_thrust_.empty())
					timed_thrust_.pop();
				return false;
			}
			if (time_passed < 0.035){ // 35ms
				return false;
			}

			/***********************************************************/
			/* Recursive least squares algorithm with vanishing memory */
			/***********************************************************/
			double thr = t_t.second;
			timed_thrust_.pop();
			
			/***********************************/
			/* Model: est_a(2) = thr1acc_ * thr */
			/***********************************/
			const double denominator = rho_ + thr * P_ * thr;
			if (!std::isfinite(thr) || !std::isfinite(P_) ||
				!std::isfinite(denominator) ||
				denominator <= kAlmostZeroValueThreshold_)
			{
				return false;
			}
			double gamma = 1 / denominator;
			double K = gamma * P_ * thr;
			double next_T_a = T_a_ + K * (est_a(2) - thr * T_a_);
			double next_P = (1 - K * thr) * P_ / rho_;
			if (!std::isfinite(next_T_a) || !std::isfinite(next_P) ||
				next_T_a <= kAlmostZeroValueThreshold_ || next_P <= 0.0)
			{
				return false;
			}
			T_a_ = std::max(next_T_a, gravity_ / max_hover_percent_);
			P_ = next_P;
			// printf("%6.3f,%6.3f,%6.3f,%6.3f\n", T_a_, gamma, K, P_);
			//fflush(stdout);

			return true;
		}
		return false;
	}
};

#endif
