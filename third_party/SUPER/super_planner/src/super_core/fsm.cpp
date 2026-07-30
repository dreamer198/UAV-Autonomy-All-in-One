/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fsm/fsm.h>
#include <memory>

using namespace super_utils;

namespace fsm {
    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        std::lock_guard<std::recursive_mutex> guard(lifecycle_mutex_);
        const uint64_t callback_epoch = getLifecycleEpoch();
        if (stop) {
            return;
        }

        if (machine_state_ != FOLLOW_TRAJ) {
            return;
        }

        if (finish_plan) {
            return;
        }

        if (plan_from_rest_) {
            plan_from_rest_ = false;
            return;
        }

        TimeConsuming replan_once_time("replan_once_time", false);

        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (callback_epoch != getLifecycleEpoch()) {
            return;
        }
        if (ret_code == FAILED) {
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }

        if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {
            gi_.new_goal = false;
            publishPolyTraj();
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    void Fsm::callMainFsmOnce() {
        std::lock_guard<std::recursive_mutex> guard(lifecycle_mutex_);
        const uint64_t callback_epoch = getLifecycleEpoch();
        if (stop) {
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
        static double last_print_t = 0.0;
        planner_ptr_->getRobotState(robot_state_);


        if (cur_t - last_print_t > 1.0) {
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) {
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            case WAIT_GOAL: {
                if (!gi_.new_goal) {
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);
                }
                resetVisualizedPath();
                break;
            }
            case GENERATE_TRAJ: {
                if (closeToGoal(cfg_.close_goal_threshold)) {
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
                if (callback_epoch != getLifecycleEpoch()) {
                    return;
                }
                if (!planner_ptr_->goalValid()) {
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == SUCCESS || retcode == FINISH) {
                    gi_.new_goal = false;
                    plan_from_rest_ = true;
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publishPolyTraj();

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);
                } else {
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    // ros::Duration(0.1).sleep();
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath();
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    bool Fsm::validateGoalPosiAndYaw(const Vec3f &p,
                                     const Quatf &q,
                                     Vec3f &effective_goal,
                                     double &effective_yaw,
                                     std::string &reason) {
        std::lock_guard<std::recursive_mutex> guard(lifecycle_mutex_);
        if (!p.allFinite()) {
            reason = "goal_position_not_finite";
            return false;
        }

        planner_ptr_->getRobotState(robot_state_);
        if (!robot_state_.rcv) {
            reason = "odometry_not_ready";
            return false;
        }

        auto map = planner_ptr_->getMap();
        if (!map->mapReady()) {
            reason = "map_not_ready";
            return false;
        }

        auto click_point = p;
        if (cfg_.click_height > -5) {
            click_point.z() = cfg_.click_height;
        }

        const auto map_cfg = map->getMapConfig();
        const double vertical_margin =
                map_cfg.inflation_resolution * static_cast<double>(1 + map_cfg.inflation_step);
        if (click_point.z() <= map_cfg.virtual_ground_height + vertical_margin) {
            reason = "goal_below_safe_ground";
            return false;
        }
        if (click_point.z() >= map_cfg.virtual_ceil_height - vertical_margin) {
            reason = "goal_above_safe_ceiling";
            return false;
        }
        if (!map->insideLocalMap(click_point)) {
            reason = "goal_out_of_local_map";
            return false;
        }

        if (!map->getNearestInfCellIs(GridType::KNOWN_FREE, click_point, effective_goal, 3.0)) {
            reason = "no_collision_free_goal_within_3m";
            return false;
        }

        if ((robot_state_.p - effective_goal).norm() < 0.1) {
            reason = "goal_too_close";
            return false;
        }

        if (cfg_.click_yaw_en) {
            if (!q.coeffs().allFinite() || q.squaredNorm() < 1e-12) {
                effective_yaw = NAN;
            } else {
                const Quatf normalized_q = q.normalized();
                effective_yaw = geometry_utils::get_yaw_from_quaternion(normalized_q);
                if (!std::isfinite(effective_yaw)) {
                    reason = "goal_yaw_not_finite";
                    return false;
                }
            }
        } else {
            effective_yaw = NAN;
        }

        reason = "ok";
        return true;
    }

    bool Fsm::setGoalPosiAndYaw(const Vec3f &p,
                                const Quatf &q,
                                Vec3f &effective_goal,
                                double &effective_yaw,
                                std::string &reason) {
        std::lock_guard<std::recursive_mutex> guard(lifecycle_mutex_);
        if (!validateGoalPosiAndYaw(p, q, effective_goal, effective_yaw, reason)) {
            fmt::print(fg(fmt::color::indian_red), "Reject goal: {}.\n", reason);
            return false;
        }

        gi_.goal_p = effective_goal;
        gi_.goal_yaw = effective_yaw;
        cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        if (std::isfinite(gi_.goal_yaw)) {
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose()
                 << "]; goal yaw: " << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
        } else {
            ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                           gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
        }
        started_ = true;
        gi_.new_goal = true;
        return true;
    }

    bool Fsm::resetLifecycle(std::string &reason) {
        lifecycle_epoch_.fetch_add(1, std::memory_order_acq_rel);
        progress_state_.store(static_cast<uint8_t>(WAIT_GOAL), std::memory_order_release);
        std::lock_guard<std::recursive_mutex> guard(lifecycle_mutex_);

        planner_ptr_->reset();
        started_ = false;
        plan_from_rest_ = false;
        finish_plan = false;
        traj_finish_ = false;
        gi_.new_goal = false;
        gi_.goal_p.setZero();
        gi_.goal_yaw = NAN;
        machine_state_ = WAIT_GOAL;
        resetVisualizedPath();
        reason = "planner_reset";
        return true;
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        machine_state_ = new_state;
        progress_state_.store(static_cast<uint8_t>(new_state), std::memory_order_release);
        publishProgressState(static_cast<uint8_t>(new_state));
    }
}
