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



#ifndef _NON_UNIFORM_BSPLINE_H_
#define _NON_UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace fast_planner {
// An implementation of non-uniform B-spline with different dimensions
// It also represents uniform B-spline which is a special case of non-uniform
// 规划器用它统一表示位置轨迹和 yaw 轨迹，并通过求导得到速度/加速度。
class NonUniformBspline {
private:
  // control points for B-spline with different dimensions.
  // Each row represents one single control point
  // The dimension is determined by column number
  // e.g. B-spline with N points in 3D space -> Nx3 matrix
  Eigen::MatrixXd control_points_;

  int             p_, n_, m_;  // p degree, n+1 control points, m = n+p+1
  Eigen::VectorXd u_;          // knots vector
  double          interval_;   // knot span \delta t

  // 由原控制点差分得到导数 B-spline 的控制点。
  Eigen::MatrixXd getDerivativeControlPoints();

  double limit_vel_, limit_acc_, limit_ratio_;  // physical limits and time adjustment ratio

public:
  NonUniformBspline() {}
  NonUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);
  ~NonUniformBspline();

  // initialize as an uniform B-spline
  // 用固定 knot 间隔构造均匀 B-spline。
  void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

  // get / set basic bspline info：控制点、knot 和有效时间范围。

  void                                   setKnot(const Eigen::VectorXd& knot);
  Eigen::VectorXd                        getKnot();
  Eigen::MatrixXd                        getControlPoint();
  double                                 getInterval();
  void                                   getTimeSpan(double& um, double& um_p);
  pair<Eigen::VectorXd, Eigen::VectorXd> getHeadTailPts();

  // compute position / derivative：De Boor 求值和构造导数轨迹。

  Eigen::VectorXd   evaluateDeBoor(const double& u);   // use u \in [up, u_mp]
  Eigen::VectorXd   evaluateDeBoorT(const double& t);  // use t \in [0, duration]
  NonUniformBspline getDerivative();

  // 3D B-spline interpolation of points in point_set, with boundary vel&acc
  // constraints
  // input : (K+2) points with boundary vel/acc; ts
  // output: (K+6) control_pts
  // PlannerManager 会用它把前端搜索采样点转换成后端优化初值。
  static void parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                    const vector<Eigen::Vector3d>& start_end_derivative,
                                    Eigen::MatrixXd&               ctrl_pts);

  /* check feasibility, adjust time：检查动力学约束，并通过拉长时间恢复可行性。 */

  void   setPhysicalLimits(const double& vel, const double& acc);
  bool   checkFeasibility(bool show = false);
  double checkRatio();
  void   lengthenTime(const double& ratio);
  bool   reallocateTime(bool show = false);

  /* for performance evaluation：统计轨迹时长、长度、jerk、速度和加速度。 */

  double getTimeSum();
  double getLength(const double& res = 0.01);
  double getJerk();
  void   getMeanAndMaxVel(double& mean_v, double& max_v);
  void   getMeanAndMaxAcc(double& mean_a, double& max_a);

  void recomputeInit();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif
