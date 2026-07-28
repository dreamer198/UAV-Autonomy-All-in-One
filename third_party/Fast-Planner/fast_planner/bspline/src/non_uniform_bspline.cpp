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



#include "bspline/non_uniform_bspline.h"
#include <ros/ros.h>

namespace fast_planner {

/*
 * 阅读主线：控制点 control_points_ 是曲线的几何自由度，但通常不是曲线必经点；knot 向量
 * u_ 决定基函数的局部支撑和参数时间，两者共同定义实际曲线。p_ 在本项目中表示“次数”
 *（位置轨迹通常 p_=3）。evaluateDeBoor() 负责求曲线点，getDerivative() 把位置曲线变成
 * 速度/加速度曲线，checkFeasibility()/reallocateTime() 则利用导数控制点检查并调整时间。
 * 类名虽为 NonUniformBspline，setUniformBspline() 构造的均匀 B-spline 也是它的一个特例。
 */

NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                     const double& interval) {
  setUniformBspline(points, order, interval);
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                          const double& interval) {
  /*
   * 输入 points 的每一行是一个控制点、列数是轨迹维度；order 在这里作为次数 p 使用，
   * interval 是相邻 knot 的时间间隔。构造后共有 n+1 个控制点、m+1=n+p+2 个 knot，
   * 满足 B-spline 的标准关系 m=n+p+1。需要非均匀时间分配时可再用 setKnot() 覆盖 u_。
   */
  control_points_ = points;
  p_              = order;
  interval_       = interval;

  n_ = points.rows() - 1;
  m_ = n_ + p_ + 1;

  u_ = Eigen::VectorXd::Zero(m_ + 1);
  /*
   * 这里生成的是等间隔、非夹持（两端不重复）的 knot。前 p 个 knot 向负时间外延，使有效
   * 区间从 u_p=0 开始；末尾也保留求值所需的支撑 knot。它们不是额外的执行时段。
   */
  for (int i = 0; i <= m_; ++i) {

    if (i <= p_) {
      u_(i) = double(-p_ + i) * interval_;
    } else if (i > p_ && i <= m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    } else if (i > m_ - p_) {
      u_(i) = u_(i - 1) + interval_;
    }
  }
}

// 只替换 u_（不会重算 n_/m_/interval_）；调用方需保证数量满足 m=n+p+1 且单调不减。
void NonUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
  // p 次 B-spline 只有 [u_p, u_{m-p}] 具备完整的 p+1 个局部支撑控制点。
  um   = u_(p_);
  um_p = u_(m_ - p_);
}

Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  // B-spline 一般不经过首末控制点；真正轨迹端点必须在有效参数边界处求值。
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {

  /*
   * 输入 knot 参数 u，输出任意维的曲线点 C(u)。De Boor 是 B-spline 版的 De Casteljau：
   * 它只访问 u 所在区间附近的 p+1 个控制点，复杂度与整条轨迹控制点数无关。
   * 越界输入先钳制到有效区间，因此 evaluateDeBoorT() 在轻微浮点越界时仍返回端点。
   */
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // 找到 u 所在的 knot 区间 [u_k, u_{k+1}]。
  int k = p_;
  while (true) {
    if (u_(k + 1) >= ub) break;
    ++k;
  }

  /*
   * 初值 d_i^(0)=P_{k-p+i}。第 r 层使用
   * alpha=(u-u_{i+k-p})/(u_{i+1+k-r}-u_{i+k-p}) 做仿射插值，最终 d_p^(p)=C(u)。
   * 内层倒序更新是为了原地复用 d，避免覆盖本层仍需读取的 d[i-1]。
   */
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
    // cout << d[i].transpose() << endl;
  }

  for (int r = 1; r <= p_; ++r) {
    for (int i = p_; i >= r; --i) {
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p_];
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
  // 局部轨迹时间 t 属于 [0,u_{m-p}-u_p]；通过 u=t+u_p 换成 De Boor 使用的 knot 参数。
  return evaluateDeBoor(t + u_(p_));
}

Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  /*
   * p 次 B-spline 的导数仍是 B-spline，次数降为 p-1，导数控制点为
   *   Q_i = p(P_{i+1}-P_i)/(u_{i+p+1}-u_{i+1})。
   * 因而时间间隔越大，同一控制点差分对应的速度越小；后面的动力学检查正是利用这一点。
   */
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    ctp.row(i) =
        p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

NonUniformBspline NonUniformBspline::getDerivative() {
  // 输出新曲线而不修改原曲线；位置可连续调用两/三次分别得到速度、加速度和 jerk。
  Eigen::MatrixXd   ctp = getDerivativeControlPoints();
  NonUniformBspline derivative(ctp, p_ - 1, interval_);

  // 次数和控制点数各减 1 后，合法 knot 数也少 2；删去原 knot 的首尾即可保持同一有效时域。
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

double NonUniformBspline::getInterval() { return interval_; }

void NonUniformBspline::setPhysicalLimits(const double& vel, const double& acc) {
  // limit_ratio_=1.1 限制一次局部重分配最多放慢 10%，避免 knot 在单轮中突变过大。
  limit_vel_   = vel;
  limit_acc_   = acc;
  limit_ratio_ = 1.1;
}

bool NonUniformBspline::checkFeasibility(bool show) {
  /*
   * 输入由 setPhysicalLimits() 设置的逐轴速度/加速度上限，返回整条轨迹是否可行。
   * B-spline 位于其控制点凸包内；同理，若所有速度/加速度曲线的控制点逐轴不超限，连续曲线
   * 也不会超限。因此这里检查导数控制点，而不必高频采样整条轨迹。
   */
  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  // 一阶导数控制点，公式与 getDerivativeControlPoints() 相同。
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {

      if (show) cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
    }
  }

  // 二阶导数控制点：对一阶导数控制点再次作带 knot 间隔的差分。
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      if (show) cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }
    }
  }

  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea;
}

double NonUniformBspline::checkRatio() {
  /*
   * 返回建议的时间伸缩倍数 s=max(v_max/v_limit, sqrt(a_max/a_limit))。
   * 原因是全局时间放慢 s 倍后，速度缩为 1/s、加速度缩为 1/s^2。ratio<=1 表示无需放慢；
   * PlannerManager 会将该值用于重新参数化，而不是在这里直接改控制点。
   */
  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  // find max vel
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));
    }
  }
  // find max acc
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));
    }
  }
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);

  return ratio;
}

bool NonUniformBspline::reallocateTime(bool show) {
  /*
   * 对每个超限导数控制点局部拉开相关 knot，并把后续 knot 整体平移，控制点保持不变。
   * 速度随时间间隔按 1/s 缩小，加速度按 1/s^2 缩小，所以两类超限分别使用 v/v_lim 和
   * sqrt(a/a_lim) 作为伸长比例；单次最多取 limit_ratio_=1.1。
   *
   * 返回值 fea 描述“进入本函数时是否已经可行”。返回 false 通常表示本轮已做时间调整，
   * 调用方应再次调用并重新检查；PlannerManager 正是以最多若干轮的循环方式使用它。
   */
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  // cout << "origin knots:\n" << u_.transpose() << endl;
  bool fea = true;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  double max_vel, max_acc;

  // 调整 u_{i+2}...u_{i+p+1} 的局部跨度，并给更晚的 knot 统一加 delta_t 以保持单调性。
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      max_vel = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }

      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;

      double time_ori = u_(i + p_ + 1) - u_(i + 1);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_);

      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

      for (int j = i + p_ + 2; j < u_.rows(); ++j) {
        u_(j) += delta_t;
      }
    }
  }

  // 加速度关联 p-1 个 knot 跨度；早期 i=1/2 使用专门分支处理轨迹起始附近的 knot。
  for (int i = 0; i < P.rows() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }

      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;
      // cout << "ratio: " << ratio << endl;

      double time_ori = u_(i + p_ + 1) - u_(i + 2);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_ - 1);

      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }

      } else {

        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}

void NonUniformBspline::lengthenTime(const double& ratio) {
  /*
   * 将内部 knot 区间 [u_5,u_{last-5}] 的跨度乘 ratio：ratio>1 拉长，ratio<1 压缩。
   * 区间内线性摊开时间变化，之后的 knot 整体平移，之前的 knot 保持不动。该实现面向本项目
   * 三阶局部轨迹，并非任意次数曲线的通用全局缩放；PlannerManager 随后会重新采样和参数化。
   */
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc   = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

// 预留的旧接口，当前实现为空；阅读主流程时可以跳过。
void NonUniformBspline::recomputeInit() {}

void NonUniformBspline::parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                              const vector<Eigen::Vector3d>& start_end_derivative,
                                              Eigen::MatrixXd&               ctrl_pts) {
  /*
   * 把 K 个等时间采样位置拟合成 K+2 个三阶均匀 B-spline 控制点，作为后端优化初值。
   * start_end_derivative 必须按 [起点速度, 终点速度, 起点加速度, 终点加速度] 排列。
   * 输出 ctrl_pts 为 (K+2)x3。这里固定使用三阶基函数系数，不能直接当作任意 order 的参数化器。
   */
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  int K = point_set.size();

  /*
   * 三阶均匀 B-spline 在采样 knot 上有：
   *   p_i=(P_i+4P_{i+1}+P_{i+2})/6，
   *   v=(P_{i+2}-P_i)/(2*ts)，a=(P_i-2P_{i+1}+P_{i+2})/ts^2。
   * 前 K 行拟合位置，后 4 行加入首尾速度和加速度，组成 A*P=b。
   */
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

  for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
  // cout << "A:\n" << A << endl;

  // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
  // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
  // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // x/y/z 共用同一个 A；右端分别拼接 K 个位置分量和 4 个边界导数分量。
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  /*
   * A 是 (K+4)x(K+2) 的超定矩阵，列主元 QR 求的是最小二乘解。因此采样点和边界导数都作为
   * 拟合方程参与，而不是另设硬等式约束；随后 B-spline 优化会继续调整内部控制点。
   */
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;

  // cout << "[B-spline]: parameterization ok." << endl;
}

double NonUniformBspline::getTimeSum() {
  // 执行时长只取有效 knot 区间，不包含两端为局部支撑而外延的 knot。
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength(const double& res) {
  // 以时间步长 res 折线采样近似弧长；res 越小越精确，但不是解析弧长。
  double          length = 0.0;
  double          dur    = getTimeSum();
  Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double NonUniformBspline::getJerk() {
  /*
   * 对规划器使用的三阶位置 B-spline 连续求导三次后，jerk 是分段常量（0 次 B-spline），
   * 因而 sum Delta_t*||J_i||^2 正是积分 int ||jerk(t)||^2 dt，用于比较候选轨迹平滑度。
   */
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();
  int             dimension = ctrl_pts.cols();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v) {
  // 对速度曲线每 0.01 s 采样一次，统计的是三维向量范数，而非逐轴物理约束。
  NonUniformBspline vel = getDerivative();
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double          vn  = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a) {
  // 与速度统计相同，对二阶导数曲线作 0.01 s 离散采样，结果用于性能评估。
  NonUniformBspline acc = getDerivative().getDerivative();
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double          an  = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}
}  // namespace fast_planner
