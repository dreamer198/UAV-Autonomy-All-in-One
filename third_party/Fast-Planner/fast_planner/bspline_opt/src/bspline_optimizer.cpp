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



#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>
// using namespace std;

namespace fast_planner {

/*
 * 阅读主线：优化变量不是连续轨迹上的采样点，而是 B-spline 的部分控制点。optimize() 决定
 * 哪些控制点可动并配置 NLopt；costFunction() 是 NLopt 回调；combineCost() 再按位掩码调用
 * 各单项代价及梯度。固定控制点提供边界状态，内部控制点在平滑、避障和动力学之间折中。
 *
 * PlannerManager 有两种典型用法：普通重规划直接跑 NORMAL_PHASE；拓扑候选先跑纯二次的
 * GUIDE_PHASE 贴近几何引导路径，再跑 NORMAL_PHASE 利用 ESDF 推离障碍并改善可行性。
 */

// 每个代价占一个 bit，调用方可用按位或组合；判断时用 cost_function_ & 某个标志。
const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
// bit 5 是旧版代价项留下的空位，不影响按位组合；WAYPOINTS 当前使用 bit 6。
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);

// 引导阶段只含平方型平滑/贴合项；普通阶段含平滑、ESDF 避障和动力学软约束。
const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  /*
   * lambda1..lambda5、lambda7 在当前实现中依次对应 smoothness、distance、feasibility、
   * endpoint、guide、waypoints。lambda6/lambda8 以及 visibility 相关参数属于保留配置，
   * 当前 combineCost() 没有使用。权重没有统一量纲，需要结合 ts 和地图距离尺度调参。
   */
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/lambda4", lambda4_, -1.0);
  nh.param("optimization/lambda5", lambda5_, -1.0);
  nh.param("optimization/lambda6", lambda6_, -1.0);
  nh.param("optimization/lambda7", lambda7_, -1.0);
  nh.param("optimization/lambda8", lambda8_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/visib_min", visib_min_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("optimization/order", order_, -1);
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  // DISTANCE 代价依赖该环境查询 ESDF 距离及梯度；不启用 DISTANCE 时不会访问它。
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  // 每行一个控制点；列数 dim_ 可为 1(yaw)、2 或 3(position)，内部统一暂存为 Vector3d。
  control_points_ = points;
  dim_            = control_points_.cols();
}

// 均匀 knot 间隔只进入动力学代价；本优化器不直接改变 knot 或总时长。
void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  // 参数是配置数组的下标，不是迭代次数/秒数本身；不同优化阶段可选择不同预算档位。
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  // cost_code 是上述 bit 的组合，例如 NORMAL_PHASE|ENDPOINT；日志仅用于确认本轮启用了哪些项。
  cost_function_ = cost_code;

  // 将位掩码翻译成可读日志，不参与目标函数计算。
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  ROS_INFO_STREAM("cost func: " << cost_str);
}

// GUIDE_PHASE 要求每个待优化内部控制点都有一个对应引导点，通常数量为控制点总数减 2*order_。
void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>&             waypt_idx) {
  // waypt_idx[i] 指定用 q[idx:idx+2] 计算第 i 个三阶均匀 B-spline knot 点。
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  /*
   * 一站式入口。输入是 Nxd 控制点、均匀 knot 间隔、代价位掩码和两类停止预算的下标；
   * 输出仍是 Nxd 控制点，端点是否可动由 ENDPOINT bit 决定。函数只优化空间控制点，不调整 ts。
   */
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /*
   * 完整流程：确定自由控制点 -> 矩阵展平为 NLopt 变量 -> 反复回调 combineCost() ->
   * 在 costFunction() 中保存历史最低代价变量 -> 无论正常结束还是抛异常，都回写 best_variable_。
   */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    /*
     * 启用 ENDPOINT：仅固定开头 order_ 个控制点，后面的控制点（包括末端三个）均可动；
     * 同时把输入曲线的实际终点 (P[n-2]+4P[n-1]+P[n])/6 记作软约束目标 end_pt_。
     * 这允许末端导数随优化改变，但通过 endpoint cost 尽量保持终点位置。
     */
    variable_num_ = dim_ * (pt_num - order_);
    // 记录输入曲线的端点，作为下面 ENDPOINT 软代价的参考目标。
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    /*
     * 未启用 ENDPOINT：首尾各固定 order_ 个控制点，只优化中间部分。对项目使用的三阶
     * B-spline，端部三个控制点共同决定端点位置、速度和加速度，所以这等价于保护边界状态。
     */
    variable_num_ = max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /*
   * isQuadratic() 识别项目实际使用的纯平方目标并选择 algorithm1_；含 ESDF/超限 hinge 的
   * 一般目标选择 algorithm2_。停止条件同时包含最大回调次数、墙上时间和相对变量容差，
   * 任何一个先满足都可终止本轮优化。
   */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  vector<double> q(variable_num_);
  // 自由控制点按 [P_order.x,P_order.y,...,P_{order+1}.x,...] 的行优先顺序展平。
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    // 位置控制点各分量限制在初值 +/-10 m 内，防止数值搜索远离局部问题；1D yaw 不加此界。
    vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;
    // NLopt 会原地更新 q，但最终回写使用回调保存的历史最优 best_variable_，见下方说明。
    nlopt::result result = opt.optimize(q, final_cost);

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    /*
     * 回写历史最低 cost 对应的变量，而不是盲目采用 NLopt 最后一次评估点。这样即使达到时间上限
     * 或求解器抛异常，只要至少完成过一次回调，仍能保留本轮见过的最好控制点。
     */
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, double& cost,
                                          vector<Eigen::Vector3d>& gradient) {
  /*
   * 三阶 B-spline 的 jerk 与控制点三阶差分
   *   J_i=q_{i+3}-3q_{i+2}+3q_{i+1}-q_i
   * 成正比，因此最小化 sum ||J_i||^2 可抑制 jerk。这里省略了只与固定 ts 有关的比例因子，
   * 由 lambda1_ 吸收；实现也按三阶公式写死，所以项目配置 order_=3 是重要前提。
   * 输出 gradient[k] 是该代价对完整控制点 q[k] 的解析梯度。
   */
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    // 每个 J_i 只影响连续四个控制点，梯度系数对应 [-1,3,-3,1]。
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  /*
   * 对可动范围内的控制点查询 ESDF。只有 d<dist0_ 时产生平方 hinge：
   *   f_dist=(d-dist0_)^2，grad=2(d-dist0_)*nabla d。
   * 代码把 ESDF 梯度归一化后使用，主要保留“远离障碍”的方向。这里检查的是控制点而非密集曲线
   * 采样点；B-spline 的局部支撑和凸包性质使移动控制点能平滑地推开附近轨迹段。
   */
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  // 末端固定时不必对最后 order_ 个点计避障；ENDPOINT 模式下它们可动，故纳入代价。
  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, double& cost,
                                           vector<Eigen::Vector3d>& gradient) {
  /*
   * 对均匀 B-spline，速度和加速度曲线的控制点正好分别是
   * v_i=Delta q_i/ts、a_i=Delta^2 q_i/ts^2：求导公式中的次数会与相应 knot 跨度相消。
   * 若其平方超过 max_vel_^2 / max_acc_^2，则对超出量再平方，形成连续可导的软惩罚；
   * 未超限的导数控制点代价和梯度均为 0。基于凸包性质，导数控制点逐轴满足限制即可保证
   * 连续导数曲线逐轴满足限制；这里仍采用软惩罚，优化结束后由 checkFeasibility() 和
   * reallocateTime() 做最终检查与时间调整。
   */
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // 预计算 1/ts^2、1/ts^4，因为下面直接比较速度平方和加速度平方。
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  // vd=(Delta q/ts)^2-vmax^2；正半轴上的代价 vd^2 对相邻两点给出方向相反的梯度。
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; j++) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  // ad=(Delta^2 q/ts^2)^2-amax^2；二阶差分对三点的梯度系数为 [1,-2,1]。
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient) {
  /*
   * ENDPOINT 是软代价而非 NLopt 等式约束。三阶均匀 B-spline 的实际终点为最后三个控制点的
   * [1,4,1]/6 加权和；惩罚它与 optimize() 记录的 end_pt_ 的平方距离，并按相同基函数权重
   * 把梯度分回三个控制点。
   */
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // 实际端点误差 dq 由末三个控制点的三阶均匀基函数权重计算。
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient) {
  /*
   * WAYPOINTS 用于 yaw 等指定采样约束。waypt_idx 指向三个局部控制点的起始下标，实际曲线点
   * 同样是 [1,4,1]/6 加权和；平方误差及梯度只影响这三个具有局部支撑的控制点。
   */
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/*
 * GUIDE 不把 guide_pts_ 当作必须经过的曲线航点，而是给每个内部控制点分配一个几何参考点，
 * 惩罚二者距离。它先把控制多边形拉入正确的拓扑通道，随后 NORMAL_PHASE 再负责真实避障。
 */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient) {
  // GUIDE_PHASE 首尾控制点固定，因此 guide_pts_[0] 对应 q[order_]。
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /*
   * 输入 x/输出 grad 都采用 NLopt 的一维布局；本函数先恢复完整控制点序列 g_q_，再计算
   * f=sum lambda_i*f_i，最后只把自由控制点的总梯度展平回 grad。固定端点虽然不出现在 x 中，
   * 仍保留在 g_q_ 中参与相邻差分和曲线局部支撑计算。
   */

  // 缓存统一用 Vector3d；1D/2D 曲线把不存在的高维分量补 0，各单项函数即可共用实现。
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    // 普通模式把尾部固定控制点从原 control_points_ 补回；ENDPOINT 模式没有固定尾部。
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*
   * 位标志决定是否调用某项；lambda 映射为 1平滑、2距离、3可行性、4终点、5引导、7航点。
   * 每项先生成对“全部控制点”的梯度，下面再截取从 order_ 开始的自由变量部分并加权累加。
   */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  /*
   * NLopt 要求 C 风格静态回调，func_data 保存 this 指针。每次回调都同时返回标量目标和解析
   * 梯度，并计入 iter_num_；NLopt 的 maxeval 实际限制的正是这类目标函数评估次数。
   */
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /*
   * best_variable_ 始终对应本轮迄今最低 cost，而不是最后一次 x。optimize() 最终统一回写它，
   * 这也是本类应对 maxtime、maxeval 或 NLopt 异常终止的关键数据流。
   */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  // 仅做 N x 3 矩阵到控制点向量的表示转换，不执行 B-spline 求值。
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  /*
   * GUIDE_PHASE 和 SMOOTHNESS|WAYPOINTS 都是“线性控制点组合的平方和”，可使用 quadratic
   * 配置 algorithm1_。含 ESDF 或超限 hinge 的 NORMAL_PHASE 不是纯二次型，走 algorithm2_。
   * 这里按项目实际出现的完整位组合精确匹配，而不是逐项自动证明目标类型。
   */
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner
