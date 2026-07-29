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



#include <plan_env/edt_environment.h>

namespace fast_planner {
/* ============================== edt_environment ==============================
 */
void EDTEnvironment::init() {
}

void EDTEnvironment::setMap(shared_ptr<SDFMap> map) {
  // 保存地图指针，并缓存分辨率倒数，梯度插值时会频繁使用。
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

void EDTEnvironment::setObjPrediction(ObjPrediction prediction) {
  this->obj_prediction_ = prediction;
}

void EDTEnvironment::setObjScale(ObjScale scale) {
  this->obj_scale_ = scale;
}

double EDTEnvironment::distToBox(int idx, const Eigen::Vector3d& pos, const double& time) {
  // Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluate(time);
  // 预测动态障碍在给定时间的位置，并计算点到该障碍 AABB 的欧氏距离。
  Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);

  Eigen::Vector3d box_max = pos_box + 0.5 * obj_scale_->at(idx);
  Eigen::Vector3d box_min = pos_box - 0.5 * obj_scale_->at(idx);

  Eigen::Vector3d dist;

  for (int i = 0; i < 3; i++) {
    dist(i) = pos(i) >= box_min(i) && pos(i) <= box_max(i) ? 0.0 : min(fabs(pos(i) - box_min(i)),
                                                                       fabs(pos(i) - box_max(i)));
  }

  return dist.norm();
}

double EDTEnvironment::minDistToAllBox(const Eigen::Vector3d& pos, const double& time) {
  // 遍历所有预测障碍物，取最近距离。静态环境可跳过这条路径。
  double dist = 10000000.0;
  for (int i = 0; i < obj_prediction_->size(); i++) {
    double di = distToBox(i, pos, time);
    if (di < dist) dist = di;
  }

  return dist;
}

void EDTEnvironment::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
  // 查询包围目标点的 8 个栅格中心距离，为三线性插值准备数据。
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        dists[x][y][z] = sdf_map_->getDistance(pts[x][y][z]);
      }
    }
  }
}

void EDTEnvironment::interpolateTrilinear(double values[2][2][2],
                                          const Eigen::Vector3d& diff,
                                          double& value,
                                          Eigen::Vector3d& grad) {
  // 三线性插值：把离散 ESDF 转成连续距离，并同时计算空间梯度。
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv_;
}

void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector3d& pos,
                                         double time, double& dist,
                                         Eigen::Vector3d& grad) {
  // 优化器调用的主要接口：连续查询位置处的距离和梯度。
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinear(dists, diff, dist, grad);
}

double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector3d& pos, double time) {
  // 粗略距离查询：静态模式只查 ESDF，动态模式还取预测障碍距离的较小值。
  double d1 = sdf_map_->getDistance(pos);
  if (time < 0.0) {
    return d1;
  } else {
    double d2 = minDistToAllBox(pos, time);
    return min(d1, d2);
  }
}

// EDTEnvironment::
}  // namespace fast_planner
