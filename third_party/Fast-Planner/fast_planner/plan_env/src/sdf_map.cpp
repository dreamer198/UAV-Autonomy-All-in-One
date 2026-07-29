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



#include "plan_env/sdf_map.h"
#include <cstdint>
#include <limits>
#include <stdexcept>

/*
 * 阅读导航：SDFMap 同时承担“局部占据建图”和“ESDF 生成”两项工作。
 *
 *   深度图 + 相机位姿
 *       -> projectDepthImage()          像素反投影到世界坐标系
 *       -> raycastProcess()             端点 hit、沿途 miss，融合进 log-odds 占据栅格
 *       -> clearAndInflateLocalMap()    清理局部旧数据，并按机体安全半径膨胀障碍
 *       -> updateESDF3d()               对膨胀后的二值障碍做三轴距离变换
 *
 *   世界坐标系点云 + odom
 *       -> cloudCallback()              跳过概率融合，直接构造局部膨胀障碍
 *       -> updateESDF3d()
 *
 * 头文件中的 mp_（MappingParameters）是 initMap() 后基本不变的配置和派生量；
 * md_（MappingData）是每帧变化的图像、位姿、栅格缓存、局部边界和 dirty flags。
 * 最终供规划器查询的是 md_.distance_buffer_all_，其障碍集合已经包含 inflation。
 *
 * 单位约定：世界坐标、地图尺寸、射线长度和 ESDF 均为米；栅格索引无量纲；相机内参
 * 以像素为单位；深度原始值除以 k_depth_scaling_factor_ 后得到米。传感器位姿和点云必须
 * 与 frame_id_ 所代表的地图坐标系一致，本类不查询 TF。
 */

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void SDFMap::initMap(ros::NodeHandle& nh) {
  node_ = nh;

  /*
   * 地图几何参数（米）。resolution_ 是立方体素边长；local_update_range_ 是以当前相机
   * 为中心的三个方向半宽；obstacles_inflation_ 是构造规划用障碍时的膨胀半径。
   */
  double x_size, y_size, z_size;
  node_.param("sdf_map/resolution", mp_.resolution_, -1.0);
  node_.param("sdf_map/map_size_x", x_size, -1.0);
  node_.param("sdf_map/map_size_y", y_size, -1.0);
  node_.param("sdf_map/map_size_z", z_size, -1.0);
  node_.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  /*
   * The upstream PointCloud2 path hard-coded vertical inflation to one voxel.
   * Retain that default for upstream-compatible launch files, while allowing
   * a plugin profile to reserve additional vertical tracking clearance.
   */
  node_.param("sdf_map/obstacles_inflation_z", mp_.obstacles_inflation_z_,
              mp_.resolution_);
  /*
   * The native PointCloud2 callback treats every message as a complete local
   * snapshot and clears the previous one.  A registered spinning-lidar cloud
   * is only a partial view, so static deployments may retain prior occupied
   * voxels.  Keep the upstream snapshot behavior unless explicitly enabled.
   */
  node_.param("sdf_map/accumulate_cloud", mp_.accumulate_cloud_, false);

  /* 针孔相机内参（像素），projectDepthImage() 假设深度沿相机坐标系 +Z 方向。 */
  node_.param("sdf_map/fx", mp_.fx_, -1.0);
  node_.param("sdf_map/fy", mp_.fy_, -1.0);
  node_.param("sdf_map/cx", mp_.cx_, -1.0);
  node_.param("sdf_map/cy", mp_.cy_, -1.0);

  /*
   * 深度预处理参数。skip_pixel_ 控制二维下采样步长；margin 排除图像边缘；有效深度
   * 范围以米计。输入若是 16UC1，约定 depth_m = raw / k_depth_scaling_factor_；32FC1
   * 会在回调中先按相同比例转换为 16UC1。
   */
  node_.param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

  /*
   * 逆传感器模型。p_hit/p_miss 是一次观测的占据/空闲证据；p_min/p_max 限制累计
   * 置信度，p_occ 是最终二值化阈值。min_ray_length_ 在当前实现中未启用，实际只用
   * max_ray_length_ 截断远距离射线。
   */
  node_.param("sdf_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("sdf_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("sdf_map/p_min", mp_.p_min_, 0.12);
  node_.param("sdf_map/p_max", mp_.p_max_, 0.97);
  node_.param("sdf_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

  /* 可视化切片/截断高度和虚拟天花板均是 frame_id_ 下的绝对 z 坐标（米）。 */
  node_.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
  node_.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

  node_.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
  // pose_type_: 1 = depth + PoseStamped，2 = depth + Odometry；二者均使用近似时间同步。
  node_.param("sdf_map/pose_type", mp_.pose_type_, 1);

  node_.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
  node_.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("sdf_map/ground_height", mp_.ground_height_, 1.0);
  double max_memory_mb;
  node_.param("sdf_map/max_memory_mb", max_memory_mb, 512.0);

  if (!std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0 ||
      !std::isfinite(x_size) || !std::isfinite(y_size) || !std::isfinite(z_size) ||
      x_size <= 0.0 || y_size <= 0.0 || z_size <= 0.0 ||
      !std::isfinite(mp_.obstacles_inflation_) ||
      !std::isfinite(mp_.obstacles_inflation_z_) ||
      mp_.obstacles_inflation_ < 0.0 || mp_.obstacles_inflation_z_ < 0.0 ||
      !std::isfinite(max_memory_mb) || max_memory_mb <= 0.0 ||
      mp_.skip_pixel_ <= 0) {
    throw std::invalid_argument("invalid SDF map geometry, memory limit, or pixel stride");
  }

  // ESDF 更新框至少向外留一个体素，避免距离变换恰好截断在最新观测边缘。
  mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
  mp_.resolution_inv_ = 1 / mp_.resolution_;
  // 保持上游默认的居中行为，同时允许固定地图 profile 显式给出最小角。
  double origin_x = -x_size / 2.0;
  double origin_y = -y_size / 2.0;
  double origin_z = mp_.ground_height_;
  node_.param("sdf_map/origin_x", origin_x, origin_x);
  node_.param("sdf_map/origin_y", origin_y, origin_y);
  node_.param("sdf_map/origin_z", origin_z, origin_z);
  if (!std::isfinite(origin_x) || !std::isfinite(origin_y) || !std::isfinite(origin_z)) {
    throw std::invalid_argument("SDF map origin must be finite");
  }
  mp_.map_origin_ = Eigen::Vector3d(origin_x, origin_y, origin_z);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  /*
   * 将概率转成 L = log(p / (1-p))。独立观测在 log-odds 空间可直接相加，最后再钳制
   * 到 [clamp_min_log_, clamp_max_log_]，避免少量历史观测让体素永远无法翻转状态。
   */
  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // 每轴向上取整覆盖配置尺寸；合法索引是闭区间 [0, map_voxel_num_-1]。
  uint64_t voxel_count = 1;
  for (int i = 0; i < 3; ++i) {
    const double axis_voxels = ceil(mp_.map_size_(i) / mp_.resolution_);
    if (!std::isfinite(axis_voxels) || axis_voxels < 1.0 ||
        axis_voxels > double(std::numeric_limits<int>::max())) {
      throw std::overflow_error("SDF map axis voxel count is invalid");
    }
    mp_.map_voxel_num_(i) = static_cast<int>(axis_voxels);
    if (voxel_count > std::numeric_limits<uint64_t>::max() /
                          static_cast<uint64_t>(mp_.map_voxel_num_(i))) {
      throw std::overflow_error("SDF map voxel count overflow");
    }
    voxel_count *= static_cast<uint64_t>(mp_.map_voxel_num_(i));
  }
  if (voxel_count > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
    throw std::overflow_error("SDF map exceeds the 32-bit address space used by Fast-Planner");
  }
  // Seven numeric distance/occupancy scratch buffers plus flags/counters are
  // conservatively budgeted at 64 bytes per voxel.
  const long double estimated_bytes = static_cast<long double>(voxel_count) * 64.0L;
  const long double max_bytes = static_cast<long double>(max_memory_mb) * 1024.0L * 1024.0L;
  if (estimated_bytes > max_bytes) {
    throw std::length_error("SDF map allocation exceeds sdf_map/max_memory_mb");
  }

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  /*
   * 坐标约定集中在头文件内联函数中：posToIndex() 对 (pos-origin)/resolution 取 floor，
   * indexToPos() 返回体素中心；isInMap(pos) 按上述物理 AABB 判断并给盒面留 1e-4 裕量，
   * boundIndex() 则把越界索引钳到最近合法体素。由于体素数向上取整，最后一格可能在
   * 几何上略超出配置的 map_size，位置输入是否合法仍以 map_*_boundary_ 为准。
   */
  mp_.map_min_idx_ = Eigen::Vector3i::Zero();
  mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  /*
   * 所有 3D 栅格都展平成一维：address = x * Ny * Nz + y * Nz + z，故 z 连续。
   * occupancy_buffer_ 实际存 log-odds，不是概率。初值比 clamp_min_log_ 再低
   * unknown_flag_，用这个越过正常钳制下界的哨兵区分 unknown 与 known free。
   *
   * occupancy_buffer_inflate_：规划使用的膨胀后二值障碍；
   * distance_buffer_ / distance_buffer_neg_：到障碍 / 到自由空间的非负距离（米）；
   * distance_buffer_all_：二者合成的 signed ESDF；自由空间为正，膨胀障碍内非正，
   * 最外层障碍体素通常为 0，更深的内部体素通常为负。
   */

  int buffer_size = static_cast<int>(voxel_count);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.distance_buffer_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

  /*
   * 以下计数和标签只服务于一次 raycast 批次：先聚合同一体素收到的 hit/miss，随后每个
   * 体素只执行一次 log-odds 更新；标签用批次号去重，免去每帧清空整张地图的 O(N) 开销。
   */
  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  // 两个 tmp buffer 保存分轴 EDT 的中间“平方栅格距离”，最后才 sqrt 并乘 resolution_。
  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);
  md_.raycast_num_ = 0;

  // 固定容量按 640x480 和 skip_pixel_ 预分配；运行配置应保证实际投影点数不超过此容量。
  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /*
   * 深度输入路径由 pose_type_ 选择同步消息类型。同步回调只保存“最新一帧”并置 dirty
   * flag，真正的投影和融合由 20 Hz 定时器完成；如果传感器更快，中间帧会被合并掉。
   */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/sdf_map/depth", 50));

  if (mp_.pose_type_ == POSE_STAMPED) {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/sdf_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));

  } else if (mp_.pose_type_ == ODOMETRY) {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/sdf_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&SDFMap::depthOdomCallback, this, _1, _2));
  }

  /*
   * 点云 + odom 订阅始终创建，常用于仿真的 pcl_render_node。通常通过 launch/remap 只
   * 驱动选定的一条输入路径；点云回调不使用 pose_type_，也不会将点从机体系变换到世界系。
   */

  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("/sdf_map/odom", 10, &SDFMap::odomCallback, this);

  // 三个 20 Hz 定时器分别处理占据融合、ESDF 和可视化，dirty flags 串起前两阶段。
  occ_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateOccupancyCallback, this);
  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::visCallback, this);

  // 发布消息统一使用 frame_id_；这些 topic 主要供 RViz/调试，不参与规划器内部查询。
  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

  /*
   * dirty flag 状态机（深度路径）：
   *   sensor callback -> occ_need_update_
   *   raycastProcess   -> local_updated_
   *   occupancy timer  -> esdf_need_update_
   *   ESDF timer       -> 全部清零，等待下一帧
   * 点云路径已经直接得到 inflated occupancy，因此从 cloudCallback 直接置 esdf_need_update_。
   */
  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.esdf_need_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;

  md_.esdf_time_ = 0.0;
  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_esdf_time_ = 0.0;
  md_.max_fuse_time_ = 0.0;

  rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  rand_noise2_ = normal_distribution<double>(0, 0.2);
  random_device rd;
  eng_ = default_random_engine(rd());
}

void SDFMap::resetBuffer() {
  // 全图版本主要用于显式复位；局部点云模式通常调用下面带范围的重载。
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {

  /*
   * 输入是世界坐标，转换后将两端索引钳制在地图内，循环边界均包含。
   * 注意这里只清 occupancy_buffer_inflate_ 和正距离缓存，不清概率 occupancy；这是为
   * cloudCallback 的“每帧重建局部二值地图”服务。combined ESDF 随后的 updateESDF3d()
   * 会覆盖 local_bound 内对应区域。
   */
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_.distance_buffer_[toAddress(x, y, z)] = 10000;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  /*
   * Felzenszwalb/Huttenlocher 一维平方距离变换，计算
   *
   *   D(q) = min_i ((q - i)^2 + f(i))
   *
   * f(i) 在“特征体素”处为 0，其余为 +inf；后续分轴调用时 f(i) 也可以是上一轴已经
   * 算出的平方距离。v[] 保存下包络中仍可能成为最近点的抛物线中心，z[] 保存相邻
   * 抛物线交点。先构造下包络，再线性扫描求值，因此一条轴线复杂度为 O(n)。dim 仅
   * 用来确定临时数组长度，实际读写位置由两个 lambda 捕获的另外两维决定。
   */
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  /*
   * 仅重算最近一次观测得到的局部闭区间 [local_bound_min_, local_bound_max_]。
   * 三维欧氏距离的平方可分离为 dx^2 + dy^2 + dz^2，所以依次沿 z、y、x 做三次
   * 一维 EDT 即可得到当前 local_bound 内特征集合的精确栅格欧氏距离，而不必对每个
   * 体素搜索该局部范围内的所有障碍点。局部框外的障碍不会参与本轮距离变换。
   */
  Eigen::Vector3i min_esdf = md_.local_bound_min_;
  Eigen::Vector3i max_esdf = md_.local_bound_max_;

  /*
   * 正距离 pass 1/3（z 轴）：膨胀障碍体素是特征点 f=0，其他体素 f=+inf。
   * 因而最终 distance_buffer_ 在自由空间表示到最近膨胀障碍的距离，在障碍内为 0。
   * unknown 没有写入 inflated buffer，故在此距离变换中与自由空间同样处理。
   */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  // 正距离 pass 2/3（y 轴）：tmp_buffer1_ -> tmp_buffer2_，值仍是体素数的平方。
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  // 正距离 pass 3/3（x 轴）：开平方并乘体素边长，转换成米。
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val),
                 //      md_.distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /*
   * 构造互补二值图：原 inflated==0 的体素变成特征点 1，障碍变成 0。随后对这个图
   * 重复三轴 EDT，就得到障碍内部到最近自由体素的距离。自由体素本身的负距离为 0。
   */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] == 0) {
          md_.occupancy_buffer_neg[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;

  // 负距离同样按 z -> y -> x 三轴计算，临时 buffer 可直接复用。
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                            y * mp_.map_voxel_num_(2) + z] == 1 ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[toAddress(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[toAddress(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_neg_[toAddress(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /*
   * 合成 signed ESDF：
   *   自由体素：d_neg == 0，保留 d_pos > 0；
   *   障碍体素：d_pos == 0，写成 -d_neg + resolution_。
   * 加一个体素边长用于补偿“从体素中心到最近自由体素中心”的一格偏移，使最外层障碍
   * 体素约为 0、越深入障碍数值越负。这里的零表面对应膨胀障碍，而非原始点云表面。
   */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = toAddress(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
      }
}

int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
  /*
   * 暂存一次体素观测而不立刻改全局 log-odds。occ=1 表示射线端点 hit，occ=0 表示
   * no-return 端点或射线穿越产生的 miss。一个批次内同一体素可能被许多射线触及：
   * count_hit_and_miss_ 记总票数，count_hit_ 记 hit 票数，第一次出现时才把索引入队。
   * raycastProcess() 最后消费队列并清零计数。调用方必须先保证 pos 位于地图内。
   */
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1) md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void SDFMap::projectDepthImage() {
  /*
   * 将深度像素 (u,v,d) 反投影为相机坐标
   *
   *   p_c = ((u-cx)d/fx, (v-cy)d/fy, d)
   *   p_w = R_wc * p_c + t_wc
   *
   * 因此 camera_q_ 必须表示“相机系到地图系”的旋转，camera_pos_ 是地图系位置。
   * 输出 proj_points_ 只是当前批次的有效数组，真实长度由 proj_points_cnt 指定。
   */
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t* row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

  // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  if (!mp_.use_depth_filter_) {
    /*
     * 无滤波路径逐像素投影，不跳采样，也不在这里检查 0/过近/过远深度；过远点稍后由
     * raycastProcess() 截到 max_ray_length_，而原始 0 深度会投影到相机位置附近。
     */
    for (int v = 0; v < rows; v++) {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++) {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240) std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else {

    /*
     * 第一帧只用于初始化 last_*，不产生投影点。虽然跨帧一致性检查目前被 if(false)
     * 禁用，代码仍保留了这一帧预热行为，所以首次有效 raycast 从第二帧深度开始。
     */
    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else {
      // 启用滤波时按 skip_pixel_ 下采样，并跳过 margin 区域以避免访问图像边界。
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_) {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          /*
           * 深度分类：过近点直接丢弃；无返回和超过 filter_maxdist 的点被改成略大于
           * max_ray_length_，随后 raycast 将其截断并把端点记为 miss。这样会清空可见
           * 空间，但不会在最大量程处凭空制造障碍。
           *
           * 注意 row_ptr 在读取 depth 后已经前移，当前代码的 *row_ptr==0 实际检查下一
           * 个采样位置而非刚读取位置；这里保留原始行为，排查深度空洞时应留意这一点。
           */
          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0) {
            depth = mp_.max_ray_length_ + 0.1;
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // 所有通过筛选的点都投影到地图坐标系，后续 raycasting 只处理世界坐标。
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          /*
           * 预留的跨帧一致性检查：将当前世界点重投影到上一帧，并用 tolerance 比较深度。
           * 外层固定为 false，因此当前版本不会执行，也不会利用 depth_filter_tolerance_。
           */
          // check consistency with last image, disabled...
          if (false) {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_) {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            } else {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* 无论本帧是否产生投影点，都保存深度和位姿，供下一帧的预留一致性检查使用。 */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void SDFMap::raycastProcess() {
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0) return;

  /*
   * 一帧深度的核心融合：每个投影点定义 camera_pos_ -> point 的视线。
   *
   *   有效且量程内的测量端点       -> hit
   *   地图外或超过 max_ray_length_ -> 截断后的端点为 miss
   *   端点与相机之间穿过的体素     -> miss
   *
   * 本函数先把证据聚合进 cache，最后每个体素只做一次有界 log-odds 更新。该设计既减少
   * 同帧密集射线造成的过度自信，也允许“端点 hit 与穿越 miss”在同一体素内投票消歧。
   */
  ros::Time t1, t2;

  // 批次号作为 flag_rayend_/flag_traverse_ 的时间戳，不必每帧清空两个全图标签数组。
  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // 记录相机与截断后端点的轴对齐包围盒；后续 ESDF 仅在该局部索引范围内重算。
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    /*
     * 先规范化射线端点：
     * 1. 地图外的点沿 camera->point 方向裁到地图盒内部；
     * 2. 裁剪后若仍超量程，再缩到 max_ray_length_；
     * 3. 只要发生地图/量程截断，端点都是 miss；原始点在地图内且量程内才是 hit。
     * 这样地图边界和传感器量程边界不会被错误当成真实障碍表面。
     */

    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);

    } else {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    /*
     * 同一投影体素可能对应许多相邻像素。端点证据已经在上面累计，但每个唯一端点只
     * raycast 一次，避免对重合视线重复清空自由空间。
     */

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    /*
     * RayCaster 在“体素坐标”中从端点反向走向相机。step() 返回整数体素坐标；加 half
     * 后再乘 resolution_ 得到该体素中心的世界坐标。遍历包含端点体素、不包含相机
     * 体素，因此端点常同时收到一票 hit 和一票 miss；后面的 hit>=miss 规则使平票仍
     * 判为 hit。
     */
    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        /*
         * 一条新射线一旦进入本批次已经遍历过的体素，朝相机方向的剩余部分通常也与
         * 旧射线重合，因此直接 break，进一步减少重复 miss 更新。
         */
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  /*
   * 更新框必须覆盖相机，否则射线途经区域可能漏算。z 上界至少包含 ground_height_；
   * x/y 再按 local_bound_inflate_ 向外扩展，z 方向当前不扩展。boundIndex() 最终将
   * 闭区间裁入全局地图。
   */
  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // 该标志通知 occupancy timer：本轮确有射线更新，需要重建 inflated map 并触发 ESDF。
  md_.local_updated_ = true;

  /*
   * 这段逻辑的意图是只保留相机周围 local_update_range_ 内的历史：队列中落在该
   * 范围外、且未被下面的同向饱和分支提前跳过的体素，会先退回最低置信度，再应用
   * 当前批次证据，防止局部地图随飞行不断积累远处旧障碍。
   */
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty()) {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    /*
     * 每个体素按多数票选择一次 hit 或 miss 增量，而不是把本帧所有像素证据逐票累加；
     * 平票偏向 hit，是更保守的碰撞策略。取出后立即清零临时计数，为下批次复用。
     */
    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
        mp_.prob_hit_log_ :
        mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // 已饱和且新证据继续同向时可提前结束；否则加法后再钳制到合法 log-odds 范围。
    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  /*
   * 求 camera_pt 沿 pt-camera_pt 射线第一次与地图 AABB 相交的位置。对三个轴分别计算
   * 到 min/max 平面的正参数 t，取最小值即首先碰到的盒面，再减一个很小的参数量保证
   * 返回点严格位于 isInMap() 接受的范围内。调用处假定 camera_pt 本身在地图内。
   * 函数名 closet 是原项目拼写，实际含义是 closest point on map boundary。
   */
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() {
  /*
   * 深度融合完成后的二值化阶段，分两步：
   * 1. 将局部窗口刚离开的邻接薄壳恢复为 unknown，抹掉随机器人移动而过期的数据；
   * 2. 把 local_bound 内 log-odds 超过 p_occ 的体素膨胀到 occupancy_buffer_inflate_。
   * 后续搜索和 ESDF 都只看 inflated buffer，因此 inflation 是规划安全边界的一部分。
   */
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  /*
   * min_cut/max_cut 是观测框外再留 local_map_margin_ 个体素；min_cut_m/max_cut_m 又向外
   * 扩 5 格。下面三个双循环分别清 z、y、x 方向的两片薄壳，而非扫描/清空整幅全局图。
   * occupancy 写回 unknown 哨兵，combined distance 写 10000 表示尚无有效 ESDF。
   */
  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

      for (int z = min_cut_m(2); z < min_cut(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

      for (int y = min_cut_m(1); y < min_cut(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

      for (int x = min_cut_m(0); x < min_cut(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  /*
   * 膨胀半径先向上取整为体素步数，inflatePoint() 当前生成完整立方邻域，即按 L_inf
   * 距离膨胀而非球形欧氏膨胀。这样略保守；配置值只要大于 0，通常至少膨胀一格。
   */

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // 先清当前更新框的旧二值结果，再完全由最新概率 occupancy 重建，避免障碍只增不减。
  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // 只有 log-odds 严格高于 p_occ 对应阈值的原始体素才作为膨胀种子。
  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  /*
   * 可选虚拟天花板：把指定世界 z 对应的一整层体素直接标成障碍，限制飞行高度。
   * 配置必须确保 ceil_id 位于地图 z 索引范围内；该约束仅在深度融合路径中加入。
   */
  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void SDFMap::visCallback(const ros::TimerEvent& /*event*/) {
  /*
   * 当前默认只发布膨胀障碍点云。其余发布函数保留作按需调试，取消下面注释即可在 RViz
   * 查看局部更新框、ESDF 水平切片、unknown 体素或深度反投影点。
   */
  publishMap();
  publishMapInflate(false);
  // publishUpdateRange();
  // publishESDF();

  // publishUnknown();
  // publishDepth();
}

void SDFMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
  // dirty gate 让定时器在没有新同步深度时立即返回，避免重复融合同一帧。
  if (!md_.occ_need_update_) return;

  /*
   * 深度路径的前半流水线：投影 -> 射线概率融合 -> 局部清理与障碍膨胀。
   * projectDepthImage() 首帧预热或没有有效像素时，raycast 不会置 local_updated_，因而
   * 不会误触发 ESDF。计时覆盖这三个阶段，不含后续距离变换。
   */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  projectDepthImage();
  raycastProcess();

  if (md_.local_updated_) clearAndInflateLocalMap();

  t2 = ros::Time::now();

  md_.fuse_time_ += (t2 - t1).toSec();
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  if (mp_.show_occ_time_)
    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  /*
   * 消费本帧 dirty flag。只有 raycastProcess() 确认局部有更新时才把下一阶段置 dirty；
   * local_updated_ 是 occupancy timer 内部的一次性握手标志，用完即清。
   */
  md_.occ_need_update_ = false;
  if (md_.local_updated_) md_.esdf_need_update_ = true;
  md_.local_updated_ = false;
}

void SDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  // 深度路径由 occupancy timer 置位；点云路径由 cloudCallback 直接置位。
  if (!md_.esdf_need_update_) return;

  /*
   * 流水线后半段：在 md_.local_bound_ 闭区间重算 signed ESDF。计算完成后直接覆盖查询
   * 缓存，无需再发布 ROS 消息；搜索器/优化器通过 SDFMap/EDTEnvironment 内存接口读取。
   */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();

  md_.esdf_time_ += (t2 - t1).toSec();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
}

void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  /*
   * pose_type_=POSE_STAMPED 的入口。ApproximateTime 已把时间相近的 depth/pose 配成一对；
   * 回调只覆盖 md_ 中的最新快照，耗时的投影和建图留给 occupancy timer。这里直接把
   * pose 当作相机光心在地图系的位姿，不额外应用机体到相机的外参或 TF。
   */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  // 统一内部存储为 16UC1；典型 k=1000 时，32FC1 的米转换成毫米整数。
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                     pose->pose.orientation.y, pose->pose.orientation.z);
  /*
   * 相机在全局地图内才允许建图，否则防止 raycast 依赖的“射线起点在盒内”不变量被破坏。
   * has_odom_ 在这里更准确地表示“已有合法定位”，即便输入消息类型是 PoseStamped。
   */
  if (isInMap(md_.camera_pos_)) {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
  /*
   * 独立 odom 主要为 cloudCallback 提供局部窗口中心。启用深度滤波并完成首帧预热后，
   * 位姿应只来自与图像同步的回调，所以这里立即返回，避免异步 odom 覆盖配对好的
   * 相机位姿；未启用滤波时 has_first_depth_ 不由投影函数置位，需依赖启动时的话题配置
   * 避免两条输入路径同时驱动同一 md_。
   */
  if (md_.has_first_depth_) return;

  const auto& p = odom->pose.pose.position;
  if (odom->header.stamp.isZero() || odom->header.frame_id != mp_.frame_id_ ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    ROS_ERROR_THROTTLE(1.0, "SDFMap rejected invalid odometry contract");
    md_.has_odom_ = false;
    return;
  }

  // 点云输入模式下，odom 提供当前传感器/机体在世界系中的位置。
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

  /*
   * 独立点云输入路径与深度路径的关键差别：点必须已经位于地图/world 坐标系，本函数
   * 不使用消息 frame_id、odom 姿态或 TF 做坐标变换；odom 位置仅用于裁剪局部范围。
   * 该路径不做 raycasting，也不更新概率 occupancy_buffer_，而是直接更新二值
   * occupancy_buffer_inflate_，所以不能从点云推断射线沿途 free/unknown 状态。
   * accumulate_cloud=true 时只累积已观测 occupied voxel，适用于静态环境中的稀疏
   * 注册点云；进程重启会清空该插件私有地图。
   */
  if (img->header.stamp.isZero() || img->header.frame_id != mp_.frame_id_) {
    ROS_ERROR_THROTTLE(1.0, "SDFMap rejected point cloud outside the world-frame contract");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  if (!md_.has_odom_) {
    // std::cout << "no odom!" << std::endl;
    return;
  }

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;

  // Snapshot 模式清除上一帧局部障碍；静态累积模式保留已有 occupied voxel。
  const Eigen::Vector3d update_min = md_.camera_pos_ - mp_.local_update_range_;
  const Eigen::Vector3d update_max = md_.camera_pos_ + mp_.local_update_range_;
  if (!mp_.accumulate_cloud_) {
    this->resetBuffer(update_min, update_max);
  }

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  /*
   * Point-cloud mapping uses an anisotropic box: the existing scalar controls
   * x/y, while obstacles_inflation_z controls vertical clearance. This keeps
   * the default one-voxel upstream behavior but lets the PX4/SE3 integration
   * account for vertical tracking lag near wall tops.
   */
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = ceil(mp_.obstacles_inflation_z_ / mp_.resolution_);

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    // 只接收落在以 camera_pos_ 为中心、三轴 local_update_range_ 半宽内的世界系点。
    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2)) {

      // 直接枚举点周围的离散长方体并写入 inflated buffer，不经过 p_occ 阈值。
      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z) {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt)) continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  // ESDF 始终更新当前局部窗口。Snapshot 模式借此清除已消失障碍的旧距离场；
  // 静态累积模式则重新计算包含历史 occupied voxel 的局部距离场。
  min_x = update_min.x();
  min_y = update_min.y();
  min_z = update_min.z();
  max_x = update_max.x();
  max_y = update_max.y();
  max_z = update_max.z();

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // inflated occupancy 已经就绪，跳过 occupancy timer，直接唤醒 ESDF timer。
  md_.has_cloud_ = true;
  md_.esdf_need_update_ = true;
}

void SDFMap::publishMap() {
  /*
   * 发布局部规划障碍点云。紧随其后的整段注释代码是旧的“原始概率占据”发布方式；
   * 当前生效的实现读取 occupancy_buffer_inflate_，所以 /sdf_map/occupancy 实际也包含安全膨胀。
   * 每个点取体素中心的地图坐标，并用 visualization_truncate_height_ 裁掉过高部分。
   */
  // pcl::PointXYZ pt;
  // pcl::PointCloud<pcl::PointXYZ> cloud;

  // Eigen::Vector3i min_cut = md_.local_bound_min_ -
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  // Eigen::Vector3i max_cut = md_.local_bound_max_ +
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);

  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // for (int x = min_cut(0); x <= max_cut(0); ++x)
  //   for (int y = min_cut(1); y <= max_cut(1); ++y)
  //     for (int z = min_cut(2); z <= max_cut(2); ++z) {

  //       if (md_.occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_) continue;

  //       Eigen::Vector3d pos;
  //       indexToPos(Eigen::Vector3i(x, y, z), pos);
  //       if (pos(2) > mp_.visualization_truncate_height_) continue;

  //       pt.x = pos(0);
  //       pt.y = pos(1);
  //       pt.z = pos(2);
  //       cloud.points.push_back(pt);
  //     }

  // cloud.width = cloud.points.size();
  // cloud.height = 1;
  // cloud.is_dense = true;
  // cloud.header.frame_id = mp_.frame_id_;

  // sensor_msgs::PointCloud2 cloud_msg;
  // pcl::toROSMsg(cloud, cloud_msg);
  // map_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 可视范围是最近 ESDF 更新框外加半个 local_map_margin_，并钳制到合法地图索引。
  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void SDFMap::publishMapInflate(bool all_info) {
  /*
   * 显式发布 /sdf_map/occupancy_inflate。内容与 publishMap() 使用同一 inflated buffer；
   * 差异主要在范围：默认严格使用 local_bound，all_info=true 时向外扩完整 margin。
   */
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info) {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

void SDFMap::publishUnknown() {
  /*
   * unknown 由特殊 log-odds 哨兵判断，而不是 inflated buffer。它只对深度概率融合路径
   * 有完整语义；点云直写路径没有维护 occupancy_buffer_ 的已知/未知状态。
   */
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void SDFMap::publishDepth() {
  // 发布 projectDepthImage() 当前有效的世界系投影点，便于核对内参、深度尺度和相机位姿。
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void SDFMap::publishUpdateRange() {
  // 将最近一次 local_bound 的最小/最大体素中心转换为一个半透明 AABB Marker。
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void SDFMap::publishESDF() {
  /*
   * 把 esdf_slice_height_ 上的水平距离切片编码为 PointXYZI：距离钳制到 [0,3] 米并
   * 归一化进 intensity，所有显示点的 z 固定为 -0.2。它是 RViz 热度图，不是障碍点云；
   * 负距离会被截成 0。当前 visCallback 默认没有调用本函数。
   */
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
      Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                          vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad, int sign) {
  /*
   * 以用户给定采样间隔 res 遍历 [xmin,xmax] x [ymin,ymax] 水平区域，返回 (x,y,d)
   * 及三线性插值梯度。sign 参数是旧接口遗留，当前实现始终查询 combined signed ESDF。
   * 本函数只追加输出 vector，不负责 clear。
   */
  double dist;
  Eigen::Vector3d gd;
  for (double x = range(0); x <= range(1); x += res)
    for (double y = range(2); y <= range(3); y += res) {

      dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
      slice.push_back(Eigen::Vector3d(x, y, dist));
      grad.push_back(gd);
    }
}

void SDFMap::checkDist() {
  // 遍历全图调用距离/梯度插值的调试桩；空 if 表明当前版本没有实际检查或输出。
  for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
    for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
      for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);

        Eigen::Vector3d grad;
        double dist = getDistWithGradTrilinear(pos, grad);

        if (fabs(dist) > 10.0) {
        }
      }
}

bool SDFMap::odomValid() { return md_.has_odom_; }

bool SDFMap::hasDepthObservation() { return md_.has_first_depth_; }

double SDFMap::getResolution() { return mp_.resolution_; }

Eigen::Vector3d SDFMap::getOrigin() { return mp_.map_origin_; }

int SDFMap::getVoxelNum() {
  return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  // ori 是地图 AABB 的最小角，size 是配置的三轴物理尺寸，单位均为米。
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],
                            Eigen::Vector3d& diff) {
  /*
   * 为三线性插值定位包围 pos 的 8 个体素中心。减去 0.5*resolution_ 是因为
   * posToIndex() 按体素边界取 floor，而插值节点位于体素中心；diff 是 pos 在这个
   * 2x2x2 单元中的归一化坐标，内部点各分量通常位于 [0,1]。这里只返回几何位置，
   * 实际距离值和解析梯度由头文件中的 getDistWithGradTrilinear() 读取/计算。
   */
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom) {
  /*
   * pose_type_=ODOMETRY 的同步入口，用途与 depthPoseCallback 相同：把近似同时间的 odom
   * 位姿和 depth 保存为一个建图快照，然后置 occ_need_update_。这里同样把 odom pose
   * 直接解释为相机在地图系的位姿，不应用机体-相机外参。
   *
   * 与 PoseStamped 版本不同，原实现不检查相机是否在地图内，也不更新 has_odom_/
   * update_num_；因此该模式依赖输入位姿本身满足 raycasting 的地图边界前提。
   */
  /* get pose */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  // 深度格式统一方式与 depthPoseCallback 相同：内部最终按 16UC1 原始尺度保存。
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
}

void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr& img) {
  // 早期独立订阅调试回调，initMap() 当前没有注册，仅打印时间戳。
  std::cout << "depth: " << img->header.stamp << std::endl;
}

void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
  // 早期独立订阅调试回调，initMap() 当前没有注册；正式路径使用同步 depthPoseCallback。
  std::cout << "pose: " << pose->header.stamp << std::endl;

  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
}

// SDFMap
