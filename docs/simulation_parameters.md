# 仿真参数说明

本文说明执行 `./launch/sim.sh start` 时实际使用的参数、加载顺序、默认值和修改方式。这里的“仿真参数”分为 Gazebo 物理模型、PX4 SITL、MAVROS、定位适配、Diff-Planner、轨迹服务器和 SE3 控制器；它们不是从一个文件统一加载。

## 1. 加载顺序

```text
launch/sim.sh 中的环境变量与启动开关
  -> PX4 outdoor_mid360.launch
     -> Gazebo world + iris_mid360 SDF
     -> PX4 gazebo-classic_iris airframe
     -> MAVROS px4_config.yaml
  -> simulation localization.launch
  -> common planner.launch + planner.yaml
  -> common trajectory_converter.launch
  -> common controller.launch
     -> common controller.yaml
     -> simulation controller.yaml（同名项后加载并覆盖）
  -> simulation goal_bridge.launch + RViz
```

主要参数所有权如下。

| 类别 | 参数来源 | 是否需要重建仿真镜像 |
|---|---|---|
| 启动开关、原生起飞与 OFFBOARD 交接 | [`launch/sim.sh`](../launch/sim.sh) 环境变量 | 否 |
| 初始位姿、PX4/MAVROS 连接 | [`outdoor_mid360.launch`](../simulation/assets/px4/launch/outdoor_mid360.launch) | 是 |
| Gazebo 世界和物理步长 | [`ego_swarm.world`](../simulation/assets/px4/worlds/ego_swarm.world) | 是 |
| Iris 与 MID-360 组合、雷达外参 | [`iris_mid360.sdf`](../simulation/assets/px4/models/iris_mid360/iris_mid360.sdf) | 是 |
| MID-360 扫描参数 | [`Mid360.sdf`](../simulation/assets/px4/models/Mid360/Mid360.sdf) | 是 |
| MAVROS 插件参数 | [`px4_config.yaml`](../simulation/assets/px4/launch/px4_config.yaml) | 是 |
| Planner、地图和优化器 | [`planner.yaml`](../common/config/planner.yaml) | 否，重启 ROS 栈即可 |
| 轨迹预瞄和 yaw | [`trajectory_server.yaml`](../common/config/trajectory_server.yaml) | 否，重启 ROS 栈即可 |
| 公共控制安全策略 | [`common/controller.yaml`](../common/config/controller.yaml) | 否，重启 ROS 栈即可 |
| 仿真车辆控制标定 | [`simulation/controller.yaml`](../simulation/config/controller.yaml) | 否，重启 ROS 栈即可 |
| SE3 增益 | [`se3_ctrl.cpp`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp)、[`tune.cfg`](../third_party/Diff-Planner-PX4/src/se3_controller/cfg/tune.cfg) | 否；`sim.sh restart` 重建 overlay，当前不建议在线修改，见 11.3 节 |

## 2. `sim.sh` 启动参数

`sim.sh start` 默认启动 Gazebo GUI、Planner、轨迹转换器、SE3、RViz 目标桥和 RViz，但不会进入 OFFBOARD 或解锁。

| 环境变量 | 默认值 | 含义 |
|---|---:|---|
| `SIM_GAZEBO_GUI` | `true` | 是否启动 Gazebo GUI |
| `SIM_START_PLANNER` | `true` | 是否启动 Diff-Planner 和 `traj_server` |
| `SIM_START_SE3` | `true` | 是否启动 SE3 控制器 |
| `SIM_START_GOAL_BRIDGE` | `true` | 是否启动 RViz 2D 目标桥 |
| `SIM_START_RVIZ` | `true` | 是否启动 RViz |
| `SIM_SCENE` | `default` | `simulation/config/scenes` 中的场景名称 |
| `SIM_WORLD` | 场景值 | 临时覆盖容器内 Gazebo world 路径 |
| `SIM_SPAWN_X/Y/Z/YAW` | 场景值 | 临时覆盖车辆出生位姿 |
| `SIM_RVIZ_GOAL_Z` | `1.0 m` | RViz 2D Nav Goal 被转换成 3D 目标时补充的高度 |
| `SIM_TAKEOFF_HEIGHT` | `1.0 m` | 期望实际 Home 相对起飞高度 |
| `SIM_TAKEOFF_TIMEOUT` | `30 s` | 原生起飞高度监测超时 |
| `SIM_TAKEOFF_TOLERANCE` | `0.1 m` | 脚本对实际相对高度的完成判定容差，不写入 PX4 |
| `SIM_COMMAND_TIMEOUT` | `15 s` | PX4 解锁、AUTO.TAKEOFF 和 OFFBOARD 请求超时 |
| `SIM_PREFLIGHT_TIMEOUT` | `5.0 s` | `arm/goal/mission` 并行等待状态、定位和连续 setpoint 的最长时间 |
| `SIM_REQUIRE_ARMED_GOAL` | `true` | CLI 目标是否要求车辆 armed + OFFBOARD；`false` 仅用于 Planner-only 测试 |
| `SIM_PLANNER_CONFIG` | 空 | 可选 Planner 完整 YAML 覆盖，使用容器内路径 |
| `SIM_CONTROLLER_CONFIG` | `/etc/sim2real/simulation/controller.yaml` | 仿真车辆控制标定文件 |
| `SIM_RVIZ_CONFIG` | `/etc/sim2real/simulation/rviz/sim.rviz` | RViz 配置文件 |
| `SIM_GPU_MODE` | `auto` | 容器图形设备：`auto/nvidia/dri/none` |

临时覆盖示例：

```bash
SIM_TAKEOFF_HEIGHT=1.2 \
SIM_GAZEBO_GUI=false \
SIM_START_RVIZ=false \
./launch/sim.sh restart
```

`SIM_TAKEOFF_HEIGHT` 是 PX4 原生起飞目标。不同 Gazebo world 的 Home 与本地原点
可能存在不同偏移，因此仿真脚本会在 `/mavros/altitude/local` 和 `relative` 中自动选择
最接近 PX4 起飞目标的一项；真机流程仍固定使用 Home 相对高度。原生起飞不经过
Planner，因此不使用 Planner 的
`virtual_ground/virtual_ceil` 检查；后续 `goal` 仍受 Planner 高度范围限制。

## 3. Gazebo 世界参数

默认场景的 world 是 `ego_swarm.world`，使用 ODE 物理引擎。使用
`./launch/sim.sh --scene NAME restart` 可只替换 world 和出生位姿，场景文件位于
`simulation/config/scenes/*.env`。

| 参数 | 当前值 |
|---|---:|
| 重力 | `(0, 0, -9.8) m/s^2` |
| `max_step_size` | `0.004 s` |
| `real_time_update_rate` | `250 Hz` |
| 目标 `real_time_factor` | `1.0` |
| 地面尺寸 | `100 x 100 m` |
| 风场 | 未配置主动风 |

仿真实际是否能达到实时倍率还取决于 CPU、GPU、雷达射线计算和 RViz 订阅负载。

## 4. Iris 物理模型

`outdoor_mid360.launch` 使用：

| 参数 | 当前值 |
|---|---:|
| Gazebo 模型名 | `iris` |
| SDF | `iris_mid360/iris_mid360.sdf` |
| 初始位置 | `(2.0, 0, 0) m` |
| 初始 RPY | `(0, 0, 0) rad` |

`outdoor_mid360.launch` 中虽然声明了 `est=ekf2`，但当前没有把 `est` 传给 `posix_sitl.launch`，因此修改这个 arg 不会生效。实际 estimator 由 PX4 airframe 和启动默认值决定。

`iris_mid360.sdf` 并未重新定义四旋翼动力学，而是组合两个嵌套模型：

1. PX4 v1.14.3 镜像中的标准 `model://iris`；
2. 仓库控制的 `model://Mid360`。

标准 Iris 基础模型位于容器内：

```text
/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/
  sitl_gazebo-classic/models/iris/iris.sdf
```

当前固定版本下的关键物理值如下。

| 参数 | 当前值 |
|---|---:|
| Iris 各刚体质量合计（含 GPS） | 约 `1.550 kg` |
| Iris 内嵌 GPS 质量 | `0.015 kg` |
| MID-360 质量 | `0.2 kg` |
| 组合模型质量 | 约 `1.750 kg` |
| Iris `base_link` 惯量 `Ixx/Iyy/Izz` | `0.029125 / 0.029125 / 0.055225 kg*m^2` |
| `base_link` 碰撞盒 | `0.47 x 0.47 x 0.11 m` |
| 单个转子质量 | `0.005 kg` |
| 转子半径 | `0.128 m` |
| 最大转速参数 | `1100 rad/s` |
| 电机上升/下降时间常数 | `0.0125 / 0.025 s` |
| `motorConstant` | `5.84e-6` |
| `momentConstant` | `0.06` |
| `rotorVelocitySlowdownSim` | `10` |

Iris 基础 SDF 来自 [`simulation/versions.env`](../simulation/versions.env) 固定的 PX4 v1.14.3。不要直接修改运行中容器内的 `iris.sdf`；需要长期修改质量、电机或惯量时，应先把基础模型纳入 `simulation/assets/`，再由 Dockerfile 复制并重建镜像。

## 5. MID-360 参数

### 5.1 安装外参

MID-360 相对 `iris::base_link` 的固定变换：

| 参数 | 当前值 |
|---|---:|
| 平移 `(x,y,z)` | `(0.07, 0, 0.072) m` |
| RPY | `(0, 0.3925, 0) rad` |
| 俯仰角 | 约 `22.5 deg` |
| 雷达内部射线原点 | `livox_link` 上方 `0.05 m` |

同一外参也由 [`localization.launch`](../simulation/ros_pkgs/sim2real_simulation/launch/localization.launch) 发布为 `base_link -> livox_link` TF。修改 SDF 外参时必须同步修改该 TF，否则点云外观和碰撞模型会不一致。

### 5.2 扫描参数

| 参数 | 当前值 |
|---|---:|
| ROS 原始话题 | `/livox/lidar` |
| frame | `livox_link` |
| 发布频率 | `10 Hz` |
| 插件采样数 | `18000` |
| `downsample` | `1` |
| 最小/最大距离 | `0.1 / 45 m` |
| 距离分辨率 | `0.02 m` |
| 噪声均值/标准差 | `0 / 0` |
| 扫描模式文件 | `mid360-real-centr.csv` |
| 自身过滤 | 开启 |
| Gazebo 射线绘制 | 关闭 |

关闭 `<visualize>` 只是不让 Gazebo GUI 绘制大量射线，不会减少 `/livox/lidar` 的采样数。

## 6. PX4 SITL 与 MAVROS

### 6.1 PX4 airframe

`vehicle=iris` 会让 PX4 设置：

```text
PX4_SIM_MODEL=gazebo-classic_iris
```

因此 PX4 加载镜像内的 `10015_gazebo-classic_iris` airframe，并继承 `rc.mc_defaults`。该 airframe 设置四旋翼控制分配、四个电机输出函数和转子力矩方向；其余 EKF、姿态环、角速度环和 failsafe 参数来自固定版本 PX4 的默认参数。

当前 PX4 控制分配值：

| 转子 | `CA_ROTOR*_PX` | `CA_ROTOR*_PY` | `CA_ROTOR*_KM` | 输出函数 |
|---|---:|---:|---:|---:|
| 0 | `0.1515` | `0.2450` | `0.05` | `101` |
| 1 | `-0.1515` | `-0.1875` | `0.05` | `102` |
| 2 | `0.1515` | `-0.2450` | `-0.05` | `103` |
| 3 | `-0.1515` | `0.1875` | `-0.05` | `104` |

这里是 PX4 control allocator 的几何参数，不是 Gazebo SDF 中用于显示和关节仿真的转子坐标。

仓库当前没有额外的 PX4 参数覆盖文件。通过 QGC 或 `/mavros/param/set` 修改的是本次 SITL 运行参数，文件位于 `runtime/simulation/runs/<run-id>/ros_home/`；不要把它视为可复现的仓库配置。需要永久修改时，应增加仓库控制的 PX4 airframe/参数加载步骤并重建镜像。

SE3 最终向 PX4 发送姿态四元数和归一化推力，所以 PX4 的多旋翼姿态环、角速度环、控制分配和 failsafe 仍然生效；PX4 的位置控制参数不是这条 OFFBOARD 控制链路的主要调参入口。

### 6.2 MAVROS

| 参数 | 当前值 |
|---|---:|
| FCU URL | `udp://:14540@localhost:14557` |
| MAVLink 协议 | `v2.0` |
| target system/component | `1 / 1` |
| GCS URL | 空 |
| `/mavros/setpoint_raw/attitude` thrust scaling | `1.0` |
| 位置/速度 setpoint MAV frame | `LOCAL_NED` |
| MAVROS local-position TF | 关闭，由仿真定位适配器统一发布 |

[`px4_config.yaml`](../simulation/assets/px4/launch/px4_config.yaml) 是 MAVROS 插件配置，不是 PX4 飞控参数文件。

## 7. 仿真定位适配参数

仿真不运行 FAST-LIO。适配层将 PX4/MAVROS 里程计和模拟点云转换为公共接口：

| 输入 | 输出 | 关键设置 |
|---|---|---|
| `/mavros/local_position/odom` | `/localization/odom` | `frame_id=world`，`child_frame_id=base_link`，发布 TF |
| `/livox/lidar` | `/localization/cloud_registered` | 转换到 `world`，`filter_enable=false`，不做体素、距离或点数过滤 |

Planner 只订阅这两个公共接口，不直接读取 Gazebo 或 MAVROS 原始点云。

`localization.launch` 同时设置了 `max_points=80000`，但 `pointcloud_to_world.py` 在 `filter_enable=false` 时会在过滤逻辑前直接发布，所以该上限当前不生效。

## 8. Diff-Planner 参数

仿真和真机默认加载同一个 [`planner.yaml`](../common/config/planner.yaml)。

### 8.1 FSM 与目标

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `flight_type` | `1` | 使用外部目标 |
| `thresh_replan_time` | `1.0 s` | 重规划时间阈值 |
| `planning_horizon` | `3.0 m` | 局部目标/规划视距 |
| `emergency_time` | `1.0 s` | 紧急时间参数 |
| `goal_position_tolerance` | `0.2 m` | 终点位置容差 |
| `goal_yaw_tolerance_deg` | `5 deg` | 终点 yaw 容差 |
| `max_goal_distance` | `200 m` | 单次目标相对当前里程计位置的最大三维直线距离；仅用于拒绝异常目标，不代表全局路径能力 |
| `realworld_experiment` | `true` | 等待外部目标，不自动执行 YAML waypoint |
| `fail_safe` | `true` | 开启规划安全逻辑 |
| `mondify_final_goal` | `true` | 允许调整碰撞中的终点 |
| `enable_stuck_detect` | `true` | 开启卡死检测 |

### 8.2 局部占据地图

| 参数 | 当前值 | 实际效果 |
|---|---:|---|
| `resolution` | `0.11 m` | 体素边长；膨胀半径正好量化为 3 个体素 |
| `local_update_range_x/y` | `5.5 / 5.5 m` | ring-buffer 半范围；无需额外量化 |
| `local_update_range_z` | `1.8 m` | 竖直半范围；量化后为 `1.875 m` |
| 原始地图窗口 | - | 约 `11.0 x 11.0 x 3.75 m` |
| `obstacles_inflation` | `0.33 m` | XYZ 统一膨胀 3 个体素；相对 `0.325 m` 半对角线只留 `0.005 m` 理论余量 |
| `virtual_ground/virtual_ceil` | `0.1 / 3.0 m` | Planner 硬高度边界 |
| `p_hit/p_miss` | `0.65 / 0.35` | 命中/穿过概率 |
| `p_min/p_max/p_occ` | `0.12 / 0.90 / 0.80` | log-odds 范围与占据阈值 |
| `fading_time` | `-1.0` | 关闭时间衰减 |
| `visualize_all_directions` | `true` | 发布完整 360 deg 原始/膨胀地图 |
| `visualization_period` | `0.5 s` | 原始/膨胀地图仅以 2 Hz 输出给 RViz 或 rosbag，不影响内部地图更新 |
| `frame_id` | `world` | 地图坐标系 |

当前使用注册点云建图，不使用深度图。因此 `cx/cy/fx/fy`、深度过滤参数和 `skip_pixel` 对默认仿真点云链路没有作用。另外，当前编译的 ring-buffer `grid_map.cpp` 没有读取 YAML 中的 `ground_height`、`max_ray_length`、`depth_filter_maxdist` 和 `visualization_truncate_height`；`min_ray_length` 虽然被读取，但当前 cloud raycast 没有使用它。修改这些兼容参数不会改变当前仿真行为。

### 8.3 运动和优化约束

| 参数 | 当前值 |
|---|---:|
| `manager.max_vel` / `optimization.max_vel` | `0.5 m/s` |
| `manager.max_acc` / `optimization.max_acc` | `0.8 m/s^2` |
| `manager.max_jer` / `optimization.max_jer` | `8.0 m/s^3` |
| `polyTraj_piece_length` | `1.5 m` |
| `feasibility_tolerance` | `0.05` |
| `constraint_points_perPiece` | `5` |
| `weight_obstacle` | `10000` |
| `weight_obstacle_soft` | `5000` |
| `weight_swarm` | `10000` |
| `weight_feasibility` | `10000` |
| `weight_sqrvariance` | `10000` |
| `weight_time` | `10` |
| `obstacle_clearance` | `0.1 m` |
| `obstacle_clearance_soft` | `0.5 m` |
| `swarm_clearance` | `0.15 m` |
| `vel_tolerance / acc_tolerance` | `1.0 / 1.0` |
| `record_opt` | `true` |
| `use_multitopology_trajs` | `false` |

这些值限制 Planner 生成的轨迹，不会改变 Gazebo Iris 的质量、电机或 PX4 内环参数。

## 9. `traj_server` 参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `time_forward` | `1.0 s` | 轨迹预瞄时间 |
| `yaw_dot_max_deg_s` | `35 deg/s` | yaw 角速度上限 |
| `yaw_dot_dot_max_deg_s2` | `90 deg/s^2` | yaw 角加速度上限 |
| `goal_yaw_switch_distance` | `0.5 m` | 距终点多近时开始对准最终 yaw |

`./launch/sim.sh goal X Y Z` 省略 yaw 时会发布零四元数作为“不限定 yaw”标记。Planner 会把轨迹的 `has_goal_yaw` 设为 `false`，因此 `goal_yaw_switch_distance` 不会触发终点对准；`traj_server` 在飞行中使用路径方向 yaw，到达后保持最后的路径方向。只有显式提供 `YAW_DEG` 时才限定终点朝向。

## 10. SE3 控制器参数

### 10.1 YAML 覆盖顺序

[`controller.launch`](../common/launch/controller.launch) 先加载公共配置，再加载仿真车辆配置，因此仿真文件中的同名参数优先。

公共安全和前馈参数：

| 参数 | 当前值 |
|---|---:|
| `auto_request_offboard` | `false` |
| `auto_request_arm` | `false` |
| `auto_land_on_geofence` | `false` |
| `enable_thrust_estimation` | `false` |
| `use_acceleration_feedforward` | `true` |
| `use_yaw_rate_feedforward` | `true` |
| `max_feedforward_acc` | `1.2 m/s^2` |
| `odom_timeout` | `0.2 s` |

仿真车辆标定：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `enable_sim` | `true` | 与 `auto_request_offboard/arm` 联合控制自动切模式和解锁；后两项当前均关闭 |
| `hover_percent` | `0.755` | 归一化悬停推力标定 |
| `max_hover_percent` | `0.85` | 仅在 `enable_thrust_estimation=true` 时约束在线估计的推力模型；当前不生效 |
| `min_output_thrust` | `0.20` | 归一化输出下限 |
| `max_output_thrust` | `0.95` | 归一化输出上限 |
| `geo_fence.x/y/z` | `50 / 50 / 6 m` | x/y 检查 `+/-50 m`，z 只检查 `z >= 6 m`，不检查负高度下界 |
| `ki_pz` | `0.0` | 竖直积分关闭 |
| `int_limit_z` | `5.0` | 竖直积分限幅 |

`sim.sh arm` 先保存 `NAV_MC_ALT_RAD`，起飞期间临时将其收紧到 `SIM_TAKEOFF_TOLERANCE`（默认 `0.1 m`），并把 `MIS_TAKEOFF_ALT` 直接设置为 `SIM_TAKEOFF_HEIGHT`。这样 SITL 不会因默认 `0.8 m` 接受半径过早结束爬升，真机也不会因把接受半径叠加到目标而爬到 `1.8 m`。脚本确认高度处于目标容差内，且定位垂直速度连续 `SIM_TAKEOFF_STABLE_TIME`（默认 `0.5 s`）不超过 `SIM_TAKEOFF_MAX_VERTICAL_SPEED`（默认 `0.2 m/s`）后，恢复原接受半径，再从 `AUTO.TAKEOFF/AUTO.LOITER` 切换 OFFBOARD。异常退出时也会尝试恢复参数。

### 10.2 SE3 增益

控制器启动时由 `se3_ctrl.cpp` 写入以下基础增益。表中的“当前控制作用”专指目前发布给 PX4 的姿态四元数和归一化推力通道：

| 增益 | 当前值 `(x,y,z)` | 当前控制作用 |
|---|---|---|
| `Kp_p` | `(0.85, 0.85, 1.5)` | 位置误差修正期望速度，会影响姿态和推力 |
| `Kd_p` | `(0.1, 0.1, 0.0)` | 位置误差差分修正期望速度，会影响姿态和推力 |
| `Kp_v` | `(1.5, 1.5, 1.5)` | 速度误差修正期望加速度，会影响姿态和推力 |
| `Kd_v` | `(0, 0, 0)` | 与 `Kp_v` 同一通道，当前增益为零 |
| `Ki_pz` | `(0, 0, 0)` | 位置积分修正期望加速度，当前 `ki_pz=0`，未启用 |
| `Kp_a` | `(1.5, 1.5, 1.5)` | 加速度误差修正 jerk，主要影响计算出的 body-rate；当前 body-rate 被屏蔽 |
| `Kd_a` | `(0, 0, 0)` | 与 `Kp_a` 同一通道，当前增益为零且 body-rate 被屏蔽 |
| `Kp_q` | `(5.5, 5.5, 0.1)` | 只修正计算出的 body-rate；当前 body-rate 被屏蔽 |
| `Kp_w` | `(1.5, 1.5, 0.1)` | 对应控制代码路径已注释，不参与当前输出 |
| `Kd_q`、`Kd_w` | `(0, 0, 0)` | 对应控制代码路径已注释，不参与当前输出 |

误差限幅：

| 参数 | 当前值 |
|---|---:|
| `limit_err_p/v/a` | `3.0 / 2.0 / 1.0` |
| `limit_d_err_p/v/a` | `3.5 / 1.0 / 1.0` |

这些增益不在车辆 YAML 中。控制器最终发布 `/mavros/setpoint_raw/attitude`；当前 `type_mask` 忽略 roll、pitch、yaw 三轴 body-rate，PX4 实际执行姿态四元数和归一化推力。因此调整只作用于 body-rate 的增益，在当前模式下不会改变飞行表现。

## 11. 参数修改如何生效

### 11.1 修改 Planner 或控制 YAML

修改以下挂载文件后重启栈即可：

```text
common/config/planner.yaml
common/config/trajectory_server.yaml
common/config/controller.yaml
simulation/config/controller.yaml
```

```bash
./launch/sim.sh restart
```

### 11.2 修改 world、SDF、PX4 launch 或 MAVROS 配置

`simulation/assets/` 会在构建时复制进仿真镜像，必须重建并重新创建容器：

```bash
./launch/sim.sh stop
./launch/sim_container.sh build
./launch/sim_container.sh recreate
./launch/sim.sh start
```

初始位姿由场景配置中的 `SCENE_SPAWN_X/Y/Z/ROLL/PITCH/YAW` 保存；运行时也可通过
`SIM_SPAWN_X/Y/Z/ROLL/PITCH/YAW` 临时覆盖，不需要修改
`outdoor_mid360.launch` 或重建镜像。需要长期保存新起点时，应新增或修改
`simulation/config/scenes/*.env`。

### 11.3 修改 SE3 增益

当前应优先修改 [`se3_ctrl.cpp`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp) 中的基础增益，再执行 `./launch/sim.sh restart` 重新构建变更并启动。

虽然节点提供 dynamic_reconfigure，但 [`tune.cfg`](../third_party/Diff-Planner-PX4/src/se3_controller/cfg/tune.cfg) 的默认值与实际源码启动值不完全一致：例如 `kp_px/py` 是 `1.5` 而不是 `0.85`，`kd_px/py` 是 `0` 而不是 `0.1`，多项误差限幅也是 `1`。dynamic_reconfigure 每次提交的是完整配置，因此只执行一次单参数 `dynparam set`，也会把其他内部增益一并改成 `tune.cfg` 的值。

同理，首次在线修改前执行 `dynparam get` 得到的是 dynamic_reconfigure 配置，不代表控制器内部正在使用的源码启动值。在统一 `tune.cfg` 与源码默认值之前，不要用单参数在线命令调节 SE3。硬编码的 `kp_*`、`kd_*` 和误差限幅目前不能通过车辆 YAML 持久化；`ki_pz` 和 `int_limit_z` 则已经由车辆 YAML 配置。

## 12. 运行时检查

启动仿真后进入容器：

```bash
./launch/sim.sh shell
```

检查 ROS 参数：

```bash
rosparam get /drone_0_diff_planner_node
rosparam get /drone_0_traj_server
rosparam get /se3_controller_node
```

其中 `rosparam get` 可检查 YAML 加载结果，但不能显示 `se3_ctrl.cpp` 中硬编码的基础增益。

检查 PX4 参数：

```bash
rosservice call /mavros/param/get "param_id: 'SYS_AUTOSTART'"
rosservice call /mavros/param/get "param_id: 'MC_ROLL_P'"
rosservice call /mavros/param/get "param_id: 'MC_PITCH_P'"
```

检查模型和状态：

```bash
rosservice call /gazebo/get_model_state "model_name: 'iris'"
rostopic echo -n1 /mavros/state
rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic hz /drone_0_diff_planner_node/grid_map/occupancy_inflate
```

## 13. 安全边界

- `sim.sh start` 只启动栈，车辆保持 disarmed。
- 必须显式执行 `sim.sh arm` 才会解锁、完成 PX4 原生起飞并自动交接到 OFFBOARD。
- RViz 在 armed OFFBOARD 前发送的目标会被丢弃；`arm` 完成后需要重新点击。
- 默认目标高度必须满足 `0.1 < z < 1.5 m`。
- Planner 最大速度/加速度不是 Gazebo 动力学上限，也不是 PX4 内环上限。
- SE3 `geo_fence`、Planner 虚拟墙和 PX4 failsafe 是三套不同层级的约束，不能相互替代。

更深入的算法和控制说明见 [Diff-Planner 原理](diff_planner_principles.md) 与 [SE3 控制器](se3_controller.md)。
