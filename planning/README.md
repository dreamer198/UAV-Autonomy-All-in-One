# 多规划器插件框架

规划器插件是独立 catkin workspace 中的 ROS 节点束，不是同进程 `pluginlib`。
这种结构隔离上游同名包、库和消息。

| ID | 私有命名空间 | 算法 |
|---|---|---|
| `diff` | `/planning/backends/diff` | Diff-Planner |
| `fast-kino` | `/planning/backends/fast_kino` | Kinodynamic A* + B-spline |
| `fast-topo` | `/planning/backends/fast_topo` | Topological PRM + B-spline |
| `super` | `/planning/backends/super` | SUPER + ROGMap |

## 选择规划器

`start` 和 `restart` 必须明确指定规划器：

```bash
./launch/sim.sh --scene room --planner diff start
./launch/sim.sh --scene room --planner fast-kino start
./launch/sim.sh --scene room --planner fast-topo start
./launch/sim.sh --scene room --planner super start

./launch/real.sh --planner diff start
./launch/real.sh --planner fast-kino start
./launch/real.sh --planner fast-topo start
./launch/real.sh --planner super start
```

只能停止完整栈后再选择其他插件。`SIM_PLANNER` 和 `REAL_PLANNER` 分别等价于命令行
的 `--planner`。内置插件都接入相同的真机启动入口；软件链路能够构建和启动不代表
已经完成真实飞行验证。

## 输入与地图

公共框架只共享以下输入，gateway 和 adapter 的职责不同：

| 名称 | 类型 | 使用者与约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | adapter 和 gateway；`world` pose、`base_link` twist、非零测量时间戳；gateway 仅用它在轨迹交接或取消期间生成实测位置 HOLD |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | adapter；点坐标和 `frame_id` 均为 `world` |
| `/goal` | `geometry_msgs/PoseStamped` | gateway；`world` 目标，零四元数表示不约束 yaw |
| `/mavros/state` | `mavros_msgs/State` | gateway；检查连接、armed、OFFBOARD 和消息新鲜度 |

框架不共享地图。Diff 使用随飞机移动的 GridMap，SUPER 使用随飞机移动的 ROGMap；
Fast Kino/Topo 使用固定范围 SDFMap。Fast 默认地图参数在
[`config/planner.yaml`](ros_pkgs/sim2real_fast_adapter/config/planner.yaml)，
`forest` 仿真自动叠加
[`config/scenes/forest.yaml`](ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml)。
固定地图边界通过 `PlannerCapabilities` 公布并由目标验证服务执行。
SUPER 只提供 `local` profile，默认规划上限为 `2.4 m/s`、`3.0 m/s²`，并通过
`PlannerCapabilities` 上报。其 `click_height=1.5 m` 会改写请求目标的高度，
实际生效目标通过 `PlannerStatus.active_goal` 返回。ROGMap 和生命周期参数位于
[`config/planner.yaml`](ros_pkgs/sim2real_super_adapter/config/planner.yaml)。

## 规划器调参

先按 [SE3 控制器调参顺序](../docs/se3_controller.md#调参顺序)完成悬停推力、跟踪增益
和前馈上限标定，再调整规划器。规划器调参不能补偿定位跳变、推力饱和或控制跟踪误差。

统一按以下顺序调整：

1. 根据机体尺寸、定位误差和场景确定地图范围、分辨率、障碍膨胀及高度边界；
2. 从保守速度、加速度开始，并同步同一配置中重复的动力学上限；
3. 调整搜索范围、重规划频率和计算预算；
4. 最后调整轨迹平滑、避障和时间代价，每次只改一组并用同一 Mission 与 rosbag 对比。

仿真修改后使用 `./launch/sim.sh --scene SCENE --planner ID restart` 重载。也可用
`SIM_PLANNER_CONFIG=容器内路径` 为任一规划器临时加载插件主配置，文件必须存在于仿真
规划器容器中。Fast 的 `forest` 地图几何仍会在主配置之后叠加。真机配置位于镜像内；
可按 [参数设置](../docs/deployment.md#参数设置)停止飞行栈并重建镜像，或通过
`PLANNER_CONFIG=/root/tmp/FILE.yaml` 临时加载对应插件格式的主配置。

### Diff

配置文件为
[`sim2real_diff_adapter/config/planner.yaml`](ros_pkgs/sim2real_diff_adapter/config/planner.yaml)。

- 地图与安全距离：先调整 `planner/grid_map/resolution` 和
  `planner/grid_map/obstacles_inflation`。插件模式下
  `planner/grid_map/enable_virtual_wall` 应保持 `true`；仅关闭该项不会关闭 adapter
  的目标高度验证。`planner/grid_map/obstacles_inflation`、
  `planner/grid_map/virtual_ground`、`planner/grid_map/virtual_ceil` 分别与
  `adapter/backend/obstacle_inflation`、`adapter/backend/virtual_ground`、
  `adapter/backend/virtual_ceil` 保持一致。
- 动力学：同步修改 `planner/manager/max_vel`、`planner/manager/max_acc`、
  `planner/optimization/max_vel`、`planner/optimization/max_acc`、
  `adapter/backend/max_velocity` 和 `adapter/backend/max_acceleration`。
- 重规划：`planner/fsm/planning_horizon` 与 `planner/manager/planning_horizon`
  保持一致，再调整 `planner/fsm/thresh_replan_time` 和
  `planner/fsm/emergency_time`。
- 轨迹权衡：确认地图正确后，再调整 `planner/optimization/obstacle_clearance`、
  `planner/optimization/obstacle_clearance_soft`、
  `planner/optimization/weight_obstacle`、
  `planner/optimization/weight_obstacle_soft`、
  `planner/optimization/weight_feasibility` 和
  `planner/optimization/weight_time`。到达位置容差需同步
  `planner/fsm/goal_position_tolerance` 与 `adapter/backend/goal_position_tolerance`。

### Fast Kino

Fast Kino 与 Fast Topo 共用
[`sim2real_fast_adapter/config/planner.yaml`](ros_pkgs/sim2real_fast_adapter/config/planner.yaml)；
修改 `sdf_map`、`manager` 或 `optimization` 会同时影响两者。

- 地图：先确定 `sdf_map/origin_*`、`sdf_map/map_size_*`、
  `sdf_map/resolution`、`sdf_map/obstacles_inflation` 和
  `sdf_map/obstacles_inflation_z`。`forest` 仅用
  [`scenes/forest.yaml`](ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml)
  覆盖地图几何。
- 动力学：`manager/max_vel`、`manager/max_acc` 与
  `optimization/max_vel`、`optimization/max_acc` 对两个 Fast 后端都应一致；Kino
  还要同步 `search/max_vel` 和 `search/max_acc`，Topo 不读取 `search`。
- Kinodynamic A*：用 `search/horizon` 控制局部搜索距离；搜索过慢时检查
  `search/resolution_astar`、`search/lambda_heu` 和 `search/allocate_num`，需要更密
  碰撞检查时再提高 `search/check_num`。
- B-spline：`optimization/dist0` 决定障碍代价作用距离，
  `optimization/lambda1`、`optimization/lambda2` 和 `optimization/lambda3`
  分别侧重平滑、避障和动力学可行性。

### Fast Topo

地图、动力学和 B-spline 参数与 Fast Kino 共用；Topo 特有的前端采样参数集中在同一
文件的 `topo_prm`：

- `topo_prm/clearance` 决定 PRM 节点和边的最小净空；
- `topo_prm/sample_inflate_*` 决定绕行采样范围，`topo_prm/max_sample_time` 和
  `topo_prm/max_sample_num` 决定单轮计算预算；
- `topo_prm/reserve_num` 和 `topo_prm/ratio_to_short` 决定保留的候选数量与长度范围；
- `optimization/lambda5` 决定最终轨迹贴合拓扑引导路径的程度；
- 跟踪误差或制动后重规划使用 `fsm/max_tracking_error`、
  `fsm/emergency_stop_velocity` 和 `fsm/emergency_stop_settle_time`。

狭窄场景无解时先核对固定地图边界和点云，再增加采样预算；不要通过缩小机体净空来
掩盖地图或定位问题。

### SUPER

配置文件为
[`sim2real_super_adapter/config/planner.yaml`](ros_pkgs/sim2real_super_adapter/config/planner.yaml)。

- 地图与机体：联合调整 `rog_map/resolution`、`rog_map/inflation_resolution`、
  `rog_map/inflation_step`、`rog_map/map_size` 和 `super_planner/robot_r`。
- 动力学：同步修改 `traj_opt/boundary/max_vel`、
  `traj_opt/boundary/max_acc`、`adapter/max_velocity` 和
  `adapter/max_acceleration`，并确认控制器
  `max_feedforward_acc` 能覆盖规划加速度。
- 局部重规划：`super_planner/planning_horizon` 决定局部规划距离，
  `super_planner/receding_dis` 决定复用旧轨迹的范围，
  `super_planner/replan_forward_dt` 同时用于重规划前瞻和耗时上限，
  `fsm/replan_rate` 决定状态机触发频率。
- 走廊与优化：先调 `super_planner/corridor_bound_dis`、
  `super_planner/corridor_line_max_length`、
  `super_planner/safe_corridor_line_max_length` 和
  `super_planner/iris_iter_num`，再调
  `traj_opt/exp_traj/penna_*`；权重过大可能使优化更难收敛。
- 目标：`fsm/click_height>-5` 会覆盖请求目标高度；如需保留请求高度，将其设为不大于
  `-5`。到达距离需同步 `fsm/close_goal_threshold` 与
  `adapter/goal_position_tolerance`，允许的目标改写幅度由
  `adapter/max_effective_goal_shift` 限制。

SUPER 的规划器专用控制增益位于
[`planning/plugins/super/controller.yaml`](plugins/super/controller.yaml)，调参原则和
加载优先级见 [SE3 控制器](../docs/se3_controller.md#配置)。

## 公共 API

`sim2real_planning_msgs` 定义：

| 消息或服务 | 作用 |
|---|---|
| `PlannerGoal` | session、goal ID、PLAN/CANCEL、目标 pose 和 yaw 约束 |
| `PlannerStatus` | STARTING/READY/PLANNING/ACTIVE/HOLDING/REACHED/FAULT、readiness 与错误原因 |
| `PlannerCommand` | trajectory ID、NORMAL/HOLD/BRAKE 和单个轨迹采样点 |
| `PlannerCapabilities` | 插件能力、动力学上限和可选固定地图 AABB |
| `ValidateGoal` | 由当前插件按自身地图和约束验证目标 |

稳定名称如下：

| 名称 | 发布者 → 使用者或含义 |
|---|---|
| `/goal` | 上层/RViz/Mission → gateway |
| `/planning/goal` | gateway → 观察者；已分配 ID 的 PLAN/CANCEL 记录 |
| `/planning/status` | gateway → 上层；已校验的生命周期状态 |
| `/planning/capabilities` | gateway → 上层；已与 manifest 核对的运行时能力 |
| `/planning/command` | gateway → 观察者；已通过门控的命令，不是 SE3 输入 |
| `/planning/validate_goal` | 上层 → gateway → adapter |
| `/planning/cancel` | 上层 → gateway → adapter |
| `/command/trajectory` | gateway → SE3 |
| `/planning/viz/occupancy` | visualization bridge → RViz |
| `/planning/viz/inflated_occupancy` | visualization bridge → RViz |
| `/planning/viz/planning_bounds` | visualization bridge → RViz；仅固定地图有边界 |

原生规划消息留在 `/planning/backends/<namespace>/native/`；算法调试可视化使用
`/planning/viz/backend/*`。未纳入统一显示的原生调试 topic 继续留在插件私有
namespace；公共 RViz 配置不依赖插件 ID。

## Workspace 隔离

生成目录为 `planning/workspaces/`：

| Workspace | 内容 | Underlay |
|---|---|---|
| `interfaces_ws` | 公共消息和服务 | ROS Noetic |
| `control_ws` | gateway、公共飞行逻辑、SE3 和环境适配 | `interfaces_ws` |
| `diff_ws` | Diff 上游包与 adapter | `interfaces_ws` |
| `fast_ws` | Fast 上游包与 adapter | `interfaces_ws` |
| `super_ws` | SUPER、ROGMap、私有消息与 adapter | `interfaces_ws` |

`control_ws`、`diff_ws`、`fast_ws` 和 `super_ws` 互不 overlay。仿真启动器增量构建这些
workspace；真机镜像在 Docker build 阶段构建它们。每个 backend 子进程只 source
当前插件的 `setup.bash`。Diff 与 SUPER 各自携带 wire schema 不同的同名
`quadrotor_msgs`；构建器会验证其包路径和消息 MD5 始终隔离。

SUPER 只固化运行所需的 `super_planner`、`rog_map` 和 `mars_quadrotor_msgs`，版本
固定为上游提交 `2ad3419c127a617c6d7df6925e81a14175a9c096`。构建时复制到可写的
`super_ws`，不会运行上游仿真器、控制器或任务节点。

## Manifest

内置 manifest 位于 `planning/plugins/<id>/planner.plugin.yaml`，包含：

- `api_version`、插件 ID、ROS namespace、算法 variant 和可信 adapter 节点名；
- workspace `setup.bash`、launch 及插件专用控制器覆盖文件；
- profile、启动/状态/命令超时和最低频率；
- simulation、yaw、cancel、目标验证和 RViz 能力。

未知字段、重复 ID、接口版本错误、非法名称、未声明的 launch 覆盖、缺失
workspace/launch/控制器配置或运行时能力与 manifest 不一致都会使启动失败。

仓库外插件可通过只读、冒号分隔的绝对路径加入：

```bash
SIM2REAL_PLANNER_PLUGIN_PATH=/abs/plugin-a:/abs/plugin-b \
./launch/sim_container.sh recreate
```

外部 ID 不能覆盖内置插件。其 workspace 路径相对 manifest 所在目录解析，并以只读
方式挂入容器。

## 添加规划器

1. 建立独立 workspace，固定上游版本并保留许可证。
2. 编写 adapter：订阅公共 odom/cloud 和私有 `goal`，在 backend namespace 下发布
   `status`、`command`、`capabilities`，并提供 `validate_goal` 服务；adapter 节点名
   必须与 manifest 一致，gateway 只信任该 ROS caller ID。
3. launch 至少接受 `backend_id`、`profile`、`scene`、`odom_topic` 和
   `cloud_topic`，并在 manifest 中声明 `scene`、`odom_topic`、`cloud_topic`，否则
   manager 的公共覆盖会被拒绝。`runtime_mode` 默认读取
   `SIM2REAL_RUNTIME_MODE`；`config` 默认读取 `SIM2REAL_PLANNER_CONFIG`，为空时使用
   插件自己的默认配置；需要独立 ROS namespace 时再接受 `backend_namespace`。
4. 添加 `planning/plugins/<id>/planner.plugin.yaml`，声明 launch 参数、控制器覆盖、
   超时、最低频率和能力；运行时 capabilities 必须与之相符。
5. 如需新构建域，扩展 `build_planner_workspaces.sh` 和两个镜像。
6. 覆盖 manifest、输入校验、目标关联、命令校验和故障测试。

新增插件不应修改 Mission、SE3 或 gateway 的算法分支。

## 安全约束

- gateway 只接受当前 backend/session/goal/trajectory 的新鲜有限命令，并检查
  `/command/trajectory` 没有第二个发布者。
- 新目标、失去 connected/armed/OFFBOARD、readiness 丢失、状态或命令超时、取消和后端
  `FAULT` 都会撤销旧命令。
- Fast 的目标验证会检查固定地图边界、已观测障碍净空和虚拟地面；轨迹采样若穿过虚拟
  地面会触发 `FAULT`。Diff、Fast 和 SUPER 都使用实测里程计判定 `REACHED`。
  SUPER 在取消或抢占后等待实测速度稳定，再从静止状态重新规划。
- 当前框架不提供地图共享、动态障碍预测接口、空中切换、自动 fallback 或多规划器
  并行控制。
