# 公共自主飞行接口

`common/` 提供仿真与真机共用的飞行命令、Mission、定位保护和 SE3 入口。
规划器插件由 [`planning/`](../planning/README.md) 管理；传感器与定位适配分别位于
`simulation/` 和 `deployment/`。

## 定位契约

规划器 adapter 的公共传感器边界只有以下两个 topic；各算法如何建图由插件自行负责。

| Topic | 类型 | 约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | 非零测量时间戳；`frame_id=world`；`child_frame_id=base_link`；pose 在 `world`，twist 在 `base_link` |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 非零采集时间戳；点坐标和 `frame_id` 均为 `world`；包含有效 XYZ 字段 |

仿真适配 MAVROS 与模拟 MID-360；真机适配 FAST-LIO。需要改变坐标系时必须使用采集
时刻的 TF 变换数值，不能只改 `frame_id`。Diff 保留 rolling GridMap，SUPER 保留
rolling ROGMap，Fast Kino/Topo 保留固定范围 SDFMap。

## 数据流

```text
/localization/odom ────────────────┬─→ selected adapter ↔ native planner
                                   └─→ planner_gateway（轨迹交接或取消期间生成实测位置 HOLD）
/localization/cloud_registered ───────→ selected adapter
/mavros/state ────────────────────────→ planner_gateway
/goal ─→ planner_gateway ─→ /planning/backends/<ns>/goal ─→ selected adapter
selected adapter ── status / command / capabilities ───────→ planner_gateway
planner_gateway ──→ public status / command / capabilities
                └─→ /command/trajectory ─→ SE3 ─→ MAVROS ─→ PX4
```

`planner_gateway` 是 `/command/trajectory` 的唯一发布者。原生 `PolyTraj`、
`Bspline` 和 `PositionCommand` 只存在于插件私有命名空间。

## 公共规划接口

| 名称 | 类型 | 作用 |
|---|---|---|
| `/goal` | `geometry_msgs/PoseStamped` | 输入给网关的 `world` 目标；零四元数表示不约束终点 yaw |
| `/planning/goal` | `PlannerGoal` | 网关发布的 PLAN/CANCEL 记录，包含其分配的 session 和 goal ID |
| `/planning/status` | `PlannerStatus` | 后端状态、目标/轨迹 ID、readiness 和实际目标 |
| `/planning/capabilities` | `PlannerCapabilities` | 能力、动力学上限和可选固定地图边界 |
| `/planning/validate_goal` | `ValidateGoal` | 调用当前插件验证目标 |
| `/planning/cancel` | `std_srvs/Trigger` | 取消当前目标 |
| `/planning/command` | `PlannerCommand` | 网关已接受的插件命令观测流，不是控制器输入 |
| `/command/trajectory` | `MultiDOFJointTrajectory` | SE3 控制输入 |
| `/planning/viz/occupancy` | `PointCloud2` | 当前插件的统一占据显示 |
| `/planning/viz/inflated_occupancy` | `PointCloud2` | 当前插件的统一膨胀显示 |
| `/planning/viz/planning_bounds` | `Marker` | 插件声明的固定地图边界；无固定边界时删除 |

## 安全语义

- 插件只能在整栈启动时选择，不支持空中切换或自动 fallback。
- 新目标会关闭旧命令授权。新的运动轨迹只有在 backend、session、goal 和 trajectory
  ID 全部匹配，并满足 `ACTIVE + armable`、armed/OFFBOARD、输入 readiness、最低频率
  和新鲜度要求后才能重新打开命令门。
- 非 adapter 来源、错误或回退的 ID、非法数值以及过期消息会被拒绝。readiness 丢失、
  状态或命令流超时、频率不足、后端 `FAULT`、失去 connected/armed/OFFBOARD 会撤销
  当前目标授权；检测到第二个 `/command/trajectory` 发布者时网关直接退出。
- `localization_guard.py` 检查时间戳、停更、有限值和位置跳变。速度上限默认关闭；
  定位故障会锁存，并在自主飞行中请求 `AUTO.LAND`。
- 当前所有内置插件都声明目标验证能力，因此网关在接受每个 `/goal` 前都会调用当前
  插件的验证服务。单目标、RViz 和 Mission 入口还会在发布前调用公共验证服务；Mission
  起飞前逐点预检，但滚动地图暂时覆盖不到的 `goal_out_of_local_map` 会延后判断，每个
  航点在实际下发前仍会重新验证。

人工切出 OFFBOARD 后不会自动恢复旧目标。定位故障锁存后必须重启完整栈。

## 参数归属

| 文件 | 内容 |
|---|---|
| [`common/config/controller.yaml`](config/controller.yaml) | 公共 SE3、输入超时和命令来源 |
| [`simulation/config/controller.yaml`](../simulation/config/controller.yaml) | 仿真载体推力、积分和围栏 |
| [`deployment/config/controller.yaml`](../deployment/config/controller.yaml) | 真机推力、积分和围栏 |
| [`planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) | Diff 全部参数 |
| [`planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml) | Fast Kino/Topo 共用参数 |
| [`planning/ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml) | `forest` 仿真的 Fast 地图覆盖 |
| [`planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml) | SUPER、ROGMap 与生命周期参数 |
| [`planning/plugins/super/controller.yaml`](../planning/plugins/super/controller.yaml) | SUPER 专用控制增益、姿态对齐与前馈加速度上限 |

`SIM_PLANNER_CONFIG` 和 `PLANNER_CONFIG` 可为当前选择的任一规划器临时指定插件主
配置，路径分别按仿真和真机容器内解释，文件格式由对应插件定义。Fast 在 `forest`
仿真中仍会在主配置之后叠加 `config/scenes/forest.yaml`。所有内置插件目前都只提供
manifest 中的 `local` profile。

## 修改边界

- 新传感器或定位源：在环境目录适配为两个公共 localization topic。
- 新规划器：使用独立 workspace 和 adapter，不修改 Mission、SE3 或公共消息语义。
- 规划器参数只放在对应插件配置；载体控制参数只放在对应环境配置。
- 原生规划器不得直接依赖 `/Odometry`、`/cloud_registered` 或 `/livox/lidar`。
