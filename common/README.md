# 公共自主飞行接口

`common/` 提供仿真与真机共用的飞行命令、Mission、定位保护和 SE3 入口。
规划器插件由 [`planning/`](../planning/README.md) 管理；传感器与定位适配分别位于
`simulation/` 和 `deployment/`。

## 定位契约

所有规划器只依赖以下输入，内部地图不受公共框架约束。

| Topic | 类型 | 约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | 非零测量时间戳；`frame_id=world`；`child_frame_id=base_link`；pose 在 `world`，twist 在 `base_link` |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 非零采集时间戳；点坐标和 `frame_id` 均为 `world`；包含有效 XYZ 字段 |

仿真适配 MAVROS 与模拟 MID-360；真机适配 FAST-LIO。需要改变坐标系时必须使用采集
时刻的 TF 变换数值，不能只改 `frame_id`。Diff 保留 rolling GridMap，Fast
Kino/Topo 保留固定范围 SDFMap。

## 数据流

```text
/localization/odom ────────────────→ selected planner adapter
/localization/cloud_registered ────→ selected planner adapter
/goal ─────────────────────────────→ planner_gateway ─→ selected planner adapter
                                           ▲                    │
                                           └─ status / command ─┘
                                           │
                                           └─→ /command/trajectory
                                                           │
                                                           ▼
                                                   SE3 → MAVROS → PX4
```

`planner_gateway` 是 `/command/trajectory` 的唯一发布者。原生 `PolyTraj`、
`Bspline` 和 `PositionCommand` 只存在于插件私有命名空间。

## 公共规划接口

| 名称 | 类型 | 作用 |
|---|---|---|
| `/goal` | `geometry_msgs/PoseStamped` | `world` 目标；零四元数表示不约束终点 yaw |
| `/planning/goal` | `PlannerGoal` | 网关分配 session 和 goal ID 后的目标 |
| `/planning/status` | `PlannerStatus` | 后端状态、目标/轨迹 ID、readiness 和实际目标 |
| `/planning/capabilities` | `PlannerCapabilities` | 能力、动力学上限和可选固定地图边界 |
| `/planning/validate_goal` | `ValidateGoal` | 调用当前插件验证目标 |
| `/planning/cancel` | `std_srvs/Trigger` | 取消当前目标 |
| `/planning/command` | `PlannerCommand` | 网关接受的插件命令观测流 |
| `/command/trajectory` | `MultiDOFJointTrajectory` | SE3 控制输入 |
| `/planning/viz/occupancy` | `PointCloud2` | 当前插件的统一占据显示 |
| `/planning/viz/inflated_occupancy` | `PointCloud2` | 当前插件的统一膨胀显示 |
| `/planning/viz/planning_bounds` | `Marker` | 插件声明的固定地图边界；无固定边界时删除 |

## 安全语义

- 插件只能在整栈启动时选择，不支持空中切换或自动 fallback。
- 新目标会撤销旧命令授权。运动命令必须匹配当前 backend、session、goal 和
  trajectory ID，并满足 `ACTIVE + armable`、armed/OFFBOARD、readiness、频率和
  超时检查。
- 错误来源、旧 ID、非法数值、状态/命令超时、后端故障、离开 OFFBOARD 或第二个
  `/command/trajectory` 发布者都会关闭命令门。
- `localization_guard.py` 检查时间戳、停更、有限值和位置跳变。速度上限默认关闭；
  定位故障会锁存，并在自主飞行中请求 `AUTO.LAND`。
- `goal` 和 Mission 在发布前调用当前插件的目标验证服务；Mission 在起飞前校验全部
  航点。

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
| `planning/plugins/*/planner.plugin.yaml` | 插件身份、workspace、launch、超时、频率与能力 |

`SIM_PLANNER_CONFIG` 和 `PLANNER_CONFIG` 只覆盖 Diff 配置。

## 修改边界

- 新传感器或定位源：在环境目录适配为两个公共 localization topic。
- 新规划器：使用独立 workspace 和 adapter，不修改 Mission、SE3 或公共消息语义。
- 规划器参数只放在对应插件配置；载体控制参数只放在对应环境配置。
- 原生规划器不得直接依赖 `/Odometry`、`/cloud_registered` 或 `/livox/lidar`。
