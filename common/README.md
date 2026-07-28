# 公共自主飞行链路

`common/` 保存仿真与真机共用的飞行命令、Mission、安全保护、SE3 控制入口和
兼容 launch。规划器的发现、隔离启动、消息适配与命令门控位于
[`planning/`](../planning/README.md)；环境专用的传感器和定位适配分别位于
`simulation/` 与 `deployment/`。

## 定位输入契约

所有规划器只依赖相同的两个输入，但可以使用完全不同的内部地图：

| Topic | 类型 | 约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | 非零测量时间戳；`header.frame_id=world`；`child_frame_id=base_link`；pose 在 `world`，twist 按 ROS 规范表达在 `base_link` |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 非零采集时间戳；点坐标与 `header.frame_id` 均为 `world`；点云非空且字段布局有效 |

仿真由 `sim2real_simulation/localization.launch` 生成这两个 topic；真机由
`odom_to_base.py` 和 `cloud_relay.py` 适配 FAST-LIO 输出。需要跨坐标系时，点云
必须使用采集时刻的 TF 做数值变换；时间戳或 TF 无效时直接丢弃，禁止只修改
`frame_id`。规划器适配器负责按自身约定处理速度坐标系，例如 Fast adapter 会把
`base_link` 线速度旋转到 `world`。

## 公共链路

```text
/localization/odom ───────────────┐
/localization/cloud_registered ───┤
/goal ────────────────────────────┴─> planner_gateway
                                      │
                                      ├─> 选中的插件 adapter
                                      │     ├─ Diff GridMap + Diff-Planner
                                      │     └─ Fast SDFMap + Kino/Topo
                                      │
                                      ├─> /planning/status
                                      ├─> /planning/capabilities
                                      └─> /command/trajectory
                                                  │
                                                  ▼
                                         se3_controller_node
                                                  │
                                                  ▼
                                     /mavros/setpoint_raw/attitude
```

`planner_gateway` 是 `/command/trajectory` 的唯一合法发布者。原生 `PolyTraj`、
`Bspline` 和各版本 `PositionCommand` 只存在于插件私有命名空间，不属于公共 API。
SE3 状态反馈来自 MAVROS 的 `/mavros/local_position/odom` 与
`/mavros/imu/data`。

公共规划接口如下：

| 名称 | 类型 | 作用 |
|---|---|---|
| `/goal` | `geometry_msgs/PoseStamped` | 操作层目标；`world` 坐标，零四元数表示不约束终点 yaw |
| `/planning/goal` | `sim2real_planning_msgs/PlannerGoal` | 网关分配 session/goal ID 后的目标记录 |
| `/planning/status` | `PlannerStatus` | 当前后端状态、目标/轨迹 ID、实际目标、地图与里程计 readiness |
| `/planning/capabilities` | `PlannerCapabilities` | 当前后端能力、动力学限制和可选地图边界 |
| `/planning/validate_goal` | `ValidateGoal` service | 把目标验证代理给当前后端 |
| `/planning/cancel` | `std_srvs/Trigger` | 取消当前目标；是否支持由插件能力声明 |
| `/planning/command` | `PlannerCommand` | 已通过网关校验的命令观测流 |
| `/command/trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | SE3 控制器输入 |
| `/planning/viz/occupancy` | `sensor_msgs/PointCloud2` | 与插件无关的归一化规划障碍显示 |
| `/planning/viz/inflated_occupancy` | `sensor_msgs/PointCloud2` | 与插件无关的归一化安全膨胀显示 |
| `/planning/viz/planning_bounds` | `visualization_msgs/Marker` | 当前插件声明的可选固定地图边界 |

目标、Mission 和 RViz 桥只依赖这些公共状态与服务，不再读取 Diff 私有节点参数或
订阅 `PolyTraj`。RViz 同样不订阅 Fast/Diff 原生地图和轨迹；插件 raw 可视化先经过
manager 的公共归一化节点。

## 安全语义

- 插件只能在整栈启动时选择；不支持飞行中热切换或故障后自动启动另一规划器。
- 新目标先关闭旧命令门。首条运动命令必须同时匹配 backend、session、goal ID 和
  trajectory ID，并具有 `ACTIVE + armable` 状态。
- 已授权目标可以继续输出对应的 `HOLD/BRAKE` 安全命令；目标到达后只允许
  `HOLD`。
- 离开 armed OFFBOARD、地图/里程计失效、后端 `FAULT`、状态或命令超时、取消
  目标，以及检测到第二个 `/command/trajectory` 发布者，都会撤销当前目标授权。
  重新进入 OFFBOARD 不会重放旧目标，必须发布一个新目标。
- `localization_guard.py` 独立检查里程计时间戳、接收间隔、有限数值和位置跳变。速度上限默认关闭，因为正常飞行速度属于规划器/机体能力而不是定位接口约束；需要传感器毛刺上限时可显式设置私有参数 `~max_speed`。
  定位故障会锁存并请求 `AUTO.LAND`；完整栈重启前不再接受自主命令。

`arm_executor.py`、`goal_executor.py`、`mission_executor.py` 与
`waypoint_mission.py` 还会检查 MAVROS、定位和当前规划器 readiness。目标或 Mission
在发布前调用当前插件的 `ValidateGoal`，因此固定地图插件可以在起飞前拒绝越界
航点。

## `common/` 内容

| 路径 | 作用 |
|---|---|
| `launch/planner.launch` | 兼容入口；转交给 `sim2real_planner_manager`，不直接启动原生规划器 |
| `launch/trajectory_converter.launch` | 已弃用的空兼容层；禁止再启动绕过网关的转换器 |
| `launch/controller.launch` | 加载公共与载体专用配置并启动 SE3 |
| `launch/planning_control.launch` | 组合插件管理器与控制器 |
| `scripts/arm_executor.py` | 原生起飞与 OFFBOARD 交接 |
| `scripts/goal_executor.py` | 单目标飞前检查、后端验证与发布 |
| `scripts/mission_executor.py` | Mission 飞行状态机 |
| `scripts/waypoint_mission.py` | Mission JSON 校验、全航点预验证和到达监测 |
| `scripts/localization_guard.py` | 定位失效保护 |
| `scripts/rviz_goal_to_diff_planner.py` | 历史文件名保留；实际面向当前选中插件的安全 RViz 目标桥 |

仓库根目录的 `launch/` 负责容器与整条链路编排；ROS XML launch 保留在对应 ROS
包内。

## 参数归属

| 路径 | 归属 |
|---|---|
| `common/config/controller.yaml` | 两端共享的 SE3 算法与安全默认值 |
| `simulation/config/controller.yaml` | 仿真载体悬停推力、推力限制和围栏 |
| `deployment/config/controller.yaml` | 真机悬停推力、积分、推力限制和围栏 |
| `planning/ros_pkgs/sim2real_diff_adapter/config/` | Diff 插件打包的默认规划、轨迹采样与 adapter readiness 参数 |
| `planning/ros_pkgs/sim2real_fast_adapter/config/` | Fast Kino/Topo 共用参数及唯一的 `30 × 30 m` 水平固定地图配置 |
| `planning/plugins/*/planner.plugin.yaml` | 插件身份、workspace、launch、profile、超时、最低频率和能力声明 |

`SIM_PLANNER_CONFIG` 和 `PLANNER_CONFIG` 只适用于 `diff`，Fast 的地图与算法参数
由其 profile 管理。不要把某个规划器的地图参数复制到 `common/`、`simulation/`
或 `deployment/`。

## 修改原则

- 新定位或传感器：在环境目录实现适配器，只输出两个公共 localization topic。
- 新规划器或地图：在独立插件 workspace 中实现，遵守公共消息接口；不要让上层
  Mission 依赖其原生消息。
- 修改规划器默认值：只改对应插件配置，并完成该插件的仿真回归。
- 修改悬停推力、推力上限或载体围栏：只改对应环境的 controller 配置。
- 原生规划器不得直接订阅环境私有的 `/Odometry`、`/cloud_registered` 或
  `/livox/lidar`。
