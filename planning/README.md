# 多规划器插件框架

规划器插件是独立 catkin workspace 中的 ROS 节点束，不是同进程 `pluginlib`。
这种结构隔离上游同名包、库和消息。

| ID | 私有命名空间 | 算法 |
|---|---|---|
| `diff` | `/planning/backends/diff` | Diff-Planner |
| `fast-kino` | `/planning/backends/fast_kino` | Kinodynamic A* + B-spline |
| `fast-topo` | `/planning/backends/fast_topo` | Topological PRM + B-spline |

## 选择规划器

`start` 和 `restart` 必须明确指定规划器：

```bash
./launch/sim.sh --scene room --planner diff start
./launch/sim.sh --scene room --planner fast-kino start
./launch/sim.sh --scene room --planner fast-topo start

./launch/real.sh --planner diff start
```

只能停止完整栈后再选择其他插件。`SIM_PLANNER` 和 `REAL_PLANNER` 分别等价于命令行
的 `--planner`。

## 输入与地图

所有插件只共享：

| 名称 | 类型 | 约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | `world` pose、`base_link` twist、有效测量时间戳 |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 点坐标和 `frame_id` 均为 `world` |
| `/goal` | `geometry_msgs/PoseStamped` | `world` 目标；零四元数表示不约束 yaw |
| `/mavros/state` | `mavros_msgs/State` | armed/OFFBOARD 命令门控 |

框架不共享地图。Diff 使用随飞机移动的 GridMap；Fast Kino/Topo 使用固定范围
SDFMap。Fast 默认地图参数在
[`config/planner.yaml`](ros_pkgs/sim2real_fast_adapter/config/planner.yaml)，
`forest` 仿真自动叠加
[`config/scenes/forest.yaml`](ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml)。
固定地图边界通过 `PlannerCapabilities` 公布并由目标验证服务执行。

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

| 名称 | 方向 |
|---|---|
| `/planning/goal` | gateway → 目标记录 |
| `/planning/status` | gateway → 上层状态 |
| `/planning/capabilities` | gateway → 上层能力 |
| `/planning/command` | gateway → 已接受命令观测 |
| `/planning/validate_goal` | 上层 → gateway → adapter |
| `/planning/cancel` | 上层 → gateway → adapter |
| `/command/trajectory` | gateway → SE3 |
| `/planning/viz/occupancy` | 统一占据显示 |
| `/planning/viz/inflated_occupancy` | 统一膨胀显示 |
| `/planning/viz/planning_bounds` | 可选固定地图边界 |

原生规划消息留在 `/planning/backends/<namespace>/native/`；算法调试可视化使用
`/planning/viz/backend/*`。公共 RViz 配置不依赖插件 ID。

## Workspace 隔离

生成目录为 `planning/workspaces/`：

| Workspace | 内容 | Underlay |
|---|---|---|
| `interfaces_ws` | 公共消息和服务 | ROS Noetic |
| `control_ws` | gateway、公共飞行逻辑、SE3 和环境适配 | `interfaces_ws` |
| `diff_ws` | Diff 上游包与 adapter | `interfaces_ws` |
| `fast_ws` | Fast 上游包与 adapter | `interfaces_ws` |

`control_ws`、`diff_ws` 和 `fast_ws` 互不 overlay。仿真启动器增量构建这些
workspace；真机镜像在 Docker build 阶段构建它们。运行时只 source 当前插件的
`setup.bash`。

## Manifest

内置 manifest 位于 `planning/plugins/<id>/planner.plugin.yaml`，包含：

- `api_version`、插件 ID、ROS namespace、算法 variant；
- workspace `setup.bash` 与 launch；
- profile、启动/状态/命令超时和最低频率；
- simulation、yaw、cancel、目标验证和 RViz 能力。

未知字段、重复 ID、接口版本错误、非法名称、缺失 workspace/launch 或运行时能力
不一致都会使启动失败。

仓库外插件可通过只读、冒号分隔的绝对路径加入：

```bash
SIM2REAL_PLANNER_PLUGIN_PATH=/abs/plugin-a:/abs/plugin-b \
./launch/sim_container.sh recreate
```

外部 ID 不能覆盖内置插件。其 workspace 路径相对 manifest 所在目录解析，并以只读
方式挂入容器。

## 添加规划器

1. 建立独立 workspace，固定上游版本并保留许可证。
2. 编写 adapter：订阅公共 odom/cloud，处理 `PlannerGoal`，发布 status、command、
   capabilities，并实现 `ValidateGoal`。
3. 提供接受 backend、profile、odom 和 cloud 参数的 launch。
4. 添加 `planning/plugins/<id>/planner.plugin.yaml`。
5. 如需新构建域，扩展 `build_planner_workspaces.sh` 和两个镜像。
6. 覆盖 manifest、输入校验、目标关联、命令校验和故障测试。

新增插件不应修改 Mission、SE3 或 gateway 的算法分支。

## 安全约束

- gateway 只接受当前 backend/session/goal/trajectory 的新鲜有限命令，并检查
  `/command/trajectory` 没有第二个发布者。
- 新目标、离开 armed/OFFBOARD、readiness 丢失、状态或命令超时、取消和后端
  `FAULT` 都会撤销旧命令。
- Fast 轨迹必须留在固定地图和虚拟地面之上；Diff 和 Fast 都使用实测里程计判定
  `REACHED`。
- 当前框架不提供地图共享、动态障碍预测接口、空中切换、自动 fallback 或多规划器
  并行控制。
