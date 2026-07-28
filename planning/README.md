# 多规划器插件框架

`planning/` 把 Diff-Planner、Fast-Planner Kinodynamic 和 Fast-Planner
Topological 封装为同一套 ROS1 接口。这里的“插件”是独立 catkin workspace 中的
ROS 节点束，而不是同进程 `pluginlib` 类；这种结构用于隔离上游同名包、库和不兼容
消息。

三个内置插件 ID 为：

| ID | ROS namespace | 算法 | Profile | 仿真 | 真机 |
|---|---|---|---|---|---|
| `diff` | `diff` | Diff-Planner | `local` | 已启用 | 已启用 |
| `fast-kino` | `fast_kino` | Kinodynamic A* + B-spline | `local`（唯一默认项） | 已启用 | 未放行 |
| `fast-topo` | `fast_topo` | Topological PRM + B-spline | `local`（唯一默认项） | 已启用 | 未放行 |

ROS1 graph name 不允许连字符，因此公开 ID 保留 `fast-kino` / `fast-topo`，topic
命名空间使用下划线形式。Fast 的 `real_flight` capability 当前为 `false`；真机入口
会在启动前拒绝它们。

## 选择规划器

先查看无需构建即可发现的插件：

```bash
./launch/sim.sh planners
./launch/real.sh planners
```

仿真在启动前选择一个后端：

```bash
./launch/sim.sh --planner diff start
./launch/sim.sh --planner fast-kino start
./launch/sim.sh --planner fast-topo start
```

Fast Kino/Topo 共用同一份 `30 × 30 × 5 m` 固定地图配置，场景不再触发 profile
切换。例如森林场景仍然只选择场景和规划器：

```bash
./launch/sim.sh --scene outdoor_rectangular_forest \
  --planner fast-kino start
```

必须先安全停止完整栈，才能换一个插件重新启动。不存在空中热切换、并行命令输出或
自动 fallback。内置插件只需用 `SIM_PLANNER` 或 `REAL_PLANNER` 设置默认选择。
`--planner-profile`、`SIM_PLANNER_PROFILE` 和 `REAL_PLANNER_PROFILE` 仅为可能提供
多个配置的仓库外插件保留；三个内置插件无需设置。

## 数据流与接口

```text
 /localization/odom              /localization/cloud_registered
          │                                  │
          └──────────────┬───────────────────┘
                         ▼
 /goal ─────────> planner_gateway ─────> selected backend adapter
                         ▲                         │
                         │         private native planner/map/messages
                         │                         │
                         └──── status/command/capabilities
                         │
                         └────> /command/trajectory ──> SE3
```

### 传感器和操作输入

| 名称 | 类型 | 契约 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | `world` pose、`base_link` twist、非零测量时间戳、有限数值 |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 点与 `frame_id` 均为 `world`，非零采集时间戳 |
| `/goal` | `geometry_msgs/PoseStamped` | `world` 目标；零四元数表示不约束终点 yaw |
| `/mavros/state` | `mavros_msgs/State` | 网关 armed/OFFBOARD 门控 |

框架不统一地图。Diff 保留 rolling GridMap；两个 Fast 后端保留 SDFMap/ESDF。地图
分辨率、边界、膨胀、unknown 策略和更新方式全部属于插件私有实现。

### 公共规划 API

`sim2real_planning_msgs` 定义：

- `PlannerGoal`：gateway session、单调递增 goal ID、`PLAN/CANCEL`、目标 pose 和
  yaw 是否受约束；
- `PlannerStatus`：`STARTING/READY/PLANNING/ACTIVE/HOLDING/REACHED/FAULT`、
  goal/trajectory ID、实际采用的 `active_goal`、odom/map readiness、armable 和
  原因；
- `PlannerCommand`：相关 ID、`NORMAL/HOLD/BRAKE` 和一个
  `MultiDOFJointTrajectoryPoint`；
- `PlannerCapabilities`：接口版本、backend/variant、仿真与真机放行、yaw/cancel/
  验证/RViz 能力、动力学限制和可选固定地图 AABB；
- `ValidateGoal`：由 gateway 转发给当前插件的目标验证服务。

稳定公共名称为：

| 名称 | 方向 |
|---|---|
| `/planning/goal` | gateway 发布带 ID 的目标记录 |
| `/planning/status` | gateway 发布当前选中后端的状态 |
| `/planning/capabilities` | gateway 发布并校验后的能力 |
| `/planning/command` | gateway 发布已接受的命令观测流 |
| `/planning/validate_goal` | 公共目标验证服务 |
| `/planning/cancel` | 公共取消服务 |
| `/command/trajectory` | gateway 唯一发布的 SE3 输入 |
| `/planning/viz/occupancy` | 公共节点过滤后的当前规划障碍显示 |
| `/planning/viz/inflated_occupancy` | 公共节点过滤后的安全膨胀显示 |
| `/planning/viz/planning_bounds` | capabilities 声明的可选固定地图线框 |

每个 adapter 位于 `/planning/backends/<ros_namespace>`，使用同名的 `goal`、
`status`、`command`、`capabilities` 和 `validate_goal`。原生消息必须继续留在该
namespace 的 `native/` 子树中。

插件地图可视化只能发布到 `/planning/viz/raw/occupancy` 和
`/planning/viz/raw/inflated_occupancy`；算法特有 marker 放在
`/planning/viz/backend/*`。`planner_visualization` 是稳定地图显示话题的唯一发布者，
统一 frame、有限值、显示高度、失效清空、固定边界和已接受命令路径。Fast adapter
注入的虚拟地面属于规划私有输入，不进入 raw occupancy；膨胀图中的低层地面体素也只
在显示层过滤，规划地图本身保持不变。

## Workspace 隔离

生成的 workspace 位于 `planning/workspaces/`：

| Workspace | 内容 | Underlay |
|---|---|---|
| `interfaces_ws` | `sim2real_planning_msgs` | ROS Noetic |
| `control_ws` | manager/gateway、`common`、SE3 和环境公共包 | `interfaces_ws` |
| `diff_ws` | Diff 上游源码和 `sim2real_diff_adapter` | `interfaces_ws` |
| `fast_ws` | 固定版本 Fast 上游源码和 `sim2real_fast_adapter` | `interfaces_ws` |

`control_ws`、`diff_ws` 与 `fast_ws` 互不 overlay。运行时 manager 只 source 被选中
插件的 `setup.bash`，因此 Diff 和 Fast 的 `plan_env`、`path_searching`、
`traj_utils`、`quadrotor_msgs` 不会互相遮蔽。

容器入口会自动构建这些 workspace。需要手工构建或调试时：

```bash
./planning/scripts/build_planner_workspaces.sh \
  --flavor simulation --test

# 只重建接口和 Fast；interfaces 仍必须先可用
./planning/scripts/build_planner_workspaces.sh \
  --flavor simulation --workspaces interfaces,fast

./planning/scripts/check_planner_integration.sh
```

生成的 `build/`、`devel/` 和 source symlink 不是源码，不应提交。

## Manifest 发现与校验

内置 manifest 位于 `planning/plugins/<id>/planner.plugin.yaml`。严格 schema 包含：

- `api_version`、`id`、`ros_namespace`、`adapter_node`、`display_name`、
  `variant`；
- `workspace_setup`（内置插件相对仓库根目录，外部插件相对 manifest 所在目录）；
- launch package、文件和固定参数；
- `default_profile` 与允许的 `profiles`；
- startup/status/command timeout 和最低发布频率；
- simulation、real-flight、yaw、cancel、goal-validation、RViz capability。

未知字段、重复 ID、非法 ROS 名称、接口版本不匹配、未知 profile、未放行的运行模式
都会导致启动失败。启动阶段还会检查 workspace `setup.bash`、ROS package 和 launch
文件。运行时 `PlannerCapabilities` 必须与 manifest 完全一致。

仓库外插件可通过只读、冒号分隔的 `SIM2REAL_PLANNER_PLUGIN_PATH` 加入扫描路径；
外部插件不能覆盖内置 ID。启动器把这些目录以原绝对路径只读挂入容器。后端子进程
还要求 `SIM2REAL_RUNTIME_MODE=simulation|real` 与 launch 参数完全一致，不能绕过
manifest 的运行模式放行。宿主侧工具不依赖 ROS：

```bash
python3 planning/scripts/planner_manifest.py \
  --project-root "$PWD" \
  --manifest-root "$PWD/planning/plugins" list

python3 planning/scripts/planner_manifest.py \
  --project-root "$PWD" \
  --manifest-root "$PWD/planning/plugins" \
  resolve fast-kino --mode simulation
```

## 添加规划器

1. 为算法建立独立 workspace；若它与现有后端不存在包名/ABI 冲突，也可以复用一个
   已隔离 workspace。把上游源码固定到可复现版本并保留许可证。
2. 编写 adapter，只订阅公共 odom/cloud，接收 `PlannerGoal`，发布
   `PlannerStatus`、`PlannerCommand`、latched `PlannerCapabilities`，并实现
   `ValidateGoal`。adapter 必须校验 frame、时间戳、有限值、空输入和原生轨迹 ID。
3. 提供一个 launch，接受 manifest 声明的 `backend_id`、`profile`、
   `odom_topic` 和 `cloud_topic`；有连字符的 ID 另传合法 `backend_namespace`。
   adapter ROS node 名必须与 manifest 的 `adapter_node` 一致，gateway 只接受该
   caller ID 发布的 status、capabilities 和 command。
4. 新建 `planning/plugins/<id>/planner.plugin.yaml`。新后端首先设置
   `real_flight: false`，完成独立真机验收后才能放行。
5. 如新增 build domain，在 `build_planner_workspaces.sh` 和镜像中加入其 source
   link/build 步骤；不要把冲突 workspace 互相 extend。
6. 增加 manifest、输入契约、目标验证、状态关联、命令有限值及故障注入测试，运行
   `check_planner_integration.sh` 和完整仿真测试。

新增插件不应修改 Mission、SE3 或 gateway 的算法分支；差异通过 manifest、
capabilities、profile 和 adapter 消化。

## 安全限制

- gateway 只接受当前 backend/session/goal/trajectory 的有限、结构合法且时间新鲜的
  命令，并审计 `/command/trajectory` 不存在第二个发布者。
- 每个新目标都关闭旧命令门；只有 `ACTIVE + armable` 的首条 `NORMAL` 命令能打开。
  已授权目标可继续发 `HOLD/BRAKE`，到达后只允许 `HOLD`。
- 离开 armed OFFBOARD、输入 readiness 丢失、`FAULT`、状态或命令超时、取消以及
  显式安全关闭会永久撤销当前 goal。即使重新进入 OFFBOARD，也必须发新目标。
- Diff 的 CANCEL/瞬时输入故障使用私有可恢复制动，停车后可接收新目标；真正的
  `mandatory_stop` 和深度安全锁止仍不可恢复。传感器恢复不会复播被撤销的旧目标。
- Fast Kino/Topo 共用唯一的固定地图配置。Mission 会在起飞前逐点调用
  `ValidateGoal`；只对固定地图 AABB 和障碍膨胀边距做检查。局部地图更新窗口会在
  AABB 边缘自动裁剪，不会再用 `local_update_range` 缩小可接受目标范围。地图范围
  为 `30 × 30 × 5 m`，原点为 `(-15, -15, -1)`，分辨率为 `0.1 m`；对应
  `300 × 300 × 50 = 4,500,000` 个体素，保守内存估算约 `275 MiB`，低于
  `512 MiB` 分配上限。垂直更新半径保留原生 `4.5 m`，确保悬停时仍能融合地面点云。
  由于 Fast 的独立点云路径每帧重建占据图，而 MID-360 单帧地面比原生深度图稀疏，
  Fast adapter 会在私有点云中补充配置的稠密虚拟地面，同时拒绝穿越该地面的目标和
  原生轨迹。这些都是 adapter 配置，不修改 Fast 搜索或优化算法本体。
- Fast 的 `manager/max_vel` / `manager/max_acc` 是搜索与优化使用的名义动力学参数，
  不是插件边界的硬拒绝阈值。adapter 继续拒绝非有限或结构非法的轨迹，但有限的
  Topo refinement 不会仅因短时超过名义值而触发 `FAULT`、撤销目标。
- 统一地图无法覆盖仓库现有的百米级 `mission_outdoor*.json` 全部航点；Fast 会在
  起飞前明确拒绝越界 Mission。此类路线需要拆分地图/任务或使用支持滚动地图的后端，
  不能通过关闭边界检查绕过。
- Fast 只有在真实 odom 的位置、速度（约束 yaw 时还包括 yaw 和 yaw rate）连续满足
  配置阈值后才发布 `REACHED`。轨迹时间结束但机体尚未到达时继续保持 `ACTIVE + HOLD`，
  不会把轨迹播放完成误报为飞行成功。
- 当前版本只处理静态障碍，不提供动态障碍共享、地图共享、空中切换、自动 fallback
  或多规划器并行控制。
