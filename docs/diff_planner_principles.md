# Diff-Planner 工作原理与限制

本文说明本仓库中 Diff-Planner 的实际数据流、主要机制和使用边界。启动命令见
[仿真说明](simulation.md) 和 [真机说明](deployment.md)；下游控制器见
[SE3 控制器](se3_controller.md)。

## 1. 系统位置

Diff-Planner 是局部轨迹规划器，不是基于完整环境地图求解起点到终点的全局规划器。

```text
/localization/odom + /localization/cloud_registered
                    │
                    ▼
          GridMap 局部占据地图
                    │
/goal ──► DiffReplanFSM ──► MINCO 局部轨迹
                    │
                    ▼
             traj_server 采样
                    │ PositionCommand
                    ▼
       trajectory_msg_converter.py
                    │ MultiDOFJointTrajectory
                    ▼
          SE3 控制器 ──► MAVROS ──► PX4
```

仿真和真机在进入 Planner 前都被适配成相同接口：

- 里程计：`/localization/odom`，位姿在 `world`，速度按
  `nav_msgs/Odometry` 约定表达在 `child_frame_id`；
- 注册点云：`/localization/cloud_registered`，点坐标在 `world`；
- 目标：`/goal`，位置在 `world`。

Planner 将里程计速度旋转到世界系，再用于轨迹边界条件。因此仿真 MAVROS
里程计和真机 FAST-LIO 里程计共用同一套规划参数。

## 2. 局部地图

### 2.1 Ring buffer 占据地图

`GridMap` 维护随飞机移动的三维 ring buffer：

- `occupancy_buffer_` 保存 log-odds 占据概率；
- `occupancy_buffer_inflate_` 保存膨胀邻域内的障碍计数；
- 点云端点作为命中，传感器到端点之间的体素作为空闲射线更新；
- 深度图和点云射线短于 `grid_map/min_ray_length` 时不更新地图；
- 窗口移动时只清理滑出的体素层，内存大小保持不变。

地图不维护 ESDF。规划器只查询原始占据和膨胀占据，旧障碍是否随时间衰减由
`grid_map/fading_time` 决定。射线端点超出窗口时会先裁到地图边界，再检查最短
长度；`min_ray_length` 用于过滤机体附近回波和零长度射线，不是传感器最大量程。

### 2.2 障碍膨胀与虚拟墙

`obstacles_inflation` 会被换算成整数体素数，并在 x、y、z 三个方向扩展为
轴对齐的体素立方体。这个参数是规划碰撞包络，不应直接解释为某个机体尺寸的
“半径”或“外接圆半径”；更换机体、桨叶保护圈或定位精度后需要重新验证。

启用虚拟墙时，满足下列条件的位置直接视为不可通行：

```text
z <= virtual_ground
z >= virtual_ceil
```

目标点还需为完整膨胀包络留出空间，因此有效目标高度满足：

```text
virtual_ground + obstacles_inflation
    < goal_z <
virtual_ceil - obstacles_inflation
```

具体数值以 [Diff 插件配置](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) 为准。

### 2.3 未知空间

当前 `getOccupancy()` 和 `getInflateOccupancy()` 对 ring buffer 之外的位置返回
空闲。也就是说，局部地图之外是“未知但允许搜索”，不是“未知即障碍”。

这项行为有两个后果：

- 局部规划不会因为目标超出当前感知窗口就立即拒绝；
- 远距离或隔着复杂障碍的目标没有全局安全保证，后续只能依赖滚动感知和重规划。

## 3. 从目标到局部轨迹

### 3.1 全局参考轨迹

收到目标后，Planner 先生成从当前点到目标点的 minimum-jerk 几何参考轨迹。
这条轨迹用于选择滚动局部目标，生成时不对完整环境做全局碰撞搜索。

因此 RViz 中显示的“全局轨迹”不等于安全的全局路线。墙体、U 形障碍、死胡同
或尚未进入局部地图的障碍，都可能使后续局部规划失败。长距离任务应拆成经过
验证的 Mission 航点；若需要自动绕过大尺度拓扑障碍，应接入持久化全局地图和
全局规划器。

### 3.2 局部目标

`getLocalTarget()` 沿参考轨迹向前搜索，在 `planning_horizon` 附近选择本轮
局部目标。若候选点已经落入膨胀障碍，Planner 会尝试：

1. 在候选点附近寻找有进展的自由点；
2. 沿当前参考轨迹向后寻找自由点；
3. 找不到有效替代点时让本轮规划失败。

自由点只有在膨胀地图中与当前飞机位置存在无碰撞直线连接时才会被采用，避免把
孤立自由体素或障碍另一侧的点当作可执行局部目标。若地图更新恰好把飞机所在体素
标为占用，连接检查只忽略该起始体素；离开该体素后的所有采样仍必须自由。

局部目标不是最终目标。飞机执行一段局部轨迹后，状态机会用最新里程计和地图
继续重规划。

### 3.3 A* 的作用

A* 不是从飞机当前位置到最终目标运行一次的全局搜索。后端发现初始轨迹的某一
段穿过障碍时，才对该碰撞段运行三维栅格 A*，用搜索折线确定从哪一侧绕开障碍。

搜索使用膨胀占据地图和 26 邻接节点，并受预分配搜索池和单次计算时间限制。
当碰撞段端点因体素量化落入占据格时，端点会沿远离碰撞段的方向做有界调整；
有界范围内找不到自由格则返回失败。

由于地图窗口外被视为空闲，A* 路径只能说明已知局部地图中的采样点未占据，不能
证明整条远距离路线已经被观测或全局可达。

### 3.4 MINCO 与轨迹优化

本仓库使用五阶 minimum-jerk 多项式。MINCO 以中间路点和每段时长作为优化变量，
通过带状线性系统恢复多项式系数。L-BFGS 同时考虑：

- jerk 平滑代价；
- A* 生成的避障方向；
- 速度、加速度和 jerk 可行性；
- 轨迹时间及采样点间距。

优化器使用软代价和有限次数重试，因此“有可行路线”不等于每次都能数值收敛。
规划失败会交给状态机重试或急停处理。

## 4. 状态机、重规划和急停

`DiffReplanFSM` 的主要状态为：

```text
INIT → WAIT_TARGET → SEQUENTIAL_START → EXEC_TRAJ
                         │                  │
                         └── 规划失败重试   ├── REPLAN_TRAJ
                                            └── EMERGENCY_STOP
```

主状态机以 100 Hz 检查状态，但不代表每 10 ms 都求解一次轨迹。正常重规划由
`fsm/thresh_replan_time`、局部轨迹剩余时间和安全事件触发；独立安全回调以
20 Hz 扫描当前轨迹的后续碰撞。

关键恢复机制：

- 单次规划失败后，按 `fsm/replan_retry_interval` 间隔重试；
- 持续失败达到 `fsm/replan_failure_timeout` 后发布急停轨迹；
- 预计碰撞已经进入 `fsm/emergency_time` 时，安全回调可更早急停；
- 飞机实际三维位移在 `fsm/stuck_timeout` 内始终小于
  `fsm/stuck_progress_threshold` 时触发失速保护；侧向或后退绕障也算移动，
  仅局部目标变化不算，终点位置已到达而只等待 yaw 时不判卡住；
- 急停制动期间收到的新目标会排队，停稳后可从当前里程计重新规划；
- 任务完成后，`traj_server` 继续保持精确终点，控制器可继续消除剩余位置误差。

若最终目标后来落入膨胀障碍，`fsm/mondify_final_goal` 允许 Planner 将目标改到
参考轨迹上的自由点。上层 Mission 会根据 Planner 实际接受的目标监测到达状态，
但操作人员仍应关注日志中的目标替换提示。

## 5. 轨迹执行与控制接口

规划成功后，FSM 发布包含完整多项式系数的 `PolyTraj`。`traj_server` 以
100 Hz 采样并发布：

- 位置、速度、加速度和 jerk；
- yaw 和 yaw rate；
- 轨迹 ID 与状态。

`trajectory_msg_converter.py` 当前只把位置、速度、加速度、yaw 和 yaw rate
送入 SE3 控制器，jerk 不进入主控制链路。SE3 最终向 PX4 发送姿态四元数和推力；
当前消息掩码忽略 body-rate，因此 yaw-rate 和 jerk 相关角速率前馈不会被 PX4
执行。详见 [SE3 控制器](se3_controller.md)。

`traj_server` 依赖 Planner 心跳。心跳超时时会终止当前轨迹并发布保持指令，避免
继续沿失去监管的轨迹飞行。

## 6. 配置来源

仿真和真机共享 [Diff 插件配置](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml)，yaw 采样参数位于
[trajectory_server.yaml](../common/config/trajectory_server.yaml)。常用参数分组：

| 分组 | 主要作用 |
|---|---|
| `fsm/*` | 重规划、急停、到达判定和失速检测 |
| `grid_map/*` | 地图范围、分辨率、膨胀、虚拟墙和占据概率 |
| `manager/*` | 速度、加速度、规划视距和多项式分段 |
| `optimization/*` | 障碍、速度、加速度、jerk、时间等约束与权重 |
| `traj_server/*` | yaw 前视、角速度和角加速度限制 |

不要在文档中复制一套“默认值”作为配置来源；修改和核对时以 YAML 及启动日志为准。

## 7. 使用边界

使用或排障时应牢记：

1. 单个 `/goal` 只提供目标，不提供经过全局验证的安全路线。
2. Planner 只保留局部滚动地图，窗口外空间当前按空闲处理。
3. 障碍安全性取决于点云覆盖、定位、外参、体素分辨率和膨胀包络。
4. 轨迹优化是数值求解，复杂障碍中可能失败并进入急停。
5. 急停后能接受新目标，不代表原目标自动变得可达。

出现“不飞”时，先区分三类状态：

- Planner 没有接受目标：检查是否已解锁且处于 OFFBOARD、目标高度、里程计和订阅者；
- Planner 规划失败或急停：检查规划日志、膨胀地图和局部目标；
- Planner 有轨迹但飞机不跟踪：检查转换器、SE3 输出和 PX4 模式。
