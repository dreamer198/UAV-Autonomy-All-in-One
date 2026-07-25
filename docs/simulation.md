# PX4 / Gazebo 仿真

`simulation/` 目录包含项目的仿真专用部分：PX4 SITL、Gazebo Classic、模拟 MID-360、Iris 机体标定，以及把仿真数据转换到公共定位接口的 ROS 节点。

仿真不运行 FAST-LIO。Planner、轨迹转换和 SE3 控制器直接使用 `common`，与真机保持一致。

## 数据链路

```text
PX4 SITL + Gazebo + iris_mid360
  ├─ MAVROS odom ──> sim_odometry_adapter.py
  └─ 模拟点云 + TF ─> pointcloud_to_world.py
                         │
                         ▼
              /localization/odom
              /localization/cloud_registered
                         │
                         ▼
             Diff-Planner → converter → SE3 → PX4 SITL
```

## 前置条件

- Docker、tmux；
- 当前用户可以访问 Docker daemon；
- 图形模式需要 X11；
- 首次构建需要联网下载固定版本的 PX4 和 MID360 插件源码。

仿真使用独立容器 `diff_planner_px4_sim`，不依赖 `ros_noetic` 或真机容器。

## 快速开始

所有命令都在仓库根目录执行。

首次启动：

```bash
./launch/sim.sh start
```

切换 Gazebo 场景仍使用同一个入口。例如加载室外 bag 重建场景：

```bash
./launch/sim.sh --scene outdoor_rectangular_forest restart
```

之后的 `arm/goal/mission/land/stop` 命令不变。场景配置位于
`simulation/config/scenes/*.env`；可版本化的重建资产位于同名子目录。配置只保存
world 路径和出生位姿，控制器、Planner、传感器适配和飞行状态机不会随场景切换。
也可以不创建场景配置，直接临时覆盖：

```bash
SIM_WORLD=/root/simulation_runtime/reconstructed/new_scene/world.world \
SIM_SPAWN_X=0 SIM_SPAWN_Y=0 SIM_SPAWN_YAW=0 \
./launch/sim.sh restart
```

该命令会自动：

1. 构建或检查仿真镜像和容器；
2. 增量编译仓库 ROS 源码；
3. 启动 ROS Master、PX4 SITL、Gazebo、MID360 和 MAVROS；
4. 启动定位适配、Diff-Planner、轨迹转换和 SE3；
5. 启动 RViz，并逐级检查关键 topic。

车辆启动后保持 disarmed。执行完整飞行流程：

```bash
./launch/sim.sh arm
./launch/sim.sh goal 1.0 0.0 1.0
./launch/sim.sh land
./launch/sim.sh stop
```

`arm` 会先让 SITL 解锁并进入 PX4 原生 `AUTO.TAKEOFF`，上升到相对 Home 默认 `1.0 m`。达到高度后，脚本确认 SE3 当前位置预热 setpoint 持续有效，再自动切换到 OFFBOARD；SE3 锁定切换瞬间的位姿悬停。只有整段交接完成后才应发布规划目标。

## 发布目标

```text
./launch/sim.sh goal X Y Z [YAW_DEG]
```

示例：

```bash
# 不限定终点 yaw
./launch/sim.sh goal 1.0 0.0 1.0

# 到达后朝向 90°
./launch/sim.sh goal 1.0 1.0 1.0 90
```

仿真和真机入口都调用共享 `goal_executor.py`。它在一个 ROS 进程中并行等待 connected、armed + OFFBOARD、新鲜定位、10 条连续 SE3 姿态/推力输出、Planner 节点以及 `/goal` 的两个消费者，检查目标高度围栏后直接发布。正常飞行逻辑在两端完全一致，不再串行执行多次 `rostopic/rosnode/rosparam`；`SIM_REQUIRE_ARMED_GOAL=false` 只保留为仿真 Planner-only 测试开关。

规则：

- 目标坐标系为 `world`；
- 省略 yaw 时不限定终点朝向，飞行中按路径方向生成 yaw；
- 提供 yaw 时，单位为度，到达终点前会转向该角度；
- 默认高度必须满足 `0.1 < Z < 3.0 m`；
- 单次目标相对当前里程计位置的三维直线距离不得超过默认 `200 m`；
- 默认要求车辆已经 armed 且处于 OFFBOARD；
- `arm` 完成前点击的 RViz 目标会被忽略，进入 armed OFFBOARD 后需要重新点击。

## 顺序航点任务

```bash
./launch/sim.sh mission mission_outdoor.json
```

`waypoints` 数组按顺序执行；`x/y/z` 是 `world` 中的绝对位置。Mission 自动把每个中间点的 yaw 设为“当前点 → 下一点”的方向，最后一点沿用进入方向；通常不必在 JSON 中手工填写 yaw，显式 `yaw` 只用于特殊覆盖。仿真与真机执行同一个共享 `mission_executor.py` 状态机。飞机未解锁时先使用 `takeoff_height` 完成 PX4 原生起飞，等待高度进入目标容差且垂直速度稳定后再进入 OFFBOARD；已经处于 armed OFFBOARD 时跳过起飞和位置 setpoint 预热，只验证新的 SE3 输出。`takeoff_settle_time` 默认为 `0.0 s`，表示不在必需的稳定判断之后追加等待。

默认 `fly_through=true`：中间航点同时进入 `fly_through_tolerance=0.5 m` 并满足自动 yaw 容差后发布下一目标；不等待减速或稳定时间，因此朝向下一航段后连续通过而不停车。最后一个航点仍会沿进入方向停稳后再自动降落。某个中间点需要停车时，在该航点设置 `"fly_through": false`。若配置航点落入膨胀障碍，Mission 仍接受 Planner 返回的安全替代点。

自动 yaw 会跳过 x/y 水平距离小于 `fly_through_tolerance` 的近重合航点，向后查找第一个足够远的点；末尾近重合点沿用最近的有效航段方向。整条任务都没有有效水平航段时，所有缺省 yaw 会锁定为任务开始时的新鲜里程计朝向，纯垂直任务不会产生随机转向。

自动起飞期间切换到 `AUTO.TAKEOFF/AUTO.LOITER` 以外的模式，或航点执行期间离开 OFFBOARD，任务都会立即中止且不会自动降落；重新进入 OFFBOARD 也不会恢复旧任务。Planner 因临近碰撞发布临时急停轨迹时，任务默认等待 `2.0 s` 自动重规划；若规划器停稳后回到等待目标状态，Mission 会用新的时间戳重新发送当前实际航点（包括已接受的安全替代点），默认最多重试 `3` 次。可分别通过 `planner_recovery_timeout` 和 `planner_retry_limit` 调整；重试耗尽，或发生定位、规划和到达监测失败时，才请求一次 `AUTO.LOITER`，不继续后续航点。

RViz 的 `2D Nav Goal` 使用独立话题 `/sim2real/rviz_goal`，目标桥再转发到 `/goal`；不要改回 MAVROS 同样会使用的 `/move_base_simple/goal`。

## 常用操作

| 命令 | 作用 |
|---|---|
| `./launch/sim.sh start` | 构建变更并启动完整仿真 |
| `./launch/sim.sh restart` | 停止并重新启动 |
| `./launch/sim.sh stop` | 降落并停止仿真栈 |
| `./launch/sim.sh build` | 只构建 ROS overlay |
| `./launch/sim.sh test` | 构建并运行测试和 launch 校验 |
| `./launch/sim.sh status` | 查看容器、tmux 和 ROS 状态 |
| `./launch/sim.sh attach` | 进入 tmux 日志界面 |
| `./launch/sim.sh shell` | 进入仿真容器 |
| `./launch/sim.sh arm/land` | SITL 原生起飞并自动 OFFBOARD，或降落 |
| `./launch/sim.sh mission FILE` | 自动起飞、顺序执行 JSON 航点并降落 |

tmux 中使用 `Ctrl-b n/p` 切换窗口，`Ctrl-b d` 退出查看但不停止仿真。

## 无界面运行

```bash
SIM_GAZEBO_GUI=false \
SIM_START_RVIZ=false \
./launch/sim.sh restart
```

没有可用 GPU 时，可显式使用软件渲染：

```bash
./launch/sim.sh stop
SIM_GPU_MODE=none ./launch/sim_container.sh recreate
```

## 修改代码后的操作

普通 ROS 源码或 YAML 修改：

```bash
./launch/sim.sh test
./launch/sim.sh restart
```

只有修改以下内容时才需要重建镜像：

- `simulation/Dockerfile`；
- `simulation/versions.env`；
- `simulation/assets/`；
- 镜像内系统依赖。

重建流程：

```bash
./launch/sim.sh stop
./launch/sim_container.sh build
./launch/sim_container.sh recreate
./launch/sim.sh start
```

容器检查：

```bash
./launch/sim_container.sh verify
./launch/sim_container.sh status
```

## 主要配置

| 文件 | 作用 |
|---|---|
| `simulation/versions.env` | 固定 PX4 与 MID360 插件版本 |
| `simulation/assets/px4/` | PX4 launch、world 和模型覆盖 |
| `simulation/config/controller.yaml` | 仿真 Iris 的 SE3 标定 |
| `simulation/config/scenes/*.env` | 可切换的 Gazebo world 与出生位姿 |
| `simulation/config/rviz/sim.rviz` | RViz 显示配置 |
| `simulation/ros_pkgs/sim2real_simulation/` | 仿真定位、点云和目标适配节点 |
| `common/config/planner.yaml` | 仿真与真机共享的 Planner 参数 |

常用环境变量：

| 变量 | 默认值 | 作用 |
|---|---|---|
| `SIM_GAZEBO_GUI` | `true` | 是否启动 Gazebo GUI |
| `SIM_START_RVIZ` | `true` | 是否启动 RViz |
| `SIM_GPU_MODE` | `auto` | `auto/nvidia/dri/none` |
| `SIM_BUILD_JOBS` | `4` | overlay 编译并行度 |
| `SIM_IMAGE_BUILD_JOBS` | `4` | 镜像内 PX4/MID360 编译并行度 |
| `SIM_SKIP_BUILD` | `false` | 启动前是否跳过 overlay 编译 |
| `SIM_SCENE` | `default` | `simulation/config/scenes` 中的场景名称 |
| `SIM_WORLD` | 场景配置值 | 临时覆盖容器内 Gazebo world 路径 |
| `SIM_SPAWN_X/Y/Z/YAW` | 场景配置值 | 临时覆盖车辆出生位姿 |
| `SIM_TAKEOFF_HEIGHT` | `1.0` | PX4 原生起飞的 Home 相对高度 |
| `SIM_TAKEOFF_TIMEOUT` | `30` | 原生起飞高度监测超时，单位 s |
| `SIM_TAKEOFF_TOLERANCE` | `0.1` | 脚本对实际相对高度的完成判定容差，不修改 PX4 接受半径 |
| `SIM_COMMAND_TIMEOUT` | `15` | 解锁与模式切换超时，单位 s |
| `SIM_PREFLIGHT_TIMEOUT` | `5.0` | 并行等待状态、高度和连续 setpoint 的最长秒数 |
| `SIM_RVIZ_GOAL_Z` | `1.0` | RViz 2D 目标使用的高度 |
| `SIM_PLANNER_CONFIG` | 空 | 可选 Planner 完整 YAML 覆盖 |

## 验证运行状态

进入容器：

```bash
./launch/sim.sh shell
```

在容器中检查：

```bash
rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic echo -n1 /mavros/state
rosnode list | grep -E 'diff_planner|traj_server|trajectory_msg_converter|se3_controller'
```

发布目标后还应看到：

```bash
rostopic hz /drone_0_planning/trajectory
rostopic hz /command/trajectory
rostopic hz /mavros/setpoint_raw/attitude
```

## 日志与排错

每次运行的日志位于：

```text
runtime/simulation/runs/<run-id>/
```

常见问题：

- 目标无响应：先执行 `sim.sh arm`，等待原生起飞和自动 OFFBOARD 交接完成后重新发布目标；
- Gazebo/RViz 无法显示：关闭 GUI 后使用 headless 模式；
- 容器挂载或 GPU 配置过期：运行 `sim_container.sh verify`，必要时 `recreate`；
- 修改源码未生效：运行 `sim.sh restart`，不要只重启容器；
- 修改 Dockerfile 或 PX4 资产未生效：重新 build 并 recreate 容器。

## 可复现版本

当前仿真镜像固定使用：

- PX4 `v1.14.3`，commit 记录在 `simulation/versions.env`；
- MID360 仿真插件 commit 记录在 `simulation/versions.env`；
- Gazebo Classic 和 ROS Noetic 由镜像统一提供。

镜像构建完成后，日常启动不再需要联网。
