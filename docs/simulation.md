# PX4 / Gazebo 仿真

仿真栈包含 PX4 SITL、Gazebo Classic、模拟 MID-360 和仿真适配节点。Planner、轨迹转换与 SE3 控制器来自公共链路；仿真不运行 FAST-LIO。

```text
PX4 SITL + Gazebo + iris_mid360
  ├─ MAVROS odom ────────┐
  └─ 模拟点云 + TF ──────┴─> /localization/*
                              → Diff-Planner → SE3 → PX4 SITL
```

适配器只在显式配置下把 MAVROS `map` 视为与 Gazebo `world` 数值重合，不做坐标
数值变换；收到其他父/子坐标系的里程计会直接丢弃。点云严格使用测量时刻的 TF，
不回退到最新 TF。

所有命令均在仓库根目录执行。

## 快速开始

默认场景：

```bash
./launch/sim.sh start
```

内置室外场景：

```bash
./launch/sim.sh --scene outdoor_rectangular_forest restart
```

场景来源和重新生成方法见[室外重建场景](outdoor_bag_gazebo.md)。

启动完成后飞机保持未解锁。一次单目标流程为：

```bash
SIM_TAKEOFF_HEIGHT=1.5 ./launch/sim.sh arm
./launch/sim.sh goal 1.0 0.0 1.5
./launch/sim.sh land
./launch/sim.sh stop
```

`SIM_TAKEOFF_HEIGHT` 指定相对 PX4 Home 的起飞高度，省略时默认为 `1.0 m`。`arm` 会解锁、执行 PX4 原生 `AUTO.TAKEOFF`，在高度和垂直速度稳定后自动进入 OFFBOARD 悬停。

起飞过程会持续检查 MAVROS 状态、高度和定位。定位失效时请求 `AUTO.LAND`；其他
自主执行故障先确认 `AUTO.LOITER`，无法确认时再尝试 `AUTO.LAND`。状态本身失联
时无法确认恢复模式。

## 场景

场景配置位于 `simulation/config/scenes/NAME.env`，统一保存 Gazebo world 的容器内
路径和出生位姿。推荐通过 `--scene NAME` 选择；自动化脚本也可设置等价的
`SIM_SCENE=NAME`。

新增场景时添加配置文件，不要通过临时 `SIM_WORLD` 或 `SIM_SPAWN_*` 参数启动。启动器会先检查场景引用的 world 是否存在。

切换场景只改变 world 和出生位姿，不改变 Planner、控制器或飞行状态机。

## 单目标飞行

```text
./launch/sim.sh goal X Y Z [YAW_DEG]
```

例如：

```bash
./launch/sim.sh goal 1.0 0.0 1.5
./launch/sim.sh goal 1.0 1.0 1.5 90
```

目标规则：

- `X/Y/Z` 是 `world` 中的绝对坐标；
- 省略 yaw 时不限定终点朝向，提供时单位为度；
- 高度必须满足当前 Planner 围栏，并为 `obstacles_inflation` 留出余量；
- 飞机必须已解锁且处于 OFFBOARD；
- `arm` 完成前发布的 RViz 目标不会排队，进入 OFFBOARD 后需要重新发布。

CLI 会先检查飞行状态、定位、SE3 输出、Planner 和目标订阅者。命令成功只表示目标已经发布，不表示 Planner 一定接受或飞机已经到达。

目标没有距离硬限制，但 Diff-Planner 使用滚动局部地图。RViz 中的长距离曲线是局部规划参考线，不是基于全局地图得到的无碰撞路线。墙体、死胡同或未知区域可能使任意距离的目标失败；复杂路线应拆成经过确认的 Mission 航点。

RViz 的 `2D Nav Goal` 使用 `/sim2real/rviz_goal`，由目标桥转发到 `/goal`，避免与
MAVROS 使用的 `/move_base_simple/goal` 混淆。目标桥非锁存，并在每次点击时检查
有限数值、`world` 坐标系、armed/OFFBOARD、MAVROS 状态新鲜度、定位保护和当前
Planner 高度范围；不合格目标直接丢弃，不会排队。

## Mission

核对任务文件后执行；根目录的 `mission_*.json` 可作为格式参考：

```bash
./launch/sim.sh mission MISSION_FILE.json
```

关键行为：

- `waypoints` 按顺序执行，坐标是 `world` 绝对坐标；
- 未解锁时根据 `takeoff_height` 原生起飞并自动进入 OFFBOARD；已解锁且处于
  OFFBOARD 时直接执行；
- 省略航点 yaw 时按有效航段自动生成朝向；
- `fly_through=true` 允许连续通过中间航点，单个航点可覆盖；
- `land_after_mission=true` 时成功后请求 `AUTO.LAND`，否则在最终点保持 OFFBOARD；
- 切出 OFFBOARD、解除锁定或人工接管会中止任务，不会恢复旧目标；
- Planner 临时急停时会按 JSON 中的恢复超时和重试次数处理；
- 状态或定位失联、规划失败会中止任务。定位失效请求 `AUTO.LAND`，规划等其他
  任务故障请求 `AUTO.LOITER`；起飞或 OFFBOARD 交接失败时，无法确认
  `AUTO.LOITER` 才继续尝试 `AUTO.LAND`。

字段格式可参考根目录的 `mission_*.json`；其中坐标和高度属于具体场地，执行前必须
重新核对。

## 常用命令

| 命令 | 作用 |
|---|---|
| `./launch/sim.sh start` | 构建变更并启动完整仿真 |
| `./launch/sim.sh restart` | 尝试降落，停止后重新启动 |
| `./launch/sim.sh stop` | 尝试降落并停止仿真进程 |
| `./launch/sim.sh build` | 只构建 ROS overlay |
| `./launch/sim.sh test` | 构建并运行测试与 launch 校验 |
| `./launch/sim.sh status` | 查看容器、tmux、ROS 和飞行状态 |
| `./launch/sim.sh attach` | 打开 tmux 日志 |
| `./launch/sim.sh shell` | 进入仿真容器 |
| `./launch/sim.sh arm` | 原生起飞并自动进入 OFFBOARD |
| `./launch/sim.sh land` | 请求降落并等待解除锁定 |
| `./launch/sim.sh goal ...` | 发布一个目标 |
| `./launch/sim.sh mission FILE` | 执行 JSON 航点任务 |

tmux 中使用 `Ctrl-b n/p` 切换窗口，`Ctrl-b d` 退出查看。

## rosbag 与日志

启动栈时默认创建 `/flight_recorder`。rosbag 使用 LZ4、1 GiB 分卷和较低 CPU 优先级，并为当前一次运行保留最新 10 个分卷，约 10 GiB：

```text
runtime/simulation/flight_bags/se3_test_<run-id>_*.bag
```

默认记录定位、点云、规划、控制、MAVROS 状态、膨胀地图，以及仿真的 `/clock` 和 `/gazebo/model_states`。常用设置：

```bash
SIM_START_ROSBAG=false ./launch/sim.sh restart
SIM_ROSBAG_RECORD_RAW_LIDAR=false ./launch/sim.sh restart

SIM_ROSBAG_SPLIT_SIZE_MB=512 SIM_ROSBAG_MAX_SPLITS=8 \
./launch/sim.sh restart
```

`stop/restart` 会先向 rosbag 发送 `SIGINT`，默认最多等待 60 秒完成索引。容量上限
只作用于当前运行；历史 bag 不会自动删除。启动前空间已经低于保留阈值时，整个栈
会拒绝启动；录制过程中跌破阈值时，只有 rosbag 停止，规划和控制继续运行。

tmux 与 ROS 日志位于：

```text
runtime/simulation/runs/<run-id>/
```

## 无界面与 GPU

```bash
SIM_GAZEBO_GUI=false SIM_START_RVIZ=false ./launch/sim.sh restart
```

`SIM_GPU_MODE=auto` 会依次选择 NVIDIA、DRI 或软件渲染。需要强制软件渲染时：

```bash
./launch/sim.sh stop
SIM_GPU_MODE=none ./launch/sim_container.sh recreate
SIM_GPU_MODE=none ./launch/sim.sh start
```

后续重建或重启容器时继续传入相同的 `SIM_GPU_MODE`。

## 配置与开发

| 内容 | 配置来源 |
|---|---|
| 启动、起飞、界面和录包 | [`launch/sim.sh`](../launch/sim.sh) |
| 场景 world 与出生位姿 | [`simulation/config/scenes`](../simulation/config/scenes) |
| Planner、地图和优化器 | [`common/config/planner.yaml`](../common/config/planner.yaml) |
| 轨迹预瞄与 yaw | [`common/config/trajectory_server.yaml`](../common/config/trajectory_server.yaml) |
| 公共控制与安全开关 | [`common/config/controller.yaml`](../common/config/controller.yaml) |
| 仿真载体控制参数 | [`simulation/config/controller.yaml`](../simulation/config/controller.yaml) |
| 镜像资产与固定版本 | [`simulation/assets`](../simulation/assets)、[`simulation/versions.env`](../simulation/versions.env) |

控制器先加载公共配置，再由仿真载体配置覆盖同名值。载体配置只包含 SE3 控制和
安全参数；质量、惯量、电机、桨叶及 PX4 内环参数来自 Gazebo 模型和 PX4 airframe。
控制器细节见 [SE3 控制器](se3_controller.md)。

### Planner 高度与地图过滤

Planner 配置由仿真和真机共享。启用虚拟墙时，CLI、RViz 和 Mission 目标必须满足：

```text
virtual_ground + obstacles_inflation
  < goal.z <
virtual_ceil - obstacles_inflation
```

这些参数使用 `world` 坐标系，边界本身不允许。`grid_map/min_ray_length` 是传感器
原点到射线端点的最短地图更新距离，可过滤机体附近回波和零长度射线；它不是雷达
最小量程，也不限制最大感知距离。

长期修改应直接编辑 `common/config/planner.yaml`。临时测试完整配置时：

```bash
cp common/config/planner.yaml runtime/simulation/planner_override.yaml
# 编辑 runtime/simulation/planner_override.yaml
SIM_PLANNER_CONFIG=/root/simulation_runtime/planner_override.yaml \
./launch/sim.sh restart
```

`SIM_PLANNER_CONFIG` 使用容器内路径，不能填写只存在于宿主机的绝对路径。

### 修改后如何生效

| 修改内容 | 操作 |
|---|---|
| `common/`、`simulation/config/` 或仿真 ROS 源码 | `./launch/sim.sh test`，然后 `restart` |
| 场景 `.env` 或仓库内 world | `./launch/sim.sh restart` |
| `simulation/assets/`、`simulation/versions.env` 或 Dockerfile | 停止并重建镜像和容器 |

重建镜像：

```bash
./launch/sim.sh stop
./launch/sim_container.sh build
./launch/sim_container.sh recreate
./launch/sim.sh start
```

## 状态检查与排错

```bash
./launch/sim.sh status
./launch/sim.sh shell

rosparam get /drone_0_diff_planner_node/grid_map
rosparam get /drone_0_traj_server
rosparam get /se3_controller_node

rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic echo -n1 /mavros/state
rostopic hz /mavros/setpoint_raw/attitude
```

常见问题：

- 目标无响应：等待 `arm` 完成并确认 OFFBOARD，再发布新目标；
- Planner 急停：检查目标参考线、膨胀地图、当前点是否落入膨胀障碍和 Planner 日志；
- 定位保护触发：检查未启动、停更、重复/乱序/过期时间戳、跳变或异常速度；
  watchdog 使用系统单调时钟，`/clock` 冻结也会触发，故障锁存后需重启完整栈；
- Gazebo/RViz 无法显示：使用无界面模式确认核心链路；
- 容器或 GPU 配置过期：运行 `./launch/sim_container.sh verify`，必要时重建容器；
- 修改未生效：普通源码运行 `sim.sh restart`，镜像内容则重新 build 和 recreate。

镜像版本固定在 `simulation/versions.env`；构建完成后，日常启动不需要联网。
