# UAV Autonomy All-in-One

`UAV Autonomy All-in-One` 是基于 ROS1 Noetic、PX4 和 Gazebo Classic 的一体化
无人机自主飞行平台，覆盖仿真、传感器与定位适配、多规划器、轨迹控制、任务执行和
Jetson 真机部署。当前提供 Diff-Planner、Fast-Planner Kinodynamic 和
Fast-Planner Topological 三个可启动时选择的规划插件，默认使用 `diff`。

```text
仿真：PX4 SITL + Gazebo + 模拟 MID-360
真机：PX4 + MID-360 + FAST-LIO
                  │
                  ▼
       /localization/odom
       /localization/cloud_registered
                  │
                  ▼
  planner_gateway → 选中的规划器插件 → SE3 → MAVROS → PX4
```

所有命令均在仓库根目录执行。

## 环境要求

- Docker，且当前用户可以访问 Docker daemon；
- tmux；
- 图形模式需要 X11；首次使用可执行 `xhost +SI:localuser:root`；
- 首次构建镜像需要联网。

运行时文件位于 `runtime/`；真机宿主 tmux 日志默认位于
`~/uav-autonomy-aio_logs/`。这些内容不属于源码。

## 仿真快速开始

```bash
./launch/sim.sh start
SIM_TAKEOFF_HEIGHT=1.5 ./launch/sim.sh arm
./launch/sim.sh goal 2.0 2.0 1.5
./launch/sim.sh land
./launch/sim.sh stop
```

`arm` 使用 PX4 原生 `AUTO.TAKEOFF` 起飞，稳定后自动切入 OFFBOARD。`SIM_TAKEOFF_HEIGHT` 是相对 PX4 Home 的起飞高度，省略时默认为 `1.0 m`。

启动时切换规划器：

```bash
./launch/sim.sh --planner diff start
./launch/sim.sh --planner fast-kino start
./launch/sim.sh --planner fast-topo start
```

不需要先执行 `./launch/sim.sh planners`。该命令只是可选的诊断工具，用于检查
manifest 扫描结果、插件 workspace 是否已构建，以及插件允许的仿真/真机模式和默认
profile；固定使用三个内置规划器时可以直接通过 `--planner` 启动。真机侧对应命令为
`./launch/real.sh planners`。

规划器只能在完整栈停止后切换，不支持空中热切换或自动 fallback。Fast Kino/Topo
共用唯一的固定地图配置，不再区分 `local`/`outdoor`，所以三个内置规划器启动时都
不需要 `--planner-profile`。Fast 地图名义范围为 `30 × 30 × 5 m`，原点为
`(-15, -15, -1)`，分辨率为 `0.1 m`；计入 `0.1 m` 障碍膨胀后，可接受目标的
x/y 范围为 `[-14.9, 14.9] m`。森林场景不会再自动替换 Fast 地图配置。Fast 的到达
状态以真实里程计收敛为准，不再只依据轨迹播放时间。Fast adapter 还会在其私有地图
输入中补充仿真场景的稠密地面，避免单帧 MID-360 稀疏点云让规划器从墙体下方穿出。
Fast 的 `manager/max_vel` 和 `manager/max_acc` 是轨迹优化参数，不是公共接口的硬
拒绝阈值；adapter 会拒绝 NaN/Inf 等非法输出，但不会因为有限轨迹暂时超过这两个
名义值而撤销整个目标。

仿真 RViz 对三个规划器使用相同的显示布局。默认环境层
`/planning/viz/environment` 只由公共 world 点云生成：它过滤 MID-360 的无回波端点、
隐藏地面并持续累积已重复观测的障碍，因此靠近墙体后已显示的场景不会随 Fast 的
单帧局部地图一起消失。该层仅用于显示，不会回灌或修改任一规划器的私有地图；
原始激光、归一化占据障碍和安全膨胀层默认打开。插件只发布 raw/backend 调试数据，
公共 `planner_visualization` 统一过滤显示：Fast 的私有虚拟地面仍参与碰撞检查，但
不会遮挡 RViz；膨胀层使用高不透明度高度着色，青色线框表示插件声明的固定地图
边界。算法原生轨迹和搜索图默认关闭，公共紫色线只根据网关已接受的控制指令生成。
橙色箭头表示 RViz 输入目标，绿色球表示 Planner 当前实际目标，青色轨迹表示实测
飞行路径。

切换内置场景：

```bash
./launch/sim.sh --scene outdoor_rectangular_forest restart
```

完整命令、Mission、场景、日志和排错见[仿真运行指南](docs/simulation.md)。

## 真机入口

> 真机操作前必须验证定位、外参、控制参数、遥控接管和 PX4 failsafe。设备、网络和
> system ID 没有通用值，应按照[真机部署指南](docs/deployment.md)完成配置和飞前检查。

先构建并创建容器：

```bash
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh run
```

按部署指南启动完整栈并完成检查后，飞行入口为：

```bash
./launch/real.sh --planner diff start
REAL_TAKEOFF_HEIGHT=1.5 ./launch/real.sh arm
./launch/real.sh goal 2.0 2.0 1.5
./launch/real.sh land
```

`arm` 使用 PX4 原生起飞并自动交接到 OFFBOARD。`stop/restart` 不会请求降落，
飞机仍解锁或 MAVROS 状态无法确认时会被拒绝。

Fast Kino/Topo 当前 manifest 的 `real_flight` 为 `false`，真机入口会拒绝启动；
只有完成各自独立的真机验收后才能放行。

## 目标与 Mission

```text
./launch/sim.sh goal X Y Z [YAW_DEG]
./launch/real.sh goal X Y Z [YAW_DEG]
```

坐标系为 `world`，yaw 单位为度。目标会先交给当前插件验证：Diff 使用滚动局部
地图；Fast 只检查统一固定地图的边界及障碍膨胀，不会用局部更新窗口额外缩小目标
范围。复杂路线应使用经过确认的 Mission
航点，Mission 会在起飞前逐点调用当前插件的目标验证服务：

```bash
./launch/sim.sh mission MISSION_FILE.json
# 或
./launch/real.sh mission MISSION_FILE.json
```

完整行为和安全恢复分别以[仿真指南](docs/simulation.md)和
[真机指南](docs/deployment.md)为准。根目录的 `mission_*.json` 是场地任务，使用前
必须重新核对坐标、净空和高度。

## 目录

| 目录 | 内容 |
|---|---|
| `common/` | 两端共享的 ROS launch、参数和飞行命令执行器 |
| `planning/` | 公共规划消息、manager/gateway、Diff/Fast adapters、插件 manifest 和隔离 workspace 构建脚本 |
| `simulation/` | 仿真镜像、场景、模型、适配节点和仿真控制参数 |
| `deployment/` | 真机镜像、Livox/FAST-LIO 适配、外参和真机控制参数 |
| `launch/` | 宿主机入口脚本 |
| `third_party/` | Diff-Planner、Fast-Planner、SE3、FAST-LIO 和 Livox 源码 |
| `docs/` | 操作、算法与调参文档 |
| `runtime/` | 自动生成的构建缓存、日志和 rosbag |

公共飞行接口和参数归属见 [`common/README.md`](common/README.md)，插件 API、
workspace 隔离和扩展方法见 [`planning/README.md`](planning/README.md)。

## 测试与日志

```bash
./launch/sim.sh test
```

仿真和真机默认自动录制调试 rosbag，使用 LZ4、1 GiB 分卷，并为当前一次运行最多保留 10 个分卷，约 10 GiB。历史运行不会自动删除。

- 仿真日志：`runtime/simulation/runs/<run-id>/`；
- 仿真 bag：`runtime/simulation/flight_bags/`；
- 真机 bag 与容器 ROS 日志：`runtime/flight_bags/`；
- 真机宿主日志：`~/uav-autonomy-aio_logs/<run-id>/`。

## 进一步阅读

| 文档 | 内容 |
|---|---|
| [仿真运行指南](docs/simulation.md) | 启动、场景、目标、Mission、日志和排错 |
| [真机部署指南](docs/deployment.md) | Jetson、传感器、飞控配置与操作流程 |
| [公共自主飞行接口](common/README.md) | 仿真与真机共享的话题、坐标系和 ROS 入口 |
| [多规划器插件框架](planning/README.md) | 插件选择、公共消息、隔离 workspace、manifest 和扩展方法 |
| [控制器调参与掉高排查](docs/controller_tuning.md) | 悬停推力、竖直积分和高度问题 |
| [Diff-Planner 原理](docs/diff_planner_principles.md) | 局部地图、规划流程与能力边界 |
| [SE3 控制器](docs/se3_controller.md) | 轨迹到姿态、推力的控制链路 |
| [室外场景重建](docs/outdoor_bag_gazebo.md) | 从室外 rosbag 生成 Gazebo 场景 |
| [仿真资产](simulation/assets/README.md) | 仿真上游源码及固定版本 |

## 致谢

本项目基于并集成了以下开源项目，感谢其作者和维护者：

- 规划与控制：[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)、
  [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)、
  [Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner)、
  [Diff-Planner-PX4](https://github.com/Tfly6/Diff-Planner-PX4) 和
  [SE3 Controller](https://github.com/HITSZ-MAS/se3_controller)；
- 飞控与仿真：[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)、
  [Gazebo Classic](https://github.com/gazebosim/gazebo-classic)、
  [MAVROS](https://github.com/mavlink/mavros) 和 [ROS](https://www.ros.org/)；
- 定位与传感器：[FAST-LIO](https://github.com/hku-mars/FAST_LIO)、
  [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2) 和
  [Mid360 PX4 Simulation Plugin](https://github.com/Tfly6/Mid360_px4_sim_plugin)。

各上游项目的版权与许可证归其原作者所有；使用和分发时请遵守对应许可证。
