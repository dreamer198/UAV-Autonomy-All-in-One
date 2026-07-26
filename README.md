# UAV Autonomy All-in-One

`UAV Autonomy All-in-One` 是基于 ROS1 Noetic、PX4 和 Gazebo Classic 的一体化
无人机自主飞行平台，覆盖仿真、传感器与定位适配、规划、轨迹控制、任务执行和
Jetson 真机部署。当前默认集成 Diff-Planner，后续规划算法可复用公共定位、目标、
轨迹和控制接口。

```text
仿真：PX4 SITL + Gazebo + 模拟 MID-360
真机：PX4 + MID-360 + FAST-LIO
                  │
                  ▼
       /localization/odom
       /localization/cloud_registered
                  │
                  ▼
  Planner（默认 Diff-Planner）→ traj_server → SE3 → MAVROS → PX4
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
REAL_TAKEOFF_HEIGHT=1.5 ./launch/real.sh arm
./launch/real.sh goal 2.0 2.0 1.5
./launch/real.sh land
```

`arm` 使用 PX4 原生起飞并自动交接到 OFFBOARD。`stop/restart` 不会请求降落，
飞机仍解锁或 MAVROS 状态无法确认时会被拒绝。

## 目标与 Mission

```text
./launch/sim.sh goal X Y Z [YAW_DEG]
./launch/real.sh goal X Y Z [YAW_DEG]
```

坐标系为 `world`，yaw 单位为度。目标没有距离硬限制，但 Diff-Planner 使用滚动
局部地图，长距离参考线不是全局无碰撞路线。复杂路线应使用经过确认的 Mission 航点：

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
| `simulation/` | 仿真镜像、场景、模型、适配节点和仿真控制参数 |
| `deployment/` | 真机镜像、Livox/FAST-LIO 适配、外参和真机控制参数 |
| `launch/` | 宿主机入口脚本 |
| `third_party/` | Diff-Planner、SE3、FAST-LIO 和 Livox 源码 |
| `docs/` | 操作、算法与调参文档 |
| `runtime/` | 自动生成的构建缓存、日志和 rosbag |

公共接口和参数归属见 [`common/README.md`](common/README.md)。

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
| [控制器调参与掉高排查](docs/controller_tuning.md) | 悬停推力、竖直积分和高度问题 |
| [Diff-Planner 原理](docs/diff_planner_principles.md) | 局部地图、规划流程与能力边界 |
| [SE3 控制器](docs/se3_controller.md) | 轨迹到姿态、推力的控制链路 |
| [室外场景重建](docs/outdoor_bag_gazebo.md) | 从室外 rosbag 生成 Gazebo 场景 |
| [仿真资产](simulation/assets/README.md) | 仿真上游源码及固定版本 |

## 致谢

本项目基于并集成了以下开源项目，感谢其作者和维护者：

- 规划与控制：[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)、
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
