# Diff-Planner PX4 Sim-to-Real

本项目是一套基于 ROS1 Noetic 的无人机局部规划工程，包含 PX4 SITL 仿真和 Jetson 真机部署。仿真与真机使用同一套 Diff-Planner、轨迹转换、SE3 控制器和规划参数，只在传感器、定位、外参及机体标定层面分别适配。

## 系统结构

```text
仿真：PX4 SITL + Gazebo + 模拟 MID-360
真机：PX4 + MID-360 + FAST-LIO
                  │
                  ▼
       /localization/odom
       /localization/cloud_registered
                  │
                  ▼
       Diff-Planner → traj_server
                  │
                  ▼
       trajectory converter → SE3
                  │
                  ▼
               MAVROS → PX4
```

公共定位接口约定：

| Topic | 类型 | 坐标约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | pose 在 `world`，twist 在 `base_link`，`child_frame_id=base_link` |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 点云已转换到 `world` |

仿真直接适配 MAVROS odom 和模拟点云，不运行 FAST-LIO；真机使用 FAST-LIO，并把公共里程计回灌到 `/mavros/vision_pose/pose` 供 PX4 EKF 融合。

## 环境要求

所有命令均在仓库根目录执行。

- Docker，并且当前用户可以访问 Docker daemon；
- tmux；
- 图形模式需要 X11；
- 首次构建镜像需要联网下载系统依赖和固定版本的 PX4/MID360 仿真源码。

运行时产生的 overlay、日志和 rosbag 位于 `runtime/`，不属于源码并被 Git 忽略。

默认容器分工：仿真使用 `diff_planner_px4_sim`，Jetson 真机使用 `diff_planner_px4_real`；工作站上的 `ros_noetic` 只用于远程 RViz，不参与仿真或真机计算。

使用 Gazebo 或 RViz 图形界面前，在宿主机授权容器内的 root 用户访问 X11：

```bash
xhost +SI:localuser:root
```

## 快速运行仿真

首次启动会自动构建缺失的镜像和 ROS overlay：

```bash
./launch/sim.sh start
```

飞机启动后保持未解锁。推荐操作顺序：

```bash
./launch/sim.sh arm
./launch/sim.sh goal 1.0 0.0 1.0
./launch/sim.sh land
./launch/sim.sh stop
```

`arm` 会让 SITL 解锁并使用 PX4 原生 `AUTO.TAKEOFF` 上升到相对 Home `1.0 m`，到达后由脚本自动切入 OFFBOARD；SE3 锁定切换瞬间的位姿悬停。`goal` 格式为：

```text
./launch/sim.sh goal X Y Z [YAW_DEG]
```

- 坐标系固定为 `world`；
- 省略 yaw 时不限定终点朝向；
- 提供 yaw 时，单位为度；
- 默认 Planner 高度范围为 `0.1 < Z < 3.0 m`；
- 必须在 `arm` 完成并确认自动 OFFBOARD 交接后发布目标；此前的 RViz 目标不会排队。

仿真和真机的 CLI `goal` 都执行同一份单进程检查逻辑：并行确认飞行状态、新鲜定位、连续 SE3 输出、Planner 高度围栏和两个目标消费者，再由该进程直接发布 `/goal`，避免为每项检查重复启动容器和 ROS 命令。

无图形界面运行：

```bash
SIM_GAZEBO_GUI=false SIM_START_RVIZ=false ./launch/sim.sh restart
```

完整说明见 [仿真运行说明](docs/simulation.md)。

## 快速部署真机

> `real.sh start/restart` 不会自动切换飞行模式或解锁；显式执行 `real.sh arm` 会请求 PX4 解锁，使用原生 `AUTO.TAKEOFF` 起飞，并在达到实际高度、确认连续预热 setpoint 后自动进入 OFFBOARD。命令验证 SE3 姿态/推力输出后才返回。首次飞行前必须完成 PX4 external-vision EKF、雷达外参、控制器和 failsafe 配置，并由飞手保留随时接管能力。

先修改以下机体配置：

- `deployment/config/livox/MID360s_config.json`：Jetson 和雷达 IP；
- `deployment/config/controller.yaml`：悬停推力、积分、推力范围和围栏；
- `MOUNT_*` 环境变量：`base_link → MID-360 内置 IMU（FAST-LIO body）` 外参，不能直接填写量到点云原点的平移。

`MID360s_config.json` 中的重要参数：

- `Mid360s.host_net_info[0].host_ip`：Jetson 直连雷达网卡的静态 IP，当前为 `192.168.1.101`；
- `lidar_configs[0].ip`：MID-360S 的设备 IP，当前为 `192.168.1.199`；
- `lidar_configs[0].extrinsic_parameter`：保持全零；本项目通过 `MOUNT_*` 在 FAST-LIO 后统一处理安装外参，避免重复补偿。

两端 IP 必须处于同一子网。协议类型、UDP 端口、点云格式和扫描模式保持仓库默认值。

在 Jetson 上构建并创建容器：

```bash
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh run
```

启动完整 ROS 栈：

```bash
FCU_URL='/dev/ttyACM0:921600' \
GCS_URL='udp://:14555@172.20.10.3:14550' \
ROS_IP=172.20.10.5 \
MAVROS_TGT_SYSTEM=5 \
PLANNER_RESOLUTION=0.11 \
PLANNER_OBSTACLES_INFLATION=0.33 \
./launch/real.sh start
```

`MAVROS_TGT_SYSTEM` 必须与 PX4 的 `MAV_SYS_ID` 一致；当前真机实测为 `5`。
两个 Planner 环境变量与公共默认值一致：`0.11 m` 分辨率、`0.33 m` 膨胀半径。后者相对 `0.65 m` 正方形机体的 `0.325 m` 半对角线只保留 `0.005 m` 理论余量，并正好量化为 3 个体素；省略这两个变量时使用相同的公共默认值。

查看状态或日志：

```bash
./launch/real.sh status
./launch/real.sh attach
```

在同网段工作站的 ROS Noetic GUI 容器中打开 RViz：

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
RVIZ_GOAL_Z=1.0 \
RVIZ_GOAL_FRAME=world \
./launch/real_rviz.sh
```

真机详细配置和飞前检查见 [真机部署说明](docs/deployment.md)。

## 目录结构

| 目录 | 内容 |
|---|---|
| `common/` | 仿真和真机共享的 ROS launch、参数、目标桥及完整 Mission 状态机 |
| `simulation/` | 仿真镜像、PX4/Gazebo 资产、车辆标定和适配节点 |
| `deployment/` | 真机镜像、Livox/FAST-LIO 适配、外参和机体标定 |
| `launch/` | 宿主机上的容器和完整系统入口 |
| `third_party/` | Diff-Planner、SE3、FAST-LIO、Livox SDK/驱动源码 |
| `docs/` | 仿真、真机部署、算法、控制器和参数说明 |
| `runtime/` | 构建缓存、运行日志和 rosbag，自动生成 |

根目录 `launch/` 只负责系统编排；ROS XML launch 位于各自 ROS package 内。

## 配置归属

| 文件 | 用途 |
|---|---|
| `common/config/planner.yaml` | 两端唯一的 Planner 和地图默认参数 |
| `common/config/trajectory_server.yaml` | 轨迹采样和 yaw 参数 |
| `common/config/controller.yaml` | 两端共享的控制器安全默认值 |
| `simulation/config/controller.yaml` | 仿真 Iris 机体标定 |
| `deployment/config/controller.yaml` | 真机标定和围栏 |
| `deployment/config/livox/MID360s_config.json` | 真机 Livox 网络配置 |
| `simulation/versions.env` | 仿真 PX4 和 MID360 插件固定版本 |

规划参数只维护一份公共配置，不应在 `simulation/` 和 `deployment/` 中复制。

## 常用命令

| 命令 | 作用 |
|---|---|
| `./launch/sim.sh start/restart/stop` | 启动、重启或停止仿真 |
| `./launch/sim.sh build/test` | 构建 overlay 或运行测试 |
| `./launch/sim.sh status/attach/shell` | 查看状态、日志或进入容器 |
| `./launch/sim.sh arm/land/goal .../mission FILE` | SITL 单目标或顺序航点飞行操作 |
| `./launch/sim_container.sh ...` | 单独管理仿真镜像和容器 |
| `./launch/real_container.sh ...` | 管理真机镜像和容器 |
| `./launch/real.sh start/restart/stop` | 管理真机 ROS/tmux 栈 |
| `./launch/real.sh status/attach` | 查看真机运行状态和日志 |
| `./launch/real.sh arm/land/goal .../mission FILE` | 真机原生起飞、单目标或顺序航点及自动降落操作，带定位、Planner 确认和遥控器接管门禁 |
| `./launch/real_rviz.sh` | 从工作站连接 Jetson 并打开 RViz |
| `./launch/real_bag.sh ...` | 在 Jetson 安全回放真机 bag，不启动飞控链路 |
| `./launch/real_bag_rviz.sh` | 使用离线专用 RViz 查看原始扫描、累计场景地图和轨迹 |

也可以使用统一转发入口，例如：

```bash
./launch/stack.sh sim restart
./launch/stack.sh real status
```

## 测试与日志

运行自动构建、单元测试和 launch 校验：

```bash
./launch/sim.sh test
```

常用日志位置：

- 仿真：`runtime/simulation/runs/<run-id>/`；
- 真机 rosbag 和容器 ROS 日志：`runtime/flight_bags/`；
- 真机宿主 tmux 日志：`~/diff-planner-px4-deployment_logs/<run-id>/`。

真机默认调试包包含定位、控制与规划轨迹、Livox 原始点云、FAST-LIO 较高密度去畸变点云、规划器输入点云和 2 Hz 膨胀地图，并使用 LZ4 和约 5 GB 分卷。原始 `CustomMsg` 在离线回放时转换为等密度 `PointCloud2` 供 RViz 显示。具体选项见 [真机部署说明](docs/deployment.md#日志与-rosbag)。

## 安全说明

- 飞手和遥控器始终拥有最终控制权；`real.sh arm` 会请求 PX4 解锁和原生 `AUTO.TAKEOFF`，执行前必须解除 Kill 并确认场地净空；
- `real.sh arm` 成功返回表示原生起飞、自动 OFFBOARD 和 SE3 输出验证均已完成；飞手仍须确认定位、点云和 PX4 EKF 正常，并保留随时切回人工模式的能力；
- OFFBOARD 前发布的目标不会排队，必须在 `/mavros/state` 确认 OFFBOARD 后重新发布；
- SE3 围栏不能替代 PX4 的 RC、Offboard、电池和估计器 failsafe；
- `hover_percent`、`ki_pz`、推力限制和雷达外参必须针对实际机体重新验证；
- 建议先卸桨完成接口和方向检查，再进行受控低空测试。

## 进一步阅读

- [仿真运行说明](docs/simulation.md)
- [真机部署说明](docs/deployment.md)
- [公共 Sim-to-Real 接口](common/README.md)
- [仿真参数说明](docs/simulation_parameters.md)
- [Diff-Planner 原理](docs/diff_planner_principles.md)
- [SE3 控制器](docs/se3_controller.md)
- [竖直积分增益整定](docs/ki_pz_tuning_guide.md)
- [轨迹跟踪掉高排查](docs/trajectory_tracking_altitude.md)
