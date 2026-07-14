# diff-planner-px4-deployment

这个仓库用于在 Jetson 真机上部署一套 ROS1 版无人机自主避障链路：MID-360/MID-360S 激光雷达提供点云和 IMU，FAST-LIO 输出里程计，Diff-Planner 生成局部避障轨迹，SE3 控制器通过 MAVROS 给 PX4 发送姿态和推力控制指令。

它不是通用无人机框架，而是把一套已经跑通的真机部署环境整理成可以迁移、可以复现的工程目录。

整体链路如下：

```text
Livox MID-360/MID-360S
  -> livox_ros_driver2
  -> FAST-LIO
  -> /Odometry
  -> odom_to_base.py
  -> /Odometry_base
  -> MAVROS vision pose + Diff-Planner
  -> trajectory_msg_converter.py
  -> SE3 controller
  -> /mavros/setpoint_raw/attitude
  -> PX4
```

真机安全策略：

- 代码默认不会自动解锁。
- 代码默认不会循环请求切换 OFFBOARD。
- 是否切 OFFBOARD 由飞手通过遥控器决定。
- 飞行过程中可以随时通过遥控器切回定点、姿态或其他模式接管。
- MID-360 倾斜安装外参在 FAST-LIO 后面单独转换，不直接改雷达驱动或 FAST-LIO 内部逻辑。

## 仓库内容

```text
docker/
  Dockerfile                         Jetson 上使用的 ROS Noetic 镜像
  docker_run_real.sh                 构建、启动、停止、重建容器

scripts/
  start_real_px4_mid360_fastlio.sh   Jetson 真机一键启动脚本
  start_jetson_ros1_rviz.sh          本机打开 RViz 的辅助脚本
  odom_to_base.py                    传感器里程计转换为 base_link 里程计
  odom_to_pose.py                    /Odometry_base 转 MAVROS vision pose
  rviz_goal_to_diff_planner.py       RViz 目标点桥接到 /goal

config/
  livox/MID360s_config.json          Livox 网络配置
  rviz/jetson_real_stack.rviz        RViz 可视化配置

local_pkgs/
  px4_realflight_tools               本仓库维护的小型 ROS 工具包

third_party/
  Livox-SDK2
  livox_ros_driver2
  FAST_LIO
  Diff-Planner-PX4
```

Docker 镜像内部保留两个 ROS 工作空间：

```text
/root/livox_ws/src/livox_ros_driver2
/root/catkin_ws/src/FAST_LIO
/root/catkin_ws/src/Diff-Planner-PX4
/root/catkin_ws/src/px4_realflight_tools
```

## 硬件和网络假设

当前配置默认假设：

- Jetson 作为机载计算机。
- 飞控通过 USB 连接 Jetson，设备通常是 `/dev/ttyACM0`。
- MID-360/MID-360S 通过网线连接 Jetson。
- Jetson 的雷达网卡 IP 是 `192.168.1.101`。
- 雷达 IP 是 `192.168.1.199`。
- QGC 所在电脑 IP 是 `10.0.30.196`。
- Jetson 在 ROS1 网络中的 IP 类似 `10.0.30.108`。

如果你的硬件或网络不同，优先检查这些位置：

- `config/livox/MID360s_config.json`
- `FCU_URL`
- `FCU_DEVICE`
- `GCS_URL`
- `ROS_IP` 或 `ROS_IP_TARGET`
- 本机 RViz 脚本中的 `JETSON_IP`

## 在 Jetson 上部署

先安装主机侧依赖：

```bash
sudo apt update
sudo apt install -y git docker.io tmux
sudo usermod -aG docker "$USER"
```

加入 docker 用户组后，需要退出登录再重新登录。

克隆仓库：

```bash
cd /home/jetson2
git clone https://github.com/dreamer198/diff-planner-px4-deployment.git
cd diff-planner-px4-deployment
```

构建镜像：

```bash
./docker/docker_run_real.sh build
```

创建或重建容器：

```bash
FCU_DEVICE=/dev/ttyACM0 \
./docker/docker_run_real.sh restart
```

默认容器名是 `ros_noetic_realflight`。

## 启动真机链路

在 Jetson 上执行：

```bash
cd ~/diff-planner-px4-deployment
FCU_URL='serial:///dev/ttyACM0:57600' \
GCS_URL='udp://:14550@10.0.30.196:14550' \
SE3_HOVER_PERCENT=0.50 \
SE3_MAX_FEEDFORWARD_ACC=1.2 \
DIFF_PLANNER_INFLATION_SIZE=0.2 \
DIFF_PLANNER_VIRTUAL_CEIL=1.5 \
DIFF_PLANNER_VIRTUAL_GROUND=0.1 \
./scripts/start_real_px4_mid360_fastlio.sh restart
```

`DIFF_PLANNER_VIRTUAL_CEIL` 是规划器虚拟天花板高度，超过该 z 值的空间会被规划器视为不可通行。

`SE3_HOVER_PERCENT=0.50` 是本机（机体/电池/桨调整后）实测真实悬停油门；`SE3_KI_PZ=0.30` 是竖直积分增益，消除随电压/载荷的缓慢掉高。整定方法见 [docs/ki_pz_tuning_guide.md](docs/ki_pz_tuning_guide.md)。

`SE3_GEOFENCE_Z` 只由 `se3_controller_node` 用于超限检测。默认 `SE3_AUTO_LAND_ON_GEOFENCE=false` 时，超限后只打印告警，不会限制规划器轨迹，也不会主动压低控制指令。如需让它触发 PX4 降落，需要显式设置 `SE3_AUTO_LAND_ON_GEOFENCE=true`，并建议让 `SE3_GEOFENCE_Z` 略高于 `DIFF_PLANNER_VIRTUAL_CEIL`。

常用命令：

```bash
./scripts/start_real_px4_mid360_fastlio.sh status
./scripts/start_real_px4_mid360_fastlio.sh attach
./scripts/start_real_px4_mid360_fastlio.sh stop
./scripts/start_real_px4_mid360_fastlio.sh restart
```

`attach` 会进入 tmux 会话。常用 tmux 操作：

- `Ctrl-b n`：切到下一个窗口
- `Ctrl-b p`：切到上一个窗口
- `Ctrl-b d`：退出 attach，不停止程序

一键启动脚本会按顺序启动：

1. `roscore`
2. `livox_ros_driver2`
3. `FAST-LIO`
4. `odom_to_base.py`
5. `mavros`
6. `odom_to_pose.py`
7. `Diff-Planner`
8. `trajectory_msg_converter.py`
9. `se3_controller_node`

## MID-360 安装外参

当前默认假设 MID-360：

- 比飞控高 `0.10 m`
- 相对机体系前倾 `30 deg`

可以通过环境变量修改：

```bash
MOUNT_X=0.0
MOUNT_Y=0.0
MOUNT_Z=0.10
MOUNT_ROLL_DEG=0.0
MOUNT_PITCH_DEG=30.0
MOUNT_YAW_DEG=0.0
```

外参转换发生在 FAST-LIO 后面：

```text
/Odometry -> odom_to_base.py -> /Odometry_base
```

FAST-LIO 和原始点云仍保持传感器坐标逻辑，下游规划器、控制器、MAVROS vision pose 使用转换后的 `/Odometry_base`。

## 在本机打开 RViz

Jetson 上运行的是容器里的 ROS1。如果本机主要是 ROS2，建议在本机也准备一个 ROS1 Docker 容器用于 RViz。

本机容器要求：

- 默认容器名是 `ros_noetic`。
- 容器内需要有 ROS Noetic 和 RViz。
- 容器最好使用 host 网络，否则 ROS1 跨机器通信容易失败。
- 容器需要能访问 X11 显示。

在本机执行：

```bash
/home/dreamer198/diff-planner-px4-deployment/scripts/start_jetson_ros1_rviz.sh
```

如需覆盖默认配置：

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=10.0.30.108 \
LOCAL_IP=10.0.30.196 \
/home/dreamer198/diff-planner-px4-deployment/scripts/start_jetson_ros1_rviz.sh
```

这个脚本会：

- 连接到 `ROS_MASTER_URI=http://JETSON_IP:11311`
- 发布 `world -> camera_init` 和 `world -> map` 静态 TF
- 把 RViz 的 `/move_base_simple/goal` 和 `/clicked_point` 桥接到 `/goal`
- 用 `config/rviz/jetson_real_stack.rviz` 打开 RViz

## 通过终端发送目标点

除了在 RViz 中点目标，也可以直接发布 `/goal`：

```bash
rostopic pub -1 /goal geometry_msgs/PoseStamped "header:
  frame_id: 'world'
pose:
  position:
    x: 3.4
    y: 0.8
    z: 1.0
  orientation:
    w: 1.0"
```

正常流程是：先让规划器生成轨迹，再由飞手通过遥控器切换到 OFFBOARD，随后无人机开始跟踪轨迹。

## 起飞前检查

建议至少检查：

- `/Odometry_base` 是否稳定，且机体前后左右上下移动方向正确。
- `/mavros/vision_pose/pose` 是否稳定。
- RViz 中点云、局部地图、障碍物膨胀和规划轨迹是否正常。
- OFFBOARD 前是否已经有有效轨迹。
- 遥控器是否可以随时切回其他模式接管。

这个仓库不替代飞控校准、PX4 参数设置、QGC 安全检查和试飞场地安全流程。

## 参考项目和论文

本仓库主要参考和集成了以下项目。

路径规划和轨迹生成：

- [Tfly6/Diff-Planner-PX4](https://github.com/Tfly6/Diff-Planner-PX4)
- [DifferentialRobotics/Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner)
- [ZJU-FAST-Lab/EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)
- [EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors](https://arxiv.org/abs/2008.08835)

激光惯性里程计：

- [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)
- [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://arxiv.org/abs/2107.06829)

Livox 雷达驱动：

- [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)

PX4 和 MAVROS：

- [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- [mavlink/mavros](https://github.com/mavlink/mavros)

SE3 控制：

- [HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller)
- [Geometric Tracking Control of a Quadrotor UAV on SE(3)](https://www.researchgate.net/publication/224220605_Geometric_Tracking_Control_of_a_Quadrotor_UAV_on_SE3)
- [Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)](https://arxiv.org/abs/1003.2005)


室外测试步骤
1、选点位
固定在某个位置和朝向上电，抱着无人机找出合适的目标点位，记录坐标值（x,y,z）
docker exec -it ros_noetic_realflight bash
rostopic list
rostopic echo ...

2、远程启动程序
ssh jetson2@10.251.142.1
FCU_URL='serial:///dev/ttyACM0:57600' GCS_URL='udp://:14550@10.251.142.172:14550' SE3_HOVER_PERCENT=0.5 SE3_MAX_FEEDFORWARD_ACC=1.2 DIFF_PLANNER_INFLATION_SIZE=0.3 DIFF_PLANNER_VIRTUAL_CEIL=1.0 DIFF_PLANNER_VIRTUAL_GROUND=0.1 ./scripts/start_real_px4_mid360_fastlio.sh restart

3、本机启动rviz
CONTAINER_NAME=ros_noetic JETSON_IP=10.251.142.1 LOCAL_IP=10.251.142.172 /home/dreamer198/diff-planner-px4-deployment/scripts/start_jetson_ros1_rviz.sh

<!-- 发点 -->
ssh jetson2@10.251.142.1
docker exec -it ros_noetic_realflight bash -lc 'source ~/.bashrc && rostopic pub -1 /goal geometry_msgs/PoseStamped "header:
  frame_id: world
pose:
  position:
    x: 7.84
    y: 17.45
    z: 0.75
  orientation:
    w: 1.0"'

docker exec -it ros_noetic_realflight bash -lc 'source ~/.bashrc && rostopic pub -1 /goal geometry_msgs/PoseStamped "header:
  frame_id: world
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.75
  orientation:
    w: 1.0"'