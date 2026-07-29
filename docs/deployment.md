# Jetson 真机部署

真机链路为：

```text
MID-360 → Livox driver → FAST-LIO
                        ├─> /localization/odom
                        └─> /localization/cloud_registered
                                      │
                                      ▼
                              selected planner
                                      │
                                      ▼
                         planner gateway → SE3 → MAVROS → PX4
```

> `real.sh start/restart` 不会解锁，`stop/restart` 也不会降落。飞机仍解锁或活动栈的
> MAVROS 状态无法确认时，停止和重启会被拒绝。`--force` 仅用于飞手已经确保飞机
> 安全后的维护。

除远程 RViz 命令外，以下操作均在 Jetson 仓库根目录执行。

## 部署前配置

### MID-360 网络

编辑 [`deployment/config/livox/MID360s_config.json`](../deployment/config/livox/MID360s_config.json)：

- `Mid360s.host_net_info[0].host_ip`：Jetson 雷达网卡地址；
- `lidar_configs[0].ip`：雷达地址；
- `extrinsic_parameter` 保持全零，安装外参由 `MOUNT_*` 适配层处理。

Jetson 与雷达必须位于同一有线子网。

### 安装外参

`MOUNT_X`、`MOUNT_Y`、`MOUNT_Z`、`MOUNT_ROLL_DEG`、`MOUNT_PITCH_DEG` 和
`MOUNT_YAW_DEG` 定义 `T_base_fastlio_body`。这里的 `body` 是 MID-360 内置 IMU
原点，不是点云原点。默认值只适用于当前机体；更换安装位置后必须重新标定。

### PX4

首次飞行前确认：

- external-vision EKF 已配置，位置、方向和尺度正确；
- `MAV_SYS_ID`、飞控串口和 QGC 链路正确；
- RC 模式切换、Kill、RC loss、Offboard loss、电池和估计器 failsafe 已验证；
- 卸桨完成解锁、模式切换和控制方向测试。

### 控制器

在 [`deployment/config/controller.yaml`](../deployment/config/controller.yaml) 中重新验证：

- `hover_percent`；
- `ki_pz`、`int_limit_z`；
- `min_output_thrust`、`max_output_thrust`；
- `geo_fence`。

更换电机、桨、电池、重量或载荷后必须重新标定。

## 镜像与容器

```bash
# 构建 Jetson 镜像
./launch/real_container.sh build

# 创建并启动容器，挂载 PX4 串口
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh run

# 查看容器或进入容器
./launch/real_container.sh status
./launch/real_container.sh shell
```

容器挂载 `runtime/flight_bags`、`runtime/tmp`、Livox 配置和真机控制器配置。
替换容器前必须停止真机栈并确认飞机已落地、解除武装。

## 启动真机栈

`start` 和 `restart` 必须指定 `diff`、`fast-kino` 或 `fast-topo`。以下网络值仅为
示例：

```bash
FCU_URL='/dev/ttyACM0:921600' \
GCS_URL='udp://:14555@172.20.10.3:14550' \
ROS_IP=172.20.10.5 \
MAVROS_TGT_SYSTEM=5 \
./launch/real.sh --planner diff start
```

`MAVROS_TGT_SYSTEM` 必须等于 PX4 的 `MAV_SYS_ID`。非默认设备、网络和 `MOUNT_*`
需要在每次 `start/restart` 时继续传入。

```bash
./launch/real.sh status   # 容器、tmux、ROS、规划器和 MAVROS 状态
./launch/real.sh attach   # 查看实时日志；Ctrl-b d 退出
```

启动器按顺序启动 Livox、FAST-LIO、定位适配、MAVROS、定位保护、规划器、SE3 和
rosbag；任一关键阶段失败都会清理本次部分启动。

## 飞前检查

进入容器：

```bash
./launch/real_container.sh shell
```

进入容器后检查：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic hz /mavros/vision_pose/pose
rostopic echo -n1 /planning/status
rostopic echo -n1 /mavros/state
rosrun tf tf_echo world base_link
```

起飞前确认：

1. Livox 点云和 IMU 连续；
2. `/localization/odom` 时间戳持续前进，位置、速度、尺度和方向正确；
3. 注册点云位于 `world`，并与同一时刻的里程计和障碍对齐；
4. PX4 已连接且采用 external vision；
5. `/planning/status` 为 READY，`odom_ready`、`map_ready` 为 true；
6. 目标、Mission、规划器地图边界和控制器围栏符合现场；
7. 悬停推力、外参、failsafe 和遥控接管已经在当前机体上验证。

## 飞行命令

### 起飞

```bash
# 解锁，执行 PX4 AUTO.TAKEOFF，到达相对 Home 1.5 m 后进入 OFFBOARD
REAL_TAKEOFF_HEIGHT=1.5 ./launch/real.sh arm
```

默认起飞高度为 `1.0 m`。命令会检查 MAVROS、定位、SE3 和预热 setpoint；进入
OFFBOARD 后还会验证姿态/推力输出。起飞完成前发布的目标不会排队。

### 单目标

```bash
# 不约束终点 yaw
./launch/real.sh goal 1.0 0.0 1.5

# 终点 yaw 为 90°
./launch/real.sh goal 1.0 0.0 1.5 90
```

坐标为 `world` 绝对坐标，yaw 单位为度。命令先检查 armed/OFFBOARD、定位、SE3、
规划器状态和目标验证服务。Diff 在 rolling GridMap 中局部规划；Fast Kino/Topo
还会检查固定地图边界。发布成功不代表路径一定可达。

### Mission

```bash
./launch/real.sh mission MISSION_FILE.json
```

Mission 在起飞前校验全部航点；未解锁时自动起飞并进入 OFFBOARD，已在
armed/OFFBOARD 时直接执行。缺省 yaw 按有效航段生成，显式 `yaw` 覆盖自动值。
人工切换模式会终止任务且不抢回 OFFBOARD；定位故障请求 `AUTO.LAND`，其他任务
故障优先请求 `AUTO.LOITER`。

### 降落与停止

```bash
# 请求并确认 PX4 进入 AUTO.LAND，不强制解除武装
./launch/real.sh land

# 确认落地并解除武装后停止 ROS 栈
./launch/real.sh stop
```

`stop` 不会降落，也不停止真机容器。飞行中不得把 `stop --force` 当作应急操作。

## 远程 RViz

工作站需要一个可运行 ROS Noetic 与 RViz 的容器，默认名为 `ros_noetic`：

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
RVIZ_GOAL_Z=1.5 \
RVIZ_GOAL_FRAME=world \
./launch/real_rviz.sh
```

`2D Nav Goal` 的箭头方向作为终点 yaw，经 `/sim2real/rviz_goal` 转发到 `/goal`。
桥接器只在 armed/OFFBOARD、定位和规划器就绪且目标验证通过时发布。

## 参数与更新

| 内容 | 文件 |
|---|---|
| Livox 网络 | [`deployment/config/livox/MID360s_config.json`](../deployment/config/livox/MID360s_config.json) |
| 真机控制器 | [`deployment/config/controller.yaml`](../deployment/config/controller.yaml) |
| Diff | [`planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) |
| Fast Kino/Topo | [`planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml) |

Livox 与控制器配置通过 bind mount 加载，落地解除武装后执行
`./launch/real.sh --planner ID restart` 即可重载。ROS 源码、规划器配置、消息或
Dockerfile 需要重建镜像：

```bash
./launch/real.sh stop
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh restart
```

Diff 可用 `PLANNER_CONFIG=/root/tmp/FILE.yaml` 临时加载完整配置；该变量不适用于 Fast。

## 日志与 rosbag

| 内容 | 路径 |
|---|---|
| rosbag | `runtime/flight_bags/` |
| 容器 ROS 日志 | `runtime/flight_bags/ros_logs/<run-id>/` |
| 宿主 tmux 日志 | `~/uav-autonomy-aio_logs/<run-id>/` |

rosbag 默认使用 LZ4、1 GiB 分卷，当前运行最多保留 10 卷。历史运行不会自动删除。

```bash
# 不记录原始 Livox 数据
ROSBAG_RECORD_RAW_LIDAR=false ./launch/real.sh --planner diff start

# 完全关闭录包
START_ROSBAG=false ./launch/real.sh --planner diff start
```

### 离线回放

真机栈停止后执行：

```bash
./launch/real_bag.sh play runtime/flight_bags/FILE.bag
./launch/real_bag.sh status
./launch/real_bag.sh attach
./launch/real_bag.sh stop

# 工作站打开离线 RViz
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
./launch/real_bag_rviz.sh
```

省略 bag 路径时 `play` 选择最新完成文件。`BAG_RATE` 控制播放倍率，
`BAG_LOOP=true` 循环播放。

## 安全边界

- 飞手和遥控器始终拥有最终控制权；
- 软件围栏不能替代 PX4 failsafe；
- MAVROS 状态失联时软件无法确认恢复模式，必须人工接管；
- 定位故障会锁存并请求 `AUTO.LAND`，排除原因后需重启完整栈；
- 更换载荷、动力系统或雷达安装位置后必须重新标定。

控制器原理与调参见 [SE3 控制器](se3_controller.md)。
