# Jetson 真机部署

真机链路为：

```text
MID-360 → Livox driver → FAST-LIO
                        ├─> /localization/odom ─────────────┬─> selected planner
                        │                                  └─> MAVROS vision pose → PX4
                        └─> /localization/cloud_registered ───> selected planner

selected planner → planner gateway → SE3 → MAVROS attitude setpoint → PX4
```

> `real.sh start/restart` 不会解锁，`stop/restart` 也不会降落。飞机仍解锁或活动栈的
> MAVROS 状态无法确认时，停止和重启会被拒绝。`--force` 仅用于飞手已经人工确认
> 飞机落地、解除武装后的故障恢复或维护。

除 `real_rviz.sh` 和 `real_bag_rviz.sh` 外，以下操作均在 Jetson 仓库根目录执行。

## 部署前配置

### MID-360 网络

编辑 [`deployment/config/livox/MID360s_config.json`](../deployment/config/livox/MID360s_config.json)：

- `Mid360s.host_net_info[0].host_ip`：Jetson 雷达网卡地址；
- `lidar_configs[0].ip`：雷达地址；
- `extrinsic_parameter` 保持全零，安装外参由 `MOUNT_*` 适配层处理。

Jetson 与雷达必须位于同一有线子网。

### 安装外参

`MOUNT_X`、`MOUNT_Y`、`MOUNT_Z`、`MOUNT_ROLL_DEG`、`MOUNT_PITCH_DEG` 和
`MOUNT_YAW_DEG` 定义 `T_base_fastlio_body`，即 MID-360 内置 IMU 坐标系到
`base_link` 的变换。平移量是 IMU 原点在 `base_link` 中的位置，单位为米；姿态角
表示 IMU 坐标轴在 `base_link` 中的姿态，按 `Rz(yaw) Ry(pitch) Rx(roll)` 组合，
单位为度。`base_link` 使用 x 向前、y 向左、z 向上的右手坐标系。

参数默认值位于 [`launch/real.sh`](../launch/real.sh) 顶部的六个 `MOUNT_*` 变量。
标定阶段可在当前终端临时覆盖，再执行后文的 `start/restart` 命令：

```bash
export MOUNT_X=0.109
export MOUNT_Y=0.024
export MOUNT_Z=0.006
export MOUNT_ROLL_DEG=0.7
export MOUNT_PITCH_DEG=28.1
export MOUNT_YAW_DEG=0.5
```

将示例值替换为实际标定结果；确认无误后写回 `launch/real.sh` 中对应变量的默认值。
修改外参不需要重建镜像，但每次 `start/restart` 都必须使用同一组数值。
[`MID360s_config.json`](../deployment/config/livox/MID360s_config.json) 中的
`extrinsic_parameter` 继续保持全零，避免重复应用外参。

简要标定方法：

1. 根据 MID-360 机械图和机架基准，测量从 `base_link` 原点到内置 IMU 原点的
   前、左、上偏移，作为 `MOUNT_X/Y/Z`；
2. 根据安装面或角度仪测量 IMU 坐标轴相对 `base_link` 的 roll、pitch、yaw，
   按右手定则确定正负号；
3. 卸桨启动真机栈，静置并手动沿机体前、左、上方向移动，再分别绕三轴转动。在
   RViz 中同时观察机体坐标轴和 `/localization/cloud_registered`，并用
   `rosrun tf tf_echo world base_link` 检查数值：运动方向应与实际一致，固定环境
   点云应保持稳定，绕 `base_link` 原点转动时不应出现与杆臂方向相关的系统性位移；
4. 用 rosbag 复核后固化数值。若需要更高精度，再使用动作捕捉或其他外部位姿做
   手眼标定。

这里的 `body` 不是点云原点；默认值只适用于当前机体，更换雷达安装位置后必须重新
标定。

### PX4

首次飞行前确认：

- external-vision EKF 已配置，位置、方向和尺度正确；
- `MAV_SYS_ID`、飞控串口和 QGC 链路正确；
- RC 模式切换、Kill、RC loss、Offboard loss、电池和估计器 failsafe 已验证；
- 卸桨完成解锁、模式切换和控制方向测试。

### 控制器

[`deployment/config/controller.yaml`](../deployment/config/controller.yaml) 保存真机载体参数，
其中的当前值只适用于已经标定的机体，不能直接照搬到其他动力系统或载荷。
控制器按“公共配置 → 当前规划器的 `controller_config` → 真机载体配置”的顺序加载，
后加载的同名参数覆盖前者；因此表中仅列真机载体层需要重点复核的参数。

| 参数 | 作用与确认方法 |
|---|---|
| `hover_percent` | 机体抵消重力所需的归一化推力。先从稳定悬停日志标定，再在低高度 OFFBOARD 悬停中确认不会持续上升或下沉。 |
| `min_output_thrust`、`max_output_thrust` | SE3 允许输出的推力范围。必须覆盖悬停推力并保留控制余量；若飞行中长期触及上限，应先检查载荷、动力和电池。 |
| `ki_pz`、`int_limit_z` | 消除高度稳态误差的积分增益及累计误差上限。只有悬停推力正确、定位稳定且推力未饱和后才能调整。 |
| `geo_fence.x/y/z` | 基于 `/mavros/local_position/odom` 的边界：x/y 是相对 PX4 本地原点的正负对称边界，z 只有上限。应按现场净空设置；当前 `auto_land_on_geofence=false`，越界只告警，不能替代 PX4 geofence 和 failsafe。 |

按 `hover_percent`、推力范围、高度积分、围栏的顺序确认，具体方法见
[SE3 控制器调参顺序](se3_controller.md#调参顺序)。更换电机、桨、电池、机体重量或
载荷后，至少重新标定悬停推力并复核推力范围和高度积分。

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

所有规划器默认共用镜像 `uav_autonomy_real:latest` 和运行容器
`uav_autonomy_real`；如需使用自定义容器，可通过 `CONTAINER_NAME=自定义名称`
覆盖，并在 `real_container.sh`、`real.sh` 和 `real_bag.sh` 中始终使用同一名称。

容器挂载 `runtime/flight_bags`、`runtime/tmp`、Livox 配置和真机控制器配置。
替换容器前必须停止真机栈并确认飞机已落地、解除武装。

`FCU_DEVICE` 决定创建容器时映射哪个宿主串口；后文的 `FCU_URL` 决定 MAVROS
连接的容器内串口和波特率。使用非默认串口时，两者的设备路径必须一致。

## 启动真机栈

`start` 和 `restart` 必须指定 `diff`、`fast-kino`、`fast-topo` 或 `super`。
以下 IP 地址仅为示例，需根据 Jetson、QGC 工作站和实际网络调整：

```bash
FCU_URL='/dev/ttyACM0:921600' \
GCS_URL='udp://:14555@172.20.10.3:14550' \
ROS_IP=172.20.10.5 \
MAVROS_TGT_SYSTEM=5 \
./launch/real.sh --planner diff start
```

`MAVROS_TGT_SYSTEM` 必须等于 PX4 的 `MAV_SYS_ID`。`ROS_IP` 应是其他 ROS 主机
可访问的 Jetson 地址；省略时脚本会按路由自动探测，但远程 RViz 场景建议显式设置。
`FCU_URL`、`GCS_URL`、`ROS_IP`、`MAVROS_TGT_SYSTEM` 和 `MOUNT_*` 的临时覆盖值
需要在每次 `start/restart` 时继续传入，或预先 `export` 到当前终端。

```bash
./launch/real.sh status   # 容器、tmux、ROS、规划器和 MAVROS 状态
./launch/real.sh attach   # 查看实时日志；Ctrl-b d 退出
```

启动器按顺序启动坐标系别名、Livox、FAST-LIO、里程计/点云适配、MAVROS、
external-vision 桥接、定位保护、所选规划器、SE3 和 rosbag；任一关键阶段失败都会
清理本次部分启动。

## 飞前检查

进入容器：

```bash
./launch/real_container.sh shell
```

进入容器后逐条检查；`rostopic hz` 和 `tf_echo` 需观察数秒，再按 `Ctrl-C` 进入下一项：

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
OFFBOARD 后还会验证姿态/推力输出。它会把 PX4 的 `MIS_TAKEOFF_ALT` 设为目标高度、
设置 `COM_TAKEOFF_ACT=0`，并临时收紧 `NAV_MC_ALT_RAD`；交接后只恢复
`NAV_MC_ALT_RAD`。真机不会改写已保存的 `MPC_THR_HOVER`。起飞完成前发布的目标
不会排队。

### 单目标

```bash
# 不约束终点 yaw
./launch/real.sh goal 1.0 0.0 1.5

# 终点 yaw 为 90°
./launch/real.sh goal 1.0 0.0 1.5 90
```

坐标为 `world` 绝对坐标，yaw 单位为度。命令先检查 armed/OFFBOARD、定位、SE3、
规划器状态和目标验证服务。Diff 在 rolling GridMap 中局部规划，SUPER 在 rolling
ROGMap 中局部规划；Fast Kino/Topo 还会检查固定地图边界。发布成功不代表路径一定
可达。

### Mission

```bash
./launch/real.sh mission MISSION_FILE.json
```

Mission 文件格式与[仿真 Mission](simulation.md#mission)相同。任务会在解锁前调用
所选规划器校验全部航点；滚动局部地图只延后“目标暂时超出当前局部地图”这一范围
判定，并在每个航点下发前重新校验。未解锁时自动起飞并进入 OFFBOARD，已经
armed/OFFBOARD 时直接执行。缺省 yaw 按有效航段生成，显式 `yaw` 覆盖自动值。

`land_after_mission=true` 时，完成后请求 `AUTO.LAND` 并等待 PX4 自动解除武装；
设为 `false` 时保持最终点和 OFFBOARD。人工切换模式会终止任务且不抢回 OFFBOARD；
定位故障请求 `AUTO.LAND`，其他任务故障优先请求 `AUTO.LOITER`，无法确认 LOITER
时再尝试 `AUTO.LAND`。

### 降落与停止

```bash
# 请求并确认 PX4 进入 AUTO.LAND，不强制解除武装
./launch/real.sh land

# 确认落地并解除武装后停止 ROS 栈
./launch/real.sh stop
```

`stop` 不会降落，也不停止真机容器。飞行中不得把 `stop --force` 当作应急操作。

## 远程 RViz

工作站需要一个可运行 ROS Noetic 与 RViz 的容器，默认名为 `ros_noetic`。以下 IP
地址仅为示例，需根据实际情况调整：

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
RVIZ_GOAL_Z=1.5 \
RVIZ_GOAL_FRAME=world \
./launch/real_rviz.sh
```

`JETSON_IP` 是工作站可访问的 Jetson 地址；`LOCAL_IP` 是工作站在该链路上的地址，
省略后者时脚本会按到 Jetson 的路由自动探测。`RVIZ_GOAL_Z` 是所有 `2D Nav Goal`
使用的固定 world 高度，必须按现场和当前规划器边界设置。

`2D Nav Goal` 的箭头方向作为终点 yaw，经 `/sim2real/rviz_goal` 转发到 `/goal`。
桥接器只在 armed/OFFBOARD、定位和规划器就绪且目标验证通过时发布。

## 参数设置

| 内容 | 文件 |
|---|---|
| Livox 网络 | [`deployment/config/livox/MID360s_config.json`](../deployment/config/livox/MID360s_config.json) |
| 控制器公共接口与安全参数 | [`common/config/controller.yaml`](../common/config/controller.yaml) |
| 规划器控制器覆盖 | 各插件 manifest 的 `controller_config`；当前 SUPER 使用 [`planning/plugins/super/controller.yaml`](../planning/plugins/super/controller.yaml) |
| 真机载体参数 | [`deployment/config/controller.yaml`](../deployment/config/controller.yaml) |
| Diff | [`planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) |
| Fast Kino/Topo | [`planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml) |
| SUPER | [`planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml) |

Livox 与真机载体配置通过 bind mount 加载，落地解除武装后执行
`./launch/real.sh --planner ID restart` 即可重载。修改镜像内构建或启动的 ROS 包、
规划器默认配置、消息、Dockerfile，以及公共或规划器控制器配置后，需要重建镜像：

```bash
./launch/real.sh stop
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh restart
```

`real_container.sh restart` 只重建运行容器，不会启动 ROS 栈；完成后需按
[启动真机栈](#启动真机栈)重新传入实际设备、网络、外参和规划器参数执行 `start`。

所有规划器都可用 `PLANNER_CONFIG=/root/tmp/FILE.yaml` 临时加载当前插件的一份
完整配置。先将文件放到宿主机 `runtime/tmp/FILE.yaml`；Diff、Fast Kino/Topo 与
SUPER 的配置格式分别以表中的默认文件为准。这里必须传容器内路径，不能传
`runtime/tmp/FILE.yaml`：

```bash
PLANNER_CONFIG=/root/tmp/FILE.yaml \
./launch/real.sh --planner fast-kino start
```

## 日志与 rosbag

| 内容 | 路径 |
|---|---|
| rosbag | `runtime/flight_bags/` |
| 容器 ROS 日志 | `runtime/flight_bags/ros_logs/<run-id>/` |
| 宿主 tmux 日志 | `~/uav-autonomy-aio_logs/<run-id>/` |

rosbag 默认使用 LZ4、1024 MB（约 1 GiB）分卷，轮换参数
`ROSBAG_MAX_SPLITS=10`；rosbag 切分时可能额外保留一卷，完成的运行也可能有
11 个分卷，因此该参数不是严格的 10 GiB 空间上限。启动时默认要求
`runtime/flight_bags` 至少有 5 GiB 可用空间，历史运行不会自动删除。

```bash
# 不记录原始 Livox 数据
ROSBAG_RECORD_RAW_LIDAR=false ./launch/real.sh --planner diff start

# 完全关闭录包
START_ROSBAG=false ./launch/real.sh --planner diff start
```

`ROSBAG_RECORD_RAW_LIDAR=false` 时，默认话题集合不再追加 `/livox/lidar`，但仍记录
定位点云、里程计、规划器、控制器和 MAVROS 等默认话题。

### 离线回放

真机栈停止后执行：

```bash
./launch/real_bag.sh play runtime/flight_bags/FILE.bag
./launch/real_bag.sh status
./launch/real_bag.sh attach
./launch/real_bag.sh stop

# 工作站打开离线 RViz；以下 IP 地址仅为示例，需根据实际情况调整
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
./launch/real_bag_rviz.sh
```

省略 bag 路径时 `play` 只在 `runtime/flight_bags` 中选择最新的已完成 `.bag`
文件，不会自动选择 `.bag.active`。`BAG_RATE` 控制播放倍率，`BAG_LOOP=true`
循环播放，例如：

```bash
BAG_RATE=0.5 BAG_LOOP=true ./launch/real_bag.sh play runtime/flight_bags/FILE.bag
```

离线回放会启动独立 ROS master（或复用一个不含真机节点的现有 master）、回放
已记录话题；若 bag 含原始 Livox 消息，还会转换为 `/livox/lidar_points` 供 RViz
显示。它不会启动 FAST-LIO、规划器、SE3 或 MAVROS，也不会启用 RViz 目标转发。

## 安全边界

- 飞手和遥控器始终拥有最终控制权；
- 软件围栏不能替代 PX4 failsafe；
- SE3 的 MAVROS 里程计或 IMU 失效时会停止姿态/推力输出，并在 armed/OFFBOARD
  且 MAVROS 状态可信时请求 `AUTO.LOITER`；MAVROS 状态本身失联时无法确认恢复
  模式，只能依赖 PX4 Offboard-loss failsafe 并由飞手接管；
- 定位故障会锁存，并在自主飞行中请求 `AUTO.LAND`；排除原因后需重启完整栈；
- 更换载荷、动力系统或雷达安装位置后必须重新标定。

控制器原理与调参见 [SE3 控制器](se3_controller.md)。
