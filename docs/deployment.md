# 真机部署指南：Jetson 与地面站

本文面向单机真机部署，说明两台电脑各自运行什么、首次如何安装，以及每次飞行和离线回放的标准操作顺序。所有命令均从仓库根目录执行。

> **安全提示**：`start/stop/restart` 用于管理 ROS 栈；解锁和起飞使用 `arm`，降落使用
> `land`。飞机仍处于 armed（已解锁）状态，或活动栈的 MAVROS 状态无法确认时，
> 停止和重启会被拒绝。
> `--force` 仅用于飞手已经人工确认飞机落地、解除武装后的故障恢复或维护。

## 两台电脑的职责

真机部署使用两个不同的容器。两台电脑应放置同一版本的完整仓库，并按角色执行脚本：

| 电脑 | 运行内容 | 容器管理 | 日常入口 |
|---|---|---|---|
| 机载 Jetson | Livox、FAST-LIO、规划器、SE3、MAVROS、rosbag | `./launch/real_container.sh` | `./launch/real.sh`、`./launch/real_bag.sh` |
| 地面站 | 实时/嵌入式 RViz、交互目标面板、离线回放显示 | `./launch/ground_station_container.sh` | `./launch/real_rviz.sh`、`./launch/embedded_rviz.sh`、`./launch/real_bag_rviz.sh` |

交互目标 Action 由 Jetson 真机栈启动，地面站 RViz 只负责显示、确认和
发送请求。`embedded_rviz.sh` 供 `swarm-uav-mapping` 嵌入三维模式使用；
离线回放使用独立入口 `./launch/real_bag_rviz.sh`。

系统数据流如下：

```text
MID-360 → Livox 驱动 → FAST-LIO
                        ├─> /localization/odom ─────────────┬─> 所选规划器 / SE3
                        │                                  └─> MAVROS ODOMETRY → PX4 EKF2
                        └─> /localization/cloud_registered ───> 所选规划器

所选规划器 → 规划器网关 → SE3 → MAVROS 姿态设定值 → PX4
无规划轨迹时                         └─> MAVROS 本地位置保持 → PX4
```

## 部署前准备

### 网络

Jetson、地面站和 QGC 所在网络必须按实际拓扑配置。当前单机联调固定使用：

| 设备 | 有线地址 |
|---|---|
| Jetson | `192.168.1.123` |
| 地面站 | `192.168.1.124` |

部署前确认：

1. Jetson 和地面站可以通过上述地址双向通信；
2. Jetson 启动 ROS 时公布的是地面站可访问的地址；
3. 若启用了防火墙，允许两台主机之间的 ROS 1 双向通信，包括自主飞行栈的
   `11312` 和节点使用的动态端口；现有全景相机/桥接栈继续使用 `11311`；
4. Jetson 的雷达网卡和 MID-360 位于同一有线子网。

三个容易混淆的变量分别表示：

| 变量 | 设置位置 | 含义 |
|---|---|---|
| `ROS_IP` | Jetson | Jetson 向其他 ROS 节点公布的可访问地址 |
| `JETSON_IP` | 地面站 | 地面站用于连接 Jetson ROS master（主节点）的地址 |
| `LOCAL_IP` | 地面站 | 地面站向 Jetson 公布的回程地址；通常可自动探测 |

### Jetson

Jetson 宿主机需要：

- Docker；
- 可识别的 PX4 串口，例如 `/dev/ttyACM0`；
- 已配置的 MID-360 有线网卡；
- 足够的录包空间。默认启动要求 `runtime/flight_bags` 至少有 5 GiB 可用空间。

### 地面站

地面站宿主机需要：

- Docker 和可用的桌面图形会话；
- 到 Jetson 的双向 IP 路由。

## 首次配置

### MID-360 网络

编辑 [`deployment/config/livox/MID360s_config.json`](../deployment/config/livox/MID360s_config.json)：

- `Mid360s.host_net_info[0].host_ip`：Jetson 雷达网卡地址；
- `lidar_configs[0].ip`：MID-360 地址。

安装外参统一由 `MOUNT_*` 适配层处理，`extrinsic_parameter` 保持全零，以免重复应用变换。

### 安装外参

`MOUNT_X/Y/Z` 和 `MOUNT_ROLL/PITCH/YAW_DEG` 定义从 MID-360 内置 IMU 坐标系到
`base_link` 的变换 `T_base_fastlio_body`：

- 平移是 IMU 原点在 `base_link` 中的位置，单位为米；
- 姿态按 `Rz(yaw) Ry(pitch) Rx(roll)` 组合，单位为度；
- `base_link` 采用 x 向前、y 向左、z 向上的右手坐标系。

[`launch/real.sh`](../launch/real.sh) 顶部的默认值对应当前已标定机体。
标定时可以先在当前 Jetson 终端临时覆盖：

```bash
export MOUNT_X=0.109
export MOUNT_Y=0.024
export MOUNT_Z=0.006
export MOUNT_ROLL_DEG=0.0
export MOUNT_PITCH_DEG=34.9
export MOUNT_YAW_DEG=0.5
```

用实际标定值替换示例。确认后写回 `launch/real.sh` 的默认值。修改外参后重启真机栈
即可；每次 `start/restart` 必须使用同一组值。

建议的最小验证流程：

1. 根据机械图和机架基准测量 IMU 原点相对 `base_link` 的前、左、上偏移；
2. 按右手定则测量 IMU 坐标轴相对机体的 roll、pitch、yaw；
3. 卸桨启动真机栈，沿机体三轴平移并分别转动，检查 RViz 中固定环境点云是否稳定；
4. 使用 `rosrun tf tf_echo world base_link` 和 rosbag 复核后固化数值。

更换雷达安装位置后必须重新标定。需要更高精度时，应使用动作捕捉或其他外部位姿进行手眼标定。

### PX4 与控制器

首次飞行前确认 PX4：

- external-vision EKF 的位置、方向和尺度正确；
- `MAV_SYS_ID`、飞控串口和 QGC 链路正确；
- RC 模式切换、Kill、RC loss、Offboard loss、电池和估计器 failsafe 已验证；
- 已卸桨完成解锁、模式切换和控制方向测试。

真机载体参数位于
[`deployment/config/controller.yaml`](../deployment/config/controller.yaml)。更换动力系统
或载荷时，应使用对应机体的标定值，并按以下顺序复核：

| 参数 | 复核重点 |
|---|---|
| `hover_percent` | 从稳定悬停日志标定，确认低高度 OFFBOARD 悬停不持续升降 |
| `min_output_thrust`、`max_output_thrust` | 覆盖悬停推力并保留控制余量；持续饱和时先检查载荷、动力和电池 |
| `ki_pz`、`int_limit_z` | 在悬停推力正确、定位稳定且推力未饱和后调整 |
| `geo_fence.x/y/z` | 按现场净空设置，并与 PX4 geofence 和 failsafe 配合使用 |

详细方法见 [SE3 控制器调参顺序](se3_controller.md#调参顺序)。更换电机、桨、电池、
机体重量或载荷后，至少重新标定悬停推力并复核推力范围和高度积分。

## 首次创建容器

### Jetson：机载容器

```bash
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh run
./launch/real_container.sh status
```

`build` 构建完整真机镜像；`run` 创建或启动容器，并把 `FCU_DEVICE` 指定的宿主串口
映射到容器。容器准备完成后，通过 `real.sh` 启动真机 ROS 栈。

`FCU_DEVICE` 与后续 `FCU_URL` 的职责不同：

- `FCU_DEVICE=/dev/ttyACM0`：创建容器时映射宿主设备；
- `FCU_URL='/dev/ttyACM0:921600'`：启动 MAVROS 时指定容器内设备和波特率。

非默认串口的设备路径必须在两处保持一致。

### 地面站：RViz 容器

```bash
./launch/ground_station_container.sh build
./launch/ground_station_container.sh run
./launch/ground_station_container.sh verify
```

`build` 构建地面站镜像，`run` 创建或启动容器，`verify` 检查 RViz、
交互目标面板和 Action 消息依赖。

地面站容器直接使用宿主机网络连接 Jetson，并通过桌面图形会话显示 RViz。
容器内的 `ground_station_telemetry.py` 可订阅 MAVROS 状态并向宿主 Qt 程序输出
JSON，因此 `swarm-uav-mapping --aio-real` 不依赖 ROS2 `onboard_msgs` 或
`domain_bridge`。

## 启动真机栈

以下操作均在 Jetson 执行。先查看可用规划器：

```bash
./launch/real.sh planners
```

`start` 和 `restart` 必须指定 `diff`、`fast-kino`、`fast-topo` 或 `super`。下面是完整
启动示例，地址和系统 ID 必须替换为现场值：

```bash
FCU_URL='/dev/ttyACM0:921600' \
GCS_URL='udp://:14555@192.168.1.124:14550' \
ROS_IP=192.168.1.123 \
MAVROS_TGT_SYSTEM=2 \
./launch/real.sh --planner diff start
```

| 参数 | 含义 |
|---|---|
| `--planner` | 本次运行使用的规划器 |
| `FCU_URL` | MAVROS 使用的容器内串口和波特率 |
| `GCS_URL` | MAVROS 到 QGC 的 UDP 链路 |
| `ROS_IP` | Jetson 对地面站公布的地址；默认 `192.168.1.123`，维护时可显式覆盖 |
| `MAVROS_TGT_SYSTEM` | 必须等于 PX4 的 `MAV_SYS_ID` |

命令前设置的 `FCU_URL`、`GCS_URL`、`ROS_IP`、`MAVROS_TGT_SYSTEM` 和 `MOUNT_*`
作用于本次启动。需要重复使用时，可以提前 `export` 到当前终端。

启动后使用：

```bash
./launch/real.sh status
./launch/real.sh attach   # 查看 tmux 日志；按 Ctrl-b d 退出
```

启动器依次启动坐标系别名、Livox、FAST-LIO、里程计/点云适配、MAVROS、
外部视觉位姿桥、定位保护、所选规划器、SE3 和 rosbag。任一关键阶段失败都会清理本次启动产生的部分进程。

## 飞前检查

先进入 Jetson 容器：

```bash
./launch/real_container.sh shell
```

在容器中逐项检查。`rostopic hz` 和 `tf_echo` 需要观察数秒，再按 `Ctrl-C`：

```bash
rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic hz /mavros/odometry/out
rostopic info /mavros/odometry/out
rostopic echo -n1 /planning/status
rostopic echo -n1 /mavros/state
rosrun tf tf_echo world base_link
rosrun tf tf_echo odom_ned world
```

起飞前必须确认：

1. Livox 点云和 IMU 连续；
2. `/localization/odom` 时间戳持续前进，位置、速度、尺度和方向正确；
3. 注册点云位于 `world`，并与同一时刻的里程计和障碍对齐；
4. `/mavros/odometry/out` 的发布者为 `/external_odometry_bridge`、订阅者包含
   `/mavros`，且 `odom_ned <- world`、`base_link_frd <- base_link` 两个 TF 连通；
5. PX4 已连接并正确采用 external vision；
6. `/planning/status` 为 READY，`odom_ready`、`map_ready` 为 true；
7. 目标、Mission、规划器地图边界和控制器围栏符合现场；
8. 外参、悬停推力、failsafe 和遥控接管已经在当前机体上验证。

真机定位必须通过 MAVROS odometry 插件进入 PX4。该插件发送
`MAVLink ODOMETRY`，父坐标系为 `MAV_FRAME_LOCAL_FRD`，因此 FAST-LIO 每次
启动时形成的任意局部航向会由 EKF2 旋转到飞控地球坐标系。不要把
`/localization/odom` 改接到 `/mavros/vision_pose/pose`：PX4 1.16 会把
`VISION_POSITION_ESTIMATE` 无条件标记为 NED；若 FAST-LIO 的局部 x 轴没有
恰好对准北向，会使位置反馈方向与飞控航向不一致并破坏定点闭环。

QGC 的 MAVLink Inspector 中，由飞控组件 1 发出的 `ODOMETRY` 仍会显示
`frame_id=1`（`MAV_FRAME_LOCAL_NED`）以及 z 向下。这是 PX4 对外发布自身
NED 状态的正常形式，不能用它判断传入 PX4 的 MID360 坐标系。验证输入链路
应查看 `/mavros/odometry/out`、上述 TF 和 PX4 EKF 状态。

当前 PX4 1.16 真机配置保持 `EKF2_EV_CTRL=3`，即只启用 Horizontal position
和 Vertical position：

- 不启用 `3D velocity`。当前 FAST-LIO `/Odometry` 不提供可直接融合的三维
  线速度，误启用会把无效或近零速度当成观测值；
- 不启用 `Yaw`。在 `MAV_FRAME_LOCAL_FRD` 下，EKF2 即使不融合 external-vision
  yaw，也会使用外部姿态与飞控姿态的差值对齐两个位置坐标系，同时保留飞控
  当前航向源；
- 若以后要改用激光里程计航向作为主航向源，必须独立验证航向稳定性、方差、
  静止和电机运行工况，再评估 `EKF2_EV_CTRL=11`，不能在首次定位修复中同时
  改动。

## 地面站实时 RViz

确认 Jetson 真机栈已正常启动，且以下两个 Action status 话题均已存在后，
可以单独打开 RViz：

- `/ground_station/interactive_goal/status`
- `/ground_station/flight_command/status`

```bash
./launch/real_rviz.sh
```

`JETSON_IP`/`LOCAL_IP` 默认分别为 `192.168.1.123`/`192.168.1.124`，
`ROS_MASTER_PORT` 默认为 `11312`，维护时可显式覆盖。`swarm-uav-mapping` 使用不带伪终端的
`./launch/embedded_rviz.sh`，取得 X11 窗口 ID 后嵌入其三维模式。

嵌入窗口隐藏 RViz 侧栏；工具栏保留 `2D Nav Goal`，并在其后依次显示
`Takeoff`、`Land`。`2D Nav Goal` 只生成候选 x/y/yaw，随后弹窗设置目标高度
和必要时的起飞高度，范围均为 0.5–2.5 m，默认 1.5 m。请求通过
`/ground_station/interactive_goal` Action 提交，并显示校验、解锁、
`AUTO.TAKEOFF`、OFFBOARD 交接和目标发布结果。

- 已解锁且处于 OFFBOARD：校验后直接发送；
- 已解锁但不在 OFFBOARD：拒绝，不自动切换模式；
- 未解锁：必须有新鲜的 MAVROS 状态且明确为 `ON_GROUND`，再经过默认
  选中“取消”的二次确认，才会自动解锁、起飞、交接 OFFBOARD 并发送目标。

机载端在解锁前调用 `/planning/validate_goal`，并与 `real.sh arm/goal/mission`
以及工具栏起飞/降落共用同一个生命周期文件锁。目标被接受后保持 OFFBOARD，
不自动降落；界面提示“目标已接受”只表示规划器已接收目标，不表示已经到达。

`Takeoff` 通过 `/ground_station/flight_command` 发送独立起飞命令。它仅在收到
新鲜 MAVROS 状态、飞行器未解锁且明确为 `ON_GROUND` 时启用；确认后执行解锁、
PX4 `AUTO.TAKEOFF`，到达 0.5–2.5 m 范围内设置的高度后进入经过验证的
OFFBOARD 悬停，不会发布规划目标。此阶段由 PX4 本地位置环保持切换瞬间的
位置和航向，SE3 不发原始姿态/推力。使用 `2D Nav Goal` 产生首条有效轨迹后，
再通过锁存的 FAST-LIO-world 到飞控姿态对齐，在 1.5 s 内平滑切入 SE3 控制。

`Land` 也通过 `/ground_station/flight_command`，仅在无人机已解锁、明确处于
空中且由本系统自主模式控制时启用。机载端反复请求并以新鲜
`/mavros/state.mode == AUTO.LAND` 为成功判据；不会强制反解锁，也不会把遥控器
接管的 `STABILIZED`/`POSCTL` 覆盖成自动降落。成功提示表示 PX4 已接受自动降落
模式，操作者仍须观察至实际落地并停止旋翼。

## 飞行命令

以下命令均在 Jetson 执行。

### 起飞

```bash
REAL_TAKEOFF_HEIGHT=1.5 ./launch/real.sh arm
```

默认起飞高度为 `1.0 m`。该命令检查 MAVROS、定位、SE3 和控制指令预热，随后解锁、
执行 PX4 `AUTO.TAKEOFF`，到达相对 Home 的目标高度后进入经过验证的 OFFBOARD
本地位置悬停。在收到首条规划轨迹前不启用 SE3 原始姿态控制。
进入 OFFBOARD 悬停后，系统开始接受规划目标。

### 单目标

```bash
# 不约束终点 yaw
./launch/real.sh goal 1.0 0.0 1.5

# 终点 yaw 为 90°
./launch/real.sh goal 1.0 0.0 1.5 90
```

坐标是 `world` 绝对坐标，yaw 单位为度。命令在发布前检查 armed/OFFBOARD、定位、
SE3、规划器状态和目标验证服务。发布成功只表示目标被接受，不保证一定存在可行路径。

### Mission

```bash
./launch/real.sh mission MISSION_FILE.json
```

Mission 格式与[仿真 Mission](simulation.md#mission)相同。任务会在解锁前校验航点；滚动
局部地图规划器会延后“目标暂时超出局部地图”的范围判定，并在航点下发前再次校验。

- 当前未解锁：自动起飞并进入 OFFBOARD 后执行；
- 已经 armed/OFFBOARD：直接执行；
- `land_after_mission=true`：完成后进入 `AUTO.LAND` 并等待自动解除武装；
- `land_after_mission=false`：保持最终点和 OFFBOARD；
- 飞手人工切换模式：任务终止，并保持飞手选择的模式。

定位故障会请求 `AUTO.LAND`；其他任务故障优先请求 `AUTO.LOITER`，无法确认 LOITER
时再尝试 `AUTO.LAND`。

### 降落与停止

```bash
./launch/real.sh land   # 请求并确认 AUTO.LAND，由 PX4 处理落地解除武装
./launch/real.sh stop   # 确认落地并解除武装后停止 ROS 栈
```

`stop` 的作用范围是 ROS 栈，机载容器继续运行。降落应先执行 `land` 或由飞手通过 RC
完成。飞行中不得把 `stop --force` 当作应急操作。

## 日志与 rosbag

| 内容 | 宿主机路径 |
|---|---|
| rosbag | `runtime/flight_bags/` |
| 容器 ROS 日志 | `runtime/flight_bags/ros_logs/<run-id>/` |
| tmux 日志 | `~/uav-autonomy-aio_logs/<run-id>/` |

默认压缩录包，每卷约 1024 MB，`ROSBAG_MAX_SPLITS=10`。切卷期间可能额外保留一卷，
磁盘空间应按至少 11 卷预留；历史 bag 由使用者定期整理。

```bash
# 仅记录定位、规划、控制和 MAVROS 等默认话题
ROSBAG_RECORD_RAW_LIDAR=false ./launch/real.sh --planner diff start

# 完全关闭录包
START_ROSBAG=false ./launch/real.sh --planner diff start
```

### 离线回放

先停止真机栈，然后分别在两台电脑执行。

Jetson：

```bash
./launch/real_bag.sh play runtime/flight_bags/FILE.bag
./launch/real_bag.sh status
./launch/real_bag.sh attach
./launch/real_bag.sh stop
```

地面站：

```bash
JETSON_IP=172.20.10.5 \
./launch/real_bag_rviz.sh
```

省略 bag 路径时，`play` 选择 `runtime/flight_bags` 中最新的已完成 `.bag`；播放中的
`.bag.active` 文件会被忽略。播放倍率和循环可通过以下变量设置：

```bash
BAG_RATE=0.5 BAG_LOOP=true \
./launch/real_bag.sh play runtime/flight_bags/FILE.bag
```

离线回放使用独立 ROS master 发布 bag 中的话题，并在需要时转换原始 Livox 消息供
RViz 显示。地面站通过专用离线入口连接该回放环境。

## 参数设置

### 配置文件位置

| 内容 | 文件 |
|---|---|
| Livox 网络 | [`deployment/config/livox/MID360s_config.json`](../deployment/config/livox/MID360s_config.json) |
| 控制器公共接口与安全参数 | [`common/config/controller.yaml`](../common/config/controller.yaml) |
| 规划器控制器覆盖 | 各插件 manifest 的 `controller_config`；当前 SUPER 使用 [`planning/plugins/super/controller.yaml`](../planning/plugins/super/controller.yaml) |
| 真机载体参数 | [`deployment/config/controller.yaml`](../deployment/config/controller.yaml) |
| Diff | [`planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) |
| Fast Kino/Topo | [`planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml) |
| SUPER | [`planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml) |

控制器配置按“公共配置 → 当前规划器的 `controller_config` → 真机载体配置”的顺序
加载，后加载的同名参数覆盖前者。

### 修改后需要做什么

| 修改内容 | 生效方式 |
|---|---|
| `MOUNT_*`、`launch/real.sh` 运行参数 | 落地解除武装后重启真机栈 |
| Livox 配置、真机载体配置 | 先重建 Jetson 镜像和容器，再重启真机栈；镜像源码哈希也覆盖这些 Docker COPY 输入 |
| Jetson 镜像内的 ROS 包、规划器、消息、Dockerfile、公共或规划器控制器配置 | 重建 Jetson 镜像和容器 |
| `deployment/ground_station/`、`sim2real_ground_station`、`sim2real_planning_msgs` | 重建地面站镜像和容器 |
| RViz 配置 | 重建 Jetson 镜像（机载默认副本）；地面站端配置会在每次启动时复制，只需重新运行 RViz 入口 |
| `embedded_rviz.py`、`interactive_goal_ui.py` | 重建地面站镜像和容器，再重新运行 RViz 入口 |

#### 重建 Jetson

```bash
./launch/real.sh stop
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh restart
```

容器重建完成后，按[启动真机栈](#启动真机栈)重新传入设备、网络、外参和规划器参数，
启动 ROS 栈。

#### 重建地面站

```bash
./launch/ground_station_container.sh build
./launch/ground_station_container.sh recreate
./launch/ground_station_container.sh verify
```

`run` 会检查地面站镜像是否包含当前内容的 `deployment/ground_station/`、
`sim2real_ground_station` 和 `sim2real_planning_msgs`；内容不一致时会提示重建容器。

### 临时规划器配置

所有规划器都可用 `PLANNER_CONFIG` 加载当前插件的一份完整配置。先把文件放到 Jetson
宿主机 `runtime/tmp/FILE.yaml`，再按下例设置容器内路径：

```bash
PLANNER_CONFIG=/root/tmp/FILE.yaml \
./launch/real.sh --planner fast-kino start
```

Diff、Fast Kino/Topo 与 SUPER 的配置格式分别以本节表中的默认文件为准。

## 常见问题

### 地面站提示镜像或容器过期

地面站源码或容器使用的镜像已经变化。按[重建地面站](#重建地面站)更新镜像和容器。

### 实时 RViz 因机载 Action 未就绪而退出

这通常表示 Jetson 上的规划器尚未就绪，或目标、起降 Action 中至少一个没有启动。先在 Jetson 执行：

```bash
./launch/real.sh status
./launch/real_container.sh shell
```

然后在容器中检查：

```bash
rostopic echo -n1 /planning/status
rosservice info /planning/validate_goal
rostopic echo -n1 /mavros/state
rostopic info /ground_station/interactive_goal/status
rostopic info /ground_station/flight_command/status
```

该检查确保实时 RViz 打开后具备安全发送目标、起飞和降落的条件。

### 地面站无法连接 Jetson

依次确认：

1. 地面站可以访问 `JETSON_IP`，Jetson 也可以访问 `LOCAL_IP`；
2. Jetson 启动栈时设置的 `ROS_IP` 与 `JETSON_IP` 指向同一个可达地址；
3. 地面站容器直接使用宿主机网络；
4. 主机防火墙没有阻止 ROS 1 节点之间的双向连接。

### 飞控串口未映射

确认 Jetson 上设备路径存在，再用同一路径重建机载容器：

```bash
ls -l /dev/ttyACM0
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh restart
```

重建前必须先停止真机栈，并确认飞机已经落地、解除武装。

### 停止或重启被拒绝

脚本检测到 PX4 仍然 armed，或检测到活动中的真机栈/录包进程。应先通过飞手、RC 或
`./launch/real.sh land` 安全降落，确认解除武装后再执行 `./launch/real.sh stop`。
只有在飞机安全状态已经由飞手独立确认后，维护人员才可以考虑 `--force`。

## 安全边界

- 飞手和遥控器始终拥有最终控制权；
- PX4 failsafe 是主要保护机制，软件围栏作为辅助；当前
  `auto_land_on_geofence=false` 时越界只告警；
- SE3 的 MAVROS 里程计或 IMU 失效时会停止姿态/推力输出，并在 armed/OFFBOARD 且
  MAVROS 状态可信时请求 `AUTO.LOITER`；
- MAVROS 状态失联时无法确认恢复模式，只能依赖 PX4 Offboard-loss failsafe 并由飞手
  接管；
- 定位故障会锁存，并在自主飞行中请求 `AUTO.LAND`；排除原因后需重启完整栈；
- 更换载荷、动力系统或雷达安装位置后必须重新标定。

控制器原理与详细调参方法见 [SE3 控制器](se3_controller.md)。
