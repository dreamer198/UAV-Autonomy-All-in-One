# Jetson 真机部署

真机栈包含 Livox 驱动、FAST-LIO、定位适配、MAVROS、Diff-Planner、轨迹转换和 SE3 控制器。

> `real.sh start/restart` 不会解锁或切换飞行模式，`real.sh stop/restart` 也不会请求降落。飞机仍解锁或活动栈无法确认 MAVROS 状态时，停止和重启会被拒绝。`--force` 仅用于飞手已确保飞机安全后的应急维护。

```text
MID-360 → Livox driver → FAST-LIO
                        → /localization/odom
                        → /localization/cloud_registered
                        → Diff-Planner → SE3 → MAVROS → PX4
```

公共里程计还会回灌到 `/mavros/vision_pose/pose`，保留原测量时间戳，并拒绝过期、
重复或乱序数据。PX4 是否采用该定位取决于飞控端 external-vision EKF 配置，必须
单独确认。注册点云按其测量时刻的 TF 变换到 `world`；需要跨坐标系时，时间戳或
对应 TF 不可用便丢弃，不会只修改 `frame_id`。

除“远程 RViz”和“离线回放”中明确标为工作站的命令外，其余命令均在 Jetson 的
仓库根目录执行。

## 部署前配置

### Livox 网络

编辑 `deployment/config/livox/MID360s_config.json`：

- `Mid360s.host_net_info[0].host_ip`：Jetson 直连雷达网卡地址；
- `lidar_configs[0].ip`：MID-360 地址；
- `lidar_configs[0].extrinsic_parameter`：本项目保持全零，安装外参由后续适配层统一处理。

Jetson 与雷达必须位于同一有线子网。仓库中的 IP 只是当前设备基线。

### 控制器

编辑 `deployment/config/controller.yaml`，至少重新验证：

- `hover_percent`；
- `ki_pz` 与 `int_limit_z`；
- `min_output_thrust` / `max_output_thrust`；
- `geo_fence`。

更换电机、桨、电池、重量或载荷后不能沿用旧标定。

### 雷达外参

启动时通过 `MOUNT_*` 设置 `base_link → FAST-LIO body`：

```text
MOUNT_X / MOUNT_Y / MOUNT_Z
MOUNT_ROLL_DEG / MOUNT_PITCH_DEG / MOUNT_YAW_DEG
```

这里的 `body` 是 MID-360 内置 IMU 原点，不是点云原点。若只能量到 LiDAR 原点，需要结合 FAST-LIO 中的 LiDAR→IMU 内参换算，不能直接填写尺量平移。仓库默认值仅适用于当前机体，使用前应在多个朝向下检查点云与机体运动是否一致。

### PX4

首次飞行前至少完成：

- external-vision EKF 配置与方向验证；
- `MAV_SYS_ID`、串口和 QGC 链路配置；
- 遥控器模式、Kill、RC loss、Offboard loss、电池和估计器 failsafe；
- 卸桨状态下的解锁、模式和控制方向测试。

## 构建并创建容器

```bash
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh run
```

容器会挂载飞控设备、Livox 配置、控制器配置和 `runtime/`。

```bash
./launch/real_container.sh status
./launch/real_container.sh shell
```

重建或重启容器会删除旧容器。脚本检测到真机栈、飞行节点或 recorder 活动时会拒绝
修改容器。应先停止真机栈并确认飞机已经落地、解除锁定；`--force` 只用于应急维护。

## 启动 ROS 栈

以下网络值是示例，应按现场修改：

```bash
FCU_URL='/dev/ttyACM0:921600' \
GCS_URL='udp://:14555@172.20.10.3:14550' \
ROS_IP=172.20.10.5 \
MAVROS_TGT_SYSTEM=5 \
./launch/real.sh start
```

`MAVROS_TGT_SYSTEM` 必须与 PX4 的 `MAV_SYS_ID` 一致。非默认 `MOUNT_*`、设备和网络变量需要在每次 `start/restart` 时继续传入。

启动器会按依赖顺序启动传感器、定位、MAVROS、规划、控制和 rosbag，并检查关键 topic。任一阶段失败都会清理本次部分启动。

```bash
./launch/real.sh status
./launch/real.sh attach
```

`status` 只反映进程状态，不代表定位、点云或飞行状态正确。

## 飞前检查

在容器中检查：

```bash
./launch/real_container.sh shell

rostopic hz /livox/lidar
rostopic hz /livox/imu
rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic hz /mavros/vision_pose/pose
rostopic echo -n1 /mavros/state
rosrun tf tf_echo world base_link
```

起飞前确认：

1. `/livox/lidar` 类型为 `livox_ros_driver2/CustomMsg`，雷达和 IMU 连续；
2. `/localization/odom` 新鲜，时间戳持续前进，位置、速度、尺度和方向正确；
3. `/localization/cloud_registered` 位于 `world`，时间戳有效，并与同一时刻的
   里程计和真实障碍对齐；
4. `/mavros/state` 已连接，且未处于 `FLIGHT_TERMINATION`；
5. `/mavros/vision_pose/pose` 连续，PX4 EKF 已采用 external vision；
6. Planner 高度范围、控制器围栏、Mission 航点和现场净空一致；
7. 悬停推力、推力范围、外参和 failsafe 已在当前机体上验证；
8. 飞手可以随时切回人工模式。

## 飞行命令

### 解锁并起飞

```bash
REAL_TAKEOFF_HEIGHT=1.5 ./launch/real.sh arm
```

`REAL_TAKEOFF_HEIGHT` 是相对 PX4 Home 的起飞高度，省略时默认为 `1.0 m`。命令先
检查 MAVROS、定位、SE3 和预热 setpoint，再设置 PX4 原生起飞高度，并临时收紧
`NAV_MC_ALT_RAD`。高度和垂直速度稳定后恢复原接受半径，自动进入 OFFBOARD，并
确认姿态/推力输出。

命令成功返回表示自动交接已完成，不代表定位质量和场地安全可以省略复核。交接前发布的目标不会排队，进入 OFFBOARD 后必须发布新目标。

起飞过程中会持续检查 MAVROS 状态、高度和定位。定位失效时直接请求
`AUTO.LAND`；其他自主执行故障先确认 `AUTO.LOITER`，无法确认时再尝试
`AUTO.LAND`。飞手已切入其他模式时不会被覆盖；MAVROS 状态本身失联时必须立即
遥控接管。

### 发布单目标

```bash
./launch/real.sh goal 1.0 0.0 1.5
./launch/real.sh goal 1.0 0.0 1.5 90
```

格式为 `goal X Y Z [YAW_DEG]`。坐标系为 `world`；省略 yaw 时不限定终点朝向，提供时单位为度。

命令会检查飞机已解锁且处于 OFFBOARD、定位、SE3 输出、Planner、高度围栏和目标
订阅者。成功返回只表示 `/goal` 已发布。

目标没有距离硬限制，但 Planner 使用滚动局部地图。长距离参考线不是全局无碰撞路线，短距离目标也可能被墙体、死胡同或膨胀障碍阻断。未知环境应通过 Mission 提供经过确认的中间航点，并持续观察规划轨迹和日志。

### Mission

核对任务文件后执行；根目录的 `mission_*.json` 可作为格式参考：

```bash
./launch/real.sh mission MISSION_FILE.json
```

`waypoints` 是 `world` 中的绝对坐标。共享 Mission 执行器的主要行为：

- 未解锁时按 `takeoff_height` 原生起飞并自动进入 OFFBOARD；
- 已解锁且处于 OFFBOARD 时直接开始航点；
- 已解锁但处于其他模式时中止，避免覆盖飞手控制；
- 省略 yaw 时按有效航段自动生成；
- 中间点可选择 `fly_through` 或停稳；
- `land_after_mission=true` 时成功后请求 `AUTO.LAND`，否则在最终点保持 OFFBOARD；
- 人工接管、状态或定位失联、规划失败会中止后续航点；
- 定位失效请求 `AUTO.LAND`；规划等其他任务故障请求 `AUTO.LOITER`，且不覆盖
  飞手模式。起飞或 OFFBOARD 交接失败时，`AUTO.LOITER` 无法确认才继续尝试
  `AUTO.LAND`。

字段格式可参考根目录的 `mission_*.json`；其中坐标和高度属于具体场地，不能直接
用于其他场地。

### 降落与停止

```bash
./launch/real.sh land
```

`land` 直接请求 PX4 `AUTO.LAND`，不通过 Planner，也不会强制解除锁定。脚本只等待模式切换成功；下降、接地检测和自动解除锁定由 PX4 负责。

确认飞机已经落地并解除锁定后，才能执行：

```bash
./launch/real.sh stop
```

`stop` 只停止 ROS 栈，不会请求降落，也不能作为飞行中的应急操作。飞机仍解锁，
或活动栈的 `/mavros/state` 无法读取时，`stop/restart` 会拒绝执行。飞手确认飞机
安全后若需应急清理，可显式使用 `stop --force` 或 `restart --force`。
普通启停与 `arm`、`goal`、`mission` 使用同一生命周期锁，避免停止检查通过后又被
并发解锁；`land` 不受该锁限制，任务运行中仍可单独请求降落。

## 远程 RViz

工作站需预先创建包含 ROS Noetic 和 RViz 的 GUI 容器，默认名为 `ros_noetic`。
在同一网络的工作站仓库根目录执行：

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
RVIZ_GOAL_Z=1.5 \
RVIZ_GOAL_FRAME=world \
./launch/real_rviz.sh
```

目标桥把 RViz 专用 `/sim2real/rviz_goal` 转发到 `/goal`，避免误接收 MAVROS 的
`/move_base_simple/goal`。脚本退出时会清理目标桥，不会留下后台节点。

目标桥以非锁存方式发布，并在每次点击时检查数值、`world` 坐标系、MAVROS 状态、
armed/OFFBOARD、定位保护和当前 Planner 高度范围。任一条件不满足都会丢弃目标，
不会排队或稍后重放。`RVIZ_GOAL_Z` 应按本次 Planner 范围设置；无效配置会使目标桥
启动失败。

## Planner 参数

Diff 默认参数来自 `planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml`。目标和 Mission 航点必须满足：

```text
virtual_ground + obstacles_inflation < Z
Z < virtual_ceil - obstacles_inflation
```

修改飞行高度时，编辑 `grid_map/virtual_ground` 和 `grid_map/virtual_ceil`。数值应
依据本次 `world` 坐标系的高度原点、现场净空、任务高度和控制器围栏确定，边界值
本身不允许。

`grid_map/resolution` 和 `grid_map/obstacles_inflation` 必须依据机体完整三维碰撞包络、桨叶、定位误差和所需安全余量验证，不能只根据机身宽度推导。

公共 Planner 配置会复制进真机镜像。永久修改后需要重建镜像和容器。临时测试可在飞机落地并解除锁定后使用挂载目录：

```bash
cp planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml runtime/tmp/planner_test.yaml
# 修改 runtime/tmp/planner_test.yaml
PLANNER_CONFIG=/root/tmp/planner_test.yaml ./launch/real.sh restart
```

`PLANNER_CONFIG` 是完整 YAML，不能与 `PLANNER_RESOLUTION` 或 `PLANNER_OBSTACLES_INFLATION` 同时设置。后两个变量只用于临时覆盖对应参数。

## 更新代码或配置

修改 Dockerfile、镜像内 ROS 源码、公共 Planner 配置或 ROS 消息后：

```bash
./launch/real.sh stop
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh restart
```

随后重复“启动 ROS 栈”一节的完整现场命令，不能遗漏 FCU、网络、system ID 或
非默认外参变量。

`real.sh restart` 只重启 ROS 栈，`real_container.sh restart` 只重建容器；二者都不会构建新镜像。

Livox JSON 和真机 controller YAML 通过 bind mount 加载，只修改这两个文件时不必重建镜像。确认飞机已落地并解除锁定后，执行 `./launch/real.sh restart` 重新加载。

## 日志与 rosbag

默认位置：

- rosbag：`runtime/flight_bags/`；
- 容器 ROS 日志：`runtime/flight_bags/ros_logs/<run-id>/`；
- 宿主 tmux 日志：`~/uav-autonomy-aio_logs/<run-id>/`。

默认 rosbag 使用 LZ4 和 1 GiB 分卷，并为当前运行保留最新 10 个分卷，约
10 GiB。历史运行不会自动删除。启动前空间已经低于保留阈值时，整个栈会拒绝
启动；录制过程中跌破阈值时，只有 rosbag 停止，规划和控制继续运行。

记录内容包括定位、MAVROS 状态、控制输入输出、目标、规划轨迹、原始 Livox 数据、Planner 输入点云和膨胀地图。关闭原始点云或全部录制：

```bash
ROSBAG_RECORD_RAW_LIDAR=false ./launch/real.sh start
START_ROSBAG=false ./launch/real.sh start
```

`stop/restart` 会先向 rosbag 发送 `SIGINT`，默认最多等待 60 秒完成索引。无法确认索引的文件保留为 `.bag.active`，不会直接改名。

### 离线回放

真机栈必须先停止。播放指定 bag，或省略路径选择最新完成文件：

```bash
./launch/real_bag.sh play runtime/flight_bags/se3_test_YYYYMMDD_HHMMSS_0.bag
./launch/real_bag.sh status
./launch/real_bag.sh attach
./launch/real_bag.sh stop
```

播放脚本不会启动 MAVROS、Planner 或 SE3。在工作站仓库根目录打开离线 RViz：

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
./launch/real_bag_rviz.sh
```

离线 RViz 可显示原始扫描转换结果、Planner 输入点云和轨迹。显示中的累计地图只用于检查定位与障碍关系，不等同于全局高密度地图。

## 安全边界

- 遥控器和飞手始终拥有最终控制权；
- `real.sh arm` 会启动螺旋桨并自动交接到 OFFBOARD；
- `real.sh stop/restart` 不会降落；解锁或状态不可确认时脚本会拒绝执行，
  `--force` 只用于飞手已确保安全后的应急维护；
- 定位未启动、停更、时间戳不前进或异常，以及跳变和异常速度，会由公共保护节点
  锁存；监控不依赖 ROS 时间继续前进，并在自主模式下请求 `AUTO.LAND`。排查原因
  并重启完整栈后才能再次自主飞行；
- MAVROS 状态失联时软件无法确认恢复模式，飞手必须立即接管；
- 软件围栏不能替代 PX4 的 RC、Offboard、电池和估计器 failsafe；
- 目标发布成功不代表路径安全或可达；
- 更换载荷、动力系统或雷达安装位置后必须重新标定。

控制器原理与调参见 [SE3 控制器](se3_controller.md) 和 [控制器调参](controller_tuning.md)。
