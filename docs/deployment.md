# Jetson 真机部署

`deployment/` 目录包含项目的真机专用部分：Jetson Docker 镜像、Livox/FAST-LIO 适配、雷达网络和外参配置、真机控制器标定以及远程 RViz 配置。

> 启动或重启真机栈不会自动切换飞行模式或解锁。显式执行 `real.sh arm` 会请求 PX4 解锁并使用原生 `AUTO.TAKEOFF` 起飞到相对 Home 默认 `1.0 m`，随后自动进入经过 SE3 输出验证的 OFFBOARD 悬停。飞手切换到其他模式时脚本立即停止交接且不会抢回控制权。本文不能替代 PX4/QGC 校准、遥控器与 failsafe 配置、卸桨测试和现场安全规程。

## 数据链路

```text
MID-360
  → livox_ros_driver2
  → FAST-LIO
  → /Odometry + /cloud_registered
  → sim2real_deployment 适配
  → /localization/odom + /localization/cloud_registered
  → Diff-Planner → converter → SE3
  → MAVROS → PX4
```

公共里程计还会发布到 `/mavros/vision_pose/pose`。PX4 是否真正采用该定位，取决于飞控端 external-vision EKF 配置，必须单独验证。

## 前置条件

- Jetson 上安装 Docker、tmux；
- PX4 串口设备可访问，例如 `/dev/ttyACM0`；
- Jetson 与 MID-360 网络配置正确；
- Jetson、地面工作站和 QGC 网络互通；
- 已完成 PX4 external-vision EKF、遥控器和 failsafe 配置；
- 已测量 MID-360 内置 IMU 相对 `base_link` 的安装外参；
- 已针对实际机体标定悬停推力和控制参数。

本文的户外手机热点示例使用以下地址：

- Jetson 无线网络：`172.20.10.5`；
- 本机/地面工作站：`172.20.10.3`；
- MID-360 仍使用独立的有线雷达网段，不要改成热点地址。

所有命令都在 Jetson 的仓库根目录执行。

## 1. 修改机体配置

### Livox 网络

编辑：

```text
deployment/config/livox/MID360s_config.json
```

仓库当前默认值：

- Jetson 雷达网卡：`192.168.1.101`；
- MID-360：`192.168.1.199`。

硬件地址不同时必须修改该文件。

### 控制器标定

编辑：

```text
deployment/config/controller.yaml
```

至少重新确认：

- `hover_percent`；
- `ki_pz`；
- `min_output_thrust` / `max_output_thrust`；
- `geo_fence`。

仓库中的值只代表当前机体基线，不能直接用于不同电机、桨、电池、重量或载荷。

### 雷达外参

默认外参通过环境变量传给 `real.sh`：

```text
MOUNT_X=0.109
MOUNT_Y=0.024
MOUNT_Z=0.006
MOUNT_ROLL_DEG=0.7
MOUNT_PITCH_DEG=28.1
MOUNT_YAW_DEG=0.5
```

`MOUNT_*` 表示 `base_link → FAST-LIO body`，其中 FAST-LIO 的 `body` 是 MID-360
内置 IMU 原点，不是点云原点。上面的默认值是根据当前机体静态实测对齐结果以及
FAST-LIO 的雷达到 IMU 内参换算得到的基线，仍需在多个朝向下验证。

如果只能量到点云原点，不能直接把尺量结果填入 `MOUNT_X/Y/Z`。设量得的
`base_link → LiDAR` 平移为 `t_BL`，FAST-LIO 配置中的 LiDAR→IMU 平移为
`t_IL`，则应使用 `t_BI = t_BL - R_BI t_IL` 换算到内置 IMU 原点，其中
`R_BI t_IL` 表示旋转矩阵与平移向量相乘。

## 2. 构建并创建容器

```bash
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh run
```

默认镜像和容器名都是 `diff_planner_px4_real`。`run` 会映射：

- PX4 串口设备；
- Livox 和控制器配置；
- `runtime/flight_bags/`；
- X11（如果宿主可用）。

检查容器：

```bash
./launch/real_container.sh status
./launch/real_container.sh shell
```

## 3. 启动真机 ROS 栈

```bash
FCU_URL='/dev/ttyACM0:921600' \
GCS_URL='udp://:14555@172.20.10.3:14550' \
ROS_IP=172.20.10.5 \
MAVROS_TGT_SYSTEM=5 \
MOUNT_X=0.109 \
MOUNT_Y=0.024 \
MOUNT_Z=0.006 \
MOUNT_ROLL_DEG=0.7 \
MOUNT_PITCH_DEG=28.1 \
MOUNT_YAW_DEG=0.5 \
PLANNER_RESOLUTION=0.11 \
PLANNER_OBSTACLES_INFLATION=0.33 \
./launch/real.sh start
```

`MAVROS_TGT_SYSTEM` 必须与 PX4 参数 `MAV_SYS_ID` 一致。

上例显式使用公共默认值 `0.11/0.33 m`。`0.65 m` 正方形机体的旋转包络半径为 `0.325 m`，因此只剩 `0.005 m` 理论余量。膨胀层数按 `ceil((inflation-1e-5)/resolution)` 取整，这组数正好是 3 层，不会意外量化成更大的半径；不传这两个变量时使用相同的公共默认值。

启动器会依次启动并检查：

1. ROS Master 和静态坐标别名；
2. Livox 驱动；
3. FAST-LIO；
4. 公共定位、动态 `world → base_link` TF 和点云适配；
5. MAVROS；
6. vision pose 回灌；
7. Diff-Planner、轨迹转换和 SE3；
8. rosbag 录制。

任一阶段超时都会清理本次部分启动。

管理命令：

```bash
./launch/real.sh status   # 查看容器、tmux 窗口和进程状态
./launch/real.sh attach   # 进入 tmux，查看各节点的实时输出
./launch/real.sh stop     # 停止完整 ROS 栈并清理相关进程
./launch/real.sh restart  # 停止后重新启动完整 ROS 栈（不会重建镜像）
```

`status` 只检查容器、tmux 和进程，不代表传感器方向、定位质量或飞行状态正确。

## 4. 真机控制命令

### 解锁并起飞

确认场地净空、遥控器 Kill 已解除并完成飞前检查，然后执行：

```bash
./launch/real.sh arm
```

该命令执行以下流程：

1. 在同一个常驻 ROS 进程中并行检查 MAVROS 状态、相对高度、新鲜定位、SE3 节点和 10 个连续 OFFBOARD 预热 setpoint；
2. 读取并保存 `NAV_MC_ALT_RAD`，起飞期间临时将其收紧到高度容差（默认 `0.1 m`），将 `MIS_TAKEOFF_ALT` 直接设置为 `REAL_TAKEOFF_HEIGHT`，并将 `COM_TAKEOFF_ACT` 设置为 `0`；
3. 复用同一组 ROS 服务代理，通过 `/mavros/cmd/arming` 请求 PX4 解锁，并在实际状态确认解锁后立即请求原生 `AUTO.TAKEOFF`；
4. 连续监测 `/mavros/state`、`/mavros/altitude/relative` 和定位垂直速度；高度进入目标容差且垂直速度稳定后，恢复原 `NAV_MC_ALT_RAD`，再请求 OFFBOARD，并用 5 个新的 SE3 姿态/推力 setpoint 验证交接；真机可保持在 `AUTO.TAKEOFF`，不依赖自动进入 `AUTO.LOITER`；

真机飞行记录表明，外部请求的 PX4 `AUTO.TAKEOFF` 会直接追踪 `MIS_TAKEOFF_ALT`，而当前 SITL 会利用较大的 `NAV_MC_ALT_RAD` 提前判定到达。共享状态机因此在两端都临时使用 `0.1 m` 接受半径，并把 `1.0 m` 直接写入 `MIS_TAKEOFF_ALT`；结束或异常退出时均尝试恢复原值。

飞前数据检查、PX4 参数操作、解锁、`AUTO.TAKEOFF`、高度监测与 OFFBOARD 交接都由同一个 `arm_executor.py` 进程完成，不再逐项拆成 `docker exec`：这样可避免 Jetson 负载较高时，容器 shell 和 ROS discovery 的重复启动延迟耗尽 PX4 的飞前自动锁定窗口。安全门禁、10 个位置预热 setpoint 和 5 个姿态/推力 setpoint 检查仍然保留；脚本不会为此修改 `COM_DISARM_PRFLT`。

起飞达到高度后不会在仍有明显上升速度时直接切换控制器。脚本要求相对高度处于目标 `+/-0.1 m`，并确认垂直速度默认连续 `0.5 s` 不超过 `0.2 m/s`，再从 PX4 原生 `AUTO.TAKEOFF/AUTO.LOITER` 切到 `OFFBOARD`；切入瞬间 SE3 锁定当前位姿，并在观察到连续姿态/推力输出后返回，但不会执行起飞前发布的目标。仿真和真机入口均执行这一份共享状态机。

### 发布规划目标

完成 `real.sh arm` 并确认已经在 OFFBOARD 悬停后，发布世界坐标系目标：

```bash
./launch/real.sh goal 1.0 0.0 1.0       # 终点 yaw 不限定
./launch/real.sh goal 1.0 0.0 1.0 0     # 终点 yaw = 0 deg
```

命令格式为 `goal X Y Z [YAW_DEG]`。省略 yaw 表示不限定终点朝向，由轨迹方向决定 yaw；提供 yaw 时单位为度。

仿真和真机的 `goal` 都执行同一份 `goal_executor.py`。它用一个 ROS 进程并行确认 MAVROS 已连接、飞机 armed + OFFBOARD、定位新鲜、10 条连续 SE3 姿态/推力 setpoint、Planner 节点、目标高度围栏以及 Planner 与轨迹转换器两个 `/goal` 消费者，然后由同一个发布器发送 `geometry_msgs/PoseStamped`。因此保留了原有安全门禁，同时消除了逐项启动 `docker exec + rostopic/rosnode/rosparam` 的延迟。

该目标不会直接发送给 PX4：Diff-Planner 还会检查目标相对当前里程计位置的三维直线距离不超过默认 `200 m`、地图边界和膨胀障碍物，生成轨迹后由转换节点发布 `/command/trajectory`，最后由 SE3 生成 MAVROS 姿态/推力指令。`200 m` 只是单次目标输入保护，不代表 Planner 已经掌握或保证整段全局无碰撞路线。

命令成功返回只代表 `/goal` 已发布，不代表 Planner 一定接受或飞机已经到达；应同时观察 Planner 日志和规划轨迹。

### 顺序航点任务

复制并修改示例任务：

```bash
cp common/config/mission.example.json mission_outdoor.json
```

任务文件的核心结构如下：

```json
{
  "takeoff_height": 1.0,
  "takeoff_settle_time": 0.0,
  "land_after_mission": true,
  "fly_through": true,
  "fly_through_tolerance": 0.5,
  "waypoints": [
    {"x": 1.0, "y": 0.0, "z": 1.0},
    {"x": 1.0, "y": 1.0, "z": 1.0},
    {"x": 0.0, "y": 1.0, "z": 1.0}
  ]
}
```

执行：

```bash
./launch/real.sh mission mission_outdoor.json
```

`x/y/z` 是 `world` 中的绝对目标位置，数组顺序就是执行顺序。Mission 会自动计算 yaw：每个中间航点朝向“当前点 → 下一点”，最后一点保持“倒数第二点 → 最后一点”的进入方向，因此任务文件通常不需要填写 yaw；个别航点仍可用显式 `yaw`（度）覆盖自动值。`takeoff_height` 是仅在飞机尚未解锁时使用的 PX4 Home 相对起飞高度，不是 `world.z`。系统始终先等待 PX4 原生起飞进入目标高度容差且垂直速度稳定；`takeoff_settle_time` 是在此之后可选的额外悬停时间，默认为 `0.0 s`。

`fly_through=true` 表示中间航点是连续飞行的切换面：飞机必须同时进入 `fly_through_tolerance`（当前 `0.5 m`）并满足自动计算 yaw 的容差（当前 `30 deg`），才发送下一航点；它不等待低速或 `hold_time`，因此对准下一航段后继续飞行而不停车。没有有效水平航段时，缺省 yaw 会在运行时锁定为当前朝向。最终航点始终使用位置、速度、yaw 和稳定时间判定，以保证自动降落前已经停稳。需要在某个中间点停留时，可在该航点设置 `"fly_through": false`。

自动 yaw 不使用水平距离小于 `fly_through_tolerance` 的近重合点对：它会继续向后寻找第一个足够远的航点，任务末尾的近重合点沿用最近的有效航段方向。若整条任务的 x/y 变化都小于该阈值（例如纯垂直任务），Mission 会在获得新鲜定位后把所有缺省 yaw 锁定为任务开始时的当前朝向，不会根据坐标噪声生成任意角度。

仿真和真机的 `mission` 都直接执行同一个 `common/scripts/mission_executor.py`。Shell 只负责把同一执行器、同一航点 runner 和 JSON 放进各自容器；以下状态机没有仿真/真机分支：

1. 在解锁前创建 Planner 发布器和全部订阅，完整校验 JSON、坐标和 Planner 高度围栏，同时预热首航点链路；
2. 未解锁：使用 PX4 原生 `AUTO.TAKEOFF`，高度和垂直速度稳定后，按 `takeoff_settle_time` 追加可选等待，再请求 OFFBOARD；
3. 已经处于 armed OFFBOARD：验证新鲜控制输出后直接开始航点任务；已解锁但不在 OFFBOARD 则视为飞手控制并中止；
4. 发布航点并等待带相同目标时间戳的有效 Planner 轨迹；若 Planner 返回安全替代点，接受该坐标作为当前实际航点；
5. 根据相邻航点自动生成 yaw，中间 fly-through 航点同时满足切换半径和 yaw 后立即发布下一点；非 fly-through 点和最终点才等待停稳；
6. 所有航点成功后统一请求 PX4 `AUTO.LAND`，不强制解除锁定。

当前两份任务的 fly-through 半径和最终位置容差均为 `0.5 m`，yaw 容差为 `30 deg`；最终点还要求 `0.15 m/s` 速度阈值和 `0.5 s` 稳定时间。Planner 发布临时急停轨迹后默认允许 `2.0 s` 自动恢复；若停稳后规划器需要新目标，Mission 会用新时间戳重新发送当前实际航点，默认最多 `3` 次。完整字段见示例文件。

飞手在自动起飞阶段切换到 `AUTO.TAKEOFF/AUTO.LOITER` 以外的模式，或在航点阶段切出 OFFBOARD，任务都会立即中止，不发送后续航点，也不再请求 OFFBOARD 或自动降落。旧目标缓存同时失效，因此之后重新进入 OFFBOARD 不会恢复旧任务；需要重新执行 `mission` 或发布新目标。Planner 因临近碰撞发布临时急停轨迹时，任务先等待其自动重规划；若规划器停稳并清空目标，则有限次数重发当前实际航点。恢复正常轨迹后继续执行；重试耗尽、定位超时或航点超时时，任务才请求一次 PX4 `AUTO.LOITER` 并停止，要求飞手接管。任务命令应保持在前台运行；终端收到 `SIGINT/SIGTERM` 时也按失败处理并尝试进入 Hold。

### 自动降落

请求 PX4 自动降落：

```bash
./launch/real.sh land
```

`land` 不经过 Planner，也不生成 SE3 下降轨迹，而是通过 `/mavros/set_mode` 直接请求 PX4 `AUTO.LAND`，并等待 `/mavros/state.mode` 确认切换成功。下降速度、接地检测和落地后的自动解除锁定由 PX4 负责；脚本不会强制解除锁定。飞机已经解除锁定时该命令直接返回，模式请求超时则报错。降落全程保持遥控器接管能力。

## 5. 远程 RViz

工作站需要一个带 RViz 的 ROS Noetic GUI 容器，默认名称为 `ros_noetic`。

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
RVIZ_GOAL_Z=1.0 \
RVIZ_GOAL_FRAME=world \
./launch/real_rviz.sh
```

脚本会：

- 连接 Jetson 的 ROS Master；
- 更新 GUI 容器中的 Jetson 主机映射；
- 将 RViz 专用 `/sim2real/rviz_goal` 转发到 `/goal`；
- 打开真机 RViz 配置。

独立输入话题用于避开 MAVROS 自己发布的 `/move_base_simple/goal`，防止飞控消息被误认为 RViz 点击目标。

当前 Planner 启用了 `virtual_ground=0.1 m` 和 `virtual_ceil=3.0 m`，规划目标必须满足 `0.1 < Z < 3.0 m`。`deployment/config/controller.yaml` 中的 SE3 `geo_fence` 当前为 `x/y=50 m、z=3 m`，但公共配置保持 `auto_land_on_geofence=false`，因此超限只记录警告，不限制轨迹，也不会自动降落。

关闭 RViz 不会自动停止后台目标桥；再次运行脚本会先清理同名旧节点。

在解锁和切换 OFFBOARD 前至少确认：

1. `/livox/lidar` 类型为 `livox_ros_driver2/CustomMsg`，`/livox/imu` 连续；
2. FAST-LIO `/Odometry` 连续，点云方向和尺度正确；
3. `/localization/odom` 的 pose 在 `world`，twist 在 `base_link`；
4. `/localization/cloud_registered` 位于 `world`，障碍与里程计对齐；
5. `/mavros/state` 显示 `connected: True`，且飞前 `system_status` 不能为 `8`（`FLIGHT_TERMINATION`）；如果为 `8`，优先检查遥控器 kill switch；
6. `/mavros/vision_pose/pose` 连续，PX4 EKF 已确认采用 external vision；
7. `/mavros/local_position/odom` 新鲜且 `/mavros/setpoint_position/local` 持续发布预热 setpoint；控制器不会在定位缺失时发送虚构位置；
8. `real.sh arm` 完成并确认自动 OFFBOARD 后，再发布一个全新的 `/goal`；OFFBOARD 前的目标不会排队；
9. 飞手可随时通过遥控器切回人工模式。

在真机容器中检查：

```bash
./launch/real_container.sh shell

rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic echo -n1 /mavros/state
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/setpoint_position/local
rosrun tf tf_echo world base_link
rosrun tf tf_echo world body
```

切入 OFFBOARD 后再确认：

```bash
rostopic hz /mavros/setpoint_raw/attitude
```

OFFBOARD 前通过 RViz 或 `rostopic pub /goal` 发布的目标会被真机转换节点忽略，不会在切换后自动执行。进入 OFFBOARD 后必须重新发布目标；仍不要把 RViz 点击当作无副作用的预览，因为一旦已经处于 armed OFFBOARD，目标会立即进入规划流程。

## 更新代码或配置

修改 Dockerfile、镜像内源码或 ROS 消息后：

```bash
./launch/real.sh stop
./launch/real_container.sh build
FCU_DEVICE=/dev/ttyACM0 ./launch/real_container.sh restart
./launch/real.sh start
```

`real.sh restart` 只重启 ROS 栈，不会重建镜像。`real_container.sh restart` 会重新创建容器，但也不会自动重新构建镜像。

只修改通过 bind mount 加载的 Livox JSON 或控制器 YAML 时，不需要重建镜像或容器，执行 `./launch/real.sh restart` 让节点重新加载配置即可。

## 常用环境变量

| 变量 | 默认值 | 作用 |
|---|---|---|
| `FCU_DEVICE` | `/dev/ttyACM0` | 映射到容器的 PX4 设备 |
| `FCU_URL` | 空 | MAVROS 飞控 URL，建议显式设置 |
| `GCS_URL` | 空 | MAVROS 到 QGC 的连接，建议显式设置 |
| `ROS_IP` | 自动探测 | Jetson 在 ROS1 网络中的地址 |
| `MAVROS_TGT_SYSTEM` | `5` | MAVROS 连接的飞控 system ID，必须与 PX4 `MAV_SYS_ID` 一致；当前真机为 `5` |
| `MOUNT_*` | 见上文 | `base_link → MID-360 内置 IMU（FAST-LIO body）` 外参 |
| `DRONE_ID` | `0` | Planner topic 使用的无人机 ID |
| `START_ROSBAG` | `true` | 是否启动默认调试 rosbag |
| `ROSBAG_RECORD_RAW_LIDAR` | `true` | 是否录制原始密度 `/livox/lidar`；离线时可等密度转换为 PointCloud2 |
| `ROSBAG_SPLIT_SIZE_MB` | `5120` | 单个分卷 bag 的最大容量，单位 MB（约 5 GB） |
| `ROSBAG_NICE_LEVEL` | `10` | rosbag 进程 nice 值，保证规划和控制优先 |
| `ROSBAG_MIN_FREE_GB` | `5` | 低于该剩余空间时停止录包；`0` 表示关闭保护 |
| `ROSBAG_STOP_TIMEOUT` | `60` | 停止时等待 rosbag 写完索引并完成分卷重命名的最长秒数 |
| `REAL_COMMAND_TIMEOUT` | `15` | PX4 解锁、AUTO.TAKEOFF 和 AUTO.LAND 模式请求的超时秒数 |
| `REAL_TAKEOFF_HEIGHT` | `1.0` | 期望实际 Home 相对起飞高度，并直接写入 `MIS_TAKEOFF_ALT` |
| `REAL_TAKEOFF_TIMEOUT` | `30` | 等待本地高度达到预期起飞高度的最长秒数；超时不自动改模式 |
| `REAL_TAKEOFF_TOLERANCE` | `0.1` | 脚本对实际相对高度的监测容差，不修改 PX4 接受半径 |
| `REAL_TAKEOFF_STABLE_TIME` | `0.5` | 高度处于目标容差内时，垂直速度必须连续满足阈值的秒数 |
| `REAL_TAKEOFF_MAX_VERTICAL_SPEED` | `0.2` | OFFBOARD 交接前允许的最大垂直速度绝对值，单位 `m/s` |
| `REAL_PREFLIGHT_TIMEOUT` | `5.0` | 并行等待状态、定位、高度和连续 setpoint 的最长秒数 |
| `START_DIFF_PLANNER` | `true` | 是否启动 Planner |
| `START_SE3_CONTROLLER` | 跟随 Planner | 是否启动 SE3 控制器 |
| `PLANNER_CONFIG` | 空 | 可选 Planner 完整 YAML 覆盖 |
| `PLANNER_RESOLUTION` | 空 | 启动时临时覆盖 `grid_map/resolution`；与 `PLANNER_CONFIG` 互斥 |
| `PLANNER_OBSTACLES_INFLATION` | 空 | 启动时临时覆盖 `grid_map/obstacles_inflation`；与 `PLANNER_CONFIG` 互斥 |

多网卡环境应显式设置 `ROS_IP`。重建容器或重启栈时，需要继续传入相同的非默认设备、网络和外参变量。

## 日志与 rosbag

- 真机 rosbag：`runtime/flight_bags/`；
- 容器 ROS 日志：`runtime/flight_bags/ros_logs/<run-id>/`；
- 宿主 tmux 日志：`~/diff-planner-px4-deployment_logs/<run-id>/`。

默认调试 rosbag 使用 LZ4 压缩和约 5 GB 分卷，记录：

- 定位、MAVROS 状态、控制输入输出和目标点；
- Livox 原始密度点云 `/livox/lidar`；
- FAST-LIO 去畸变、未经过 `0.5 m` 体素滤波的较高密度点云 `/cloud_registered_body`；
- 规划器实际使用的降采样注册点云 `/localization/cloud_registered`；
- 规划多项式轨迹、位置指令和规划诊断；
- 2 Hz 膨胀地图 `/drone_0_diff_planner_node/grid_map/occupancy_inflate`。

膨胀地图的 2 Hz 只影响 RViz/rosbag 可视化输出，不降低规划器内部地图更新频率。录包进程默认以较低 CPU 优先级运行；磁盘剩余空间小于 5 GB 时，rosbag 会自动停止录制，但不会停止规划和控制。

`/livox/lidar` 是 Livox `CustomMsg`，不能由 RViz 直接显示；离线回放脚本会把它逐点转换为 `/livox/lidar_points`，点数和密度不变。一次实测帧在转换前后均为 `20064` 点。

`/cloud_registered_body` 在 FAST-LIO 预处理阶段受 `point_filter_num=3` 影响，约保留三分之一有效点，但没有再经过 `0.5 m` 体素滤波，明显密于规划器使用的 `/localization/cloud_registered`。该点云位于 FAST-LIO `body` 坐标系；bag 同时记录 `/tf`，离线时可按消息时间转换到 `world`，因此可以用于重建较高密度的场景地图，而不会增加规划器输入密度。

如果不需要原始密度或离线重跑 FAST-LIO，可以关闭原始点云录制：

```bash
ROSBAG_RECORD_RAW_LIDAR=false ./launch/real.sh start
```

不要同时额外录制 `/cloud_registered` 和 `/localization/cloud_registered`，两者内容基本重复。关闭所有 rosbag 录制：

```bash
START_ROSBAG=false ./launch/real.sh start
```

FAST-LIO 的 PCD 保存默认关闭，避免长时间运行时无界累积点云。

执行 `./launch/real.sh stop` 或 `restart` 时，脚本会先向 rosbag 发送 `SIGINT`，最多等待 60 秒完成索引和从 `.bag.active` 到 `.bag` 的重命名，再停止其他节点。若进程已经退出但文件索引完整，脚本会自动补上最终重命名；无法读取索引的文件会保留 `.active` 并提示使用 `rosbag reindex`，不会盲目改名。

### 离线回放与专用 RViz

真机栈必须先停止。Jetson 上播放指定 bag；省略路径时自动选择最新的已完成 `.bag`：

```bash
./launch/real.sh stop
./launch/real_bag.sh play runtime/flight_bags/se3_test_YYYYMMDD_HHMMSS_0.bag
```

播放脚本启动独立 `roscore`（或复用未运行飞行节点的现有 master）、点云格式转换和 `rosbag play`，不会启动 MAVROS、Planner 或 SE3。管理命令：

```bash
./launch/real_bag.sh status
./launch/real_bag.sh attach
./launch/real_bag.sh stop
```

工作站使用离线专用 RViz，不启动目标桥：

```bash
CONTAINER_NAME=ros_noetic \
JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
./launch/real_bag_rviz.sh
```

RViz 默认显示两类点云：由 `/livox/lidar` 等密度转换得到的 `/livox/lidar_points` 用于查看当前原始扫描；已经配准到 `world` 的 `/localization/cloud_registered` 会在 RViz 中累计 3600 秒，用于还原飞行经过区域的场景地图。配置中还提供默认关闭的 `/cloud_registered_body` 显示项，用于逐帧检查密度更高的去畸变点云。原始点云转换只改变 ROS 消息格式，不进行降采样；回放使用录制的 TF 将 `body` 和 `livox_frame` 点云转换到 `world`。

累计场景地图使用规划器实际接收的点云，因此受到 FAST-LIO 当前 `0.5 m` 体素参数影响，适合检查定位、障碍物和规划关系，不等同于原始密度地图。超过一小时的 bag 会逐步淘汰更早的 RViz 显示数据；这只影响显示，不影响 bag 中已经记录的消息。需要制作高密度地图时，应使用 bag 中的 `/livox/lidar`、IMU 和定位数据离线重建，而不是在飞行时反复记录不断增大的全局地图。

可用环境变量调整播放：

```bash
BAG_RATE=0.5 BAG_LOOP=true ./launch/real_bag.sh play <bag-file>
```

建议把 `runtime/` 放在独立 NVMe 上。创建真机容器时可以指定：

```bash
RUNTIME_DIR=/mnt/nvme/diff-planner-runtime \
FCU_DEVICE=/dev/ttyACM0 \
./launch/real_container.sh run
```

修改默认规划参数或规划器源码后需要重新构建真机镜像；仅调整上述 rosbag 环境变量不需要重建镜像。

## 安全边界

- 飞手和遥控器始终拥有最终控制权；`real.sh arm` 会调用 MAVROS 解锁服务并请求 PX4 `AUTO.TAKEOFF`，执行前必须解除 Kill 并确认场地净空；
- `real.sh arm` 会在达到实际起飞高度后自动进入 OFFBOARD 并验证 SE3 输出；飞手切到其他模式即被视为人工接管，脚本不会抢回控制权；
- PX4 的 RC、Offboard、电池和估计器 failsafe 必须在飞控端配置并实测；
- SE3 围栏默认只检测超限，不能替代 PX4 failsafe；
- 首次测试应卸桨验证方向、topic 和模式切换，再进行受控低空飞行；
- 更换载荷、电池、动力系统或雷达安装位置后，必须重新标定相关参数。

进一步调参参考：

- [SE3 控制器说明](se3_controller.md)
- [竖直积分增益整定](ki_pz_tuning_guide.md)
- [轨迹跟踪掉高排查](trajectory_tracking_altitude.md)
