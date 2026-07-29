# PX4 / Gazebo 仿真

仿真栈包含 PX4 SITL、Gazebo Classic、模拟 MID-360、所选规划器、SE3 和 RViz。
MAVROS `map` 与 Gazebo `world` 由显式配置设为数值重合；点云使用采集时刻的 TF
转换到 `world`。

## 启动与飞行

`start` 和 `restart` 必须同时提供场景和规划器。内置场景为 `room`、`forest`，
规划器为 `diff`、`fast-kino`、`fast-topo`。

```bash
# 构建改动并启动 room + Diff；飞机保持未解锁
./launch/sim.sh --scene room --planner diff start

# PX4 原生起飞到相对 Home 1.5 m，稳定后进入 OFFBOARD
SIM_TAKEOFF_HEIGHT=1.5 ./launch/sim.sh arm

# 发送 world 目标；YAW_DEG 可省略，单位为度
./launch/sim.sh goal X Y Z [YAW_DEG]
./launch/sim.sh goal 1.0 0.0 1.5
./launch/sim.sh goal 1.0 1.0 1.5 90

# 请求 AUTO.LAND；落地后停止进程、容器并完成 rosbag 索引
./launch/sim.sh land
./launch/sim.sh stop
```

`SIM_TAKEOFF_HEIGHT` 默认为相对 PX4 Home `1.0 m`。`arm` 完成前发送的目标不会排队。
目标命令成功只表示目标已发布；插件仍可能因边界、障碍或规划失败而拒绝执行。

切换场景或规划器：

```bash
./launch/sim.sh --scene forest --planner fast-kino restart
./launch/sim.sh --scene forest --planner fast-topo restart
```

自动化脚本可用 `SIM_SCENE`、`SIM_PLANNER` 代替对应选项。场景配置位于
`simulation/config/scenes/NAME.env`，负责 world 和出生位姿，不修改控制器或飞行状态机。

## Mission

```bash
# room 航点
./launch/sim.sh mission mission_indoor.json

# forest 航点
./launch/sim.sh mission mission_forest.json
```

Mission 在起飞前调用当前插件校验全部航点；未解锁时自动起飞并进入 OFFBOARD。
缺省 yaw 按有效航段生成，显式 `yaw` 覆盖自动值。`fly_through` 控制中间点是否停稳，
`land_after_mission` 控制完成后是否降落。

人工切出 OFFBOARD、解除武装、状态/定位失联或规划失败会终止后续航点。定位故障请求
`AUTO.LAND`；其他任务故障优先请求 `AUTO.LOITER`。

## RViz

RViz `2D Nav Goal` 使用固定高度 `SIM_RVIZ_GOAL_Z`，默认 `1.0 m`，箭头方向作为
终点 yaw。桥接器只在 armed/OFFBOARD、定位和规划器就绪且目标通过插件验证时发布
`/goal`。

| 显示 | Topic | 含义 |
|---|---|---|
| `Persistent obstacles` | `/planning/viz/environment` | 公共点云形成的显示专用累积环境 |
| `Observed obstacles` | `/planning/viz/occupancy` | 当前插件的统一占据显示 |
| `Safety clearance` | `/planning/viz/inflated_occupancy` | 当前插件的膨胀占据，按高度着色 |
| `Fixed planning bounds` | `/planning/viz/planning_bounds` | Fast 固定地图边界；Diff 不发布固定边界 |
| `Active planner goal` | `/planning/viz/active_goal` | 当前插件实际接受的目标 |
| `Actual flight path` | `/planning/viz/executed_path` | armed 期间的实测里程计路径 |
| `Backend initial path` | `/planning/viz/backend/global_trajectory` | Diff 初始路径；Fast 当前不发布 |
| `Backend trajectory` | `/planning/viz/backend/trajectory` | 当前插件的原生执行轨迹 |

`Persistent obstacles` 会排除无回波远端、地面和单次噪点。Fast 的公共占据显示使用
几何平面识别隐藏实测地面，不按固定 z 阈值删除低矮物体；注入 Fast 私有地图的虚拟
地面不会进入公共 RViz。上述处理只影响显示，不改变规划地图或碰撞检查。原始单帧
MID-360 点云默认关闭。

## 状态与日志

```bash
./launch/sim.sh status   # 汇总容器、tmux、ROS、规划器和 MAVROS 状态
./launch/sim.sh attach   # 查看 tmux 实时日志；Ctrl-b d 退出
./launch/sim.sh shell    # 进入仿真容器
```

每次运行的数据位于：

```text
runtime/simulation/runs/<run-id>/
runtime/simulation/flight_bags/se3_test_<run-id>_*.bag
```

rosbag 默认使用 LZ4、1 GiB 分卷，当前运行最多保留 10 卷。`stop/restart` 先发送
`SIGINT` 并等待索引；历史运行不会自动删除。

```bash
# 禁用录包
SIM_START_ROSBAG=false \
./launch/sim.sh --scene room --planner diff restart

# 不记录原始 MID-360 点云
SIM_ROSBAG_RECORD_RAW_LIDAR=false \
./launch/sim.sh --scene room --planner diff restart

# 修改分卷大小和数量
SIM_ROSBAG_SPLIT_SIZE_MB=512 SIM_ROSBAG_MAX_SPLITS=8 \
./launch/sim.sh --scene room --planner diff restart
```

## 无界面与渲染

```bash
# 关闭 Gazebo GUI 和 RViz
SIM_GAZEBO_GUI=false SIM_START_RVIZ=false \
./launch/sim.sh --scene room --planner diff restart

# 强制软件渲染
./launch/sim.sh stop
SIM_GPU_MODE=none ./launch/sim_container.sh recreate
SIM_GPU_MODE=none ./launch/sim.sh --scene room --planner diff start
```

## 配置与生效

| 内容 | 文件 |
|---|---|
| 场景 | [`simulation/config/scenes`](../simulation/config/scenes) |
| Diff | [`planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) |
| Fast Kino/Topo | [`planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml) |
| `forest` Fast 覆盖 | [`planning/ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml) |
| 公共控制 | [`common/config/controller.yaml`](../common/config/controller.yaml) |
| 仿真载体控制 | [`simulation/config/controller.yaml`](../simulation/config/controller.yaml) |

ROS 源码、参数、场景或 world 修改后执行 `restart`。Dockerfile、
`simulation/assets/` 或 `simulation/versions.env` 修改后：

```bash
./launch/sim.sh stop
./launch/sim_container.sh build
./launch/sim_container.sh recreate
./launch/sim.sh --scene room --planner diff start
```

## 排错

```bash
./launch/sim.sh status
./launch/sim.sh attach
./launch/sim.sh shell
```

进入容器后检查：

```bash
rostopic hz /localization/odom
rostopic hz /localization/cloud_registered
rostopic echo -n1 /planning/status
rostopic echo -n1 /mavros/state
rostopic hz /mavros/setpoint_raw/attitude
```

- 目标无响应：确认 `arm` 已完成、PX4 为 armed/OFFBOARD，再重新发送目标。
- Planner 拒绝或急停：检查 `/planning/status.reason`、目标边界、占据和轨迹。
- 定位保护触发：检查里程计停更、时间戳和位置跳变；锁存后重启完整栈。
- GUI 异常：先关闭 Gazebo GUI 和 RViz，确认核心链路。
- 容器布局或 GPU 过期：运行 `./launch/sim_container.sh verify`，失败后重建容器。
