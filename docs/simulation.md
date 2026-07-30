# PX4 / Gazebo 仿真

默认仿真栈包含 PX4 SITL、Gazebo Classic、模拟 MID-360、所选规划器、SE3 和 RViz。
MAVROS `map` 与 Gazebo `world` 由显式配置设为数值重合；点云使用采集时刻的 TF
转换到 `world`。

## 启动与飞行

`start`、`restart` 和 `test` 必须选择场景和规划器。仓库内置场景为 `room`、
`forest`，内置规划器为 `diff`、`fast-kino`、`fast-topo`、`super`；可用
`./launch/sim.sh planners` 查看当前实际发现的插件。

```bash
# 构建改动并启动 room + Diff；飞机保持未解锁
./launch/sim.sh --scene room --planner diff start

# PX4 原生起飞到相对 Home 1.5 m，稳定后进入 OFFBOARD
SIM_TAKEOFF_HEIGHT=1.5 ./launch/sim.sh arm

# 发送 world 目标；YAW_DEG 可省略，单位为度
./launch/sim.sh goal X Y Z [YAW_DEG]
./launch/sim.sh goal 1.0 0.0 1.5
./launch/sim.sh goal 1.0 1.0 1.5 90

# 请求 AUTO.LAND；stop 在仍处于 armed 时也会先尝试降落
./launch/sim.sh land

# 停止进程和容器，并优先等待 rosbag 正常关闭
./launch/sim.sh stop
```

`SIM_TAKEOFF_HEIGHT` 默认为相对 PX4 Home `1.0 m`。`arm` 完成前发送的目标不会排队。
`goal` 会先检查 armed/OFFBOARD、定位、控制输出和规划器状态，再调用所选插件校验目标。
命令成功表示目标已通过当时的校验并发布，不代表后续规划、重规划或飞行一定成功。

切换场景或规划器：

```bash
./launch/sim.sh --scene forest --planner fast-kino restart
./launch/sim.sh --scene forest --planner fast-topo restart
./launch/sim.sh --scene forest --planner super restart
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

`mission` 只在当前已启动的仿真栈上执行文件，不会根据文件名切换场景或规划器。Mission
在起飞前调用当前插件校验全部航点；滚动地图尚未覆盖远端航点时，仅暂缓
`goal_out_of_local_map`，每个航点发送前仍会再次校验。未解锁时自动起飞并进入
OFFBOARD。缺省 yaw 按有效水平航段生成；没有有效航段时沿用任务开始航向，显式
`yaw` 覆盖自动值。`fly_through` 控制中间点是否停稳，最终航点始终停稳；
`land_after_mission` 控制完成后是否降落。

人工切出 OFFBOARD、解除武装、状态或定位失联，以及无法恢复的规划故障会终止后续
航点。定位故障请求 `AUTO.LAND`；其他任务故障请求 `AUTO.LOITER`。飞行准备阶段若
无法确认 `AUTO.LOITER`，会继续尝试 `AUTO.LAND`；检测到人工接管后不会覆盖飞行
模式。

## RViz

RViz `2D Nav Goal` 使用固定高度 `SIM_RVIZ_GOAL_Z`，默认 `1.0 m`，箭头方向作为
终点 yaw。桥接器只在 armed/OFFBOARD、定位和规划器就绪且目标通过插件验证时发布
`/goal`。

| 显示 | Topic | 含义 |
|---|---|---|
| `Persistent obstacles` | `/planning/viz/environment` | 公共点云形成的显示专用累积环境 |
| `Observed obstacles` | `/planning/viz/occupancy` | 当前插件的统一占据显示 |
| `Safety clearance` | `/planning/viz/inflated_occupancy` | 当前插件的膨胀占据，按高度着色 |
| `Fixed planning bounds` | `/planning/viz/planning_bounds` | Fast 固定地图边界；Diff 和 SUPER 不发布固定边界 |
| `Active planner goal` | `/planning/viz/active_goal` | 当前插件状态报告的活动目标 |
| `Actual flight path` | `/planning/viz/executed_path` | armed 期间的实测里程计路径 |
| `Backend initial path` | `/planning/viz/backend/global_trajectory` | Diff 初始路径；Fast 和 SUPER 不发布 |
| `Backend trajectory` | `/planning/viz/backend/trajectory` | 当前插件的原生执行轨迹 |

`Persistent obstacles` 会排除无回波远端、地面和单次噪点。Fast 的公共占据显示使用
几何平面识别隐藏实测地面，不按固定 z 阈值删除低矮物体；这一显示过滤不改变规划
输入。Fast 注入私有规划点云的虚拟地面会参与规划和碰撞检查，但不会进入公共 RViz。
公共显示中的累积、过滤和着色均不修改规划地图。单帧 `Live lidar`
（`/localization/cloud_registered`）默认关闭。

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

rosbag 默认启动，使用 LZ4 和 1 GiB 分卷；`SIM_ROSBAG_MAX_SPLITS=10` 控制旧分卷
轮换。正在写入或最后关闭的分卷可能使文件数比该值多一个，因此它不是严格的总空间
上限。录制前默认要求至少 5 GiB 可用空间。`stop/restart` 优先发送 `SIGINT` 并等待
关闭；只有能被 `rosbag info` 读取的 `.bag.active` 才会改名为 `.bag`，否则保留原文件
并提示执行 `rosbag reindex`。历史运行不会自动删除。

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
| Diff 参数 | [`planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) |
| Fast Kino/Topo 参数 | [`planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/planner.yaml) |
| SUPER 参数 | [`planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml`](../planning/ros_pkgs/sim2real_super_adapter/config/planner.yaml) |
| `forest` 场景的 Fast 覆盖 | [`planning/ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml`](../planning/ros_pkgs/sim2real_fast_adapter/config/scenes/forest.yaml) |
| 公共控制参数 | [`common/config/controller.yaml`](../common/config/controller.yaml) |
| SUPER 控制覆盖 | [`planning/plugins/super/controller.yaml`](../planning/plugins/super/controller.yaml) |
| 仿真载体控制参数 | [`simulation/config/controller.yaml`](../simulation/config/controller.yaml) |

`SIM_PLANNER_CONFIG=容器内路径` 可为任一规划器临时加载一份完整配置，文件格式必须与
当前插件的默认配置一致。默认情况下，宿主机 `runtime/simulation/` 对应容器内
`/root/simulation_runtime/`。Fast 在 `forest` 仿真中会在完整配置之后加载场景覆盖，
因此覆盖文件中的同名参数最终生效。

SE3 参数按“公共配置 → 当前插件的 `controller_config` → 仿真载体配置”依次加载，
后加载的同名参数优先。`SIM_PLANNER_CONFIG` 只替换规划器配置，不替换这些控制参数。

挂载目录中的 ROS 源码、参数、场景或 `forest` world 修改后执行 `restart`。提交前可用
以下命令构建并测试五个隔离 workspace，同时校验所选插件和仿真 launch；它不会启动
飞行栈：

```bash
./launch/sim.sh --scene room --planner diff test
```

Dockerfile、`simulation/assets/`（包括 `room` 使用的默认 world）或
`simulation/versions.env` 修改后：

```bash
./launch/sim.sh stop
./launch/sim_container.sh build
./launch/sim_container.sh recreate
./launch/sim.sh --scene room --planner diff start
```

所有规划器默认共用镜像 `uav_autonomy_sim:noetic` 和仓库管理的运行容器
`uav_autonomy_sim`；`SIM_DEV_CONTAINER=名称` 只修改该容器名称，并不接受任意 ROS
容器布局。仍在运行的会话会根据所有权标记继续使用启动时的容器。切换规划器不更换
镜像或容器，只切换 manifest 指定的工作空间、后端参数和控制器覆盖。

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
- 规划器拒绝或急停：检查 `/planning/status.reason`、目标边界、占据和轨迹。
- 定位保护触发：检查里程计停更、时间戳和位置跳变；锁存后重启完整栈。
- GUI 异常：先关闭 Gazebo GUI 和 RViz，确认核心链路。
- 容器布局或 GPU 模式异常：运行 `./launch/sim_container.sh verify`；镜像不完整时
  先 `build`，镜像、挂载或 GPU 模式不一致时停止仿真后 `recreate`。
