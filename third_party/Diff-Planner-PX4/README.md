# Diff-Planner-PX4（本仓库维护版本）

本目录基于 [DifferentialRobotics/Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner) 和 [Tfly6/Diff-Planner-PX4](https://github.com/Tfly6/Diff-Planner-PX4) 修改，包含 Diff-Planner、轨迹消息/工具、可视化组件和 SE3 控制器。它是本仓库的算法源码，不再单独保存仿真/真机两套顶层 launch。

仓库级使用说明见 [../../README.md](../../README.md)。顶层启动统一由公共 `sim2real_common` launch 和仓库根目录 `launch/` 编排；本目录不提供环境专用入口。

## 包结构

- `src/diff_planner/`：局部占据地图、路径搜索、轨迹优化、规划 FSM、轨迹服务和多机相关组件；
- `src/se3_controller/`：把期望轨迹转换为 MAVROS 姿态和归一化推力；
- `src/user_command/`：多点任务等可选用户命令层；
- `src/Utils/`：消息、可视化与通用工具。

支持本仓库使用的 Ubuntu 20.04 / ROS Noetic。仿真与真机镜像会按各自 Dockerfile 安装依赖并构建该目录；推荐使用仓库根目录 `launch/` 入口，而不是把本目录单独复制到另一套 PX4 环境。

## 在本项目中的位置

环境适配层必须先提供：

```text
/localization/odom              nav_msgs/Odometry，pose 在 world、twist 在 base_link
/localization/cloud_registered  sensor_msgs/PointCloud2，frame_id=world
```

然后两端共同运行：

```text
sim2real_common/planner.launch
  ├─ diff_planner_node
  └─ traj_server
       -> /drone_0_planning/pos_cmd

sim2real_common/trajectory_converter.launch
  -> /command/trajectory

sim2real_common/controller.launch
  -> se3_controller_node
  -> /mavros/setpoint_raw/attitude
```

公共 Planner 参数唯一默认来源是 [../../common/config/planner.yaml](../../common/config/planner.yaml)。本目录内的上游示例参数不是仓库顶层链路的配置入口，不应继续按环境分叉。

## 运行

仿真：

```bash
cd ../..
./launch/sim.sh start
./launch/sim.sh test
```

真机：

```bash
cd ../..
./launch/real_container.sh build
./launch/real_container.sh run
./launch/real.sh start
```

## 主要 ROS 接口

### Diff-Planner 输入

| Topic | 作用 |
|---|---|
| `/goal` | `geometry_msgs/PoseStamped` 目标位置和最终 yaw |
| `/localization/odom` | 公共机体里程计；FSM 和 grid map 共用 |
| `/localization/cloud_registered` | 公共世界系注册点云 |
| `/mandatory_stop_to_planner` | 外部强制停止 |
| `/traj_start_trigger` | 预设 waypoint 模式的可选触发 |

### Diff-Planner 输出

| Topic | 作用 |
|---|---|
| `/drone_0_planning/trajectory` | 优化后的 `PolyTraj` |
| `/drone_0_planning/pos_cmd` | `traj_server` 采样的 `PositionCommand` |
| `/drone_0_planning/data_display` | 规划调试可视化 |
| `/drone_0_traj_server/heartbeat` | 轨迹服务心跳 |
| `/broadcast_traj_from_planner` | 可选多机轨迹广播入口 |

`trajectory_msg_converter.py` 将 `/drone_0_planning/pos_cmd` 转换为 `/command/trajectory`。当前有效主通道是位置、速度、加速度和 yaw；jerk 不通过该转换器传入 SE3。

### SE3 输入/输出

| 方向 | Topic | 作用 |
|---|---|---|
| 输入 | `/mavros/local_position/odom` | PX4/MAVROS 当前位姿与速度反馈 |
| 输入 | `/mavros/imu/data` | IMU 反馈 |
| 输入 | `/mavros/state` | 连接、模式和解锁状态 |
| 输入 | `/command/trajectory` | Planner 期望状态 |
| 输出 | `/mavros/setpoint_raw/attitude` | 姿态与推力主控制输出 |
| 输出 | `/mavros/setpoint_position/local` | OFFBOARD 前的 setpoint 流 |
| 输出 | `/desire_odom_pub` | 期望状态调试输出 |

SE3 节点内部的自动 OFFBOARD/解锁受 `enable_sim`、`auto_request_offboard`、`auto_request_arm` 共同约束，本仓库两端配置均关闭。仿真和真机都必须显式执行仓库级 `arm` 命令：同一个共享执行器先请求 PX4 原生起飞，到达实际高度并确认预热 setpoint 后自动切换 OFFBOARD。两端都不依赖 SE3 节点或 RViz 目标桥在启动时自动解锁。

## 修改与测试

修改消息、Planner 或 SE3 后执行：

```bash
./launch/sim.sh test
```

该命令构建完整 overlay、运行 catkin 测试并静态展开公共 launch。通过后再运行 headless 或 GUI 仿真验证端到端链路。真机镜像内是构建时复制的源码，因此合并修改后需要重新执行 `./launch/real_container.sh build` 并重建容器。

## 原理文档

- [../../docs/diff_planner_principles.md](../../docs/diff_planner_principles.md)
- [../../docs/se3_controller.md](../../docs/se3_controller.md)
- [../../docs/controller_tuning.md](../../docs/controller_tuning.md)

## 参考

- [DifferentialRobotics/Diff-Planner](https://github.com/DifferentialRobotics/Diff-Planner)
- [ZJU-FAST-Lab/EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)
- [HITSZ-MAS/se3_controller](https://github.com/HITSZ-MAS/se3_controller)
- [Tfly6/Mid360_px4_sim_plugin](https://github.com/Tfly6/Mid360_px4_sim_plugin)
