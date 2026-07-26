# 公共自主飞行链路

`common/` 保存仿真与真机共同使用的接口、launch、参数和飞行命令执行器。环境专用
的传感器、定位适配和载体参数分别位于 `simulation/` 与 `deployment/`。

## 定位输入契约

| Topic | 类型 | 约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | pose 在 `world`；twist 按消息规范表达在 `base_link`；`child_frame_id=base_link` |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 点已转换到 `world`；`header.frame_id=world` |

仿真由 `sim2real_simulation/localization.launch` 生成这两个 topic。真机由 `odom_to_base.py` 和 `cloud_relay.py` 适配 FAST-LIO 输出。

Diff-Planner 会把 `base_link` 线速度旋转到 `world` 后用于规划。适配器不能提前把 twist 标记成 world-frame 数据。

里程计必须携带有效、持续递增的测量时间戳。真机点云由 `cloud_relay.py` 使用其
测量时刻的 TF 变换到 `world`；需要跨坐标系时，时间戳为空或对应 TF 不可用便
丢弃，禁止只改 `frame_id`。仿真把 MAVROS `map` 显式声明为与 Gazebo `world`
数值重合；该 identity 别名不做数值变换，输入父/子坐标系不匹配时直接丢弃。

## 公共链路

```text
/localization/odom ───────────────┐
/localization/cloud_registered ───┴─> diff_planner_node
                                      → /drone_0_planning/trajectory
                                      → traj_server
                                      → /drone_0_planning/pos_cmd
                                      → trajectory_msg_converter
                                      → /command/trajectory
                                      → se3_controller_node
                                      → /mavros/setpoint_raw/attitude
```

SE3 状态反馈来自 MAVROS 的 `/mavros/local_position/odom` 与 `/mavros/imu/data`。真机把公共里程计回灌到 `/mavros/vision_pose/pose`，再由 PX4 EKF 和 MAVROS 提供控制反馈；仿真直接使用 SITL 状态。

## 内容

| 路径 | 作用 |
|---|---|
| `launch/planner.launch` | Diff-Planner 与 `traj_server` |
| `launch/trajectory_converter.launch` | `PositionCommand` 转 `/command/trajectory` |
| `launch/controller.launch` | 加载公共与载体专用配置后启动 SE3 |
| `launch/planning_control.launch` | 组合上述规划与控制节点 |
| `scripts/arm_executor.py` | 两端共享的原生起飞与 OFFBOARD 交接 |
| `scripts/goal_executor.py` | 单目标飞前检查与发布 |
| `scripts/mission_executor.py` | Mission 飞行状态机 |
| `scripts/waypoint_mission.py` | Mission JSON 校验、航点规划确认与到达监测 |
| `scripts/localization_guard.py` | 定位失效保护 |
| `scripts/rviz_goal_to_diff_planner.py` | 真机远程 RViz 目标桥 |

`localization_guard.py` 使用系统单调时钟监测接收间隔，并检查里程计时间戳、数值、
跳变和速度。流停止、时间戳不前进或仿真 `/clock` 冻结都会锁存故障；自主模式下
请求 `AUTO.LAND`，完整栈重启前不再接受自主命令。

`arm_executor.py`、`mission_executor.py` 和 `waypoint_mission.py` 还会独立检查
MAVROS 状态、高度与定位的新鲜度。定位失效直接请求 `AUTO.LAND`；其他自主执行
故障优先确认 `AUTO.LOITER`，其中起飞或 OFFBOARD 交接失败时还会以
`AUTO.LAND` 兜底。飞手已切换模式时不会覆盖；若 MAVROS 状态本身失联，则无法
确认恢复模式，必须由飞手接管。

真机与仿真的 RViz 目标桥均使用非锁存 `/goal`，只转发有限数值、`world` 坐标系、
当前 Planner 高度范围内，且飞机已解锁并处于 OFFBOARD、定位保护正常的目标。
不满足条件的点击不会排队。

仓库根目录的 `launch/` 只负责编排容器和整条链路；ROS XML launch 保留在对应 ROS 包内。

## 参数归属

| 文件 | 归属 |
|---|---|
| `config/planner.yaml` | 两端共享的地图、规划、优化和目标参数 |
| `config/trajectory_server.yaml` | 两端共享的轨迹采样与 yaw 参数 |
| `config/controller.yaml` | 两端共享的控制算法与安全默认值 |
| `simulation/config/controller.yaml` | 仿真载体的悬停推力、推力限制和围栏 |
| `deployment/config/controller.yaml` | 真机的悬停推力、积分、推力限制和围栏 |

不要在 `simulation/` 或 `deployment/` 复制 Planner 参数。临时实验可通过 `SIM_PLANNER_CONFIG` 或 `PLANNER_CONFIG` 加载完整 YAML；需要两端长期一致的修改应合并回 `common/config/planner.yaml`。

Planner 的 `resolution` 与 `obstacles_inflation` 必须按机体完整碰撞包络、定位误差和安全余量验证。载体专用悬停推力、推力限制与围栏只修改对应 controller YAML。

## 修改原则

- 规划算法、地图或飞行包络：修改公共 Planner，并先通过仿真验证；
- 悬停推力、推力上限或载体围栏：修改对应载体配置；
- 新定位或传感器：在环境目录实现适配器，输出两个公共 localization topic；
- Planner 不直接订阅 `/mavros/local_position/odom`、`/Odometry`、`/cloud_registered` 或 `/livox/lidar`。
