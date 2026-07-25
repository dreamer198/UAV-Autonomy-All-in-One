# 公共 Sim-to-Real 链路

`common/` 保存仿真与真机共同使用的 ROS 接口、launch、参数和目标适配工具。环境专用的 PX4/Gazebo 代码属于 `simulation/`，Livox/FAST-LIO/Jetson 代码和适配节点属于 `deployment/`。

## 输入契约

公共 Planner 只接受：

| Topic | 类型 | 约定 |
|---|---|---|
| `/localization/odom` | `nav_msgs/Odometry` | `header.frame_id=world`，`child_frame_id=base_link`；pose 表达在 `world`，twist 按标准 `nav_msgs/Odometry` 语义表达在 `base_link` |
| `/localization/cloud_registered` | `sensor_msgs/PointCloud2` | 点已经变换到 `world`，`header.frame_id=world` |

仿真由 `sim2real_simulation/localization.launch` 实现契约；真机由 `launch/real.sh` 编排 `sim2real_deployment` 中 FAST-LIO 后的 `odom_to_base.py` 与 `cloud_relay.py` 实现契约。Diff-Planner 的 odom callback 会按姿态把 `base_link` 线速度旋到 `world`，再用于规划初始状态；适配器不要提前把 twist 伪装成 world-frame 数据。

## 公共 ROS 包

`common/` 本身就是 ROS 包 `sim2real_common`，提供：

- `launch/planner.launch`：Diff-Planner 与 `traj_server`；
- `launch/trajectory_converter.launch`：`PositionCommand` 到 `/command/trajectory`；
- `launch/controller.launch`：SE3 控制器；
- `launch/planning_control.launch`：以上三项的组合入口；
- `scripts/rviz_goal_to_diff_planner.py`：RViz 目标桥；
- `scripts/waypoint_mission.py`：两端共用的有序航点验证、Planner 轨迹确认和到达监测执行器。

真机专用的 `odom_to_base.py`、`cloud_relay.py` 和 `odom_to_pose.py` 位于 `deployment/ros_pkgs/sim2real_deployment`；公共包不依赖 Livox、FAST-LIO 或真机回灌逻辑。

ROS XML launch 必须留在 ROS package 内；仓库根目录的 `launch/` 只负责编排容器和整条链路。

## 参数所有权

| 文件 | 所有者 | 可以包含什么 |
|---|---|---|
| `config/planner.yaml` | 公共 | 地图、规划范围、速度/加速度/jerk、优化器、目标容差 |
| `config/trajectory_server.yaml` | 公共 | 轨迹采样和 yaw 平滑参数 |
| `config/controller.yaml` | 公共 | 算法开关与两端一致的安全默认值 |
| `deployment/config/controller.yaml` | 真机 | 真机悬停推力、积分增益、推力限制、围栏 |
| `simulation/config/controller.yaml` | 仿真 | `iris_mid360` 悬停推力、推力限制、仿真围栏 |

不要在 `simulation/` 或 `deployment/` 复制 `planner.yaml`。需要临时实验参数时，可以通过 `SIM_PLANNER_CONFIG` 或真机 `PLANNER_CONFIG` 指向另一份完整 YAML；确认有效后应把希望两端共享的变更合并回公共文件。

## 公共主链路

```text
/localization/odom ───────────────┐
/localization/cloud_registered ───┴─> diff_planner_node
                                      -> /drone_0_planning/trajectory
                                      -> traj_server
                                      -> /drone_0_planning/pos_cmd
                                      -> trajectory_msg_converter
                                      -> /command/trajectory
                                      -> se3_controller_node
                                      -> /mavros/setpoint_raw/attitude
```

SE3 当前状态反馈仍来自 MAVROS 的 `/mavros/local_position/odom` 与 `/mavros/imu/data`。真机侧把 `/localization/odom` 回灌到 `/mavros/vision_pose/pose`，由 PX4 EKF/MAVROS 输出控制反馈；仿真侧直接使用 PX4 SITL 的状态。因此从公共定位接口进入 Planner，到 MAVROS 输出为止，两端的节点、topic 契约和核心参数一致。

## 修改规则

- 改路径规划算法、地图或飞行包络：改公共 Planner 源码/参数，并在仿真后再做真机检查。
- 改悬停油门、推力上限或载体围栏：只改对应车辆的 controller YAML。
- 接入新的定位或传感器：在环境目录实现适配器，输出两个公共 localization topic。
- 不要让 Planner 直接订阅 `/mavros/local_position/odom`、`/Odometry`、`/cloud_registered` 或 `/livox/lidar`；这些都属于适配层输入。
