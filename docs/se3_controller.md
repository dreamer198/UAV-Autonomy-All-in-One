# SE3 控制器工作原理详解

> 本文是 `se3_controller` 包的权威说明，目标是让一个**懂基本 ROS 与多旋翼概念、但不熟悉本代码库**的工程师，能够完整理解这套控制器「输入什么、内部怎么算、输出什么」，以及部署时每个参数的含义与调法。
>
> 所有结论均对照源码逐行核对过，引用均为可点击链接。核心源码位于
> [src/se3_controller/](../third_party/Diff-Planner-PX4/src/se3_controller/)：
> - 节点与状态机：[se3_ctrl.cpp](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp)、[se3_ctrl.h](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_ctrl.h)
> - 控制律内核：[se3_controller.hpp](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp)
> - 数学工具：[utils.hpp](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/utils.hpp)
> - 入口：[se3_controller_node.cpp](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_controller_node.cpp)
> - 部署参数：[scripts/start_real_px4_mid360_fastlio.sh](../scripts/start_real_px4_mid360_fastlio.sh)

---

## 先读导引

这篇文档回答的是：**规划器给出“期望位置/速度/加速度/yaw”之后，控制器怎样把它变成 PX4 能执行的“姿态四元数 + 归一化油门”。**

在 4 篇文档里的位置：

| 前后关系 | 文档 | 你会得到什么 |
|---|---|---|
| 上游 | [diff_planner_principles.md](diff_planner_principles.md) | 规划器怎样生成轨迹 |
| 本文 | 本文 | 控制器怎样跟踪轨迹、参数怎样起作用 |
| 专题 | [ki_pz_tuning_guide.md](ki_pz_tuning_guide.md) | `ki_pz` 竖直积分增益的真机整定步骤 |
| 排障 | [trajectory_tracking_altitude.md](trajectory_tracking_altitude.md) | 接规划器飞行时的掉高风险定位 |

如果只想先理解能飞起来的主链路，按这个顺序读：

| 目标 | 建议阅读 |
|---|---|
| 知道控制器接什么、发什么 | §2、§3、§11 |
| 知道真机状态怎么切换 | §6、§7 |
| 知道掉高/蹿高为什么和油门标定有关 | §8.6、§8.7、§10、§15 |
| 查细节和坑 | §5、§12–§16 |

先记住四件事：

1. 真机上**不会自动 OFFBOARD、不会自动解锁**，飞手切入后控制器才接管。
2. 真正生效的主链路是**位置/速度反馈 + 加速度前馈 → 姿态/油门**；当前代码里的 `Kd_*` 实际不生效。
3. `hover_percent` 决定基础推力标定；本真机启动脚本默认用 `0.90`，不要被 C++ fallback 的 `0.45` 误导。
4. 实际发给 PX4 的是**姿态 + 油门**；`bodyrates` 虽然算了但被 `type_mask` 忽略，因此 jerk/yaw-rate/Kp_q/Kp_a 这些只进入角速率字段的量，不要按「已被 PX4 执行」来理解。

---

## 目录

1. [文档目的与适用范围](#1-文档目的与适用范围)
2. [SE3 控制器是什么 / 在系统中的角色](#2-se3-控制器是什么--在系统中的角色)
3. [ROS 接口总览](#3-ros-接口总览)
4. [关键数据结构](#4-关键数据结构)
5. [坐标系与 frame 约定](#5-坐标系与-frame-约定)
6. [节点生命周期与状态机](#6-节点生命周期与状态机)
7. [期望状态的三个来源与优先级](#7-期望状态的三个来源与优先级)
8. [控制律详解：级联结构](#8-控制律详解级联结构)
9. [从期望加速度到姿态：微分平坦与 Hopf 纤维化](#9-从期望加速度到姿态微分平坦与-hopf-纤维化)
10. [推力模型与在线推力估计](#10-推力模型与在线推力估计)
11. [控制指令如何发给 PX4](#11-控制指令如何发给-px4)
12. [地理围栏与安全机制](#12-地理围栏与安全机制)
13. [参数全表](#13-参数全表)
14. [与规划器虚拟天花板/地板的关系](#14-与规划器虚拟天花板地板的关系)
15. [调参与排障指南](#15-调参与排障指南)
16. [已知细节与陷阱（gotchas 汇总）](#16-已知细节与陷阱gotchas-汇总)

---

## 1. 文档目的与适用范围

SE3 控制器（`se3_controller_node`）是本部署链路里**「轨迹 → 姿态+油门」的最后一环**：它把 Diff-Planner 规划出的期望轨迹点，结合 FAST-LIO 的实时里程计与 IMU，换算成 PX4 飞控能直接执行的**姿态四元数 + 归一化油门**，通过 MAVROS 以 100 Hz 发送。

本文覆盖：节点架构、状态机、控制律数学、推力模型、坐标系约定、参数含义、调参与排障。

本文**不**覆盖：FAST-LIO 内部原理、Diff-Planner 轨迹优化算法、PX4 固件内部姿态/角速率环（这些在 PX4 侧，本控制器只向其发送姿态设定值）。

---

## 2. SE3 控制器是什么 / 在系统中的角色

「SE(3)」指三维刚体运动所在的特殊欧氏群（位置 ∈ ℝ³ + 姿态 ∈ SO(3)）。本控制器是一个**基于微分平坦（differential flatness）的几何控制器**：它在 SE(3) 上直接对位置与姿态做跟踪，而不是先解算欧拉角再做角度环。

### 在整条链路中的位置

```text
 Livox MID-360            ┌──────────────┐
   点云 + IMU  ──────────►│   FAST-LIO   │  里程计
                          └──────┬───────┘
                                 │ /Odometry → odom_to_base.py → /Odometry_base
                                 ▼
                     ┌────────────────────────┐
                     │   MAVROS  (vision pose) │ ──► PX4 EKF 融合
                     └───────────┬─────────────┘
                                 │ /mavros/local_position/odom  (位姿+速度)
                                 │ /mavros/imu/data             (实测加速度/姿态)
   ┌──────────────┐              ▼
   │ Diff-Planner │  /drone_0_planning/pos_cmd
   │  局部规划器  │ ──► trajectory_msg_converter.py ──► /command/trajectory
   └──────────────┘              │  (期望 位置/速度/加速度/偏航)
                                 ▼
                     ┌────────────────────────┐
                     │      SE3 控制器         │   ← 本文主角
                     │  (se3_controller_node)  │
                     └───────────┬─────────────┘
                                 │ /mavros/setpoint_raw/attitude
                                 ▼   (四元数 q + 归一化推力 thrust)
                              ┌──────┐
                              │ PX4  │  姿态/角速率/电机环
                              └──────┘
```

一句话概括：**规划器只说「下一刻该到哪、速度多少」，PX4 只吃「姿态 + 油门」，SE3 控制器是中间的翻译器**——把期望状态与实测状态做差，算出这一拍该给的姿态和油门。

> 部署链路里规划器输出的是 `pos_cmd`，由 `trajectory_msg_converter.py` 转成控制器订阅的 `/command/trajectory`（`trajectory_msgs/MultiDOFJointTrajectory`）。详见
> [scripts/start_real_px4_mid360_fastlio.sh:35-38](../scripts/start_real_px4_mid360_fastlio.sh#L35-L38)。

---

## 3. ROS 接口总览

构造函数 [se3_ctrl.cpp:8-91](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L8-L91) 建立了全部通信拓扑。

### 订阅（输入）

| 话题 | 类型 | 回调 | 作用 |
|---|---|---|---|
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | [`OdomCallback`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L240) | 当前位姿/速度/角速度，并在此做地理围栏检测 |
| `/mavros/imu/data` | `sensor_msgs/Imu` | [`IMUCallback`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L260) | 实测加速度/姿态（用于加速度环与推力估计） |
| `/mavros/state` | `mavros_msgs/State` | [`StateCallback`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L265) | 飞控连接/解锁/飞行模式 |
| `/command/trajectory` | `trajectory_msgs/MultiDOFJointTrajectory` | [`multiDOFJointCallback`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L292) | **主期望轨迹来源**（来自规划器） |
| `/desire_odom` | `nav_msgs/Odometry` | [`DesireOdomCallback`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L269) | 另一种期望状态注入接口（备用） |

### 发布（输出）

| 话题 | 类型 | 作用 |
|---|---|---|
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | **主输出**：姿态四元数 + 归一化油门 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 进 OFFBOARD 前/接管时发布位置 setpoint（维持 PX4 setpoint 流） |
| `/desire_odom_pub` | `nav_msgs/Odometry` | 期望状态回显，便于 RViz/录包监测 |

### 服务与客户端

| 名称 | 方向 | 作用 |
|---|---|---|
| `/land` (`std_srvs/SetBool`) | server | 外部触发降落，转入 `LANDING` 状态 |
| `/mavros/set_mode` | client | 切换 PX4 飞行模式（如 `AUTO.LAND`） |
| `/mavros/cmd/arming` | client | 解锁（仅在仿真自动解锁时使用） |

### 执行定时器

主循环是一个 **100 Hz** 定时器
[se3_ctrl.cpp:24](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L24)：
`exec_timer_ = nh_.createTimer(ros::Duration(0.01), &se3Ctrl::execFSMCallback, this)`。
真正的控制计算都发生在这个回调里。

---

## 4. 关键数据结构

定义于 [se3_controller.hpp:13-166](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L13-L166)。

### `Odom_Data_t` — 当前状态（实测）
[se3_controller.hpp:13-64](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L13-L64)

| 字段 | 含义 |
|---|---|
| `p` | 位置 (世界系) |
| `v` | 速度（经 `feed()` 处理后为世界系，见第 5 节） |
| `q` | 姿态四元数 |
| `w` | 机体角速度 |
| `rcv_stamp` | 接收时间戳，用于 0.1 s 数据陈旧检测 |

`feed()` 把 `nav_msgs/Odometry` 解析进来，并按 `enu_frame`/`vel_in_body` 做坐标变换。

### `Desired_State_t` — 期望状态（指令）
[se3_controller.hpp:66-98](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L66-L98)

| 字段 | 含义 |
|---|---|
| `p, v, a` | 期望位置/速度/加速度；当前姿态与推力输出主要由这三者决定 |
| `j` | 期望 jerk；在主 `/command/trajectory` 链路里被置零，后续只会影响 `bodyrates` |
| `q, yaw` | 期望姿态/偏航；`yaw` 决定期望姿态里的航向 |
| `yaw_rate` | 期望偏航角速率；当前只会影响 `bodyrates`，而 `bodyrates` 发给 PX4 时被忽略 |

注意它有两个构造：默认零态，以及 `Desired_State_t(Odom_Data_t)`——用当前里程计构造一个「原地悬停」期望。

### `Imu_Data_t` — IMU 实测
[se3_controller.hpp:114-166](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L114-L166)

含 `a`（线加速度，机体系）、`q`（姿态）、`w`（角速度）。`a` 用于加速度反馈环与推力估计；但加速度反馈环的输出是 `j/bodyrates`，在当前 `type_mask` 下不会改变 PX4 实际执行的姿态/油门。

### `Controller_Output_t` — 控制器输出
[se3_controller.hpp:100-112](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L100-L112)

| 字段 | 含义 |
|---|---|
| `q` | 期望机体姿态（相对世界系） |
| `bodyrates` | 机体角速度 [rad/s] |
| `thrust` | **质量归一化**的集合推力（即 PX4 需要的 [0,1] 油门） |

> ⚠️ 见第 11 节：实际发给 PX4 时 `type_mask` **忽略了 bodyrates**，只用 `q` + `thrust`。

---

## 5. 坐标系与 frame 约定

控制器内部统一在 **ENU + 世界系** 下工作。两个开关在
[se3_ctrl.cpp:48-49](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L48-L49) 写死：

```cpp
enu_frame_  = true;   // 使用 ENU；若为 NED 则触发 R_mid 变换
vel_in_body_ = true;  // odom 的速度是机体系，需要旋到世界系
```

### `vel_in_body_ = true`
FAST-LIO / MAVROS 发布的 odom `twist`（速度）通常表达在**机体系**。`feed()` 在
[se3_controller.hpp:61-62](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L61-L62) 用 `v = q.toRotationMatrix() * v` 把它旋到世界系，保证后续位置/速度环都在同一个世界系下做差。头文件顶部还有对应的 `#define VEL_IN_BODY`（[se3_controller.hpp:10](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L10)）。

### `enu_frame_ = true`：`R_mid` 变换默认关闭
当 `!enu_frame`（即 NED）时，代码会用一个固定矩阵做 ENU↔NED 风格的相似变换：

```text
R_mid = | 0  1  0 |     （交换 x/y，翻转 z）
        | 1  0  0 |
        | 0  0 -1 |
```

它出现在三处：`Odom_Data_t::feed`（[se3_controller.hpp:49-59](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L49-L59)）、`Imu_Data_t::feed`（[se3_controller.hpp:144-153](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L144-L153)）、以及控制输出路径（[se3_controller.hpp:406-412](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L406-L412)）。

> **本部署中 `enu_frame_ = true`，所以这些 NED 分支全部不执行。** 了解它存在即可，便于读懂代码，但日常无需关心。
>
> 细节：输出路径里的 `q_mid` 实际由 `R_mid.inverse()` 构造（[se3_controller.hpp:409](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L409)）；因为 `R_mid` 是正交矩阵，`R_mid.inverse() == R_mid`，二者等价。

### 与 FCU 帧对齐
控制律算出的期望姿态在「odom 世界系」下，发给飞控前需对齐到 FCU 的姿态参考。这一步在
[se3_controller.hpp:386](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L386)：
`output.q = imu_data.q * odom_data.q.inverse() * desired_odom.q`，即「从 odom 帧映射到 imu(FCU) 帧」。

### 偏航提取
`utils::fromQuaternion2yaw`（[utils.hpp:8-10](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/utils.hpp#L8-L10)）用标准 `atan2` 公式从四元数取偏航角；另有 `quat2euler`/`euler2quat` 辅助函数。

---

## 6. 节点生命周期与状态机

枚举 `FlightState`（[se3_ctrl.h:59](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_ctrl.h#L59)）：

```cpp
enum FlightState { WAITING_FOR_CONNECTED, WAITING_FOR_OFFBOARD,
                   TAKEOFF, MISSION_EXECUTION, LANDING, LANDED };
```

> ⚠️ **`TAKEOFF` 是死代码**：枚举里声明了，但 `switch` 里没有这个分支，也没有任何地方把 `node_state_` 设为 `TAKEOFF`。实际只用到 5 个状态。

主循环 [execFSMCallback](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L95) 以 100 Hz 在这些状态间切换：

```text
 ┌──────────────────────┐
 │ WAITING_FOR_CONNECTED │  阻塞自旋直到 /mavros/state.connected
 └──────────┬───────────┘
            │ connected == true
            ▼
 ┌──────────────────────┐   每拍发布当前位置 setpoint，维持 PX4 setpoint 流
 │ WAITING_FOR_OFFBOARD  │   真机：等待飞手用遥控器切 OFFBOARD + 解锁
 └──────────┬───────────┘
            │ mode == "OFFBOARD" && armed
            │ → 锁定当前位姿为期望，等待新轨迹
            ▼
 ┌──────────────────────┐   每拍调用 calControl()，发姿态+油门
 │   MISSION_EXECUTION   │   若被切出 OFFBOARD/上锁 → 退回保持当前位姿
 └──────────┬───────────┘
            │ /land 服务触发
            ▼
 ┌──────────┐    切 AUTO.LAND     ┌────────┐  disarm 后停止定时器
 │ LANDING  │ ──────────────────► │ LANDED │
 └──────────┘                     └────────┘
```

### `WAITING_FOR_CONNECTED`
[se3_ctrl.cpp:95-103](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L95-L103)：阻塞式 `while(!connected) ros::spinOnce()`，直到与飞控建立连接，然后转入下一态。这是 FSM 里唯一显式调用 `ros::spinOnce()` 的地方。

### `WAITING_FOR_OFFBOARD`
[se3_ctrl.cpp:104-122](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L104-L122)：
- 每拍发布一个位置 setpoint 到 `/mavros/setpoint_position/local`——有里程计就发当前位姿，否则发 `init_pose_ = [0,0,0.5]`。**这是为了满足 PX4「进 OFFBOARD 前必须有持续 setpoint 流」的要求。**
- 调用 `trigger_offboard()` / `trigger_arm()`（见下方「真机 vs 仿真」）。
- 一旦 `mode == "OFFBOARD" && armed`，把期望状态锁成当前里程计（`setDesiredStateToCurrentOdom()`，[se3_ctrl.cpp:116](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L116)），并置 `has_trajectory_after_offboard_ = false`，打印 *"OFFBOARD entered. Holding current pose until a fresh trajectory is received."*，转入 `MISSION_EXECUTION`。

### `MISSION_EXECUTION`
[se3_ctrl.cpp:124-148](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L124-L148)：
- **安全降级**：若 `mode != "OFFBOARD" || !armed`（飞手用遥控器接管了），立刻回到「保持当前位姿」并 `return`，不再发控制量（[se3_ctrl.cpp:125-134](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L125-L134)）。
- **主控制**：`has_odom_ && has_imu_` 时调用 `calControl()` 算出姿态+油门并 `send_cmd()` 发送；若 `enable_thrust_estimation_` 为真则在线估计推力系数。

### `LANDING` / `LANDED`
[se3_ctrl.cpp:150-166](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L150-L166)：`LANDING` 调 `set_mode` 切到 PX4 原生 `AUTO.LAND`，此后由飞控自主降落，控制器不再发姿态；进入 `LANDED` 后，**当 `!armed` 时**才 `exec_timer_.stop()` 停止主循环（[se3_ctrl.cpp:160-163](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L160-L163)）。

### 真机 vs 仿真：自动 OFFBOARD/解锁 的关键差异
`trigger_offboard()`（[se3_ctrl.h:131-146](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_ctrl.h#L131-L146)）与 `trigger_arm()`（[se3_ctrl.h:148-163](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_ctrl.h#L148-L163)）**都在开头就 early-return**，除非：

```cpp
if (!(sim_enable_ && auto_request_offboard_)) return;   // offboard
if (!(sim_enable_ && auto_request_arm_))      return;   // arm
```

部署脚本里 `SE3_ENABLE_SIM=false`（[start_real_px4_mid360_fastlio.sh:40](../scripts/start_real_px4_mid360_fastlio.sh#L40)），所以**真机上控制器永远不会自动切 OFFBOARD、也不会自动解锁**——这两件事必须由飞手通过遥控器完成。这与仓库 README 的安全策略一致，也是设计上的硬性安全保证。

### `takeoff_height` 的真实行为（重要澄清）
构造函数把期望 z 初始化为 `takeoff_height_`（[se3_ctrl.cpp:73-75](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L73-L75)）。但要注意：

- 进入 `WAITING_FOR_OFFBOARD` / `MISSION_EXECUTION` 时，只要 `has_odom_` 为真，就会调用 `setDesiredStateToCurrentOdom()`（[se3_ctrl.cpp:209](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L209)），**用当前里程计无条件覆盖整个期望位置（含 z）**。
- 因此**只有在拿不到里程计时，`takeoff_height_` 这个初值才会保留**；正常有 odom 时它会被当前高度覆盖。
- `takeoffFlag_`（[se3_ctrl.cpp:136-139](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L136-L139)）只在高度接近 `takeoff_height_` 时打印一句 *"takeoff completed"*，**不参与任何控制逻辑**。

> 结论：在本真机流程里，`takeoff_height` 并不真正驱动一次自动起飞。典型做法是飞手在定点模式下手动把飞机带到飞行高度，再切 OFFBOARD；此时控制器锁定当前（已在空中的）位姿并等待规划器轨迹。`takeoff_height` 实质是 odom 缺失时的兜底初值 + 一条日志阈值。

---

## 7. 期望状态的三个来源与优先级

`desired_state_` 在运行时被以下来源更新（后者覆盖前者）：

1. **构造初值**：`(0, 0, takeoff_height_)`，零速、零加速度（[se3_ctrl.cpp:73-76](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L73-L76)）。
2. **原地保持**：进 OFFBOARD / 被接管 / 等待新轨迹时，`setDesiredStateToCurrentOdom()` 把期望锁为当前里程计、速度清零（[se3_ctrl.cpp:209-231](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L209-L231)）。
3. **规划器轨迹**（主来源）：`/command/trajectory` 到来后，`multiDOFJointCallback` 用轨迹点覆盖期望 位置/速度/加速度/偏航（[se3_ctrl.cpp:292-338](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L292-L338)）。该回调有两道闸：
   - **未解锁或非 OFFBOARD 时直接丢弃轨迹**（[se3_ctrl.cpp:294-297](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L294-L297)）；
   - 空消息丢弃（[se3_ctrl.cpp:299-302](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L299-L302)）。
   - 加速度前馈受 `max_feedforward_acc_` 限幅、且仅当 `use_acceleration_feedforward_` 为真才使用（[se3_ctrl.cpp:320-325](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L320-L325)）。
   - `desired_state_.j` 在该回调里直接置零（[se3_ctrl.cpp:330](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L330)），所以规划器发布的 jerk 不会进入 SE3 控制器。
   - 偏航角速率仅当 `use_yaw_rate_feedforward_` 为真才读取（[se3_ctrl.cpp:338](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L338)），但当前只进入被忽略的 `bodyrates`。

（备用来源 `/desire_odom` 走 [`DesireOdomCallback`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L269)，机制类似，本链路一般不用。）

---

## 8. 控制律详解：级联结构

核心函数 [`calControl`](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L331)（se3_controller.hpp:331-448）。严格按当前 PX4 输出路径看，一拍计算可以分成两层：

- **实际生效层**：位置误差修正速度，速度误差修正加速度，再由加速度生成期望姿态和推力。
- **已计算但当前被忽略层**：加速度误差修正 jerk，jerk/yaw_rate/姿态误差生成 `bodyrates`；但 `send_cmd` 的 `type_mask` 忽略角速率，所以 PX4 不执行这些 `bodyrates`。

因此调参时应优先理解 `Kp_p/Kp_v`、加速度前馈、`hover_percent/T_a_` 与推力限幅；不要期待 `Kp_a`、`Kp_q` 或 yaw-rate 前馈在当前配置下直接改变飞机响应。

### 8.0 入口保护
若里程计超过 0.1 s 未更新，`calControl` 直接返回 false、不发指令（[se3_controller.hpp:331-334](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L331-L334)）。

### 8.1 位置环 → 修正期望速度
[se3_controller.hpp:337-349](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L337-L349)

```
err_p = p_meas - p_des            （限幅到 ±limit_err_p_）
v_des = v_des - Kp_p ∘ err_p - Kd_p ∘ d_err_p
```

直觉：位置偏差越大，越往「拉回目标」的方向叠加速度修正。

### 8.2 速度环 → 修正期望加速度（含重力补偿）
[se3_controller.hpp:350-358](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L350-L358)

```
err_v = v_meas - v_des            （限幅到 ±limit_err_v_）
a_des = a_des - Kp_v ∘ err_v - Kd_v ∘ d_err_v + g_vec
```

其中 `g_vec = [0,0,9.81]`（[se3_controller.hpp:296](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L296)）在比例/微分反馈**之后**叠加，实现重力补偿——这样悬停时期望加速度约等于 g，对应油门约等于 `hover_percent`。

### 8.3 加速度环 → 修正期望 jerk（当前不进入 PX4 有效输出）
[se3_controller.hpp:363-370](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L363-L370)

用 IMU 实测加速度旋到世界系做闭环：

```
a_world = q_meas.toRotationMatrix() * imu.a
err_a   = a_world - a_des          （限幅到 ±limit_err_a_）
j_des   = j_des - Kp_a ∘ err_a - Kd_a ∘ d_err_a
```

这段代码会改变 `desired_state.j`，而 `j` 在 Hopf 纤维化里只用于生成 `desired_odom.w/bodyrates`。由于第 11 节所述 `bodyrates` 当前被 PX4 忽略，所以这个加速度环是**算了但不改变实际姿态/油门输出**。它只有在以后取消 `IGNORE_*_RATE`、改为角速率设定值控制时才会真正进入闭环。

### 8.4 各增益与误差限幅
增益在 [se3_ctrl.cpp:54-71](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L54-L71) 写死（可经 dynamic_reconfigure 在线改，见第 13 节）：

| 增益 | 值 (x, y, z) | 作用 |
|---|---|---|
| `Kp_p` | (0.85, 0.85, **1.5**) | 位置环；z 更硬（抗掉高/抗地效） |
| `Kp_v` | (1.5, 1.5, 1.5) | 速度环 |
| `Kp_a` | (1.5, 1.5, 1.5) | 加速度误差→jerk/bodyrates；当前 PX4 忽略 bodyrates，实际姿态/油门不受它直接影响 |
| `Kp_q` | (**5.5**, **5.5**, 0.1) | 姿态误差→bodyrates；当前被 `type_mask` 忽略，z(偏航)很弱 |
| `Kp_w` | (1.5, 1.5, 0.1) | 角速率项——**当前激活路径未使用**（见 §9） |
| `Kd_p` | (0.1, 0.1, 0) | 微分项——**实际恒为 0，见下方** |
| `Kd_v / Kd_a / Kd_q / Kd_w` | 全 0 | — |

每个误差和误差增量都过 `limitErr` 饱和（`limit_err_*` / `limit_d_err_*`，[se3_ctrl.cpp:66-71](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L66-L71)）。这是重要的安全设计：即使某一拍里程计跳变、误差骤增，修正量也被钳在有限范围内，避免瞬间满舵。

### 8.5 ⚠️ 关于「微分项」：当前实现里 `Kd` 是死的
级联本身只有「前馈 + 比例 + 有限差分微分」，**默认不含积分项**（v2 起新增了可选的**竖直积分项**，默认关闭，见 [§8.7](#87-竖直积分项可选默认关闭)）。而且微分项在当前代码里**实际恒为零**：

- `have_last_err_` 在 `init()` 里置 false（[se3_controller.hpp:304](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L304)），且**代码里从未被置回 true**。
- 因此每拍都会执行 `if(have_last_err_==false) last_err = err`（如 [se3_controller.hpp:345-347](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L345-L347)），使 `d_err = err - last_err ≡ 0`。
- 结果：`Kd_p / Kd_v / Kd_a` 这些微分增益**对输出没有任何贡献**，无论取值多少。

> 实践影响：对当前有效输出，可以先把本控制器理解为**位置/速度比例反馈 + 加速度前馈 + 推力标定**。这在本部署里是良性的（`Kd` 本就大多为零），但若你打算靠调 `Kd` 来加阻尼，会发现不起作用——根因就在这个永不翻转的标志位。另外，即便它生效，`d_err` 也未除以时间步长（是有限差分而非真导数，[se3_controller.hpp:347](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L347)），效果会随控制频率变化。

### 8.6 推力标量
[se3_controller.hpp:376-379](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L376-L379)

```
thr          = a_des · (q_meas * ẑ)        # 期望加速度在机体 z 轴上的投影
output.thrust = thr / T_a_                 # 归一化到 [0,1]
output.thrust = clamp(thrust, min_output_thrust, max_output_thrust)
```

几何含义：多旋翼只能沿机体 z 轴产生推力，所以把期望加速度投影到当前机体 z 轴，得到需要的推力大小，再用 `T_a_` 归一化（见第 10 节）、并钳到上下限。

### 8.7 竖直积分项（可选，默认关闭）

> v2 新增。用于根治「随电池电压下降/载荷变化而缓慢掉高」——因为悬停所需推力 = f(机重, 当前电压)，不是常数，而无积分的前馈/比例链路只能在一个工作点精确，偏离后只能用稳态高度误差换取推力偏置。积分项把这个稳态缺口累积补上，使悬停推力自适应、稳态高度误差归零（等价于 PX4 定点模式那个让你「定点正常」的积分器）。

只在**竖直 z 通道**加一个带抗饱和与切换清零的积分：

```
int_err_p += err_p · dt           # dt=0.01s 固定步长；err_p 已限幅
int_err_p  = clamp(int_err_p, ±int_limit_z)
a_des     -= Ki_p ∘ int_err_p     # Ki_p=(0,0,ki_pz)：低于目标→加推力→消除下垂
```

关键实现点（[se3_controller.hpp](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp)）：
- 累加 + 逐轴钳位：[L341](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L341)；积分配平到 `a_des`：[L358](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L358)。
- **抗饱和（三重）**：① 上一拍推力饱和时**冻结累加**（`out_saturated_`，[L378](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L378)）；② 逐轴钳位 `int_limit_z`；③ **切换清零**——`resetIntegral()`（[L282](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L282)）在 [`setDesiredStateToCurrentOdom`](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L209) 里被调用，覆盖等待 OFFBOARD / 进入 OFFBOARD / 被接管，杜绝地面或待机期间积分饱和。
- 增益来源：构造时由 rosparam `ki_pz` / `int_limit_z` 经 `setIntegral()`（[L278](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L278)）注入，并可经 `rqt_reconfigure` 在线整定（[se3_ctrl.h DynamicTuneCallback](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_ctrl.h#L76)）。

**整定流程（务必低空、手放遥控器）**：`ki_pz` 默认 0（关闭，行为同无积分模式）。悬停中用 rqt 把 `ki_pz` 从 0 每次 +0.05 缓慢上调，直到随电压下降仍稳住高度、不缓慢下垂；若高度上下振荡则调小（典型 0.1–0.6，视机而定）。整定好后写入脚本 `SE3_KI_PZ` 默认值固化。**完整的逐步整定方法、判读标准与陷阱见 [ki_pz 整定指南](ki_pz_tuning_guide.md)。**

---

## 9. 从期望加速度到姿态：微分平坦与 Hopf 纤维化

### 9.1 直觉：为什么加速度能决定姿态
多旋翼是微分平坦系统：给定期望的（位置、偏航）及其各阶导数，就能**解析地**反推出所需的姿态与角速率。关键洞察是——**期望推力方向 = 期望加速度方向**，而期望加速度的方向唯一决定了机体 z 轴指向；偏航再补上绕 z 的自由度。jerk（加速度的导数）则前馈出机体角速率。

但在本部署的 MAVROS 输出里，PX4 只使用姿态四元数和推力，**不使用角速率字段**。所以这里的「jerk → 角速率」应理解为代码保留的完整几何控制计算，不是当前真机实际执行链路的关键量。

代码里有两种实现，激活的是 **Hopf 纤维化** 版本。

### 9.2 `computeFlatInput`（经典构造，未启用）
[se3_controller.hpp:184-213](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L184-L213)

标准做法：`zb = a.normalized()`，再用偏航辅助向量叉乘出 `xb, yb`，组成旋转矩阵；角速率前馈由 jerk 给出：
`ω₀ = -yb·j / a_zb`，`ω₁ = xb·j / a_zb`，`ω₂ = [yaw_rate·(xc·xb) + (yc·zb)·ω₁] / |yc×zb|`（其中 `a_zb = zb·a`，注意 ω₂ 依赖已算出的 ω₁）。

> `calControl` 实际调用的是下面的 Hopf 版本，本函数保留但未启用。

### 9.3 `computeFlatInput_Hopf_Fibration`（启用）
[se3_controller.hpp:215-246](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L215-L246)

把归一化期望加速度记为 `(a, b, c) = â`，`(ȧ, ḃ, ċ)` 是其导数（由 jerk 投影得到，[se3_controller.hpp:218](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L218)）。用 Hopf 纤维化直接构造姿态四元数 + 角速率，并按 `c` 的符号分两支：

- **`c > 0`**（推力方向在上半球，正常飞行）：[se3_controller.hpp:224-231](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L224-L231)
- **`c ≤ 0`**（推力方向指向下半球的奇异/极端姿态）：[se3_controller.hpp:232-245](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L232-L245)，用 `sqrt(2(1-c))` 归一化并加 `yaw += 2·atan2(a,b)`，**避开 `c → -1` 处的奇异点**。

> 第二支主要是数值稳健性兜底：常规飞行 `c` 远大于 0，几乎一直走第一支。

### 9.4 姿态对齐与机体角速率修正
回到 `calControl`：

1. **对齐 FCU 帧**：`output.q = imu.q * odom.q⁻¹ * desired_odom.q`（[se3_controller.hpp:386](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L386)）。
2. **姿态误差转角速率修正**：`err_q = odom.q⁻¹ * desired_odom.q`，按 `err_q.w()` 符号取最短旋转方向，`err_br = Kp_q ∘ err_q.vec()`（[se3_controller.hpp:391-404](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L391-L404)）。
3. **输出角速率**：`output.bodyrates = desired_odom.w + err_br`（前馈角速率 + 比例修正，[se3_controller.hpp:404](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L404)）。

> 注意：激活路径计算了 `Kp_q` 对 `bodyrates` 的修正，但这些 `bodyrates` 当前被 PX4 忽略；`Kp_w / Kd_q / Kd_w` 出现在被注释掉的另一种实现里（[se3_controller.hpp:414-443](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L414-L443)），当前不参与计算。

---

## 10. 推力模型与在线推力估计

### 10.1 归一化推力模型
PX4 的 `AttitudeTarget.thrust` 取值 `[0,1]`。控制器用一个标量 `T_a_` 把「期望加速度 (m/s²)」映射到这个归一化油门：

```
output.thrust = thr / T_a_                         （se3_controller.hpp:377）
T_a_ 初值     = gravity_ / hover_percent_           （se3_controller.hpp:278）
```

初值的物理意义：悬停时期望竖直加速度 ≈ g，代入得 `thrust ≈ hover_percent`。所以 **`hover_percent` 必须接近你这台机器真实的悬停油门**，否则起飞瞬间要么压不住（掉高）要么蹿高。

### 10.2 在线推力系数估计 `estimateTa`
[se3_controller.hpp:451-485](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L451-L485)，仅当 `enable_thrust_estimation_` 为真时每拍调用。用**带遗忘因子的递归最小二乘（RLS）** 在线辨识 `T_a_`：

- 模型：`est_a(2) = T_a_ · thr`（竖直实测加速度 ∝ 当时发出的油门）。
- **延迟补偿**：每条油门命令带时间戳入队（[se3_controller.hpp:445-447](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L445-L447)）；估计时只取**距今 35–45 ms** 的那条命令，匹配「指令→实际加速度」的物理时延。超过 45 ms 的丢弃并继续找，不足 35 ms 的直接 `return false` 等下一拍（[se3_controller.hpp:455-463](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L455-L463)）。
- RLS 更新（[se3_controller.hpp:475-479](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L475-L479)）：`P_=1e6` 初值协方差、`rho_=0.998`（代码注释写作 "confidence"，数学上即遗忘因子）。
- **下限保护**：`T_a_ = max(T_a_, g / max_hover_percent)`（[se3_controller.hpp:479](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L479)），即 `max_hover_percent` 给 `T_a_` 设了下限，限制对推力权限的高估。

### 10.3 默认关闭的影响
部署脚本里 `SE3_ENABLE_THRUST_ESTIMATION=false`（[start_real_px4_mid360_fastlio.sh:48](../scripts/start_real_px4_mid360_fastlio.sh#L48)）。此时 `estimateTa` 不被调用，**`T_a_` 固定在 `g/hover_percent` 不再更新**——但它仍然每拍被用于推力归一化（[se3_controller.hpp:377](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp#L377)）。换句话说：默认配置下，推力标定完全取决于你给的 `hover_percent`，把它调准是首要任务。

---

## 11. 控制指令如何发给 PX4

`send_cmd`（[se3_ctrl.cpp:175-191](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L175-L191)）封装 `mavros_msgs/AttitudeTarget`：

```cpp
cmd.orientation = output.q;     // 期望姿态四元数
cmd.thrust      = output.thrust;// 归一化油门 [0,1]
cmd.body_rate   = output.bodyrates;
cmd.type_mask   = IGNORE_ROLL_RATE | IGNORE_PITCH_RATE | IGNORE_YAW_RATE;  // 见下
```

> ⚠️ **关键点**：`type_mask` 设了 `IGNORE_*_RATE`（[se3_ctrl.cpp:186-189](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L186-L189)），即虽然算了 `bodyrates`，但**实际发给 PX4 时忽略机体角速率，只用「姿态四元数 + 油门」**。PX4 内部的姿态环/角速率环负责把姿态设定值跟踪到位。本控制器因此是一个「姿态 + 推力」设定值控制器，而非角速率控制器。

直接后果：
- 规划器/转换脚本传进来的 `yaw_rate` 只会进入 `bodyrates`，当前不会改变 PX4 执行。
- `desired_state.j`、`Kp_a` 加速度环、`Kp_q` 姿态误差修正也只影响 `bodyrates`，当前不要把它们当作有效调参旋钮。
- 真正决定姿态/推力设定值的是位置/速度误差修正后的 `desired_state.a`、`yaw` 和推力模型。

---

## 12. 地理围栏与安全机制

围栏检测在每次里程计回调中执行（[se3_ctrl.cpp:240-255](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L240-L255)）：

```cpp
judge_x = |p(0)| >= geo_fence_[0];     // x 对称 ±
judge_y = |p(1)| >= geo_fence_[1];     // y 对称 ±
judge_z =  p(2)  >= geo_fence_[2];     // z 仅上限！
judge   = judge_x || judge_y || judge_z;
```

- **`z` 只判上限**，没有下限保护（贴地不会触发）。
- 默认 `auto_land_on_geofence_ = false`：越界**只 `ROS_WARN` 告警**，不限制轨迹、不压低控制量（[se3_ctrl.cpp:247-248](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L247-L248)）。
- 设为 `true` 时：**同时满足「越界 && auto_land_on_geofence && 当前不在 LAND 模式」** 才切到 PX4 `AUTO.LAND` 并转入 `LANDED`（[se3_ctrl.cpp:250-255](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L250-L255)）。

其他安全机制散见全文：进 OFFBOARD 前维持 setpoint 流、收到新轨迹前原地保持、被接管即降级、误差饱和限幅、里程计陈旧即停发、真机不自动解锁/不自动 OFFBOARD。

---

## 13. 参数全表

### 13.1 部署脚本里的 `SE3_*`（运行时 `rosparam`）
来自 [start_real_px4_mid360_fastlio.sh:40-65](../scripts/start_real_px4_mid360_fastlio.sh#L40-L65)，在 [第 378 行](../scripts/start_real_px4_mid360_fastlio.sh#L378) 通过 `rosparam set` 注入。**真机运行时以启动脚本注入值为准**；`se3_ctrl.cpp` 里的 `nh.param(..., fallback)` 只是脚本没设置参数时才会用到的兜底值。

| 环境变量 | 默认值 | 节点参数 | 含义 |
|---|---|---|---|
| `SE3_ENABLE_SIM` | `false` | `enable_sim` | 仿真模式；真机务必 false |
| `SE3_AUTO_REQUEST_OFFBOARD` | `false` | `auto_request_offboard` | 自动切 OFFBOARD（仅 sim 生效） |
| `SE3_AUTO_REQUEST_ARM` | `false` | `auto_request_arm` | 自动解锁（仅 sim 生效） |
| `SE3_AUTO_LAND_ON_GEOFENCE` | `false` | `auto_land_on_geofence` | 越界自动降落；false 仅告警 |
| `SE3_TAKEOFF_HEIGHT` | `0.3` | `takeoff_height` | 期望 z 初值/日志阈值（见 §6 澄清） |
| `SE3_ENABLE_THRUST_ESTIMATION` | `false` | `enable_thrust_estimation` | 在线推力辨识；默认关 |
| `SE3_USE_ACCELERATION_FEEDFORWARD` | `true` | `use_acceleration_feedforward` | 使用轨迹加速度前馈 |
| `SE3_USE_YAW_RATE_FEEDFORWARD` | `true` | `use_yaw_rate_feedforward` | 读取轨迹偏航角速率；当前只进入被忽略的 `bodyrates`，实际 PX4 输出不受它影响 |
| `SE3_MAX_FEEDFORWARD_ACC` | `1.2` | `max_feedforward_acc` | 加速度前馈限幅 (m/s²)，高于当前规划 `max_acc=0.8` |
| `SE3_HOVER_PERCENT` | `0.90` | `hover_percent` | **悬停油门**，最关键；本机 MID-360 + Jetson 载荷实测偏高 |
| `SE3_MAX_HOVER_PERCENT` | `0.95` | `max_hover_percent` | `T_a_` 下限对应的最大悬停油门 |
| `SE3_MIN_OUTPUT_THRUST` | `0.20` | `min_output_thrust` | 输出油门下限 |
| `SE3_MAX_OUTPUT_THRUST` | `1.00` | `max_output_thrust` | 输出油门上限；必须高于真实悬停油门，否则会被钳到悬停之下 |
| `SE3_GEOFENCE_X` | `2.0` | `geo_fence/x` | x 半幅围栏 (±) |
| `SE3_GEOFENCE_Y` | `2.0` | `geo_fence/y` | y 半幅围栏 (±) |
| `SE3_GEOFENCE_Z` | `1.8` | `geo_fence/z` | z 上限围栏 |
| `SE3_KI_PZ` | `0.0` | `ki_pz` | 竖直积分增益，0=关闭；在线整定后填入（见 §8.7） |
| `SE3_INT_LIMIT_Z` | `5.0` | `int_limit_z` | 积分抗饱和钳位 [m·s] |

### 13.2 代码中硬编码的增益
见 [se3_ctrl.cpp:54-71](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L54-L71)，已在第 8.4 节列出。这些不是脚本参数，需改源码或用 dynamic_reconfigure。

> 注意：`se3_ctrl.cpp:36-41` 中的 `takeoff_height=2.0`、`hover_percent=0.45`、`max_output_thrust=0.85` 等是 **C++ fallback 默认值**，不是本项目真机脚本默认值。只要按 `start_real_px4_mid360_fastlio.sh` 启动，实际会被 §13.1 的 rosparam 覆盖。

### 13.3 在线调参（dynamic_reconfigure）
`DynamicTuneCallback`（[se3_ctrl.h:76-119](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_ctrl.h#L76-L119)）允许运行时通过 `rqt_reconfigure` 改全部 `Kp_*/Kd_*` 和 `limit_*`：回调把新值写入成员变量后调用 `se3_controller_.setup()` 应用（[se3_ctrl.h:111-114](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_ctrl.h#L111-L114)）。
（注意 §8.5 与 §11：受 `have_last_err_` 和 `type_mask` 影响，改 `Kd_*`、`Kp_a`、`Kp_q` 当前不会改变 PX4 实际执行的姿态/油门；优先调 `Kp_p/Kp_v` 与推力标定。）

---

## 14. 与规划器虚拟天花板/地板的关系

二者都「限制高度」，但工作在不同层级，属于**分层防御**，不重复：

| | 规划器虚拟天花板/地板 | 控制器地理围栏 |
|---|---|---|
| 参数 | `virtual_ceil` / `virtual_ground`（`grid_map`） | `geo_fence/z` 等（本控制器） |
| 部署变量 | `DIFF_PLANNER_VIRTUAL_CEIL=1.6` / `_GROUND=0.1` | `SE3_GEOFENCE_Z=1.8` |
| 作用层级 | **规划层**：超过的空间被标记为不可通行 | **控制层**：监控**实际**里程计位置 |
| 作用对象 | 规划出来的**轨迹** | 飞机**真实**到达的位置 |
| 触发后 | 规划器不生成越界轨迹 | 仅告警（默认）/ 切 `AUTO.LAND`（开启时） |

即使规划器严格遵守 `virtual_ceil`，飞机真实位置仍可能因跟踪误差、状态估计漂移、外界扰动而冲过去——围栏是独立于规划器的最后一道底线。**建议 `SE3_GEOFENCE_Z` 略高于 `VIRTUAL_CEIL`**（当前 1.8 > 1.6），这样正常贴顶飞行不会误触；只有在你启用 `auto_land_on_geofence=true` 时围栏才真正有「兜底降落」的意义。

---

## 15. 调参与排障指南

### 起步顺序
1. **先标 `hover_percent`**：用遥控器手动悬停，读飞控日志/地面站的实际悬停油门，把 `SE3_HOVER_PERCENT` 设到接近值。当前启动脚本默认 `0.90` 是针对本机 MID-360 + Jetson 载荷的实测配置，不是通用值。
2. 确认 `min/max_output_thrust` 给足余量（当前脚本默认 0.20–1.00）。如果真实悬停油门已经接近 1.0，控制器没有足够爬升余量。
3. 位置/速度跟踪不够紧再动 `Kp_p / Kp_v`（用 dynamic_reconfigure 在线试）。

### 常见故障

| 现象 | 可能原因 | 排查方向 |
|---|---|---|
| 切 OFFBOARD 后掉高/蹿高 | `hover_percent` 与真机不符 | 重标悬停油门；必要时开 `enable_thrust_estimation` |
| 进不去 OFFBOARD | 真机本就不自动切，需飞手遥控器操作；或 setpoint 流中断 | 确认有里程计、`/mavros/state` 正常；检查 §6 |
| 收到轨迹但不跟随 | 未解锁/非 OFFBOARD 时轨迹被丢弃 | 看是否打印 "Ignoring trajectory until armed and OFFBOARD" |
| 位置振荡 | `Kp` 偏高 | 调小 `Kp_p/Kp_v`（注意 `Kd` 当前无效，见 §8.5） |
| 飞行中突然降落 | 触发了围栏 + `auto_land_on_geofence=true` | 检查围栏尺寸与实际飞行范围；看 "obs Land enabled" 日志 |
| 频繁告警 "Geofence exceeded" | 实际位置越界但未开自动降落 | 调大围栏或人工接管 |
| 完全不发姿态指令 | 里程计 >0.1s 陈旧，`calControl` 返回 false | 检查 FAST-LIO / odom 频率 |

---

## 16. 已知细节与陷阱（gotchas 汇总）

1. **`TAKEOFF` 状态是死代码**——枚举里有，但从不被进入（§6）。
2. **真机不自动 OFFBOARD、不自动解锁**——`trigger_offboard/arm` 仅在 `sim_enable && auto_request_*` 时动作（§6）。
3. **`takeoff_height` 不真正驱动起飞**——有里程计时被当前位姿覆盖，仅作 odom 缺失兜底 + 一条日志（§6）。
4. **微分项 `Kd_*` 当前恒为 0**——`have_last_err_` 永不置 true，使 `d_err ≡ 0`；靠调 `Kd` 加阻尼不起作用（§8.5）。
5. **默认无积分项，但可选开启竖直积分**——无积分模式下稳态推力偏置只能靠高度误差换取补偿；随电压/载荷漂移会缓慢掉高，可用 `ki_pz` 开启竖直积分项自适应补偿（默认 0=关闭，§8.7）。
6. **`bodyrates` 被丢弃**——`type_mask` 忽略角速率，PX4 只用姿态+油门；因此 jerk、yaw_rate、`Kp_a`、`Kp_q` 当前都不要当作有效输出通道来调（§11）。
7. **`Kp_w / Kd_q / Kd_w` 在激活路径未使用**——只出现在被注释的替代实现里（§9.4）。
8. **`Kp_q` 的 xy=5.5、`Kp_w` 的 xy=1.5**——不同矩阵，勿混淆；且当前二者都不改变实际 PX4 姿态/油门输出（§8.4、§11）。
9. **围栏 z 仅判上限，无下限**；x/y 为对称 ±（§12）。
10. **推力估计默认关闭**——`T_a_` 固定 = g/hover_percent，`hover_percent` 标不准就会掉/蹿（§10.3）。
11. **NED/`R_mid` 分支默认不执行**——本部署 `enu_frame_=true`（§5）。
12. **里程计速度是机体系**——`vel_in_body_=true` 会把它旋到世界系（§5）。

---

*本文档基于源码逐行核对（含对抗式校验）。若控制器代码后续修改，请同步更新本文与上述行号引用。*
