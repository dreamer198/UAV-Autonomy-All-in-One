# 控制器标定与高度排障

本文给出悬停推力标定、`ki_pz` 整定和轨迹跟踪高度排障的实用流程。控制律背景见
[SE3 控制器](se3_controller.md)。

## 1. 先区分问题

| 现象 | 优先检查 |
|---|---|
| 一进入 OFFBOARD 就明显掉高或蹿高 | `hover_percent`、推力映射、姿态方向 |
| 悬停最终停在目标高度上方或下方 | `hover_percent`，再考虑 `ki_pz` |
| 爬升时推力顶到上限且高度跟不上 | 推力余量、载荷、电池和轨迹加速度 |
| 动态飞行才掉高，悬停正常 | 定位、推力饱和、前馈限幅和跟踪增益 |
| 上下反复振荡 | `ki_pz` 或位置/速度增益偏大，或定位噪声 |
| 姿态/推力突然停止发布 | 状态、里程计或 IMU 超时，或离开 OFFBOARD |

核心原则：

- `hover_percent` 决定基础悬停推力，必须先标定；
- `ki_pz` 只补偿缓慢、近似恒定的稳态偏差；
- 动态掉高应先查推力饱和、定位和加速度前馈，不能只加积分。

## 2. 安全前提

真机调参前必须满足：

- 场地净空，螺旋桨和人员保持安全距离；
- 遥控器和人工模式切换已验证，飞手全程准备接管；
- PX4 failsafe、external vision、外参和里程计已经独立验证；
- 使用与实际任务相同的机体、桨、电池和载荷；
- 从低风险悬停开始，再逐步增加水平和竖直运动；
- 默认 rosbag 正常记录，便于事后对比期望、反馈和推力。

不要在旋翼运行时用手推压飞机，也不要故意设置明显不足的悬停推力制造大幅掉高。

## 3. 标定 `hover_percent`

`hover_percent` 是抵消重力所需的归一化总推力。它不等于通用机型常数，也不能
直接照抄另一台飞机或仿真值。

建议流程：

1. 使用 PX4 已验证的人工定点模式稳定悬停；
2. 从飞控日志或地面站读取稳定悬停阶段的实际推力；
3. 排除明显加速、阵风和地效阶段，取稳定区间；
4. 将结果写入
   [deployment/config/controller.yaml](../deployment/config/controller.yaml)；
5. 重启真机栈，在低风险 OFFBOARD 悬停中复核。

判断结果：

- 切入后立即下沉：基础推力可能偏低；
- 切入后立即上蹿：基础推力可能偏高；
- 推力已接近上限：先解决载荷、动力或电池问题，不要依靠积分补救。

仿真参数位于
[simulation/config/controller.yaml](../simulation/config/controller.yaml)，只适用于当前
SITL 载体模型。

## 4. 整定 `ki_pz`

只有在 `hover_percent` 已接近真实悬停值、定位稳定且输出未饱和后，才整定
`ki_pz`。

### 4.1 它能解决什么

`ki_pz` 可减少推力模型存在小偏差时的稳态高度误差，例如电池状态或载荷带来的
缓慢变化。它不能解决：

- 最大推力不足；
- 定位漂移或跳变；
- OFFBOARD setpoint 中断；
- 动态加速度前馈被裁剪；
- 规划轨迹本身过于激进。

### 4.2 操作方法

1. 保持已标定的 `hover_percent`，先记录 `ki_pz=0` 时的悬停表现；
2. 从较小增量开始提高 `ki_pz`，每次只修改这一项；
3. 每个值都重新进入 OFFBOARD，使积分从零开始；
4. 观察高度误差是否平顺收敛，以及是否出现过冲或持续振荡；
5. 选择能消除稳态偏差且不过冲的最小值；
6. 写回
   [deployment/config/controller.yaml](../deployment/config/controller.yaml)，重启后复核。

dynamic reconfigure 会整组提交控制增益，其默认值已与控制器启动基础值对齐。
真机常规整定仍应优先修改 YAML 后重启；在线调参前先读取并保存完整参数组。具体
增量和最终数值随机体而异，不应把文档示例当作推荐值。

### 4.3 判读

| 现象 | 处理 |
|---|---|
| 长时间仍有稳定高度偏差，推力未饱和 | 小幅提高 `ki_pz` |
| 高度过冲或缓慢上下振荡 | 降低 `ki_pz` |
| 推力长期贴住上限或下限 | 停止调积分，先解决推力余量或基础标定 |
| 误差伴随里程计跳变 | 先处理定位，控制增益无法修复错误状态估计 |

## 5. 动态轨迹掉高

悬停稳定后仍可能在轨迹跟踪阶段出现高度误差。按以下顺序检查。

### 5.1 推力饱和

同时爬升、水平倾斜、抗风和跟踪误差都会消耗推力余量。检查
`/mavros/setpoint_raw/attitude` 中的 thrust 是否贴近配置上下限。

饱和时积分器会冻结，但无法产生物理上不存在的推力。应降低轨迹加速度、减轻
载荷、改善动力或更换电池。

### 5.2 定位健康

对比：

- `/localization/odom`；
- `/mavros/local_position/odom`；
- `/desire_odom_pub`。

检查掉频、跳变、缓慢漂移及 FAST-LIO 到 PX4 EKF 的回灌差异。若里程计超过
`odom_timeout`，SE3 会停止姿态/推力输出；MAVROS 状态仍新鲜时还会请求
`AUTO.LOITER`。

### 5.3 加速度前馈

公共 [controller.yaml](../common/config/controller.yaml) 中的
`max_feedforward_acc` 应覆盖
[Diff 插件 planner.yaml](../planning/ros_pkgs/sim2real_diff_adapter/config/planner.yaml) 的规划加速度范围。若提高 Planner
加速度而不调整前馈限幅，控制器会裁剪期望加速度，动态误差随之增大。

### 5.4 位置和速度增益

确认推力和定位正常后，再评估 `Kp_p`、`Kp_v`、`Kd_p` 和 `Kd_v`。微分项使用
实际控制周期并受 `limit_d_err_*` 限制。每次只修改一个参数组并保留完整 rosbag。

## 6. 录包对比

仿真和真机默认记录控制、估计和规划白名单话题。仿真文件位于
`runtime/simulation/flight_bags/`，真机文件位于 `runtime/flight_bags/`。
录制采用分片和保留数量上限；实际设置以 `sim.sh` / `real.sh` 的启动输出为准。

建议至少对比：

| 数据 | 用途 |
|---|---|
| `/desire_odom_pub` 与 `/mavros/local_position/odom` | 期望与实际跟踪误差 |
| `/localization/odom` 与 MAVROS odom | 定位适配和 EKF 一致性 |
| `/mavros/setpoint_raw/attitude` | 姿态、推力及饱和 |
| `/command/trajectory` | 控制器实际收到的 P/V/A/yaw |
| `/mavros/state` | OFFBOARD、解锁和人工接管时刻 |

## 7. 完成条件

调参结束前确认：

- `hover_percent` 来自当前机体和载荷的实测；
- OFFBOARD 悬停没有明显蹿高、掉高或稳态偏差；
- 推力上下均有余量；
- 真实轨迹中没有持续高度振荡；
- Planner 加速度与控制器前馈限幅一致；
- 动态定位无明显掉频、跳变或漂移；
- 配置已写回 YAML，并在重启后复核。
