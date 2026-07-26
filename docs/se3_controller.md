# SE3 控制器

本文说明本仓库中 SE3 控制器的实际输入、控制链路、安全门禁和已知限制。控制器
调参流程见 [控制器标定与高度排障](controller_tuning.md)。

## 1. 作用和接口

SE3 控制器位于轨迹规划与 PX4 之间：

```text
Diff-Planner PositionCommand
           │
           ▼
trajectory_msg_converter.py
           │ /command/trajectory
           ▼
     SE3 控制器
           │ /mavros/setpoint_raw/attitude
           ▼
      MAVROS → PX4
```

主要输入：

| 话题 | 用途 |
|---|---|
| `/mavros/local_position/odom` | 当前位姿、速度和角速度 |
| `/mavros/imu/data` | 当前姿态和线加速度 |
| `/mavros/state` | 连接、解锁和飞行模式 |
| `/command/trajectory` | 规划器的期望位置、速度、加速度和 yaw |
| `/desire_odom` | 备用期望状态接口，默认规划链路不使用 |

主要输出：

| 接口 | 用途 |
|---|---|
| `/mavros/setpoint_raw/attitude` | PX4 姿态四元数和归一化推力 |
| `/mavros/setpoint_position/local` | OFFBOARD 交接前的当前位置预热 setpoint |
| `/desire_odom_pub` | 实际送入控制器的期望状态回显，供 RViz 和 rosbag 使用 |
| `/land` | 请求 PX4 `AUTO.LAND` |

控制循环周期为 10 ms。

## 2. 模式交接和轨迹门禁

SE3 节点默认不会自行解锁，也不会自行请求 OFFBOARD。公共配置中的
`auto_request_arm` 和 `auto_request_offboard` 均关闭；仿真和真机的
`sim.sh arm` / `real.sh arm` 使用共享执行器完成：

1. PX4 原生 `AUTO.TAKEOFF`；
2. 达到并稳定在请求高度；
3. 验证当前位置预热 setpoint；
4. 自动切入 OFFBOARD；
5. 验证 SE3 姿态/推力输出。

源码保留了 `TAKEOFF` 状态枚举，但默认状态机不进入该分支；原生起飞由上述仓库级
执行器完成。

SE3 在等待 OFFBOARD 时持续把新鲜里程计位置作为预热 setpoint。飞机已解锁并进入
OFFBOARD 后，它先保持切换瞬间的位置，直到收到一条满足门禁的新轨迹。

`/command/trajectory` 只有同时满足以下条件才会被接受：

- PX4 已解锁且处于 OFFBOARD；
- `/mavros/state`、里程计和 IMU 均有效且未超时；
- 消息至少包含位置和速度。公共轨迹转换器会在此前拒绝非有限数值。

离开 OFFBOARD 或解除锁定后，控制器清除“已接收新轨迹”状态，并锁定新鲜里程计
位置。任一控制输入失效后，旧轨迹会失效；数据恢复时先保持当前位置，收到新轨迹
后才恢复跟踪。

## 3. 坐标约定

控制器内部使用 ENU 世界系。`/mavros/local_position/odom` 的 twist 按
`nav_msgs/Odometry` 约定位于 `child_frame_id`，当前代码将其由机体系旋转到
世界系后再计算误差。

最终姿态还会用 IMU 姿态与 odom 姿态之间的关系对齐到 FCU 参考系。当前部署
固定使用 ENU；保留的 NED 转换分支不参与默认运行。

## 4. 实际生效的控制链路

控制器按级联方式工作：

```text
位置误差
  → 修正期望速度
速度误差
  → 修正期望加速度
轨迹加速度前馈 + 重力补偿 + 竖直积分
  → 期望推力方向和大小
期望推力方向 + yaw
  → 姿态四元数
```

### 4.1 位置、速度和加速度

位置误差经 `Kp_p` 修正期望速度，速度误差经 `Kp_v` 修正期望加速度。轨迹
加速度前馈由 `use_acceleration_feedforward` 控制，并按
`max_feedforward_acc` 逐轴限幅。随后叠加重力补偿。

IMU 加速度误差支路只影响当前被忽略的 body-rate，详见 5.2 节。

### 4.2 竖直积分

`ki_pz` 只作用于 z 轴位置误差，用于补偿悬停推力模型的缓慢、近似恒定偏差。
积分器包含：

- 积分幅值限制 `int_limit_z`；
- 上一拍推力饱和时冻结积分；
- 模式交接、人工接管或落地后清零。

积分不能补偿推力已经饱和、定位错误或动态前馈不足。整定顺序和判读方法见
[控制器标定与高度排障](controller_tuning.md)。

### 4.3 姿态和推力

期望加速度方向决定机体 z 轴，yaw 决定绕 z 轴的朝向。当前实现使用 Hopf
fibration 形式生成期望姿态。

推力计算分为三步：

1. 将期望加速度投影到当前机体 z 轴；
2. 用 `T_a = g / hover_percent` 映射为归一化推力；
3. 限制在 `min_output_thrust` 与 `max_output_thrust` 之间。

因此 `hover_percent` 是首要载体标定参数。在线推力估计默认关闭，此时运行中
不会自动修正 `T_a`。

## 5. 容易误解的控制支路

### 5.1 `Kd_*`

`Kd_p`、`Kd_v` 和 `Kd_a` 已参与位置、速度和加速度误差的微分反馈。第一拍微分
为零；此后按相邻误差之差除以实际控制周期计算，并经过 `limit_d_err_*` 限幅。
模式交接、输入失效或控制重置会清除微分历史，避免沿用旧误差。

### 5.2 body-rate 支路

控制器会计算 body-rate，但发送 `mavros_msgs/AttitudeTarget` 时设置了
`IGNORE_ROLL_RATE`、`IGNORE_PITCH_RATE` 和 `IGNORE_YAW_RATE`。PX4 当前只使用：

- 姿态四元数；
- 推力。

因此 jerk、yaw-rate、`Kp_a`、`Kd_a` 和 `Kp_q` 对 body-rate 的影响不会进入
PX4 执行链路。主要有效调参对象是 `hover_percent`、推力上下限、`Kp_p`、
`Kd_p`、`Kp_v`、`Kd_v` 和 `ki_pz`。

## 6. 安全行为

### 6.1 控制输入失效

`state_timeout`、`odom_timeout` 和 `imu_timeout` 分别限制 MAVROS 状态、里程计和
IMU 的有效时长。控制器同时检查回调接收时间；消息包含非零时间戳时也检查源时间，
因此反复回放旧消息不能维持“新鲜”状态。

输入失效时，控制器立即停止姿态/推力输出并清除旧轨迹：

- 状态仍新鲜，但里程计或 IMU 失效：限频请求 PX4 `AUTO.LOITER`；
- MAVROS 状态本身失效：无法可信判断模式，不发送模式请求，交由 PX4 的
  OFFBOARD-loss failsafe；
- 数据恢复：保持当前里程计位置，等待一条新轨迹。

公共 `localization_guard.py` 还会独立检测定位故障、锁存故障并请求
`AUTO.LAND`；完整栈需要重启后才能再次执行自主飞行。

### 6.2 控制器围栏

控制器围栏检查实际 MAVROS 里程计：

- x、y 使用对称上下界；
- z 只检查上限，没有地面下限；
- `auto_land_on_geofence=false` 时只打印警告，不限制轨迹或控制输出；
- 开启 `auto_land_on_geofence` 后才会请求 `AUTO.LAND`。

因此默认围栏是监控阈值，不是自动安全边界。Planner 的虚拟地面/天花板作用于
规划空间，两者不能互相替代。

### 6.3 人工接管

飞行中离开 OFFBOARD 会立即停止轨迹跟踪。仓库级 `arm` 或 Mission 执行器不会
在检测到人工模式切换后主动抢回 OFFBOARD。

## 7. 配置来源

[controller.launch](../common/launch/controller.launch) 先加载公共参数，再加载
载体参数；后者覆盖同名值。

| 文件 | 内容 |
|---|---|
| [common/config/controller.yaml](../common/config/controller.yaml) | 模式门禁、前馈、输入超时、模式重试和推力估计开关 |
| [deployment/config/controller.yaml](../deployment/config/controller.yaml) | 真机悬停推力、推力限制、积分和围栏 |
| [simulation/config/controller.yaml](../simulation/config/controller.yaml) | SITL 载体悬停推力、推力限制、积分和围栏 |

数值应直接查看 YAML、dynamic reconfigure 配置和节点启动日志，避免从文档复制
参数快照。`Kp_*`、`Kd_*` 和误差限幅由 dynamic reconfigure 管理，其默认值与
控制器启动值一致。界面会整组提交参数；真机修改前仍应核对全部字段。

## 8. 排障顺序

控制器无输出或飞机不跟踪时，按以下顺序检查：

1. `/mavros/state` 是否已连接、解锁并处于 OFFBOARD；
2. `/mavros/state`、`/mavros/local_position/odom` 和 `/mavros/imu/data`
   是否持续更新且时间戳有效；
3. `/command/trajectory` 是否持续更新；
4. `/desire_odom_pub` 是否与当前目标一致；
5. `/mavros/setpoint_raw/attitude` 是否持续发布且推力未饱和；
6. PX4 是否已进入 `AUTO.LOITER`，或触发 OFFBOARD、EKF 等 failsafe。

高度问题的进一步判断见 [控制器标定与高度排障](controller_tuning.md)。
