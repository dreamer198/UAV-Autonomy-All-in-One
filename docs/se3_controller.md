# SE3 控制器原理、调参与排障

## 接口

```text
selected planner → planner_gateway → /command/trajectory
                                      │
                                      ▼
                                    SE3
                                      │
                                      ▼
                       /mavros/setpoint_raw/attitude → PX4
```

| 输入 | 作用 |
|---|---|
| `/mavros/local_position/odom` | 当前位姿、速度和角速度 |
| `/mavros/imu/data` | 当前姿态和线加速度 |
| `/mavros/state` | 连接、解锁和飞行模式 |
| `/command/trajectory` | 期望位置、速度、加速度和 yaw |

| 输出话题 | 作用 |
|---|---|
| `/mavros/setpoint_raw/attitude` | 姿态四元数和归一化推力 |
| `/mavros/setpoint_position/local` | OFFBOARD 交接前的位置 setpoint |
| `/desire_odom_pub` | 控制器实际采用的期望状态 |

控制器还提供 `/land`（`std_srvs/SetBool`）服务。只有 `data=true` 才会进入降落
流程并循环请求 PX4 `AUTO.LAND`；该服务不会强制解除武装。

控制定时器的标称周期为 10 ms。控制器使用 ENU，并固定将 MAVROS odom 的
`twist.linear` 视为机体系速度，再用当前姿态旋转到世界系参与反馈。代码不会读取
`child_frame_id` 自动判断速度坐标系，因此输入必须遵守这一约定。

## 模式和命令门禁

当前公共配置将 `auto_request_offboard` 和 `auto_request_arm` 均设为 `false`，
因此 SE3 不主动解锁或进入 OFFBOARD。`sim.sh arm`、`real.sh arm` 和 Mission
执行器负责 PX4 原生起飞与模式交接。

进入 OFFBOARD 后，控制器先保持当前位置，直到收到合法轨迹。轨迹必须：

- 来自 `/planner_gateway`；
- 在 armed/OFFBOARD 下接收；
- 至少包含一个 transform 和 velocity；控制器只使用第一条轨迹点，加速度可选；
- 包含有限的位置、速度、可选加速度和有效姿态四元数；
- 在里程计、IMU、MAVROS 状态有效时接收。

`trajectory_command_timeout` 默认为 `0.08 s`。命令流中断后，控制器把移动 setpoint
替换为当前位置零速度保持；后续合法命令可以恢复跟踪。离开 OFFBOARD、解除武装或
MAVROS state、里程计、IMU 失效都会清除旧轨迹状态。

输入失效时：

- MAVROS 状态失效：停止姿态/推力输出，由 PX4 OFFBOARD-loss failsafe 处理；
- 里程计或 IMU 失效且状态仍可信：停止输出并请求 `AUTO.LOITER`；
- 公共定位保护确认定位故障：锁存故障并请求 `AUTO.LAND`。

## 控制链路

```text
位置误差
  → 修正期望速度
速度误差
  → 修正期望加速度
轨迹加速度前馈 + 重力补偿 + z 积分
  → 期望推力方向和大小
推力方向 + yaw
  → 姿态四元数
```

位置环使用 `Kp_p`、`Kd_p`，速度环使用 `Kp_v`、`Kd_v`。误差和微分误差均有限幅；
控制重置会清除微分历史。

加速度前馈由 `use_acceleration_feedforward` 控制，并按
`max_feedforward_acc` 逐轴裁剪。随后叠加重力和 z 轴积分补偿。

推力计算为：

```text
T_a = g / hover_percent
normalized_thrust = projected_acceleration / T_a
```

结果限制在 `min_output_thrust` 与 `max_output_thrust` 之间。上一拍推力饱和时积分
冻结，防止继续 wind-up。

### 实际生效的姿态接口

控制器虽然计算 body rate，但发送的 `AttitudeTarget` 设置了
`IGNORE_ROLL_RATE`、`IGNORE_PITCH_RATE` 和 `IGNORE_YAW_RATE`。PX4 实际使用：

- 姿态四元数；
- 归一化推力。

因此主要调节对象是 `hover_percent`、推力上下限、`Kp_p/Kd_p`、`Kp_v/Kd_v`、
`ki_pz` 和加速度前馈。`Kp_a/Kd_a`、`Kp_q/Kd_q`、`Kp_w/Kd_w` 以及 yaw-rate
前馈只影响当前被忽略的 body-rate 支路。

## 围栏

SE3 围栏检查实际 MAVROS 里程计：

- x、y 为对称边界；
- z 只检查上限；
- `auto_land_on_geofence=false` 时仅告警；
- 开启 `auto_land_on_geofence` 后才请求 `AUTO.LAND`。

Planner 地图边界决定能否规划，SE3 围栏监控实际位置，两者不能互相替代。

## 配置

[controller.launch](../common/launch/controller.launch) 依次加载公共配置、当前规划器的
控制器覆盖和载体配置；后加载的文件覆盖同名参数。规划器 manifest 的
`controller_config` 决定第二层使用哪个文件。

| 文件 | 内容 |
|---|---|
| [common/config/controller.yaml](../common/config/controller.yaml) | 模式门禁、前馈、输入/命令超时和命令发布者 |
| [common/config/controller_plugin_default.yaml](../common/config/controller_plugin_default.yaml) | Diff、Fast Kino/Topo 使用的空覆盖 |
| [planning/plugins/super/controller.yaml](../planning/plugins/super/controller.yaml) | SUPER 专用反馈增益、姿态对齐和前馈上限 |
| [simulation/config/controller.yaml](../simulation/config/controller.yaml) | SITL 推力、积分和围栏 |
| [deployment/config/controller.yaml](../deployment/config/controller.yaml) | 真机推力、积分和围栏 |

`Kp_*`、`Kd_*`、误差限幅、`ki_pz` 和 `int_limit_z` 由 dynamic reconfigure
管理；启动时先读取上述 YAML，未覆盖项使用
[`tune.cfg`](../third_party/Diff-Planner-PX4/src/se3_controller/cfg/tune.cfg) 中的
默认值。运行中可通过 dynamic reconfigure 临时调整，重启后恢复 YAML/编译默认值；
修改 `tune.cfg` 才需要重新构建。

若某个规划器需要独立增益，应在该插件目录提供控制器覆盖，并由 manifest 引用，不要
修改所有规划器共用的默认值。悬停推力和围栏由仿真/真机载体 YAML 管理，输入超时和
命令来源由公共 YAML 管理。

## 调参顺序

### 1. 标定 `hover_percent`

`hover_percent` 是当前机体和载荷抵消重力所需的归一化推力，不能照抄其他机体或
仿真值。

1. 使用已经验证的 PX4 人工定点模式稳定悬停；
2. 从飞控日志读取稳定阶段推力，排除加速、阵风和地效；
3. 写入 [`deployment/config/controller.yaml`](../deployment/config/controller.yaml)；
4. 在低风险 OFFBOARD 悬停中复核。

切入 OFFBOARD 后立即下沉通常表示基础推力偏低，立即上蹿通常表示偏高。若推力接近
上限，应先处理载荷、动力或电池。

### 2. 整定 `ki_pz`

只有在基础悬停推力正确、定位稳定且推力未饱和后才调整 `ki_pz`。

1. 先记录 `ki_pz=0` 的悬停误差；
2. 每次只小幅提高 `ki_pz`；
3. 每次修改后重新进入 OFFBOARD；dynamic reconfigure 更新也会立即清空积分；
4. 选择能消除稳态高度偏差且不过冲的最小值。

| 现象 | 处理 |
|---|---|
| 高度存在稳定偏差，推力未饱和 | 小幅提高 `ki_pz` |
| 过冲或缓慢上下振荡 | 降低 `ki_pz` |
| 推力长期贴住上下限 | 停止调积分，检查动力和基础推力 |
| 误差伴随里程计跳变 | 先修定位 |

积分不能修复最大推力不足、定位错误、命令中断或动态前馈不足。

### 3. 动态轨迹

悬停稳定后再检查动态飞行：

1. 查看 `/mavros/setpoint_raw/attitude` 的 thrust 是否饱和；
2. 对比 `/localization/odom`、`/mavros/local_position/odom` 和
   `/desire_odom_pub`；
3. 确认 `max_feedforward_acc` 覆盖当前规划器的加速度范围；
4. 最后调整 `Kp_p/Kd_p` 和 `Kp_v/Kd_v`，每次只改一组。

## rosbag 判读

| 数据 | 用途 |
|---|---|
| `/desire_odom_pub` 与 `/mavros/local_position/odom` | 期望与实际误差 |
| `/localization/odom` 与 MAVROS odom | 定位适配与 PX4 EKF 一致性 |
| `/mavros/setpoint_raw/attitude` | 姿态、推力和饱和 |
| `/command/trajectory` | gateway 发出的 SE3 输入；是否被控制器接受还需结合状态与日志判断 |
| `/mavros/state` | OFFBOARD、解锁和人工接管时刻 |

仿真 bag 位于 `runtime/simulation/flight_bags/`，真机 bag 位于
`runtime/flight_bags/`。

## 排错顺序

1. `/mavros/state` 是否连接、armed 且为 OFFBOARD；
2. MAVROS state、odom 和 IMU 是否持续更新且时间戳有效；
3. `/command/trajectory` 是否持续更新；
4. `/desire_odom_pub` 是否符合当前目标；
5. `/mavros/setpoint_raw/attitude` 是否持续发布且推力未饱和；
6. PX4 是否已进入 `AUTO.LOITER`、`AUTO.LAND`，或触发 OFFBOARD/EKF failsafe。

真机调参必须在有净空的场地进行，飞手全程准备遥控接管，并保持 rosbag 录制。
