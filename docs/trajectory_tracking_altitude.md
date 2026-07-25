# 规划轨迹跟踪阶段的掉高风险与排障

> 适用场景：你已经按 [ki_pz 整定指南](ki_pz_tuning_guide.md) 把竖直积分项 `ki_pz` 调好、**悬停/定高已经很稳**，现在要把 Diff-Planner 的局部避障规划接进来跑自主飞行。本文回答一个关键问题：
>
> **跟踪规划出的轨迹时，无人机还会不会跟不上、掉高度？会因为什么？怎么防、怎么定位？**
>
> 控制器原理见 [se3_controller.md](se3_controller.md)。

---

## 先读导引

这篇是“接上规划器以后，高度为什么还可能不稳”的专题排障文档。阅读前最好已经完成两件事：

- `hover_percent` 已经接近真实悬停油门；
- 按 [ki_pz_tuning_guide.md](ki_pz_tuning_guide.md) 验证过 OFFBOARD 悬停不再慢慢掉高。

如果还没完成，先不要把问题归因到规划器。因为悬停基础没稳时，轨迹跟踪里的现象会混在一起，很难判断。

快速分流：

| 观察到的现象 | 更可能的原因 | 先看 |
|---|---|---|
| 爬升段油门贴近 `max_output_thrust`，高度跟不上 | 推力余量不够 | §3.①、§5 |
| `/localization/odom` 跳变、掉频，或它与 MAVROS 高度明显分离 | 真机 FAST-LIO/外参/EKF 链路退化 | §3.②、§6 |
| 期望 z 和实际 z 有短时误差，但油门未饱和 | 瞬态跟踪滞后或增益问题 | §3.③、§6 |
| 跟踪竖直轨迹后上下振荡 | `ki_pz` 动态下偏大 | §3.④、§5 |
| 提高规划加速度后才出问题 | 前馈加速度被裁剪或推力不足 | §3.⑤、§7 |

一句话：**`ki_pz` 修的是“悬停慢漂”，轨迹跟踪掉高要先查推力余量和里程计健康。**

---

## 0. 一句话结论

竖直积分项**只解决「稳态悬停推力偏置」那一种掉高**（随电压/载荷缓慢匀速下沉）。跟踪轨迹时仍可能掉高，但**诱因换了**——主要是 **① 爬升段推力余量不够** 和 **② 里程计在动态下退化**，跟悬停那种慢漂不是一回事。用当前温和限幅 + 电量充足时通常没问题；风险随**电量变低、限幅调激进、环境让 LIO 退化**而上升。

---

## 1. 链路与关键参数（已核对）

| 项 | 值 | 来源 |
|---|---|---|
| 轨迹链路 | `/drone_0_planning/pos_cmd` → [trajectory_msg_converter.py](../third_party/Diff-Planner-PX4/src/diff_planner/plan_manage/scripts/trajectory_msg_converter.py) → `/command/trajectory` → [multiDOFJointCallback](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L292) | — |
| z 前馈 | **位置/速度/加速度透传并生效**（竖直是前馈+反馈，非纯反馈）；jerk 不透传，yaw-rate 当前只进被忽略的 bodyrates | [converter L91–112](../third_party/Diff-Planner-PX4/src/diff_planner/plan_manage/scripts/trajectory_msg_converter.py#L91-L112) + [se3_ctrl.cpp L330](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L330) / [L338](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp#L338) |
| 规划限幅 | `max_vel=0.5` / `max_acc=0.8` / `max_jer=8.0`（温和） | [公共 planner.yaml](../common/config/planner.yaml) |
| 竖直范围 | `virtual_ground=0.1 ~ virtual_ceil=1.5`（local_update_range_z=1.8） | [公共 planner.yaml](../common/config/planner.yaml) |
| 加速度前馈裁剪 | `max_feedforward_acc=1.2 ≥ max_acc=0.8` → 当前**不裁** | [公共 controller.yaml](../common/config/controller.yaml) + [se3_ctrl.cpp](../third_party/Diff-Planner-PX4/src/se3_controller/src/se3_ctrl.cpp) |
| 推力 | `hover_percent`、`max_output_thrust` 按真机标定 | [真机 controller.yaml](../deployment/config/controller.yaml) |
| **悬停之上的推力余量** | `max_output_thrust - 实际悬停推力`；必须按当前电池/载荷实测 | — |

---

## 2. 积分器解决了什么 / 没解决什么

- ✅ **解决**：随电压/载荷漂移的**稳态悬停推力缺口** → 不再「悬停时缓慢匀速下沉」。
- ❌ **没解决**：下面第 3 节的所有「跟踪专属」诱因。积分项是慢的、补偿恒定偏置的，管不了动态饱和、瞬态滞后、感知退化。

---

## 3. 跟踪轨迹时的残余掉高诱因

### ① 爬升时推力饱和（头号风险）
规划器爬升（竖直加速度 `az`）所需推力近似为 `hover × (1 + az/g)`。代入 `az=0.8`，相对悬停需要约 **+8%**。例如若某次实测悬停推力已经接近 `0.93`，理想爬升推力约 `1.01`；在 `max_output_thrust=1.0` 时必然饱和。这里的数字只是计算示例，应使用当前机体、电池和载荷实测值。

抗饱和会冻结积分（不乱积），但**变不出物理上没有的推力**。
> **非对称**：下降/平飞避障不吃余量、很安全；**只有向上爬升段才吃推力**（如越过低矮障碍）。横向倾斜在温和限幅下耗推力可忽略（0.8 m/s² 仅倾 ~5°、推力 +0.3%）——吃余量的是竖直爬升，不是横向。

### ② 里程计（FAST-LIO）动态退化
真机快速平移/偏航可能让 LIO 退化（运动模糊、特征少、IMU 漂移）；仿真使用 MAVROS odom/TF 适配，不经过 FAST-LIO，因此这一项主要是真机风险。
- z 估计漂 → 控制器**忠实跟随错误的 z** → 真实高度漂；
- odom 超过 `odom_timeout`（默认 `0.2 s`）不更新 → [calControl 返回 false、不发指令](../third_party/Diff-Planner-PX4/src/se3_controller/include/se3_controller/se3_controller.hpp) → 由 PX4 Offboard failsafe 接管。

**跟踪比悬停更容易触发**，很可能也是早期「某些情况掉高」的一部分。任何控制增益都治不了，只能靠温和运动 + 监控 LIO 健康。

### ③ 瞬态跟踪滞后（比例环的活，不是积分器）
动态中 `err_p(2)` = 慢偏置（积分管）+ 快瞬态（滞后）。瞬态靠 `Kp_p/Kp_v` 刚度 + 前馈。温和限幅下很小，急竖直段会有一点。

### ④ 悬停整定的 `ki_pz` 在动态下可能偏大
积分会把 `err_p(2)` **全部**累积，含爬升/下降段的瞬态滞后（不只偏置）。所以悬停最优的 ki，在跟竖直轨迹改平的瞬间可能**轻微过冲/振荡**。
> **建议**：悬停整定出 ki 后，用真实（温和）轨迹**再验证一遍**；竖直方向出现振荡就把 `ki_pz` 往回调一档。悬停值是起点、不一定是动态终值。

### ⑤ 前馈裁剪（当前 OK，调激进就有）
`max_feedforward_acc=1.2 > max_acc=0.8` 现在不裁。若把规划 `max_acc` 调到 >1.2，z 前馈被裁 → 爬升滞后。**永远保持 `max_feedforward_acc ≥ 规划 max_acc`。**

---

## 4. 对本配置的风险结论

当前公共**温和限幅（0.5 / 0.8）+ 电量充足**时，积分器修好悬停后，跟踪应能稳住高度。真机残余掉高**主要来自**：

1. **电量低 + 爬升段**（推力余量耗尽）——头号；
2. **里程计动态退化**；

③④⑤ 为次要、可调。风险随**电量变低 / 限幅调激进 / 环境让 LIO 退化**而上升。

---

## 5. 实操防护清单（按重要性）

1. **守住推力余量**：盯 `rostopic echo /mavros/setpoint_raw/attitude/thrust`。爬升时若贴近配置的 `max_output_thrust`，就是余量见底；返航阈值应通过本机电池测试制定。`hover_percent` 设成真实悬停值，不要人为抬高。
2. **限幅保持温和**：`max_vel=0.5 / max_acc=0.8` 很好；要提速先小步加，并保持 `max_feedforward_acc ≥ max_acc`。
3. **`ki_pz` 动态再验证**：悬停调好后用真实轨迹复验，振荡就回调一档（见 [ki_pz 整定指南](ki_pz_tuning_guide.md)）。
4. **渐进放飞**：先给近、平、慢的目标 → 看 thrust 是否顶 1.0、实际 z 是否跟得上期望 z → 再上复杂/带爬升的任务。
5. **监控 LIO/odom 健康**：动态中别掉帧、别漂；温和偏航/平移降低感知压力。

---

## 6. 录包定位法（判断是哪一类）

`./launch/real.sh` 默认用 `START_ROSBAG=true` 录制控制/估计/规划白名单话题，bag 保存在宿主 `runtime/flight_bags/`。飞完对比：

| 对比 | 看什么 | 指向 |
|---|---|---|
| 期望 z（`/desire_odom_pub` 或 `/command/trajectory`） vs 控制反馈 z（`/mavros/local_position/odom`） | 跟踪滞后大小、方向 | ③瞬态 / ④积分过冲 |
| `/localization/odom` vs `/mavros/local_position/odom` | LIO/vision 回灌与 PX4 EKF 是否一致 | ②定位链路 |
| `/mavros/setpoint_raw/attitude/thrust` | 是否在爬升段顶到 `max_output_thrust` | ①推力余量 |
| FAST-LIO `/Odometry` 与公共 odom 的频率、跳变 | 是否退化、掉帧或外参错误 | ②里程计 |

哪条对上，就对应上面哪一类诱因，再按第 5 节对症处理。

---

## 7. 参数一致性快速自查

- [ ] 真机 `max_output_thrust` 与真实悬停油门之间留有余量，爬升不顶上限。
- [ ] 真机 `hover_percent` ≈ 当前载荷的真实悬停值。
- [ ] 公共 `max_feedforward_acc ≥ planner.yaml` 的 `max_acc`。
- [ ] `ki_pz` 已用真实轨迹动态验证、无竖直振荡。
- [ ] 公共 `enable_thrust_estimation: false`（本机架）。
- [ ] 规划竖直范围（`virtual_ground~virtual_ceil`）与飞行场景匹配。

---

*相关文档：[se3_controller.md](se3_controller.md)（控制器原理与积分项）、[ki_pz_tuning_guide.md](ki_pz_tuning_guide.md)（积分增益整定）。*
