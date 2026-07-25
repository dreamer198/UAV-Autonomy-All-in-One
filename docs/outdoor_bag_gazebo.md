# 室外 bag 全场景 Gazebo 重建

该环境使用修复后的
`se3_test_20260723_151241_0.bag` 中全部 2298 帧
`/localization/cloud_registered`，重建测试轨迹周围的静态三维表面。
网格同时用于 Gazebo 碰撞和模拟 MID360 射线返回。

它只复现环境和无人机起点，不回放原测试中的目标、模式切换、控制指令或碰撞事件。
启动后可以自由更改 Planner 参数并重新发送目标。

## 直接使用

场景首次生成或修改重建参数后：

```bash
./launch/outdoor_bag_sim.sh generate
```

生成操作只负责得到 Gazebo world。启动、解锁、目标和降落全部使用统一的
`sim.sh`，场景配置只改变 world 和无人机出生位姿：

```bash
./launch/sim.sh --scene se3_test_20260723_151241_0 restart
```

无人机默认保持未解锁。开始一次新测试：

```bash
./launch/sim.sh arm
./launch/sim.sh goal 61.65 -19.03 1.0
```

目标可以替换为任意需要测试的 `world` 坐标。停止环境：

```bash
./launch/sim.sh stop
```

其他入口：

```bash
./launch/sim.sh status
./launch/sim.sh attach
./launch/sim.sh shell
./launch/sim.sh land
```

`outdoor_bag_sim.sh` 的非生成操作仅保留为旧命令兼容入口，内部同样转发到
`sim.sh`，不包含另一套飞行流程。

## 场景与起点

- 重建范围：约 `x=-7.14～65.66 m`、`y=-21.98～7.14 m`、
  `z=0.14～3.22 m`；
- bag 初始位置：`(-0.121, -0.036, 0.008) m`；
- Gazebo 生成位置：`x=-0.121, y=-0.036, z=0`，yaw 约 `0°`；
- 默认无风，避免把 bag 中没有测量的外部扰动编造成已知量；
- 控制器、Planner、MID360、点云处理和解锁流程均沿用默认仿真配置。

生成文件位于：

```text
runtime/simulation/reconstructed/se3_test_20260723_151241_0/
├── se3_outdoor_reconstruction.world
├── meshes/scene.obj
├── metadata.json
└── recorded_trajectory.csv
```

对应的场景配置是
`simulation/config/scenes/se3_test_20260723_151241_0.env`。以后增加其他 world，
只需复制一个场景配置并修改 `SCENE_WORLD` 和 `SCENE_SPAWN_*`，无需新建启动脚本。

## 调整重建精度

默认使用 `0.14 m` 体素、每两帧处理一帧，只保留至少重复观测两次的体素，
并截取原轨迹周围 `7 m`：

```bash
OUTDOOR_SIM_VOXEL_SIZE=0.11 \
OUTDOOR_SIM_CLOUD_STRIDE=1 \
OUTDOOR_SIM_MIN_OBSERVATIONS=2 \
OUTDOOR_SIM_CORRIDOR_RADIUS=7.0 \
./launch/outdoor_bag_sim.sh generate
```

更小体素和更密采样会增加 OBJ 面数及 Gazebo 射线计算量。若需要做横风敏感性测试，
可显式生成有风版本：

```bash
OUTDOOR_SIM_WIND_SPEED=1.5 \
OUTDOOR_SIM_WIND_DIRECTION_X=-1 \
OUTDOOR_SIM_WIND_DIRECTION_Y=0 \
./launch/outdoor_bag_sim.sh generate
```

## 能力边界

重建只能包含真实 MID360 在原轨迹上看到的表面；被遮挡区域、材质、风速和真实机体
质量/电机参数无法从 bag 唯一恢复。当前动力学仍是 PX4 SITL Iris。因此该环境适合
复测地图构建、路径选择和碰撞趋势，但不能把某次仿真轨迹与实飞轨迹的厘米级差异
视为等价。
