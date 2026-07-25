# 室外 bag 全场景 Gazebo 重建

该环境使用修复后的
`se3_test_20260723_151241_0.bag` 中全部 2298 帧
`/localization/cloud_registered`。重建器先保留重复观测，再把三维体素投影为
水平密度图；平滑后的局部密度峰值视为树木，生成圆柱树干和低多边形树冠。
稀疏离群回波不会再变成悬浮小方块。实测覆盖之外使用确定性最远点采样补齐
起点到目标点围成的矩形树林，补齐树与实测树使用不同的可复现尺寸变化。
网格同时用于 Gazebo 碰撞和模拟 MID360 射线返回。

它只复现环境和无人机起点，不回放原测试中的目标、模式切换、控制指令或碰撞事件。
启动后可以自由更改 Planner 参数并重新发送目标。

## 直接使用

场景首次生成或修改重建参数后：

```bash
./launch/outdoor_bag_sim.sh generate
```

生成操作会把轻量场景发布到仓库内的版本化目录。原始 bag 仍留在
`runtime/`，不会进入 Git。启动、解锁、目标和降落全部使用统一的 `sim.sh`，
场景配置只改变 world 和无人机出生位姿：

```bash
./launch/sim.sh --scene outdoor_rectangular_forest restart
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

- 矩形范围：`x=-0.121～61.65 m`、`y=-19.03～-0.036 m`；
- 左上角是 bag 起点 `(-0.121, -0.036)`，右下角是默认目标
  `(61.65, -19.03)`，二者互为矩形对角点；
- 默认生成约 190 棵树，其中约 91 棵来自实测密度峰值，其余用于补齐矩形空白；
- 树高约 `2.9～4.0 m`、树干半径约 `0.13～0.27 m`、树冠半径约
  `0.47～1.01 m`，具体数值由固定随机种子和局部密度决定；
- bag 初始位置：`(-0.121, -0.036, 0.008) m`；
- Gazebo 生成位置：`x=-0.121, y=-0.036, z=0`，yaw 约 `0°`；
- 默认无风，避免把 bag 中没有测量的外部扰动编造成已知量；
- 控制器、Planner、MID360、点云处理和解锁流程均沿用默认仿真配置。

可直接提交的生成文件位于：

```text
simulation/config/scenes/outdoor_rectangular_forest/
├── se3_outdoor_reconstruction.world
├── meshes/scene.obj
├── meshes/scene.mtl
├── metadata.json
├── trees.json
└── recorded_trajectory.csv
```

对应的场景配置是
`simulation/config/scenes/outdoor_rectangular_forest.env`。以后增加其他 world，
只需复制一个场景配置并修改 `SCENE_WORLD` 和 `SCENE_SPAWN_*`，无需新建启动脚本。

## 调整重建精度

默认使用 `0.14 m` 体素、每两帧处理一帧，只保留至少重复观测两次的体素，
并截取原轨迹周围 `7 m`。密度图网格为 `0.28 m`，平滑半径 `0.40 m`，
树木最小间距 `1.20 m`，只保留密度最高的局部峰值。矩形空白区域继续补树，
直到任一点到最近树木不超过约 `2.0 m`；轨迹附近保留 `0.65 m` 树干通道，
起点和目标角点保留 `1.5 m` 净空：

```bash
OUTDOOR_SIM_VOXEL_SIZE=0.11 \
OUTDOOR_SIM_CLOUD_STRIDE=1 \
OUTDOOR_SIM_MIN_OBSERVATIONS=2 \
OUTDOOR_SIM_CORRIDOR_RADIUS=7.0 \
OUTDOOR_SIM_TREE_MIN_SPACING=1.20 \
OUTDOOR_SIM_TREE_DENSITY_QUANTILE=0.80 \
OUTDOOR_SIM_TREE_MAX_HEIGHT=4.20 \
OUTDOOR_SIM_FOREST_FILL_SPACING=2.0 \
OUTDOOR_SIM_FOREST_PATH_CLEARANCE=0.65 \
./launch/outdoor_bag_sim.sh generate
```

减小 `FOREST_FILL_SPACING` 会提高补齐密度；修改 `FOREST_SEED` 可得到另一组
可复现的补树位置和尺寸。提高密度分位数会减少实测树木，增大最小间距会合并
相邻峰值。诊断原始点云时可临时设
`OUTDOOR_SIM_GEOMETRY_MODE=voxels`，但不建议把该模式生成的零散体素场景用于飞行
测试。若需要做横风敏感性测试，
可显式生成有风版本：

```bash
OUTDOOR_SIM_WIND_SPEED=1.5 \
OUTDOOR_SIM_WIND_DIRECTION_X=-1 \
OUTDOOR_SIM_WIND_DIRECTION_Y=0 \
./launch/outdoor_bag_sim.sh generate
```

## 能力边界

实测树的位置来自 MID360 密度峰值；矩形补齐树只是满足场景边界和树林连续性的
程序生成估计，不代表传感器实际观测。被遮挡区域、材质、风速和真实机体质量/电机
参数也无法从 bag 唯一恢复。当前动力学仍是 PX4 SITL Iris。因此该环境适合复测地图
构建、路径选择和碰撞趋势，但不能把某次仿真轨迹与实飞轨迹的厘米级差异视为等价。
