# 室外重建场景

`outdoor_rectangular_forest` 是由室外飞行 bag 的注册点云生成并提交到仓库的 Gazebo
树林场景。它复现环境几何和无人机起点，不回放原测试中的目标、模式切换、控制指令或
碰撞事件。

场景的精确来源、生成参数和统计信息记录在
[`metadata.json`](../simulation/config/scenes/outdoor_rectangular_forest/metadata.json)，
无需在本文重复维护。

## 直接启动

已提交的场景可以直接使用，不需要先运行生成器：

```bash
./launch/sim.sh --scene outdoor_rectangular_forest restart
```

先用附近目标确认定位、点云和控制链路：

```bash
SIM_TAKEOFF_HEIGHT=1.0 ./launch/sim.sh arm
./launch/sim.sh goal 2.0 0.0 1.0
./launch/sim.sh land
```

场地航点回归可使用：

```bash
./launch/sim.sh mission mission_outdoor_park.json
```

Mission 文件是当前场景的测试路线，不代表 Planner 已验证整条路线全局可达。Planner
会按自身地图能力逐点验证：Diff 使用滚动局部地图；Fast Kino/Topo 使用统一的
`30 × 30 m` 固定水平范围，因此会在起飞前拒绝这两个百米级 outdoor Mission 中的
越界航点。较远目标或被障碍阻断的航段仍可能重规划失败并紧急停止。

场景入口只改变 world 和出生位姿。Planner、控制器、MID-360、录包和飞行命令仍使用
公共仿真配置。

## 重新生成

只有更换源 bag 或调整重建参数时才需要生成。源 bag 不进入 Git，应先放入：

```text
runtime/simulation/flight_bags/
```

然后执行：

```bash
OUTDOOR_SIM_BAG_NAME=source.bag \
./launch/outdoor_bag_sim.sh generate
```

生成器读取 `/localization/odom` 和 `/localization/cloud_registered`，在
`runtime/simulation/reconstructed/` 生成中间结果，再把可提交的轻量场景发布到：

```text
simulation/config/scenes/outdoor_rectangular_forest/
├── se3_outdoor_reconstruction.world
├── meshes/
├── metadata.json
├── trees.json
└── recorded_trajectory.csv
```

常用调整项：

| 变量 | 作用 |
|---|---|
| `OUTDOOR_SIM_VOXEL_SIZE` | 点云体素尺寸 |
| `OUTDOOR_SIM_CLOUD_STRIDE` | 处理帧间隔 |
| `OUTDOOR_SIM_MIN_OBSERVATIONS` | 体素最少重复观测次数 |
| `OUTDOOR_SIM_CORRIDOR_RADIUS` | 保留原轨迹周围的点云范围 |
| `OUTDOOR_SIM_TREE_MIN_SPACING` | 检测树木的最小间距 |
| `OUTDOOR_SIM_TREE_DENSITY_QUANTILE` | 树木密度峰值阈值 |
| `OUTDOOR_SIM_FOREST_FILL_SPACING` | 未观测区域的补树间距 |
| `OUTDOOR_SIM_FOREST_PATH_CLEARANCE` | 原轨迹附近的树干净空 |
| `OUTDOOR_SIM_FOREST_CORNER_CLEARANCE` | 起点和目标角点净空 |
| `OUTDOOR_SIM_WIND_SPEED`、`OUTDOOR_SIM_WIND_DIRECTION_*` | 可选风场 |

完整参数和当前默认值以
[`launch/outdoor_bag_sim.sh`](../launch/outdoor_bag_sim.sh) 为准。
`OUTDOOR_SIM_GEOMETRY_MODE=voxels` 可用于检查原始点云，但零散体素场景不适合作为常规
飞行测试环境。

## 场景配置

入口配置位于
[`outdoor_rectangular_forest.env`](../simulation/config/scenes/outdoor_rectangular_forest.env)。
其中 `SCENE_WORLD` 是容器内路径。新增场景时，应把 world 和依赖资源放在
`simulation/config/scenes/` 下，并使用
`/etc/sim2real/simulation/scenes/...` 路径引用；宿主机绝对路径在仿真容器中不可见。

## 能力边界

- 实测树木来自点云密度峰值，不是逐棵测量的真实模型；
- 未观测区域的补树是确定性生成结果，不代表传感器实际观测；
- 材质、遮挡区域、风和真实机体动力学不能从 bag 唯一恢复；
- 当前飞行器仍是 PX4 SITL Iris，仿真轨迹不能与实飞轨迹做厘米级等价比较；
- 该场景适合复测建图、局部路径选择和碰撞趋势，不提供全局路线安全保证。
