# Fast-Planner

Fast-Planner 是一个面向复杂未知环境的四旋翼局部轨迹规划与在线重规划系统。它将在线建图、动力学路径搜索、B-spline 轨迹优化和四旋翼闭环仿真整合在同一套 ROS 1/catkin 工程中。

本 README 面向当前仓库的代码结构和已经验证的 ROS Noetic 容器环境。[上游项目](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)的原始英文说明保存在 [README_old.md](README_old.md)。

<p align="center">
  <img src="files/ral19_3.gif" alt="Kinodynamic planning demo" width="48%">
  <img src="files/icra20_3.gif" alt="Topological planning demo" width="48%">
</p>

## 功能特性

- 使用深度图或点云与位姿在线构建占据地图和 ESDF。
- 使用 kinodynamic A* 搜索满足四旋翼动力学约束的初始轨迹。
- 使用 topological PRM 搜索不同拓扑类别的候选路径。
- 使用非均匀 B-spline 表示并优化轨迹的平滑性、安全距离和动力学可行性。
- 将规划轨迹转换为 `quadrotor_msgs::PositionCommand`，接入 SO3 控制器。
- 提供随机地图、局部传感器、四旋翼动力学和 RViz 可视化组成的轻量仿真环境。

## 环境要求

上游代码支持以下环境：

- Ubuntu 18.04 + ROS Melodic
- Ubuntu 20.04 + ROS Noetic
- NLopt 2.7.1
- Armadillo
- Eigen、PCL 和 OpenCV 等 ROS 常用依赖

当前开发环境已经在 `ros_noetic` 容器中完成编译，catkin 工作空间为 `/root/catkin_ws`。宿主机使用 ROS 2 Humble，不能直接在宿主终端中运行本项目的 `roslaunch` 命令。

## 快速开始

### 使用已配置的 Docker 容器

以下命令均在宿主机执行。

首先允许容器中的 root 用户访问当前 X Server，并启动已有容器：

```bash
xhost +SI:localuser:root
docker start ros_noetic
```

打开第一个终端，启动 RViz：

```bash
docker exec -it -e DISPLAY="$DISPLAY" ros_noetic bash -lc \
  'source /opt/ros/noetic/setup.bash &&
   source /root/catkin_ws/devel/setup.bash &&
   roslaunch plan_manage rviz.launch'
```

打开第二个终端，启动 kinodynamic 规划器和完整仿真环境：

```bash
docker exec -it ros_noetic bash -lc \
  'source /opt/ros/noetic/setup.bash &&
   source /root/catkin_ws/devel/setup.bash &&
   roslaunch plan_manage kino_replan.launch'
```

`kino_replan.launch` 已经包含随机地图、局部传感器、轨迹服务器、SO3 控制器和四旋翼动力学仿真，不需要额外启动 `simulator.xml`。

### 设置飞行目标

仿真节点启动后：

1. 等待 RViz 显示随机地图和无人机模型。
2. 确认 RViz 的 Fixed Frame 为 `world`。
3. 选择工具栏中的 `2D Nav Goal`。
4. 在空闲区域点击并拖动，设置目标位置和朝向。

默认地图大小为 `40 m × 20 m × 5 m`，应将目标设置在 `x ∈ (-20, 20)`、`y ∈ (-10, 10)` 范围内，并避开地图边界和障碍物。手动目标的飞行高度由状态机固定为 `1.0 m`。

规划成功后，无人机会立即执行轨迹，并在飞行过程中持续检查碰撞和触发局部重规划。

### 切换到 topological 规划

先停止正在运行的 `kino_replan.launch`，再启动：

```bash
docker exec -it ros_noetic bash -lc \
  'source /opt/ros/noetic/setup.bash &&
   source /root/catkin_ws/devel/setup.bash &&
   roslaunch plan_manage topo_replan.launch'
```

Kino 和 Topo 模式使用相同的节点名及 ROS topic，不能同时启动。

### 停止仿真

在两个 `roslaunch` 终端中分别按 `Ctrl-C`，然后执行：

```bash
docker stop ros_noetic
xhost -SI:localuser:root
```

## 安装与编译

已经配置好上述容器时，可以跳过本节。

### 安装依赖

安装 Armadillo 和基础编译工具：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libarmadillo-dev
```

安装 NLopt 2.7.1：

```bash
git clone --branch v2.7.1 --depth 1 https://github.com/stevengj/nlopt.git
cmake -S nlopt -B nlopt/build -DCMAKE_BUILD_TYPE=Release
cmake --build nlopt/build -j"$(nproc)"
sudo cmake --install nlopt/build
sudo ldconfig
```

### 创建 catkin 工作空间

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/dreamer198/Fast-Planner.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

如使用 Melodic，请将 `/opt/ros/noetic` 替换为 `/opt/ros/melodic`。

### 在现有容器中重新编译

修改容器工作空间中的代码后执行：

```bash
docker exec -it ros_noetic bash
source /opt/ros/noetic/setup.bash
cd /root/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

当前容器运行的是 `/root/catkin_ws/src/Fast-Planner` 中的独立源码副本，对应的宿主机目录为 `/home/ziqian/docker/ros_root/catkin_ws/src/Fast-Planner`。它与当前 Git 工作区不是同一目录；在其他仓库副本中修改代码后，需要先同步到该路径并重新编译，否则仿真不会使用最新改动。

### GPU 深度渲染

GPU 深度渲染是可选功能。当前 `ENABLE_CUDA=false`，编译得到的 `pcl_render_node` 使用 CPU 生成局部点云，并通过 `/pcl_render_node/cloud` 接入 SDFMap。

如需改用深度图与相机位姿输入，请修改 [uav_simulator/local_sensing/CMakeLists.txt](uav_simulator/local_sensing/CMakeLists.txt) 中的 `ENABLE_CUDA`，根据显卡设置 CUDA 架构参数并重新编译。GPU 版本会发布 `/pcl_render_node/depth` 和 `/pcl_render_node/camera_pose`。

## 项目结构

```text
Fast-Planner/
├── fast_planner/       # 建图、搜索、轨迹表示、优化和规划管理
├── uav_simulator/      # 地图、传感器、控制器和四旋翼仿真
├── files/              # 演示素材和 BibTeX
├── README.md           # 当前中文说明
├── README_old.md       # 上游英文 README
└── LICENSE
```

核心模块如下：

| 模块 | 职责 | 主要入口 |
| --- | --- | --- |
| `plan_manage` | 状态机、规划调度、轨迹发布和 launch 配置 | `fast_planner_node.cpp`、`planner_manager.cpp` |
| `plan_env` | 概率占据地图、raycasting 和 ESDF | `sdf_map.cpp`、`edt_environment.cpp` |
| `path_searching` | kinodynamic A*、A* 和 topological PRM | `kinodynamic_astar.cpp`、`topo_prm.cpp` |
| `bspline` | 非均匀 B-spline 表示、求导和可行性检查 | `non_uniform_bspline.cpp` |
| `bspline_opt` | 平滑、避障和动力学约束优化 | `bspline_optimizer.cpp` |
| `poly_traj` | 多项式轨迹生成 | `polynomial_traj.cpp` |
| `traj_utils` | 规划结果可视化 | `planning_visualization.cpp` |
| `uav_simulator` | 随机地图、局部感知、SO3 控制和动力学仿真 | `simulator.xml` |

## 系统流程

Kinodynamic 仿真的主要规划与控制链路为：

```text
RViz 2D Nav Goal
  -> waypoint_generator
  -> KinoReplanFSM
  -> KinodynamicAstar
  -> B-spline parameterization and optimization
  -> /planning/bspline
  -> traj_server
  -> /planning/pos_cmd
  -> SO3 controller
  -> quadrotor simulator
```

地图链路为：

```text
random map
  -> /map_generator/global_cloud
  -> local sensing
  -> cloud + odometry (CPU default)
     or depth + camera pose (CUDA)
  -> SDFMap
  -> occupancy map and ESDF
  -> path search and trajectory optimization
```

## 常用 ROS Topic

| Topic | 类型/用途 |
| --- | --- |
| `/move_base_simple/goal` | RViz 发布的手动目标 |
| `/waypoint_generator/waypoints` | 规划状态机接收的目标路径 |
| `/state_ukf/odom` | 默认状态估计/仿真里程计 |
| `/pcl_render_node/cloud` | CPU 默认配置生成的局部点云 |
| `/pcl_render_node/depth` | CUDA 配置生成的深度图 |
| `/pcl_render_node/camera_pose` | CUDA 配置生成的深度相机位姿 |
| `/planning/bspline` | 规划器输出的 B-spline 轨迹 |
| `/planning/pos_cmd` | 轨迹服务器输出的位置控制命令 |
| `/sdf_map/occupancy` | 占据地图可视化 |
| `/sdf_map/esdf` | ESDF 可视化 |

仿真运行后，可以使用以下命令检查数据链路：

```bash
rostopic hz /state_ukf/odom
rostopic hz /pcl_render_node/cloud
rostopic echo -n 1 /planning/bspline
rostopic hz /planning/pos_cmd
```

以上命令应在已经加载 ROS Noetic 和 catkin 工作空间的容器终端中执行；后两项需要先在 RViz 中设置有效目标。

当前配置下，`/state_ukf/odom` 应持续以约 `200 Hz` 发布，`/pcl_render_node/cloud` 应持续输出局部点云；设置目标后，`/planning/pos_cmd` 应以约 `100 Hz` 发布。

## 配置说明

常用配置入口：

| 文件 | 主要配置 |
| --- | --- |
| `launch/kino_replan.launch` | 地图大小、目标来源、动力学上限、随机障碍数量 |
| `launch/kino_algorithm.xml` | 建图、搜索、B-spline 优化和 FSM 参数 |
| `launch/topo_replan.launch` | Topo 模式及全局航点 |
| `launch/topo_algorithm.xml` | Topological PRM 和路径引导优化 |
| `launch/simulator.xml` | 初始状态、传感器、控制器和仿真器 |
| `local_sensing/params/camera.yaml` | 深度相机分辨率和内参 |

这些文件位于 [fast_planner/plan_manage/launch](fast_planner/plan_manage/launch) 和 [uav_simulator/local_sensing/params](uav_simulator/local_sensing/params)。

Kino 模式的主要默认值：

| 参数 | 默认值 |
| --- | --- |
| 地图大小 | `40 × 20 × 5 m` |
| 地图分辨率 | `0.1 m` |
| 初始位置 | `(-19, 0, 1) m` |
| 最大速度 | `3.0 m/s` |
| 最大加速度 | `2.0 m/s²` |
| 传感器范围 | `5.0 m` |
| CUDA 相机分辨率 | `640 × 480` |
| 手动目标高度 | `1.0 m` |

`kino_algorithm.xml` 和 `topo_algorithm.xml` 依赖外层 launch 传入参数，不应直接使用 `roslaunch` 启动。

## 故障排查

### 找不到 `roslaunch` 或 `plan_manage`

确认当前终端位于 ROS 1 环境，并重新加载工作空间：

```bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
rospack find plan_manage
```

### RViz 无法打开

如果出现 `Invalid MIT-MAGIC-COOKIE-1`、`xcb` 或 `cannot connect to display`：

```bash
xhost +SI:localuser:root
docker exec -it -e DISPLAY="$DISPLAY" ros_noetic bash
```

同时确认容器挂载了 `/tmp/.X11-unix` 和有效的 Xauthority 文件。

### 节点启动后立即退出

不要重复启动 `kino_replan.launch`，也不要同时运行 Kino 和 Topo。重复节点名会导致旧节点被 ROS master 注销，并可能使 `so3_control` 退出。

### 找不到 NLopt 动态库

```bash
sudo ldconfig
ldd /root/catkin_ws/devel/lib/plan_manage/fast_planner_node | grep "not found"
```

## 推荐读码顺序

1. `fast_planner/plan_manage/launch/kino_replan.launch`
2. `fast_planner/plan_manage/launch/kino_algorithm.xml`
3. `fast_planner/plan_manage/launch/simulator.xml`
4. `fast_planner/plan_manage/src/fast_planner_node.cpp`
5. `fast_planner/plan_manage/src/kino_replan_fsm.cpp`
6. `fast_planner/plan_manage/src/planner_manager.cpp`
7. `fast_planner/plan_env/src/sdf_map.cpp`
8. `fast_planner/path_searching/src/kinodynamic_astar.cpp`
9. `fast_planner/bspline/src/non_uniform_bspline.cpp`
10. `fast_planner/bspline_opt/src/bspline_optimizer.cpp`
11. `fast_planner/plan_manage/src/traj_server.cpp`

## 论文与引用

Fast-Planner 的主要算法来自以下工作：

- Boyu Zhou, Fei Gao, Luqi Wang, Chuhao Liu and Shaojie Shen, [“Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight,”](https://ieeexplore.ieee.org/document/8758904) RA-L, 2019.
- Boyu Zhou, Fei Gao, Jie Pan and Shaojie Shen, [“Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths,”](https://arxiv.org/abs/1912.12644) ICRA, 2020.
- Boyu Zhou, Jie Pan, Fei Gao and Shaojie Shen, [“RAPTOR: Robust and Perception-aware Trajectory Replanning for Quadrotor Fast Flight,”](https://arxiv.org/abs/2007.03465) T-RO.

论文链接和 BibTeX 参见 [files/bib.txt](files/bib.txt) 及 [上游英文 README](README_old.md)。

## 致谢与许可

Fast-Planner 由 HKUST Aerial Robotics Group 和 ZJU FAST Lab 的研究人员开发，并使用 NLopt 完成非线性轨迹优化。作者、演示视频和上游项目更新记录请参见 [README_old.md](README_old.md)。

本项目依据 [GPLv3](LICENSE) 发布。该软件属于研究代码，不提供适销性或特定用途适用性的保证。
