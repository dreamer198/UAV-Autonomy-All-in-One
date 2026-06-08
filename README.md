# diff-planner-px4-deployment

这是一个不碰现有运行链路的整理目录，目标是把 Jetson 上现在这套真机 ROS1 链路收拢成三层：

1. `docker/Dockerfile`
   - 按 Jetson 容器真实结构构建镜像
   - 保留双工作区：
     - `/root/livox_ws`
     - `/root/catkin_ws`

2. `docker/docker_run_real.sh`
   - 构建并启动整理版容器
   - 默认容器名：`ros_noetic_realflight`

3. `scripts/start_real_px4_mid360_fastlio.sh`
   - 在整理版容器里用 `tmux` 拉起整条 ROS 链路

## 这次整理的边界

- 不改你现在 Jetson 上正在使用的代码
- 不改你现在 live 的 `ros_noetic` 容器
- 只在 `/home/dreamer198/diff-planner-px4-deployment` 里维护整理版

## 当前采用的真值来源

这次整理已经按 Jetson 容器真实结构核对过，确认：

- `livox_ros_driver2` 在 `/root/livox_ws/src/livox_ros_driver2`
- `FAST_LIO` 在 `/root/catkin_ws/src/FAST_LIO`
- `Diff-Planner-PX4` 在 `/root/catkin_ws/src/Diff-Planner-PX4`
- `px4_realflight_tools` 在 `/root/catkin_ws/src/px4_realflight_tools`

## 目录说明

- `docker/`
  - 整理版镜像与容器启动脚本
- `scripts/`
  - 真机启动脚本和辅助脚本
- `local_pkgs/`
  - 从 Jetson 现状补出来的本地最小 ROS 包
- `third_party/`
  - 收集到的上游或现有源码快照
- `config/`
  - RViz / Livox 配置
- `docs/`
  - 迁移过程里的参考拷贝

## 建议用法

```bash
cd /home/dreamer198/diff-planner-px4-deployment
./docker/docker_run_real.sh build
./docker/docker_run_real.sh run
./scripts/start_real_px4_mid360_fastlio.sh start
```

这套整理版默认使用独立容器名 `ros_noetic_realflight`，不会主动去占用你当前 live 的 `ros_noetic`。
