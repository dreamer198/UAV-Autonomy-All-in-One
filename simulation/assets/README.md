# 仿真镜像资产

本目录保存由 [`simulation/Dockerfile`](../Dockerfile) 复制进仿真镜像的 PX4 和
Gazebo 覆盖文件。PX4 与 MID-360 的上游版本只在
[`simulation/versions.env`](../versions.env) 中维护。

仓库仅保存自定义 launch、world、模型定义和 MAVROS 配置。MID-360 网格与扫描模式
CSV 在构建镜像时从固定版本的上游仓库取得。

`px4/models/Mid360/Mid360.sdf` 保留上游扫描模式，同时关闭 Gazebo 射线渲染以降低
`gzclient` 负载；ROS 点云仍然发布。

修改本目录后需要重建镜像并重新创建容器：

```bash
./launch/sim.sh stop
./launch/sim_container.sh build
./launch/sim_container.sh recreate
```
