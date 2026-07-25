# 真机运行命令流程（两套网络）

根据当前网络选择下面其中一套命令，不要混用两套 IP。

## 修改室外单航点

机载程序启动后，在 Jetson 上获取 mission 实际使用的 `world` 局部坐标：

```bash
docker exec -it diff_planner_px4_real bash -lc 'source ~/.bashrc && rostopic echo /localization/odom/pose/pose/position'
```

该命令会持续输出位置，按 `Ctrl-C` 停止。输出中的 `x`、`y`、`z` 可以写入工作站本地
`mission_outdoor_park.json` 的 `waypoints[0]`。修改完成后，在工作站执行：

```bash
cd /home/dreamer198/diff-planner-px4-deployment
rsync -av mission_outdoor_park.json jetson2@172.20.10.5:/home/jetson2/diff-planner-px4-deployment/
```

使用网络一时，将同步命令中的 `172.20.10.5` 改为 `10.0.30.108`。同步后在
Jetson 上执行：

```bash
./launch/real.sh mission mission_outdoor_park.json
```

## 网络二：172.20.10

### 工作站：连接 Jetson

```bash
ssh jetson2@172.20.10.5
```

### Jetson：启动机载程序

```bash
cd /home/jetson2/diff-planner-px4-deployment

FCU_URL='/dev/ttyACM0:921600' \
GCS_URL='udp://:14555@172.20.10.3:14550' \
ROS_IP=172.20.10.5 \
MAVROS_TGT_SYSTEM=5 \
PLANNER_RESOLUTION=0.11 \
PLANNER_OBSTACLES_INFLATION=0.33 \
./launch/real.sh start
```

### 工作站：打开远程 RViz

```bash
cd /home/dreamer198/diff-planner-px4-deployment

JETSON_IP=172.20.10.5 \
LOCAL_IP=172.20.10.3 \
START_GOAL_BRIDGE=false \
./launch/real_rviz.sh
```

### Jetson：执行飞行命令

```bash
cd /home/jetson2/diff-planner-px4-deployment

./launch/real.sh arm
./launch/real.sh goal X Y Z
./launch/real.sh goal X Y Z YAW_DEG
./launch/real.sh mission mission_indoor.json
./launch/real.sh mission mission_outdoor_park.json
./launch/real.sh land
./launch/real.sh stop3
+
```

关闭 RViz：在工作站 RViz 终端按 `Ctrl-C`。
