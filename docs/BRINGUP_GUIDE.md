# G1-D 上机部署教程与适配清单

本文是把 `g1d_ros2_ws` 装到导航主机、接上 G1-D、逐步放开自主导航的**操作手册**：每一步给出命令、通过标准和要记录的数据。最后的第 10 节是**需要适配的点**总表，上线前逐项打勾。

相关文档：
- 原理：[`principles.html`](principles.html)（用浏览器打开）；
- 参数与故障排查速查：[`DEPLOYMENT.md`](DEPLOYMENT.md)；
- 标定细节：[`CALIBRATION.md`](CALIBRATION.md)。

> 仿真中验证过的内容见 `PROJECT_OVERVIEW.md` 第 9 节。真机上一定会有仿真里没有的差异，所以严格按阶段推进：**上一阶段的通过标准没达到，不进入下一阶段**。

---

## 0. 安全守则（每次上机前读一遍）

1. 前两个阶段（P1、P2）**限速 0.3 m/s**，侧后方有人值守，**硬件急停按钮在手边**。
2. 第一次让机器人自己动之前，必须先通过 P0.5 的放行门槛：停发指令后，底盘自己在 0.6 s 内停住。
3. 任何时候都可以这样让机器人停下：
   - 推遥控器摇杆：守护 5 s 内不写底盘，导航中止；
   - Web 急停按钮，或执行 `ros2 run g1d_tools nav_cli estop on`（锁存，需要手动 `estop off` 解除）；
   - 硬件急停按钮。
4. 导航期间不要动手臂、腰、升降：姿态锁定会中止导航，但点云在这之前可能已经被干扰。
5. 场地里不要有玻璃墙、镜面、低于 8 cm 的障碍（雷达和代价地图都看不准），或者先用纸箱挡住。

---

## 1. 准备清单

| 类别 | 项目 | 说明 |
|---|---|---|
| 主机 | x86_64，Ubuntu 24.04，≥ 8 核 | 导航只用 CPU；开发机 RTX 5090 可直接用 |
| 网络 | 千兆网线、USB 网卡（可选） | 自主导航**必须用网线**接机器人内网 `192.168.123.0/24`；WiFi 只用于监控和低速遥控 |
| 机器人 | G1-D 电量 > 50%，宇树服务正常 | PC2（`192.168.123.164`）**不安装任何东西** |
| 场地 | ≥ 3 m 空地（P0.5）；平整地面 + 墙 / 箱子（标定）；走廊和桌子（验收） | 地面贴 2 个标记点（T3 / T7 用） |
| 工具 | 卷尺、纸箱若干、一张桌子（桌面高于底盘、低于肩部） | 桌子用于 T6 悬空障碍验收 |
| 人员 | 操作员 1 人 + 安全员 1 人 | 安全员只负责急停 |

---

## 2. 主机安装

```bash
git clone --recursive https://github.com/hitliujianyun/g1d_ros2_ws.git ~/g1d_ros2_ws
cd ~/g1d_ros2_ws
sudo scripts/ci/install_deps.sh          # ROS 2 Jazzy 依赖、Livox-SDK2（装到 /usr/local）、rosdep
sudo scripts/install_host_extras.sh      # RViz2、foxglove_bridge、MCAP 录包插件
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to-regex '^g1d_' --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 src/g1d_description/scripts/fetch_meshes.py    # 机器人模型网格（只用于显示，可选）
```

没有 ROS 2 Jazzy 时先按官方文档装 `ros-jazzy-ros-base`，`install_deps.sh` 不负责安装 ROS 本身。

**为什么用 `--symlink-install`**：后面的适配步骤要反复修改 `src/` 下的 YAML（守护、身体、自身点盒子、外参、Nav2）。用了这个选项，安装目录里的配置是指向源码的链接，改完**重启对应 launch 即可生效**。没用的话，每次修改都要 `colcon build --packages-select <包名>`。下文提到"改完重启"时都按前一种情况描述。

**验证安装**（不连机器人也能做）：

```bash
ros2 launch g1d_bringup sim.launch.py lio:=true nav:=true     # 仿真：场景 room
# 另一个终端
ros2 run g1d_tools nav_cli go 2.0 0.0                          # 仿真机器人前进 2 m
```

`nav_cli` 打印 `SUCCEEDED` 说明整条链路安装正确。之后 Ctrl-C 结束。

---

## 3. 网络与 DDS

### 3.1 静态地址

导航主机接机器人内网的网卡需要固定地址。示例用 `192.168.123.222`，避开机器人已用的地址：

```bash
ip -br addr                       # 记下网卡名，例如 enp3s0
sudo nmcli con add type ethernet ifname enp3s0 con-name g1d ipv4.method manual \
     ipv4.addresses 192.168.123.222/24
sudo nmcli con up g1d
ping -c 3 192.168.123.164         # PC2
ping -c 3 192.168.123.120         # Mid-360S（地址待第 5.1 节确认）
```

### 3.2 环境变量（建议写进 `~/.bashrc`）

```bash
source /opt/ros/jazzy/setup.bash
source ~/g1d_ros2_ws/install/setup.bash
export G1D_NET_IF=enp3s0            # 接机器人内网的网卡
export G1D_LINK=wired               # wifi：改为单播发现 PC2（G1D_PEER 默认 192.168.123.164）
source ~/g1d_ros2_ws/install/share/g1d_bringup/scripts/g1d_env.sh
```

正确时会打印 `g1d_env: RMW=cyclonedds domain=0 if=enp3s0 link=wired`。

### 3.3 验证 DDS 互通

```bash
ros2 topic list | grep -E 'lowstate|hispeed|slamware|wirelesscontroller|cmd_vel_no_limit'
ros2 topic hz /lowstate --qos-reliability best_effort
```

能看到这些话题、`hz` 有频率才算通。看不到时依次检查四项：
1. `G1D_NET_IF` 是否是接机器人那块网卡；
2. 防火墙是否放行 UDP 7400–7600（`sudo ufw disable` 先排除）；
3. 是否设置了 `ROS_DOMAIN_ID=0`；
4. 是否用了 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。

---

## 4. P0：接口确认（不动车）

**目的**：确认代码对宇树接口的假设与真机一致。代码里所有这类假设都列在第 10 节 A 组，这一步就是逐条核对。

```bash
ros2 run g1d_tools check_interfaces --duration 10 --out logs/interfaces.json
```

终端每行一个话题：`OK` 表示收到，`--` 表示没收到，后面是频率和时间戳偏差。逐项核对：

| 话题 / 项目 | 期望 | 不符合时 |
|---|---|---|
| `/slamware_ros_sdk_server_node/odom`、`/scan` | 有数据；`stamp_offset_s` 约 +181 s（正常，会被重打） | 名称不同 → 改 `robot_bridge.yaml` 的 `odom_in` / `scan_in` |
| `/hispeed_state` | Point32，`height_y` 随升降变化 | 类型或字段不同 → 改 `body_state.py`（A3） |
| `/lowstate` | `unitree_hg/LowState`，`waist_yaw_q12` 与腰偏航一致 | 序号不同 → 第 6.2 节 |
| `/waistpitchstate` | MotorState，`q` 随腰俯仰变化 | 老款没有 → `require_waist_pitch: false` |
| `/wirelesscontroller` | 推摇杆时 `lx/ly/rx/ry` 变化 | 没有 → 守护无法识别遥控接管，**必须解决后才能自主导航** |
| `/utlidar/cloud_livox_mid360` | 有数据说明宇树服务占用了 Mid-360S | 决定第 5.1 节用哪种接入 |
| `publishers on /cmd_vel_no_limit` | `none` | 有其他写入者 → 找出并停掉（守护会拒绝启动） |
| `chassis HTTP` | `ok: True` | 地址不对 → 改 `chassis_http`；看 `last` 里事件的字段名（A6） |

**记录**：保存 `logs/interfaces.json`，第 10 节 A 组照此填写。

另外用 `ros2 topic info -v /cmd_vel_no_limit` 确认话题类型是 `geometry_msgs/msg/Twist`（A1）。

---

## 5. P0.5：底盘指令行为（会动车，3 m 空地）

**这一步必须先停掉守护**：工具直接写 `/cmd_vel_no_limit`，发现其他写入者会拒绝运行。此时也不要启动 `robot_interface`。

```bash
ros2 run g1d_tools cmd_vel_probe --tests silence,zero,clamp,hold --i-know-it-moves
```

每个测试开始前按回车确认，机器人会前进约 0.4 m 再停：

| 测试 | 测什么 | 用途 |
|---|---|---|
| `silence` | 以 0.2 m/s 行驶后突然停发，底盘多久停住 | **放行门槛**：`release_gate.pass` 必须为 `true`（≤ 0.6 s）；给出 `zero_hold` 建议值 |
| `zero` | 发零速时底盘是否立即停 | 守护停车依赖它 |
| `clamp` | 发超大速度时底盘是否自己限幅 | 了解底盘保护 |
| `hold` | 持续发零速时是否锁轮 | 决定 `zero_hold` 是否需要更长 |

**通过标准**：`release_gate.pass = true`。不满足就**不要进入自主导航**：守护跑在导航主机上，主机或网络断了只能靠底盘自身超时。备选方案是在 PC2 上加一个极简看门狗（设计文档 6.2 节），这会违反"不改 PC2"的原则，需要单独评估。

**写回**：把 `suggested_guard.zero_hold` 写入 `src/g1d_guard/config/guard.yaml`。

---

## 6. 机器人描述与身体适配

### 6.1 启动机器人接口

```bash
ros2 launch g1d_bringup robot_interface.launch.py require_cloud:=false
```

`require_cloud:=false` 让守护暂时只用底盘雷达（Mid-360S 还没接）。检查四项：
1. `ros2 topic echo /g1d/guard_state --once` 中 `blocked` 为空，`estop` 为 false；
2. `/joint_states` 有数据；
3. RViz（`ros2 launch g1d_bringup viz.launch.py`）中机器人模型的姿态与实物一致；
4. `/chassis/scan` 的点落在真实墙面上。

### 6.2 关节序号与方向（A4、B5）

逐个轻微转动关节（腰偏航、左右臂各关节，可用宇树 App），对照两处：
- `ros2 topic echo /lowstate --field motor_state --qos-reliability best_effort` 中变化的序号；
- RViz 模型对应关节的转动方向。

序号不同 → 改 `robot_bridge.yaml` 的 `waist_yaw_index` / `arm_first_index`；方向相反 → 记下来，需要在 `g1d_core/body.py` 的 `joint_positions` 里取反。

### 6.3 升降比例与零点（B4）

1. 升降到最低，`ros2 topic echo /hispeed_state --once` 记下 `y0`；卷尺量头部某固定点离地高度 `h0`；
2. 升降到最高，记下 `y1`、`h1`；
3. `lift_scale = (h1 − h0) / (y1 − y0)`，`lift_offset = −lift_scale · y0`，写入 `robot_bridge.yaml`；
4. 重启后在两个高度下对比 RViz 模型高度与实测，误差 < 1 cm。

完成雷达标定后可以改用 `calib_extrinsics` 读雷达高度，更准确（`CALIBRATION.md` 第 2 节）。

### 6.4 底盘高度与底盘雷达（B2、B3）

- `base_to_agv_z`：卷尺量地面到 `AGV_link` 原点（底盘上表面参考点），写入 `src/g1d_description/config/extrinsics.yaml`；
- 底盘雷达：机器人正对墙 1–2 m，RViz 固定坐标系设为 `base_link`，`/chassis/scan` 的墙线应与 y 轴平行、距离与卷尺一致（±2 cm）。有偏差就在 `g1d.urdf.xacro` 的 `chassis_laser_joint` 补偿（`CALIBRATION.md` 第 6 节）。

### 6.5 导航姿态（B6）

摆好导航时的腰部和手臂姿态（手臂收拢、不遮挡雷达），然后：

```bash
ros2 topic echo /joint_states --once
```

把 `Yaw_Joint`（实际是腰俯仰）写入 `nav_waist_pitch`，14 个手臂关节按 `left_shoulder_pitch … right_wrist_yaw` 的顺序写入 `arm_tuck`，重启后 `ros2 topic echo /g1d/body_lock --once` 应为 `locked: true`。

---

## 7. P1：只用守护（底盘雷达）

```bash
ros2 launch g1d_bringup robot_interface.launch.py require_cloud:=false
ros2 launch g1d_bringup web.launch.py          # 浏览器打开 http://<主机IP>:8080
```

验收（`acceptance` 工具或 Web「验收」页）：

| 测试 | 做法 | 通过标准 |
|---|---|---|
| T1 停车距离 | 正对墙 / 桌腿 / 人腿 2–3 m，以 0.3 m/s 遥控前进直到守护拦停，10 次 | 每次停车后前方净距 ≥ 0.40 m |
| T2 指令中断 | 行驶 2 s 后停发，3 次 | 停车时间 ≤ 0.6 s |

```bash
ros2 run g1d_tools acceptance T1
ros2 run g1d_tools acceptance T2
```

**可调参数**：停车距离不够 → 加大 `guard.yaml` 的 `stop_m`；减速太早 → 减小 `slow_m`。**不要加大 `brake_v`**（> 1.5 m/s² 有前倾风险，见原理文档 11.4）。参数可在 Web「参数」页热更新，并写回 `guard.yaml`。

---

## 8. P2：雷达与 LIO

### 8.1 选择接入方式（A5）

| 情况 | 启动参数 |
|---|---|
| Mid-360S 没被宇树服务占用（P0 中没有 `/utlidar/*`），主机能 ping 通雷达 | `lidar_source:=livox host_ip:=192.168.123.222 lidar_ip:=<雷达IP>` |
| 被宇树服务占用（有 `/utlidar/*`） | `lidar_source:=unitree` |

雷达 IP 可以用 Livox Viewer 2 查，或抓包看 Livox 端口（56000–56500）上的源地址：`sudo tcpdump -i $G1D_NET_IF -n udp portrange 56000-56500 -c 10`。

`unitree` 方式下，`utlidar_adapter` 按字段名（`time` / `timestamp` / `t` / `offset_time`）识别每个点的时间。P0 报告里 `utlidar_cloud.sample.fields` 显示了实际字段，名称不在列表里时用参数 `time_field` 指定（A7）。

### 8.2 自身点过滤（B7）

```bash
ros2 launch g1d_bringup lio.launch.py lidar_source:=livox host_ip:=192.168.123.222
```

机器人静止、导航姿态，RViz 同时显示 `/livox/lidar_raw`（过滤前）和 `/g1d/obstacle_cloud`（过滤后）。调整 `src/g1d_bringup/config/self_filter.yaml` 的三个盒子，直到满足两个条件：
- 车身 0.5 m 内没有稳定点簇；
- 前方地面 0.3 m 处放一个纸箱仍能看到。

改完重启 `lio.launch.py`。

### 8.3 外参标定（B1）

按 `CALIBRATION.md` 第 1 节：平整地面、导航姿态、前方 ≥ 2 m 空地、周围 1–4 m 有墙。

```bash
ros2 run g1d_tools calib_extrinsics --yaw-from-scan --write src/g1d_description/config/extrinsics.yaml
```

然后重启 `robot_interface`（它加载外参生成 URDF）和 `lio`。注意终端打印的 "lidar height … error"：大于 2 cm 时核对 `base_to_agv_z` 和 `lift_offset`。

### 8.4 LIO 验证

1. 启动 `lio.launch.py` 时**机器人静止 2 s**（IMU 初始化）；
2. `ros2 topic echo /g1d/lio_health --once` 应为 `ok: true`；
3. 改回 `require_cloud:=true`（默认）重启 `robot_interface`，守护同时使用两种雷达；
4. 验收：

| 测试 | 做法 | 通过标准 |
|---|---|---|
| T3 矩形闭环 | 从地面标记出发，遥控绕约 10 m 矩形回到标记并摆正 | 位置误差 < 0.10 m、航向 < 2° |
| T4 原地旋转 | 自动 12 次 +90°（3 圈） | 平移漂移 < 0.05 m |

T4 不过（转圈时位置画圆）几乎一定是外参问题，回到 8.3。

---

## 9. P3–P5：建图、定位导航、NavGPT

### 9.1 P3 建图

建图时**关闭 FAST_LIO 自动重启**：重启后 LIO 坐标系会变，位姿图会被破坏。

```bash
ros2 launch g1d_bringup robot_interface.launch.py
ros2 launch g1d_bringup lio.launch.py lidar_source:=livox host_ip:=192.168.123.222 respawn:=false
ros2 launch g1d_bringup mapping.launch.py maps_dir:=~/g1d_maps
ros2 launch g1d_bringup web.launch.py
```

建图要领：
- 低速（≤ 0.3 m/s）遥控绕场地；
- 转弯慢；
- 经过同一地点时从不同方向再走一次；
- **最后回到起点**形成闭环。

保存地图：

```bash
ros2 service call /g1d/save_map g1d_msgs/srv/SaveMap "{name: office}"
```

保存后检查三项：
1. `~/g1d_maps/office/map.pgm` 里桌子、货架应为黑色（占据）；
2. 通道应为白色；
3. 墙线没有重影（有重影说明闭环没成功，重新建图）。

重复建图精度（可选）：从同一标记出发再建一次 `office2`，然后比较：

```bash
ros2 run g1d_tools map_compare ~/g1d_maps/office ~/g1d_maps/office2
```

### 9.2 P4 定位与导航

```bash
ros2 launch g1d_bringup robot_interface.launch.py
ros2 launch g1d_bringup lio.launch.py lidar_source:=livox host_ip:=192.168.123.222
ros2 launch g1d_bringup localization.launch.py mode:=map map:=~/g1d_maps/office init:=auto
ros2 launch g1d_bringup navigation.launch.py global_frame:=map
ros2 launch g1d_bringup web.launch.py
```

`init` 有三种取值：
- `auto`：在建图轨迹附近做全局搜索；
- `last`：使用上次关机位置；
- `none`：在 RViz 用 2D Pose Estimate 或在 Web 上手动给初值。

等 `/g1d/localization_status` 的 `localized: true` 后再下发目标：Web 地图上点选，或执行 `ros2 run g1d_tools nav_cli go 2.0 0.0 --frame map`。

验收：

| 测试 | 做法 | 通过标准 |
|---|---|---|
| T5 走廊绕障 | 走廊中间放纸箱、椅子，≥ 10 个穿过障碍的目标 | 成功率 ≥ 9/10 |
| T6 桌沿 | 目标设在桌子另一侧，≥ 10 次 | 10/10 无碰撞完成，或安全拒绝（`no_path` / `blocked` / `goal_in_obstacle`） |
| T7 重定位重复精度 | 多次回到地面标记并摆正（中间可重启定位或调 `/g1d/relocalize`），≥ 5 次 | map 系位姿离散 < 0.05 m / 1° |

没有地图时可以用纯里程计模式：`localization.launch.py mode:=odom` + `navigation.launch.py global_frame:=odom`。

### 9.3 P5 NavGPT 接入

```python
from g1d_nav_manager.client import NavClient, prompt_feedback
nav = NavClient(node)                                      # node: rclpy 节点
gid = nav.send_relative(dx, dy, dyaw, stamp=image_stamp)   # 图像时刻（主机时钟）
r = nav.wait(gid)
prompt += prompt_feedback(r)                               # 失败原因 + 可通行方向
```

图像时间戳必须是**导航主机时钟**。相机接在别的计算机上时，要么在主机上按接收时间打戳，要么先做时钟同步（A8）。旧版 JSON 接口仍可用：`/nav/goal` 和 `/nav/status`。

### 9.4 控制器选择

默认是 RPP。想试 MPPI 时启动导航加上 `controller:=mppi`，并保持 `config/nav2_mppi.yaml` 中的 `regenerate_noises: true`。两者都要重新跑 T5 / T6。调参不必改仓库文件：

```bash
ros2 launch g1d_bringup navigation.launch.py global_frame:=map overlay:=~/my_nav_tuning.yaml
```

`my_nav_tuning.yaml` 只写要改的键，会深度合并到默认参数上。

---

## 10. 需要适配的点（总表）

状态一栏上机时填写：✅ 已确认，✏️ 已修改（写明新值），❌ 有问题。

### A. 接口假设（代码按这些假设编写，必须在真机上核实）

| # | 假设 | 代码位置 | 怎么核实 | 不符时改哪里 | 状态 |
|---|---|---|---|---|---|
| A1 | 底盘控制话题 `/cmd_vel_no_limit`，类型 `geometry_msgs/Twist`，`linear.x` m/s、`angular.z` rad/s | `g1d_guard/guard_node.py` | `ros2 topic info -v`；P0.5 速度与里程计一致 | 守护发布类型 / 单位换算 | |
| A2 | SLAMWARE 里程计 / 雷达话题为 `/slamware_ros_sdk_server_node/odom`、`/scan`，标准 Odometry / LaserScan，odom 带 twist | `robot_bridge.yaml`（`odom_in`、`scan_in`） | P0 报告 | `robot_bridge.yaml` | |
| A3 | `/hispeed_state` 是 Point32，y = 升降高度（m），z = 升降速度 | `g1d_robot_bridge/body_state.py` | P0 报告，升降时观察 | `body_state.py` | |
| A4 | `/lowstate` 是 `unitree_hg/LowState`，序号 12 = 腰偏航，15–28 = 左臂 7 + 右臂 7；`/waistpitchstate` 是 `unitree_hg/MotorState` | `robot_bridge.yaml`、`g1d_core/body.py` | 第 6.2 节 | `waist_yaw_index`、`arm_first_index`、`require_waist_pitch` | |
| A5 | Mid-360S IP `192.168.123.120`；未被宇树服务占用时可直连 | `lio.launch.py` 参数 `lidar_ip` | ping、Livox Viewer、tcpdump | 启动参数 `lidar_ip` / `lidar_source` | |
| A6 | 底盘 HTTP `http://192.168.123.163:1448`，事件接口 `/api/platform/v1/events`，事件字段 `type`（`BUMPER_TRIGGERED` / `CLIFF_DETECTED`）与 `timestamp` | `g1d_core/chassis_http.py`、`g1d_core/guard.py` | P0 报告 `chassis_http.last`；轻碰保险杠看事件 | `chassis_http` 参数；字段名改 `chassis_http.py` / `ESTOP_EVENTS` | |
| A7 | `/utlidar/cloud_livox_mid360` 的点时间字段为 `time` / `timestamp` / `t` / `offset_time` 之一 | `g1d_lidar/src/utlidar_adapter_node.cpp` | P0 报告 `fields` | 参数 `time_field`、`time_unit` | |
| A8 | 所有时间戳都是导航主机时钟（Mid-360S 未做 PTP；宇树话题按接收时刻） | 全局 | `check_interfaces` 的 `stamp_offset_s`：除 SLAMWARE 外应接近 0 | 外部相机图像需在主机打戳 | |
| A9 | `/wirelesscontroller` 是 `unitree_go/WirelessController`，摇杆 `lx/ly/rx/ry` 范围 −1..1 | `g1d_guard/guard_node.py` | P0 报告，推摇杆 | `manual_deadzone` 或消息字段 | |
| A10 | 宇树遥控器、App 不会在导航期间写 `/cmd_vel_no_limit` | 守护启动检查 | 守护运行时用宇树 App 试推 | 运行导航时不要用 App 遥控 | |

### B. 实测 / 标定量

| # | 参数 | 文件 | 当前值 | 怎么得到 | 状态 |
|---|---|---|---|---|---|
| B1 | `head_to_livox_xyz / rpy` | `g1d_description/config/extrinsics.yaml` | `0.05 0 0.55` / `π −0.04 0` | 8.3 `calib_extrinsics` | |
| B2 | `base_to_agv_z` | 同上 | 0.10 m | 卷尺（6.4） | |
| B3 | 底盘雷达位姿 `chassis_laser_joint` | `g1d_description/urdf/g1d.urdf.xacro` | 单位变换（= 上游 `Lidar_Link`） | 6.4 墙面核对 | |
| B4 | `lift_scale / lift_offset` | `g1d_robot_bridge/config/robot_bridge.yaml` | 1 / 0 | 6.3 | |
| B5 | 关节方向 | `g1d_core/body.py` | 与 URDF 相同 | 6.2 | |
| B6 | `nav_waist_pitch`、`arm_tuck` | `robot_bridge.yaml` | 全 0 | 6.5 | |
| B7 | 自身点盒子 | `g1d_bringup/config/self_filter.yaml` | g1d_ws 估计值 | 8.2 | |
| B8 | `zero_hold` | `g1d_guard/config/guard.yaml` | 1.0 s | P0.5 `suggested_guard` | |
| B9 | 底盘停发后停车时间（放行门槛） | — | — | P0.5 `release_gate` | |
| B10 | `odom_latency` / `scan_latency` | `robot_bridge.yaml` | 0 | `CALIBRATION.md` 第 8 节（通常不需要） | |
| B11 | 机器人最高点高度 | — | — | 卷尺；须低于代价地图 / 栅格高度带上限 1.85 m | |

### C. 现场调参（验收后按需调整）

| # | 参数 | 文件 | 默认 | 什么时候调 |
|---|---|---|---|---|
| C1 | `stop_m` / `slow_m` / `creep_v` | `guard.yaml` | 0.45 / 1.20 / 0.05 | T1 净距不够、窄通道过不去（守护减速太早） |
| C2 | `vx_max`、`cmd_timeout`、`brake_v` | `guard.yaml` | 0.40 / 0.25 / 1.5 | **一般不改**：三者共同决定停车时间 ≤ 0.6 s；`brake_v` 不得超过 1.5（前倾） |
| C3 | 膨胀半径 `inflation_radius` | `nav2.yaml`（两个代价地图） | 0.55 m | 窄通道过不去（减小，但不小于 0.41 m 外接圆半径）或离障碍太近（加大） |
| C4 | 底盘轮廓 `footprint` | `nav2.yaml` | 0.60 × 0.56 m | 实测底盘尺寸不同；手臂伸出时 |
| C5 | 体素层高度带 `origin_z / z_voxels`、`min/max_obstacle_height` | `nav2.yaml` | 0.08 – 1.88 m | 机器人更高、或地面有斜坡 / 门槛被误判为障碍 |
| C6 | RPP `desired_linear_vel`、`lookahead_dist` | `nav2.yaml` | 0.35 m/s、0.6 m | 走得慢 / 弯道切角 |
| C7 | 到达容差 `xy_goal_tolerance` / `yaw_goal_tolerance` | `nav2.yaml` | 0.20 m / 0.25 rad | NavGPT 对精度的要求 |
| C8 | 进展检查 `movement_time_allowance` | `nav2.yaml` | 8 s | 守护减速区里频繁 "Failed to make progress" |
| C9 | `goal_search_radius`、`max_goal_distance` | `nav2.yaml`（`nav_manager`） | 1.0 m、9 m | 目标外移太远 / 不够 |
| C10 | 定位判据 `min_inlier_ratio`、`max_failed_registrations`、`registration_stale_s` | `localization_monitor` 参数 | 0.5 / 4 / 10 s | 现场与地图差异大（家具挪动）导致误报 |
| C11 | FAST_LIO `blind`、`filter_size_surf/map` | `g1d_bringup/config/fast_lio.yaml` | 0.6 / 0.3 / 0.4 | 自身遮挡多、场地很大或很小 |
| C12 | pgo 闭环 `loop_search_radius`、`loop_score_tresh` | `g1d_bringup/config/pgo.yaml` | 1.0 m、0.15 | 闭环没成功（墙线重影）或误闭环 |
| C13 | LIO 健康阈值（跳变、与轮速一致性） | `lio_bridge` 参数 | 0.3 m / 15°；2 s 内 0.3 m / 10° | 轮子打滑地面误报 |

### D. 流程与运维

| # | 项目 | 说明 |
|---|---|---|
| D1 | 启动顺序 | robot_interface → lio（静止 2 s）→ localization → navigation → web（第 11 节脚本） |
| D2 | 地图维护 | 家具大幅变动后重新建图；地图目录保留 `last_pose.yaml` 供 `init:=last` |
| D3 | 网络 | 自主导航只用网线；WiFi 下 Mid-360S 数据量过大 |
| D4 | 宇树服务 | 导航期间不要用宇树 App 遥控底盘；不要在 PC2 上启停服务 |
| D5 | 数据记录 | 每次现场测试录 `record.launch.py preset:=nav`，问题复现靠它 |

---

## 11. 日常运行

### 11.1 一键启动（tmux 示例）

保存为 `~/g1d_start.sh`，按现场修改地图名和 IP：

```bash
#!/usr/bin/env bash
MAP=~/g1d_maps/office
LIDAR="lidar_source:=livox host_ip:=192.168.123.222"
ENV='source ~/.bashrc'
tmux new-session -d -s g1d
tmux send-keys -t g1d "$ENV; ros2 launch g1d_bringup robot_interface.launch.py" C-m
sleep 5
tmux split-window -t g1d; tmux send-keys -t g1d "$ENV; ros2 launch g1d_bringup lio.launch.py $LIDAR" C-m
sleep 5                                   # 机器人保持静止
tmux split-window -t g1d; tmux send-keys -t g1d "$ENV; ros2 launch g1d_bringup localization.launch.py mode:=map map:=$MAP init:=last" C-m
tmux split-window -t g1d; tmux send-keys -t g1d "$ENV; ros2 launch g1d_bringup navigation.launch.py global_frame:=map" C-m
tmux split-window -t g1d; tmux send-keys -t g1d "$ENV; ros2 launch g1d_bringup web.launch.py" C-m
tmux select-layout -t g1d tiled; tmux attach -t g1d
```

### 11.2 启动后的检查（30 秒）

```bash
ros2 run g1d_tools nav_cli status
```

输出的是 `/nav/status`，逐项确认：
- `guard` 中 `blocked` 为空、`estop` 和 `manual` 为 false；
- `lio_ok: true`；
- `localized: true`；
- `body_locked: true`。

任何一项不对，先查对应的详细话题（`/g1d/guard_state`、`/g1d/lio_health`、`/g1d/localization_status`、`/g1d/body_lock` 的 `reasons`），再查 `DEPLOYMENT.md` 第 7 节。

### 11.3 停止

先 `nav_cli stop`，**确认机器人静止**后再 `tmux kill-session -t g1d`。守护退出时如果底盘还在动，只会补发几次零速，之后靠底盘自身超时停车，所以不要在行驶中直接关掉。

---

## 12. 记录数据与反馈问题

```bash
ros2 launch g1d_bringup record.launch.py preset:=nav out:=~/g1d_bags
```

| 预设 | 内容 | 用途 |
|---|---|---|
| `mapping` | 原始雷达、IMU、底盘、本体、TF | 离线重跑 LIO / 建图 |
| `nav` | `mapping` + 定位、代价地图、路径、守护、导航状态 | 导航问题复现（**现场默认用这个**） |
| `debug` | 全部话题 | 疑难问题 |

反馈问题时附上四样东西：
- bag 目录；
- `logs/interfaces.json`；
- `logs/acceptance/` 下的验收结果；
- 出问题时刻的 `ros2 topic echo /g1d/guard_state --once` 和 `/g1d/localization_status --once`。

---

## 13. 回滚

- **参数改坏了**：`git diff src/` 查看改动，`git checkout -- <文件>` 恢复，重新 `colcon build`；
- **版本问题**：`git log --oneline` 找到上一个验证过的提交，`git checkout <提交>` 后重新构建；
- **随时可以退回只用遥控的状态**：只启动 `robot_interface`（守护仍然保护遥控指令），不启动导航。
