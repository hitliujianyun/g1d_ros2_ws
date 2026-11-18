# 移植到实机与参数设置

本文面向把工程装到导航主机、接上 G1-D 并逐步放开自主导航的工程师。设计背景见 [`PROJECT_OVERVIEW.md`](PROJECT_OVERVIEW.md)，外参标定见 [`CALIBRATION.md`](CALIBRATION.md)。

## 1. 硬件与网络

| 设备 | 说明 |
|---|---|
| 导航主机 | Ubuntu 24.04 + ROS 2 Jazzy，x86_64（开发机 RTX 5090，导航本身只用 CPU） |
| 机器人 PC2 | Jetson Orin NX，Ubuntu 20.04，`192.168.123.164`。**不安装任何东西**，宇树服务照常运行 |
| 连接 | **自主导航请用网线**接入机器人内网 `192.168.123.0/24`（Mid-360S 可直连）；WiFi 只用于监控和低速遥控 |
| DDS | CycloneDDS，域 0（宇树 SDK2 = 0.10.2，Jazzy 的 rmw_cyclonedds 也是 0.10.x） |

导航主机需要一个 `192.168.123.x` 的静态地址（例如 `192.168.123.222`，避开机器人已用地址），并记下网卡名（`ip -br addr`）。

## 2. 安装

```bash
git clone --recursive https://github.com/hitliujianyun/g1d_ros2_ws.git
cd g1d_ros2_ws
sudo scripts/ci/install_deps.sh          # apt 依赖、Livox-SDK2（/usr/local）、rosdep
sudo scripts/install_host_extras.sh      # RViz2、foxglove_bridge、MCAP 录制
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to-regex '^g1d_' --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 src/g1d_description/scripts/fetch_meshes.py    # 机器人模型网格（可选，仅显示）
```

每个终端都要加载环境（建议写进 `~/.bashrc`）：

```bash
source /opt/ros/jazzy/setup.bash && source ~/g1d_ros2_ws/install/setup.bash
export G1D_NET_IF=enp3s0            # 接机器人内网的网卡
export G1D_LINK=wired               # wifi：单播发现 PC2（G1D_PEER 默认 192.168.123.164）
source ~/g1d_ros2_ws/install/share/g1d_bringup/scripts/g1d_env.sh
```

## 3. 启动文件一览

| 启动文件 | 内容 |
|---|---|
| `robot_interface.launch.py` | URDF / TF、SLAMWARE 重打时间戳、关节 → `/joint_states` + 姿态锁定、底盘事件、**守护** |
| `lio.launch.py lidar_source:=livox\|unitree\|sim` | Mid-360S 驱动或 `/utlidar/*` 适配 → 自身点过滤 → FAST_LIO → `lio_bridge` |
| `mapping.launch.py` | pgo 闭环 + `map_manager`（`/g1d/save_map`） |
| `localization.launch.py mode:=map map:=<目录> \| mode:=odom` | 先验地图重定位 + 监控 + map_server；或纯里程计 |
| `navigation.launch.py global_frame:=map\|odom` | Nav2 + `nav_manager` |
| `web.launch.py` / `viz.launch.py` / `record.launch.py` | 浏览器调试台 / RViz2、Foxglove / rosbag2 预设 |
| `sim.launch.py` | 无硬件闭环仿真（`lio:=true mapping:=true nav:=true`） |

## 4. 必须实测 / 标定的参数

代码中这些值都是**估计值**，上线前逐项确认（括号内为文件）。

| # | 参数 | 当前值 | 怎么得到 |
|---|---|---|---|
| 1 | 底盘停发后的停车时间（**自主导航放行门槛 ≤ 0.6 s**） | 仿真 0.5 s 超时 | `cmd_vel_probe --tests silence` |
| 2 | `zero_hold`（`g1d_guard/config/guard.yaml`） | 1.0 s | 同上，取 停车时间 + 0.3 s |
| 3 | 底盘限幅、零速是否锁轮、与遥控器的优先级、AEB 是否生效 | 未知 | `cmd_vel_probe --tests zero,clamp,hold` + 现场观察 |
| 4 | Mid-360S IP、是否被宇树服务占用（决定 `lidar_source`） | `192.168.123.120`（G1 值） | `check_interfaces`（看 `/utlidar/*`）、Livox Viewer |
| 5 | `head_to_livox_xyz / rpy`（`g1d_description/config/extrinsics.yaml`） | `0.05 0 0.55` / `π −0.04 0` | `calib_extrinsics`（[CALIBRATION.md](CALIBRATION.md)） |
| 6 | `base_to_agv_z` | 0.10 m | 卷尺：地面到 `AGV_link` 原点 |
| 7 | `lift_scale / lift_offset`（`g1d_robot_bridge/config/robot_bridge.yaml`） | 1 / 0 | 升降到最低 / 最高，对比 `hispeed_state.y` 与雷达高度变化 |
| 8 | `waist_yaw_index`、`arm_first_index`、关节方向 | 12 / 15 | 逐个转动关节，看 `/joint_states` 与 RViz 模型是否一致 |
| 9 | `nav_waist_pitch`、`arm_tuck`（导航姿态） | 全 0 | 摆好导航姿态后读 `/joint_states` |
| 10 | 自身点盒子（`g1d_bringup/config/self_filter.yaml`） | g1d_ws 估计值 | 静止录点云，调到车身附近没有稳定点簇 |
| 11 | 底盘 HTTP 事件字段名（`type` / `timestamp`） | 假设 | `check_interfaces` 的 `chassis_http.last` |
| 12 | 守护停车距离 `stop_m`、膨胀半径 | 0.45 m / 0.55 m | T1 / T6 验收后调整 |

## 5. 分阶段上线

每一阶段通过后再进入下一阶段。前两个阶段限速 0.3 m/s，侧后方有人值守，急停按钮在手边。

### P0 接口确认（不动车）

```bash
ros2 run g1d_tools check_interfaces --duration 10
```

检查：SLAMWARE odom / scan（时间戳偏差约 181 s）、`/hispeed_state`、`/lowstate`、`/waistpitchstate`、`/wirelesscontroller` 都有数据；`/cmd_vel_no_limit` 没有其他发布者；`/utlidar/*` 是否存在；底盘 HTTP 可达。报告在 `logs/interfaces.json`。

### P0.5 底盘指令行为（会动车，3 m 空地）

```bash
ros2 run g1d_tools cmd_vel_probe --tests silence,zero,clamp,hold --i-know-it-moves
```

**`release_gate.pass` 必须为 true**（停发后底盘自己在 0.6 s 内停住）。不满足就不要进入自主导航：守护运行在导航主机上，主机或网络故障时只能靠底盘自身超时。备选方案是在 PC2 上加一个极简看门狗（设计文档 6.2 节）。

### P1 只用守护（底盘雷达）

```bash
ros2 launch g1d_bringup robot_interface.launch.py require_cloud:=false
ros2 run g1d_tools acceptance T1      # 正对墙 / 桌 / 人腿，10 次停车净距 >= 0.40 m
ros2 run g1d_tools acceptance T2      # 指令中断停车时间 <= 0.6 s
```

### P2 LIO

1. 标定外参（[CALIBRATION.md](CALIBRATION.md)），重启 `robot_interface`。
2. `ros2 launch g1d_bringup lio.launch.py lidar_source:=livox host_ip:=192.168.123.222`（被占用时 `lidar_source:=unitree`）。**启动时机器人静止 2 s**（IMU 初始化）。
3. `/g1d/lio_health` 应为 `ok`；RViz 中 `/g1d/obstacle_cloud` 车身附近无点簇。
4. 验收：`acceptance T3`（10 m 矩形闭环 < 0.10 m / 2°）、`acceptance T4`（原地 3 圈漂移 < 0.05 m）。
5. 守护改为 `require_cloud:=true`（默认）。

### P3 建图

```bash
ros2 launch g1d_bringup mapping.launch.py maps_dir:=~/g1d_maps
# 遥控或 Web 绕场地一周，尽量回到起点形成闭环
ros2 service call /g1d/save_map g1d_msgs/srv/SaveMap "{name: office}"
```

检查 `~/g1d_maps/office/map.pgm`（桌子、货架应为黑色占据）。重复建图精度：从同一标记出发再建一次，`ros2 run g1d_tools map_compare ~/g1d_maps/office ~/g1d_maps/office2`。

### P4 定位与导航

```bash
ros2 launch g1d_bringup localization.launch.py mode:=map map:=~/g1d_maps/office init:=auto   # 或 last / none
ros2 launch g1d_bringup navigation.launch.py global_frame:=map
ros2 launch g1d_bringup web.launch.py          # http://<主机>:8080
```

验收：T5（走廊绕障 ≥ 9/10）、T6（桌沿 10/10 无碰撞或安全拒绝）、T7（重定位重复精度 < 0.05 m / 1°）。没有地图时用 `mode:=odom` + `global_frame:=odom`。

### P5 NavGPT

```python
from g1d_nav_manager.client import NavClient, prompt_feedback
nav = NavClient(node)
gid = nav.send_relative(dx, dy, dyaw, stamp=image_stamp)   # 图像时刻（主机时钟）
r = nav.wait(gid)
prompt += prompt_feedback(r)                                # 失败原因 + 可通行方向
```

旧版 JSON 接口仍可用：`/nav/goal`（= DDS `rt/nav/goal`）与 `/nav/status`。

## 6. 主要参数文件

| 文件 | 内容 |
|---|---|
| `src/g1d_guard/config/guard.yaml` | 守护：限速、加减速度（**减速度不要超过 1.5 m/s²，防前倾**）、超时、反射层距离、遥控 |
| `src/g1d_robot_bridge/config/robot_bridge.yaml` | 时间戳延迟、升降换算、关节序号、导航姿态与锁定容差、底盘 HTTP |
| `src/g1d_description/config/extrinsics.yaml` | 外参（标定工具写回） |
| `src/g1d_bringup/config/fast_lio.yaml` | FAST_LIO |
| `src/g1d_bringup/config/self_filter.yaml` | 自身点盒子 |
| `src/g1d_bringup/config/pgo.yaml` | 闭环：关键帧间隔、闭环搜索半径 / 时间 / ICP 阈值 |
| `src/g1d_bringup/config/nav2.yaml` | Nav2：代价地图（体素高度带 0.08–1.85 m、膨胀 0.55 m）、RPP 控制器（0.35 m/s）、NavFn、行为、`nav_manager`（目标外移半径 `goal_search_radius` 1.0 m） |
| `src/g1d_bringup/config/nav2_mppi.yaml` | MPPI 控制器（`controller:=mppi` 时替换 RPP）；`regenerate_noises` 必须为 `true` |
| 任意 YAML（`overlay:=` 或环境变量 `G1D_NAV2_OVERLAY`） | 调参覆盖：深度合并到 Nav2 参数上（值为 `null` 表示删除该键），不用改仓库里的文件 |
| `src/g1d_web/config/web.yaml` | Web 端口与显示 |

修改 `install/` 外的源文件后需要 `colcon build` 才生效（或用 `--symlink-install` 构建）。守护参数可在 Web「参数」页热更新并写回 `guard.yaml`。

## 7. 故障排查

| 现象 | 检查 / 处理 |
|---|---|
| `ros2 topic list` 看不到宇树话题 | `G1D_NET_IF` 是否正确；`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；`ROS_DOMAIN_ID=0`；WiFi 用 `G1D_LINK=wifi`；关掉主机防火墙对 UDP 7400+ 的拦截 |
| 能看到话题但收不到数据 | QoS：宇树话题用 best-effort 订阅（`ros2 topic echo --qos-reliability best_effort`） |
| 守护拒绝启动："other publishers on /cmd_vel_no_limit" | 宇树遥控 / 其他程序在写底盘；停掉它，或确认后 `other_writers:=warn` |
| 守护 `blocked: scan_stale / cloud_stale` | `/chassis/scan` 或 `/g1d/obstacle_cloud` 中断；P1 阶段用 `require_cloud:=false` |
| 守护 `blocked: front` 但前方没东西 | 自身点泄漏：调 `self_filter.yaml`；或 `stop_m` 过大 |
| LIO 不启动 / `no_lio` | 雷达在线？`host_ip` 是否本机地址；被宇树服务占用 → `lidar_source:=unitree`；启动时必须静止 |
| LIO `jump` / `wheel_inconsistent` | 长走廊等退化场景；轮子打滑；外参错误（原地转圈时位置画圈 → 重新标定） |
| `/joint_states` 不发布（"stale lift, lowstate …"） | 宇树话题未到；老款腰部没有 `/waistpitchstate` → `require_waist_pitch: false` |
| 导航立即失败 `body_not_locked` | 手臂 / 腰 / 升降不在导航姿态；`/g1d/body_lock` 的 `reasons` 指出哪一项 |
| `localization_lost` | 看 `/g1d/localization_status` 的 `reasons`：`registration_failing`（连续 4 次配准失败）= 先验地图与现场差异大或初值错 → Web「自动重定位」/ RViz 2D Pose Estimate；`registration_stale`（10 s 没有配准结果）= small_gicp 卡住或 `/g1d/registered_scan` 断了。`registration_slow` 只是提示（主机负载高），不影响导航 |
| 目标被拒 `goal_in_obstacle` / `goal_out_of_range` | 目标在障碍内且 1.0 m 内无可达点（`goal_search_radius`）/ 超出全局代价地图 |
| 日志 "LIO gap … odom re-anchored" | FAST_LIO 崩溃后被自动重启（上游偶发启动段错误），odom 用车轮里程计续接；建图时不自动重启（`respawn:=false`），需重新建图 |
| 导航很慢、走走停停 | MPPI 时检查 `regenerate_noises: true`、控制周期 = `model_dt`；两种控制器都检查守护是否在减速区（`/g1d/guard_state` 的 `front_clear` < 1.2 m） |
| 机器人在桌下停住 / 绕不过桌沿 | 体素层高度带（0.08–1.85 m）与膨胀半径；Web 中查看代价地图 |
| `nav2_unavailable` | Nav2 生命周期未激活：查看 `lifecycle_manager_navigation` 日志 |
