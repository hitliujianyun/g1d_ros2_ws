# g1d_ros2_ws

宇树 G1-D 轮式人形机器人的 ROS 2（Jazzy）激光定位建图 + 导航避障工程：头部 Livox Mid-360S + FAST_LIO 建图与重定位，Nav2（MPPI）导航，独立安全守护 `g1d_guard` 作为底盘指令 `/cmd_vel_no_limit` 的唯一写入者。

> 开发中（当前完成 M0–M2：仓库 / CI、机器人接口与 TF、安全守护）。计划与进度见 [`PLAN.md`](PLAN.md)，设计见 [`docs/superpowers/specs/2026-11-17-g1d-ros2-nav-design.md`](docs/superpowers/specs/2026-11-17-g1d-ros2-nav-design.md)。

## 运行环境

- 导航计算机：Ubuntu 24.04 + ROS 2 Jazzy（`rmw_cyclonedds_cpp`），网线或 WiFi 接入机器人内网 `192.168.123.0/24`。
- 机器人 PC2（Ubuntu 20.04）上**不安装任何东西**；宇树 SDK2 话题在 ROS 2 中直接可见（DDS 域 0，`rt/xxx` = `/xxx`）。

## 构建

```bash
git clone --recursive https://github.com/hitliujianyun/g1d_ros2_ws.git
cd g1d_ros2_ws
sudo scripts/ci/install_deps.sh            # apt 依赖 + rosdep
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to-regex '^g1d_'
source install/setup.bash
python3 src/g1d_description/scripts/fetch_meshes.py   # 可选：RViz 显示用的网格
```

## 无硬件仿真

```bash
ros2 launch g1d_bringup sim.launch.py world:=corridor
ros2 topic pub -r 20 /nav/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"   # 模拟导航指令
ros2 topic echo /g1d/guard_state                                                    # 守护状态
ros2 topic pub --once /g1d/estop std_msgs/msg/Bool "{data: true}"                   # 急停（false 解除）
```

## 接入真机（第 0 阶段）

```bash
export G1D_NET_IF=enp3s0 G1D_LINK=wired          # WiFi：G1D_LINK=wifi（单播发现 PC2）
source install/share/g1d_bringup/scripts/g1d_env.sh
ros2 run g1d_tools check_interfaces --duration 5  # 话题、频率、时间戳偏差、写入者、底盘 HTTP
# 底盘指令行为实测（会让机器人移动；先停掉 g1d_guard）：
ros2 run g1d_tools cmd_vel_probe --tests silence,zero,clamp --i-know-it-moves
ros2 launch g1d_bringup robot_interface.launch.py require_cloud:=false   # M3 之前只用底盘雷达
```

`cmd_vel_probe` 的 `release_gate` 必须通过（停发后底盘 ≤ 0.6 s 停车），才允许进入自主导航阶段。

## 测试

```bash
pip install -e "src/g1d_core[test]" && pytest src/g1d_core/test   # Windows / Linux，无需 ROS
colcon test --packages-select-regex '^g1d_' --executor sequential && colcon test-result --verbose
```

GitHub Actions 在每次推送时运行以上全部测试（含仿真闭环：导航停发后停车时间、急停、遥控器接管、TF 链）。
