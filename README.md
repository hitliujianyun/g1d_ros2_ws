# g1d_ros2_ws

宇树 G1-D 轮式人形机器人的 ROS 2（Jazzy）**激光定位建图 + 导航避障**工程：

- 头部 Livox Mid-360S（倒装）+ FAST_LIO 激光惯性里程计，关节反馈驱动的 TF（升降 / 腰部运动）；
- 建图：关键帧位姿图闭环（GTSAM），保存三维点云地图与高度带 2D 栅格（桌沿等悬空障碍可见）；
- 定位：先验地图上 small_gicp 持续重定位（RViz / Web 给初值、自动全局搜索或上次位姿），或无地图纯里程计；
- 导航：Nav2（NavFn + Regulated Pure Pursuit，MPPI 可选 `controller:=mppi`），体素 + 底盘雷达代价地图，目标自动外移与明确的失败原因；
- 安全：独立守护 `g1d_guard` 是底盘指令 `/cmd_vel_no_limit` 的唯一写入者（限幅、看门狗、反射停车、急停、遥控接管）；
- NavGPT：相对航点 + 图像时刻，延迟补偿，失败返回原因与可通行方向；
- 工具：RViz2 / Foxglove、浏览器调试台、接口探测、底盘指令实测、外参标定、rosbag2 预设、验收 T1–T7。

文档：[项目详细介绍](docs/PROJECT_OVERVIEW.md) · [移植到实机与参数设置](docs/DEPLOYMENT.md) · [标定说明](docs/CALIBRATION.md) · [计划与进度](PLAN.md) · [设计文档](docs/superpowers/specs/2026-11-17-g1d-ros2-nav-design.md)

## 运行环境

| 角色 | 环境 |
|---|---|
| 导航主机 | Ubuntu 24.04 + ROS 2 Jazzy（`rmw_cyclonedds_cpp`），网线接入机器人内网 `192.168.123.0/24` |
| 机器人 PC2 | Ubuntu 20.04，宇树服务照常运行，**不需要安装任何东西**（宇树 SDK2 话题在 ROS 2 中直接可见） |

## 构建

```bash
git clone --recursive https://github.com/hitliujianyun/g1d_ros2_ws.git && cd g1d_ros2_ws
sudo scripts/ci/install_deps.sh            # apt 依赖、Livox-SDK2、rosdep
sudo scripts/install_host_extras.sh        # RViz2、foxglove_bridge、MCAP（可选）
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to-regex '^g1d_' --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 无硬件仿真

```bash
ros2 launch g1d_bringup sim.launch.py world:=room lio:=true nav:=true     # 底盘 + Mid-360 仿真 + LIO + Nav2
ros2 launch g1d_bringup web.launch.py display_frame:=map                  # http://localhost:8080
ros2 run g1d_tools nav_cli go 3.0 1.0                                     # 相对当前位姿的目标
ros2 launch g1d_bringup sim.launch.py world:=room lio:=true mapping:=true # 建图，然后：
ros2 service call /g1d/save_map g1d_msgs/srv/SaveMap "{name: room}"
```

## 真机

```bash
export G1D_NET_IF=enp3s0 G1D_LINK=wired
source install/share/g1d_bringup/scripts/g1d_env.sh
ros2 run g1d_tools check_interfaces                                       # P0 接口
ros2 run g1d_tools cmd_vel_probe --tests silence --i-know-it-moves        # 放行门槛：停发后 <= 0.6 s 停车
ros2 launch g1d_bringup robot_interface.launch.py                         # TF + 守护
ros2 launch g1d_bringup lio.launch.py lidar_source:=livox host_ip:=192.168.123.222
ros2 launch g1d_bringup mapping.launch.py                                 # 建图（或下面的定位）
ros2 launch g1d_bringup localization.launch.py mode:=map map:=~/g1d_maps/office init:=auto
ros2 launch g1d_bringup navigation.launch.py global_frame:=map
ros2 launch g1d_bringup web.launch.py
```

分阶段上线、必须标定的参数和故障排查见 [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)。

## NavGPT 调用

```python
from g1d_nav_manager.client import NavClient, prompt_feedback
nav = NavClient(node)
gid = nav.send_relative(dx, dy, dyaw, stamp=image_stamp)     # 相对图像时刻的车体
result = nav.wait(gid)                                        # state / reason / free_sectors / partial / adjusted
prompt += prompt_feedback(result)
```

旧版 JSON 接口（`rt/nav/goal` / `rt/nav/status`）仍然可用。

## 测试

```bash
pip install -e "src/g1d_core[test]" && pytest src/g1d_core/test          # Windows / Linux，无需 ROS
colcon test --packages-select-regex '^g1d_' --executor sequential && colcon test-result --verbose
```

GitHub Actions 每次推送运行全部单测与闭环集成测试（停车时间、急停、LIO 精度与自身点过滤、建图质量、重定位、导航场景），测量值见 CI 日志的 "measured values" 步骤与 artifact。

## 目录

```text
src/g1d_*          本工程的包（g1d_core 为不依赖 ROS 的算法库）
src/vendor/        修改过的上游代码（原样提交 + 补丁提交，见各 UPSTREAM.md）
src/external/      未修改的上游（git 子模块）     third_party/Livox-SDK2（子模块，系统安装）
docs/              文档                           scripts/   安装、上游克隆、检查脚本
```

许可：本工程代码 Apache-2.0；vendor 代码保持各自许可（FAST_LIO / ikd-Tree 为 GPL-2.0，对外分发前需按 GPL 处理）。
