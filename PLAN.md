# g1d_ros2_ws 开发计划 / TODO / 进度

> 设计文档：[`docs/superpowers/specs/2026-11-17-g1d-ros2-nav-design.md`](docs/superpowers/specs/2026-11-17-g1d-ros2-nav-design.md)
> 本文件在开发过程中持续更新：计划（第 1 节）、TODO（第 2 节，随进度勾选）、进度日志（第 3 节）、上机待办（第 4 节）。

## 1. 计划概要

| 项 | 内容 |
|---|---|
| 运行平台 | 外接 Ubuntu 24.04 + RTX 5090 开发机，ROS 2 Jazzy（rmw_cyclonedds），经网线 / WiFi 与 PC2 用 DDS 域 0 通信 |
| 核心组件 | livox_ros_driver2 → 自身点过滤 → FAST_LIO → lio_bridge；GTSAM 闭环建图；small_gicp 重定位；Nav2（NavFn + MPPI）；rclpy 守护 |
| 本机验证 | Windows pytest（`g1d_core`）+ GitHub Actions（`ros:jazzy` 构建、单测、闭环仿真集成测试） |
| 仓库 | `github.com/hitliujianyun/g1d_ros2_ws`（公开） |

## 2. TODO

### M0 仓库与 CI
- [x] git 初始化、`.gitattributes`（LF）、`.gitignore`、设计文档、本文件
- [ ] 上游克隆到 `_upstream/`；submodule（未修改）与 vendor（原样提交 → 补丁）
- [ ] 包骨架、GitHub Actions（core-tests / ros-build / ros-integration）
- [ ] 创建 GitHub 仓库并推送

### M1 接口与 TF
- [ ] `g1d_msgs`
- [ ] `g1d_description`（G1-D URDF + livox / base_link / chassis_laser）
- [ ] `g1d_robot_bridge`（重打时间戳、joint_states、姿态锁定、HTTP 事件）
- [ ] `g1d_sim` 底盘部分（差速、指令超时、滞后时间戳、宇树关节话题）
- [ ] `check_interfaces`、`cmd_vel_probe`（ROS 2 版）

### M2 安全守护
- [ ] `g1d_core.guard`（迁移 `guard_core.py` + 单测）
- [ ] `g1d_guard` 节点、遥控 deadman、急停、写入者检查
- [ ] CI：杀导航 ≤ 0.6 s 停车

### M3 雷达与 LIO
- [ ] livox_ros_driver2 补丁（Mid-360S 配置、时钟映射）
- [ ] `g1d_lidar`（自身点过滤、`/utlidar/*` 适配）
- [ ] FAST_LIO 补丁（Jazzy / C++17、坐标系参数化）
- [ ] `g1d_lio_bridge`（odom → base_link、障碍点云、健康监测）
- [ ] `g1d_sim` 雷达 / IMU；CI：LIO 精度

### M4 建图
- [ ] 关键帧 + GTSAM 闭环 + SaveMap
- [ ] `g1d_core.grid` 高度带栅格 + 单测
- [ ] 建图指标；CI：建图 → 保存

### M5 定位
- [ ] small_gicp 重定位 + 初始位姿 + KISS-Matcher 自动初始化
- [ ] 纯里程计模式；定位状态
- [ ] CI：偏离初值重定位

### M6 导航
- [ ] Nav2 参数（代价地图、MPPI、RotationShim、BT）
- [ ] `g1d_nav_manager`（NavigateRelative、延迟补偿、目标外移、reason、free_sectors、JSON 兼容层、客户端库）
- [ ] CI：绕障、桌面悬空、目标在障碍内、延迟补偿

### M7 可视化与工具
- [ ] RViz2 / Foxglove 配置
- [ ] 录制预设、标定工具、T1–T6 + 建图 / 重定位指标

### M8 Web 调试台
- [ ] `g1d_web` 迁移（rclpy hub）+ 端到端测试

### M9 文档
- [ ] README、docs/PROJECT_OVERVIEW、docs/DEPLOYMENT（移植 + 参数 + 分阶段上线 + 故障排查）、docs/CALIBRATION

## 3. 进度日志

| 时间 | 进度 |
|---|---|
| 2026-11-17 | 需求澄清与方案评审：Jazzy 跑在 5090 主机、守护为主机 rclpy 节点、模块化建图 / 重定位、CI 验证、Web 纳入本期。本机为 OpenStack 虚拟机（无硬件虚拟化，WSL2 / Docker 不可用）。设计文档提交 |

## 4. 上机待办（本机无法完成）

- [ ] Q1 `cmd_vel_probe`：停发后停车时间（放行门槛 ≤ 0.6 s）、限幅、零速保持、优先级、AEB
- [ ] Q2 Jazzy ↔ 宇树 DDS 逐话题互通与 QoS
- [ ] Q3 Mid-360S 占用情况、IP、`/utlidar/*`
- [ ] Q4 外参与身体链偏移标定
- [ ] Q5–Q7 HTTP 事件格式、WiFi 带宽、关节序号
