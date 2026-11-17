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

### M0 仓库与 CI（计划：`docs/superpowers/plans/2026-11-17-m0-m2-foundation.md`）
- [x] git 初始化、`.gitattributes`（LF）、`.gitignore`、设计文档、本文件
- [x] `upstream.lock` + `scripts/clone_upstream.py`；`unitree_ros2` 子模块 @ `668d1ec`
- [x] GitHub Actions：lint-text、core-tests（Ubuntu + Windows）、ros（`ros:jazzy` 构建 + 单测 + 闭环集成测试）
- [x] 公开仓库 `hitliujianyun/g1d_ros2_ws` 并推送

### M1 接口与 TF
- [x] `g1d_msgs`（GuardState、BodyLock）
- [x] `g1d_description`：HF URDF 原样提交 → mesh 路径补丁；xacro 增加 `base_link` / `livox_frame` / `livox_imu` / `chassis_laser`；`extrinsics.yaml`；mesh 下载脚本
- [x] `g1d_core.kinematics`（URDF 正运动学，与手算链一致到 1e-5）、`g1d_core.body`（关节映射、姿态锁定）
- [x] `g1d_robot_bridge`：`chassis_restamp`、`body_state`、`chassis_events`
- [x] `g1d_sim` `chassis_sim`（差速 + 指令超时、滞后 181 s 时间戳、底盘雷达、宇树关节 / 遥控话题、HTTP 事件、真值）
- [x] `g1d_tools`：`check_interfaces`、`cmd_vel_probe`（含放行门槛判定）

### M2 安全守护
- [x] `g1d_core.guard`（原样迁移 `guard_core.py` + 17 项单测，新增部署参数停车测试）
- [x] `g1d_guard` 节点：TF 输入、急停锁存、遥控接管、运行时参数白名单、写入者检查
- [x] `g1d_bringup`：`robot_interface.launch.py`、`sim.launch.py`、CycloneDDS 有线 / WiFi 配置、`g1d_env.sh`
- [x] CI 闭环：导航停发后停车 ≤ 0.6 s、急停、遥控器接管、TF 链、重打时间戳、姿态锁定

### M3 雷达与 LIO（计划：`docs/superpowers/plans/2026-11-17-m3-lio.md`）
- [x] livox_ros_driver2 @2144554 vendor：colcon 构建补丁（package.xml、ROS2 / 发行版默认、C++17）。**时钟映射不需要**（驱动已用主机时钟打戳）；Livox-SDK2 子模块 + 安装脚本
- [x] `g1d_lidar`：`self_filter`（TF 放置的机身盒子）、`utlidar_adapter`（PointCloud2 → CustomMsg，逐点时间自动识别）；gtest
- [x] FAST_LIO ROS2 @cc3c9e8 vendor：C++17、坐标系参数、`publish.tf_en`、`fclose(NULL)` 保护
- [x] `g1d_core.lio` + `g1d_localization/lio_bridge`（锚定、SE(2) 投影、50 Hz 轮速外推、健康监测）
- [x] `g1d_lidar_sim`（Mid-360 非重复扫描 + IMU + 机身自遮挡）；`lio.launch.py`（livox | unitree | sim）；CI：LIO 闭环 + 自身点过滤

### M4 建图
- [x] FASTLIO2_ROS2 `pgo` @7baa59f vendor（GTSAM iSAM2 + ICP 闭环）：rosdep 键、`publish_tf`
- [x] `g1d_core.grid` / `pcd`（高度带栅格、map_server 格式、指标）
- [x] `g1d_mapping/map_manager`（`/g1d/save_map`：map 系 PCD、关键帧、轨迹、栅格、meta）；`mapping.launch.py`；CI：建图 → 保存 → 质量

### M5 定位
- [x] small_gicp 子模块 @fa0cfc9；small_gicp_relocalization @81c9bd3 vendor：C++17、`~/status`、依赖顺序
- [x] `g1d_core.reloc`：以建图轨迹为种子的全局搜索（体素命中率、粗到细）——替代 KISS-Matcher（依赖链更轻，单测覆盖）
- [x] `localization_monitor`（定位状态、`/g1d/relocalize` pose | auto | last、上次位姿、启动初始化）；`localization.launch.py`（map | odom，含 map_server）
- [ ] CI：自动初始化、行驶跟踪、错误初值重新收敛（测试已写，等待 CI 结果）

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
| 2026-11-17 | M0–M2 完成：仓库与 CI；G1-D 描述（URDF 来自 HF 数据集 @27b13d3）；机器人接口三个节点；仿真底盘；守护节点；现场工具；CI 全绿（Windows / Ubuntu 核心库 45 项；Jazzy 容器 58 项含闭环）。闭环实测：0.40 m/s 时导航停发 → 守护指令 0.486 s 归零、仿真底盘 0.503 s 停住（要求 ≤ 0.6 s）。修正：Mid-360 IMU 外参符号（livox→imu = (0.011, 0.02329, −0.04412)）；守护减速度保持 1.5 m/s²（前倾临界约 2.7 m/s²），`vx_max` 降为 0.40 = MPPI 上限，停车 ≈ 0.52 s。CI 踩坑：`set -u` 与 ROS setup.bash 冲突；unitree 消息包未声明 `rosidl_generator_dds_idl`；并行测试串扰（改为每包独立 `ROS_DOMAIN_ID` + 顺序执行）；best-effort 发布者需 best-effort 订阅 |

| 2026-11-17 | M3：上游阅读后取消时钟映射补丁；vendor livox_ros_driver2 / FAST_LIO（补丁均为独立提交）；自身点过滤、宇树话题适配、雷达仿真、lio_bridge。CI：约 8 m 方形闭环位置误差 0.087–0.088 m（要求 < 0.10）、航向 0.02–0.04°，自身点漏检 0。踩坑：驱动 `.gitignore` 忽略了 package.xml |
| 2026-11-17 | M4：vendor pgo（GTSAM），map_manager 保存地图。CI：8 万点、48 关键帧，点到真实表面距离中位数 3.0 cm / P90 9.2 cm，栅格占据精度 0.889 |
| 2026-11-17 | M5：small_gicp 重定位 + 自研种子全局搜索 + 定位监控。单测：3 个位姿均在 0.2 m / 5° 内找到，陌生扫描被拒绝 |

## 4. 上机待办（本机无法完成）

- [ ] Q1 `cmd_vel_probe`：停发后停车时间（放行门槛 ≤ 0.6 s）、限幅、零速保持、优先级、AEB
- [ ] Q2 Jazzy ↔ 宇树 DDS 逐话题互通与 QoS
- [ ] Q3 Mid-360S 占用情况、IP、`/utlidar/*`
- [ ] Q4 外参与身体链偏移标定
- [ ] Q5–Q7 HTTP 事件格式、WiFi 带宽、关节序号
