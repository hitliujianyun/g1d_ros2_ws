# G1-D ROS 2 激光定位建图 + 导航避障：设计文档

| 项 | 内容 |
|---|---|
| 日期 | 2026-11-17 |
| 状态 | 已与需求方确认（方案评审通过） |
| 输入 | `design/ROS2_Reuse_Notes.md`（可复用信息）、`design/G1D_LIO_MPPI_Nav_Plan.md`（原始方案）、`g1d_ws`（非 ROS 版本代码，GitHub `hitliujianyun/g1d_nav_ws_b`） |
| 仓库 | `github.com/hitliujianyun/g1d_ros2_ws`（公开） |

本文记录方案评审中确定的决策和设计。凡标注 **【待实测】** 的内容都不作为事实写进代码：代码里做成参数或可切换的模式，在真机第 0 阶段确认。

---

## 1. 已确认的关键决策

| # | 决策 | 结论 | 理由 |
|---|---|---|---|
| D1 | 运行平台 | 导航系统运行在**外接的 Ubuntu 24.04 + RTX 5090 开发机**，通过网线或路由器 WiFi 与机器人 PC2（Ubuntu 20.04）用 DDS 通信。PC2 上不安装任何东西 | PC2 只有 Foxy（已停止维护、Nav2 无 MPPI）；不改动宇树系统 |
| D2 | ROS 2 发行版 | **Jazzy**（24.04 原生 apt），`rmw_cyclonedds_cpp` | Jazzy 维护到 2029-05；Humble 2027-05 即停止维护。Jazzy 的 CycloneDDS 与宇树 SDK2（0.10.2）同为 0.10.x 系列，互通【待实测】 |
| D3 | 安全守护位置 | 守护是 5090 主机上的 **rclpy 节点** `g1d_guard`，复用 `g1d_ws` 的 `guard_core` | 需求方选择。代价：主机崩溃 / 断网时只能依靠底盘自身的指令超时，见 6.2 节"放行门槛" |
| D4 | 建图 / 重定位 | **模块化组合**：FAST_LIO（ROS 2 分支）里程计 + 关键帧位姿图闭环（GTSAM）+ small_gicp 持续重定位（可选 KISS-Matcher 无初值全局匹配）+ 自研高度带 2D 栅格 | 各组件在 Mid-360 机器人上广泛使用、可单独替换、纯 CPU；建图和定位共用一个里程计 |
| D5 | 本机验证 | Windows 上 pytest 测纯 Python 核心库；完整 ROS 2 工程在 **GitHub Actions**（`ros:jazzy` 容器）中构建、单测并跑闭环仿真集成测试 | 本机是 OpenStack 虚拟机，CPU 未开放硬件虚拟化，WSL2 / Docker 不可用；无法访问 5090 主机 |
| D6 | Web 调试台 | 纳入本期（M8），把 `g1d_web` 的 DDS hub 换成 rclpy | 需求方确认 |
| D7 | 开源代码管理 | 上游先 clone 到 `_upstream/`（不入库）；未修改的用 git submodule（`src/external/`）；要修改的先原样提交到 `src/vendor/<name>`（注明 commit），再以独立提交打补丁；每个 vendor 目录有 `UPSTREAM.md` | 所有修改可追溯 |

## 2. 范围

### 2.1 本期功能

1. **建图**：Mid-360S + IMU 建立三维点云地图（闭环优化），保存 PCD；同时生成 Nav2 可用的 2D 栅格地图（高度带投影，能表示桌沿等悬空障碍）。
2. **定位**：在已保存的地图上重定位（初始位姿可在 RViz / Web 给定，或自动全局匹配），持续输出 `map → odom → base_link`；也支持无先验地图的纯里程计模式。
3. **导航避障**：Nav2（全局规划 + MPPI），代价地图含三维体素层（悬空障碍不被地面射线误清除）、底盘雷达层和膨胀；目标在障碍内自动外移，失败给出明确原因。
4. **安全**：独立守护节点是 `/cmd_vel_no_limit` 唯一写入者；导航输出到中间话题，经守护下发。
5. **NavGPT 接入**：相对航点 `(dx, dy, dyaw)` + 图像时刻 `stamp`，导航系统用位姿历史补偿延迟；失败返回原因和 `free_sectors`。
6. **可视化与操作**：RViz2 / Foxglove 配置；浏览器调试台（点云、地图、定位、路径、代价地图、点选目标、多航点任务、遥控、急停、验收测试）。
7. **工具**：接口探测、底盘指令行为实测、外参标定、rosbag2 录制、验收 T1–T6、建图 / 重定位精度指标。

### 2.2 不做

- 导航过程中腰部 / 立柱 / 手臂运动的动态补偿（导航期间要求姿态锁定，变化即停车）。
- 楼梯、坡道、多楼层地图。
- Gazebo / Isaac Sim 场景（以后可在 5090 上单独做，不进入 CI）。

## 3. 总体架构

```text
┌──────────── PC2（Ubuntu 20.04，宇树服务，不安装任何东西）────────────┐
│ SLAMWARE: /slamware_ros_sdk_server_node/odom、/scan（时间戳慢约 181 s） │
│ 本体: /hispeed_state  /lowstate  /waistpitchstate  /wirelesscontroller   │
│ 底盘: /cmd_vel_no_limit（只接受 g1d_guard 写入）     [可选] /utlidar/*   │
└───────────────────────────── DDS 域 0（CycloneDDS 0.10.x）─────────────┘
        ▲  网线（自主导航）/ WiFi（监控、低速遥控）          Mid-360S UDP（网线时直连）
┌──────────── 5090 开发机（Ubuntu 24.04 + ROS 2 Jazzy，rmw_cyclonedds）────────────┐
│ g1d_robot_bridge   重打时间戳 → /chassis/odom、/chassis/scan；关节 → /joint_states；姿态锁定 │
│ robot_state_publisher（G1-D URDF + livox 坐标系）→ base_link … head_link → livox     │
│ livox_ros_driver2 → g1d_self_filter → FAST_LIO → g1d_lio_bridge → odom → base_link   │
│ 建图: g1d_mapping（关键帧 + GTSAM 闭环 → map.pcd + 高度带栅格 map.yaml/pgm）          │
│ 定位: small_gicp_relocalization（+ KISS-Matcher）→ map → odom；纯里程计模式为单位变换   │
│ Nav2: 规划器 + MPPI + 代价地图（Voxel / 底盘雷达 / 静态 / 膨胀）→ /nav/cmd_vel         │
│ g1d_nav_manager: NavGPT 相对航点 action、目标外移、失败原因、free_sectors             │
│ g1d_guard（rclpy，复用 guard_core）→ /cmd_vel_no_limit（唯一写入者）                   │
│ RViz2 / Foxglove（foxglove_bridge）/ g1d_web                                          │
└──────────────────────────────────────────────────────────────────────────────────┘
```

### 3.1 包划分

| 包 | 语言 | 职责 |
|---|---|---|
| `g1d_msgs` | IDL | `NavigateRelative.action`、`GuardState.msg`、`BodyLock.msg`、`LocalizationStatus.msg`、`LioHealth.msg`、`SaveMap.srv` 等 |
| `g1d_description` | xacro | G1-D URDF（下载自 HF `unitreerobotics/unitree_sim_isaaclab_usds`）+ `base_link` / `livox_frame` / `livox_imu` / `chassis_laser` |
| `g1d_core` | 纯 Python | 不依赖 ROS 的算法库：守护逻辑、栅格投影、目标合法性检查、free_sectors、延迟补偿、标定、验收计量、时钟映射。Windows / CI 都能测 |
| `g1d_robot_bridge` | Python | SLAMWARE 重打时间戳、关节 → `/joint_states`、姿态锁定、底盘 HTTP 事件 |
| `g1d_lidar` | C++ | 自身点过滤（CustomMsg → CustomMsg）、`/utlidar/*` 适配器 |
| `g1d_localization` | C++ / Python | `g1d_lio_bridge`（FAST_LIO → `odom → base_link`、障碍点云、健康监测）、定位模式切换、定位状态 |
| `g1d_mapping` | C++ / Python | 关键帧提取、闭环位姿图、地图保存（PCD + 栅格 + 元数据）、建图指标 |
| `g1d_nav_manager` | Python | NavGPT action、目标处理、失败原因、free_sectors、JSON 兼容层 |
| `g1d_guard` | Python | 守护节点（rclpy 包装 `g1d_core.guard`） |
| `g1d_sim` | C++ | 闭环仿真：盒子世界、Mid-360 射线 + IMU、差速底盘、宇树话题模拟 |
| `g1d_tools` | Python | 接口探测、指令实测、标定、录制预设、验收 T1–T6、指标 |
| `g1d_web` | Python + JS | 浏览器调试台（迁移自 `g1d_ws`） |
| `g1d_bringup` | launch / yaml | 启动文件、参数、RViz / Foxglove 配置、CycloneDDS 配置 |

## 4. 坐标系与时间

### 4.1 坐标系树

```text
map ─(重定位 / 单位变换)→ odom ─(g1d_lio_bridge)→ base_link ─(固定)→ AGV_link ─→ LZ_ot … LZ_it(升降)
   ─→ Pitching ─→ Yaw_Link(腰俯仰) ─→ torso_link(腰偏航) ─→ head_link ─(标定)→ livox_frame ─→ livox_imu
AGV_link ─(固定)→ chassis_laser（URDF Lidar_Link，xyz [0.1871, −0.0161, 0.1009]）
```

- `base_link`：底盘中心的地面投影，x 前、y 左。`base_link → AGV_link` 的高度 `base_to_agv_z`【待实测】。
- 关节由 `/joint_states` 驱动：`LZ_mt_Joint = LZ_it_Joint = h/2`，`h = lift_scale · hispeed_state.y + lift_offset`【scale / offset 待实测】；`Yaw_Joint` = 腰俯仰（`/waistpitchstate.q`）；`torso_Joint` = 腰偏航（`/lowstate.motor_state[12].q`）；双臂 15–28 号。
- `head_link → livox_frame` 初值 xyz `[0.05, 0, 0.55]`、rpy `[π, −0.04, 0]`（倒装）【待标定】；`livox_frame → livox_imu` = `(0.011, 0.02329, −0.04412)`、R = I（Livox 手册：IMU 在点云坐标系中的位置；即 FAST-LIO `mid360.yaml` 中 `extrinsic_T [−0.011, −0.02329, 0.04412]`（雷达在 IMU 系中的位置）取反）。

### 4.2 时间

**所有 ROS 时间戳统一为 5090 主机时钟。**

| 来源 | 处理 |
|---|---|
| SLAMWARE odom / scan | 时间戳比主机慢约 181 s 且每天漂移 →  按主机接收时间重打 |
| `lowstate` / `hispeed_state` / `waistpitchstate` | 无消息头 → 按主机接收时间 |
| Mid-360S | livox_ros_driver2（`2144554`）在雷达未同步时按主机 `high_resolution_clock::now()`（= system_clock）给每个包打戳，同步（PTP / gPTP，主时钟为主机）时用 PTP 时间 → 已是主机时钟，**无需补丁**（M3 阅读源码后修订；原计划的时钟映射取消）。网线连接时可选 PTP 以去掉接收抖动 |

因此不需要 PC2 与主机的时钟同步。

## 5. 模块设计

### 5.1 机器人接口 `g1d_robot_bridge`

- 订阅 `/slamware_ros_sdk_server_node/odom`、`/scan`（QoS best-effort，兼容 SDK 发布端），以接收时间重打时间戳，发布 `/chassis/odom`（`frame_id: wheel_odom`，`child: base_link`，只用于速度与一致性检查，不发 TF）和 `/chassis/scan`（`frame_id: chassis_laser`）。
- 订阅 `/hispeed_state`、`/lowstate`、`/waistpitchstate`，发布 `/joint_states`（限频 50 Hz）。
- 姿态锁定：腰偏航 ≈ 0、腰俯仰 ≈ `nav_pose.waist_pitch`、升降静止且高度不变、手臂 ≈ `arm_tuck`；容差腰 2°、手臂 5°、升降 1 cm / 0.005 m/s。发布 `/g1d/body_lock`（`g1d_msgs/BodyLock`：`locked`、`reasons[]`）。
- 底盘 HTTP 事件轮询（复用 `chassis_http.py`）：只认启动后新出现的 `BUMPER_TRIGGERED` / `CLIFF_DETECTED`，发布 `/g1d/chassis_event`。字段名【待实测】。

### 5.2 雷达与 LIO

**数据源**（启动参数 `lidar_source`）：

| 取值 | 链路 | 适用 |
|---|---|---|
| `livox` | livox_ros_driver2（Livox-SDK2 ≥ 1.3.1，配置设备键 `Mid360s`）→ `/livox/lidar`（CustomMsg）+ `/livox/imu` | 主机网线接入 `192.168.123.x`，雷达未被宇树服务占用 |
| `unitree` | `/utlidar/cloud_livox_mid360` + `/utlidar/imu_livox_mid360` → `g1d_lidar/utlidar_adapter` → CustomMsg | 雷达已被宇树服务占用【话题是否存在、是否带逐点时间待实测】 |

IMU 加速度单位为 g（FAST_LIO 按 g 处理）。

**自身点过滤** `g1d_lidar/self_filter`（C++，CustomMsg → CustomMsg，在 FAST_LIO 之前）：

- 躯干 + 头部：`head_link` 系固定盒子 `[−0.20, 0.20, −0.25, 0.25, −0.95, 0.10]`；
- 立柱 + 底盘：`base_link` 系盒子，每帧按 TF 变到雷达系；
- 收拢的手臂：`head_link` 系 `[−0.25, 0.40, −0.45, 0.45, −0.85, 0.05]`；
- 盒子参数【待实测】，启动自检：静止时车身附近不应有稳定点簇。

**FAST_LIO**（`hku-mars/FAST_LIO` ROS 2 分支，vendor 后打补丁：C++17、话题 / 坐标系参数化、时间戳兼容）。参数沿用 `ROS2_Reuse_Notes.md` 第 7 节。

**`g1d_lio_bridge`**（复用 `g1d_ws` `StateEstimator` 逻辑）：

- 首帧锚定：`T_odom_lio = T_base_imu(q_t0) · T_lio_imu(t0)⁻¹`，并用重力方向修正 roll / pitch，`odom` 严格重力对齐、原点在启动时 `base_link`。
- 每帧：`T_odom_base(t) = T_odom_lio · T_lio_imu(t) · T_imu_base(q_t)`，`T_imu_base` 从 TF（当前关节状态）查询；发布 TF `odom → base_link` 和 `/odometry`（速度取轮速）。
- 输出 `/g1d/obstacle_cloud`：FAST_LIO 去畸变后的当帧点云（`livox_imu` 系），供 Nav2 VoxelLayer（传感器原点 = 该坐标系原点）与守护使用。
- 健康监测（沿用阈值）：有效点 ≥ 300、单帧跳变 0.3 m / 15°、处理 ≤ 0.15 s、z 漂移 ≤ 0.3 m、退化指标；LIO 与轮速在 2 s 窗口内的一致性。发布 `/g1d/lio_health`。
- 轮速融合：`robot_localization` EKF 作为可选配置（默认关闭）。

### 5.3 建图 `g1d_mapping`

- 在线：FAST_LIO 运行；按 0.5 m / 10° 提取关键帧（位姿 + 去畸变点云）；闭环候选按位置邻近（半径 + 时间间隔）筛选，small_gicp 配准验证（fitness / 重叠率阈值）；GTSAM iSAM2 位姿图优化。优先复用 `liangheming/FASTLIO2_ROS2` 的 `pgo` 包（vendor 后适配话题）；如果接口不合适，按同样思路实现轻量版。
- `SaveMap` 服务输出目录 `maps/<name>/`：
  - `map.pcd`：优化后的全局点云（体素降采样）；
  - `map.yaml` + `map.pgm`：2D 栅格；
  - `keyframes/`、`trajectory.csv`：关键帧与优化轨迹；
  - `meta.yaml`：外参、建图时身体姿态、参数、闭环统计。
- **2D 栅格生成**（`g1d_core.grid`，纯 Python，复用 `g1d_web/mapping.py` 思路）：
  - 地面高度：按格估计（或全局平面），高度相对地面；
  - 占据：高度带 `[0.08, 1.85]` m 内有点（命中数 ≥ 阈值）；
  - 空闲：观测到地面且无占据；或由关键帧原点到点的 2D 射线经过且不在高度带内命中；
  - 未知：其余。
  - 输出 Nav2 `map_server` 标准格式。
- 建图指标：闭环前后的闭环残差、重复建图的地图间一致性（配准 RMSE）、栅格与仿真真值的 IoU（CI）。

### 5.4 定位

| 模式 | `map → odom` | 全局代价地图 |
|---|---|---|
| `map`（有先验地图） | `small_gicp_relocalization` 持续发布：累积 `/registered_scan`（`odom` 系）与 `map.pcd` 配准 | `StaticLayer`（`map.yaml`）+ 体素 / 底盘雷达 / 膨胀 |
| `odom`（无地图） | 单位变换（静态 TF） | 20 × 20 m 滚动窗口 |

- 初始位姿：RViz 2D Pose Estimate / Web 发 `/initialpose`；或 `auto`（KISS-Matcher 全局粗配准，成功后交给 small_gicp 跟踪）；或从 `maps/<name>/last_pose.yaml` 读取上次关机位置。
- 发布 `/g1d/localization_status`：`converged`、`fitness`、`map → odom` 跳变量、上次成功时间。连续失败超过阈值 → `localization_lost`。
- 指标：仿真中从随机偏离初值（±1 m、±30°）的收敛率与时间；真机回到标记点的重复精度。

### 5.5 导航

**代价地图**：

| 参数 | local | global |
|---|---|---|
| 尺寸 / 分辨率 | 6 × 6 m / 0.05 m，滚动 | 静态地图尺寸（`map` 模式）或 20 × 20 m 滚动 / 0.1 m |
| 图层 | VoxelLayer + ObstacleLayer + InflationLayer | StaticLayer（`map` 模式）+ VoxelLayer + ObstacleLayer + InflationLayer |

- VoxelLayer：`/g1d/obstacle_cloud`，`origin_z 0.08`，`z_resolution 0.12`，`z_voxels 15`（到约 1.88 m），`min/max_obstacle_height 0.08 / 1.85`，标记 5 m、清除 6 m，`mark_threshold 0`。
- ObstacleLayer：`/chassis/scan`，标记 4 m、清除 5 m。
- Inflation：半径 0.55 m，`cost_scaling_factor 3.0`；footprint `[[0.30, 0.28], [0.30, −0.28], [−0.30, −0.28], [−0.30, 0.28]]`。
- **悬空障碍保留**是 CI 集成测试项：仿真桌子（桌面 0.75 m、底盘可从桌下看到地面），多方向接近，桌面格子不能被清除。不通过时的退路依次为：STVL；把 `g1d_ws` 的列位掩码体素层改写成 Nav2 插件。

**规划与控制**：

- 全局规划：NavFn（`allow_unknown true`、`tolerance 0.3`），1 Hz 重规划（BT）。
- 局部控制：MPPI（参数沿用 Notes 第 7 节，换成 Jazzy 参数名），外包 RotationShimController（起步转向、到达后朝向对齐）。到达容差 0.2 m / 0.25 rad。
- 进展检查：8 s 内 < 0.2 m → 恢复（wait → clear → 小角度原地转）；恢复用尽 → 失败。
- 控制器输出 remap 到 `/nav/cmd_vel`；不启用 Nav2 `collision_monitor` / `velocity_smoother`（守护负责）。

**`g1d_nav_manager`**：

- Action `g1d_msgs/action/NavigateRelative`：

  | 字段 | 说明 |
  |---|---|
  | `id` | 目标标识 |
  | `mode` | `navigate` / `rotate` / `simple` / `stop` |
  | `frame` | `base_at_stamp`（默认，相对 `stamp` 时刻车体）/ `odom` / `map` |
  | `stamp` | 图像时刻（主机时间）；超过 5 s 拒绝 |
  | `x, y, yaw, use_yaw` | 目标 |
  | `xy_tolerance, yaw_tolerance, max_speed, timeout_s` | 可选 |

  Result：`state`、`reason`、`partial`、`adjusted`、`final_pose`、`free_sectors[]`；Feedback：`distance_remaining`、`yaw_error`、`pose`、`guard_blocked`。
- 延迟补偿：TF 缓存保留 10 s，查询 `stamp` 时刻 `odom → base_link`，把相对目标换算到 `odom`（再按需到 `map`）。
- 目标合法性（复用 `g1d_ws` `sanitizeGoal`，用全局代价地图）：超过 9 m 截断（`partial`）；在障碍内沿目标→机器人方向回退或 0.5 m 半径搜索 `cost < 128` 的点（`adjusted`）；仍找不到 → `goal_in_obstacle`；地图外 → `goal_out_of_range`。
- 前置与运行中检查：姿态锁定、LIO 健康、定位状态、守护急停 / 人工接管。
- `reason` 枚举沿用 `g1d_ws`：`goal_in_obstacle`、`goal_out_of_range`、`goal_stamp_unavailable`、`no_path`、`blocked`、`timeout`、`lio_lost`、`localization_lost`、`body_not_locked`、`body_moved`、`guard_estop`、`manual_override`、`sensor_stale`、`yaw_not_reached`（SUCCEEDED）。Nav2 错误码映射到上述枚举。
- `free_sectors`：−90°…+90° 每 30°，前方 1.5 m 内是否可通行（局部代价地图 + footprint 扫掠）。
- 兼容层（可选启动）：`/nav/goal`（String JSON）→ action，action 状态 → `/nav/status`（String JSON），格式与 `g1d_ws` 相同（DDS 上即 `rt/nav/goal` / `rt/nav/status`），旧 NavGPT 代码无需修改。
- Python 客户端库 `g1d_nav_manager.client.NavClient`：`send_relative(dx, dy, dyaw, stamp)`、`wait()`、`cancel()`。

### 5.6 守护 `g1d_guard`

- `g1d_core.guard`：原样迁移 `guard_core.py`（扫掠路径反射层、爬行速度下限、旋转圆、后退保护、急停锁存、遥控接管、加速度限制、发布策略）及其单测。
- 节点输入：`/nav/cmd_vel`（0.3 s 超时）、`/g1d/teleop_cmd`（Web / 手柄 deadman 0.3 s，优先于导航）、`/chassis/scan`、`/g1d/obstacle_cloud`（变换到 `base_link`）、`/wirelesscontroller`、`/g1d/chassis_event`、`/g1d/estop`（`std_msgs/Bool`，锁存）。
- 输出：`/cmd_vel_no_limit`（`geometry_msgs/Twist`，QoS 与 SDK 订阅端兼容【待实测】）、`/g1d/guard_state`（10 Hz）。
- 参数沿用 Notes：30 Hz；vx `[−0.10, 0.50]`、wz 0.60；加速 0.5 / 减速 1.5 m/s²、角加速度 1.0 rad/s²；`stop_m 0.45`、`slow_m 1.2`、`creep_v 0.05`、`rotate_radius 0.38`、`rear_clear_m 0.5`；`zero_hold 1.0 s`【待实测】。
- 启动检查：ROS 图中 `/cmd_vel_no_limit` 若已有其他发布者（含非 ROS 的 DDS 写入者）→ 拒绝启动（参数可覆盖）。
- 启动动作：`DELETE /api/core/motion/v1/actions/:current`（中止 SLAMWARE 任务）。

### 5.7 可视化与操作

- RViz2 配置：TF、机器人模型、点云（原始 / 障碍 / 地图）、2D 栅格、代价地图、全局 / 局部路径、footprint、MPPI 轨迹、2D Pose Estimate、2D Goal。
- Foxglove：`foxglove_bridge` 启动 + 布局文件。
- `g1d_web`（M8）：迁移 `g1d_ws/python/g1d_web`，后端 hub 改用 rclpy，前端（three.js）尽量保持；功能：点云 / 地图 / 定位 / 路径 / 代价地图显示、点选目标、给初始位姿、多航点任务、遥控、急停、T1–T6 向导、参数、地图保存与加载。

### 5.8 工具

| 工具 | 内容 |
|---|---|
| `check_interfaces` | 列出机器人与本工程话题：频率、时间戳偏差、QoS、类型、样本；`/cmd_vel_no_limit` 写入者；底盘 HTTP 连通性 |
| `cmd_vel_probe` | 停掉其他写入者后实测：停发后停车时间、限幅、零速保持 / 松开、锁轮（迁移 `g1d_ws`） |
| `calib_floor` | 地面 RANSAC 求雷达 roll / pitch / 高度；与底盘雷达对齐求 yaw；升降比例 / 偏置；输出 xacro 参数 |
| 录制 | rosbag2 预设：`mapping`（雷达 / IMU / 关节 / 底盘）、`nav`（+ 代价地图 / 路径 / 守护）、`debug`（全部） |
| 验收 | T1–T6 计量（迁移 `acceptance.py`）+ 建图指标 + 重定位指标，生成报告（Markdown + CSV） |

## 6. 安全设计

### 6.1 分层

| 层 | 机制 | 兜底 |
|---|---|---|
| L1 | 代价地图 + MPPI（整车轮廓、膨胀） | L2 |
| L2 | 守护反射层（扫掠路径 0.45 m 停车 / 1.2 m 减速、后退、旋转圆） | L3 |
| L3 | 守护看门狗（指令 0.3 s、传感器 0.5 s 超时） | L4 |
| L4 | 急停锁存（话题、底盘碰撞 / 悬崖事件、Web 急停） | L5 |
| L5 | 底盘自身指令超时、AEB、硬件急停 | — |

### 6.2 失效处理与放行门槛

| 失效 | 处理 | 停车时间 |
|---|---|---|
| Nav2 / manager 崩溃 | 守护 `cmd_timeout` 0.25 s 无指令 → 按 1.5 m/s² 减速；守护限速 `vx_max` = 0.40 m/s（= MPPI 上限） | 0.25 + 0.40 / 1.5 ≈ 0.52 s ≤ 0.6 s（CI 仿真验证）。减速度不加大：重心约 1 m、底盘半长约 0.28 m，前倾临界约 2.7 m/s² |
| 守护 / 主机崩溃、网络断开 | 依赖底盘对 `cmd_vel_no_limit` 的自身超时 | **【待实测】放行门槛：第 0 阶段测得停车时间 ≤ 0.6 s 才允许自主导航。不达标时再评估在 PC2 上加极简看门狗** |
| 雷达 / 底盘雷达断流 | 守护传感器超时，只允许停车 | ≤ 0.5 s + 刹车 |
| LIO 发散 / 重定位丢失 | manager 返回 `lio_lost` / `localization_lost` | 立即 |
| 身体姿态变化 | `body_moved` | 立即 |
| 碰撞 / 悬崖 / 软急停 | 锁存，人工解除 | 立即 |

建议：自主导航用网线连接；WiFi 只用于监控和低速遥控（守护可按连接类型降低限速）。

## 7. 验证方案

### 7.1 Windows（本机）

- `src/g1d_core` 的 pytest：守护逻辑（迁移的 17 项 + 新增）、栅格投影、目标合法性、free_sectors、延迟补偿、标定、验收计量、时钟映射。
- 文本检查：LF、无 BOM、可执行位（`git ls-files --eol`）。

### 7.2 GitHub Actions

| Job | 内容 |
|---|---|
| `core-tests` | ubuntu + windows，Python 3.12：`g1d_core` pytest |
| `ros-build` | `ros:jazzy` 容器：rosdep、Livox-SDK2、源码依赖（small_gicp、KISS-Matcher 等）、`colcon build --packages-up-to g1d_bringup`、`colcon test`（单测、flake8 / xmllint 等） |
| `ros-integration` | 基于 `ros-build` 产物，`launch_testing` 闭环场景（见下表），产物（地图、轨迹图、rosbag）上传为 artifact |

**闭环仿真 `g1d_sim`**（由 `g1d_ws/src/sim` 改写为 ROS 2 节点）：盒子世界、Mid-360 非重复扫描射线 + IMU（200 Hz）、差速底盘（响应 `/cmd_vel_no_limit`、0.5 s 超时）、SLAMWARE odom / scan（时间戳滞后 181 s）、宇树关节话题（`unitree_hg` 类型）、遥控器、HTTP 事件 mock。

| 场景 | 通过标准 |
|---|---|
| LIO 精度 | 闭合轨迹终点误差 < 0.10 m / 2° |
| 建图 → 保存 → 重定位 | 地图生成；从 ±1 m / ±30° 初值收敛，误差 < 0.1 m / 2° |
| 绕障到达 | 到达、无碰撞（整机 0–1.75 m） |
| 桌面悬空障碍 | 不碰撞，桌面格子保留 |
| 目标在障碍内 | `adjusted=true` 到达或 `goal_in_obstacle` |
| 杀掉导航进程 | 底盘 ≤ 0.6 s 停止 |
| 急停 / 人工接管 | 立即停止，状态正确 |
| NavGPT 相对航点 + 延迟 | 0.5 s 前的 stamp，到达点误差 < 0.2 m |

## 8. 仓库组织

```text
g1d_ros2_ws/               # git 仓库 = colcon 工作区，.gitattributes 强制 LF
├── PLAN.md  README.md  docs/
├── src/
│   ├── g1d_msgs  g1d_description  g1d_bringup  g1d_core  g1d_robot_bridge  g1d_lidar
│   ├── g1d_localization  g1d_mapping  g1d_nav_manager  g1d_guard  g1d_sim  g1d_tools  g1d_web
│   ├── external/          # 未修改的上游（git submodule）：unitree_ros2、small_gicp、KISS-Matcher、Livox-SDK2 …
│   └── vendor/            # 要修改的上游：先原样提交（注明 commit），再提交补丁；UPSTREAM.md 记录来源、许可证、补丁列表
├── maps/                  # 运行时地图（不入库）
├── _upstream/             # 本地克隆（不入库）
└── .github/workflows/
```

- `unitree_ros2` 中的示例包不参与编译：统一用 `colcon build --packages-up-to g1d_bringup`。
- 提交粒度小、信息清楚；vendor 补丁提交信息以 `vendor(<name>):` 开头。

## 9. 阶段计划

| 阶段 | 内容 | 验证 |
|---|---|---|
| M0 | 仓库骨架、CI、上游克隆、PLAN.md、推送 GitHub | CI 跑通 |
| M1 | `g1d_msgs`、`g1d_description`、`g1d_robot_bridge`、`g1d_sim`（底盘部分）、`check_interfaces` / `cmd_vel_probe` | TF 正确、时间戳已重打 |
| M2 | `g1d_core.guard` + `g1d_guard` + 遥控 + 急停 | 单测；杀导航 ≤ 0.6 s 停车 |
| M3 | Livox 驱动补丁、自身点过滤、FAST_LIO、`g1d_lio_bridge`、`g1d_sim` 雷达 / IMU | LIO 精度 |
| M4 | 建图 + 闭环 + 2D 栅格 + 建图指标 | 闭环残差、栅格 IoU |
| M5 | 重定位（手动 / 自动初值）+ 纯里程计模式 | 收敛率、精度 |
| M6 | Nav2 配置 + `g1d_nav_manager` + NavGPT action / 兼容层 | 绕障、悬空障碍、目标外移、延迟补偿 |
| M7 | RViz / Foxglove、录制、标定、T1–T6 与指标工具 | 工具单测 |
| M8 | `g1d_web` 迁移 | 端到端测试 |
| M9 | 文档：README、项目介绍、移植与参数、标定、故障排查 | — |

## 10. 待实测项（真机第 0 阶段）

| # | 事项 | 方法 |
|---|---|---|
| Q1 | `cmd_vel_no_limit` 停发后停车时间（放行门槛）、限幅、零速保持 / 锁轮、与遥控器 / SLAMWARE 的优先级、AEB 是否生效 | `g1d_tools cmd_vel_probe` |
| Q2 | Jazzy（CycloneDDS 0.10.x）与宇树 SDK2（0.10.2）逐个话题互通、QoS | `check_interfaces` |
| Q3 | Mid-360S 是否被宇树服务占用、IP、`/utlidar/*` 是否存在及字段 | `check_interfaces` + Livox Viewer |
| Q4 | `head_to_livox`、`base_to_agv_z`、升降比例 / 偏置、自身点盒子、导航姿态关节值 | `calib_floor` + 测量 |
| Q5 | 底盘 HTTP 事件 JSON 格式 | `check_interfaces` |
| Q6 | WiFi 下 `lowstate` 等高频话题的带宽与延迟 | `check_interfaces` |
| Q7 | `lowstate` 关节序号、方向与 URDF 一致性 | 逐关节转动 |

## 11. 风险

| 风险 | 对策 |
|---|---|
| 底盘自身超时 > 0.6 s | 放行门槛；PC2 极简看门狗作为备选 |
| Jazzy 与宇树 DDS 不互通 | 第 0 阶段先测；退路：自编译 CycloneDDS 0.10.2 版 rmw，或 Humble |
| Nav2 VoxelLayer 误清除悬空障碍 | CI 场景测试；退路 STVL / 自研插件 |
| Mid-360S 被占用且 `/utlidar/*` 无逐点时间 | 关闭去畸变并降速 |
| WiFi 抖动 | 自主导航用网线；守护按连接类型限速 |
| CI 中仿真负载过高、时序不稳定 | 降低点数 / 频率；关键断言留余量；失败重跑一次 |
| 长走廊等几何退化 | 退化指标 + 轮速一致性检查；可选 EKF |
| FAST_LIO / ikd-Tree 为 GPL-2.0 | 内部使用；对外分发前按 GPL 处理 |
