# g1d_ros2_ws 项目详细介绍

## 1. 背景与目标

宇树 G1-D 是"人形上身 + 差速轮式底盘"的机器人：底盘只能通过 DDS 话题 `rt/cmd_vel_no_limit` 控制（无限幅、无避障），底盘单线雷达看不到桌沿等悬空障碍。本工程用头部 **Livox Mid-360S**（倒装）在 ROS 2 Jazzy 上实现：

1. **建图**：FAST_LIO 激光惯性里程计 + 关键帧位姿图闭环（GTSAM），保存三维点云地图与高度带 2D 栅格（能表示悬空障碍）；
2. **定位**：先验地图上 small_gicp 持续重定位（`map → odom`），初始位姿可由 RViz / Web 给定、自动全局搜索或使用上次位姿；也支持无地图的纯里程计模式；
3. **导航避障**：Nav2（NavFn + Regulated Pure Pursuit；MPPI 保留为可选项），代价地图含三维体素层、底盘雷达层与膨胀；目标在障碍内自动外移（优先机器人一侧），失败给出原因与可通行方向；
4. **安全**：独立守护 `g1d_guard` 是 `/cmd_vel_no_limit` 唯一写入者（限幅、加减速、看门狗、扫掠路径反射停车、急停锁存、遥控接管）；
5. **NavGPT 接入**：相对航点 + 图像时刻 `stamp`，用 TF 历史补偿推理延迟；
6. **可视化与工具**：RViz2 / Foxglove 配置、浏览器调试台、接口探测、底盘指令实测、外参标定、rosbag2 预设、验收 T1–T7 与建图 / 重定位指标。

## 2. 部署形态

```text
┌──────────── PC2（Ubuntu 20.04，宇树服务，不安装任何东西）────────────┐
│ SLAMWARE: /slamware_ros_sdk_server_node/odom、/scan（时间戳慢约 181 s） │
│ 本体: /hispeed_state  /lowstate  /waistpitchstate  /wirelesscontroller   │
│ 底盘: /cmd_vel_no_limit（只接受 g1d_guard 写入）     [可选] /utlidar/*   │
└───────────────────────────── DDS 域 0（CycloneDDS 0.10.x）─────────────┘
        ▲ 网线（自主导航）/ WiFi（监控）                Mid-360S UDP（网线时直连）
┌──────────── 导航主机（Ubuntu 24.04 + ROS 2 Jazzy）──────────────────────┐
│ robot_interface: URDF/TF、chassis_restamp、body_state、chassis_events、g1d_guard │
│ lio: livox 驱动 | utlidar_adapter | lidar_sim → self_filter → FAST_LIO → lio_bridge │
│ mapping: pgo + map_manager      localization: small_gicp + localization_monitor   │
│ navigation: Nav2 + nav_manager  web / RViz / Foxglove                              │
└──────────────────────────────────────────────────────────────────────────┘
```

宇树 SDK2 话题按 ROS 2 命名规则（`rt/xxx` = `/xxx`，`pkg::msg::dds_::Type_`），ROS 2 节点直接订阅 / 发布，无需桥接。

## 3. 坐标系与时间

```text
map ─(small_gicp / 单位变换)→ odom ─(lio_bridge)→ base_link ─→ AGV_link ─→ 升降两级 ─→ Pitching
  ─→ Yaw_Link（腰俯仰）─→ torso_link（腰偏航）─→ head_link ─(标定)→ livox_frame ─→ livox_imu
AGV_link ─→ Lidar_Link ─→ chassis_laser
```

- `odom`：LIO 启动时 `base_link` 的位置（地面 z = 0），重力对齐依赖 URDF + 外参；
- 关节由 `/joint_states` 驱动（升降两段各取一半、腰俯仰、腰偏航、双臂 14 关节），头部雷达随身体运动的位姿由 `robot_state_publisher` 给出；导航期间要求姿态锁定；
- **时间**：所有 ROS 时间戳都是导航主机时钟。SLAMWARE 时间戳不可信 → 按接收时间重打；宇树关节话题无消息头 → 接收时间；livox_ros_driver2 在雷达未同步时本来就按主机时钟给每个包打戳。因此不需要 PC2 与主机时钟同步。

## 4. 包与节点

| 包 | 节点 / 内容 |
|---|---|
| `g1d_core` | 不依赖 ROS 的算法：守护逻辑、URDF 正运动学、姿态锁定、LIO 锚定 / 外推 / 健康、PCD、栅格、全局重定位搜索、目标处理、free_sectors、标定、验收 T1–T7、地图一致性、仿真物理 |
| `g1d_msgs` | `GuardState`、`BodyLock`、`LioHealth`、`LocalizationStatus`、`SaveMap.srv`、`Relocalize.srv`、`NavigateRelative.action` |
| `g1d_description` | 宇树 URDF（HF 数据集，原样 + mesh 路径补丁）+ xacro 扩展坐标系 + `extrinsics.yaml` |
| `g1d_robot_bridge` | `chassis_restamp`、`body_state`、`chassis_events` |
| `g1d_guard` | `guard`（唯一写入 `/cmd_vel_no_limit`） |
| `g1d_lidar` | `self_filter`（C++）、`utlidar_adapter`（C++） |
| `g1d_localization` | `lio_bridge`、`localization_monitor` |
| `g1d_mapping` | `map_manager` |
| `g1d_nav_manager` | `nav_manager`、Python 客户端 `NavClient` |
| `g1d_sim` / `g1d_lidar_sim` | 仿真底盘（宇树话题、滞后时间戳、HTTP 事件）/ Mid-360 + IMU 仿真（C++） |
| `g1d_tools` | `check_interfaces`、`cmd_vel_probe`、`calib_extrinsics`、`acceptance`、`map_compare`、`nav_cli` |
| `g1d_web` | 浏览器调试台（rclpy hub + three.js 前端） |
| `g1d_bringup` | 启动文件、参数、RViz / Foxglove、CycloneDDS 配置 |
| `g1d_tests` | 闭环集成测试（launch_testing） |

## 5. 数据流

| 话题 | 类型 | 发布者 → 使用者 |
|---|---|---|
| `/chassis/odom`、`/chassis/scan` | Odometry、LaserScan | chassis_restamp → lio_bridge（外推 / 一致性）、守护、代价地图 |
| `/joint_states`、`/g1d/body_lock` | JointState、BodyLock | body_state → robot_state_publisher、nav_manager |
| `/livox/lidar_raw` → `/livox/lidar`、`/livox/imu` | Livox CustomMsg、Imu | 驱动 / 适配 / 仿真 → self_filter → FAST_LIO |
| `/Odometry` | Odometry（`lio_world → livox_imu`） | FAST_LIO → lio_bridge、pgo |
| `/g1d/obstacle_cloud` | PointCloud2（`livox_imu`，去畸变当帧） | FAST_LIO → 代价地图体素层、守护、pgo |
| TF `odom → base_link`、`/odometry`、`/g1d/lio_health`、`/g1d/lio_anchor` | | lio_bridge（50 Hz，LIO + 轮速外推） |
| `/g1d/registered_scan` | PointCloud2（odom） | lio_bridge → small_gicp、localization_monitor |
| TF `map → odom`、`/g1d/localization_status` | | small_gicp_relocalization、localization_monitor |
| `/nav/cmd_vel` | Twist | Nav2 控制器 / 行为 → 守护 |
| `/g1d/teleop_cmd`、`/g1d/estop` | Twist、Bool | Web / 工具 → 守护 |
| `/cmd_vel_no_limit` | Twist | **仅守护** → 底盘 |
| `/g1d/guard_state` | GuardState | 守护 → nav_manager、Web |
| `/g1d/navigate_relative` | NavigateRelative action | NavGPT / Web / 工具 → nav_manager → Nav2 |
| `/nav/goal`、`/nav/status` | String（JSON，与 g1d_ws 相同） | 兼容层 |

## 6. 建图与定位

- **建图**：pgo 按 0.5 m / 10° 取关键帧，按位置邻近（1 m 内、30 s 以前）找闭环候选，PCL ICP 验证（fitness < 0.15），GTSAM iSAM2 优化。`/g1d/save_map` 导出后由 `map_manager` 换算到 map 系（= 建图会话的 odom 系），输出 `map.pcd`（5 cm 体素）、`map.yaml/pgm`（占据：高度带 0.08–1.85 m 内有 ≥ 2 点；空闲：看到地面且无占据，向未知扩一格）、`keyframes/`、`trajectory.csv`、`meta.yaml`。
- **重定位**：small_gicp 每 0.5 s 把最近 5 帧 odom 系扫描与地图配准，`map → odom` 以 20 Hz 按当前时钟发布（配准在独立线程，不阻塞发布）。连续 4 次配准失败或 10 s 无结果才判定定位丢失；配准变慢只给出 `registration_slow` 提示。自动初始化 `g1d_core.reloc`：以建图轨迹为种子（0.5 m 间隔、±0.25 m、10° 步长），按"扫描点落在地图占据体素中的比例"评分，粗到细爬山，结果作为 `/initialpose` 交给 small_gicp；最好与次好的明显不同解得分接近时会报告歧义。
- **纯里程计模式**：`map → odom` 为静态单位变换，全局代价地图改为 20 m 滚动窗口。

## 7. 导航

- 代价地图：体素层（Mid-360S 去畸变点云，原点 = `livox_imu`，0.08–1.88 m 共 15 层，三维射线清除——只清除射线真正穿过的高度，桌面不会被地面方向的射线清除）、障碍层（底盘雷达）、膨胀 0.55 m；全局地图在 map 模式下加静态层。
- 规划 NavFn（1 Hz 重规划），控制 Regulated Pure Pursuit（0.35 m/s，前视 0.3–0.9 m，夹角 > 45° 先原地转向，曲率 / 碰撞前瞻降速），进展检查 8 s / 0.2 m，恢复 spin / backup / wait。
- MPPI（RotationShim + MPPI，参数沿用 g1d_ws）在 `config/nav2_mppi.yaml`，`controller:=mppi` 启用。沿用的 `regenerate_noises: false` 会让整个运行期复用一组噪声样本，CI 仿真中表现为持续偏转、平均约 0.05 m/s、不跟随路径；改为 `true` 后导航用例全部通过（绕箱 26.9 s，RPP 19.9 s）。对照实验同时排除了 CPU、控制频率、里程计速度、RotationShim、路径分辨率、路径类 critics 和 footprint 碰撞检查。
- 默认控制器为 RPP（CI 两轮验证）；MPPI 修正后只验证过一轮，上机时两者都可以试，以真机验收为准。
- `nav_manager`：模式 navigate / rotate（Spin）/ simple（Spin + DriveOnHeading）/ stop；坐标系 base_at_stamp（TF 历史 15 s，超过 5 s 拒绝）/ base / odom / map；目标处理（9 m 截断 `partial`、障碍内先沿目标→机器人回退，再在 1.0 m 内找"离目标近且偏向机器人一侧"的可行点 `adjusted`、地图外 `goal_out_of_range`）；运行中检查姿态锁定、LIO、定位、守护急停 / 人工接管、超时；Nav2 错误码映射为 `no_path` / `blocked` / `goal_in_obstacle` 等；失败附 7 个方向的 free_sectors；新目标抢占旧目标时不停车。

## 8. 安全

| 层 | 机制 | 兜底 |
|---|---|---|
| L1 | 代价地图 + 控制器（RPP 碰撞前瞻） | L2 |
| L2 | 守护反射层：整车矩形沿指令圆弧扫掠，0.45 m 停车 / 1.2 m 减速（爬行下限 0.05 m/s）、后退保护、旋转圆 0.38 m | L3 |
| L3 | 守护看门狗：导航指令 0.25 s、传感器 0.5 s 超时；减速度 1.5 m/s² | L4 |
| L4 | 急停锁存：`/g1d/estop`、底盘碰撞 / 悬崖事件（只认启动后新事件）、Web 急停 | L5 |
| L5 | 底盘自身指令超时、AEB、硬件急停 | — |

- 停车时间：`vx_max 0.40`、`cmd_timeout 0.25`、减速 1.5 m/s² → 0.25 + 0.27 ≈ 0.52 s（CI 实测 0.49 s 指令归零、0.50–0.52 s 仿真底盘停住）。减速度不加大：重心约 1 m、底盘半长约 0.28 m，前倾临界约 2.7 m/s²。
- 守护运行在导航主机上：主机 / 网络故障时依赖底盘自身超时 → 第 0 阶段 `cmd_vel_probe` 的放行门槛（≤ 0.6 s）。
- 守护启动时若 `/cmd_vel_no_limit` 已有其他发布者（含非 ROS 的 DDS 写入者）则拒绝启动。

## 9. 验证

本机（Windows，无 WSL2 / Docker）运行 `g1d_core` 与部分包的 pytest；GitHub Actions 在 `ros:jazzy` 容器中构建全部包并运行单测和闭环集成测试（`g1d_tests`，`g1d_sim` + `g1d_lidar_sim`）：

| 闭环测试 | 结果（CI 实测） |
|---|---|
| 导航停发后停车 | 指令 0.49 s 归零，仿真底盘 0.50–0.52 s 停住（要求 ≤ 0.6 s） |
| 急停、遥控器接管、TF 链、时间戳重打、姿态锁定 | 通过 |
| LIO 约 8 m 方形闭环 | 位置误差 0.077–0.088 m、航向 0.02–0.07°（要求 < 0.10 m / 2°）；自身点漏检 0 |
| 建图保存 | 约 10 万点、46–48 关键帧；点到真实表面距离中位数 3–4 cm、P90 9 cm；栅格占据精度 0.89–0.996 |
| 重定位 | 自动初始化误差 0.01–0.05 m；行驶中跟踪最大误差 0.01–0.04 m；错误初值（0.4 m / 15°）3–8 s 内重新收敛到 < 0.02 m |
| 先验地图上导航（部署方式：map 模式定位 + Nav2 map 坐标系） | 从地图原点外上电，自动重定位 39 s；map 坐标系目标绕箱 13 s 到达；静态障碍内的目标外移到机器人一侧；地图外目标返回 `goal_out_of_range`；全程保持定位（内点率 0.99–1.0） |
| 现场工具 launch | `record.launch.py` 录 MCAP 并在退出时正常收尾；`web.launch.py` 页面可访问；`viz.launch.py` 的 foxglove_bridge 端口可连 |
| 导航（RPP） | 绕箱到 6 m 外 19.9 s；目标在箱内 → 外移到机器人一侧；转 90° 后按旧图像时间戳的目标仍到达正确位置（延迟补偿）；桌前到达、桌下目标拒绝（`goal_in_obstacle`）；不可达目标返回原因与 7 个方向的 free_sectors；全程无碰撞，离障碍最近 0.35 m |

仿真不等于真机：雷达点云、IMU、底盘动力学都是理想化模型，最终以真机验收 T1–T7 为准。

## 10. 开源代码复用

| 上游 | 版本 | 方式 |
|---|---|---|
| unitree_ros2 | `668d1ec` | 子模块（只编译消息包） |
| Livox-SDK2 | `c0796f0` | 子模块（`third_party/`，系统安装） |
| small_gicp | `fa0cfc9` | 子模块 |
| livox_ros_driver2 | `2144554` | vendor：原样提交 → colcon 构建补丁 |
| FAST_LIO（ROS2 分支）+ ikd-Tree | `cc3c9e8` / `e2e3f4e` | vendor：C++17、坐标系参数、TF 开关、`fclose` 保护 |
| FASTLIO2_ROS2 `pgo` / `interface` | `7baa59f` | vendor：rosdep 键、TF 开关 |
| small_gicp_relocalization | `81c9bd3` | vendor：C++17、状态话题、依赖与链接、初始位姿时间戳 |
| G1-D URDF（HF `unitree_sim_isaaclab_usds`） | `27b13d3` | vendor：mesh 路径 |
| g1d_ws（非 ROS 版本） | `068e6dd` | 守护逻辑、底盘 HTTP、标定、验收、Web 调试台原样导入后适配 |
| Nav2、robot_state_publisher、xacro、tf2 等 | Jazzy apt | 依赖 |

每个 vendor 目录的 `UPSTREAM.md` 记录来源、许可与补丁清单；补丁都是独立提交（`vendor(<name>): …`）。上游克隆在 `_upstream/`（`scripts/clone_upstream.py`、`upstream.lock`）。

## 11. 已知限制

- 导航期间身体（腰、升降、手臂）必须保持导航姿态；动态补偿未做。
- 动态行人只通过体素射线清除和守护反射层处理，没有运动预测。
- 长走廊、空旷大厅等几何退化场景 LIO 可能失效（健康监测会报告并停止导航）。
- `utlidar_adapter` 的字段识别基于常见命名，`/utlidar/*` 的实际格式待实测。
- 自动重定位依赖机器人位于建图轨迹附近；完全陌生的位置需要手动给初值。
- 单层平面场景；楼梯、坡道、多楼层未覆盖。
