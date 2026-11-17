# M0–M2 基础（仓库 / CI、接口与 TF、安全守护）实施计划

> **执行方式：** 由同一会话按任务顺序内联执行（TDD：先写失败测试 → 实现 → 测试通过 → 提交）。每个任务完成后更新 `PLAN.md`。代码以仓库为准，本计划给出文件、接口、测试用例与验证命令。

**目标：** 搭好可在 CI 中构建测试的 ROS 2 Jazzy 工作区；完成机器人接口（重打时间戳、关节 → TF、姿态锁定、底盘事件）、G1-D 描述、闭环仿真底盘和独立安全守护，并证明"导航停发后 ≤ 0.6 s 停车"。

**架构：** 不依赖 ROS 的算法放在纯 Python 包 `g1d_core`（Windows + CI 都测）；ROS 节点是薄包装（`g1d_robot_bridge`、`g1d_guard`、`g1d_sim`、`g1d_tools`）；集成测试在 `g1d_tests` 中用 launch_testing 跑闭环。

**技术栈：** ROS 2 Jazzy（rclpy、robot_state_publisher、xacro、tf2_ros、sensor_msgs_py、launch_testing）、rmw_cyclonedds_cpp、numpy、pytest、GitHub Actions（`ros:jazzy` 容器）。

## 全局约束

- ROS 2 Jazzy / Ubuntu 24.04 / Python 3.12；`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；DDS 域 0。
- `/cmd_vel_no_limit` 只由 `g1d_guard` 写入。
- 所有 ROS 时间戳为主机时钟；SLAMWARE odom / scan 按接收时间重打。
- 订阅宇树 / SLAMWARE 话题用 best-effort QoS；发布 `/cmd_vel_no_limit` 用 reliable、volatile。
- `g1d_core` 不得 import 任何 ROS 包。
- 文本文件 LF；上游：未修改用 submodule（`src/external/`），修改的先原样提交再打补丁（`UPSTREAM.md`）。
- 守护参数沿用 `g1d_ws`；部署配置覆盖 `cmd_timeout: 0.25`、`brake_v: 2.0`（保证 0.5 m/s 下 0.25 + 0.25 = 0.5 s 指令归零）。

---

### Task 1：仓库骨架、上游克隆脚本、unitree_ros2 子模块、CI、GitHub 仓库

**Files:**
- Create: `README.md`（简介占位，M9 完善）、`scripts/clone_upstream.py`、`upstream.lock`、`.github/workflows/ci.yml`、`src/external/unitree_ros2`（submodule @ `668d1ec`）、`maps/README.md`

**内容：**
- `scripts/clone_upstream.py`：读取 `upstream.lock`（每行 `name url ref`），克隆 / 更新到 `_upstream/<name>`，打印实际 commit；幂等。
- CI 三个 job：`core-tests`（ubuntu + windows，Python 3.12，`pip install -e src/g1d_core[test]`，`pytest src/g1d_core/test`）、`ros`（`ros:jazzy` 容器，submodules recursive，apt 依赖，`rosdep install`，`colcon build --packages-up-to g1d_bringup g1d_tests g1d_tools`，`colcon test`，`colcon test-result --verbose`），`lint-eol`（检查 LF / 无 BOM）。
- 用 Git Credential Manager 中的凭据调用 GitHub API 创建公开仓库 `hitliujianyun/g1d_ros2_ws`，`git push -u origin main`。

**验证：** `git submodule status` 显示 `668d1ec`；推送成功；CI 在后续任务产生包后变绿。

**提交：** `chore: workspace skeleton, upstream clone script, unitree_ros2 submodule, CI`

---

### Task 2：`g1d_core` 包 + 守护逻辑迁移

**Files:**
- Create: `src/g1d_core/{package.xml, setup.py, setup.cfg, resource/g1d_core, g1d_core/__init__.py, g1d_core/guard.py, g1d_core/chassis_http.py}`
- Test: `src/g1d_core/test/test_guard.py`、`test/test_chassis_http.py`

**Interfaces (Produces):**
- `g1d_core.guard`：`GuardParams`（dataclass，字段同 `g1d_ws`，新增 `from_dict(d)`）、`GuardCore(params, now)`：`on_cmd(v,w,now)`、`on_scan_points(pts,now)`、`on_cloud_points(pts,now)`、`on_estop(on,reason)`、`on_event(type)`、`on_joystick(lx,ly,rx,ry,now)`、`on_teleop(v,w,now)`、`set_param(name,value)->Optional[str]`、`params()`、`step(now,dt)->GuardOutput`、`state(now)->dict`；`scan_to_points(...)`、`swept_clearance(...)`、`TUNABLE`、`ESTOP_EVENTS`。
- `g1d_core.chassis_http`：`ChassisHttp(base_url, timeout)`、`EventTracker`（原样迁移）。

**步骤：**
1. 原样复制 `g1d_ws/python/g1d_nav_py/guard_core.py` → `guard.py`（来源写在模块 docstring），`test_guard_core.py` → `test_guard.py`：把 `from_config(load_config())` 用例改为 `from_dict({"stop_m": 0.45, ...})`，其余 16 项用例不改。先提交"原样迁移"，再提交适配。
2. 新增用例 `test_deploy_params_stop_within_0p5s`：`GuardParams(cmd_timeout=0.25, brake_v=2.0)`，0.5 m/s 匀速后停发，断言输出在 0.5 s + 2·DT 内归零。
3. `test_chassis_http.py`：`EventTracker` 首次不报、之后只报新事件；`ChassisHttp.events()` 解析 list / `{"events": [...]}`（用本地 `ThreadingHTTPServer`）。
4. Run: `pytest src/g1d_core/test -q` → 全部 PASS。

**提交：** `feat(core): g1d_core package with guard logic and chassis HTTP client migrated from g1d_ws`

---

### Task 3：`g1d_core.kinematics`（URDF 正运动学）与 `g1d_core.body`（关节映射、姿态锁定）

**Files:**
- Create: `src/g1d_core/g1d_core/kinematics.py`、`g1d_core/body.py`
- Test: `test/test_kinematics.py`、`test/test_body.py`

**Interfaces (Produces):**
- `kinematics.Urdf.from_string(xml) / from_file(path)`；`.fk(joints: dict, root: str, tip: str) -> np.ndarray(4x4)`（支持 fixed / revolute / continuous / prismatic，轴任意，origin rpy）；`rpy_to_matrix(r,p,y)`、`matrix_to_rpy(R)`。
- `body.ARM_JOINTS`（14 个 URDF 关节名，左 7 右 7，顺序 shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw）。
- `body.BodyConfig`（dataclass）：`lift_scale=1.0, lift_offset=0.0, lift_max=0.42, waist_yaw_index=12, arm_first_index=15, nav_waist_pitch=0.0, arm_tuck=[0]*14, tol_waist_rad=radians(2), tol_arm_rad=radians(5), tol_lift_m=0.01, tol_lift_speed=0.005, stale_s=0.5, require_waist_pitch=True, require_arms=True`；`from_dict(d)`。
- `body.joint_positions(lift_y, waist_pitch, waist_yaw, arm_q, cfg) -> Dict[str,float]`：`h = clamp(lift_scale*lift_y + lift_offset, 0, lift_max)`，`LZ_mt_Joint = LZ_it_Joint = h/2`，`Yaw_Joint = waist_pitch`，`torso_Joint = waist_yaw`，手臂按 `ARM_JOINTS`；`arm_q is None` 时用 `arm_tuck`。
- `body.BodySample`（dataclass：`lift_y, lift_speed, waist_pitch, waist_yaw, arm_q, t_lift, t_lowstate, t_waist`）。
- `body.LockChecker(cfg)`：`reset_reference(sample)`；`check(sample, now) -> (locked: bool, reasons: List[str])`，原因取值 `stale_lift | stale_lowstate | stale_waist_pitch | waist_yaw | waist_pitch | lift_moving | lift_changed | arms`；首次 check 自动设置参考高度。

**测试用例：**
- FK：零位 `AGV_link → head_link` = `(0.057335, 0.000168, 0.679509)`（±1e-5）；升降 `h=0.42` 时 z 增加 0.42；腰俯仰 0.5 rad 时 head 绕 `Yaw_Link` y 轴转动（与手算一致）；腰偏航 π/2 时 head x/y 交换。
- `joint_positions` 夹紧、两段各半、手臂缺省为 `arm_tuck`。
- `LockChecker`：全部在容差内 → locked；腰偏航 3° → `waist_yaw`；升降速度 0.01 → `lift_moving`；高度变化 2 cm → `lift_changed`；手臂一关节 6° → `arms`；lowstate 超时 → `stale_lowstate`；`require_waist_pitch=False` 时缺 waistpitch 不报。

**提交：** `feat(core): URDF forward kinematics, body joint mapping and pose-lock checker`

---

### Task 4：`g1d_description`

**Files:**
- Create: `src/g1d_description/{package.xml, CMakeLists.txt, UPSTREAM.md}`、`urdf/g1_d_with_dex1_1.urdf`（先原样提交）、`urdf/g1d.urdf.xacro`、`config/extrinsics.yaml`、`launch/description.launch.py`、`scripts/fetch_meshes.py`、`meshes/.gitignore`
- Test: `src/g1d_core/test/test_description.py`（Windows：`pip install xacro`，用 `xacro.process_file` + `g1d_core.kinematics`；无 xacro 时 skip）

**内容：**
- 第一次提交：HF `unitreerobotics/unitree_sim_isaaclab_usds@main assets/urdf/g1_d_description/g1_d_with_dex1_1.urdf` 原样。第二次提交补丁：mesh 路径改为 `package://g1d_description/meshes/<file>`。
- `g1d.urdf.xacro`：参数 `base_to_agv_z`（默认 0.10，待实测）、`head_to_livox_xyz`（`0.05 0 0.55`）、`head_to_livox_rpy`（`3.14159265 -0.04 0`）、`livox_to_imu_xyz`（`0.011 0.02329 -0.04412`，IMU 在雷达系中的位置 = FAST-LIO `extrinsic_T` 取反）；`xacro:include` 原 URDF；新增 `base_link → AGV_link`、`head_link → livox_frame`、`livox_frame → livox_imu`、`Lidar_Link → chassis_laser`（单位）。
- `config/extrinsics.yaml`：以上参数（标定工具写回此文件）。
- `launch/description.launch.py`：读 `extrinsics.yaml`，xacro 生成 `robot_description`，启动 `robot_state_publisher`（`publish_frequency 50`）。
- `scripts/fetch_meshes.py`：从 HF 下载 URDF 引用的 STL 到包内 `meshes/`（不入库）。

**测试用例：** xacro 展开后（默认参数、关节零位）：`base_link → livox_frame` 平移 = `AGV→head (0.057335, 0.000168, 0.679509)` + `(0.05, 0, 0.55)` + `(0, 0, 0.10)` = `(0.107335, 0.000168, 1.329509)`（±1e-5；零位时 `head_link` 无旋转），旋转 = `rpy_to_matrix(π, −0.04, 0)`（±1e-6）；`chassis_laser` 相对 `base_link` = `(0.1871, −0.016137, 0.20085)`。

**提交：** `vendor(g1d_description): G1-D URDF from unitree_sim_isaaclab_usds (unmodified)`；`vendor(g1d_description): package:// mesh paths`；`feat(description): xacro with base_link, livox frames, chassis_laser; launch; mesh fetch script`

---

### Task 5：`g1d_msgs`

**Files:** `src/g1d_msgs/{package.xml, CMakeLists.txt, msg/GuardState.msg, msg/BodyLock.msg}`

- `GuardState.msg`：`std_msgs/Header header`、`float32 v, w, v_cmd, w_cmd, v_target, w_target`、`string blocked`、`bool estop`、`string estop_reason`、`bool manual`、`string manual_source`、`string source`、`bool publishing`、`float32 front_clear, rear_clear`（无障碍为 +inf）、`bool rotate_ok`、`float32 cmd_age, scan_age, cloud_age`（从未收到为 −1）。
- `BodyLock.msg`：`std_msgs/Header header`、`bool locked`、`string[] reasons`、`float32 lift_height, lift_speed, waist_pitch, waist_yaw`。

**验证：** CI `colcon build --packages-select g1d_msgs` 通过。

**提交：** `feat(msgs): GuardState and BodyLock messages`

---

### Task 6：`g1d_robot_bridge`

**Files:**
- Create: `src/g1d_robot_bridge/{package.xml, setup.py, setup.cfg, resource/…, g1d_robot_bridge/__init__.py, chassis_restamp.py, body_state.py, chassis_events.py, qos.py}`、`config/robot_bridge.yaml`
- Test: `src/g1d_robot_bridge/test/test_nodes.py`（rclpy 进程内）

**节点：**
- `chassis_restamp`：订阅 `/slamware_ros_sdk_server_node/odom`、`/scan`（best-effort），`header.stamp = now − latency`，odom `frame_id=wheel_odom, child=base_link`，scan `frame_id=chassis_laser`；发布 `/chassis/odom`、`/chassis/scan`（sensor QoS）；统计原始时间戳偏差（参数 `log_offset_every`）。
- `body_state`：订阅 `/hispeed_state`（Point32）、`/lowstate`（unitree_hg/LowState）、`/waistpitchstate`（unitree_hg/MotorState），50 Hz 定时器发布 `/joint_states`（`g1d_core.body.joint_positions`，任一必需源超时则不发布）与 `/g1d/body_lock`（`LockChecker`）；服务 `/g1d/body_lock/reset`（std_srvs/Trigger）重置参考高度。
- `chassis_events`：2 Hz 轮询 HTTP 事件（`EventTracker`），新事件发布 `/g1d/chassis_event`（std_msgs/String，reliable，depth 10）；`enabled:=false` 时不轮询。

**测试用例（rclpy）：** 发布滞后 181 s 的 odom → 收到的 `/chassis/odom` 时间戳与 now 差 < 0.1 s；发布 lowstate/hispeed/waistpitch → `/joint_states` 含 `LZ_mt_Joint = h/2`、`torso_Joint`；腰偏航超差 → `/g1d/body_lock.locked == False` 且含 `waist_yaw`；本地 HTTP 事件服务器新增 `BUMPER_TRIGGERED` → 收到 `/g1d/chassis_event`。

**提交：** `feat(robot_bridge): SLAMWARE restamp, body joint states + pose lock, chassis HTTP events`

---

### Task 7：`g1d_sim` 底盘仿真

**Files:**
- Create: `src/g1d_core/g1d_core/sim2d.py`（`raycast_boxes`、`footprint_gap`、`DiffDrive`，迁移自 `mock_robot.py`）、`test/test_sim2d.py`
- Create: `src/g1d_sim/{package.xml, setup.py, …, g1d_sim/chassis_sim.py, g1d_sim/world.py}`、`worlds/{empty.yaml, wall.yaml, corridor.yaml}`

**内容：**
- 世界文件：`boxes: [[x0, y0, z0, x1, y1, z1], …]`（三维，M3 雷达仿真共用）；底盘雷达只与覆盖雷达高度的盒子相交。
- `chassis_sim` 节点：订阅 `/cmd_vel_no_limit`；`DiffDrive`（`cmd_timeout 0.5`、`max_acc`、`max_dec 2.0`、`max_ang_acc`、限幅）；发布 `/slamware_ros_sdk_server_node/odom`（时间戳滞后 181 s）、`/scan`（360 束）、`/hispeed_state`、`/lowstate`（50 Hz，腰偏航 / 手臂取参数）、`/waistpitchstate`、`/wirelesscontroller`（20 Hz，摇杆值取参数 / 话题 `/sim/joystick`）；真值 `/sim/ground_truth`（nav_msgs/Odometry，`frame_id=world`）、`/sim/min_gap`（std_msgs/Float32）；测试控制：`/sim/trigger_event`（std_msgs/String，加入 HTTP 事件列表）、`/sim/body`（sensor_msgs/JointState，改变升降 / 腰 / 手臂）；可选 HTTP 服务（`http_port`）。

**测试用例：** `sim2d`：射线与盒子距离精确、footprint_gap 正负；`DiffDrive` 指令超时后按 `max_dec` 停车、限幅。

**提交：** `feat(sim): 2D physics utils in g1d_core and chassis simulator node with Unitree topics`

---

### Task 8：`g1d_guard` 节点

**Files:**
- Create: `src/g1d_guard/{package.xml, setup.py, …, g1d_guard/guard_node.py}`、`config/guard.yaml`
- Test: `src/g1d_guard/test/test_guard_node.py`（rclpy 进程内）

**节点 `guard_node`：**
- 参数：`GuardParams` 全部字段（ROS 参数），`cloud_z_min 0.08`、`cloud_z_max 1.85`、`cloud_range 3.0`、`base_frame base_link`、`other_writers refuse|warn`、`chassis_http`（为空则不调用）、`abort_slamware_on_start true`。
- 启动：等待 1.5 s 发现，`get_publishers_info_by_topic('/cmd_vel_no_limit')` 非空 → `refuse` 时报错退出；然后创建发布者（reliable）；可选 HTTP 中止 SLAMWARE 任务。
- 订阅：`/nav/cmd_vel`、`/g1d/teleop_cmd`（Twist）、`/chassis/scan`（TF：`base_frame ← scan.frame_id`，取平移与 yaw → `scan_to_points`）、`/g1d/obstacle_cloud`（PointCloud2，TF 到 `base_frame`，高度带 + 距离过滤）、`/wirelesscontroller`（best-effort）、`/g1d/chassis_event`、`/g1d/estop`（std_msgs/Bool）。
- 30 Hz 定时器：`GuardCore.step(time.monotonic(), dt)`，`publish` 时写 `/cmd_vel_no_limit`；10 Hz 发布 `/g1d/guard_state`（GuardState）。
- 参数回调：`TUNABLE` 内参数走 `GuardCore.set_param`，校验失败拒绝。
- 退出：若在运动，连发 3 次零速。

**测试用例：** 无指令时不发布；`/nav/cmd_vel` 0.3 m/s + 远处扫描 → 输出加速到 0.3；墙在 0.6 m → `blocked == "front"` 且输出 0；`/g1d/estop True` → 立即 0 并锁存，`False` 解除；已有另一个 `/cmd_vel_no_limit` 发布者且 `refuse` → 节点构造抛出 `RuntimeError`。

**提交：** `feat(guard): rclpy guard node (single writer of /cmd_vel_no_limit) with params, TF-aware inputs, state`

---

### Task 9：`g1d_bringup` 与集成测试 `g1d_tests`

**Files:**
- Create: `src/g1d_bringup/{package.xml, CMakeLists.txt, launch/robot_interface.launch.py, launch/sim.launch.py, config/cyclonedds_wired.xml, config/cyclonedds_wifi.xml, scripts/g1d_env.sh}`
- Create: `src/g1d_tests/{package.xml, setup.py, …, test/test_guard_stop.py, test/test_interface_chain.py}`

**内容：**
- `robot_interface.launch.py`：参数 `sim:=false`、`guard:=true`、`chassis_http`；启动 description、`chassis_restamp`、`body_state`、`chassis_events`、`guard`。
- `sim.launch.py`：`chassis_sim`（world 参数）+ `robot_interface.launch.py`（`chassis_http` 指向仿真 HTTP）。
- `g1d_env.sh`：`RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID=0`、按 `G1D_NET_IF` 生成 `CYCLONEDDS_URI`；WiFi 模式使用单播 peer 配置。
- `test_guard_stop.py`（launch_testing）：仿真空世界，`require_cloud=false`；测试节点以 20 Hz 发 `/nav/cmd_vel` 0.5 m/s，直到真值速度 ≥ 0.45；停发，记录真值速度 < 0.02 的时刻 → 断言 ≤ 0.6 s；随后急停 / 遥控器接管用例。
- `test_interface_chain.py`：TF `base_link → livox_imu` 可查；`/chassis/scan` 时间戳接近 now；`/g1d/body_lock` 在默认姿态为 locked。

**提交：** `feat(bringup): interface + sim launch, CycloneDDS configs; integration tests (stop <= 0.6 s, TF chain)`

---

### Task 10：`g1d_tools`：`check_interfaces`、`cmd_vel_probe`

**Files:**
- Create: `src/g1d_core/g1d_core/probe.py`（`RateStats`、`stop_stats`），`test/test_probe.py`
- Create: `src/g1d_tools/{package.xml, setup.py, …, g1d_tools/check_interfaces.py, g1d_tools/cmd_vel_probe.py}`

**内容：** 与 `g1d_ws` 同功能的 rclpy 版本：
- `check_interfaces`：best-effort 订阅所有机器人话题与本工程话题，报告频率、时间戳偏差、样本、`/cmd_vel_no_limit` 写入者（含非 ROS 写入者）、HTTP 连通性，输出 JSON。
- `cmd_vel_probe`：要求 `--i-know-it-moves`；检测到其他写入者即退出；silence / zero / clamp / hold 四项；输出 JSON 与 `zero_hold` 建议值，并判断"停发后停车时间 ≤ 0.6 s"放行门槛。

**测试用例：** `RateStats` 频率 / 中位偏差；`stop_stats` 在合成里程计序列上的停车时间和距离。

**提交：** `feat(tools): ROS 2 check_interfaces and cmd_vel_probe; probe statistics in g1d_core`

---

### Task 11：收尾

- 更新 `PLAN.md`（勾选 M0–M2、进度日志）；README 补充构建与运行 M2 的方法；推送并确认 CI 全绿（失败则修复）。

**提交：** `docs: PLAN progress M0-M2, README build/run`
