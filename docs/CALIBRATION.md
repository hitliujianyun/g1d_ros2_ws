# 标定说明

需要标定的量和工具：

| 量 | 文件 | 工具 |
|---|---|---|
| `head_to_livox_xyz / rpy`（Mid-360S 在头部的安装位姿，倒装） | `src/g1d_description/config/extrinsics.yaml` | `ros2 run g1d_tools calib_extrinsics` |
| `base_to_agv_z`（地面到 `AGV_link`） | 同上 | 卷尺 |
| `lift_scale / lift_offset`（升降读数 → URDF 行程） | `src/g1d_robot_bridge/config/robot_bridge.yaml` | 升降到最低 / 最高，用标定工具读雷达高度 |
| 自身点盒子 | `src/g1d_bringup/config/self_filter.yaml` | RViz 观察静止点云 |
| 导航姿态（`nav_waist_pitch`、`arm_tuck`） | `robot_bridge.yaml` | `ros2 topic echo /joint_states` |
| Mid-360 内置 IMU 相对点云（出厂值 `(0.011, 0.02329, −0.04412)`） | `extrinsics.yaml` / `fast_lio.yaml` | 一般不需要改 |
| 底盘雷达安装位姿（`chassis_laser` = URDF 上游的 `Lidar_Link`，单位变换） | `src/g1d_description/urdf/g1d.urdf.xacro` | 第 6 节核对；雷达偏航标定以它为基准 |
| 关节序号（`waist_yaw_index`、`arm_first_index`） | `robot_bridge.yaml` | 第 7 节 |
| SLAMWARE 数据延迟（`odom_latency`、`scan_latency`） | `robot_bridge.yaml` | 第 8 节（默认 0 即可） |

## 1. 雷达外参（地面法 + 底盘雷达对齐）

**条件**：平整地面；机器人静止，处于**导航姿态**（腰部、升降与导航时相同）；前方 ≥ 2 m 空地；周围 1–4 m 有墙或箱子（用于偏航）；`robot_interface.launch.py` 和雷达驱动在运行（不需要 FAST_LIO）：

```bash
ros2 launch g1d_bringup robot_interface.launch.py require_cloud:=false
ros2 launch g1d_bringup lio.launch.py lidar_source:=livox host_ip:=192.168.123.222    # 只需驱动 + self_filter
ros2 run g1d_tools calib_extrinsics --yaw-from-scan --write src/g1d_description/config/extrinsics.yaml
```

工具做三件事：

1. **横滚 / 俯仰 / 高度**：在雷达点云中 RANSAC 拟合地面平面（只接受与当前外参预期的"上方向"偏差 < 25° 的平面），把 `base_link → livox_frame` 的旋转修正为"地面法向 = 竖直"，平移的 z 取雷达到地面的实测高度。
2. **偏航**（`--yaw-from-scan`）：取雷达点中与底盘雷达同高（± 3 cm）的水平切片，绕雷达在地面的投影旋转，与 `/chassis/scan` 的点最佳重合（±10°，0.1° 步长）。底盘雷达的安装位姿来自 URDF。
3. 用当前关节状态下的 `base_link → head_link`，把结果换算成 `head_to_livox_xyz / rpy` 写回 `extrinsics.yaml`（终端会打印旧值），然后**重新构建 / 重启** `robot_interface`。

输出里的 "lidar height measured … URDF … error" 若 > 2 cm：要么改 `head_to_livox` 的 z（工具已写入），要么核对 `base_to_agv_z` 与 `lift_offset`（升降在零位时测量更准确）。

**验证**：启动 LIO 后原地旋转 360°（`ros2 run g1d_tools acceptance T4`），平移漂移应 < 3 cm。外参误差会表现为"原地转圈时底盘在画圆"。

## 2. 升降比例与零点

1. 升降到最低，运行 `calib_extrinsics`（不写回）记下实测雷达高度 `h0` 与 `/hispeed_state.y` 的 `y0`。
2. 升降到最高，记下 `h1`、`y1`。
3. URDF 两段升降之和 = `lift_scale · y + lift_offset`：`lift_scale = (h1 − h0) / (y1 − y0)`；在最低处 URDF 行程应为 0：`lift_offset = −lift_scale · y0`。
4. 写入 `robot_bridge.yaml`，重启，确认两种高度下 "error" 都 < 1 cm。

## 3. 自身点盒子

静止、导航姿态下，在 RViz 中显示 `/livox/lidar_raw`（过滤前）与 `/g1d/obstacle_cloud`（过滤后）。`self_filter.yaml` 的盒子在各自坐标系中（`head_link`：头、躯干、手臂；`base_link`：底盘），按 `[xmin, xmax, ymin, ymax, zmin, zmax]` 调整，直到车身附近 0.5 m 内没有稳定点簇，同时前方地面上 0.3 m 处的低矮障碍仍可见。改完重启 `lio.launch.py`。

## 4. 导航姿态

摆好导航时的腰部与手臂姿态，`ros2 topic echo /joint_states --once`，把 `Yaw_Joint` 写入 `nav_waist_pitch`（上游 URDF 的命名如此，这个关节实际是腰部**俯仰**；腰部偏航 `torso_Joint` 导航时必须为 0），14 个手臂关节按 `left_shoulder_pitch … right_wrist_yaw` 顺序写入 `arm_tuck`。容差：腰 2°、手臂 5°、升降 1 cm（`tol_*`）。

## 5. 底盘指令行为（不是标定，但决定守护参数）

```bash
ros2 run g1d_tools cmd_vel_probe --tests silence,zero,clamp,hold --i-know-it-moves
```

`silence.stop_time_s` → `release_gate`（≤ 0.6 s 放行）与 `zero_hold` 建议值；`clamp` → 底盘是否限幅；`hold` → 发零速时是否锁轮。

## 6. 底盘雷达位姿核对

`chassis_laser` 直接挂在上游 URDF 的 `Lidar_Link` 上（单位变换）。第 1 节的雷达偏航以底盘雷达为基准，所以先确认它本身没问题：机器人正对一面墙（1–2 m），RViz 固定坐标系设为 `base_link`，显示 `/chassis/scan`。墙线应与 `base_link` 的 y 轴平行、距离与卷尺一致（±2 cm）。若有固定偏差，在 `g1d.urdf.xacro` 的 `chassis_laser_joint` 里补偿，并在 `g1d_description/UPSTREAM.md` 记录。

## 7. 关节序号

`/lowstate` 的 `motor_state` 序号按宇树 G1 约定：12 = 腰部偏航，15–28 = 左臂 7 + 右臂 7（肩俯仰、肩横滚、肩偏航、肘、腕横滚、腕俯仰、腕偏航）。逐个关节轻微转动手臂（或用宇树 App 示教），对照 `ros2 topic echo /lowstate --field motor_state` 中变化的序号；不一致时改 `waist_yaw_index` / `arm_first_index`。验证：RViz 中机器人模型的手臂动作与实物一致。

## 8. SLAMWARE 数据延迟

`chassis_restamp` 用主机接收时间重打时间戳（原始戳约落后 181 s，日志定期打印）。接收时间比真实测量时刻晚一个网络 + SLAMWARE 处理延迟，通常几十毫秒，导航可以忽略。若想补偿：机器人以 0.3 m/s 直行，比较 `/chassis/odom` 与 LIO（`/odometry`）的位置曲线，使两者对齐所需的时间平移即为 `odom_latency`；`scan_latency` 用同样方法比较底盘雷达与 Mid-360S 看到的同一面墙。
