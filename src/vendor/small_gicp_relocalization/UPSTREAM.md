# 上游来源

| 项 | 内容 |
|---|---|
| 仓库 | https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization |
| 版本 | `81c9bd3`（"[pre-commit.ci] pre-commit autoupdate (#62)"） |
| 许可 | Apache-2.0（`LICENSE`） |
| 未带入 | `.github/`（上游 CI） |
| 依赖 | small_gicp（`src/external/small_gicp` 子模块 @ `fa0cfc9`，由 colcon 构建，避免上游 CMake 的 FetchContent 拉取 master） |

## 本仓库中的修改（每项为独立提交，提交信息以 `vendor(small_gicp_relocalization):` 开头）

1. 原样提交。
2. 见后续提交与下表。

| 修改 | 原因 |
|---|---|
| C++17 | Jazzy 的 rclcpp 需要 C++17 |
| `~/status` 话题（`std_msgs/Float64MultiArray`：converged、inliers、source points、error、iterations） | 上游不输出匹配质量；`g1d_localization` 据此判断收敛 / 丢失 |

用法约定（无需修改代码）：g1d 保存的 `map.pcd` 已在 map 坐标系，`base_frame` 与 `lidar_frame` 都设为 `base_link`，使上游加载地图时的变换为单位阵；输入点云为 `lio_bridge` 发布的 `/g1d/registered_scan`（odom 系）。
