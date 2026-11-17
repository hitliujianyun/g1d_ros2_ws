# 上游来源

| 项 | 内容 |
|---|---|
| 仓库 | https://github.com/Livox-SDK/livox_ros_driver2 |
| 版本 | `2144554`（"Support Mid360L Lidar"；Mid-360s / Jazzy 支持见 `13eb05e`） |
| 许可 | MIT（`LICENSE.txt`） |

## 本仓库中的修改（每项为独立提交，提交信息以 `vendor(livox_ros_driver2):` 开头）

1. 原样提交（不含 `.git`）。
2. colcon 直接构建：新增 `package.xml`（内容同 `package_ROS2.xml`），CMake 在未指定时默认 `ROS_EDITION=ROS2`、`DISTRO_ROS=$ENV{ROS_DISTRO}`（上游需 `build.sh` 复制文件并传参）；ROS 2 分支默认 C++17（Humble / Jazzy 的 rclcpp 需要）。

时间戳：未做 PTP 时驱动按主机 `std::chrono::high_resolution_clock::now()` 给每个包打戳（`pub_handler.cpp` `GetEthPacketTimestamp`），与其余 ROS 话题同一时钟，未修改。
雷达配置 JSON 与启动文件由 `g1d_bringup` 提供，不使用本包的 `config/`、`launch_ROS2/`。
