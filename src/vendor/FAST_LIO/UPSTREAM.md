# 上游来源

| 项 | 内容 |
|---|---|
| 仓库 | https://github.com/hku-mars/FAST_LIO，分支 `ROS2` |
| 版本 | `cc3c9e8`（"Merge pull request #454 …"） |
| 子模块 | `include/ikd-Tree` = https://github.com/hku-mars/ikd-Tree（分支 `fast_lio`）`e2e3f4e`，源码直接放入本目录 |
| 许可 | GPL-2.0（`LICENSE`）。仅内部使用；对外分发前按 GPL 开放修改后的源码 |
| 未带入 | `Log/`、`PCD/`、`doc/`（运行输出目录与文档图片） |

## 本仓库中的修改（每项为独立提交，提交信息以 `vendor(FAST_LIO):` 开头）

1. 原样提交。
2. 见后续提交与下表。

| 修改 | 原因 |
|---|---|
| C++17 | Jazzy 的 rclcpp 需要 C++17 |
| `common.world_frame` / `common.body_frame` 参数（默认 `camera_init` / `body`） | 原坐标系名写死；g1d 用 `lio_world` / `livox_imu`，使 `/cloud_registered_body` 直接挂在 TF 树上 |
| `publish.tf_en` 参数（默认 true） | g1d 由 `lio_bridge` 发布 `odom → base_link`，关闭 FAST_LIO 自带的 TF |
