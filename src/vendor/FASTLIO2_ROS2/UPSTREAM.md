# 上游来源

| 项 | 内容 |
|---|---|
| 仓库 | https://github.com/liangheming/FASTLIO2_ROS2 |
| 版本 | `7baa59f`（"Merge pull request #43 …"） |
| 带入 | `pgo/`（关键帧 + GTSAM iSAM2 位姿图 + PCL ICP 闭环验证 + 保存地图）、`interface/`（服务定义）；上游 README 存为 `README.upstream.md` |
| 未带入 | `fastlio2/`（g1d 用 hku-mars FAST_LIO）、`localizer/`、`hba/` |
| 许可 | MIT（`pgo/LICENSE`、`interface/LICENSE`） |

## 本仓库中的修改（每项为独立提交，提交信息以 `vendor(FASTLIO2_ROS2):` 开头）

1. 原样提交。
2. 见后续提交与下表。

| 修改 | 原因 |
|---|---|
| `pgo/package.xml`：`GTSAM` → `gtsam`，补 `yaml-cpp`、`libpcl-all-dev` | rosdep 键名（原键无法解析） |
| `publish_tf` 配置项（默认 true） | g1d 不需要 `map → lidar` 的 TF（地图坐标在保存时由 `g1d_mapping` 换算） |
