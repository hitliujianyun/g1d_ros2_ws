# 上游来源

| 文件 | 来源 | 版本 | 许可 |
|---|---|---|---|
| `urdf/g1_d_with_dex1_1.urdf` | Hugging Face 数据集 [`unitreerobotics/unitree_sim_isaaclab_usds`](https://huggingface.co/datasets/unitreerobotics/unitree_sim_isaaclab_usds)，`assets/urdf/g1_d_description/g1_d_with_dex1_1.urdf` | revision `27b13d380531c0c6ea6ab109511ee4872594e271`，SHA256 `8E55BB551E2609BBFCB44AFEEE2896D6675494B50C49EB8DF23E327A2FE03390` | 以数据集声明为准 |
| `meshes/*.STL`（不入库） | 同一数据集 `assets/urdf/g1_d_description/meshes/meshes/` | 同上 | 同上；由 `scripts/fetch_meshes.py` 下载 |

## 本仓库中的修改（每项为独立提交）

1. 原样提交（仅按 `.gitattributes` 统一为 LF 换行）。
2. mesh 路径由相对路径 `meshes/<file>` 改为 `package://g1d_description/meshes/<file>`，以便 RViz / robot_state_publisher 解析。

坐标系扩展（`base_link`、`livox_frame`、`livox_imu`、`chassis_laser`）在 `urdf/g1d.urdf.xacro` 中完成，不修改上游 URDF。
