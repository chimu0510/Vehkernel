# extensions

存放**擴充資源**：體積大、不隨 repo 提交，需自行從雲端下載的檔案。本目錄除了本說明之外都被 `.gitignore` 排除。

下載後解壓到本目錄，保持下表的目錄名稱。

| 擴充資源 | 目錄 | 內容 | 大小 | 使用者 |
|---|---|---|---|---|
| Wheeltec 車款模型 | `wheeltec_robot_urdf/` | 原廠 `wheeltec_robot_urdf` v2.1.0 完整套件（URDF + STL），未修改 | 約 1.2 GB | `ros2_ws/src/ugv_model_viewer` |

下載位置：<!-- TODO: 雲端連結 -->

完成後的結構：

```text
extensions/
└── wheeltec_robot_urdf/
    ├── urdf/*.urdf
    └── meshes/<車款>_meshes/*.STL
```
