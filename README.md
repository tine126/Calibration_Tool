# 投篮系统自动标定工具

一个基于深度相机和Open3D的篮球场地面与篮筐自动标定系统，支持离线点云文件和实时相机采集两种模式。通过点云处理算法自动计算相机到世界坐标系的4x4变换矩阵，并可自动更新目标配置文件。

## 目录

- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [安装说明](#安装说明)
- [快速开始](#快速开始)
- [配置说明](#配置说明)
- [使用指南](#使用指南)
- [输出结果](#输出结果)
- [故障排除](#故障排除)

## 功能特性

### 核心功能
- **地面检测**：使用RANSAC算法自动检测并拟合地面平面，自动过滤地面噪声点
- **坐标变换**：计算旋转矩阵将地面法向量对齐到Z轴，应用Y轴180度旋转，计算平移向量
- **篮筐检测**：基于高度过滤和RANSAC圆形拟合的篮筐检测算法
- **4x4变换矩阵**：自动生成包含旋转和平移的齐次变换矩阵（平移单位：米）
- **双运行模式**：
  - **离线模式**：从PLY/PCD点云文件加载数据
  - **实时模式**：从深度相机直接采集点云数据

### 自动配置更新
- **完全保留格式**：使用正则表达式替换，保留原配置文件的所有注释、缩进和空格
- **自动备份**：每次更新前自动备份原配置文件（Unix时间戳命名）
- **智能更新**：只更新 `transform_matrix` 和篮筐坐标，其他配置不变
- **单位转换**：自动将毫米转换为米

### 实时采集功能
- **多帧融合**：采集多帧点云并融合，降低随机噪声（支持Orbbec相机）
- **质量检测**：自动检查点云质量（点数、覆盖范围）
- **自动重试**：质量不达标时自动重新采集

### 支持的相机
- Orbbec（奥比中光）系列（如 Gemini 2L）
- Intel RealSense 系列（需安装pyrealsense2）
- Azure Kinect（需安装pyk4a）

### GUI界面
- **简单模式**：一键式自动标定，自动完成所有步骤并保存结果
- **详细模式**：9步分步标定流程，每步可单独执行和可视化
- **实时日志**：显示详细的执行日志和统计信息
- **结果显示**：展示标定结果、变换矩阵、篮筐信息等
- **配置更新**：标定完成后可一键更新目标配置文件

## 系统要求

### 硬件要求
- CPU: Intel i5 或以上
- 内存: 8GB 或以上
- GPU: 支持 OpenGL 3.3+（用于可视化）
- 深度相机: RealSense / Kinect / Orbbec（仅实时模式需要）

### 软件要求
- Python 3.7-3.10
- Windows 10/11 或 Linux

## 安装说明

### 1. 克隆或下载项目

```bash
git clone https://github.com/tine126/Calibration_Tool
cd Calibration_Tool
```

### 2. 创建虚拟环境（推荐）

```bash
# 使用 conda
conda create -n calibration python=3.9
conda activate calibration

# 或使用 venv
python -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows
```

### 3. 安装依赖包

```bash
pip install -r requirements.txt
```

**requirements.txt 内容：**
```
open3d>=0.17.0
numpy>=1.21.0
PyYAML>=6.0
PyQt5>=5.15.0
```

### 4. 安装相机SDK（仅实时模式需要）

根据您使用的相机类型安装对应的SDK：

**Orbbec相机：**
```bash
pip install pyorbbecsdk
```

**RealSense相机：**
```bash
pip install pyrealsense2
```

**Azure Kinect相机：**
```bash
# 参考官方文档安装 Azure Kinect SDK
# https://github.com/microsoft/Azure-Kinect-Sensor-SDK
pip install pyk4a
```

## 快速开始

### 一键启动（Windows）

双击运行 `start_gui.bat` 即可自动激活conda环境并启动GUI界面。

**注意：**
- 默认使用conda环境名称：`pass`
- 如需修改环境名称，编辑 `start_gui.bat` 第14行：`set CONDA_ENV=你的环境名`

### 使用GUI（推荐）

1. **启动GUI程序**
```bash
# 方式1：使用bat文件（Windows，推荐）
start_gui.bat

# 方式2：直接运行Python脚本
python gui.py
```

2. **选择数据源模式**
   - 离线模式：选择"离线模式（从文件加载）"，点击"选择点云文件"选择PLY文件
   - 实时模式：选择"实时模式（从相机采集）"

3. **选择标定模式**
   - **简单模式**：点击"一键自动标定"按钮，系统自动完成所有步骤
   - **详细模式**：点击"分步标定（详细）"按钮，手动执行每个步骤

### 使用命令行

**离线模式：**
```bash
python main.py
```

**实时模式：**
1. 修改 `CTconfig.yaml`，设置 `data_source.mode: "realtime"`
2. 运行：
```bash
python main.py
```

## 配置说明

配置文件位于 `CTconfig.yaml`，包含以下主要配置项：

### 数据源配置

```yaml
data_source:
  mode: "offline"  # 或 "realtime"

  # 离线模式配置
  offline:
    input_file: "data/1222/DepthPoints_3546026307.ply"

  # 实时模式配置
  realtime:
    camera:
      type: "orbbec"  # 相机类型: "realsense", "kinect", "orbbec"
      serial_number: ""  # 相机序列号（为空则使用第一个可用相机）
      resolution:
        width: 1280
        height: 720
      frame_rate: 30
      depth_range: [0.3, 10.0]  # 深度范围（米）

    # 多帧融合配置
    multi_frame_fusion:
      enabled: true
      num_frames: 10  # 采集帧数
      fusion_method: "average"  # "average" 或 "median"

    # 质量检测配置
    quality_check:
      enabled: true
      min_points: 50000  # 最小点数
      coverage_check: true
      max_retries: 1  # 最大重试次数
```

### 点云预处理参数

```yaml
preprocessing:
  # 降采样
  downsample:
    voxel_size: 0.01  # 体素大小（米）

  # 噪声滤波
  noise_filter:
    nb_neighbors: 50
    std_ratio: 1.0

  # 聚类
  clustering:
    eps: 0.1  # 聚类距离（米）
    min_points: 10
    max_points: 100000
```

### 地面检测参数

```yaml
ground_detection:
  distance_threshold: 200  # RANSAC距离阈值（毫米）
  ransac_n: 3
  num_iterations: 10000
  random_seed: 42
```

### 篮筐检测参数

```yaml
hoop_detection:
  standard_diameter: 0.45  # 标准篮筐直径（米）
  height_range: [2.9, 3.2]  # 篮筐高度范围（米）
  diameter_tolerance: 0.025  # 直径容差（米）
  random_seed: 42
```

### 输出配置

```yaml
output:
  dir: "output"  # 输出目录
  config_file: "config.json"  # 转换矩阵配置文件
  save_intermediate: true  # 保存中间结果
  save_raw_pointcloud: true  # 保存原始点云

  # 目标配置文件更新设置
  target_config:
    enabled: true  # 是否启用自动更新目标配置文件功能
    path: "F:\\Code\\config.yaml"  # 目标配置文件路径
    backup_dir: "backup_config"  # 备份目录（相对于当前工具目录）
```

**target_config 说明：**
- `enabled`：启用后，标定完成会自动更新目标配置文件
- `path`：目标配置文件的完整路径（通常是主系统的config.yaml）
- `backup_dir`：备份目录，每次更新前会自动备份原文件（使用Unix时间戳命名）

### 可视化配置

```yaml
visualization:
  enabled: true  # 是否启用可视化（GUI模式建议设为false）
  point_size: 2.0
```

## 使用指南

### GUI详细模式工作流程

详细模式包含9个步骤，每步都可单独执行和可视化：

#### 步骤1: 加载点云
- **离线模式**：从选定的PLY/PCD/XYZ文件加载
- **实时模式**：从相机采集并融合多帧点云
- 显示点云基本信息：点数、边界框、是否有颜色/法向量

#### 步骤2: 降采样
- 使用体素网格滤波进行降采样
- 减少点云数量，提高处理速度
- 默认体素大小：0.01m

#### 步骤3: 噪声滤波
- 统计离群值去除（Statistical Outlier Removal）
- 移除孤立的噪声点
- 参数：邻居点数、标准差倍数

#### 步骤4: 地面检测（RANSAC）
- 使用RANSAC算法拟合地面平面
- 检测地面内点
- 计算地面法向量与Z轴夹角
- 可视化：蓝色=地面点，灰色=非地面点

#### 步骤5: 过滤地面噪声
- 对地面点进行二次噪声过滤
- 计算有效地面点和噪声点
- 可视化：绿色=有效地面点，红色=噪声点

#### 步骤6: 计算旋转矩阵
- 根据地面法向量计算旋转矩阵
- 目标：将地面法向量对齐到Z轴[0,0,1]
- 显示3x3旋转矩阵

#### 步骤7: 应用变换并拟合平面
- 应用旋转矩阵到点云
- 应用Y轴180度旋转
- 计算平移向量（使地面Z坐标为0）
- 拟合最终地面平面方程：z = 0
- 可视化：显示变换后点云和地面参考平面

#### 步骤8: 篮筐检测
- 根据高度范围过滤候选点云
- 投影到XY平面并进行RANSAC圆形拟合
- 验证直径是否符合标准（450mm ± 容差）
- 计算篮筐中心3D坐标和离地高度
- 可视化：黄色球=篮筐中心，红色圆环=篮筐

#### 步骤9: 查看结果
- 显示完整的标定结果：
  - 篮筐中心位置（毫米和米）
  - 4x4变换矩阵（齐次坐标，平移单位：米）
  - 篮筐直径、离地高度、拟合误差
- 自动保存到 `output/config.json`

### 实时采集模式使用技巧

1. **相机放置**
   - 确保相机稳定固定
   - 相机视野应包含完整的篮筐和地面区域
   - 避免强光直射和反光表面

2. **采集环境**
   - 充足但不过强的环境光
   - 避免阳光直射
   - 减少移动物体（人员）

3. **质量检测失败处理**
   - 检查相机是否遮挡
   - 调整相机角度和位置
   - 修改 `CTconfig.yaml` 中的质量检测参数：
     ```yaml
     quality_check:
       min_points: 30000  # 降低点数要求
       max_retries: 3  # 增加重试次数
     ```

4. **多帧融合建议**
   - 室内环境：10帧足够
   - 噪声较大环境：增加到15-20帧
   - 快速测试：减少到5帧

## 输出结果

### 自动更新目标配置文件

标定完成后，工具会**自动将结果写入目标配置文件**（默认为 `F:\Code\config.yaml`），无需手动复制。

**自动更新的内容：**
1. **transform_matrix**：4x4变换矩阵（平移量单位：米）
2. **篮筐坐标**：
   - `trajectory_analysis.hoop_x`、`hoop_y`、`hoop_height`
   - 所有 `roi_list` 中 `exclude_cylinder` 的篮筐坐标

**配置更新功能：**
```yaml
# CTconfig.yaml 中配置
target_config:
  enabled: true  # 启用自动更新
  path: "F:\\Code\\config.yaml"  # 目标配置文件路径
  backup_dir: "backup_config"  # 备份目录
```

**特性：**
- ✅ 完全保留原文件格式（注释、空格、缩进）
- ✅ 自动备份原配置文件（时间戳命名）
- ✅ 自动单位转换（毫米→米）
- ✅ 只更新指定字段，其他内容不变

**备份文件位置：**
```
backup_config/
└── config_1735534623.yaml  # Unix时间戳命名
```

### 输出目录结构

```
output/
├── config.json              # 标定结果配置文件（最重要）
└── raw_pointcloud_YYYYMMDD_HHMMSS.ply  # 原始点云（实时模式可选保存）

backup_config/
└── config_1735534623.yaml   # 备份的目标配置文件（Unix时间戳命名）
```

### config.json 格式

标定工具输出的 `output/config.json` 格式如下：

```json
{
  "rotation_matrix": [
    [r11, r12, r13],
    [r21, r22, r23],
    [r31, r32, r33]
  ],
  "rotation_y_180": [
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, -1.0]
  ],
  "combined_rotation": [
    [cr11, cr12, cr13],
    [cr21, cr22, cr23],
    [cr31, cr32, cr33]
  ],
  "translation_vector": [tx_mm, ty_mm, tz_mm],
  "transform_matrix_4x4": [
    [cr11, cr12, cr13, tx_m],
    [cr21, cr22, cr23, ty_m],
    [cr31, cr32, cr33, tz_m],
    [0.0, 0.0, 0.0, 1.0]
  ],
  "ground_normal": [nx, ny, nz],
  "ground_plane_equation": [a, b, c, d],
  "hoop_center": [x_mm, y_mm, z_mm],
  "metadata": {
    "ground_detection": {
      "ground_point_count": 56802,
      "noise_point_count": 14906,
      "ground_z_mean_mm": 7504.86,
      "ground_z_angle_deg": 47.41
    },
    "rotated_ground": {
      "z_mean_mm": 0.55,
      "z_median_mm": 0.0,
      "z_std_mm": 49.84,
      "translation_z_mm": 5564.92,
      "plane_normal_z_angle_deg": 0.43,
      "distance_to_plane_mean_mm": 42.43,
      "distance_to_plane_std_mm": 28.93
    },
    "hoop_detection": {
      "diameter_mm": 459.89,
      "height_above_ground_mm": 2976.57,
      "fit_error_mm": 17.65,
      "z_std_mm": 32.32,
      "inlier_count": 217,
      "inlier_ratio": 0.87
    }
  }
}
```

**字段说明：**
- `rotation_matrix`: 地面对齐旋转矩阵（3x3）
- `rotation_y_180`: Y轴180度旋转矩阵
- `combined_rotation`: 组合旋转矩阵（已应用符号修正）
- `translation_vector`: 平移向量（单位：毫米）
- `transform_matrix_4x4`: 完整的4x4齐次变换矩阵（**平移单位：米**）
- `ground_plane_equation`: 地面平面方程 [a, b, c, d]，满足 ax + by + cz + d = 0
- `hoop_center`: 篮筐中心坐标（单位：毫米）
- `metadata`: 详细的检测统计信息

## 故障排除

### 问题1: 警告"未找到transform_matrix"或"缺少hoop_center"

**症状：**
```
[WARNING] 未找到transform_matrix，跳过更新
[WARNING] 警告: 标定结果中没有篮筐坐标 (hoop_center)，跳过篮筐坐标更新
```

**原因：**
- 这是v1.1.1之前版本的字段名不匹配bug

**解决方案：**
- 已在v1.1.1版本中修复
- 更新到最新版本即可解决

### 问题2: 相机初始化失败

**症状：**
```
错误: Orbbec相机初始化失败
```

**解决方案：**
1. 检查相机是否正确连接
2. 确认已安装相机SDK：`pip install pyorbbecsdk`
3. 检查USB端口是否为USB 3.0
4. Windows系统可能需要安装相机驱动

### 问题3: 采集的点云质量不达标

**症状：**
```
✗ X方向范围不足: 0.5m < 3.0m
```

**解决方案：**
1. 调整相机角度，确保视野足够大
2. 降低质量检测要求（修改 `CTconfig.yaml`）：
   ```yaml
   hoop_detection:
     height_range: [2.5, 3.5]  # 扩大高度范围
   ```

### 问题4: 地面检测失败

**症状：**
```
警告: 地面检测失败，未找到足够的地面点
```

**解决方案：**
1. 增加RANSAC迭代次数：
   ```yaml
   ground_detection:
     num_iterations: 20000
     distance_threshold: 300  # 放宽阈值
   ```
2. 检查点云中是否包含地面区域

### 问题5: 篮筐检测失败

**症状：**
```
警告: 未检测到符合标准的篮筐
```

**解决方案：**
1. 放宽篮筐直径容差：
   ```yaml
   hoop_detection:
     diameter_tolerance: 0.05  # 增加到±50mm
   ```
2. 调整篮筐高度范围：
   ```yaml
   hoop_detection:
     height_range: [2.7, 3.3]  # 扩大范围
   ```
3. 确保点云中包含完整的篮筐

### 问题6: 深度帧为None（仅第2-10帧）

**症状：**
```
警告: 深度帧为None
警告: 第 2 帧采集失败，跳过
```

**解决方案：**
- 系统已自动处理：增加了帧间延迟和缓冲区清空
- 如果仍然失败，增加延迟时间（修改 `camera_capture.py` 第431行）

### 问题7: GUI无法启动

**症状：**
```
ImportError: No module named 'PyQt5'
```

**解决方案：**
```bash
pip install PyQt5
```

### 问题8: Open3D可视化窗口无响应

**解决方案：**
- 在 `CTconfig.yaml` 中禁用可视化：
  ```yaml
  visualization:
    enabled: false
  ```
- 使用GUI详细模式查看结果

## 高级用法

### 自定义处理流程

您可以直接导入模块进行自定义处理：

```python
from camera_capture import CameraCapture
from preprocessing import preprocess_pointcloud
from ground_detection import detect_ground_plane
from hoop_detection import detect_hoop
import yaml

# 加载配置
with open('CTconfig.yaml', 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)

# 实时采集
camera = CameraCapture(config['data_source']['realtime'])
pcd = camera.capture_pointcloud()
camera.stop()

# 预处理
pcd_processed = preprocess_pointcloud(pcd, config['preprocessing'])

# 地面检测
ground_points, non_ground_points, plane_model = detect_ground_plane(
    pcd_processed, config['ground_detection']
)

# 篮筐检测
hoop_center, hoop_radius = detect_hoop(
    non_ground_points, plane_model, config['hoop_detection']
)

print(f"篮筐中心: {hoop_center}")
print(f"篮筐半径: {hoop_radius}")
```

### 批处理多个点云文件

```python
import os
from pathlib import Path

# 批量处理data目录下的所有PLY文件
data_dir = Path("data")
for ply_file in data_dir.glob("**/*.ply"):
    print(f"处理文件: {ply_file}")
    # 更新配置
    config['data_source']['offline']['input_file'] = str(ply_file)
    # 运行标定流程
    # ...
```

## 技术细节

### 坐标系定义

**相机坐标系（原始）：**
- 原点：相机光心
- X轴：水平向右
- Y轴：垂直向下
- Z轴：相机正前方（深度方向）

**世界坐标系（标定后）：**
- 原点：相机光心在地面的投影点
- X轴：平行于地面
- Y轴：平行于地面
- Z轴：垂直向上，与地面法向量对齐
- 地面平面方程：z = 0

**坐标变换流程：**
1. **地面对齐旋转**：计算旋转矩阵R，将地面法向量对齐到[0,0,1]
2. **Y轴180度旋转**：应用额外的180度旋转
3. **符号修正**：应用符号修正矩阵（确保与CloudCompare标定一致）
4. **平移**：计算平移向量T，使地面Z坐标移动到0

**最终变换矩阵：**
```
P_world = T_4x4 × P_camera_homogeneous

其中：
- P_camera_homogeneous = [x, y, z, 1]^T（相机坐标的齐次形式）
- T_4x4 = 4x4变换矩阵（包含旋转和平移）
- P_world = [x', y', z', 1]^T（世界坐标）
```

### 算法说明

**地面检测（RANSAC + 噪声过滤）：**
1. **RANSAC平面拟合**
   - 随机采样3个点拟合平面方程
   - 计算所有点到平面的距离
   - 统计内点数量（距离 < distance_threshold）
   - 迭代10000次，选择内点最多的平面
2. **地面噪声过滤**
   - 对地面点进行统计离群值滤波
   - 移除孤立的噪声点
   - 计算点到平面的距离统计信息

**旋转矩阵计算：**
1. 从地面平面方程提取法向量 n = [a, b, c]
2. 归一化法向量
3. 使用Rodrigues旋转公式计算旋转矩阵R
4. 使R将n旋转到[0, 0, 1]

**篮筐检测（高度过滤 + RANSAC圆形拟合）：**
1. **高度过滤**
   - 应用变换矩阵到点云
   - 过滤高度在[2.9m, 3.2m]范围内的点
2. **RANSAC圆形拟合**
   - 投影到XY平面（忽略Z坐标）
   - 随机采样3个点计算圆心和半径
   - 统计内点数量（距离圆 < threshold）
   - 迭代并选择最佳圆
3. **验证**
   - 检查直径是否在 450mm ± 容差范围内
   - 计算拟合误差、Z坐标标准差
   - 返回篮筐中心3D坐标和统计信息

## 联系与支持

如有问题或建议，请通过以下方式联系：

- 提交 Issue
- 发送邮件至：[tine126@163.com]



---

**版本：** 1.1.1
**最后更新：** 2026-01-05

**更新日志：**
- v1.1.1 (2026-01-05)
  - 🐛 修复字段名不匹配bug：`hoop_center_3d` → `hoop_center`
  - 🐛 修复transform_matrix单位重复转换bug
  - 📝 更新README文档，补充详细技术说明
  - 📝 添加完整的config.json格式说明

- v1.1.0 (2025-12-30)
  - ✨ 新增自动更新目标配置文件功能
  - ✨ 新增一键启动bat脚本（Windows）
  - ✨ 支持conda环境自动激活
  - 🔧 完全保留目标配置文件原有格式
  - 🔧 自动备份原配置文件

- v1.0.0 (2025-12-28)
  - 🎉 初始版本发布
  - ✨ 地面检测与篮筐检测功能
  - ✨ GUI简单模式和详细模式
  - ✨ 支持离线和实时两种数据源

