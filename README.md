# 篮球场自动标定工具

一个基于深度相机的篮球场地面和篮筐自动标定系统，支持离线点云文件和实时相机采集两种模式。

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
- **地面检测**：使用RANSAC算法自动检测并拟合地面平面
- **篮筐检测**：基于圆形拟合的篮筐中心点和半径检测
- **坐标转换**：自动计算从相机坐标系到世界坐标系的转换矩阵
- **双运行模式**：
  - **离线模式**：从PLY点云文件加载数据
  - **实时模式**：从深度相机直接采集点云数据

### 实时采集功能
- **多帧融合**：采集多帧点云并融合，降低随机噪声
- **质量检测**：自动检查点云质量（点数、覆盖范围）
- **自动重试**：质量不达标时自动重新采集

### 支持的相机
- Intel RealSense 系列
- Azure Kinect
- Orbbec（奥比中光）系列（如 Gemini 2L）

### GUI界面
- **简单模式**：一键式自动标定
- **详细模式**：9步分步标定流程，每步可视化结果

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

### 使用GUI（推荐）

1. **启动GUI程序**
```bash
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
1. 修改 `config.yaml`，设置 `data_source.mode: "realtime"`
2. 运行：
```bash
python main.py
```

## 配置说明

配置文件位于 `config.yaml`，包含以下主要配置项：

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
```

### 可视化配置

```yaml
visualization:
  enabled: true  # 是否启用可视化（GUI模式建议设为false）
  point_size: 2.0
```

## 使用指南

### GUI详细模式工作流程

详细模式包含9个步骤，每步都有可视化结果：

#### 步骤1: 加载点云
- **离线模式**：从选定的PLY文件加载
- **实时模式**：从相机采集10帧并融合

#### 步骤2: 点云预处理
- 降采样（体素网格滤波）
- 统计离群值滤波
- 欧几里得聚类（选择最大簇）

#### 步骤3: 地面检测
- 使用RANSAC算法拟合地面平面
- 分离地面点和非地面点
- 计算地面法向量

#### 步骤4: 可视化地面
- 绿色：地面点
- 蓝色：非地面点
- 红色箭头：地面法向量

#### 步骤5: 篮筐区域提取
- 根据高度范围过滤点云
- 提取篮筐候选区域

#### 步骤6: 篮筐检测
- 圆形拟合检测篮筐
- 计算篮筐中心点和半径

#### 步骤7: 可视化篮筐
- 显示检测到的篮筐位置和尺寸

#### 步骤8: 计算转换矩阵
- 计算从相机坐标系到世界坐标系的变换
- 世界坐标系定义：
  - 原点：篮筐中心在地面的投影
  - X轴：平行于地面，指向右侧
  - Y轴：平行于地面，指向前方
  - Z轴：垂直向上

#### 步骤9: 保存结果
- 保存转换矩阵到JSON文件
- 可选：保存中间结果点云

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
   - 修改 `config.yaml` 中的质量检测参数：
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

### 输出目录结构

```
output/
├── config.json              # 转换矩阵配置（最重要）
├── raw_pointcloud.ply       # 原始点云（可选）
├── preprocessed.ply         # 预处理后点云（可选）
├── ground_points.ply        # 地面点云（可选）
├── non_ground_points.ply    # 非地面点云（可选）
└── hoop_region.ply          # 篮筐区域点云（可选）
```

### config.json 格式

```json
{
  "camera_to_world_matrix": [
    [r11, r12, r13, tx],
    [r21, r22, r23, ty],
    [r31, r32, r33, tz],
    [0,   0,   0,   1]
  ],
  "ground_plane": {
    "normal": [nx, ny, nz],
    "distance": d
  },
  "hoop_center": {
    "camera_coords": [x, y, z],
    "world_coords": [x, y, z]
  },
  "hoop_radius": 0.225,
  "timestamp": "2025-01-15T10:30:45"
}
```

## 故障排除

### 问题1: 相机初始化失败

**症状：**
```
错误: Orbbec相机初始化失败
```

**解决方案：**
1. 检查相机是否正确连接
2. 确认已安装相机SDK：`pip install pyorbbecsdk`
3. 检查USB端口是否为USB 3.0
4. Windows系统可能需要安装相机驱动

### 问题2: 采集的点云质量不达标

**症状：**
```
✗ X方向范围不足: 0.5m < 3.0m
```

**解决方案：**
1. 调整相机角度，确保视野足够大
2. 降低质量检测要求（修改 `config.yaml`）：
   ```yaml
   hoop_detection:
     height_range: [2.5, 3.5]  # 扩大高度范围
   ```

### 问题3: 地面检测失败

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

### 问题4: 篮筐检测失败

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

### 问题5: 深度帧为None（仅第2-10帧）

**症状：**
```
警告: 深度帧为None
警告: 第 2 帧采集失败，跳过
```

**解决方案：**
- 系统已自动处理：增加了帧间延迟和缓冲区清空
- 如果仍然失败，增加延迟时间（修改 `camera_capture.py` 第431行）

### 问题6: GUI无法启动

**症状：**
```
ImportError: No module named 'PyQt5'
```

**解决方案：**
```bash
pip install PyQt5
```

### 问题7: Open3D可视化窗口无响应

**解决方案：**
- 在 `config.yaml` 中禁用可视化：
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
with open('config.yaml', 'r', encoding='utf-8') as f:
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

**相机坐标系：**
- 原点：相机光心
- X轴：水平向右
- Y轴：垂直向下
- Z轴：相机正前方

**世界坐标系（标定后）：**
- 原点：篮筐中心在地面的投影点
- X轴：平行于地面，指向右侧
- Y轴：平行于地面，指向前方（相机方向在地面的投影）
- Z轴：垂直向上（与地面法向量一致）

### 算法说明

**地面检测（RANSAC）：**
1. 随机采样3个点拟合平面
2. 计算所有点到平面的距离
3. 统计内点数量
4. 迭代num_iterations次，选择内点最多的平面

**篮筐检测（圆形拟合）：**
1. 根据高度范围过滤点云
2. 投影到XY平面
3. 使用最小二乘法拟合圆形
4. 验证圆的半径是否符合标准

## 联系与支持

如有问题或建议，请通过以下方式联系：

- 提交 Issue
- 发送邮件至：[your-email@example.com]

## 许可证

MIT License

---

**版本：** 1.0.0
**最后更新：** 2025-01-15
