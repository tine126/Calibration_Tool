"""
投篮系统自动标定工具 - 主程序
用于深度相机外参标定和篮筐中心位置检测
支持离线点云文件和实时相机采集两种模式
"""

import open3d as o3d
import numpy as np
import yaml
import os
import sys
import copy
import json
from pathlib import Path

# 导入预处理模块
from preprocess import preprocess_pointcloud

# 导入地面检测模块
from ground_detection import (detect_ground_plane, filter_ground_noise,
                               compute_ground_normal, compute_rotation_matrix,
                               apply_rotation_and_compute_plane,
                               visualize_ground_detection)

# 导入可视化和统计模块
from visualization import (PointCloudStatistics, compare_statistics,
                           visualize_comparison, plot_statistics_chart,
                           generate_report)

# 导入篮筐检测模块
from hoop_detection import detect_hoop, verify_hoop, visualize_hoop_detection


def load_config(config_path: str = "CTconfig.yaml") -> dict:
    """
    加载配置文件

    Args:
        config_path: 配置文件路径

    Returns:
        配置字典
    """
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        print(f"配置文件加载成功: {config_path}")
        return config
    except FileNotFoundError:
        print(f"错误: 配置文件 {config_path} 不存在")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"错误: 配置文件格式错误: {e}")
        sys.exit(1)


def load_pointcloud_from_file(file_path: str) -> o3d.geometry.PointCloud:
    """
    从文件加载点云

    Args:
        file_path: 点云文件路径

    Returns:
        点云对象
    """
    if not os.path.exists(file_path):
        print(f"错误: 点云文件 {file_path} 不存在")
        sys.exit(1)

    print(f"\n正在加载点云文件: {file_path}")

    try:
        pcd = o3d.io.read_point_cloud(file_path)
        print(f"点云加载成功!")
        print(f"  点云数量: {len(pcd.points)}")
        print(f"  是否有颜色: {pcd.has_colors()}")
        print(f"  是否有法向量: {pcd.has_normals()}")

        return pcd
    except Exception as e:
        print(f"错误: 无法加载点云文件: {e}")
        sys.exit(1)


def load_pointcloud_from_camera(config: dict) -> o3d.geometry.PointCloud:
    """
    从相机实时采集点云

    Args:
        config: 配置字典

    Returns:
        点云对象
    """
    print(f"\n正在从相机采集点云...")

    try:
        # 导入相机采集模块
        from camera_capture import CameraCapture

        camera_config = config['data_source']['realtime']

        # 初始化相机
        camera = CameraCapture(camera_config)

        # 采集点云
        pcd = camera.capture_pointcloud()

        if pcd is None:
            print(f"错误: 点云采集失败")
            sys.exit(1)

        print(f"点云采集成功!")
        print(f"  点云数量: {len(pcd.points)}")
        print(f"  是否有颜色: {pcd.has_colors()}")
        print(f"  是否有法向量: {pcd.has_normals()}")

        # 保存原始采集的点云
        if config['output'].get('save_raw_pointcloud', True):
            output_dir = config['output']['dir']
            os.makedirs(output_dir, exist_ok=True)

            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            raw_pcd_file = os.path.join(output_dir, f"raw_pointcloud_{timestamp}.ply")
            o3d.io.write_point_cloud(raw_pcd_file, pcd)
            print(f"  原始点云已保存: {raw_pcd_file}")

        return pcd

    except ImportError as e:
        print(f"错误: 无法导入相机采集模块: {e}")
        print(f"请确保已正确安装相机驱动和camera_capture.py模块")
        sys.exit(1)
    except Exception as e:
        print(f"错误: 点云采集失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


def get_pointcloud(config: dict) -> o3d.geometry.PointCloud:
    """
    根据配置获取点云数据（离线文件或实时采集）

    Args:
        config: 配置字典

    Returns:
        点云对象
    """
    data_source_config = config['data_source']
    mode = data_source_config['mode']

    print(f"\n数据源模式: {mode}")

    if mode == 'offline':
        # 离线模式：从文件加载
        file_path = data_source_config['offline']['input_file']
        return load_pointcloud_from_file(file_path)
    elif mode == 'realtime':
        # 实时模式：从相机采集
        return load_pointcloud_from_camera(config)
    else:
        print(f"错误: 未知的数据源模式: {mode}")
        print(f"支持的模式: 'offline' (离线文件), 'realtime' (实时采集)")
        sys.exit(1)


def save_pointcloud(pcd: o3d.geometry.PointCloud, file_path: str):
    """
    保存点云文件

    Args:
        pcd: 点云对象
        file_path: 保存路径
    """
    try:
        # 确保输出目录存在
        output_dir = os.path.dirname(file_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        o3d.io.write_point_cloud(file_path, pcd)
        print(f"点云已保存到: {file_path}")
    except Exception as e:
        print(f"警告: 无法保存点云文件: {e}")


def main():
    """
    主函数
    """
    print("\n" + "="*60)
    print("投篮系统自动标定工具")
    print("="*60)

    # 1. 加载配置文件
    print("\n步骤1: 加载配置文件")
    print("-" * 60)
    config = load_config("CTconfig.yaml")

    # 显示关键配置
    print(f"数据源模式: {config['data_source']['mode']}")
    print(f"可视化: {'启用' if config['visualization']['enabled'] else '禁用'}")
    print(f"保存中间结果: {'是' if config['output']['save_intermediate'] else '否'}")

    # 2. 获取点云数据（离线或实时）
    print("\n步骤2: 获取点云数据")
    print("-" * 60)
    pcd_original = get_pointcloud(config)

    # 收集统计信息
    stats_original = PointCloudStatistics("原始点云", pcd_original)
    stats_original.print_summary()

    # 3. 点云预处理
    print("\n步骤3: 点云预处理")
    print("-" * 60)

    # 获取预处理参数
    preprocess_config = config['preprocessing']

    # 3.1 降采样
    print("\n3.1 降采样处理...")
    from preprocess import downsample_pointcloud
    pcd_downsampled = downsample_pointcloud(pcd_original,
                                           voxel_size=preprocess_config['downsample']['voxel_size'])
    stats_downsampled = PointCloudStatistics("降采样后", pcd_downsampled)
    stats_downsampled.print_summary()

    # 3.2 噪声滤波
    print("\n3.2 噪声滤波...")
    from preprocess import filter_noise
    pcd_filtered = filter_noise(pcd_downsampled,
                                nb_neighbors=preprocess_config['noise_filter']['nb_neighbors'],
                                std_ratio=preprocess_config['noise_filter']['std_ratio'])
    stats_filtered = PointCloudStatistics("滤波后", pcd_filtered)
    stats_filtered.print_summary()

    # 3.3 点云聚类
    print("\n3.3 点云聚类...")
    from preprocess import cluster_pointcloud
    clusters, labels = cluster_pointcloud(pcd_filtered,
                                         eps=preprocess_config['clustering']['eps'],
                                         min_points=preprocess_config['clustering']['min_points'],
                                         max_points=preprocess_config['clustering']['max_points'])

    # 统计信息列表
    stats_list = [stats_original, stats_downsampled, stats_filtered]

    # 4. 数据对比分析
    print("\n步骤4: 数据对比分析")
    print("-" * 60)
    compare_statistics(stats_list)

    # 5. 保存中间结果
    if config['output']['save_intermediate']:
        print("\n步骤5: 保存中间结果")
        print("-" * 60)

        output_dir = config['output']['dir']

        # 保存预处理后的点云
        processed_file = os.path.join(output_dir, "processed_pointcloud.ply")
        save_pointcloud(pcd_filtered, processed_file)

        # 保存每个聚类簇
        for i, cluster in enumerate(clusters):
            cluster_file = os.path.join(output_dir, f"cluster_{i}.ply")
            save_pointcloud(cluster, cluster_file)

        print(f"所有中间结果已保存到: {output_dir}")

    # 6. 生成处理报告
    print("\n步骤6: 生成处理报告")
    print("-" * 60)
    report_file = os.path.join(config['output']['dir'], "processing_report.txt")
    generate_report(stats_list, clusters, output_file=report_file)

    # 7. 可视化对比
    if config['visualization']['enabled']:
        print("\n步骤7: 可视化对比")
        print("-" * 60)

        # 3D可视化对比
        visualize_comparison(pcd_original, pcd_filtered, clusters)

        # 绘制统计图表
        print("\n绘制统计图表...")
        chart_file = os.path.join(config['output']['dir'], "statistics_chart.png")
        plot_statistics_chart(stats_list, save_path=chart_file)

    # 8. 地面检测
    print("\n步骤8: 地面检测")
    print("-" * 60)

    ground_config = config['ground_detection']

    # 8.1 检测地面平面
    plane_model, ground_inliers, ground_stats = detect_ground_plane(
        pcd_filtered,
        distance_threshold=ground_config['distance_threshold'],
        ransac_n=ground_config['ransac_n'],
        num_iterations=ground_config['num_iterations'],
        random_seed=ground_config.get('random_seed', 42)  # 使用配置文件中的随机数种子，默认42
    )

    # 8.2 过滤地面噪声
    valid_ground_inliers, noise_inliers, noise_stats = filter_ground_noise(
        pcd_filtered,
        ground_inliers,
        plane_model,
        distance_threshold=ground_config['noise_filter']['distance_threshold'],
        nb_neighbors=ground_config['noise_filter']['nb_neighbors'],
        std_ratio=ground_config['noise_filter']['std_ratio']
    )

    # 8.3 计算旋转矩阵
    ground_normal = compute_ground_normal(plane_model)
    rotation_matrix = compute_rotation_matrix(ground_normal)

    # 8.4 应用旋转矩阵并计算旋转后的地面平面方程
    rotated_pcd, ground_plane_equation, translation_vector, rotation_stats = apply_rotation_and_compute_plane(
        pcd_filtered,
        valid_ground_inliers,
        rotation_matrix
    )

    # 8.5 保存地面检测结果
    ground_pcd = pcd_filtered.select_by_index(valid_ground_inliers)
    ground_file = os.path.join(output_dir, "ground_plane.ply")
    save_pointcloud(ground_pcd, ground_file)

    # 保存旋转后的点云
    rotated_pcd_file = os.path.join(output_dir, "rotated_pointcloud.ply")
    o3d.io.write_point_cloud(rotated_pcd_file, rotated_pcd)
    print(f"旋转后点云已保存: {rotated_pcd_file}")

    # 8.6 保存旋转矩阵和地面平面方程到config.json
    config_output_file = os.path.join(output_dir, config['output']['config_file'])

    # 构建输出配置
    output_config = {
        "rotation_matrix": rotation_matrix.tolist(),  # 转换为列表以便JSON序列化
        "ground_normal": ground_normal.tolist(),
        "ground_plane_equation": ground_plane_equation.tolist(),  # 旋转后的地面平面方程
        "translation_vector": translation_vector.tolist(),  # 平移向量
        "metadata": {
            "ground_detection": {
                "ground_point_count": noise_stats['valid_count'],
                "noise_point_count": noise_stats['noise_count'],
                "ground_z_mean_mm": float(ground_stats['z_mean']),
                "ground_z_angle_deg": float(ground_stats['z_angle'])
            },
            "rotated_ground": {
                "z_mean_mm": float(rotation_stats['z_mean']),
                "z_median_mm": float(rotation_stats['z_median']),
                "z_std_mm": float(rotation_stats['z_std']),
                "translation_z_mm": float(rotation_stats['translation_vector'][2]),
                "plane_normal_z_angle_deg": float(rotation_stats['normal_z_angle_deg']),
                "distance_to_plane_mean_mm": float(rotation_stats['distance_to_plane_mean']),
                "distance_to_plane_std_mm": float(rotation_stats['distance_to_plane_std'])
            },
            "timestamp": str(np.datetime64('now'))
        }
    }

    import json
    with open(config_output_file, 'w', encoding='utf-8') as f:
        json.dump(output_config, f, indent=2, ensure_ascii=False)

    print(f"\n配置文件已保存: {config_output_file}")
    print(f"  包含: 旋转矩阵(R), 地面法向量, 地面平面方程")

    # 同时保存文本格式的详细信息（便于查看）
    rotation_matrix_file = os.path.join(output_dir, "calibration_result.txt")
    with open(rotation_matrix_file, 'w', encoding='utf-8') as f:
        f.write("自动标定结果\n")
        f.write("="*80 + "\n\n")

        f.write("1. 旋转矩阵 R (3x3):\n")
        for i in range(3):
            f.write(f"  [{rotation_matrix[i, 0]:+.10f}, {rotation_matrix[i, 1]:+.10f}, {rotation_matrix[i, 2]:+.10f}]\n")

        f.write(f"\n2. Y轴180度旋转矩阵 (已应用):\n")
        f.write(f"  [-1.0, 0.0, 0.0]\n")
        f.write(f"  [ 0.0, 1.0, 0.0]\n")
        f.write(f"  [ 0.0, 0.0,-1.0]\n")

        f.write(f"\n3. 平移向量 T (毫米):\n")
        f.write(f"  [{translation_vector[0]:.3f}, {translation_vector[1]:.3f}, {translation_vector[2]:.3f}]\n")
        f.write(f"  说明: 使原点Z坐标位于地面上\n")

        f.write(f"\n4. 地面法向量（旋转前）:\n")
        f.write(f"  [{ground_normal[0]:.10f}, {ground_normal[1]:.10f}, {ground_normal[2]:.10f}]\n")

        f.write(f"\n5. 地面平面方程（最终坐标系，单位：毫米）:\n")
        f.write(f"  {ground_plane_equation[0]:.10f} * x + {ground_plane_equation[1]:.10f} * y + {ground_plane_equation[2]:.10f} * z + {ground_plane_equation[3]:.10f} = 0\n")
        f.write(f"  简化形式: z ≈ {-ground_plane_equation[3]/ground_plane_equation[2]:.3f} mm ({-ground_plane_equation[3]/ground_plane_equation[2]/1000:.6f} m)\n")

        f.write(f"\n6. 变换流程:\n")
        f.write(f"  步骤1: 应用旋转矩阵 R (地面对齐到XY平面)\n")
        f.write(f"  步骤2: 应用Y轴180度旋转\n")
        f.write(f"  步骤3: 应用平移向量 T (原点Z移至地面)\n")

        f.write(f"\n7. 地面统计（旋转前）:\n")
        f.write(f"  地面点数: {noise_stats['valid_count']:,}\n")
        f.write(f"  噪声点数: {noise_stats['noise_count']:,}\n")
        f.write(f"  地面Z均值: {ground_stats['z_mean']:.3f} mm\n")
        f.write(f"  法向量与Z轴夹角: {ground_stats['z_angle']:.2f}°\n")

        f.write(f"\n8. 地面统计（最终坐标系）:\n")
        f.write(f"  地面Z均值: {rotation_stats['z_mean']:.3f} mm\n")
        f.write(f"  地面Z中位数: {rotation_stats['z_median']:.3f} mm\n")
        f.write(f"  地面Z标准差: {rotation_stats['z_std']:.3f} mm\n")
        f.write(f"  法向量与Z轴夹角: {rotation_stats['normal_z_angle_deg']:.4f}°\n")
        f.write(f"  点到平面距离均值: {rotation_stats['distance_to_plane_mean']:.3f} mm\n")
        f.write(f"  点到平面距离标准差: {rotation_stats['distance_to_plane_std']:.3f} mm\n")

    print(f"标定结果详细信息已保存: {rotation_matrix_file}")

    # 8.7 可视化地面检测结果
    if config['visualization']['enabled']:
        visualize_ground_detection(pcd_filtered, valid_ground_inliers, noise_inliers,
                                   title="地面检测结果")

    # 8.8 可视化旋转后过滤地面的点云场景
    if config['visualization']['enabled']:
        print("\n步骤8.8: 可视化旋转后过滤地面的点云场景")
        print("-" * 60)

        # 获取非地面点的索引
        all_indices = np.arange(len(pcd_filtered.points))
        non_ground_indices = np.setdiff1d(all_indices, valid_ground_inliers)

        # 提取非地面点云
        non_ground_pcd = pcd_filtered.select_by_index(non_ground_indices)

        # 应用完整变换到非地面点云
        # 步骤1: 地面对齐旋转
        rotated_non_ground_pcd = copy.deepcopy(non_ground_pcd)
        rotated_non_ground_pcd.rotate(rotation_matrix, center=(0, 0, 0))

        # 步骤2: Y轴180度旋转
        rotation_y_180 = np.array([
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])
        rotated_non_ground_pcd.rotate(rotation_y_180, center=(0, 0, 0))

        # 步骤3: 应用平移
        rotated_non_ground_pcd.translate(translation_vector)

        print(f"非地面点数: {len(non_ground_pcd.points):,} ({len(non_ground_pcd.points)/len(pcd_filtered.points)*100:.2f}%)")

        # 准备可视化
        geometries = []

        # 非地面点云 - 使用原始颜色或灰色
        rotated_non_ground_colored = copy.deepcopy(rotated_non_ground_pcd)
        if rotated_non_ground_pcd.has_colors():
            # 如果有颜色，保持原色
            pass
        else:
            # 否则设为灰色
            rotated_non_ground_colored.paint_uniform_color([0.6, 0.6, 0.6])
        geometries.append(rotated_non_ground_colored)

        # 添加坐标系
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0, 0, 0])
        geometries.append(coord_frame)

        # 添加XYZ轴标签（使用球体标记）
        # X轴标签 - 红色球体
        x_label_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=80)
        x_label_sphere.translate([1200, 0, 0])
        x_label_sphere.paint_uniform_color([1.0, 0.0, 0.0])
        geometries.append(x_label_sphere)

        # Y轴标签 - 绿色球体
        y_label_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=80)
        y_label_sphere.translate([0, 1200, 0])
        y_label_sphere.paint_uniform_color([0.0, 1.0, 0.0])
        geometries.append(y_label_sphere)

        # Z轴标签 - 蓝色球体
        z_label_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=80)
        z_label_sphere.translate([0, 0, 1200])
        z_label_sphere.paint_uniform_color([0.0, 0.0, 1.0])
        geometries.append(z_label_sphere)

        # 添加地面平面参考（半透明网格）
        ground_z = -ground_plane_equation[3] / ground_plane_equation[2]

        # 创建地面参考平面网格
        plane_size = 5000  # 5000mm = 5m
        plane_grid = o3d.geometry.TriangleMesh()

        # 创建一个简单的矩形平面
        vertices = np.array([
            [-plane_size, -plane_size, ground_z],
            [plane_size, -plane_size, ground_z],
            [plane_size, plane_size, ground_z],
            [-plane_size, plane_size, ground_z]
        ])

        triangles = np.array([
            [0, 1, 2],
            [0, 2, 3]
        ])

        plane_grid.vertices = o3d.utility.Vector3dVector(vertices)
        plane_grid.triangles = o3d.utility.Vector3iVector(triangles)
        plane_grid.paint_uniform_color([0.3, 0.8, 0.3])  # 绿色
        plane_grid.compute_vertex_normals()

        geometries.append(plane_grid)

        print(f"\n显示可视化结果...")
        print(f"颜色说明:")
        print(f"  灰色 = 变换后的非地面点云（已应用旋转+Y轴180°旋转+平移）")
        print(f"  绿色平面 = 地面参考平面 (z = {ground_z:.1f} mm)")
        print(f"  坐标系 = 红色箭头(X轴), 绿色箭头(Y轴), 蓝色箭头(Z轴)")
        print(f"  坐标轴标记 = 各轴末端的彩色球体")

        o3d.visualization.draw_geometries(
            geometries,
            window_name=f"变换后场景（已过滤地面） | 地面高度={ground_z:.1f}mm",
            width=1600,
            height=900
        )

    # 9. 篮筐检测
    print("\n步骤9: 篮筐检测")
    print("-" * 60)

    hoop_config = config['hoop_detection']

    hoop_info = detect_hoop(
        pcd_filtered,
        rotation_matrix,
        ground_plane_equation,
        translation_vector,
        standard_diameter=hoop_config['standard_diameter'] * 1000,  # 转换为毫米
        diameter_tolerance=hoop_config['diameter_tolerance'] * 1000,  # 转换为毫米
        height_range=(hoop_config['height_range'][0] * 1000, hoop_config['height_range'][1] * 1000),  # 转换为毫米
        random_seed=hoop_config.get('random_seed', 42)  # 使用配置文件中的随机数种子，默认42
    )

    if hoop_info is None:
        print(f"\n警告: 未能检测到篮筐")
        print(f"可能的原因：")
        print(f"  1. 点云中没有篮筐")
        print(f"  2. 篮筐直径不在标准范围内")
        print(f"  3. 篮筐高度不符合要求")
        print(f"  4. RANSAC参数需要调整")
        print(f"\n将继续完成其他步骤...")
    else:
        # 9.1 验证篮筐
        is_valid, messages = verify_hoop(
            hoop_info,
            standard_diameter=hoop_config['standard_diameter'] * 1000,
            diameter_tolerance=hoop_config['diameter_tolerance'] * 1000
        )

        # 9.2 保存篮筐检测结果
        hoop_report_file = os.path.join(output_dir, "hoop_detection_report.txt")

        with open(hoop_report_file, 'w', encoding='utf-8') as f:
            f.write("="*80 + "\n")
            f.write("篮筐检测报告\n")
            f.write("="*80 + "\n\n")

            f.write("篮筐中心位置（最终坐标系，毫米）:\n")
            f.write(f"  X = {hoop_info['center_3d'][0]:.3f} mm\n")
            f.write(f"  Y = {hoop_info['center_3d'][1]:.3f} mm\n")
            f.write(f"  Z = {hoop_info['center_3d'][2]:.3f} mm\n\n")

            f.write("篮筐中心位置（最终坐标系，米）:\n")
            f.write(f"  X = {hoop_info['center_3d'][0]/1000:.6f} m\n")
            f.write(f"  Y = {hoop_info['center_3d'][1]/1000:.6f} m\n")
            f.write(f"  Z = {hoop_info['center_3d'][2]/1000:.6f} m\n\n")

            f.write("篮筐参数:\n")
            f.write(f"  直径: {hoop_info['diameter']:.3f} mm ({hoop_info['diameter']/1000:.6f} m)\n")
            f.write(f"  半径: {hoop_info['radius']:.3f} mm ({hoop_info['radius']/1000:.6f} m)\n")
            f.write(f"  篮筐平面高度: {hoop_info['hoop_plane_z']:.3f} mm ({hoop_info['hoop_plane_z']/1000:.6f} m)\n")
            f.write(f"  离地高度: {hoop_info['height_above_ground']:.3f} mm ({hoop_info['height_above_ground']/1000:.6f} m)\n")
            f.write(f"  直径误差: {hoop_info['diameter_error']:.3f} mm ({hoop_info['diameter_error_ratio']*100:.2f}%)\n")
            f.write(f"  高度误差: {hoop_info['height_error']:.3f} mm\n")
            f.write(f"  拟合误差: {hoop_info['fit_error']:.3f} mm\n")
            f.write(f"  Z标准差: {hoop_info['z_std']:.3f} mm\n")
            f.write(f"  内点数: {hoop_info['inlier_count']}\n")
            f.write(f"  内点比例: {hoop_info['inlier_ratio']*100:.2f}%\n")
            f.write(f"  候选点数: {hoop_info['candidate_count']}\n\n")

            f.write("篮筐平面方程（最终坐标系）:\n")
            f.write(f"  {hoop_info['hoop_plane_equation'][0]:.6f}*x + {hoop_info['hoop_plane_equation'][1]:.6f}*y + {hoop_info['hoop_plane_equation'][2]:.6f}*z + {hoop_info['hoop_plane_equation'][3]:.6f} = 0\n")
            f.write(f"  简化形式: z = {hoop_info['hoop_plane_z']:.3f} mm\n\n")

            f.write("验证结果:\n")
            f.write(f"  状态: {'通过' if is_valid else '失败'}\n")
            for msg in messages:
                f.write(f"  {msg}\n")

            f.write("\n" + "="*80 + "\n")

        print(f"\n篮筐检测报告已保存: {hoop_report_file}")

        # 9.3 更新config.json，添加篮筐中心位置
        with open(config_output_file, 'r', encoding='utf-8') as f:
            output_config = json.load(f)

        output_config['hoop_center'] = hoop_info['center_3d'].tolist()
        output_config['metadata']['hoop_detection'] = {
            'diameter_mm': float(hoop_info['diameter']),
            'height_above_ground_mm': float(hoop_info['height_above_ground']),
            'fit_error_mm': float(hoop_info['fit_error']),
            'z_std_mm': float(hoop_info['z_std']),
            'inlier_count': int(hoop_info['inlier_count']),
            'inlier_ratio': float(hoop_info['inlier_ratio']),
            'validation_passed': is_valid
        }

        with open(config_output_file, 'w', encoding='utf-8') as f:
            json.dump(output_config, f, indent=2, ensure_ascii=False)

        print(f"篮筐中心位置已添加到配置文件: {config_output_file}")

        # 9.4 可视化篮筐检测结果
        if config['visualization']['enabled']:
            visualize_hoop_detection(pcd_filtered, rotation_matrix, translation_vector,
                                   hoop_info, ground_plane_equation)

    print("\n" + "="*60)
    print("自动标定流程完成!")
    print("="*60)
    print(f"\n处理结果:")
    print(f"  原始点云: {stats_original.point_count:,} 点")
    print(f"  预处理后: {stats_filtered.point_count:,} 点 (保留率: {stats_filtered.point_count/stats_original.point_count*100:.2f}%)")
    print(f"  检测到簇: {len(clusters)} 个")
    print(f"  地面点数: {noise_stats['valid_count']:,} 点 ({noise_stats['valid_count']/stats_filtered.point_count*100:.2f}%)")
    print(f"\n标定结果:")
    print(f"  旋转前地面法向量与Z轴夹角: {ground_stats['z_angle']:.2f}°")
    print(f"  旋转后地面法向量与Z轴夹角: {rotation_stats['normal_z_angle_deg']:.4f}°")
    print(f"  地面平面方程: z = {-ground_plane_equation[3]/ground_plane_equation[2]:.1f} mm (固定为 z=0)")
    print(f"  实际地面Z均值偏差: {rotation_stats['z_mean']:.1f} mm")
    print(f"  实际地面Z中位数偏差: {rotation_stats['z_median']:.1f} mm")

    if hoop_info is not None:
        print(f"\n篮筐检测结果:")
        print(f"  篮筐中心位置: ({hoop_info['center_3d'][0]/1000:.3f}, {hoop_info['center_3d'][1]/1000:.3f}, {hoop_info['center_3d'][2]/1000:.3f}) m")
        print(f"  离地高度: {hoop_info['height_above_ground']/1000:.3f} m")
        print(f"  直径: {hoop_info['diameter']:.1f} mm ({hoop_info['diameter']/1000:.3f} m)")
        print(f"  验证状态: {'通过 ✓' if is_valid else '失败 ✗'}")

    print(f"\n输出文件:")
    print(f"  配置文件: {config_output_file}")
    print(f"  标定结果: {rotation_matrix_file}")
    print(f"  旋转后点云: {rotated_pcd_file}")
    if hoop_info is not None:
        print(f"  篮筐检测报告: {hoop_report_file}")
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
