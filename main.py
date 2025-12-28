"""
投篮系统自动标定工具 - 主程序
用于深度相机外参标定和篮筐中心位置检测
"""

import open3d as o3d
import numpy as np
import yaml
import os
import sys
from pathlib import Path

# 导入预处理模块
from preprocess import preprocess_pointcloud

# 导入可视化和统计模块
from visualization import (PointCloudStatistics, compare_statistics,
                           visualize_comparison, plot_statistics_chart,
                           generate_report)


def load_config(config_path: str = "config.yaml") -> dict:
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


def load_pointcloud(file_path: str) -> o3d.geometry.PointCloud:
    """
    加载点云文件

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
    config = load_config("config.yaml")

    # 2. 加载点云数据
    print("\n步骤2: 加载点云数据")
    print("-" * 60)
    pcd_file = config['point_cloud']['input_file']
    pcd_original = load_pointcloud(pcd_file)

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

        output_dir = config['point_cloud']['output_dir']

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
    report_file = os.path.join(config['point_cloud']['output_dir'], "processing_report.txt")
    generate_report(stats_list, clusters, output_file=report_file)

    # 7. 可视化对比
    if config['visualization']['enabled']:
        print("\n步骤7: 可视化对比")
        print("-" * 60)

        # 3D可视化对比
        visualize_comparison(pcd_original, pcd_filtered, clusters)

        # 绘制统计图表
        print("\n绘制统计图表...")
        chart_file = os.path.join(config['point_cloud']['output_dir'], "statistics_chart.png")
        plot_statistics_chart(stats_list, save_path=chart_file)

    print("\n" + "="*60)
    print("预处理流程完成!")
    print("="*60)
    print(f"\n处理结果:")
    print(f"  原始点云: {stats_original.point_count:,} 点")
    print(f"  预处理后: {stats_filtered.point_count:,} 点 (保留率: {stats_filtered.point_count/stats_original.point_count*100:.2f}%)")
    print(f"  检测到簇: {len(clusters)} 个")
    print(f"\n下一步: 地面分割和篮筐检测（待实现）\n")


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
