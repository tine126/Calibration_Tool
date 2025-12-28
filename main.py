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
from preprocess import preprocess_pointcloud, visualize_clusters


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

    # 可视化原始点云
    if config['visualization']['enabled']:
        print("\n显示原始点云...")
        o3d.visualization.draw_geometries([pcd_original],
                                         window_name="原始点云",
                                         width=1024,
                                         height=768,
                                         point_show_normal=False)

    # 3. 点云预处理
    print("\n步骤3: 点云预处理")
    print("-" * 60)

    # 获取预处理参数
    preprocess_config = config['preprocessing']

    # 执行预处理
    pcd_processed, clusters, labels = preprocess_pointcloud(
        pcd=pcd_original,
        downsample_voxel_size=preprocess_config['downsample']['voxel_size'],
        noise_nb_neighbors=preprocess_config['noise_filter']['nb_neighbors'],
        noise_std_ratio=preprocess_config['noise_filter']['std_ratio'],
        cluster_eps=preprocess_config['clustering']['eps'],
        cluster_min_points=preprocess_config['clustering']['min_points'],
        cluster_max_points=preprocess_config['clustering']['max_points']
    )

    # 4. 保存中间结果
    if config['output']['save_intermediate']:
        print("\n步骤4: 保存中间结果")
        print("-" * 60)

        output_dir = config['point_cloud']['output_dir']

        # 保存预处理后的点云
        processed_file = os.path.join(output_dir, "processed_pointcloud.ply")
        save_pointcloud(pcd_processed, processed_file)

        # 保存每个聚类簇
        for i, cluster in enumerate(clusters):
            cluster_file = os.path.join(output_dir, f"cluster_{i}.ply")
            save_pointcloud(cluster, cluster_file)

        print(f"所有中间结果已保存到: {output_dir}")

    # 5. 可视化预处理结果
    if config['visualization']['enabled']:
        print("\n步骤5: 可视化预处理结果")
        print("-" * 60)

        # 可视化预处理后的点云
        print("\n显示预处理后的点云...")
        o3d.visualization.draw_geometries([pcd_processed],
                                         window_name="预处理后的点云",
                                         width=1024,
                                         height=768,
                                         point_show_normal=False)

        # 可视化聚类结果
        if len(clusters) > 0:
            visualize_clusters(clusters, window_name="点云聚类结果")
        else:
            print("警告: 没有检测到有效的点云簇")

    print("\n" + "="*60)
    print("预处理流程完成!")
    print("="*60)
    print("\n下一步: 地面分割和篮筐检测（待实现）\n")


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
