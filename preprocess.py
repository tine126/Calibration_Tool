"""
点云预处理模块
包含降采样、噪声滤波和点云聚类功能
注意：点云单位为毫米(mm)，与ground_detection.py和hoop_detection.py保持一致
"""

import open3d as o3d
import numpy as np
from typing import List, Tuple


def downsample_pointcloud(pcd: o3d.geometry.PointCloud, voxel_size: float = 10) -> o3d.geometry.PointCloud:
    """
    使用体素网格滤波器对点云进行降采样

    Args:
        pcd: 输入点云
        voxel_size: 体素大小（毫米），默认10mm

    Returns:
        降采样后的点云
    """
    print(f"降采样前点云数量: {len(pcd.points)}")

    # 使用体素网格滤波器降采样
    pcd_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)

    print(f"降采样后点云数量: {len(pcd_downsampled.points)}")
    print(f"降采样率: {len(pcd_downsampled.points) / len(pcd.points) * 100:.2f}%")

    return pcd_downsampled


def filter_noise(pcd: o3d.geometry.PointCloud,
                 nb_neighbors: int = 50,
                 std_ratio: float = 1.0) -> o3d.geometry.PointCloud:
    """
    使用统计离群值移除算法过滤噪声点

    Args:
        pcd: 输入点云
        nb_neighbors: 每个点分析的近邻点数，默认50
        std_ratio: 距离阈值（倍标准差），默认1.0

    Returns:
        过滤后的点云
    """
    print(f"噪声滤波前点云数量: {len(pcd.points)}")

    # 统计离群值移除
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors,
                                              std_ratio=std_ratio)

    # 提取内点
    pcd_filtered = pcd.select_by_index(ind)

    print(f"噪声滤波后点云数量: {len(pcd_filtered.points)}")
    print(f"移除的噪声点数量: {len(pcd.points) - len(pcd_filtered.points)}")

    return pcd_filtered


def cluster_pointcloud(pcd: o3d.geometry.PointCloud,
                       eps: float = 100,
                       min_points: int = 10,
                       max_points: int = 100000) -> Tuple[List[o3d.geometry.PointCloud], np.ndarray]:
    """
    使用欧几里得聚类算法对点云进行聚类分割

    Args:
        pcd: 输入点云
        eps: 聚类距离阈值（毫米），默认100mm
        min_points: 形成一个簇所需的最小点数，默认10
        max_points: 单个簇的最大点数，默认100000

    Returns:
        clusters: 聚类后的点云列表
        labels: 每个点的聚类标签数组
    """
    print(f"聚类前点云数量: {len(pcd.points)}")

    # 使用DBSCAN进行欧几里得聚类
    labels = np.array(pcd.cluster_dbscan(eps=eps,
                                         min_points=min_points,
                                         print_progress=True))

    # 统计聚类结果
    max_label = labels.max()
    print(f"检测到 {max_label + 1} 个点云簇（不包括噪声点）")

    # 统计每个簇的点数
    clusters = []
    for i in range(max_label + 1):
        cluster_indices = np.where(labels == i)[0]
        cluster_size = len(cluster_indices)

        # 只保留点数在合理范围内的簇
        if min_points <= cluster_size <= max_points:
            cluster_pcd = pcd.select_by_index(cluster_indices)
            clusters.append(cluster_pcd)
            print(f"  簇 {i}: {cluster_size} 个点")
        else:
            print(f"  簇 {i}: {cluster_size} 个点 (已过滤)")

    # 统计噪声点
    noise_points = np.sum(labels == -1)
    print(f"噪声点数量: {noise_points}")
    print(f"有效簇数量: {len(clusters)}")

    return clusters, labels


def preprocess_pointcloud(pcd: o3d.geometry.PointCloud,
                          downsample_voxel_size: float = 10,
                          noise_nb_neighbors: int = 50,
                          noise_std_ratio: float = 1.0,
                          cluster_eps: float = 100,
                          cluster_min_points: int = 10,
                          cluster_max_points: int = 100000) -> Tuple[o3d.geometry.PointCloud,
                                                                       List[o3d.geometry.PointCloud],
                                                                       np.ndarray]:
    """
    完整的点云预处理流程

    Args:
        pcd: 输入原始点云
        downsample_voxel_size: 降采样体素大小（毫米）
        noise_nb_neighbors: 噪声滤波邻域点数
        noise_std_ratio: 噪声滤波距离阈值
        cluster_eps: 聚类距离阈值（毫米）
        cluster_min_points: 聚类最小点数
        cluster_max_points: 聚类最大点数

    Returns:
        processed_pcd: 预处理后的完整点云
        clusters: 聚类后的点云簇列表
        labels: 聚类标签
    """
    print("\n" + "="*50)
    print("开始点云预处理流程")
    print("="*50)

    # 步骤1：降采样
    print("\n步骤1: 降采样处理")
    print("-" * 50)
    pcd_downsampled = downsample_pointcloud(pcd, voxel_size=downsample_voxel_size)

    # 步骤2：噪声滤波
    print("\n步骤2: 噪声滤波")
    print("-" * 50)
    pcd_filtered = filter_noise(pcd_downsampled,
                                nb_neighbors=noise_nb_neighbors,
                                std_ratio=noise_std_ratio)

    # 步骤3：点云聚类
    print("\n步骤3: 点云聚类")
    print("-" * 50)
    clusters, labels = cluster_pointcloud(pcd_filtered,
                                         eps=cluster_eps,
                                         min_points=cluster_min_points,
                                         max_points=cluster_max_points)

    print("\n" + "="*50)
    print("点云预处理完成")
    print("="*50 + "\n")

    return pcd_filtered, clusters, labels


def visualize_clusters(clusters: List[o3d.geometry.PointCloud],
                       window_name: str = "点云聚类结果"):
    """
    可视化聚类结果，每个簇用不同颜色显示

    Args:
        clusters: 聚类后的点云列表
        window_name: 可视化窗口名称
    """
    print(f"\n正在可视化 {len(clusters)} 个点云簇...")

    # 为每个簇分配不同的颜色
    colors = np.random.rand(len(clusters), 3)

    for i, cluster in enumerate(clusters):
        cluster.paint_uniform_color(colors[i])

    # 可视化所有簇
    o3d.visualization.draw_geometries(clusters,
                                     window_name=window_name,
                                     width=1024,
                                     height=768)


if __name__ == "__main__":
    # 测试代码
    print("点云预处理模块")
    print("请通过主程序调用此模块")
