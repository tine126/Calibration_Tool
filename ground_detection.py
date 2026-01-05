"""
地面检测模块
基于最大内点数的RANSAC算法检测地面平面，并进行噪声过滤
注意：点云单位为毫米(mm)
"""

import open3d as o3d
import numpy as np
from typing import Tuple, Dict
import copy


def detect_ground_plane(pcd: o3d.geometry.PointCloud,
                        distance_threshold: float = 200,
                        ransac_n: int = 3,
                        num_iterations: int = 10000,
                        random_seed: int = 42) -> Tuple[np.ndarray, np.ndarray, Dict]:
    """
    基于最大内点数检测地面平面
    适用于相机倾斜安装的情况，选择点数最多的平面作为地面

    Args:
        pcd: 输入点云（单位：毫米）
        distance_threshold: RANSAC距离阈值（毫米），默认200mm
        ransac_n: RANSAC最小点数，默认3
        num_iterations: RANSAC最大迭代次数，默认10000
        random_seed: 随机数种子，默认42（设置后结果可复现）

    Returns:
        plane_model: 平面模型参数 [a, b, c, d]，平面方程为 ax + by + cz + d = 0
        ground_inliers: 地面点的索引数组
        stats: 统计信息字典
    """
    print(f"\n{'='*80}")
    print("地面平面检测（基于最大内点数）")
    print(f"{'='*80}")
    print(f"输入点云数量: {len(pcd.points):,}")
    print(f"RANSAC距离阈值: {distance_threshold} mm ({distance_threshold/1000:.3f} m)")
    print(f"RANSAC迭代次数: {num_iterations}")
    print(f"随机数种子: {random_seed}")

    # 添加诊断：检查点云的Z值分布
    points = np.asarray(pcd.points)
    z_values = points[:, 2]
    print(f"\n点云Z值分布诊断:")
    print(f"  Z最小值: {z_values.min():.1f} mm ({z_values.min()/1000:.3f} m)")
    print(f"  Z最大值: {z_values.max():.1f} mm ({z_values.max()/1000:.3f} m)")
    print(f"  Z均值: {z_values.mean():.1f} mm ({z_values.mean()/1000:.3f} m)")
    print(f"  Z标准差: {z_values.std():.1f} mm")
    print(f"  Z跨度: {z_values.max() - z_values.min():.1f} mm ({(z_values.max() - z_values.min())/1000:.3f} m)")

    # 检查是否有明显的高度分层（地面vs篮筐）
    z_histogram, z_bins = np.histogram(z_values, bins=50)
    print(f"  Z值分布: 分为50个区间，最高峰有 {z_histogram.max():,} 个点")

    # 设置随机数种子以保证结果可复现
    o3d.utility.random.seed(random_seed)

    # 执行RANSAC平面检测
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations
    )

    # 计算统计信息
    points = np.asarray(pcd.points)
    ground_points = points[inliers]

    inlier_ratio = len(inliers) / len(points)

    # 计算法向量
    normal = np.array([plane_model[0], plane_model[1], plane_model[2]])
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal

    z_angle = np.degrees(np.arccos(np.clip(normal[2], -1, 1)))

    print(f"\n检测结果:")
    print(f"  地面点数: {len(inliers):,} ({inlier_ratio*100:.2f}%)")
    print(f"  地面Z值范围: [{ground_points[:, 2].min():.1f}, {ground_points[:, 2].max():.1f}] mm")
    print(f"  地面Z值均值: {ground_points[:, 2].mean():.1f} mm ({ground_points[:, 2].mean()/1000:.3f} m)")
    print(f"  地面Z值标准差: {ground_points[:, 2].std():.1f} mm")
    print(f"  地面法向量: ({normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f})")
    print(f"  与Z轴夹角: {z_angle:.2f}°")

    # 验证检测到的平面是否为地面
    # 地面的法向量应该接近垂直方向（与Z轴夹角应该很小）
    # 对于倾斜安装的相机，这个角度会更大
    max_angle_threshold = 60.0  # 最大允许角度（度）- 适用于相机倾斜45°左右的情况

    # 额外检查：地面点数不应该是100%（应该有非地面物体）
    if inlier_ratio > 0.95:  # 如果超过95%的点都是地面，可能有问题
        print(f"\n⚠️  警告: 地面点占比过高 ({inlier_ratio*100:.2f}%)")
        print(f"  这可能意味着：")
        print(f"  1. 点云中只有单一平面（预处理聚类可能只选择了一个簇）")
        print(f"  2. RANSAC距离阈值过大")
        print(f"  3. 相机视野中主要是地面，缺少其他物体")
        print(f"  建议检查预处理步骤中的聚类结果\n")

    if z_angle > max_angle_threshold:
        print(f"\n❌ 错误: 检测到的平面与Z轴夹角过大 ({z_angle:.2f}° > {max_angle_threshold}°)")
        print(f"  平面法向量: ({normal[0]:.6f}, {normal[1]:.6f}, {normal[2]:.6f})")
        print(f"  这可能是墙壁或篮板，而不是地面！")
        print(f"\n建议解决方案：")
        print(f"  1. 如果相机倾斜角度更大，可以增加max_angle_threshold")
        print(f"  2. 检查点云是否包含地面区域")
        print(f"  3. 减小RANSAC距离阈值（当前: {distance_threshold}mm）")
        raise RuntimeError(f"检测到的平面不是地面（与Z轴夹角: {z_angle:.2f}°，法向量Z分量: {normal[2]:.3f}）")

    print(f"  ✓ 平面方向验证通过（与Z轴夹角 {z_angle:.2f}° < {max_angle_threshold}°）")
    print(f"  平面方程: {plane_model[0]:.6f}*x + {plane_model[1]:.6f}*y + {plane_model[2]:.6f}*z + {plane_model[3]:.6f} = 0")
    print(f"{'='*80}\n")

    stats = {
        'plane_model': plane_model,
        'normal': normal,
        'z_angle': z_angle,
        'inlier_count': len(inliers),
        'inlier_ratio': inlier_ratio,
        'z_min': ground_points[:, 2].min(),
        'z_max': ground_points[:, 2].max(),
        'z_mean': ground_points[:, 2].mean(),
        'z_std': ground_points[:, 2].std()
    }

    return plane_model, inliers, stats


def filter_ground_noise(pcd: o3d.geometry.PointCloud,
                        ground_inliers: np.ndarray,
                        plane_model: np.ndarray,
                        distance_threshold: float = 100,
                        nb_neighbors: int = 20,
                        std_ratio: float = 2.0) -> Tuple[np.ndarray, np.ndarray, Dict]:
    """
    过滤地面点云中的噪声点

    Args:
        pcd: 完整点云
        ground_inliers: 地面点的索引
        plane_model: 地面平面模型
        distance_threshold: 点到平面距离阈值（毫米），超过此距离视为噪声，默认100mm
        nb_neighbors: 统计离群值检测的邻域点数，默认20
        std_ratio: 统计离群值检测的标准差倍数，默认2.0

    Returns:
        valid_ground_inliers: 有效地面点的索引（相对于完整点云）
        noise_inliers: 噪声点的索引（相对于完整点云）
        stats: 统计信息字典
    """
    print(f"\n{'='*80}")
    print("地面噪声过滤")
    print(f"{'='*80}")
    print(f"原始地面点数: {len(ground_inliers):,}")
    print(f"过滤方法: 距离阈值 + 统计离群值检测")
    print(f"  距离阈值: {distance_threshold} mm")
    print(f"  统计检测: nb_neighbors={nb_neighbors}, std_ratio={std_ratio}")

    # 提取地面点云
    ground_pcd = pcd.select_by_index(ground_inliers)
    ground_points = np.asarray(ground_pcd.points)

    # 方法1: 计算每个点到平面的距离
    a, b, c, d = plane_model
    distances = np.abs(
        a * ground_points[:, 0] +
        b * ground_points[:, 1] +
        c * ground_points[:, 2] +
        d
    ) / np.sqrt(a**2 + b**2 + c**2)

    # 距离超过阈值的为噪声
    distance_noise_mask = distances > distance_threshold

    # 方法2: 统计离群值检测
    cl, stat_inliers = ground_pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )

    stat_outliers_mask = np.ones(len(ground_points), dtype=bool)
    stat_outliers_mask[stat_inliers] = False

    # 综合判断：距离超阈值 OR 统计离群点 = 噪声
    noise_mask = distance_noise_mask | stat_outliers_mask
    valid_mask = ~noise_mask

    # 转换为相对于完整点云的索引
    # ground_inliers 是索引数组，需要先转换为numpy数组再使用布尔掩码
    ground_inliers_array = np.asarray(ground_inliers)
    valid_ground_inliers = ground_inliers_array[valid_mask]
    noise_inliers = ground_inliers_array[noise_mask]

    print(f"\n过滤结果:")
    print(f"  有效地面点数: {len(valid_ground_inliers):,} ({len(valid_ground_inliers)/len(ground_inliers)*100:.2f}%)")
    print(f"  噪声点数: {len(noise_inliers):,} ({len(noise_inliers)/len(ground_inliers)*100:.2f}%)")
    print(f"  点到平面距离 - 均值: {distances[valid_mask].mean():.2f} mm, 标准差: {distances[valid_mask].std():.2f} mm")
    print(f"{'='*80}\n")

    stats = {
        'valid_count': len(valid_ground_inliers),
        'noise_count': len(noise_inliers),
        'valid_ratio': len(valid_ground_inliers) / len(ground_inliers),
        'noise_ratio': len(noise_inliers) / len(ground_inliers),
        'distance_mean': distances[valid_mask].mean(),
        'distance_std': distances[valid_mask].std()
    }

    return valid_ground_inliers, noise_inliers, stats


def compute_ground_normal(plane_model: np.ndarray) -> np.ndarray:
    """
    从平面模型计算地面法向量

    Args:
        plane_model: 平面模型参数 [a, b, c, d]

    Returns:
        normal: 归一化的地面法向量 [nx, ny, nz]
    """
    # 平面方程 ax + by + cz + d = 0 的法向量为 (a, b, c)
    normal = np.array([plane_model[0], plane_model[1], plane_model[2]])

    # 归一化法向量
    normal = normal / np.linalg.norm(normal)

    # 确保法向量指向上方（z分量为正）
    if normal[2] < 0:
        normal = -normal

    return normal


def compute_rotation_matrix(ground_normal: np.ndarray,
                            target_normal: np.ndarray = None) -> np.ndarray:
    """
    计算将地面法向量旋转到目标法向量的旋转矩阵

    Args:
        ground_normal: 当前地面法向量
        target_normal: 目标法向量，默认为 [0, 0, 1] (Z轴正方向，即XY平面)

    Returns:
        rotation_matrix: 3x3旋转矩阵
    """
    if target_normal is None:
        target_normal = np.array([0.0, 0.0, 1.0])

    print(f"\n{'='*80}")
    print("计算旋转矩阵")
    print(f"{'='*80}")
    print(f"当前地面法向量: ({ground_normal[0]:.6f}, {ground_normal[1]:.6f}, {ground_normal[2]:.6f})")
    print(f"目标法向量 (XY平面): ({target_normal[0]:.6f}, {target_normal[1]:.6f}, {target_normal[2]:.6f})")

    # 计算旋转轴（叉积）
    rotation_axis = np.cross(ground_normal, target_normal)
    rotation_axis_norm = np.linalg.norm(rotation_axis)

    # 如果法向量已经对齐，返回单位矩阵
    if rotation_axis_norm < 1e-6:
        print("\n地面法向量已经与目标法向量对齐，无需旋转")
        print("旋转矩阵: 单位矩阵")
        print(f"{'='*80}\n")
        return np.eye(3)

    # 归一化旋转轴
    rotation_axis = rotation_axis / rotation_axis_norm

    # 计算旋转角度（点积）
    cos_angle = np.dot(ground_normal, target_normal)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)  # 防止数值误差
    angle = np.arccos(cos_angle)

    print(f"\n旋转参数:")
    print(f"  旋转轴: ({rotation_axis[0]:.6f}, {rotation_axis[1]:.6f}, {rotation_axis[2]:.6f})")
    print(f"  旋转角度: {np.degrees(angle):.2f}° ({angle:.6f} rad)")

    # 使用罗德里格斯公式计算旋转矩阵
    # R = I + sin(θ)K + (1-cos(θ))K²
    # 其中K是旋转轴的反对称矩阵
    K = np.array([
        [0, -rotation_axis[2], rotation_axis[1]],
        [rotation_axis[2], 0, -rotation_axis[0]],
        [-rotation_axis[1], rotation_axis[0], 0]
    ])

    rotation_matrix = (np.eye(3) +
                      np.sin(angle) * K +
                      (1 - np.cos(angle)) * np.dot(K, K))

    print(f"\n旋转矩阵 R (3x3):")
    for i in range(3):
        print(f"  [{rotation_matrix[i, 0]:+.6f}, {rotation_matrix[i, 1]:+.6f}, {rotation_matrix[i, 2]:+.6f}]")

    # 验证旋转矩阵
    rotated_normal = np.dot(rotation_matrix, ground_normal)
    print(f"\n验证: 旋转后的法向量: ({rotated_normal[0]:.6f}, {rotated_normal[1]:.6f}, {rotated_normal[2]:.6f})")
    print(f"与目标法向量的差异: {np.linalg.norm(rotated_normal - target_normal):.8f}")
    print(f"{'='*80}\n")

    return rotation_matrix


def apply_rotation_and_compute_plane(pcd: o3d.geometry.PointCloud,
                                     ground_inliers: np.ndarray,
                                     rotation_matrix: np.ndarray) -> Tuple[o3d.geometry.PointCloud, np.ndarray, np.ndarray, Dict]:
    """
    应用旋转矩阵到点云，并计算旋转后地面的平面方程
    包含Y轴180度旋转和平移使原点Z在地面上

    Args:
        pcd: 完整点云
        ground_inliers: 地面点索引
        rotation_matrix: 地面对齐旋转矩阵

    Returns:
        rotated_pcd: 旋转后的完整点云
        ground_plane_equation: 旋转后地面的平面方程 [a, b, c, d]
        translation_vector: 平移向量 [tx, ty, tz]
        stats: 统计信息字典
    """
    print(f"\n{'='*80}")
    print("应用旋转矩阵并计算地面平面方程")
    print(f"{'='*80}")

    # 1. 复制点云并应用地面对齐旋转
    rotated_pcd = copy.deepcopy(pcd)
    rotated_pcd.rotate(rotation_matrix, center=(0, 0, 0))

    print(f"步骤1: 应用地面对齐旋转矩阵")

    # 2. 沿Y轴旋转180度
    # Y轴旋转180度矩阵:
    # [cos(180°)  0  sin(180°)]   [-1  0   0]
    # [0          1  0         ] = [ 0  1   0]
    # [-sin(180°) 0  cos(180°)]   [ 0  0  -1]
    rotation_y_180 = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0]
    ])

    rotated_pcd.rotate(rotation_y_180, center=(0, 0, 0))
    print(f"步骤2: 应用Y轴180度旋转")

    # 计算并显示组合旋转矩阵（地面对齐 + Y轴180度）
    combined_rotation = np.dot(rotation_y_180, rotation_matrix)
    print(f"\n组合旋转矩阵（地面对齐 + Y轴180度）:")
    for i in range(3):
        print(f"  [{combined_rotation[i, 0]:+.6f}, {combined_rotation[i, 1]:+.6f}, {combined_rotation[i, 2]:+.6f}]")
    print()

    # 3. 提取旋转后的地面点
    rotated_ground_pcd = rotated_pcd.select_by_index(ground_inliers)
    rotated_ground_points = np.asarray(rotated_ground_pcd.points)

    print(f"地面点数: {len(rotated_ground_points):,}")

    # 4. 计算旋转后地面点的Z值统计
    z_values = rotated_ground_points[:, 2]
    z_min = z_values.min()
    z_max = z_values.max()
    z_mean = z_values.mean()
    z_median = np.median(z_values)
    z_std = z_values.std()

    print(f"\n旋转后地面Z值统计:")
    print(f"  Z最小值: {z_min:.3f} mm")
    print(f"  Z最大值: {z_max:.3f} mm")
    print(f"  Z均值: {z_mean:.3f} mm")
    print(f"  Z中位数: {z_median:.3f} mm")
    print(f"  Z标准差: {z_std:.3f} mm")
    print(f"  Z跨度: {z_max - z_min:.3f} mm")

    # 5. 计算平移向量，使原点Z坐标在地面上
    # 使用地面Z的中位数作为地面高度
    translation_z = -z_median
    translation_vector = np.array([0.0, 0.0, translation_z])

    print(f"\n步骤3: 计算平移向量")
    print(f"  平移向量: ({translation_vector[0]:.3f}, {translation_vector[1]:.3f}, {translation_vector[2]:.3f}) mm")
    print(f"  Z平移: {translation_z:.3f} mm (使原点Z在地面上)")

    # 6. 应用平移
    rotated_pcd.translate(translation_vector)
    print(f"步骤4: 应用平移")

    # 输出完整的4×4齐次变换矩阵
    transformation_matrix_4x4 = np.eye(4)
    transformation_matrix_4x4[:3, :3] = combined_rotation
    transformation_matrix_4x4[:3, 3] = translation_vector

    print(f"\n完整的4×4齐次变换矩阵:")
    for i in range(4):
        print(f"  [{transformation_matrix_4x4[i, 0]:+.6f}, {transformation_matrix_4x4[i, 1]:+.6f}, {transformation_matrix_4x4[i, 2]:+.6f}, {transformation_matrix_4x4[i, 3]:+.6f}]")
    print(f"  说明: 包含旋转（左上3×3）+ 平移（右上3×1）")
    print()

    # 7. 重新提取地面点（平移后）
    rotated_ground_pcd = rotated_pcd.select_by_index(ground_inliers)
    rotated_ground_points = np.asarray(rotated_ground_pcd.points)

    # 8. 计算平移后地面点的Z值统计
    z_values_translated = rotated_ground_points[:, 2]
    z_mean_translated = z_values_translated.mean()
    z_median_translated = np.median(z_values_translated)
    z_std_translated = z_values_translated.std()

    print(f"\n平移后地面Z值统计:")
    print(f"  Z均值: {z_mean_translated:.3f} mm")
    print(f"  Z中位数: {z_median_translated:.3f} mm")
    print(f"  Z标准差: {z_std_translated:.3f} mm")

    # 9. 计算地面平面方程
    # 理想情况下，旋转后地面应该平行于XY平面，即法向量接近(0, 0, 1)
    # 平面方程: z = d (即 0*x + 0*y + 1*z - d = 0)
    # 使用Z的中位数或均值作为d值

    # 方法1: 使用Z中位数（对离群值更鲁棒）
    d_median = -z_median_translated
    plane_eq_median = np.array([0.0, 0.0, 1.0, d_median])

    # 方法2: 使用Z均值
    d_mean = -z_mean_translated
    plane_eq_mean = np.array([0.0, 0.0, 1.0, d_mean])

    # 方法3: 使用RANSAC重新拟合（更精确）
    print(f"\n使用RANSAC重新拟合旋转后的地面平面...")
    # 设置随机数种子以保证结果可复现
    o3d.utility.random.seed(42)
    plane_model_fitted, inliers_fitted = rotated_ground_pcd.segment_plane(
        distance_threshold=50,  # 50mm
        ransac_n=3,
        num_iterations=1000
    )

    # 归一化平面方程的法向量
    a, b, c, d = plane_model_fitted
    normal_fitted = np.array([a, b, c])
    normal_norm = np.linalg.norm(normal_fitted)
    plane_eq_fitted = plane_model_fitted / normal_norm

    # 确保法向量指向上方
    if plane_eq_fitted[2] < 0:
        plane_eq_fitted = -plane_eq_fitted

    print(f"RANSAC拟合结果:")
    print(f"  内点数: {len(inliers_fitted):,} ({len(inliers_fitted)/len(rotated_ground_points)*100:.2f}%)")
    print(f"  平面方程: {plane_eq_fitted[0]:.6f}*x + {plane_eq_fitted[1]:.6f}*y + {plane_eq_fitted[2]:.6f}*z + {plane_eq_fitted[3]:.6f} = 0")
    print(f"  法向量: ({plane_eq_fitted[0]:.6f}, {plane_eq_fitted[1]:.6f}, {plane_eq_fitted[2]:.6f})")

    # 计算法向量与Z轴的夹角
    z_axis = np.array([0.0, 0.0, 1.0])
    cos_angle = np.dot(plane_eq_fitted[:3], z_axis)
    angle = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))

    print(f"  法向量与Z轴夹角: {angle:.4f}°")

    # 验证平面质量
    print(f"\n平面质量验证:")

    # 计算所有地面点到拟合平面的距离
    distances = np.abs(
        plane_eq_fitted[0] * rotated_ground_points[:, 0] +
        plane_eq_fitted[1] * rotated_ground_points[:, 1] +
        plane_eq_fitted[2] * rotated_ground_points[:, 2] +
        plane_eq_fitted[3]
    )

    dist_mean = distances.mean()
    dist_std = distances.std()
    dist_max = distances.max()

    print(f"  点到平面距离 - 均值: {dist_mean:.3f} mm")
    print(f"  点到平面距离 - 标准差: {dist_std:.3f} mm")
    print(f"  点到平面距离 - 最大值: {dist_max:.3f} mm")

    # 检查是否成功对齐到XY平面
    if angle < 1.0:  # 法向量与Z轴夹角小于1度
        print(f"  ✓ 地面成功对齐到XY平面 (夹角 < 1°)")
    elif angle < 5.0:
        print(f"  ⚠ 地面基本对齐到XY平面 (夹角 < 5°)")
    else:
        print(f"  ✗ 地面未能完全对齐到XY平面 (夹角 = {angle:.2f}°)")

    # 选择最佳平面方程
    # 使用简化方法：z = 0（理论上应该接近0）
    # 强制设置地面平面方程为 z = 0
    ground_plane_equation = np.array([0.0, 0.0, 1.0, 0.0])

    print(f"\n最终地面平面方程:")
    print(f"  强制设置为 z = 0 (0*x + 0*y + 1*z + 0 = 0)")
    print(f"  说明: 已通过平移使地面位于 z=0 平面")
    print(f"  实际地面Z均值偏差: {z_mean_translated:.3f} mm")
    print(f"  实际地面Z中位数偏差: {z_median_translated:.3f} mm")

    # 统计信息
    stats = {
        'z_min': z_min,
        'z_max': z_max,
        'z_mean': z_mean_translated,
        'z_median': z_median_translated,
        'z_std': z_std_translated,
        'z_span': z_max - z_min,
        'translation_vector': translation_vector.tolist(),
        'plane_equation': ground_plane_equation.tolist(),
        'plane_normal': plane_eq_fitted[:3].tolist(),
        'normal_z_angle_deg': angle,
        'ransac_inlier_count': len(inliers_fitted),
        'ransac_inlier_ratio': len(inliers_fitted) / len(rotated_ground_points),
        'distance_to_plane_mean': dist_mean,
        'distance_to_plane_std': dist_std,
        'distance_to_plane_max': dist_max
    }

    print(f"{'='*80}\n")

    return rotated_pcd, ground_plane_equation, translation_vector, stats


def visualize_ground_detection(pcd: o3d.geometry.PointCloud,
                               ground_inliers: np.ndarray,
                               noise_inliers: np.ndarray = None,
                               title: str = "地面检测结果"):
    """
    可视化地面检测结果

    Args:
        pcd: 完整点云
        ground_inliers: 有效地面点索引
        noise_inliers: 噪声点索引（可选）
        title: 窗口标题
    """
    # 地面点云 - 绿色
    ground_pcd = pcd.select_by_index(ground_inliers)
    ground_pcd.paint_uniform_color([0.0, 0.8, 0.0])

    # 非地面点云 - 红色
    non_ground_inliers = np.setdiff1d(np.arange(len(pcd.points)),
                                      np.concatenate([ground_inliers, noise_inliers]) if noise_inliers is not None else ground_inliers)
    non_ground_pcd = pcd.select_by_index(non_ground_inliers)
    non_ground_pcd.paint_uniform_color([0.8, 0.2, 0.2])

    geometries = [ground_pcd, non_ground_pcd]

    # 噪声点云 - 蓝色（如果有）
    if noise_inliers is not None and len(noise_inliers) > 0:
        noise_pcd = pcd.select_by_index(noise_inliers)
        noise_pcd.paint_uniform_color([0.0, 0.0, 1.0])
        geometries.append(noise_pcd)

    # 添加坐标系
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500, origin=[0, 0, 0])
    geometries.append(coord_frame)

    print(f"\n显示可视化结果...")
    print(f"颜色说明:")
    print(f"  绿色 = 有效地面点 ({len(ground_inliers):,} 点)")
    if noise_inliers is not None and len(noise_inliers) > 0:
        print(f"  蓝色 = 地面噪声点 ({len(noise_inliers):,} 点)")
    print(f"  红色 = 非地面点 ({len(non_ground_inliers):,} 点)")
    print(f"  坐标系 = 红(X), 绿(Y), 蓝(Z)\n")

    o3d.visualization.draw_geometries(
        geometries,
        window_name=title,
        width=1600,
        height=900
    )


if __name__ == "__main__":
    print("地面检测模块")
    print("请通过主程序调用此模块")
