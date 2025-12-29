"""
篮筐检测模块
基于旋转矩阵和地面平面方程检测篮筐，计算篮筐中心位置
不依赖聚类，直接从点云中基于高度和RANSAC圆形拟合检测篮筐
注意：点云单位为毫米(mm)
"""

import open3d as o3d
import numpy as np
from typing import Tuple, Dict, List, Optional
from scipy.optimize import least_squares
import copy


def extract_candidates_by_height(pcd: o3d.geometry.PointCloud,
                                  ground_plane_z: float,
                                  height_range: Tuple[float, float] = (2800, 3300),
                                  thickness: float = 100) -> o3d.geometry.PointCloud:
    """
    根据高度范围提取候选篮筐点云

    Args:
        pcd: 完整点云（已旋转）
        ground_plane_z: 地面平面的Z坐标（毫米）
        height_range: 篮筐相对地面的高度范围（毫米），默认(2800mm, 3300mm)
        thickness: 篮筐平面厚度容差（毫米），默认100mm

    Returns:
        candidate_pcd: 候选点云
    """
    print(f"\n{'='*80}")
    print("提取候选篮筐点云 - 基于高度范围")
    print(f"{'='*80}")
    print(f"地面高度: {ground_plane_z:.1f} mm ({ground_plane_z/1000:.3f} m)")
    print(f"篮筐高度范围（相对地面）: [{height_range[0]:.0f}, {height_range[1]:.0f}] mm")
    print(f"篮筐绝对高度范围: [{ground_plane_z + height_range[0]:.0f}, {ground_plane_z + height_range[1]:.0f}] mm")
    print(f"平面厚度容差: {thickness} mm")

    points = np.asarray(pcd.points)
    z_values = points[:, 2]

    # 计算相对地面的高度
    heights_above_ground = z_values - ground_plane_z

    # 筛选在高度范围内的点
    min_height = height_range[0]
    max_height = height_range[1]

    mask = (heights_above_ground >= min_height) & (heights_above_ground <= max_height)
    candidate_indices = np.where(mask)[0]

    candidate_pcd = pcd.select_by_index(candidate_indices)

    print(f"\n筛选结果:")
    print(f"  候选点数: {len(candidate_indices):,} / {len(points):,} ({len(candidate_indices)/len(points)*100:.2f}%)")

    if len(candidate_indices) > 0:
        candidate_points = points[candidate_indices]
        z_min = candidate_points[:, 2].min()
        z_max = candidate_points[:, 2].max()
        z_mean = candidate_points[:, 2].mean()
        print(f"  候选点Z范围: [{z_min:.1f}, {z_max:.1f}] mm")
        print(f"  候选点Z均值: {z_mean:.1f} mm ({z_mean/1000:.3f} m)")
        print(f"  离地高度均值: {z_mean - ground_plane_z:.1f} mm ({(z_mean - ground_plane_z)/1000:.3f} m)")

    print(f"{'='*80}\n")

    return candidate_pcd


def fit_circle_2d_ransac(points_2d: np.ndarray,
                         standard_radius: float = 225,
                         radius_tolerance: float = 50,
                         distance_threshold: float = 20,
                         num_iterations: int = 5000,
                         random_seed: int = 42) -> Optional[Tuple[np.ndarray, float, float, np.ndarray]]:
    """
    使用RANSAC拟合2D圆

    Args:
        points_2d: Nx2的2D点数组
        standard_radius: 标准半径（毫米），默认225mm
        radius_tolerance: 半径容差（毫米），默认50mm
        distance_threshold: RANSAC距离阈值（毫米），默认20mm
        num_iterations: RANSAC迭代次数，默认5000
        random_seed: 随机数种子，默认42（设置后结果可复现）

    Returns:
        center: 圆心坐标 [x, y]
        radius: 半径
        error: 拟合误差
        inliers: 内点索引
    """
    if len(points_2d) < 3:
        return None

    # 设置随机数种子以保证结果可复现
    np.random.seed(random_seed)

    best_center = None
    best_radius = None
    best_inliers = None
    best_inlier_count = 0

    radius_min = standard_radius - radius_tolerance
    radius_max = standard_radius + radius_tolerance

    for _ in range(num_iterations):
        # 随机选择3个点
        if len(points_2d) < 3:
            continue

        sample_indices = np.random.choice(len(points_2d), 3, replace=False)
        sample_points = points_2d[sample_indices]

        # 通过3点拟合圆
        try:
            center, radius = fit_circle_from_3_points(sample_points)
        except:
            continue

        # 检查半径是否在允许范围内
        if not (radius_min <= radius <= radius_max):
            continue

        # 计算所有点到圆的距离
        distances = np.abs(np.sqrt(np.sum((points_2d - center)**2, axis=1)) - radius)

        # 找到内点
        inliers = np.where(distances <= distance_threshold)[0]
        inlier_count = len(inliers)

        # 更新最佳模型
        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            best_inliers = inliers
            best_center = center
            best_radius = radius

    if best_center is None or best_inlier_count < 10:
        return None

    # 使用所有内点重新拟合圆（最小二乘优化）
    inlier_points = points_2d[best_inliers]
    try:
        refined_center, refined_radius, error = fit_circle_2d_least_squares(inlier_points)
    except:
        refined_center = best_center
        refined_radius = best_radius
        distances = np.abs(np.sqrt(np.sum((inlier_points - best_center)**2, axis=1)) - best_radius)
        error = np.sqrt(np.mean(distances**2))

    return refined_center, refined_radius, error, best_inliers


def fit_circle_from_3_points(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    从3个点拟合圆

    Args:
        points: 3x2的点数组

    Returns:
        center: 圆心 [x, y]
        radius: 半径
    """
    p1, p2, p3 = points

    # 计算圆心
    ax = p1[0]
    ay = p1[1]
    bx = p2[0]
    by = p2[1]
    cx = p3[0]
    cy = p3[1]

    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))

    if abs(d) < 1e-6:
        raise ValueError("Points are collinear")

    ux = ((ax**2 + ay**2) * (by - cy) + (bx**2 + by**2) * (cy - ay) + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx) + (bx**2 + by**2) * (ax - cx) + (cx**2 + cy**2) * (bx - ax)) / d

    center = np.array([ux, uy])
    radius = np.sqrt((ax - ux)**2 + (ay - uy)**2)

    return center, radius


def fit_circle_2d_least_squares(points_2d: np.ndarray) -> Tuple[np.ndarray, float, float]:
    """
    使用最小二乘法拟合2D圆

    Args:
        points_2d: Nx2的2D点数组

    Returns:
        center: 圆心坐标 [x, y]
        radius: 半径
        error: 拟合误差
    """
    def calc_residuals(params, points):
        """计算残差"""
        cx, cy, r = params
        distances = np.sqrt((points[:, 0] - cx)**2 + (points[:, 1] - cy)**2)
        return distances - r

    # 初始估计：使用点的中心和平均距离
    center_init = points_2d.mean(axis=0)
    distances = np.sqrt(np.sum((points_2d - center_init)**2, axis=1))
    radius_init = distances.mean()

    # 最小二乘优化
    x0 = [center_init[0], center_init[1], radius_init]
    result = least_squares(calc_residuals, x0, args=(points_2d,))

    cx, cy, r = result.x
    center = np.array([cx, cy])

    # 计算拟合误差（均方根误差）
    residuals = calc_residuals(result.x, points_2d)
    error = np.sqrt(np.mean(residuals**2))

    return center, r, error


def detect_hoop(pcd: o3d.geometry.PointCloud,
                rotation_matrix: np.ndarray,
                ground_plane_equation: np.ndarray,
                translation_vector: np.ndarray,
                standard_diameter: float = 450,
                diameter_tolerance: float = 50,
                height_range: Tuple[float, float] = (2800, 3300),
                standard_height: float = 3050,
                random_seed: int = 42) -> Optional[Dict]:
    """
    从点云中检测篮筐

    流程：
    1. 应用旋转矩阵到点云
    2. 应用Y轴180度旋转
    3. 应用平移向量
    4. 根据高度范围提取候选点
    5. 使用RANSAC圆形拟合检测篮筐
    6. 利用地面平面方程计算篮筐高度

    Args:
        pcd: 完整点云（未旋转）
        rotation_matrix: 旋转矩阵
        ground_plane_equation: 地面平面方程 [a, b, c, d]，旋转后的
        translation_vector: 平移向量 [tx, ty, tz]
        standard_diameter: 标准篮筐直径（毫米），默认450mm
        diameter_tolerance: 直径容差（毫米），默认50mm
        height_range: 篮筐相对地面的高度范围（毫米），默认(2800mm, 3300mm)
        standard_height: 标准篮筐高度（毫米），默认3050mm (3.05m)
        random_seed: 随机数种子，默认42（设置后结果可复现）

    Returns:
        hoop_info: 篮筐信息字典，如果未检测到则返回None
    """
    print(f"\n{'='*80}")
    print("篮筐检测（基于RANSAC圆形拟合）")
    print(f"{'='*80}")
    print(f"标准篮筐直径: {standard_diameter} mm ({standard_diameter/1000:.3f} m)")
    print(f"直径容差: ±{diameter_tolerance} mm (±{diameter_tolerance/1000:.3f} m)")
    print(f"直径范围: [{standard_diameter-diameter_tolerance}, {standard_diameter+diameter_tolerance}] mm")
    print(f"标准篮筐高度（离地）: {standard_height} mm ({standard_height/1000:.3f} m)")

    # 步骤1: 应用地面对齐旋转矩阵
    print(f"\n步骤1: 应用地面对齐旋转矩阵")
    rotated_pcd = copy.deepcopy(pcd)
    rotated_pcd.rotate(rotation_matrix, center=(0, 0, 0))

    # 步骤2: 应用Y轴180度旋转
    rotation_y_180 = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0]
    ])
    rotated_pcd.rotate(rotation_y_180, center=(0, 0, 0))
    print(f"步骤2: 应用Y轴180度旋转")

    # 步骤3: 应用平移向量
    rotated_pcd.translate(translation_vector)
    print(f"步骤3: 应用平移向量 ({translation_vector[0]:.1f}, {translation_vector[1]:.1f}, {translation_vector[2]:.1f}) mm")
    print(f"点云变换完成，点数: {len(rotated_pcd.points):,}")

    # 步骤4: 计算地面高度
    # 注意：地面平面方程现在固定为 z = 0
    ground_plane_z = -ground_plane_equation[3] / ground_plane_equation[2]
    print(f"\n地面平面方程: {ground_plane_equation[0]:.6f}*x + {ground_plane_equation[1]:.6f}*y + {ground_plane_equation[2]:.6f}*z + {ground_plane_equation[3]:.6f} = 0")
    print(f"地面高度: z = {ground_plane_z:.3f} mm ({ground_plane_z/1000:.6f} m)")
    print(f"说明: 地面平面已固定为 z = 0")

    # 步骤3: 根据高度范围提取候选点
    candidate_pcd = extract_candidates_by_height(rotated_pcd, ground_plane_z, height_range)

    if len(candidate_pcd.points) < 10:
        print(f"\n警告: 候选点数量太少 ({len(candidate_pcd.points)} < 10)，无法进行圆形拟合")
        return None

    # 步骤4: 投影到XY平面并进行RANSAC圆形拟合
    print(f"{'='*80}")
    print("RANSAC圆形拟合")
    print(f"{'='*80}")

    candidate_points = np.asarray(candidate_pcd.points)
    points_2d = candidate_points[:, :2]  # XY坐标

    standard_radius = standard_diameter / 2
    radius_tolerance = diameter_tolerance / 2

    print(f"候选点数: {len(points_2d):,}")
    print(f"标准半径: {standard_radius} mm")
    print(f"半径容差: ±{radius_tolerance} mm")

    # RANSAC圆形拟合
    result = fit_circle_2d_ransac(
        points_2d,
        standard_radius=standard_radius,
        radius_tolerance=radius_tolerance,
        distance_threshold=30,  # 30mm距离阈值
        num_iterations=10000,
        random_seed=random_seed  # 使用传入的随机数种子
    )

    if result is None:
        print(f"\n警告: RANSAC圆形拟合失败")
        return None

    center_2d, radius, fit_error, inliers = result
    diameter = radius * 2

    print(f"\nRANSAC拟合结果:")
    print(f"  圆心 (XY): ({center_2d[0]:.1f}, {center_2d[1]:.1f}) mm")
    print(f"  半径: {radius:.1f} mm")
    print(f"  直径: {diameter:.1f} mm ({diameter/1000:.3f} m)")
    print(f"  内点数: {len(inliers):,} / {len(points_2d):,} ({len(inliers)/len(points_2d)*100:.2f}%)")
    print(f"  拟合误差: {fit_error:.2f} mm")

    # 步骤5: 计算篮筐平面高度（使用内点的Z均值）
    inlier_points_3d = candidate_points[inliers]
    hoop_plane_z = inlier_points_3d[:, 2].mean()
    z_std = inlier_points_3d[:, 2].std()

    print(f"\n篮筐平面:")
    print(f"  平面高度: {hoop_plane_z:.1f} mm ({hoop_plane_z/1000:.3f} m)")
    print(f"  Z标准差: {z_std:.1f} mm")

    # 步骤6: 构建篮筐中心3D坐标
    center_3d = np.array([center_2d[0], center_2d[1], hoop_plane_z])
    hoop_plane_equation = np.array([0.0, 0.0, 1.0, -hoop_plane_z])

    # 步骤7: 计算离地高度
    height_above_ground = hoop_plane_z - ground_plane_z

    # 步骤8: 计算误差
    diameter_error = abs(diameter - standard_diameter)
    diameter_error_ratio = diameter_error / standard_diameter
    height_error = abs(height_above_ground - standard_height)

    print(f"\n检测结果:")
    print(f"  篮筐中心 (3D): ({center_3d[0]:.1f}, {center_3d[1]:.1f}, {center_3d[2]:.1f}) mm")
    print(f"                 = ({center_3d[0]/1000:.3f}, {center_3d[1]/1000:.3f}, {center_3d[2]/1000:.3f}) m")
    print(f"  离地高度: {height_above_ground:.1f} mm ({height_above_ground/1000:.3f} m)")
    print(f"  直径误差: {diameter_error:.1f} mm ({diameter_error_ratio*100:.2f}%)")
    print(f"  高度误差: {height_error:.1f} mm")
    print(f"{'='*80}\n")

    hoop_info = {
        'center_3d': center_3d,
        'center_2d': center_2d,
        'radius': radius,
        'diameter': diameter,
        'hoop_plane_z': hoop_plane_z,
        'hoop_plane_equation': hoop_plane_equation,
        'height_above_ground': height_above_ground,
        'z_std': z_std,
        'fit_error': fit_error,
        'diameter_error': diameter_error,
        'diameter_error_ratio': diameter_error_ratio,
        'height_error': height_error,
        'inlier_count': len(inliers),
        'inlier_ratio': len(inliers) / len(points_2d),
        'candidate_count': len(points_2d)
    }

    return hoop_info


def verify_hoop(hoop_info: Dict,
                standard_diameter: float = 450,
                diameter_tolerance: float = 50,
                standard_height: float = 3050,
                height_tolerance: float = 300,
                max_z_std: float = 100,
                min_inlier_ratio: float = 0.3) -> Tuple[bool, List[str]]:
    """
    验证检测到的篮筐是否符合要求

    Args:
        hoop_info: 篮筐信息字典
        standard_diameter: 标准篮筐直径（毫米）
        diameter_tolerance: 直径容差（毫米）
        standard_height: 标准篮筐高度（毫米）
        height_tolerance: 高度容差（毫米）
        max_z_std: 最大Z标准差（毫米），用于检查平面度
        min_inlier_ratio: 最小内点比例

    Returns:
        is_valid: 是否有效
        messages: 验证消息列表
    """
    print(f"\n{'='*80}")
    print("篮筐验证")
    print(f"{'='*80}")

    messages = []
    is_valid = True

    # 1. 检查直径
    diameter_error = abs(hoop_info['diameter'] - standard_diameter)
    if diameter_error <= diameter_tolerance:
        messages.append(f"✓ 直径检查通过: {hoop_info['diameter']:.1f} mm (误差: {diameter_error:.1f} mm ≤ {diameter_tolerance} mm)")
    else:
        messages.append(f"✗ 直径检查失败: {hoop_info['diameter']:.1f} mm (误差: {diameter_error:.1f} mm > {diameter_tolerance} mm)")
        is_valid = False

    # 2. 检查高度
    height_error = hoop_info['height_error']
    if height_error <= height_tolerance:
        messages.append(f"✓ 高度检查通过: {hoop_info['height_above_ground']:.1f} mm ({hoop_info['height_above_ground']/1000:.3f} m) (误差: {height_error:.1f} mm ≤ {height_tolerance} mm)")
    else:
        messages.append(f"⚠ 高度偏差较大: {hoop_info['height_above_ground']:.1f} mm ({hoop_info['height_above_ground']/1000:.3f} m) (误差: {height_error:.1f} mm > {height_tolerance} mm)")
        # 高度不作为硬性失败条件，只警告

    # 3. 检查平面度（Z标准差）
    if hoop_info['z_std'] <= max_z_std:
        messages.append(f"✓ 平面度检查通过: Z标准差 = {hoop_info['z_std']:.1f} mm ≤ {max_z_std} mm")
    else:
        messages.append(f"⚠ 平面度较差: Z标准差 = {hoop_info['z_std']:.1f} mm > {max_z_std} mm")
        # 平面度不作为硬性失败条件，只警告

    # 4. 检查拟合误差
    if hoop_info['fit_error'] <= 30:  # 30mm的拟合误差
        messages.append(f"✓ 拟合误差检查通过: {hoop_info['fit_error']:.2f} mm ≤ 30 mm")
    else:
        messages.append(f"⚠ 拟合误差较大: {hoop_info['fit_error']:.2f} mm > 30 mm")

    # 5. 检查内点比例
    if hoop_info['inlier_ratio'] >= min_inlier_ratio:
        messages.append(f"✓ 内点比例检查通过: {hoop_info['inlier_ratio']*100:.1f}% ≥ {min_inlier_ratio*100:.1f}%")
    else:
        messages.append(f"⚠ 内点比例较低: {hoop_info['inlier_ratio']*100:.1f}% < {min_inlier_ratio*100:.1f}%")

    for msg in messages:
        print(f"  {msg}")

    print(f"\n验证结果: {'通过 ✓' if is_valid else '失败 ✗'}")
    print(f"{'='*80}\n")

    return is_valid, messages


def visualize_hoop_detection(pcd: o3d.geometry.PointCloud,
                             rotation_matrix: np.ndarray,
                             translation_vector: np.ndarray,
                             hoop_info: Dict,
                             ground_plane_equation: np.ndarray):
    """
    可视化篮筐检测结果

    Args:
        pcd: 完整点云（未旋转）
        rotation_matrix: 旋转矩阵
        translation_vector: 平移向量
        hoop_info: 篮筐信息
        ground_plane_equation: 地面平面方程
    """
    print(f"\n可视化篮筐检测结果...")

    geometries = []

    # 1. 应用完整变换到点云
    rotated_pcd = copy.deepcopy(pcd)
    rotated_pcd.rotate(rotation_matrix, center=(0, 0, 0))

    rotation_y_180 = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0]
    ])
    rotated_pcd.rotate(rotation_y_180, center=(0, 0, 0))
    rotated_pcd.translate(translation_vector)

    # 2. 提取候选区域点云
    ground_plane_z = -ground_plane_equation[3] / ground_plane_equation[2]
    height_range = (2800, 3300)
    candidate_pcd = extract_candidates_by_height(rotated_pcd, ground_plane_z, height_range)

    # 候选点云 - 蓝色
    if len(candidate_pcd.points) > 0:
        candidate_colored = copy.deepcopy(candidate_pcd)
        candidate_colored.paint_uniform_color([0.3, 0.3, 1.0])
        geometries.append(candidate_colored)

    # 3. 其他点云 - 浅灰色
    points = np.asarray(rotated_pcd.points)
    z_values = points[:, 2]
    heights = z_values - ground_plane_z
    other_mask = (heights < height_range[0]) | (heights > height_range[1])
    other_indices = np.where(other_mask)[0]

    if len(other_indices) > 0:
        other_pcd = rotated_pcd.select_by_index(other_indices)
        other_pcd.paint_uniform_color([0.7, 0.7, 0.7])
        geometries.append(other_pcd)

    # 4. 篮筐中心标记 - 黄色球体
    center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=50)  # 50mm半径
    center_sphere.translate(hoop_info['center_3d'])
    center_sphere.paint_uniform_color([1.0, 1.0, 0.0])
    geometries.append(center_sphere)

    # 5. 绘制篮筐圆环（在XY平面上）
    circle_points = []
    num_points = 100
    for i in range(num_points):
        angle = 2 * np.pi * i / num_points
        x = hoop_info['center_2d'][0] + hoop_info['radius'] * np.cos(angle)
        y = hoop_info['center_2d'][1] + hoop_info['radius'] * np.sin(angle)
        z = hoop_info['hoop_plane_z']
        circle_points.append([x, y, z])

    circle_pcd = o3d.geometry.PointCloud()
    circle_pcd.points = o3d.utility.Vector3dVector(np.array(circle_points))
    circle_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # 红色圆环
    geometries.append(circle_pcd)

    # 6. 坐标系
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500, origin=[0, 0, 0])
    geometries.append(coord_frame)

    print(f"\n颜色说明:")
    print(f"  蓝色 = 候选区域点云（高度范围内）")
    print(f"  红色圆环 = 拟合的篮筐圆")
    print(f"  黄色球体 = 篮筐中心")
    print(f"  浅灰色 = 其他点云")
    print(f"  坐标系 = 红(X), 绿(Y), 蓝(Z)")

    print(f"\n篮筐中心位置（旋转后坐标系）: ({hoop_info['center_3d'][0]:.1f}, {hoop_info['center_3d'][1]:.1f}, {hoop_info['center_3d'][2]:.1f}) mm")
    print(f"                              = ({hoop_info['center_3d'][0]/1000:.3f}, {hoop_info['center_3d'][1]/1000:.3f}, {hoop_info['center_3d'][2]/1000:.3f}) m\n")

    o3d.visualization.draw_geometries(
        geometries,
        window_name=f"篮筐检测 | 中心=({hoop_info['center_3d'][0]/1000:.2f}, {hoop_info['center_3d'][1]/1000:.2f}, {hoop_info['center_3d'][2]/1000:.2f})m | 直径={hoop_info['diameter']:.0f}mm",
        width=1600,
        height=900
    )


if __name__ == "__main__":
    import yaml
    import json
    import os

    print("\n" + "="*80)
    print("篮筐检测测试程序")
    print("="*80)

    # 加载配置
    config_file = "CTconfig.yaml"
    try:
        with open(config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        print(f"\n配置文件加载成功: {config_file}")
    except Exception as e:
        print(f"\n错误: 无法加载配置文件: {e}")
        exit(1)

    # 加载旋转矩阵和地面平面方程
    calibration_config_file = os.path.join(config['point_cloud']['output_dir'], "config.json")
    try:
        with open(calibration_config_file, 'r', encoding='utf-8') as f:
            calibration_config = json.load(f)
        print(f"标定配置加载成功: {calibration_config_file}")
    except Exception as e:
        print(f"\n错误: 无法加载标定配置文件: {e}")
        print(f"请先运行 main.py 完成标定流程")
        exit(1)

    rotation_matrix = np.array(calibration_config['rotation_matrix'])
    ground_plane_equation = np.array(calibration_config['ground_plane_equation'])
    translation_vector = np.array(calibration_config['translation_vector'])

    print(f"\n旋转矩阵:")
    for i in range(3):
        print(f"  [{rotation_matrix[i, 0]:+.6f}, {rotation_matrix[i, 1]:+.6f}, {rotation_matrix[i, 2]:+.6f}]")

    print(f"\n平移向量:")
    print(f"  [{translation_vector[0]:.3f}, {translation_vector[1]:.3f}, {translation_vector[2]:.3f}] mm")

    print(f"\n地面平面方程: {ground_plane_equation[0]:.6f}*x + {ground_plane_equation[1]:.6f}*y + {ground_plane_equation[2]:.6f}*z + {ground_plane_equation[3]:.6f} = 0")

    # 加载预处理后的点云
    pcd_file = os.path.join(config['point_cloud']['output_dir'], "processed_pointcloud.ply")
    print(f"\n加载预处理后的点云: {pcd_file}")

    if not os.path.exists(pcd_file):
        print(f"错误: 点云文件不存在，请先运行 main.py 进行预处理")
        exit(1)

    pcd = o3d.io.read_point_cloud(pcd_file)
    print(f"点云加载成功! 点数量: {len(pcd.points):,}")

    # 篮筐检测
    hoop_config = config['hoop_detection']

    hoop_info = detect_hoop(
        pcd,
        rotation_matrix,
        ground_plane_equation,
        translation_vector,
        standard_diameter=hoop_config['standard_diameter'] * 1000,  # 转换为毫米
        diameter_tolerance=hoop_config['diameter_tolerance'] * 1000,  # 转换为毫米
        height_range=(hoop_config['height_range'][0] * 1000, hoop_config['height_range'][1] * 1000),  # 转换为毫米
        random_seed=hoop_config.get('random_seed', 42)  # 使用配置文件中的随机数种子，默认42
    )

    if hoop_info is None:
        print(f"\n错误: 未能检测到篮筐")
        print(f"可能的原因：")
        print(f"  1. 点云中没有篮筐")
        print(f"  2. 篮筐直径不在标准范围内")
        print(f"  3. 篮筐高度不符合要求")
        print(f"  4. RANSAC参数需要调整")
        exit(1)

    # 验证篮筐
    is_valid, messages = verify_hoop(
        hoop_info,
        standard_diameter=hoop_config['standard_diameter'] * 1000,
        diameter_tolerance=hoop_config['diameter_tolerance'] * 1000
    )

    # 保存篮筐信息
    output_dir = config['point_cloud']['output_dir']
    hoop_report_file = os.path.join(output_dir, "hoop_detection_report.txt")

    with open(hoop_report_file, 'w', encoding='utf-8') as f:
        f.write("="*80 + "\n")
        f.write("篮筐检测报告\n")
        f.write("="*80 + "\n\n")

        f.write("篮筐中心位置（旋转后坐标系，毫米）:\n")
        f.write(f"  X = {hoop_info['center_3d'][0]:.3f} mm\n")
        f.write(f"  Y = {hoop_info['center_3d'][1]:.3f} mm\n")
        f.write(f"  Z = {hoop_info['center_3d'][2]:.3f} mm\n\n")

        f.write("篮筐中心位置（旋转后坐标系，米）:\n")
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

        f.write("篮筐平面方程（旋转后坐标系）:\n")
        f.write(f"  {hoop_info['hoop_plane_equation'][0]:.6f}*x + {hoop_info['hoop_plane_equation'][1]:.6f}*y + {hoop_info['hoop_plane_equation'][2]:.6f}*z + {hoop_info['hoop_plane_equation'][3]:.6f} = 0\n")
        f.write(f"  简化形式: z = {hoop_info['hoop_plane_z']:.3f} mm\n\n")

        f.write("验证结果:\n")
        f.write(f"  状态: {'通过' if is_valid else '失败'}\n")
        for msg in messages:
            f.write(f"  {msg}\n")

        f.write("\n" + "="*80 + "\n")

    print(f"\n篮筐检测报告已保存: {hoop_report_file}")

    # 可视化
    print(f"\n准备可视化篮筐检测结果...")
    visualize_hoop_detection(pcd, rotation_matrix, translation_vector, hoop_info, ground_plane_equation)

    print(f"\n" + "="*80)
    print("篮筐检测完成!")
    print("="*80)
    print(f"\n篮筐中心位置: ({hoop_info['center_3d'][0]/1000:.3f}, {hoop_info['center_3d'][1]/1000:.3f}, {hoop_info['center_3d'][2]/1000:.3f}) m")
    print(f"离地高度: {hoop_info['height_above_ground']/1000:.3f} m")
    print(f"直径: {hoop_info['diameter']:.1f} mm")
    print(f"验证状态: {'通过 ✓' if is_valid else '失败 ✗'}\n")
