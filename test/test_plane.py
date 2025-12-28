"""
地面平面方程可视化测试脚本
显示旋转后的点云和拟合的地面平面
"""

import open3d as o3d
import numpy as np
import json
import os


def create_plane_mesh(plane_equation, size=5000, grid_size=100):
    """
    根据平面方程创建平面网格用于可视化

    Args:
        plane_equation: 平面方程 [a, b, c, d]，ax + by + cz + d = 0
        size: 平面尺寸（毫米）
        grid_size: 网格密度

    Returns:
        mesh: 平面网格
    """
    a, b, c, d = plane_equation

    # 创建网格点
    x = np.linspace(-size, size, grid_size)
    y = np.linspace(-size, size, grid_size)
    X, Y = np.meshgrid(x, y)

    # 根据平面方程计算Z值
    # ax + by + cz + d = 0 => z = -(ax + by + d) / c
    if abs(c) > 1e-6:
        Z = -(a * X + b * Y + d) / c
    else:
        # 如果c接近0，平面垂直于XY平面，使用固定Z值
        Z = np.zeros_like(X)

    # 将网格转换为点云
    points = np.stack([X.flatten(), Y.flatten(), Z.flatten()], axis=1)

    # 创建三角形网格
    vertices = points
    triangles = []

    for i in range(grid_size - 1):
        for j in range(grid_size - 1):
            # 每个网格单元创建两个三角形
            idx = i * grid_size + j
            # 三角形1
            triangles.append([idx, idx + 1, idx + grid_size])
            # 三角形2
            triangles.append([idx + 1, idx + grid_size + 1, idx + grid_size])

    # 创建Open3D网格
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)

    # 计算法向量
    mesh.compute_vertex_normals()

    # 设置半透明的颜色
    mesh.paint_uniform_color([0.2, 0.8, 0.2])  # 绿色

    return mesh


def visualize_ground_plane():
    """
    可视化旋转后的点云和拟合的地面平面
    """
    print("\n" + "="*80)
    print("地面平面方程可视化")
    print("="*80)

    # 1. 加载配置文件
    config_file = "output/config.json"

    if not os.path.exists(config_file):
        print(f"\n错误: 配置文件不存在: {config_file}")
        print("请先运行 main.py 完成标定流程")
        return

    with open(config_file, 'r', encoding='utf-8') as f:
        config = json.load(f)

    print(f"\n加载配置文件: {config_file}")

    # 提取平面方程
    if 'ground_plane_equation' not in config:
        print(f"\n错误: 配置文件中缺少 'ground_plane_equation' 字段")
        print(f"请重新运行 main.py 以生成包含地面平面方程的完整配置文件")
        print(f"\n运行命令: python main.py")
        return

    plane_equation = np.array(config['ground_plane_equation'])
    rotation_matrix = np.array(config['rotation_matrix'])
    ground_normal = np.array(config['ground_normal'])

    print(f"\n地面平面方程:")
    print(f"  {plane_equation[0]:.10f} * x + {plane_equation[1]:.10f} * y + {plane_equation[2]:.10f} * z + {plane_equation[3]:.10f} = 0")

    # 简化形式（如果是接近水平的平面）
    if abs(plane_equation[2]) > 0.9:
        z_value = -plane_equation[3] / plane_equation[2]
        print(f"  简化形式: z ≈ {z_value:.3f} mm ({z_value/1000:.6f} m)")

    print(f"\n平面法向量: ({plane_equation[0]:.6f}, {plane_equation[1]:.6f}, {plane_equation[2]:.6f})")

    # 计算法向量与Z轴夹角
    z_axis = np.array([0, 0, 1])
    cos_angle = np.dot(plane_equation[:3], z_axis)
    angle = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))
    print(f"法向量与Z轴夹角: {angle:.4f}°")

    # 2. 加载旋转后的点云
    rotated_pcd_file = "output/rotated_pointcloud.ply"

    if not os.path.exists(rotated_pcd_file):
        print(f"\n错误: 旋转后的点云文件不存在: {rotated_pcd_file}")
        print("请先运行 main.py 完成标定流程")
        return

    print(f"\n加载旋转后的点云: {rotated_pcd_file}")
    rotated_pcd = o3d.io.read_point_cloud(rotated_pcd_file)
    print(f"点云数量: {len(rotated_pcd.points):,}")

    # 3. 加载地面点云
    ground_file = "output/ground_plane.ply"
    if os.path.exists(ground_file):
        print(f"加载地面点云: {ground_file}")
        ground_pcd_original = o3d.io.read_point_cloud(ground_file)

        # 应用旋转矩阵到地面点云
        ground_pcd = o3d.geometry.PointCloud(ground_pcd_original)
        ground_pcd.rotate(rotation_matrix, center=(0, 0, 0))

        print(f"地面点云数量: {len(ground_pcd.points):,}")

        # 计算地面点的Z值统计
        ground_points = np.asarray(ground_pcd.points)
        z_values = ground_points[:, 2]

        print(f"\n地面点Z值统计:")
        print(f"  最小值: {z_values.min():.3f} mm")
        print(f"  最大值: {z_values.max():.3f} mm")
        print(f"  均值: {z_values.mean():.3f} mm")
        print(f"  中位数: {np.median(z_values):.3f} mm")
        print(f"  标准差: {z_values.std():.3f} mm")

        # 计算地面点到拟合平面的距离
        distances = np.abs(
            plane_equation[0] * ground_points[:, 0] +
            plane_equation[1] * ground_points[:, 1] +
            plane_equation[2] * ground_points[:, 2] +
            plane_equation[3]
        )

        print(f"\n地面点到拟合平面的距离:")
        print(f"  均值: {distances.mean():.3f} mm")
        print(f"  标准差: {distances.std():.3f} mm")
        print(f"  最大值: {distances.max():.3f} mm")

        # 着色地面点云
        ground_pcd.paint_uniform_color([0.0, 0.8, 0.0])  # 绿色
    else:
        print(f"警告: 地面点云文件不存在，跳过")
        ground_pcd = None

    # 4. 创建拟合平面的网格
    print(f"\n创建拟合平面网格...")
    plane_mesh = create_plane_mesh(plane_equation, size=5000, grid_size=50)

    # 5. 准备可视化
    print(f"\n准备可视化...")

    geometries = []

    # 添加旋转后的点云（灰色半透明）
    rotated_pcd_colored = o3d.geometry.PointCloud(rotated_pcd)
    rotated_pcd_colored.paint_uniform_color([0.5, 0.5, 0.5])
    geometries.append(rotated_pcd_colored)

    # 添加地面点云（绿色）
    if ground_pcd is not None:
        geometries.append(ground_pcd)

    # 添加拟合平面（半透明绿色网格）
    geometries.append(plane_mesh)

    # 添加坐标系
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0, 0, 0])
    geometries.append(coord_frame)

    # 在平面上标记几个参考点
    # 中心点
    if abs(plane_equation[2]) > 1e-6:
        center_z = -plane_equation[3] / plane_equation[2]
        center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=50)
        center_sphere.translate([0, 0, center_z])
        center_sphere.paint_uniform_color([1.0, 0.0, 0.0])  # 红色
        geometries.append(center_sphere)

        print(f"\n平面中心点 (0, 0, z): z = {center_z:.3f} mm")

    print(f"\n" + "="*80)
    print("可视化说明")
    print("="*80)
    print(f"  灰色点云 = 旋转后的完整点云")
    print(f"  绿色点云 = 旋转后的地面点")
    print(f"  绿色半透明网格 = 拟合的地面平面")
    print(f"  红色球体 = 平面中心点 (0, 0, z)")
    print(f"  坐标系 = 红(X), 绿(Y), 蓝(Z)")
    print(f"\n提示:")
    print(f"  - 旋转视图查看平面与地面点的拟合情况")
    print(f"  - 地面点应该紧贴绿色平面网格")
    print(f"  - 法向量应该接近垂直向上（Z轴方向）")
    print("="*80 + "\n")

    # 显示可视化
    o3d.visualization.draw_geometries(
        geometries,
        window_name=f"地面平面方程可视化 | z ≈ {-plane_equation[3]/plane_equation[2]:.1f}mm | 角度={angle:.2f}°",
        width=1600,
        height=900
    )


if __name__ == "__main__":
    try:
        visualize_ground_plane()
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
