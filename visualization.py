"""
可视化和统计分析模块
提供点云数据的可视化和统计信息
"""

import open3d as o3d
import numpy as np
from typing import List, Dict
import matplotlib.pyplot as plt
from matplotlib import rcParams


# 设置中文字体支持
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial']
rcParams['axes.unicode_minus'] = False


class PointCloudStatistics:
    """点云统计信息类"""

    def __init__(self, name: str, pcd: o3d.geometry.PointCloud):
        """
        初始化统计信息

        Args:
            name: 点云名称
            pcd: 点云对象
        """
        self.name = name
        self.point_count = len(pcd.points)
        self.has_colors = pcd.has_colors()
        self.has_normals = pcd.has_normals()

        if self.point_count > 0:
            points = np.asarray(pcd.points)
            self.bbox_min = points.min(axis=0)
            self.bbox_max = points.max(axis=0)
            self.bbox_size = self.bbox_max - self.bbox_min
            self.center = points.mean(axis=0)
            self.std = points.std(axis=0)
        else:
            self.bbox_min = np.zeros(3)
            self.bbox_max = np.zeros(3)
            self.bbox_size = np.zeros(3)
            self.center = np.zeros(3)
            self.std = np.zeros(3)

    def print_summary(self):
        """打印统计摘要"""
        print(f"\n{'='*60}")
        print(f"点云统计信息: {self.name}")
        print(f"{'='*60}")
        print(f"点数量: {self.point_count:,}")
        print(f"包含颜色: {'是' if self.has_colors else '否'}")
        print(f"包含法向量: {'是' if self.has_normals else '否'}")

        if self.point_count > 0:
            print(f"\n空间范围 (米):")
            print(f"  X: [{self.bbox_min[0]:.3f}, {self.bbox_max[0]:.3f}] (跨度: {self.bbox_size[0]:.3f})")
            print(f"  Y: [{self.bbox_min[1]:.3f}, {self.bbox_max[1]:.3f}] (跨度: {self.bbox_size[1]:.3f})")
            print(f"  Z: [{self.bbox_min[2]:.3f}, {self.bbox_max[2]:.3f}] (跨度: {self.bbox_size[2]:.3f})")

            print(f"\n中心位置 (米): ({self.center[0]:.3f}, {self.center[1]:.3f}, {self.center[2]:.3f})")
            print(f"标准差 (米): ({self.std[0]:.3f}, {self.std[1]:.3f}, {self.std[2]:.3f})")

        print(f"{'='*60}\n")


def compare_statistics(stats_list: List[PointCloudStatistics]):
    """
    比较多个点云的统计信息

    Args:
        stats_list: 统计信息列表
    """
    print(f"\n{'='*80}")
    print("点云处理前后对比")
    print(f"{'='*80}")

    # 打印对比表格
    print(f"\n{'阶段':<20} {'点数量':<15} {'点数量变化':<20} {'保留率':<15}")
    print("-" * 80)

    for i, stats in enumerate(stats_list):
        if i == 0:
            print(f"{stats.name:<20} {stats.point_count:<15,} {'-':<20} {'100.00%':<15}")
        else:
            original_count = stats_list[0].point_count
            change = stats.point_count - original_count
            change_str = f"{change:+,} 点"
            retention = stats.point_count / original_count * 100 if original_count > 0 else 0
            print(f"{stats.name:<20} {stats.point_count:<15,} {change_str:<20} {retention:.2f}%")

    print("-" * 80)

    # 空间范围对比
    print(f"\n空间范围对比 (米):")
    print(f"{'阶段':<20} {'X跨度':<12} {'Y跨度':<12} {'Z跨度':<12}")
    print("-" * 80)
    for stats in stats_list:
        if stats.point_count > 0:
            print(f"{stats.name:<20} {stats.bbox_size[0]:<12.3f} {stats.bbox_size[1]:<12.3f} {stats.bbox_size[2]:<12.3f}")

    print(f"{'='*80}\n")


def visualize_comparison(pcd_original: o3d.geometry.PointCloud,
                         pcd_processed: o3d.geometry.PointCloud,
                         clusters: List[o3d.geometry.PointCloud]):
    """
    并排对比可视化原始点云和处理后的点云

    Args:
        pcd_original: 原始点云
        pcd_processed: 处理后的点云
        clusters: 聚类结果
    """
    print("\n" + "="*60)
    print("可视化对比")
    print("="*60)

    # 1. 原始点云 vs 预处理后点云
    print("\n1. 显示原始点云 (按任意键继续)...")
    pcd_original_vis = o3d.geometry.PointCloud(pcd_original)
    o3d.visualization.draw_geometries(
        [pcd_original_vis],
        window_name=f"原始点云 ({len(pcd_original.points):,} 点)",
        width=1280,
        height=720,
        point_show_normal=False
    )

    print("2. 显示预处理后点云 (按任意键继续)...")
    pcd_processed_vis = o3d.geometry.PointCloud(pcd_processed)
    pcd_processed_vis.paint_uniform_color([0.5, 0.5, 0.5])
    o3d.visualization.draw_geometries(
        [pcd_processed_vis],
        window_name=f"预处理后点云 ({len(pcd_processed.points):,} 点)",
        width=1280,
        height=720,
        point_show_normal=False
    )

    # 2. 聚类结果可视化 - 每个簇用不同颜色
    if len(clusters) > 0:
        print(f"3. 显示聚类结果 ({len(clusters)} 个簇)...")

        # 为每个簇分配不同颜色
        cluster_vis = []
        colors = plt.cm.tab20(np.linspace(0, 1, len(clusters)))

        for i, cluster in enumerate(clusters):
            cluster_colored = o3d.geometry.PointCloud(cluster)
            cluster_colored.paint_uniform_color(colors[i, :3])
            cluster_vis.append(cluster_colored)

        o3d.visualization.draw_geometries(
            cluster_vis,
            window_name=f"聚类结果 ({len(clusters)} 个簇)",
            width=1280,
            height=720,
            point_show_normal=False
        )

        # 3. 显示各个簇的统计信息
        print(f"\n聚类簇详细信息:")
        print(f"{'簇编号':<10} {'点数量':<15} {'中心位置 (X, Y, Z)':<40} {'尺寸 (X, Y, Z)':<30}")
        print("-" * 100)

        for i, cluster in enumerate(clusters):
            points = np.asarray(cluster.points)
            center = points.mean(axis=0)
            bbox_min = points.min(axis=0)
            bbox_max = points.max(axis=0)
            size = bbox_max - bbox_min

            print(f"簇 {i:<7} {len(points):<15,} ({center[0]:>6.3f}, {center[1]:>6.3f}, {center[2]:>6.3f})  "
                  f"({size[0]:>6.3f}, {size[1]:>6.3f}, {size[2]:>6.3f})")

        print("-" * 100)

    print("\n可视化完成!")


def plot_statistics_chart(stats_list: List[PointCloudStatistics], save_path: str = None):
    """
    绘制统计图表

    Args:
        stats_list: 统计信息列表
        save_path: 图表保存路径
    """
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('点云处理统计分析', fontsize=16, fontweight='bold')

    names = [stats.name for stats in stats_list]
    point_counts = [stats.point_count for stats in stats_list]

    # 1. 点数量变化柱状图
    ax1 = axes[0, 0]
    bars = ax1.bar(names, point_counts, color=['#1f77b4', '#ff7f0e', '#2ca02c'])
    ax1.set_ylabel('点数量', fontsize=12)
    ax1.set_title('各阶段点云数量对比', fontsize=12, fontweight='bold')
    ax1.grid(axis='y', alpha=0.3)

    # 在柱状图上标注数值
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{int(height):,}',
                ha='center', va='bottom', fontsize=10)

    # 2. 点数量保留率
    ax2 = axes[0, 1]
    if point_counts[0] > 0:
        retention_rates = [count / point_counts[0] * 100 for count in point_counts]
        bars2 = ax2.bar(names, retention_rates, color=['#1f77b4', '#ff7f0e', '#2ca02c'])
        ax2.set_ylabel('保留率 (%)', fontsize=12)
        ax2.set_title('点云数据保留率', fontsize=12, fontweight='bold')
        ax2.set_ylim([0, 110])
        ax2.grid(axis='y', alpha=0.3)

        # 标注百分比
        for bar in bars2:
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{height:.1f}%',
                    ha='center', va='bottom', fontsize=10)

    # 3. 空间范围对比
    ax3 = axes[1, 0]
    x_sizes = [stats.bbox_size[0] for stats in stats_list if stats.point_count > 0]
    y_sizes = [stats.bbox_size[1] for stats in stats_list if stats.point_count > 0]
    z_sizes = [stats.bbox_size[2] for stats in stats_list if stats.point_count > 0]
    valid_names = [stats.name for stats in stats_list if stats.point_count > 0]

    x = np.arange(len(valid_names))
    width = 0.25

    ax3.bar(x - width, x_sizes, width, label='X跨度', color='#d62728')
    ax3.bar(x, y_sizes, width, label='Y跨度', color='#9467bd')
    ax3.bar(x + width, z_sizes, width, label='Z跨度', color='#8c564b')

    ax3.set_ylabel('距离 (米)', fontsize=12)
    ax3.set_title('空间范围对比', fontsize=12, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(valid_names)
    ax3.legend()
    ax3.grid(axis='y', alpha=0.3)

    # 4. 数据处理流程摘要表格
    ax4 = axes[1, 1]
    ax4.axis('off')

    table_data = []
    for i, stats in enumerate(stats_list):
        if i == 0:
            change = "-"
            retention = "100.00%"
        else:
            change = f"{stats.point_count - stats_list[0].point_count:+,}"
            retention = f"{stats.point_count / stats_list[0].point_count * 100:.2f}%" if stats_list[0].point_count > 0 else "0%"

        table_data.append([
            stats.name,
            f"{stats.point_count:,}",
            change,
            retention
        ])

    table = ax4.table(cellText=table_data,
                     colLabels=['阶段', '点数量', '变化量', '保留率'],
                     cellLoc='center',
                     loc='center',
                     colWidths=[0.3, 0.25, 0.25, 0.2])

    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)

    # 设置表头样式
    for i in range(4):
        table[(0, i)].set_facecolor('#4CAF50')
        table[(0, i)].set_text_props(weight='bold', color='white')

    # 设置交替行颜色
    for i in range(1, len(table_data) + 1):
        for j in range(4):
            if i % 2 == 0:
                table[(i, j)].set_facecolor('#f0f0f0')

    ax4.set_title('处理流程数据摘要', fontsize=12, fontweight='bold', pad=20)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n统计图表已保存到: {save_path}")

    plt.show()


def generate_report(stats_list: List[PointCloudStatistics],
                   clusters: List[o3d.geometry.PointCloud],
                   output_file: str = None):
    """
    生成详细的处理报告

    Args:
        stats_list: 统计信息列表
        clusters: 聚类结果
        output_file: 报告输出文件路径
    """
    report_lines = []

    report_lines.append("="*80)
    report_lines.append("点云自动标定预处理报告")
    report_lines.append("="*80)
    report_lines.append("")

    # 1. 各阶段统计
    report_lines.append("1. 处理流程统计")
    report_lines.append("-" * 80)
    for stats in stats_list:
        report_lines.append(f"\n{stats.name}:")
        report_lines.append(f"  点数量: {stats.point_count:,}")
        if stats.point_count > 0:
            report_lines.append(f"  空间范围: X={stats.bbox_size[0]:.3f}m, Y={stats.bbox_size[1]:.3f}m, Z={stats.bbox_size[2]:.3f}m")
            report_lines.append(f"  中心位置: ({stats.center[0]:.3f}, {stats.center[1]:.3f}, {stats.center[2]:.3f})")

    # 2. 聚类结果
    report_lines.append(f"\n\n2. 聚类结果")
    report_lines.append("-" * 80)
    report_lines.append(f"检测到 {len(clusters)} 个有效点云簇")
    report_lines.append("")

    for i, cluster in enumerate(clusters):
        points = np.asarray(cluster.points)
        center = points.mean(axis=0)
        bbox_min = points.min(axis=0)
        bbox_max = points.max(axis=0)
        size = bbox_max - bbox_min

        report_lines.append(f"簇 {i}:")
        report_lines.append(f"  点数量: {len(points):,}")
        report_lines.append(f"  中心位置: ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})")
        report_lines.append(f"  尺寸: X={size[0]:.3f}m, Y={size[1]:.3f}m, Z={size[2]:.3f}m")
        report_lines.append("")

    report_lines.append("="*80)

    # 打印到控制台
    report_text = "\n".join(report_lines)
    print("\n" + report_text)

    # 保存到文件
    if output_file:
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(report_text)
        print(f"\n报告已保存到: {output_file}")


if __name__ == "__main__":
    print("可视化和统计分析模块")
    print("请通过主程序调用此模块")
