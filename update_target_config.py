"""
配置文件更新模块
将标定工具的输出结果自动更新到目标config.yaml文件中
使用ruamel.yaml保留注释和格式
"""

import os
import json
import shutil
import time
from typing import Dict, Any, Optional

try:
    from ruamel.yaml import YAML
    HAS_RUAMEL = True
except ImportError:
    import yaml
    HAS_RUAMEL = False
    print("警告: 未安装ruamel.yaml，将使用标准yaml（会丢失注释）")
    print("建议安装: pip install ruamel.yaml")


def backup_config(target_config_path: str, backup_dir: str) -> Optional[str]:
    """
    备份目标配置文件

    Args:
        target_config_path: 目标配置文件路径 (如 f:\Code\config.yaml)
        backup_dir: 备份目录 (如 f:\Code\Calibration_Tool\backup_config)

    Returns:
        备份文件路径，如果失败则返回None
    """
    try:
        # 确保备份目录存在
        os.makedirs(backup_dir, exist_ok=True)

        # 检查目标文件是否存在
        if not os.path.exists(target_config_path):
            print(f"警告: 目标配置文件不存在: {target_config_path}")
            return None

        # 生成备份文件名: config_XXXXXXXXXX (unix时间戳)
        timestamp = int(time.time())
        backup_filename = f"config_{timestamp}.yaml"
        backup_path = os.path.join(backup_dir, backup_filename)

        # 复制文件
        shutil.copy2(target_config_path, backup_path)
        print(f"[OK] 配置文件已备份: {backup_path}")

        return backup_path

    except Exception as e:
        print(f"[ERROR] 备份配置文件失败: {e}")
        return None


def load_calibration_result(calibration_output_file: str) -> Optional[Dict[str, Any]]:
    """
    加载标定结果文件

    Args:
        calibration_output_file: 标定输出文件路径 (如 F:\Code\Calibration_Tool\output\config.json)

    Returns:
        标定结果字典，如果失败则返回None
    """
    try:
        if not os.path.exists(calibration_output_file):
            print(f"错误: 标定结果文件不存在: {calibration_output_file}")
            return None

        with open(calibration_output_file, 'r', encoding='utf-8') as f:
            result = json.load(f)

        print(f"[OK] 标定结果已加载: {calibration_output_file}")
        return result

    except Exception as e:
        print(f"[ERROR] 加载标定结果失败: {e}")
        return None


def update_target_config(
    target_config_path: str,
    calibration_result: Dict[str, Any],
    backup_dir: str = None
) -> bool:
    """
    使用标定结果更新目标配置文件（完全保留原文件格式，只替换指定值）

    Args:
        target_config_path: 目标配置文件路径 (如 f:\Code\config.yaml)
        calibration_result: 标定结果字典
        backup_dir: 备份目录，如果提供则先备份原文件

    Returns:
        是否成功更新
    """
    import re

    try:
        # 1. 备份原配置文件（如果指定了备份目录）
        if backup_dir:
            backup_path = backup_config(target_config_path, backup_dir)
            if not backup_path:
                print("警告: 备份失败，但将继续更新配置文件")

        # 2. 读取原始文件内容（纯文本方式）
        if not os.path.exists(target_config_path):
            print(f"错误: 目标配置文件不存在: {target_config_path}")
            return False

        with open(target_config_path, 'r', encoding='utf-8') as f:
            content = f.read()

        print(f"[OK] 目标配置文件已加载: {target_config_path}")

        # 3. 提取标定结果中的关键参数
        transform_matrix = calibration_result.get('transform_matrix_4x4')
        ground_normal = calibration_result.get('ground_normal')
        hoop_center = calibration_result.get('hoop_center')  # 单位：毫米

        # 检查必需参数
        if transform_matrix is None:
            print("错误: 标定结果中缺少 transform_matrix_4x4")
            return False

        # 注意：transform_matrix_4x4中的平移量已经是米为单位，不需要再转换
        # （在gui.py中保存时已经转换为米）
        print(f"[OK] 平移量已从标定结果中读取(米): tx={transform_matrix[0][3]:.4f}m, ty={transform_matrix[1][3]:.4f}m, tz={transform_matrix[2][3]:.4f}m")

        # 4. 使用正则表达式替换transform_matrix
        # 匹配多行块样式或内联样式的transform_matrix
        def replace_transform_matrix(match):
            # 提取缩进
            indent_match = re.match(r'(\s*)', match.group(0))
            base_indent = indent_match.group(1) if indent_match else '      '

            # 构建新的transform_matrix（内联格式）
            rows = []
            for row in transform_matrix:
                row_str = '[' + ', '.join(str(v) for v in row) + ']'
                rows.append(row_str)

            formatted = base_indent + 'transform_matrix: [\n'
            for i, row in enumerate(rows):
                formatted += base_indent + '  ' + row
                if i < len(rows) - 1:
                    formatted += ',\n'
                else:
                    formatted += '\n'
            formatted += base_indent + ']'

            return formatted

        # 匹配块样式的transform_matrix（4x4矩阵）
        pattern_block = r'\s*transform_matrix:\s*\n(?:\s+- - [^\n]+\n(?:\s+- [^\n]+\n){3}){4}'

        # 匹配内联样式的transform_matrix（可能跨多行）
        # 匹配从 "transform_matrix: [" 开始到对应的 "]" 结束
        pattern_inline = r'\s*transform_matrix:\s*\[[\s\S]*?\n\s+\]'

        # 尝试替换块样式
        new_content = re.sub(pattern_block, replace_transform_matrix, content, flags=re.MULTILINE)

        if new_content == content:
            # 如果块样式没匹配到，尝试内联样式
            new_content = re.sub(pattern_inline, replace_transform_matrix, content, flags=re.MULTILINE)

        if new_content != content:
            print("[OK] transform_matrix已更新为内联格式")
            content = new_content
        else:
            print("[WARNING] 未找到transform_matrix，跳过更新")


        # 5. 更新篮筐坐标（如果有）
        if hoop_center is not None:
            # 转换单位：毫米 -> 米
            hoop_x = round(hoop_center[0] / 1000.0, 3)
            hoop_y = round(hoop_center[1] / 1000.0, 3)
            hoop_z = round(hoop_center[2] / 1000.0, 3)

            print(f"\n篮筐中心坐标: X={hoop_x:.3f}m, Y={hoop_y:.3f}m, Z={hoop_z:.3f}m")

            # 使用正则表达式替换 trajectory_analysis 中的篮筐坐标
            # 匹配 hoop_x: 数值
            content = re.sub(
                r'(\s*hoop_x:\s*)[-+]?\d*\.?\d+',
                rf'\g<1>{hoop_x}',
                content
            )
            # 匹配 hoop_y: 数值
            content = re.sub(
                r'(\s*hoop_y:\s*)[-+]?\d*\.?\d+',
                rf'\g<1>{hoop_y}',
                content
            )
            # 匹配 hoop_height: 数值
            content = re.sub(
                r'(\s*hoop_height:\s*)[-+]?\d*\.?\d+',
                rf'\g<1>{hoop_z}',
                content
            )
            print("[OK] 已更新: trajectory_analysis.hoop_x, hoop_y, hoop_height")

            # 使用正则表达式替换 exclude_cylinder 中的篮筐坐标
            # 匹配 center_x: 数值
            content = re.sub(
                r'(exclude_cylinder:.*?center_x:\s*)[-+]?\d*\.?\d+',
                rf'\g<1>{hoop_x}',
                content,
                flags=re.DOTALL
            )
            # 更精确的匹配方式：逐个替换exclude_cylinder块内的坐标
            def replace_exclude_cylinder(match):
                cylinder_block = match.group(0)
                cylinder_block = re.sub(r'(center_x:\s*)[-+]?\d*\.?\d+', rf'\g<1>{hoop_x}', cylinder_block)
                cylinder_block = re.sub(r'(center_y:\s*)[-+]?\d*\.?\d+', rf'\g<1>{hoop_y}', cylinder_block)
                cylinder_block = re.sub(r'(center_z:\s*)[-+]?\d*\.?\d+', rf'\g<1>{hoop_z}', cylinder_block)
                return cylinder_block

            # 匹配exclude_cylinder块
            pattern_cylinder = r'exclude_cylinder:.*?(?=\n\s{0,6}\w|\Z)'
            content = re.sub(pattern_cylinder, replace_exclude_cylinder, content, flags=re.DOTALL)
            print("[OK] 已更新: ROI区域的exclude_cylinder坐标")
        else:
            print("[WARNING] 警告: 标定结果中没有篮筐坐标 (hoop_center)，跳过篮筐坐标更新")

        # 6. 保存更新后的配置文件
        with open(target_config_path, 'w', encoding='utf-8') as f:
            f.write(content)

        print(f"\n[OK] 目标配置文件已更新: {target_config_path}")
        return True

    except Exception as e:
        print(f"[ERROR] 更新配置文件失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def update_config_from_calibration(
    calibration_output_file: str,
    target_config_path: str,
    backup_dir: str
) -> bool:
    """
    从标定结果文件更新目标配置文件（一站式接口）

    Args:
        calibration_output_file: 标定输出文件路径
        target_config_path: 目标配置文件路径
        backup_dir: 备份目录

    Returns:
        是否成功
    """
    print("\n" + "="*80)
    print("开始更新目标配置文件")
    print("="*80)
    print(f"标定结果: {calibration_output_file}")
    print(f"目标配置: {target_config_path}")
    print(f"备份目录: {backup_dir}")
    print("-"*80)

    # 1. 加载标定结果
    calibration_result = load_calibration_result(calibration_output_file)
    if calibration_result is None:
        return False

    # 2. 更新目标配置文件
    success = update_target_config(target_config_path, calibration_result, backup_dir)

    if success:
        print("\n" + "="*80)
        print("配置文件更新完成!")
        print("="*80)
    else:
        print("\n" + "="*80)
        print("配置文件更新失败!")
        print("="*80)

    return success


if __name__ == "__main__":
    # 测试代码
    print("配置文件更新模块")
    print("请通过主程序或GUI调用此模块")

    # 示例用法
    print("\n使用示例:")
    print("-" * 80)
    print("""
from update_target_config import update_config_from_calibration

# 一站式更新
success = update_config_from_calibration(
    calibration_output_file="F:\\Code\\Calibration_Tool\\output\\config.json",
    target_config_path="F:\\Code\\config.yaml",
    backup_dir="F:\\Code\\Calibration_Tool\\backup_config"
)

if success:
    print("配置文件已成功更新！")
else:
    print("配置文件更新失败！")
    """)
