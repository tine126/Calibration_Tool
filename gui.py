"""
投篮系统自动标定工具 - Qt图形界面
提供简单模式和详细模式两种操作方式
"""

import sys
import os
import copy
import numpy as np
import yaml
import json
from pathlib import Path
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QTextEdit, QLabel,
                             QFileDialog, QMessageBox, QGroupBox, QProgressBar,
                             QSplitter, QTabWidget, QRadioButton, QButtonGroup)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QTextCursor

# 导入标定相关模块
import open3d as o3d
from preprocess import downsample_pointcloud, filter_noise, cluster_pointcloud
from ground_detection import (detect_ground_plane, filter_ground_noise,
                               compute_ground_normal, compute_rotation_matrix,
                               apply_rotation_and_compute_plane)
from hoop_detection import detect_hoop, verify_hoop
from visualization import PointCloudStatistics


class CalibrationWorker(QThread):
    """标定工作线程"""
    log_signal = pyqtSignal(str)
    progress_signal = pyqtSignal(int, str)
    finished_signal = pyqtSignal(dict)
    error_signal = pyqtSignal(str)

    def __init__(self, config_path, mode='simple', data_source_mode='offline', pcd_file=None):
        super().__init__()
        self.config_path = config_path
        self.mode = mode
        self.data_source_mode = data_source_mode  # 'offline' 或 'realtime'
        self.pcd_file = pcd_file  # 离线模式的点云文件路径
        self.current_step = 0

    def log(self, message):
        """发送日志消息"""
        self.log_signal.emit(message)

    def update_progress(self, percent, message):
        """更新进度"""
        self.progress_signal.emit(percent, message)

    def get_pointcloud(self, config):
        """获取点云数据（离线或实时）"""
        if self.data_source_mode == 'offline':
            # 离线模式：从文件加载
            pcd_file = self.pcd_file if self.pcd_file else config['data_source']['offline']['input_file']
            if not os.path.exists(pcd_file):
                raise FileNotFoundError(f"点云文件不存在: {pcd_file}")

            self.log(f"从文件加载点云: {pcd_file}")
            pcd = o3d.io.read_point_cloud(pcd_file)
            self.log(f"点云加载成功: {len(pcd.points):,} 点")
            return pcd

        elif self.data_source_mode == 'realtime':
            # 实时模式：从相机采集
            self.log("正在从相机采集点云...")
            try:
                from camera_capture import CameraCapture

                camera_config = config['data_source']['realtime']
                camera = CameraCapture(camera_config)

                pcd = camera.capture_pointcloud()

                if pcd is None:
                    raise RuntimeError("点云采集失败")

                self.log(f"点云采集成功: {len(pcd.points):,} 点")

                # 保存原始点云
                if config['output'].get('save_raw_pointcloud', True):
                    output_dir = config['output']['dir']
                    os.makedirs(output_dir, exist_ok=True)

                    from datetime import datetime
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    raw_pcd_file = os.path.join(output_dir, f"raw_pointcloud_{timestamp}.ply")
                    o3d.io.write_point_cloud(raw_pcd_file, pcd)
                    self.log(f"原始点云已保存: {raw_pcd_file}")

                return pcd

            except ImportError as e:
                raise ImportError(f"无法导入相机采集模块: {e}\n请确保已正确安装相机驱动和camera_capture.py模块")
            except Exception as e:
                raise RuntimeError(f"点云采集失败: {e}")

        else:
            raise ValueError(f"未知的数据源模式: {self.data_source_mode}")

    def run(self):
        """执行标定流程"""
        try:
            self.log("=" * 60)
            self.log("投篮系统自动标定工具")
            self.log("=" * 60)

            # 加载配置
            self.update_progress(5, "加载配置文件...")
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self.log(f"配置文件加载成功: {self.config_path}")
            self.log(f"数据源模式: {self.data_source_mode}")

            # 获取点云（离线或实时）
            self.update_progress(10, "获取点云数据...")
            pcd_original = self.get_pointcloud(config)

            # 预处理
            self.update_progress(20, "点云预处理...")
            preprocess_config = config['preprocessing']

            # 降采样
            pcd_downsampled = downsample_pointcloud(
                pcd_original,
                voxel_size=preprocess_config['downsample']['voxel_size']
            )
            self.log(f"降采样完成: {len(pcd_downsampled.points):,} 点")

            # 噪声滤波
            pcd_filtered = filter_noise(
                pcd_downsampled,
                nb_neighbors=preprocess_config['noise_filter']['nb_neighbors'],
                std_ratio=preprocess_config['noise_filter']['std_ratio']
            )
            self.log(f"噪声滤波完成: {len(pcd_filtered.points):,} 点")

            # 地面检测
            self.update_progress(40, "检测地面平面...")
            ground_config = config['ground_detection']

            plane_model, ground_inliers, ground_stats = detect_ground_plane(
                pcd_filtered,
                distance_threshold=ground_config['distance_threshold'],
                ransac_n=ground_config['ransac_n'],
                num_iterations=ground_config['num_iterations'],
                random_seed=ground_config.get('random_seed', 42)
            )
            self.log(f"地面平面检测完成: {len(ground_inliers):,} 地面点")

            # 过滤地面噪声
            self.update_progress(50, "过滤地面噪声...")
            valid_ground_inliers, noise_inliers, noise_stats = filter_ground_noise(
                pcd_filtered,
                ground_inliers,
                plane_model,
                distance_threshold=ground_config['noise_filter']['distance_threshold'],
                nb_neighbors=ground_config['noise_filter']['nb_neighbors'],
                std_ratio=ground_config['noise_filter']['std_ratio']
            )
            self.log(f"地面噪声过滤完成: {noise_stats['valid_count']:,} 有效地面点")

            # 计算旋转矩阵
            self.update_progress(60, "计算旋转矩阵...")
            ground_normal = compute_ground_normal(plane_model)
            rotation_matrix = compute_rotation_matrix(ground_normal)
            self.log(f"旋转矩阵计算完成")

            # 应用旋转和平移
            self.update_progress(70, "应用变换...")
            rotated_pcd, ground_plane_equation, translation_vector, rotation_stats = apply_rotation_and_compute_plane(
                pcd_filtered,
                valid_ground_inliers,
                rotation_matrix
            )
            self.log(f"点云变换完成")

            # 篮筐检测
            self.update_progress(80, "检测篮筐...")
            hoop_config = config['hoop_detection']

            hoop_info = detect_hoop(
                pcd_filtered,
                rotation_matrix,
                ground_plane_equation,
                translation_vector,
                standard_diameter=hoop_config['standard_diameter'] * 1000,
                diameter_tolerance=hoop_config['diameter_tolerance'] * 1000,
                height_range=(hoop_config['height_range'][0] * 1000, hoop_config['height_range'][1] * 1000),
                random_seed=hoop_config.get('random_seed', 42)
            )

            if hoop_info is None:
                raise RuntimeError("未能检测到篮筐")

            self.log(f"篮筐检测完成")
            self.log(f"  中心位置: ({hoop_info['center_3d'][0]/1000:.3f}, {hoop_info['center_3d'][1]/1000:.3f}, {hoop_info['center_3d'][2]/1000:.3f}) m")
            self.log(f"  离地高度: {hoop_info['height_above_ground']/1000:.3f} m")
            self.log(f"  直径: {hoop_info['diameter']:.1f} mm")

            # 保存结果
            self.update_progress(90, "保存结果...")
            output_dir = config['output']['dir']
            os.makedirs(output_dir, exist_ok=True)

            # 构建4x4变换矩阵
            # 步骤1: 地面对齐旋转
            # 步骤2: Y轴180度旋转
            rotation_y_180 = np.array([
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0]
            ])
            # 组合旋转矩阵
            combined_rotation = rotation_y_180 @ rotation_matrix

            # 构建4x4齐次变换矩阵
            transform_matrix_4x4 = np.eye(4)
            transform_matrix_4x4[:3, :3] = combined_rotation
            transform_matrix_4x4[:3, 3] = translation_vector

            # 保存配置文件
            config_output_file = os.path.join(output_dir, config['output']['config_file'])
            output_config = {
                "rotation_matrix": rotation_matrix.tolist(),
                "rotation_y_180": rotation_y_180.tolist(),
                "combined_rotation": combined_rotation.tolist(),
                "translation_vector": translation_vector.tolist(),
                "transform_matrix_4x4": transform_matrix_4x4.tolist(),
                "ground_normal": ground_normal.tolist(),
                "ground_plane_equation": ground_plane_equation.tolist(),
                "hoop_center": hoop_info['center_3d'].tolist(),
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
                    "hoop_detection": {
                        'diameter_mm': float(hoop_info['diameter']),
                        'height_above_ground_mm': float(hoop_info['height_above_ground']),
                        'fit_error_mm': float(hoop_info['fit_error']),
                        'z_std_mm': float(hoop_info['z_std']),
                        'inlier_count': int(hoop_info['inlier_count']),
                        'inlier_ratio': float(hoop_info['inlier_ratio'])
                    }
                }
            }

            with open(config_output_file, 'w', encoding='utf-8') as f:
                json.dump(output_config, f, indent=2, ensure_ascii=False)

            self.log(f"配置文件已保存: {config_output_file}")

            self.update_progress(100, "完成!")
            self.log("=" * 60)
            self.log("标定流程完成!")
            self.log("=" * 60)

            # 返回结果
            result = {
                'hoop_center': hoop_info['center_3d'],
                'transform_matrix_4x4': transform_matrix_4x4,
                'combined_rotation': combined_rotation,
                'translation_vector': translation_vector,
                'config_file': config_output_file,
                'hoop_info': hoop_info,
                'ground_stats': ground_stats,
                'rotation_stats': rotation_stats
            }

            self.finished_signal.emit(result)

        except Exception as e:
            import traceback
            error_msg = f"错误: {str(e)}\n{traceback.format_exc()}"
            self.error_signal.emit(error_msg)


class MainWindow(QMainWindow):
    """主窗口"""

    def __init__(self):
        super().__init__()
        self.config_path = "config.yaml"
        self.worker = None
        self.pcd_file = None  # 离线模式的点云文件
        self.init_ui()

    def init_ui(self):
        """初始化界面"""
        self.setWindowTitle("投篮系统自动标定工具")
        self.setGeometry(100, 100, 1200, 800)

        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 主布局
        main_layout = QVBoxLayout(central_widget)

        # 标题
        title_label = QLabel("投篮系统自动标定工具")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)

        # 配置文件选择
        config_group = QGroupBox("配置文件")
        config_layout = QHBoxLayout()
        self.config_label = QLabel(f"当前配置: {self.config_path}")
        config_layout.addWidget(self.config_label)
        self.config_btn = QPushButton("选择配置文件")
        self.config_btn.clicked.connect(self.select_config)
        config_layout.addWidget(self.config_btn)
        config_group.setLayout(config_layout)
        main_layout.addWidget(config_group)

        # 数据源选择区域
        data_source_group = QGroupBox("数据源选择")
        data_source_layout = QVBoxLayout()

        # 单选按钮组
        self.data_source_btn_group = QButtonGroup()

        # 离线模式
        offline_layout = QHBoxLayout()
        self.offline_radio = QRadioButton("离线模式（从文件加载）")
        self.offline_radio.setChecked(True)  # 默认选中
        self.offline_radio.toggled.connect(self.on_data_source_changed)
        self.data_source_btn_group.addButton(self.offline_radio)
        offline_layout.addWidget(self.offline_radio)

        self.select_file_btn = QPushButton("选择点云文件")
        self.select_file_btn.clicked.connect(self.select_pointcloud_file)
        offline_layout.addWidget(self.select_file_btn)

        self.file_label = QLabel("未选择文件（将使用配置文件中的路径）")
        offline_layout.addWidget(self.file_label)

        data_source_layout.addLayout(offline_layout)

        # 实时模式
        realtime_layout = QHBoxLayout()
        self.realtime_radio = QRadioButton("实时模式（从相机采集）")
        self.realtime_radio.toggled.connect(self.on_data_source_changed)
        self.data_source_btn_group.addButton(self.realtime_radio)
        realtime_layout.addWidget(self.realtime_radio)

        self.camera_info_label = QLabel("将使用配置文件中的相机参数")
        realtime_layout.addWidget(self.camera_info_label)

        data_source_layout.addLayout(realtime_layout)

        data_source_group.setLayout(data_source_layout)
        main_layout.addWidget(data_source_group)

        # 模式选择区域
        mode_group = QGroupBox("选择模式")
        mode_layout = QHBoxLayout()

        # 简单模式按钮
        self.simple_btn = QPushButton("简单模式\n(一键自动标定)")
        self.simple_btn.setMinimumHeight(80)
        self.simple_btn.clicked.connect(self.run_simple_mode)
        simple_btn_font = QFont()
        simple_btn_font.setPointSize(12)
        self.simple_btn.setFont(simple_btn_font)
        mode_layout.addWidget(self.simple_btn)

        # 详细模式按钮
        self.detail_btn = QPushButton("详细模式\n(分步执行+可视化)")
        self.detail_btn.setMinimumHeight(80)
        self.detail_btn.clicked.connect(self.open_detail_mode)
        self.detail_btn.setFont(simple_btn_font)
        mode_layout.addWidget(self.detail_btn)

        mode_group.setLayout(mode_layout)
        main_layout.addWidget(mode_group)

        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_label = QLabel("就绪")
        main_layout.addWidget(self.progress_label)
        main_layout.addWidget(self.progress_bar)

        # 日志区域
        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        log_layout.addWidget(self.log_text)

        # 清除日志按钮
        clear_btn = QPushButton("清除日志")
        clear_btn.clicked.connect(lambda: self.log_text.clear())
        log_layout.addWidget(clear_btn)

        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

        # 结果显示区域
        result_group = QGroupBox("标定结果")
        result_layout = QVBoxLayout()
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setFont(QFont("Consolas", 10))
        result_layout.addWidget(self.result_text)
        result_group.setLayout(result_layout)
        main_layout.addWidget(result_group)

        # 设置布局比例
        main_layout.setStretch(4, 2)  # 日志区域
        main_layout.setStretch(5, 1)  # 结果区域

    def select_config(self):
        """选择配置文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择配置文件", "", "YAML文件 (*.yaml *.yml)"
        )
        if file_path:
            self.config_path = file_path
            self.config_label.setText(f"当前配置: {self.config_path}")
            self.append_log(f"已选择配置文件: {self.config_path}")

    def select_pointcloud_file(self):
        """选择点云文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择点云文件", "", "点云文件 (*.ply *.pcd *.xyz)"
        )
        if file_path:
            self.pcd_file = file_path
            self.file_label.setText(f"已选择: {os.path.basename(file_path)}")
            self.append_log(f"已选择点云文件: {self.pcd_file}")

    def on_data_source_changed(self):
        """数据源模式切换"""
        if self.offline_radio.isChecked():
            self.select_file_btn.setEnabled(True)
            self.file_label.setEnabled(True)
        else:
            self.select_file_btn.setEnabled(False)
            self.file_label.setEnabled(False)

    def get_data_source_mode(self):
        """获取当前选择的数据源模式"""
        return 'offline' if self.offline_radio.isChecked() else 'realtime'

    def append_log(self, message):
        """添加日志"""
        self.log_text.append(message)
        self.log_text.moveCursor(QTextCursor.End)

    def update_progress(self, percent, message):
        """更新进度"""
        self.progress_bar.setValue(percent)
        self.progress_label.setText(message)

    def run_simple_mode(self):
        """运行简单模式"""
        if self.worker and self.worker.isRunning():
            QMessageBox.warning(self, "警告", "标定任务正在运行中...")
            return

        # 检查配置文件
        if not os.path.exists(self.config_path):
            QMessageBox.critical(self, "错误", f"配置文件不存在: {self.config_path}")
            return

        # 获取数据源模式
        data_source_mode = self.get_data_source_mode()

        # 如果是离线模式且选择了文件，检查文件是否存在
        if data_source_mode == 'offline' and self.pcd_file:
            if not os.path.exists(self.pcd_file):
                QMessageBox.critical(self, "错误", f"点云文件不存在: {self.pcd_file}")
                return

        self.log_text.clear()
        self.result_text.clear()
        self.append_log(f"启动简单模式...")
        self.append_log(f"数据源: {data_source_mode}")

        # 禁用按钮
        self.simple_btn.setEnabled(False)
        self.detail_btn.setEnabled(False)

        # 创建工作线程
        self.worker = CalibrationWorker(
            self.config_path,
            mode='simple',
            data_source_mode=data_source_mode,
            pcd_file=self.pcd_file
        )
        self.worker.log_signal.connect(self.append_log)
        self.worker.progress_signal.connect(self.update_progress)
        self.worker.finished_signal.connect(self.on_calibration_finished)
        self.worker.error_signal.connect(self.on_calibration_error)
        self.worker.start()

    def on_calibration_finished(self, result):
        """标定完成"""
        self.simple_btn.setEnabled(True)
        self.detail_btn.setEnabled(True)

        # 显示结果
        self.result_text.clear()
        self.result_text.append("=" * 60)
        self.result_text.append("标定结果")
        self.result_text.append("=" * 60)

        # 篮筐中心位置
        hoop_center = result['hoop_center']
        self.result_text.append("\n【篮筐中心位置】")
        self.result_text.append(f"  X = {hoop_center[0]:.3f} mm = {hoop_center[0]/1000:.6f} m")
        self.result_text.append(f"  Y = {hoop_center[1]:.3f} mm = {hoop_center[1]/1000:.6f} m")
        self.result_text.append(f"  Z = {hoop_center[2]:.3f} mm = {hoop_center[2]/1000:.6f} m")

        # 4x4变换矩阵
        transform = result['transform_matrix_4x4']
        self.result_text.append("\n【4×4变换矩阵】(齐次坐标)")
        self.result_text.append("包含: 旋转(地面对齐+Y轴180°) + 平移")
        for i in range(4):
            row_str = "  ["
            for j in range(4):
                row_str += f"{transform[i, j]:+.6f}"
                if j < 3:
                    row_str += ", "
            row_str += "]"
            self.result_text.append(row_str)

        # 使用说明
        self.result_text.append("\n【使用说明】")
        self.result_text.append("应用此变换矩阵到点 P 的步骤:")
        self.result_text.append("1. 将3D点转换为齐次坐标: P_h = [x, y, z, 1]")
        self.result_text.append("2. 应用变换: P_transformed = T × P_h")
        self.result_text.append("3. 结果前3个分量即为变换后坐标")

        # 附加信息
        hoop_info = result['hoop_info']
        self.result_text.append("\n【篮筐检测信息】")
        self.result_text.append(f"  直径: {hoop_info['diameter']:.1f} mm ({hoop_info['diameter']/1000:.3f} m)")
        self.result_text.append(f"  离地高度: {hoop_info['height_above_ground']:.1f} mm ({hoop_info['height_above_ground']/1000:.3f} m)")
        self.result_text.append(f"  拟合误差: {hoop_info['fit_error']:.2f} mm")
        self.result_text.append(f"  内点数: {hoop_info['inlier_count']} ({hoop_info['inlier_ratio']*100:.1f}%)")

        self.result_text.append("\n【输出文件】")
        self.result_text.append(f"  配置文件: {result['config_file']}")
        self.result_text.append("\n" + "=" * 60)

        QMessageBox.information(self, "完成", "标定完成！结果已保存。")

    def on_calibration_error(self, error_msg):
        """标定出错"""
        self.simple_btn.setEnabled(True)
        self.detail_btn.setEnabled(True)
        self.append_log(error_msg)
        QMessageBox.critical(self, "错误", f"标定失败:\n{error_msg}")

    def open_detail_mode(self):
        """打开详细模式窗口"""
        # 检查配置文件
        if not os.path.exists(self.config_path):
            QMessageBox.critical(self, "错误", f"配置文件不存在: {self.config_path}")
            return

        # 获取数据源模式
        data_source_mode = self.get_data_source_mode()

        # 如果是离线模式且选择了文件，检查文件是否存在
        if data_source_mode == 'offline' and self.pcd_file:
            if not os.path.exists(self.pcd_file):
                QMessageBox.critical(self, "错误", f"点云文件不存在: {self.pcd_file}")
                return

        # 创建详细模式窗口
        self.detail_window = DetailModeWindow(
            self.config_path,
            data_source_mode=data_source_mode,
            pcd_file=self.pcd_file
        )
        self.detail_window.show()


class DetailModeWindow(QMainWindow):
    """详细模式窗口"""

    def __init__(self, config_path, data_source_mode='offline', pcd_file=None):
        super().__init__()
        self.config_path = config_path
        self.data_source_mode = data_source_mode
        self.pcd_file = pcd_file
        self.config = None
        self.pcd_original = None
        self.pcd_filtered = None
        self.rotation_matrix = None
        self.translation_vector = None
        self.ground_plane_equation = None
        self.valid_ground_inliers = None
        self.hoop_info = None
        self.current_step = 0

        self.init_ui()
        self.load_config()

    def init_ui(self):
        """初始化界面"""
        self.setWindowTitle("详细模式 - 分步执行")
        self.setGeometry(150, 150, 1400, 900)

        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 主布局
        main_layout = QHBoxLayout(central_widget)

        # 左侧：步骤控制面板
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_panel.setMaximumWidth(400)

        # 步骤标题
        step_label = QLabel("标定步骤")
        step_font = QFont()
        step_font.setPointSize(14)
        step_font.setBold(True)
        step_label.setFont(step_font)
        left_layout.addWidget(step_label)

        # 创建步骤按钮
        self.step_buttons = []
        steps = [
            ("1. 加载点云", self.step1_load_pointcloud),
            ("2. 降采样", self.step2_downsample),
            ("3. 噪声滤波", self.step3_filter_noise),
            ("4. 地面检测", self.step4_detect_ground),
            ("5. 过滤地面噪声", self.step5_filter_ground_noise),
            ("6. 计算旋转矩阵", self.step6_compute_rotation),
            ("7. 应用变换+拟合平面", self.step7_apply_transform),
            ("8. 篮筐检测", self.step8_detect_hoop),
            ("9. 查看结果", self.step9_show_results)
        ]

        for i, (text, callback) in enumerate(steps):
            btn = QPushButton(text)
            btn.setMinimumHeight(50)
            btn.setFont(QFont("Arial", 10))
            btn.clicked.connect(callback)
            btn.setEnabled(i == 0)  # 只启用第一个按钮
            self.step_buttons.append(btn)
            left_layout.addWidget(btn)

        # 可视化按钮
        left_layout.addWidget(QLabel(""))
        vis_label = QLabel("可视化")
        vis_label.setFont(step_font)
        left_layout.addWidget(vis_label)

        self.vis_btn = QPushButton("查看当前点云")
        self.vis_btn.setMinimumHeight(50)
        self.vis_btn.clicked.connect(self.visualize_current)
        self.vis_btn.setEnabled(False)
        left_layout.addWidget(self.vis_btn)

        # 重置按钮
        left_layout.addWidget(QLabel(""))
        reset_btn = QPushButton("重置流程")
        reset_btn.setMinimumHeight(40)
        reset_btn.clicked.connect(self.reset_workflow)
        left_layout.addWidget(reset_btn)

        left_layout.addStretch()

        # 右侧：日志和结果显示
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # 当前步骤标签
        self.current_step_label = QLabel("当前步骤: 未开始")
        current_step_font = QFont()
        current_step_font.setPointSize(12)
        current_step_font.setBold(True)
        self.current_step_label.setFont(current_step_font)
        right_layout.addWidget(self.current_step_label)

        # 标签页
        self.tab_widget = QTabWidget()

        # 日志标签页
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        self.tab_widget.addTab(self.log_text, "运行日志")

        # 结果标签页
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setFont(QFont("Consolas", 9))
        self.tab_widget.addTab(self.result_text, "当前结果")

        # 统计信息标签页
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setFont(QFont("Consolas", 9))
        self.tab_widget.addTab(self.stats_text, "统计信息")

        right_layout.addWidget(self.tab_widget)

        # 添加到主布局
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel)
        main_layout.setStretch(1, 1)

    def load_config(self):
        """加载配置"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            self.append_log(f"配置文件加载成功: {self.config_path}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载配置文件失败: {str(e)}")

    def append_log(self, message):
        """添加日志"""
        self.log_text.append(message)
        self.log_text.moveCursor(QTextCursor.End)

    def update_result(self, message):
        """更新结果显示"""
        self.result_text.append(message)
        self.result_text.moveCursor(QTextCursor.End)

    def enable_next_step(self):
        """启用下一步按钮"""
        if self.current_step < len(self.step_buttons):
            self.step_buttons[self.current_step].setEnabled(True)

    def step1_load_pointcloud(self):
        """步骤1: 获取点云（离线或实时）"""
        try:
            self.current_step_label.setText("当前步骤: 1. 获取点云")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤1: 获取点云数据")
            self.append_log("=" * 60)
            self.append_log(f"数据源模式: {self.data_source_mode}")

            if self.data_source_mode == 'offline':
                # 离线模式：从文件加载
                pcd_file = self.pcd_file if self.pcd_file else self.config['data_source']['offline']['input_file']
                if not os.path.exists(pcd_file):
                    raise FileNotFoundError(f"点云文件不存在: {pcd_file}")

                self.append_log(f"从文件加载点云: {pcd_file}")
                self.pcd_original = o3d.io.read_point_cloud(pcd_file)
                self.append_log(f"点云加载成功!")

            elif self.data_source_mode == 'realtime':
                # 实时模式：从相机采集
                self.append_log("正在从相机采集点云...")
                try:
                    from camera_capture import CameraCapture

                    camera_config = self.config['data_source']['realtime']
                    camera = CameraCapture(camera_config)

                    self.pcd_original = camera.capture_pointcloud()

                    if self.pcd_original is None:
                        raise RuntimeError("点云采集失败")

                    self.append_log(f"点云采集成功!")

                    # 保存原始点云
                    if self.config['output'].get('save_raw_pointcloud', True):
                        output_dir = self.config['output']['dir']
                        os.makedirs(output_dir, exist_ok=True)

                        from datetime import datetime
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        raw_pcd_file = os.path.join(output_dir, f"raw_pointcloud_{timestamp}.ply")
                        o3d.io.write_point_cloud(raw_pcd_file, self.pcd_original)
                        self.append_log(f"原始点云已保存: {raw_pcd_file}")

                except ImportError as e:
                    raise ImportError(f"无法导入相机采集模块: {e}\n请确保已正确安装相机驱动和camera_capture.py模块")
                except Exception as e:
                    raise RuntimeError(f"点云采集失败: {e}")

            # 显示点云信息
            self.append_log(f"点云数量: {len(self.pcd_original.points):,}")
            self.append_log(f"是否有颜色: {self.pcd_original.has_colors()}")
            self.append_log(f"是否有法向量: {self.pcd_original.has_normals()}")

            # 更新结果
            self.result_text.clear()
            self.update_result("【原始点云】")
            self.update_result(f"点数: {len(self.pcd_original.points):,}")
            stats = PointCloudStatistics("原始点云", self.pcd_original)
            self.update_result(f"边界框最小值: [{stats.bbox_min[0]:.3f}, {stats.bbox_min[1]:.3f}, {stats.bbox_min[2]:.3f}]")
            self.update_result(f"边界框最大值: [{stats.bbox_max[0]:.3f}, {stats.bbox_max[1]:.3f}, {stats.bbox_max[2]:.3f}]")
            self.update_result(f"边界框尺寸: [{stats.bbox_size[0]:.3f}, {stats.bbox_size[1]:.3f}, {stats.bbox_size[2]:.3f}]")

            # 启用可视化和下一步
            self.vis_btn.setEnabled(True)
            self.current_step = 1
            self.enable_next_step()

        except Exception as e:
            import traceback
            error_msg = f"获取点云失败: {str(e)}\n{traceback.format_exc()}"
            self.append_log(error_msg)
            QMessageBox.critical(self, "错误", error_msg)

    def step2_downsample(self):
        """步骤2: 降采样"""
        if self.pcd_original is None:
            QMessageBox.warning(self, "警告", "请先加载点云")
            return

        try:
            self.current_step_label.setText("当前步骤: 2. 降采样")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤2: 降采样")
            self.append_log("=" * 60)

            voxel_size = self.config['preprocessing']['downsample']['voxel_size']
            self.pcd_downsampled = downsample_pointcloud(self.pcd_original, voxel_size=voxel_size)

            self.append_log(f"降采样完成")
            self.append_log(f"体素大小: {voxel_size} m")
            self.append_log(f"原始点数: {len(self.pcd_original.points):,}")
            self.append_log(f"降采样后: {len(self.pcd_downsampled.points):,}")
            reduction = (1 - len(self.pcd_downsampled.points) / len(self.pcd_original.points)) * 100
            self.append_log(f"减少: {reduction:.2f}%")

            # 更新结果
            self.result_text.clear()
            self.update_result("【降采样后点云】")
            self.update_result(f"点数: {len(self.pcd_downsampled.points):,}")
            self.update_result(f"保留率: {100-reduction:.2f}%")

            self.current_step = 2
            self.enable_next_step()

        except Exception as e:
            QMessageBox.critical(self, "错误", f"降采样失败: {str(e)}")

    def step3_filter_noise(self):
        """步骤3: 噪声滤波"""
        if not hasattr(self, 'pcd_downsampled'):
            QMessageBox.warning(self, "警告", "请先执行降采样")
            return

        try:
            self.current_step_label.setText("当前步骤: 3. 噪声滤波")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤3: 噪声滤波")
            self.append_log("=" * 60)

            nb_neighbors = self.config['preprocessing']['noise_filter']['nb_neighbors']
            std_ratio = self.config['preprocessing']['noise_filter']['std_ratio']

            self.pcd_filtered = filter_noise(
                self.pcd_downsampled,
                nb_neighbors=nb_neighbors,
                std_ratio=std_ratio
            )

            self.append_log(f"噪声滤波完成")
            self.append_log(f"邻域点数: {nb_neighbors}")
            self.append_log(f"标准差倍数: {std_ratio}")
            self.append_log(f"滤波前: {len(self.pcd_downsampled.points):,}")
            self.append_log(f"滤波后: {len(self.pcd_filtered.points):,}")
            removed = len(self.pcd_downsampled.points) - len(self.pcd_filtered.points)
            self.append_log(f"移除噪声点: {removed:,}")

            # 更新结果
            self.result_text.clear()
            self.update_result("【滤波后点云】")
            self.update_result(f"点数: {len(self.pcd_filtered.points):,}")
            self.update_result(f"移除噪声: {removed:,}")

            self.current_step = 3
            self.enable_next_step()

        except Exception as e:
            QMessageBox.critical(self, "错误", f"噪声滤波失败: {str(e)}")

    def step4_detect_ground(self):
        """步骤4: 地面检测（RANSAC）"""
        if self.pcd_filtered is None:
            QMessageBox.warning(self, "警告", "请先执行噪声滤波")
            return

        try:
            self.current_step_label.setText("当前步骤: 4. 地面检测")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤4: 地面检测（RANSAC平面拟合）")
            self.append_log("=" * 60)

            ground_config = self.config['ground_detection']

            # 检测地面平面
            self.plane_model, self.ground_inliers, self.ground_stats = detect_ground_plane(
                self.pcd_filtered,
                distance_threshold=ground_config['distance_threshold'],
                ransac_n=ground_config['ransac_n'],
                num_iterations=ground_config['num_iterations'],
                random_seed=ground_config.get('random_seed', 42)
            )

            self.append_log(f"地面平面检测完成")
            self.append_log(f"地面点数（未过滤）: {len(self.ground_inliers):,}")
            self.append_log(f"地面法向量与Z轴夹角: {self.ground_stats['z_angle']:.2f}°")

            # 更新结果
            self.result_text.clear()
            self.update_result("【地面平面检测结果】")
            self.update_result(f"地面点数（RANSAC）: {len(self.ground_inliers):,}")
            self.update_result(f"内点比例: {self.ground_stats['inlier_ratio']*100:.2f}%")
            self.update_result(f"地面Z均值: {self.ground_stats['z_mean']:.1f} mm")
            self.update_result(f"地面Z标准差: {self.ground_stats['z_std']:.1f} mm")
            self.update_result(f"法向量与Z轴夹角: {self.ground_stats['z_angle']:.2f}°")
            self.update_result(f"")
            self.update_result(f"平面方程: {self.plane_model[0]:.6f}*x + {self.plane_model[1]:.6f}*y + {self.plane_model[2]:.6f}*z + {self.plane_model[3]:.6f} = 0")

            self.current_step = 4
            self.enable_next_step()

        except Exception as e:
            import traceback
            QMessageBox.critical(self, "错误", f"地面检测失败: {str(e)}\n{traceback.format_exc()}")

    def step5_filter_ground_noise(self):
        """步骤5: 过滤地面噪声"""
        if not hasattr(self, 'plane_model') or self.plane_model is None:
            QMessageBox.warning(self, "警告", "请先执行地面检测")
            return

        try:
            self.current_step_label.setText("当前步骤: 5. 过滤地面噪声")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤5: 过滤地面噪声")
            self.append_log("=" * 60)

            ground_config = self.config['ground_detection']

            # 过滤地面噪声
            self.valid_ground_inliers, self.noise_inliers, self.noise_stats = filter_ground_noise(
                self.pcd_filtered,
                self.ground_inliers,
                self.plane_model,
                distance_threshold=ground_config['noise_filter']['distance_threshold'],
                nb_neighbors=ground_config['noise_filter']['nb_neighbors'],
                std_ratio=ground_config['noise_filter']['std_ratio']
            )

            self.append_log(f"地面噪声过滤完成")
            self.append_log(f"有效地面点: {self.noise_stats['valid_count']:,}")
            self.append_log(f"噪声点: {self.noise_stats['noise_count']:,}")
            self.append_log(f"有效比例: {self.noise_stats['valid_ratio']*100:.2f}%")

            # 更新结果
            self.result_text.clear()
            self.update_result("【地面噪声过滤结果】")
            self.update_result(f"原始地面点: {len(self.ground_inliers):,}")
            self.update_result(f"有效地面点: {self.noise_stats['valid_count']:,}")
            self.update_result(f"噪声点: {self.noise_stats['noise_count']:,}")
            self.update_result(f"有效比例: {self.noise_stats['valid_ratio']*100:.2f}%")
            self.update_result(f"")
            self.update_result(f"点到平面距离均值: {self.noise_stats['distance_mean']:.2f} mm")
            self.update_result(f"点到平面距离标准差: {self.noise_stats['distance_std']:.2f} mm")

            self.current_step = 5
            self.enable_next_step()

        except Exception as e:
            import traceback
            QMessageBox.critical(self, "错误", f"过滤地面噪声失败: {str(e)}\n{traceback.format_exc()}")

    def step6_compute_rotation(self):
        """步骤6: 计算旋转矩阵"""
        if not hasattr(self, 'valid_ground_inliers') or self.valid_ground_inliers is None:
            QMessageBox.warning(self, "警告", "请先过滤地面噪声")
            return

        try:
            self.current_step_label.setText("当前步骤: 6. 计算旋转矩阵")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤6: 计算旋转矩阵")
            self.append_log("=" * 60)

            ground_normal = compute_ground_normal(self.plane_model)
            self.rotation_matrix = compute_rotation_matrix(ground_normal)

            self.append_log(f"旋转矩阵计算完成")
            self.append_log(f"地面法向量: ({ground_normal[0]:.6f}, {ground_normal[1]:.6f}, {ground_normal[2]:.6f})")

            # 更新结果
            self.result_text.clear()
            self.update_result("【旋转矩阵】")
            self.update_result("地面对齐旋转矩阵 R (3×3):")
            for i in range(3):
                self.update_result(f"  [{self.rotation_matrix[i, 0]:+.6f}, {self.rotation_matrix[i, 1]:+.6f}, {self.rotation_matrix[i, 2]:+.6f}]")
            self.update_result("")
            self.update_result(f"将地面法向量 {ground_normal} 旋转到 [0, 0, 1]")

            self.current_step = 6
            self.enable_next_step()

        except Exception as e:
            QMessageBox.critical(self, "错误", f"计算旋转矩阵失败: {str(e)}")

    def step7_apply_transform(self):
        """步骤7: 应用变换并拟合地面平面方程"""
        if self.rotation_matrix is None:
            QMessageBox.warning(self, "警告", "请先计算旋转矩阵")
            return

        try:
            self.current_step_label.setText("当前步骤: 7. 应用变换+拟合平面")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤7: 应用变换并拟合地面平面方程")
            self.append_log("=" * 60)

            rotated_pcd, self.ground_plane_equation, self.translation_vector, rotation_stats = apply_rotation_and_compute_plane(
                self.pcd_filtered,
                self.valid_ground_inliers,
                self.rotation_matrix
            )

            self.rotated_pcd = rotated_pcd
            self.rotation_stats = rotation_stats

            self.append_log(f"变换应用完成")
            self.append_log(f"步骤1: 地面对齐旋转")
            self.append_log(f"步骤2: Y轴180度旋转")
            self.append_log(f"步骤3: 平移 ({self.translation_vector[0]:.1f}, {self.translation_vector[1]:.1f}, {self.translation_vector[2]:.1f}) mm")
            self.append_log(f"地面平面方程: z = 0 (固定)")
            self.append_log(f"旋转后地面法向量与Z轴夹角: {rotation_stats['normal_z_angle_deg']:.4f}°")

            # 更新结果
            self.result_text.clear()
            self.update_result("【变换结果】")
            self.update_result("完整变换流程:")
            self.update_result("  1. 应用地面对齐旋转矩阵 R")
            self.update_result("  2. 应用Y轴180度旋转")
            self.update_result("  3. 应用平移向量 T")
            self.update_result("")
            self.update_result(f"平移向量 T: [{self.translation_vector[0]:.1f}, {self.translation_vector[1]:.1f}, {self.translation_vector[2]:.1f}] mm")
            self.update_result(f"")
            self.update_result(f"【最终地面平面方程】")
            self.update_result(f"强制设置为: z = 0")
            self.update_result(f"实际地面Z均值偏差: {rotation_stats['z_mean']:.1f} mm")
            self.update_result(f"实际地面Z中位数偏差: {rotation_stats['z_median']:.1f} mm")
            self.update_result(f"实际地面Z标准差: {rotation_stats['z_std']:.1f} mm")

            self.current_step = 7
            self.enable_next_step()

        except Exception as e:
            import traceback
            QMessageBox.critical(self, "错误", f"应用变换失败: {str(e)}\n{traceback.format_exc()}")

    def step8_detect_hoop(self):
        """步骤8: 篮筐检测"""
        if self.ground_plane_equation is None or self.translation_vector is None:
            QMessageBox.warning(self, "警告", "请先应用变换")
            return

        try:
            self.current_step_label.setText("当前步骤: 8. 篮筐检测")
            self.append_log("\n" + "=" * 60)
            self.append_log("步骤8: 篮筐检测")
            self.append_log("=" * 60)

            hoop_config = self.config['hoop_detection']

            self.hoop_info = detect_hoop(
                self.pcd_filtered,
                self.rotation_matrix,
                self.ground_plane_equation,
                self.translation_vector,
                standard_diameter=hoop_config['standard_diameter'] * 1000,
                diameter_tolerance=hoop_config['diameter_tolerance'] * 1000,
                height_range=(hoop_config['height_range'][0] * 1000, hoop_config['height_range'][1] * 1000),
                random_seed=hoop_config.get('random_seed', 42)
            )

            if self.hoop_info is None:
                raise RuntimeError("未能检测到篮筐")

            self.append_log(f"篮筐检测完成")
            self.append_log(f"中心位置: ({self.hoop_info['center_3d'][0]/1000:.3f}, {self.hoop_info['center_3d'][1]/1000:.3f}, {self.hoop_info['center_3d'][2]/1000:.3f}) m")
            self.append_log(f"离地高度: {self.hoop_info['height_above_ground']/1000:.3f} m")
            self.append_log(f"直径: {self.hoop_info['diameter']:.1f} mm")

            # 更新结果
            self.result_text.clear()
            self.update_result("【篮筐检测结果】")
            self.update_result(f"中心位置:")
            self.update_result(f"  X = {self.hoop_info['center_3d'][0]:.3f} mm = {self.hoop_info['center_3d'][0]/1000:.6f} m")
            self.update_result(f"  Y = {self.hoop_info['center_3d'][1]:.3f} mm = {self.hoop_info['center_3d'][1]/1000:.6f} m")
            self.update_result(f"  Z = {self.hoop_info['center_3d'][2]:.3f} mm = {self.hoop_info['center_3d'][2]/1000:.6f} m")
            self.update_result(f"")
            self.update_result(f"直径: {self.hoop_info['diameter']:.1f} mm ({self.hoop_info['diameter']/1000:.3f} m)")
            self.update_result(f"离地高度: {self.hoop_info['height_above_ground']:.1f} mm ({self.hoop_info['height_above_ground']/1000:.3f} m)")
            self.update_result(f"拟合误差: {self.hoop_info['fit_error']:.2f} mm")
            self.update_result(f"内点数: {self.hoop_info['inlier_count']} ({self.hoop_info['inlier_ratio']*100:.1f}%)")

            self.current_step = 8
            self.enable_next_step()

        except Exception as e:
            import traceback
            QMessageBox.critical(self, "错误", f"篮筐检测失败: {str(e)}\n{traceback.format_exc()}")

    def step9_show_results(self):
        """步骤9: 查看完整结果"""
        if self.hoop_info is None:
            QMessageBox.warning(self, "警告", "请先完成篮筐检测")
            return

        try:
            self.current_step_label.setText("当前步骤: 9. 查看结果")

            # 构建4x4变换矩阵
            rotation_y_180 = np.array([
                [-1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, -1.0]
            ])
            combined_rotation = rotation_y_180 @ self.rotation_matrix

            transform_matrix_4x4 = np.eye(4)
            transform_matrix_4x4[:3, :3] = combined_rotation
            transform_matrix_4x4[:3, 3] = self.translation_vector

            # 显示完整结果
            self.result_text.clear()
            self.update_result("=" * 60)
            self.update_result("完整标定结果")
            self.update_result("=" * 60)

            self.update_result("\n【篮筐中心位置】")
            hoop_center = self.hoop_info['center_3d']
            self.update_result(f"  X = {hoop_center[0]:.3f} mm = {hoop_center[0]/1000:.6f} m")
            self.update_result(f"  Y = {hoop_center[1]:.3f} mm = {hoop_center[1]/1000:.6f} m")
            self.update_result(f"  Z = {hoop_center[2]:.3f} mm = {hoop_center[2]/1000:.6f} m")

            self.update_result("\n【4×4变换矩阵】(齐次坐标)")
            self.update_result("包含: 旋转(地面对齐+Y轴180°) + 平移")
            for i in range(4):
                row_str = "  ["
                for j in range(4):
                    row_str += f"{transform_matrix_4x4[i, j]:+.6f}"
                    if j < 3:
                        row_str += ", "
                row_str += "]"
                self.update_result(row_str)

            self.update_result("\n【篮筐信息】")
            self.update_result(f"  直径: {self.hoop_info['diameter']:.1f} mm ({self.hoop_info['diameter']/1000:.3f} m)")
            self.update_result(f"  离地高度: {self.hoop_info['height_above_ground']:.1f} mm ({self.hoop_info['height_above_ground']/1000:.3f} m)")
            self.update_result(f"  拟合误差: {self.hoop_info['fit_error']:.2f} mm")

            self.update_result("\n" + "=" * 60)

            QMessageBox.information(self, "完成", "标定流程全部完成！")

        except Exception as e:
            QMessageBox.critical(self, "错误", f"显示结果失败: {str(e)}")

    def visualize_current(self):
        """可视化当前点云"""
        try:
            if self.current_step == 0:
                QMessageBox.warning(self, "警告", "请先加载点云")
                return

            geometries = []
            title = ""

            # 根据当前步骤选择要显示的内容
            if self.current_step == 1:
                # 步骤1: 原始点云
                geometries = [self.pcd_original]
                title = "原始点云"

            elif self.current_step == 2:
                # 步骤2: 降采样后点云
                geometries = [self.pcd_downsampled]
                title = "降采样后点云"

            elif self.current_step == 3:
                # 步骤3: 滤波后点云
                geometries = [self.pcd_filtered]
                title = "滤波后点云"

            elif self.current_step == 4:
                # 步骤4: 地面检测结果（显示RANSAC检测的地面点）
                # 创建地面点云（蓝色）
                if hasattr(self, 'ground_inliers'):
                    ground_pcd = self.pcd_filtered.select_by_index(self.ground_inliers)
                    ground_pcd = copy.deepcopy(ground_pcd)
                    ground_pcd.paint_uniform_color([0.0, 0.5, 1.0])  # 蓝色

                    # 创建非地面点云（灰色）
                    all_indices = np.arange(len(self.pcd_filtered.points))
                    non_ground_indices = np.setdiff1d(all_indices, self.ground_inliers)
                    non_ground_pcd = self.pcd_filtered.select_by_index(non_ground_indices)
                    non_ground_pcd = copy.deepcopy(non_ground_pcd)
                    non_ground_pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 灰色

                    geometries = [ground_pcd, non_ground_pcd]
                    title = f"地面检测结果 (蓝色=RANSAC地面点 {len(self.ground_inliers):,}, 灰色=非地面)"
                else:
                    geometries = [self.pcd_filtered]
                    title = "滤波后点云"

            elif self.current_step == 5:
                # 步骤5: 过滤地面噪声后的结果
                if hasattr(self, 'valid_ground_inliers') and hasattr(self, 'noise_inliers'):
                    # 有效地面点（绿色）
                    valid_ground_pcd = self.pcd_filtered.select_by_index(self.valid_ground_inliers)
                    valid_ground_pcd = copy.deepcopy(valid_ground_pcd)
                    valid_ground_pcd.paint_uniform_color([0.0, 0.8, 0.0])  # 绿色

                    # 噪声点（红色）
                    noise_pcd = self.pcd_filtered.select_by_index(self.noise_inliers)
                    noise_pcd = copy.deepcopy(noise_pcd)
                    noise_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # 红色

                    # 其他点（灰色）
                    all_indices = np.arange(len(self.pcd_filtered.points))
                    other_indices = np.setdiff1d(all_indices, np.concatenate([self.valid_ground_inliers, self.noise_inliers]))
                    if len(other_indices) > 0:
                        other_pcd = self.pcd_filtered.select_by_index(other_indices)
                        other_pcd = copy.deepcopy(other_pcd)
                        other_pcd.paint_uniform_color([0.5, 0.5, 0.5])
                        geometries = [valid_ground_pcd, noise_pcd, other_pcd]
                    else:
                        geometries = [valid_ground_pcd, noise_pcd]

                    title = f"地面噪声过滤 (绿色=有效地面 {len(self.valid_ground_inliers):,}, 红色=噪声 {len(self.noise_inliers):,})"
                else:
                    geometries = [self.pcd_filtered]
                    title = "滤波后点云"

            elif self.current_step == 6:
                # 步骤6: 计算旋转矩阵（显示旋转前的有效地面点）
                if hasattr(self, 'valid_ground_inliers'):
                    # 有效地面点（绿色）
                    valid_ground_pcd = self.pcd_filtered.select_by_index(self.valid_ground_inliers)
                    valid_ground_pcd = copy.deepcopy(valid_ground_pcd)
                    valid_ground_pcd.paint_uniform_color([0.0, 0.8, 0.0])  # 绿色

                    # 非地面点（灰色）
                    all_indices = np.arange(len(self.pcd_filtered.points))
                    non_ground_indices = np.setdiff1d(all_indices, self.valid_ground_inliers)
                    non_ground_pcd = self.pcd_filtered.select_by_index(non_ground_indices)
                    non_ground_pcd = copy.deepcopy(non_ground_pcd)
                    non_ground_pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 灰色

                    geometries = [valid_ground_pcd, non_ground_pcd]

                    # 添加坐标系
                    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0, 0, 0])
                    geometries.append(coord_frame)

                    title = f"旋转矩阵计算完成 (绿色=有效地面点 {len(self.valid_ground_inliers):,})"
                else:
                    geometries = [self.pcd_filtered]
                    title = "旋转矩阵计算完成"

            elif self.current_step == 7:
                # 步骤7: 应用变换后的点云+地面平面
                if hasattr(self, 'rotated_pcd') and self.ground_plane_equation is not None:
                    geometries = [self.rotated_pcd]
                    # 添加坐标系
                    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0, 0, 0])
                    geometries.append(coord_frame)

                    # 添加地面参考平面
                    ground_z = -self.ground_plane_equation[3] / self.ground_plane_equation[2]
                    plane_size = 5000
                    vertices = np.array([
                        [-plane_size, -plane_size, ground_z],
                        [plane_size, -plane_size, ground_z],
                        [plane_size, plane_size, ground_z],
                        [-plane_size, plane_size, ground_z]
                    ])
                    triangles = np.array([[0, 1, 2], [0, 2, 3]])
                    plane_mesh = o3d.geometry.TriangleMesh()
                    plane_mesh.vertices = o3d.utility.Vector3dVector(vertices)
                    plane_mesh.triangles = o3d.utility.Vector3iVector(triangles)
                    plane_mesh.paint_uniform_color([0.3, 0.8, 0.3])
                    plane_mesh.compute_vertex_normals()
                    geometries.append(plane_mesh)

                    title = f"变换后点云 (绿色平面=地面 z={ground_z:.1f}mm)"
                else:
                    geometries = [self.pcd_filtered]
                    title = "变换后点云"

            elif self.current_step >= 8:
                # 步骤8及以后: 显示变换后点云+篮筐检测结果
                if hasattr(self, 'rotated_pcd'):
                    # 变换后点云
                    pcd_display = copy.deepcopy(self.rotated_pcd)
                    if not pcd_display.has_colors():
                        pcd_display.paint_uniform_color([0.6, 0.6, 0.6])
                    geometries.append(pcd_display)

                    # 坐标系
                    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0, 0, 0])
                    geometries.append(coord_frame)

                    # 如果已检测到篮筐，显示篮筐
                    if self.hoop_info is not None:
                        # 篮筐中心标记（黄色球体）
                        center_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=50)
                        center_sphere.translate(self.hoop_info['center_3d'])
                        center_sphere.paint_uniform_color([1.0, 1.0, 0.0])
                        geometries.append(center_sphere)

                        # 篮筐圆环（红色）
                        circle_points = []
                        num_points = 100
                        for i in range(num_points):
                            angle = 2 * np.pi * i / num_points
                            x = self.hoop_info['center_2d'][0] + self.hoop_info['radius'] * np.cos(angle)
                            y = self.hoop_info['center_2d'][1] + self.hoop_info['radius'] * np.sin(angle)
                            z = self.hoop_info['hoop_plane_z']
                            circle_points.append([x, y, z])

                        circle_pcd = o3d.geometry.PointCloud()
                        circle_pcd.points = o3d.utility.Vector3dVector(np.array(circle_points))
                        circle_pcd.paint_uniform_color([1.0, 0.0, 0.0])
                        geometries.append(circle_pcd)

                        title = f"篮筐检测结果 (黄色球=中心, 红色圆环=篮筐, 直径={self.hoop_info['diameter']:.0f}mm)"
                    else:
                        title = "变换后点云"
                else:
                    geometries = [self.pcd_filtered]
                    title = "当前点云"

            # 显示点云
            if geometries:
                self.append_log(f"\n显示: {title}")
                o3d.visualization.draw_geometries(
                    geometries,
                    window_name=title,
                    width=1200,
                    height=800
                )
            else:
                QMessageBox.warning(self, "警告", "没有可显示的点云")

        except Exception as e:
            import traceback
            error_msg = f"可视化失败: {str(e)}\n{traceback.format_exc()}"
            self.append_log(error_msg)
            QMessageBox.critical(self, "错误", error_msg)

    def reset_workflow(self):
        """重置工作流"""
        reply = QMessageBox.question(
            self, "确认", "确定要重置工作流吗？所有进度将丢失。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # 清空数据
            self.pcd_original = None
            self.pcd_filtered = None
            self.rotation_matrix = None
            self.translation_vector = None
            self.ground_plane_equation = None
            self.valid_ground_inliers = None
            self.hoop_info = None
            self.current_step = 0

            # 重置按钮状态
            for i, btn in enumerate(self.step_buttons):
                btn.setEnabled(i == 0)

            self.vis_btn.setEnabled(False)

            # 清空显示
            self.log_text.clear()
            self.result_text.clear()
            self.stats_text.clear()
            self.current_step_label.setText("当前步骤: 未开始")

            self.append_log("工作流已重置")


def main():
    """主程序入口"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
