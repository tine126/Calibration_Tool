"""
相机点云采集模块
支持Orbbec深度相机（Gemini 2L等）
实现多帧融合和点云质量检测功能
"""

import open3d as o3d
import numpy as np
from typing import Optional, List, Tuple
import time


class CameraCapture:
    """
    Orbbec深度相机点云采集类
    支持多帧融合和质量检测
    专为Orbbec Gemini 2L优化
    """

    def __init__(self, config: dict):
        """
        初始化相机采集器

        Args:
            config: 相机配置字典
        """
        self.config = config
        self.camera_type = config['camera']['type']
        self.camera = None
        self.pipeline = None

        # 初始化相机
        self._initialize_camera()

    def _initialize_camera(self):
        """初始化深度相机"""
        print(f"\n初始化 {self.camera_type} 相机...")

        if self.camera_type == 'orbbec':
            self._initialize_orbbec()
        else:
            raise ValueError(f"不支持的相机类型: {self.camera_type}，当前仅支持 'orbbec'")

        print(f"相机初始化成功!")

    def _initialize_orbbec(self):
        """初始化Orbbec相机（Gemini 2L）"""
        try:
            from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat, PointCloudFilter, OBError

            # 创建pipeline
            self.pipeline = Pipeline()

            # 创建配置
            config = Config()

            # 配置深度流 - 使用默认配置
            try:
                depth_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                if depth_profile_list is None:
                    raise RuntimeError("未找到深度传感器配置")

                depth_profile = depth_profile_list.get_default_video_stream_profile()
                config.enable_stream(depth_profile)
                print(f"  深度流配置成功")
            except Exception as e:
                print(f"  错误: 配置深度流失败: {e}")
                raise

            # 配置彩色流 - 使用默认配置
            try:
                color_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                if color_profile_list:
                    color_profile = color_profile_list.get_default_video_stream_profile()
                    config.enable_stream(color_profile)
                    print(f"  彩色流配置成功")
                    self.has_color_sensor = True
                else:
                    print(f"  警告: 无法获取彩色流配置，将仅使用深度数据")
                    self.has_color_sensor = False
            except OBError as e:
                print(f"  警告: 彩色流配置失败: {e}")
                self.has_color_sensor = False

            # 启动pipeline
            self.pipeline.start(config)
            print(f"  Orbbec相机已启动")

            # 获取相机内参
            try:
                camera_param = self.pipeline.get_camera_param()
                self.camera_intrinsics = camera_param
                print(f"  已获取相机内参")

                # 创建点云滤波器并设置相机参数
                self.point_cloud_filter = PointCloudFilter()
                self.point_cloud_filter.set_camera_param(camera_param)
                print(f"  点云滤波器已配置相机参数")
            except Exception as e:
                print(f"  警告: 配置点云滤波器失败: {e}")
                # 如果设置参数失败，尝试不带参数创建
                self.point_cloud_filter = PointCloudFilter()
                self.camera_intrinsics = None

            # 等待相机稳定
            print("  等待相机稳定...")
            stable_count = 0
            for i in range(30):
                try:
                    frames = self.pipeline.wait_for_frames(1000)
                    if frames is not None:
                        stable_count += 1
                except Exception as e:
                    if i == 0:
                        print(f"  等待帧时出现警告: {e}")
                    continue

            print(f"  相机稳定完成 (成功接收 {stable_count}/30 帧)")

        except ImportError:
            raise ImportError("未安装pyorbbecsdk库。请运行: pip install pyorbbecsdk")
        except Exception as e:
            raise RuntimeError(f"Orbbec相机初始化失败: {e}")

    def capture_single_frame(self) -> Optional[o3d.geometry.PointCloud]:
        """
        采集单帧点云（Orbbec Gemini 2L）

        Returns:
            点云对象，失败返回None
        """
        return self._capture_orbbec_frame()

    def _capture_orbbec_frame(self) -> Optional[o3d.geometry.PointCloud]:
        """采集Orbbec单帧点云"""
        try:
            from pyorbbecsdk import OBFormat

            print(f"    [调试] 开始等待帧...")
            # 等待帧（位置参数，单位：毫秒）
            frames = self.pipeline.wait_for_frames(1000)

            if frames is None:
                print(f"  警告: wait_for_frames返回None")
                return None

            print(f"    [调试] 帧接收成功，正在获取深度帧...")
            # 获取深度帧
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                print(f"  警告: 深度帧为None")
                return None

            print(f"    [调试] 深度帧获取成功，检查彩色帧...")
            # 获取彩色帧（如果可用）
            color_frame = frames.get_color_frame() if self.has_color_sensor else None

            # 暂时强制使用POINT格式（仅深度），避免RGB_POINT可能的问题
            print(f"    [调试] 使用POINT格式生成点云（仅深度）...")
            point_format = OBFormat.POINT

            # # 根据是否有彩色帧选择点云格式
            # if color_frame is not None:
            #     print(f"    [调试] 使用RGB_POINT格式生成点云...")
            #     point_format = OBFormat.RGB_POINT
            # else:
            #     print(f"    [调试] 使用POINT格式生成点云（仅深度）...")
            #     point_format = OBFormat.POINT

            self.point_cloud_filter.set_create_point_format(point_format)

            print(f"    [调试] 准备处理帧集合...")
            print(f"    [调试] frames类型: {type(frames)}")
            print(f"    [调试] depth_frame类型: {type(depth_frame)}")
            print(f"    [调试] 点云格式: {point_format}")

            # 尝试获取帧集合的更多信息
            try:
                print(f"    [调试] 深度帧宽度: {depth_frame.get_width()}, 高度: {depth_frame.get_height()}")
            except Exception as e:
                print(f"    [调试] 无法获取帧尺寸: {e}")

            print(f"    [调试] 开始调用process()...")
            import sys
            sys.stdout.flush()  # 强制刷新输出

            # 关键修复：process方法传入frames（帧集合），而不是depth_frame
            point_cloud_frame = self.point_cloud_filter.process(frames)

            print(f"    [调试] process()调用完成")
            print(f"    [调试] point_cloud_frame类型: {type(point_cloud_frame)}")

            print(f"    [调试] 计算点云数据...")
            # 计算点云数据
            points = self.point_cloud_filter.calculate(point_cloud_frame)

            if len(points) == 0:
                print(f"  警告: 生成的点云为空")
                return None

            print(f"    [调试] 提取点坐标，原始点数: {len(points)}...")
            # 提取点坐标（将毫米转换为米）
            points_array = np.asarray(points)

            # 根据点云格式提取数据
            if point_format == OBFormat.RGB_POINT and points_array.shape[1] >= 6:
                # RGB_POINT格式: [x, y, z, r, g, b]
                positions = points_array[:, :3] * 0.001  # mm to m
                colors = points_array[:, 3:6] / 255.0     # 归一化到[0,1]
            else:
                # POINT格式: [x, y, z]
                positions = points_array.reshape(-1, 3) * 0.001  # mm to m
                colors = None

            print(f"    [调试] 过滤无效点...")
            # 过滤无效点：
            # 1. 移除距离原点过近的点（< 0.1m）
            # 2. 移除超出深度范围的点
            distances = np.linalg.norm(positions, axis=1)

            # 获取深度范围配置（单位：米）
            depth_min = self.config['camera']['depth_range'][0]  # 默认0.3m
            depth_max = self.config['camera']['depth_range'][1]  # 默认10.0m

            valid_mask = (distances > depth_min) & (distances < depth_max)

            print(f"    [调试] 深度范围: {depth_min}m - {depth_max}m")
            print(f"    [调试] 过滤前: {len(positions)} 点")
            print(f"    [调试] 过滤后: {np.sum(valid_mask)} 点")

            if np.sum(valid_mask) == 0:
                print(f"  警告: 过滤后没有有效点")
                return None

            positions = positions[valid_mask]
            if colors is not None:
                colors = colors[valid_mask]

            print(f"    [调试] 创建Open3D点云对象，有效点数: {len(positions)}...")
            # 创建Open3D点云对象
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(positions)
            if colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(colors)

            # 检查点云空间范围
            if len(positions) > 0:
                bbox = positions.max(axis=0) - positions.min(axis=0)
                print(f"  成功生成点云: {len(pcd.points):,} 点, 范围: X={bbox[0]:.2f}m Y={bbox[1]:.2f}m Z={bbox[2]:.2f}m")
            else:
                print(f"  成功生成点云: {len(pcd.points):,} 点")

            return pcd

        except Exception as e:
            print(f"警告: 采集Orbbec帧失败: {e}")
            import traceback
            traceback.print_exc()
            return None

    def capture_multi_frames(self, num_frames: int) -> List[o3d.geometry.PointCloud]:
        """
        采集多帧点云

        Args:
            num_frames: 采集帧数

        Returns:
            点云列表
        """
        print(f"\n开始采集 {num_frames} 帧点云...")

        point_clouds = []
        success_count = 0

        # 直接开始采集，不需要清空缓冲区
        # （相机在初始化时已经稳定了30帧）

        for i in range(num_frames):
            print(f"  正在采集第 {i+1}/{num_frames} 帧...")
            pcd = self.capture_single_frame()

            if pcd is not None and len(pcd.points) > 0:
                point_clouds.append(pcd)
                success_count += 1
                print(f"  ✓ 采集第 {success_count}/{num_frames} 帧成功 (点数: {len(pcd.points):,})")
            else:
                print(f"  ✗ 第 {i+1} 帧采集失败，跳过")

            # 增加延迟，确保相机有足够时间准备下一帧
            if i < num_frames - 1:  # 最后一帧不需要延迟
                time.sleep(0.15)  # 150ms延迟

        print(f"\n成功采集 {success_count}/{num_frames} 帧")

        return point_clouds

    def fuse_point_clouds(self, point_clouds: List[o3d.geometry.PointCloud],
                         method: str = 'average') -> o3d.geometry.PointCloud:
        """
        融合多帧点云

        Args:
            point_clouds: 点云列表
            method: 融合方法 ('average' 或 'median')

        Returns:
            融合后的点云
        """
        if not point_clouds:
            raise ValueError("点云列表为空")

        if len(point_clouds) == 1:
            return point_clouds[0]

        print(f"\n使用 {method} 方法融合 {len(point_clouds)} 帧点云...")

        # 收集所有点和颜色
        all_points = []
        all_colors = []

        for pcd in point_clouds:
            all_points.append(np.asarray(pcd.points))
            if pcd.has_colors():
                all_colors.append(np.asarray(pcd.colors))

        # 合并所有点
        merged_points = np.vstack(all_points)
        merged_colors = np.vstack(all_colors) if all_colors else None

        # 创建临时点云
        temp_pcd = o3d.geometry.PointCloud()
        temp_pcd.points = o3d.utility.Vector3dVector(merged_points)
        if merged_colors is not None:
            temp_pcd.colors = o3d.utility.Vector3dVector(merged_colors)

        # 使用体素下采样进行融合（在每个体素内平均）
        # 体素大小越大，点云越薄，但细节越少
        # 注意：这个体素大小应该>=预处理阶段的降采样大小(10mm)
        voxel_size = 0.02  # 20mm，与预处理保持一致或更大
        fused_pcd = temp_pcd.voxel_down_sample(voxel_size)

        print(f"  原始点数: {len(merged_points):,}")
        print(f"  体素大小: {voxel_size*1000:.1f}mm")
        print(f"  融合后点数: {len(fused_pcd.points):,}")

        return fused_pcd

    def check_point_cloud_quality(self, pcd: o3d.geometry.PointCloud) -> Tuple[bool, List[str]]:
        """
        检查点云质量

        Args:
            pcd: 点云对象

        Returns:
            (是否合格, 检查消息列表)
        """
        quality_config = self.config.get('quality_check', {})

        if not quality_config.get('enabled', True):
            return True, ["质量检测已禁用"]

        messages = []
        is_valid = True

        # 检查1: 点数量
        min_points = quality_config.get('min_points', 50000)
        point_count = len(pcd.points)

        if point_count < min_points:
            is_valid = False
            messages.append(f"✗ 点数量不足: {point_count:,} < {min_points:,}")
        else:
            messages.append(f"✓ 点数量合格: {point_count:,}")

        # 检查2: 覆盖范围
        if quality_config.get('coverage_check', True):
            points = np.asarray(pcd.points)

            # 计算点云边界
            bbox_min = points.min(axis=0)
            bbox_max = points.max(axis=0)
            bbox_size = bbox_max - bbox_min

            # 检查XYZ范围（单位：米）
            # 注意：点云坐标已经是米为单位，bbox_size也是米
            min_x_range = 3.0  # 至少3米宽
            min_y_range = 3.0  # 至少3米深
            min_z_range = 2.5  # 至少2.5米高

            if bbox_size[0] < min_x_range:
                is_valid = False
                messages.append(f"✗ X方向范围不足: {bbox_size[0]:.2f}m < {min_x_range}m")
            else:
                messages.append(f"✓ X方向范围合格: {bbox_size[0]:.2f}m")

            if bbox_size[1] < min_y_range:
                is_valid = False
                messages.append(f"✗ Y方向范围不足: {bbox_size[1]:.2f}m < {min_y_range}m")
            else:
                messages.append(f"✓ Y方向范围合格: {bbox_size[1]:.2f}m")

            if bbox_size[2] < min_z_range:
                is_valid = False
                messages.append(f"✗ Z方向范围不足: {bbox_size[2]:.2f}m < {min_z_range}m")
            else:
                messages.append(f"✓ Z方向范围合格: {bbox_size[2]:.2f}m")

        return is_valid, messages

    def capture_pointcloud(self) -> Optional[o3d.geometry.PointCloud]:
        """
        采集点云（主接口）
        支持多帧融合和质量检测

        Returns:
            点云对象，失败返回None
        """
        fusion_config = self.config.get('multi_frame_fusion', {})
        quality_config = self.config.get('quality_check', {})

        # 是否启用多帧融合
        enable_fusion = fusion_config.get('enabled', True)
        num_frames = fusion_config.get('num_frames', 10) if enable_fusion else 1
        fusion_method = fusion_config.get('fusion_method', 'average')

        # 最大重试次数
        max_retries = quality_config.get('max_retries', 3) if quality_config.get('enabled', True) else 1

        for attempt in range(max_retries):
            if attempt > 0:
                print(f"\n第 {attempt + 1} 次尝试采集...")
                time.sleep(1)  # 重试前等待1秒

            # 采集点云
            if enable_fusion and num_frames > 1:
                point_clouds = self.capture_multi_frames(num_frames)
                if not point_clouds:
                    print(f"警告: 采集失败，没有有效帧")
                    continue

                pcd = self.fuse_point_clouds(point_clouds, method=fusion_method)
            else:
                pcd = self.capture_single_frame()
                if pcd is None:
                    print(f"警告: 采集失败")
                    continue

            # 质量检测
            is_valid, messages = self.check_point_cloud_quality(pcd)

            print(f"\n点云质量检测:")
            for msg in messages:
                print(f"  {msg}")

            if is_valid:
                print(f"\n✓ 点云质量合格!")
                return pcd
            else:
                print(f"\n✗ 点云质量不达标")
                if attempt < max_retries - 1:
                    print(f"将进行第 {attempt + 2} 次采集...")

        print(f"\n错误: 达到最大重试次数 ({max_retries})，点云质量仍不达标")
        return None

    def stop(self):
        """停止Orbbec相机"""
        if self.pipeline is not None:
            self.pipeline.stop()
            print("Orbbec相机已停止")

    def __del__(self):
        """析构函数"""
        self.stop()
