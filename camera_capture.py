"""
相机点云采集模块
支持多种深度相机（RealSense, Kinect, Orbbec等）
实现多帧融合和点云质量检测功能
"""

import open3d as o3d
import numpy as np
from typing import Optional, List, Tuple
import time


class CameraCapture:
    """
    深度相机点云采集类
    支持多帧融合和质量检测
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

        if self.camera_type == 'realsense':
            self._initialize_realsense()
        elif self.camera_type == 'kinect':
            self._initialize_kinect()
        elif self.camera_type == 'orbbec':
            self._initialize_orbbec()
        else:
            raise ValueError(f"不支持的相机类型: {self.camera_type}")

        print(f"相机初始化成功!")

    def _initialize_realsense(self):
        """初始化Intel RealSense相机"""
        try:
            import pyrealsense2 as rs

            # 创建pipeline
            self.pipeline = rs.pipeline()
            config = rs.config()

            # 配置相机参数
            camera_config = self.config['camera']
            serial_number = camera_config.get('serial_number', '')

            if serial_number:
                config.enable_device(serial_number)

            # 配置深度和颜色流
            resolution = camera_config['resolution']
            frame_rate = camera_config['frame_rate']

            config.enable_stream(rs.stream.depth, resolution['width'], resolution['height'],
                               rs.format.z16, frame_rate)
            config.enable_stream(rs.stream.color, resolution['width'], resolution['height'],
                               rs.format.rgb8, frame_rate)

            # 启动pipeline
            profile = self.pipeline.start(config)

            # 获取深度传感器
            depth_sensor = profile.get_device().first_depth_sensor()

            # 设置深度范围
            depth_range = camera_config['depth_range']
            depth_scale = depth_sensor.get_depth_scale()

            print(f"  分辨率: {resolution['width']}x{resolution['height']}")
            print(f"  帧率: {frame_rate} fps")
            print(f"  深度范围: {depth_range[0]}-{depth_range[1]} m")
            print(f"  深度缩放因子: {depth_scale}")

            # 等待相机稳定
            print("  等待相机稳定...")
            for _ in range(30):
                self.pipeline.wait_for_frames()

        except ImportError:
            raise ImportError("未安装pyrealsense2库。请运行: pip install pyrealsense2")
        except Exception as e:
            raise RuntimeError(f"RealSense相机初始化失败: {e}")

    def _initialize_kinect(self):
        """初始化Azure Kinect相机"""
        try:
            import pyk4a
            from pyk4a import Config, PyK4A

            # 配置Kinect
            camera_config = self.config['camera']
            resolution = camera_config['resolution']

            # 创建Kinect配置
            k4a_config = Config(
                color_resolution=pyk4a.ColorResolution.RES_720P,
                depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,
                synchronized_images_only=True
            )

            # 启动相机
            self.camera = PyK4A(k4a_config)
            self.camera.start()

            print(f"  Kinect相机已启动")
            print(f"  深度模式: NFOV_UNBINNED")

            # 等待相机稳定
            print("  等待相机稳定...")
            for _ in range(30):
                self.camera.get_capture()

        except ImportError:
            raise ImportError("未安装pyk4a库。请参考Azure Kinect SDK安装文档")
        except Exception as e:
            raise RuntimeError(f"Kinect相机初始化失败: {e}")

    def _initialize_orbbec(self):
        """初始化Orbbec相机"""
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

            # 创建点云滤波器
            self.point_cloud_filter = PointCloudFilter()

            # 启动pipeline
            self.pipeline.start(config)
            print(f"  Orbbec相机已启动")

            # 获取相机内参（可选，点云滤波器会自动使用）
            try:
                camera_param = self.pipeline.get_camera_param()
                self.camera_intrinsics = camera_param
                print(f"  已获取相机内参")
            except Exception as e:
                print(f"  警告: 无法获取相机内参 (点云滤波器将使用默认参数): {e}")
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
        采集单帧点云

        Returns:
            点云对象，失败返回None
        """
        if self.camera_type == 'realsense':
            return self._capture_realsense_frame()
        elif self.camera_type == 'kinect':
            return self._capture_kinect_frame()
        elif self.camera_type == 'orbbec':
            return self._capture_orbbec_frame()
        else:
            return None

    def _capture_realsense_frame(self) -> Optional[o3d.geometry.PointCloud]:
        """采集RealSense单帧点云"""
        try:
            import pyrealsense2 as rs

            # 等待帧
            frames = self.pipeline.wait_for_frames()

            # 对齐到颜色帧
            align = rs.align(rs.stream.color)
            aligned_frames = align.process(frames)

            # 获取对齐后的深度和颜色帧
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                return None

            # 转换为Open3D RGBD图像
            depth_image = o3d.geometry.Image(np.asanyarray(depth_frame.get_data()))
            color_image = o3d.geometry.Image(np.asanyarray(color_frame.get_data()))

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image, depth_image,
                depth_scale=1000.0,  # RealSense深度单位为毫米
                depth_trunc=self.config['camera']['depth_range'][1] * 1000,
                convert_rgb_to_intensity=False
            )

            # 获取相机内参
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            pinhole_camera = o3d.camera.PinholeCameraIntrinsic(
                intrinsics.width, intrinsics.height,
                intrinsics.fx, intrinsics.fy,
                intrinsics.ppx, intrinsics.ppy
            )

            # 生成点云
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, pinhole_camera
            )

            return pcd

        except Exception as e:
            print(f"警告: 采集RealSense帧失败: {e}")
            return None

    def _capture_kinect_frame(self) -> Optional[o3d.geometry.PointCloud]:
        """采集Kinect单帧点云"""
        try:
            # 获取捕获
            capture = self.camera.get_capture()

            if capture.depth is None or capture.color is None:
                return None

            # 转换为Open3D RGBD图像
            depth_image = o3d.geometry.Image(capture.transformed_depth)
            color_image = o3d.geometry.Image(capture.color)

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image, depth_image,
                depth_scale=1000.0,
                depth_trunc=self.config['camera']['depth_range'][1] * 1000,
                convert_rgb_to_intensity=False
            )

            # 获取相机内参
            calibration = self.camera.calibration
            intrinsics = calibration.get_camera_matrix(pyk4a.calibration.CalibrationType.COLOR)

            pinhole_camera = o3d.camera.PinholeCameraIntrinsic(
                capture.color.shape[1], capture.color.shape[0],
                intrinsics[0, 0], intrinsics[1, 1],
                intrinsics[0, 2], intrinsics[1, 2]
            )

            # 生成点云
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, pinhole_camera
            )

            return pcd

        except Exception as e:
            print(f"警告: 采集Kinect帧失败: {e}")
            return None

    def _capture_orbbec_frame(self) -> Optional[o3d.geometry.PointCloud]:
        """采集Orbbec单帧点云"""
        try:
            from pyorbbecsdk import OBFormat

            # 等待帧（位置参数，单位：毫秒）
            frames = self.pipeline.wait_for_frames(1000)

            if frames is None:
                print(f"  警告: wait_for_frames返回None")
                return None

            # 获取深度帧
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                print(f"  警告: 深度帧为None")
                return None

            # 获取彩色帧（如果可用）
            color_frame = frames.get_color_frame() if self.has_color_sensor else None

            # 使用PointCloudFilter生成点云
            # 根据是否有彩色帧选择点云格式
            point_format = OBFormat.RGB_POINT if color_frame is not None else OBFormat.POINT
            self.point_cloud_filter.set_create_point_format(point_format)

            # 处理帧生成点云
            point_cloud_frame = self.point_cloud_filter.process(frames)

            # 计算点云数据
            points = self.point_cloud_filter.calculate(point_cloud_frame)

            if len(points) == 0:
                print(f"  警告: 生成的点云为空")
                return None

            # 提取点坐标（将毫米转换为米）
            points_array = np.asarray(points)

            # 根据点云格式提取数据
            if point_format == OBFormat.RGB_POINT:
                # RGB_POINT格式: [x, y, z, r, g, b]
                positions = points_array[:, :3] * 0.001  # mm to m
                colors = points_array[:, 3:6] / 255.0     # 归一化到[0,1]
            else:
                # POINT格式: [x, y, z]
                positions = points_array.reshape(-1, 3) * 0.001  # mm to m
                colors = None

            # 过滤无效点：移除距离原点过近的点（< 0.1m，即100mm）
            # 这些通常是无效的深度数据
            distances = np.linalg.norm(positions, axis=1)
            valid_mask = distances > 0.1  # 大于10cm

            if np.sum(valid_mask) == 0:
                print(f"  警告: 过滤后没有有效点")
                return None

            positions = positions[valid_mask]
            if colors is not None:
                colors = colors[valid_mask]

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

        # 先清空缓冲区，丢弃旧帧
        print("  清空帧缓冲区...")
        for _ in range(5):
            try:
                self.pipeline.wait_for_frames(100)
            except:
                pass

        for i in range(num_frames):
            pcd = self.capture_single_frame()

            if pcd is not None and len(pcd.points) > 0:
                point_clouds.append(pcd)
                success_count += 1
                print(f"  采集第 {success_count}/{num_frames} 帧 (点数: {len(pcd.points):,})")
            else:
                print(f"  警告: 第 {i+1} 帧采集失败，跳过")

            # 增加延迟，确保相机有足够时间准备下一帧
            time.sleep(0.1)  # 100ms延迟

        print(f"成功采集 {success_count}/{num_frames} 帧")

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
        voxel_size = 0.005  # 5mm
        fused_pcd = temp_pcd.voxel_down_sample(voxel_size)

        print(f"  原始点数: {len(merged_points):,}")
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
        """停止相机"""
        if self.camera_type == 'realsense' and self.pipeline is not None:
            self.pipeline.stop()
            print("RealSense相机已停止")
        elif self.camera_type == 'kinect' and self.camera is not None:
            self.camera.stop()
            print("Kinect相机已停止")
        elif self.camera_type == 'orbbec' and self.pipeline is not None:
            self.pipeline.stop()
            print("Orbbec相机已停止")

    def __del__(self):
        """析构函数"""
        self.stop()
