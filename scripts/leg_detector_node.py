#!/usr/bin/env python3
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException

from dr_spaam_model import Detector


class LegDetectorNode(Node):
    """2D LiDAR (DR-SPAAM / DROW3) tabanlı ve Statik Harita Ön-Filtreli bacak tespit düğümü."""

    def __init__(self):
        super().__init__("leg_detector_node")

        self.declare_parameter("weight_file", "ckpt_jrdb_ann_ft_dr_spaam_e20.pth")
        self.declare_parameter("detector_model", "DR-SPAAM")
        self.declare_parameter("use_gpu", False)
        self.declare_parameter("conf_thresh", 0.35)
        self.declare_parameter("stride", 1)
        self.declare_parameter("panoramic_scan", True)
        self.declare_parameter("scan_topic_name", "/scan")
        self.declare_parameter("filter_static_map", True)
        self.declare_parameter("map_obstacle_margin", 0.22)  # metre (sabit engellere yakınlık sınırı)

        pkg_share = get_package_share_directory("social_nav")
        weight_name = self.get_parameter("weight_file").value
        self.weight_file = os.path.join(pkg_share, "weights", weight_name)
        self.detector_model = self.get_parameter("detector_model").value
        self.use_gpu = self.get_parameter("use_gpu").value
        self.conf_thresh = self.get_parameter("conf_thresh").value
        self.stride = self.get_parameter("stride").value
        self.panoramic_scan = self.get_parameter("panoramic_scan").value
        self.scan_topic = self.get_parameter("scan_topic_name").value
        self.filter_static_map = self.get_parameter("filter_static_map").value
        self.map_obstacle_margin = self.get_parameter("map_obstacle_margin").value

        # Statik harita verisi ve TF2 tamponu
        self._map_grid = None
        self._map_info = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        if not os.path.isfile(self.weight_file):
            self.get_logger().error(
                f"Model ağırlık dosyası bulunamadı: {self.weight_file}"
            )
            self._detector = None
            return

        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=self.use_gpu,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )
        self.get_logger().info(
            f"DR-SPAAM modeli yüklendi: {self.weight_file} (filter_static_map={self.filter_static_map})"
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        scan_qos = QoSProfile(
            reliability=qos_profile_sensor_data.reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=qos_profile_sensor_data.durability,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._dets_pub = self.create_publisher(
            PoseArray, "/dr_spaam_detections", pub_qos
        )
        self._rviz_pub = self.create_publisher(
            Marker, "/dr_spaam_rviz", pub_qos
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )
        self._scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, scan_qos
        )

    def _map_callback(self, msg: OccupancyGrid):
        self._map_info = msg.info
        self._map_grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.get_logger().info(
            f"Statik harita (/map) alındı: {msg.info.width}x{msg.info.height}, "
            f"çözünürlük={msg.info.resolution:.3f}m"
        )

    def _get_lidar_to_map_matrix(self, lidar_frame: str):
        """LiDAR koordinat sisteminden /map koordinat sistemine 2B dönüşüm parametrelerini döndürür."""
        if self._map_grid is None or self._map_info is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", lidar_frame, rclpy.time.Time()
            )
        except TransformException:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        return tx, ty, r00, r01, r10, r11

    def _is_point_on_static_map(self, x_map: float, y_map: float, margin_cells: int) -> bool:
        """Verilen (x_map, y_map) noktasının haritada sabit bir engele veya bilinmeyen alana denk gelip gelmediğini kontrol eder."""
        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        h, w = self._map_grid.shape

        mx = int((x_map - ox) / res)
        my = int((y_map - oy) / res)

        if mx < 0 or mx >= w or my < 0 or my >= h:
            return True

        x_min = max(0, mx - margin_cells)
        x_max = min(w, mx + margin_cells + 1)
        y_min = max(0, my - margin_cells)
        y_max = min(h, my + margin_cells + 1)

        patch = self._map_grid[y_min:y_max, x_min:x_max]
        return bool(np.any(patch >= 50) or self._map_grid[my, mx] < 0)

    def _mask_static_scan_rays(self, scan: np.ndarray, angles: np.ndarray, lidar_frame: str):
        """
        ÖN-FİLTRELEME (Pre-Filtering):
        Ham LiDAR ışınlarını DR-SPAAM modeline vermeden önce /map ile karşılaştırır.
        Duvar, kutu veya kolon gibi haritada kayıtlı siyah piksellere çarpan ışınları
        29.99m (boş uzay) yaparak modelin sabit engelleri görmesini engeller.
        """
        tf_params = self._get_lidar_to_map_matrix(lidar_frame)
        if tf_params is None:
            valid = (scan > 0.05) & (scan < 11.9)
            fg_x = scan[valid] * np.cos(angles[valid])
            fg_y = scan[valid] * np.sin(angles[valid])
            return scan, np.stack((fg_x, fg_y), axis=1)

        tx, ty, r00, r01, r10, r11 = tf_params
        res = self._map_info.resolution
        margin_cells = max(1, int(round(self.map_obstacle_margin / res)))

        masked_scan = scan.copy()
        valid_indices = np.where((masked_scan > 0.05) & (masked_scan < 11.9))[0]

        if len(valid_indices) == 0:
            return masked_scan, np.empty((0, 2), dtype=np.float32)

        r_valid = masked_scan[valid_indices]
        ang_valid = angles[valid_indices]
        x_lidar = r_valid * np.cos(ang_valid)
        y_lidar = r_valid * np.sin(ang_valid)

        x_map = tx + r00 * x_lidar + r01 * y_lidar
        y_map = ty + r10 * x_lidar + r11 * y_lidar

        fg_keep = np.ones(len(valid_indices), dtype=bool)
        for idx in range(len(valid_indices)):
            if self._is_point_on_static_map(x_map[idx], y_map[idx], margin_cells):
                masked_scan[valid_indices[idx]] = 29.99
                fg_keep[idx] = False

        fg_xy = np.stack((x_lidar[fg_keep], y_lidar[fg_keep]), axis=1)
        return masked_scan, fg_xy

    def _filter_detections(
        self,
        dets_xy: np.ndarray,
        dets_cls: np.ndarray,
        fg_xy: np.ndarray,
        lidar_frame: str,
    ):
        """
        SON-FİLTRELEME (Post-Filtering):
        1. Tespit edilen bacak merkezinin yakınında (< 0.45m) haritadan silinmemiş en az 1 dinamik
           LiDAR noktası (fg_xy) bulunmasını şart koşar.
        2. Tespit merkezinin kendisi de haritada sabit bir engele denk geliyorsa eler.
        """
        if len(dets_xy) == 0:
            return dets_xy, dets_cls

        if len(fg_xy) == 0:
            return np.empty((0, 2), dtype=np.float32), np.empty((0,), dtype=np.float32)

        tf_params = self._get_lidar_to_map_matrix(lidar_frame)
        margin_cells = 1
        if tf_params is not None:
            tx, ty, r00, r01, r10, r11 = tf_params
            margin_cells = max(1, int(round(self.map_obstacle_margin / self._map_info.resolution)))

        keep_mask = np.ones(len(dets_xy), dtype=bool)
        for i in range(len(dets_xy)):
            dx, dy = dets_xy[i, 0], dets_xy[i, 1]

            # 1) Yakınında gerçekten dinamik (silinmemiş) LiDAR noktası var mı?
            dists_to_fg = np.hypot(fg_xy[:, 0] - dx, fg_xy[:, 1] - dy)
            if np.min(dists_to_fg) > 0.45:
                keep_mask[i] = False
                continue

            # 2) Merkez noktası haritada sabit engel üzerinde mi?
            if tf_params is not None:
                x_map = tx + r00 * dx + r01 * dy
                y_map = ty + r10 * dx + r11 * dy
                if self._is_point_on_static_map(x_map, y_map, margin_cells):
                    keep_mask[i] = False

        return dets_xy[keep_mask], dets_cls[keep_mask]

    def _scan_callback(self, msg: LaserScan):
        if self._detector is None:
            return

        scan = np.array(msg.ranges, dtype=np.float32)
        num_pts = len(scan)
        if num_pts == 0:
            return

        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        # LiDAR ışınlarının gerçek açılarını hesapla ve [-pi, pi) aralığına sırala
        # (TurtleBot3 [0, 2*pi] gönderirken DR-SPAAM [-pi, pi] beklediği için 180 derece kaymayı önler)
        raw_angles = msg.angle_min + np.arange(num_pts, dtype=np.float32) * msg.angle_increment
        wrapped_angles = (raw_angles + np.pi) % (2.0 * np.pi) - np.pi
        sort_order = np.argsort(wrapped_angles)

        scan_sorted = scan[sort_order]
        angles_sorted = wrapped_angles[sort_order]

        if not self._detector.is_ready():
            fov_deg = float(np.rad2deg(angles_sorted[-1] - angles_sorted[0]))
            self._detector.set_laser_fov(fov_deg)

        # Haritada sabit engel olan LiDAR ışınlarını modele vermeden önce maskele
        if self.filter_static_map:
            scan_sorted, fg_xy = self._mask_static_scan_rays(
                scan_sorted, angles_sorted, msg.header.frame_id
            )
        else:
            valid = (scan_sorted > 0.05) & (scan_sorted < 11.9)
            fg_xy = np.stack(
                (
                    scan_sorted[valid] * np.cos(angles_sorted[valid]),
                    scan_sorted[valid] * np.sin(angles_sorted[valid]),
                ),
                axis=1,
            )

        dets_xy, dets_cls, _ = self._detector(scan_sorted)

        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        if len(dets_xy) > 0:
            dist_mask = np.hypot(dets_xy[:, 0], dets_xy[:, 1]) < 11.5
            conf_mask = np.logical_and(conf_mask, dist_mask)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        if self.filter_static_map:
            dets_xy, dets_cls = self._filter_detections(
                dets_xy, dets_cls, fg_xy, msg.header.frame_id
            )

        dets_msg = detections_to_pose_array(dets_xy)
        dets_msg.header = msg.header
        self._dets_pub.publish(dets_msg)

        rviz_msg = detections_to_rviz_marker(dets_xy)
        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)


def detections_to_rviz_marker(dets_xy):
    msg = Marker()
    msg.action = Marker.ADD
    msg.ns = "dr_spaam_ros"
    msg.id = 0
    msg.type = Marker.LINE_LIST

    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.04  # çizgi kalınlığı
    msg.color.r = 0.0
    msg.color.g = 1.0
    msg.color.b = 0.2
    msg.color.a = 1.0

    r = 0.35
    ang = np.linspace(0, 2 * np.pi, 20)
    xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

    points = []
    for d_xy in dets_xy:
        for i in range(len(xy_offsets) - 1):
            p0 = Point()
            p0.x = float(d_xy[0] + xy_offsets[i, 0])
            p0.y = float(d_xy[1] + xy_offsets[i, 1])
            p0.z = 0.0
            points.append(p0)

            p1 = Point()
            p1.x = float(d_xy[0] + xy_offsets[i + 1, 0])
            p1.y = float(d_xy[1] + xy_offsets[i + 1, 1])
            p1.z = 0.0
            points.append(p1)
    msg.points = points
    return msg


def detections_to_pose_array(dets_xy):
    pose_array = PoseArray()
    for d_xy in dets_xy:
        p = Pose()
        p.position.x = float(d_xy[0])
        p.position.y = float(d_xy[1])
        p.position.z = 0.0
        p.orientation.w = 1.0
        pose_array.poses.append(p)
    return pose_array


def main(args=None):
    rclpy.init(args=args)
    node = LegDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
