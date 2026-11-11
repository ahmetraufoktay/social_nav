#!/usr/bin/env python3

import os

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory


class YoloNode(Node):
    def __init__(self):
        super().__init__('human_bounding_box_node')

        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('infer_interval', 0.1)
        self.declare_parameter('conf_threshold', 0.4)
        self.publish_debug = self.get_parameter('publish_debug_image').value
        self.infer_interval = self.get_parameter('infer_interval').value
        self.conf_threshold = self.get_parameter('conf_threshold').value

        package_share = get_package_share_directory('social_nav')
        model_path = os.path.join(package_share, 'weights', 'yolov8n.pt')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'YOLO Cihazı: {self.device}')
        self.model = YOLO(model_path).to(self.device)

        self.bridge = CvBridge()

        self.last_run_time = self.get_clock().now()

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_image = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            sub_qos
        )

        self.pub_detections = self.create_publisher(
            Detection2DArray,
            '/human_detections',
            1
        )
        self.pub_debug_image = self.create_publisher(Image, '/human_detections/image', 1)

    def image_callback(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_run_time).nanoseconds / 1e9 < self.infer_interval:
            return
        self.last_run_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge hatası: {e}')
            return

        result = self.model(
            cv_image, 
            classes=[0], 
            conf=self.conf_threshold,
            imgsz=320, 
            device=self.device, 
            verbose=False
        )[0]

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        for box in result.boxes:
            cx, cy, w, h = box.xywh[0].tolist()

            det = Detection2D()
            det.header = msg.header

            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = "person"
            hypo.hypothesis.score = float(box.conf[0])
            det.results.append(hypo)

            detections_msg.detections.append(det)

        self.pub_detections.publish(detections_msg)

        # --- Debug görüntüsü: cv_bridge KULLANMADAN, hata olsa da düğüm ölmez ---
        if self.publish_debug and self.pub_debug_image.get_subscription_count() > 0:
            try:
                annotated = np.ascontiguousarray(result.plot(), dtype=np.uint8)
                img = Image()
                img.header = msg.header
                img.height, img.width = annotated.shape[:2]
                img.encoding = 'bgr8'
                img.is_bigendian = 0
                img.step = img.width * 3
                img.data = annotated.tobytes()
                self.pub_debug_image.publish(img)
            except Exception as e:
                self.get_logger().warn(f'Debug görüntüsü yayınlanamadı: {e}',
                                       throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
