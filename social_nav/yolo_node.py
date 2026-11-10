#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class YoloNode(Node):
    def __init__(self):
        super().__init__('human_bounding_box_node')

        package_share = get_package_share_directory('social_nav')
        model_path = os.path.join(package_share, 'weights', 'yolov8n.pt')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'YOLO Cihazı: {self.device}')
        self.model = YOLO(model_path).to(self.device)

        self.bridge = CvBridge()

        self.last_run_time = self.get_clock().now()
        self.infer_interval = 0.1

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

    def image_callback(self, msg: Image):
        # 10 Hz Throttle
        now = self.get_clock().now()
        if (now - self.last_run_time).nanoseconds / 1e9 < self.infer_interval:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge hatası: {e}')
            return

        results = self.model(
            cv_image, 
            classes=[0], 
            imgsz=320, 
            device=self.device, 
            verbose=False
        )

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header 

        for box in results[0].boxes:
            conf = float(box.conf[0])
            cx, cy, w, h = box.xywh[0].tolist()

            det = Detection2D()
            det.header = msg.header

            det.bbox.center.position.x = float(cx)
            det.bbox.center.position.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = "person"
            hypo.hypothesis.score = conf
            det.results.append(hypo)

            detections_msg.detections.append(det)

        self.pub_detections.publish(detections_msg)


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
