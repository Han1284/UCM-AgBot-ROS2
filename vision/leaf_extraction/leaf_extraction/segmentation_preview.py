import json
import os

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from ultralytics import YOLO


def default_model_path():
    return os.path.join(
        get_package_share_directory('leaf_extraction'),
        'segmentation_model',
        'citrus.pt',
    )


class SegmentationPreview(Node):
    """Run leaf instance segmentation on an organized RGB point cloud."""

    def __init__(self):
        super().__init__('leaf_segmentation_preview')

        self.declare_parameter(
            'point_cloud_topic',
            '/perception_test_camera/depth/color/points',
        )
        self.declare_parameter('model_path', default_model_path())
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('device', '0')
        self.declare_parameter('image_size', 640)

        self.point_cloud_topic = (
            self.get_parameter('point_cloud_topic')
            .get_parameter_value()
            .string_value
        )
        self.model_path = (
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        self.confidence = (
            self.get_parameter('confidence').get_parameter_value().double_value
        )
        self.device = (
            self.get_parameter('device').get_parameter_value().string_value
        )
        self.image_size = (
            self.get_parameter('image_size')
            .get_parameter_value()
            .integer_value
        )

        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f'Leaf segmentation model does not exist: {self.model_path}'
            )

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.overlay_publisher = self.create_publisher(
            Image, '/leaf_segmentation/overlay', 1
        )
        self.mask_publisher = self.create_publisher(
            Image, '/leaf_segmentation/combined_mask', 1
        )
        self.status_publisher = self.create_publisher(
            String, '/leaf_segmentation/status', 10
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self.point_cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'Leaf segmentation ready: '
            f'model={self.model_path}, topic={self.point_cloud_topic}, '
            f'confidence={self.confidence:.3f}, device={self.device}'
        )

    @staticmethod
    def point_cloud_to_bgr(msg):
        if msg.height <= 1 or msg.width <= 1:
            raise ValueError(
                'Point cloud must be organized (height and width > 1)'
            )

        rgb_field = next(
            (field for field in msg.fields if field.name == 'rgb'), None
        )
        if rgb_field is None:
            raise ValueError('Point cloud has no packed rgb field')
        if msg.is_bigendian:
            raise ValueError('Big-endian PointCloud2 is not supported')

        point_count = msg.height * msg.width
        expected_size = point_count * msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < expected_size:
            raise ValueError(
                f'Point cloud data is truncated: {raw.size} < {expected_size}'
            )

        points = raw[:expected_size].reshape(point_count, msg.point_step)
        rgb_bytes = np.ascontiguousarray(
            points[:, rgb_field.offset:rgb_field.offset + 4]
        )
        packed = rgb_bytes.view('<u4').reshape(-1)

        rgb = np.empty((point_count, 3), dtype=np.uint8)
        rgb[:, 0] = (packed >> 16) & 0xFF
        rgb[:, 1] = (packed >> 8) & 0xFF
        rgb[:, 2] = packed & 0xFF
        rgb = rgb.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def point_cloud_callback(self, msg):
        try:
            bgr = self.point_cloud_to_bgr(msg)
            result = self.model.predict(
                bgr,
                imgsz=self.image_size,
                conf=self.confidence,
                device=self.device,
                verbose=False,
            )[0]

            overlay = result.plot()
            combined_mask = np.zeros(
                (msg.height, msg.width), dtype=np.uint8
            )
            if result.masks is not None:
                masks = result.masks.data.detach().cpu().numpy()
                for mask in masks:
                    resized = cv2.resize(
                        mask,
                        (msg.width, msg.height),
                        interpolation=cv2.INTER_NEAREST,
                    )
                    combined_mask[resized > 0.5] = 255

            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_publisher.publish(overlay_msg)

            mask_msg = self.bridge.cv2_to_imgmsg(
                combined_mask, encoding='mono8'
            )
            mask_msg.header = msg.header
            self.mask_publisher.publish(mask_msg)

            confidences = []
            if result.boxes is not None:
                confidences = [
                    round(float(value), 6)
                    for value in result.boxes.conf.detach().cpu()
                ]
            status = {
                'frame_id': msg.header.frame_id,
                'detections': len(confidences),
                'confidences': confidences,
                'inference_ms': round(float(result.speed['inference']), 3),
            }
            self.status_publisher.publish(
                String(data=json.dumps(status, separators=(',', ':')))
            )
        except Exception as exc:
            self.get_logger().error(
                f'Leaf segmentation frame failed: {exc}',
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
