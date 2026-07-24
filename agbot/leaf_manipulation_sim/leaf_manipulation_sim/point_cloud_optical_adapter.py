#!/usr/bin/env python3

"""Convert Gazebo's X-forward RGB-D cloud into a ROS optical-frame cloud."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudOpticalAdapter(Node):
    def __init__(self):
        super().__init__('point_cloud_optical_adapter')
        self.declare_parameter(
            'input_topic', '/camera/depth/color/points_gz')
        self.declare_parameter(
            'output_topic', '/camera/depth/color/points')
        self.declare_parameter(
            'optical_frame', 'camera_depth_optical_frame')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Converting Gazebo X-forward cloud {input_topic} to '
            f'ROS optical cloud {output_topic} '
            f'({self.optical_frame})')

    def callback(self, source):
        offsets = {field.name: field.offset for field in source.fields}
        if not all(axis in offsets for axis in ('x', 'y', 'z')):
            self.get_logger().error(
                'Point cloud does not contain x, y and z fields')
            return

        target = PointCloud2()
        target.header = source.header
        target.header.frame_id = self.optical_frame
        target.height = source.height
        target.width = source.width
        target.fields = source.fields
        target.is_bigendian = source.is_bigendian
        target.point_step = source.point_step
        target.row_step = source.row_step
        target.is_dense = source.is_dense
        target.data = bytearray(source.data)

        count = source.height * source.width
        float_dtype = '>f4' if source.is_bigendian else '<f4'

        def field_view(offset):
            return np.ndarray(
                shape=(count,),
                dtype=float_dtype,
                buffer=target.data,
                offset=offset,
                strides=(source.point_step,),
            )

        gazebo_x = field_view(offsets['x']).copy()
        gazebo_y = field_view(offsets['y']).copy()
        gazebo_z = field_view(offsets['z']).copy()

        # The existing sensor pose rotates the rendered image by 180 degrees
        # to match the physical D435 model.  In this configuration the exact
        # Gazebo-camera -> URDF-optical mapping is:
        #   optical_x = gazebo_y
        #   optical_y = gazebo_z
        #   optical_z = gazebo_x
        field_view(offsets['x'])[:] = gazebo_y
        field_view(offsets['y'])[:] = gazebo_z
        field_view(offsets['z'])[:] = gazebo_x

        self.publisher.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudOpticalAdapter()
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
