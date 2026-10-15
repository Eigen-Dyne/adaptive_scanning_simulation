#!/usr/bin/env python3

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def gazebo_to_optical(msg):
    """Rotate Gazebo FLU coordinates into optical RDF; preserve other fields."""
    result = copy.deepcopy(msg)
    data = bytearray(msg.data)
    fields = {field.name: field for field in msg.fields}
    coordinates = []
    for name in ("x", "y", "z"):
        field = fields[name]
        types = {PointField.FLOAT32: "f4", PointField.FLOAT64: "f8"}
        if field.datatype not in types or field.count != 1:
            raise ValueError("cloud XYZ fields must be scalar float32 or float64")
        dtype = (">" if msg.is_bigendian else "<") + types[field.datatype]
        coordinates.append(np.ndarray(
            (msg.height, msg.width), dtype=dtype, buffer=data, offset=field.offset,
            strides=(msg.row_step, msg.point_step)))
    x, y, z = [coordinate.copy() for coordinate in coordinates]
    coordinates[0][:] = -y
    coordinates[1][:] = -z
    coordinates[2][:] = x
    result.data = bytes(data)
    return result


class PointCloudFrameRelay(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_frame_relay")

        self.declare_parameter("input_topic", "/camera/depth/color/points_internal")
        self.declare_parameter("output_topic", "/camera/depth/color/points")
        self.declare_parameter("target_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("convert_gazebo_axes", False)
        self.convert_gazebo_axes = bool(self.get_parameter("convert_gazebo_axes").value)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, 10
        )

        self.get_logger().info(
            f"Relaying '{input_topic}' -> '{output_topic}' with frame_id='{self.target_frame_id}'"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self.convert_gazebo_axes:
            try:
                msg = gazebo_to_optical(msg)
            except (KeyError, ValueError) as exc:
                self.get_logger().error(f"Cannot convert Gazebo cloud: {exc}")
                return
        msg.header.frame_id = self.target_frame_id
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = PointCloudFrameRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
