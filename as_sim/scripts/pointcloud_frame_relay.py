#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudFrameRelay(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_frame_relay")

        self.declare_parameter("input_topic", "/camera/depth/color/points_internal")
        self.declare_parameter("output_topic", "/camera/depth/color/points")
        self.declare_parameter("target_frame_id", "camera_depth_optical_frame")

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
