#!/usr/bin/env python3
"""Own the temporary calibration board in Gazebo, independently of Robot."""
import subprocess

from ament_index_python.packages import get_package_prefix
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class HandeyeTarget(Node):
    def __init__(self):
        super().__init__("handeye_target_server")
        self.declare_parameter("world_name", "scanner_env")
        self.declare_parameter("dataset_model_file", "")
        for axis in ("x", "y", "z", "R", "P", "Y"):
            self.declare_parameter("dataset_" + axis, 0.0)
        self.visible = False
        self.dataset_removed = False
        self.show_service = self.create_service(Trigger, "/armx/simulation/handeye_target/show", self.show)
        self.hide_service = self.create_service(Trigger, "/armx/simulation/handeye_target/hide", self.hide)

    def show(self, _request, response):
        try:
            if not self.visible:
                if self.get_parameter("dataset_model_file").value and not self.dataset_removed:
                    self.remove("dataset_object")
                    self.dataset_removed = True
                subprocess.run(["ros2", "run", "as_sim", "spawn_charuco.py", "--ros-args",
                                "-p", f"world_name:={self.get_parameter('world_name').value}"],
                               check=True, timeout=35)
                self.visible = True
            response.success = True
            response.message = "Gazebo calibration board ready"
        except Exception as error:
            response.message = str(error)
        return response

    def hide(self, _request, response):
        try:
            if self.visible:
                self.remove("charuco_calibration_board")
                self.visible = False
            if self.dataset_removed:
                command = [str(Path(get_package_prefix("ros_gz_sim")) / "lib/ros_gz_sim/create"),
                           "-world", self.get_parameter("world_name").value,
                           "-name", "dataset_object", "-file", self.get_parameter("dataset_model_file").value]
                for axis in ("x", "y", "z", "R", "P", "Y"):
                    command.extend(["-" + axis, str(self.get_parameter("dataset_" + axis).value)])
                subprocess.run(command, check=True, timeout=30)
                self.dataset_removed = False
            response.success = True
            response.message = "Gazebo calibration board removed"
        except Exception as error:
            response.message = str(error)
        return response

    def remove(self, entity):
        subprocess.run([str(Path(get_package_prefix("ros_gz_sim")) / "lib/ros_gz_sim/remove"), "--ros-args",
                        "-p", f"world:={self.get_parameter('world_name').value}",
                        "-p", f"entity_name:={entity}"], check=True, timeout=30)


def main():
    rclpy.init()
    node = HandeyeTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
