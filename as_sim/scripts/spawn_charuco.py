#!/usr/bin/env python3
"""Spawn only a fixed ChArUco board, preserving the existing table."""
from pathlib import Path
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node('spawn_charuco')
    defaults = {'world_name': 'scanner_env', 'table_surface_z': 0.02,
                'table_center_x': 0.65, 'table_center_y': 0.0, 'board_yaw': 0.0}
    for key, value in defaults.items():
        node.declare_parameter(key, value)
    arg = lambda key: node.get_parameter(key).value
    try:
        x, y, z = arg('table_center_x'), arg('table_center_y'), arg('table_surface_z')
        texture = (Path(get_package_share_directory('as_sim')) /
                   'worlds/charuco/materials/textures/calibrx.png').as_uri()
        # 5 mm white border around the 175 x 245 mm active pattern.
        sdf = f'''<sdf version="1.9"><model name="charuco_calibration_board">
          <static>true</static>
          <link name="board">
            <visual name="charuco"><pose>0 0 0 0 0 0</pose>
              <geometry><plane><normal>0 0 1</normal><size>0.185 0.255</size></plane></geometry>
              <cast_shadows>false</cast_shadows><material>
                <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
                <pbr><metal><albedo_map>{texture}</albedo_map><metalness>0</metalness>
                <roughness>1</roughness></metal></pbr>
              </material>
            </visual>
          </link></model></sdf>'''
        with tempfile.TemporaryDirectory(prefix='charuco_') as directory:
            path = Path(directory) / 'model.sdf'
            path.write_text(sdf)
            node.get_logger().info(f'Spawning fixed board in object area ({x:.3f}, {y:.3f}, {z:.3f})')
            subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', arg('world_name'),
                            '-name', 'charuco_calibration_board', '-file', str(path),
                            '-x', str(x), '-y', str(y), '-z', str(z),
                            '-Y', str(arg('board_yaw'))],
                           check=True, timeout=30)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
