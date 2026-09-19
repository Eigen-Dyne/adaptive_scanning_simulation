#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("name", default_value="dsr01", description="NAME_SPACE"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="ROBOT_IP"),
        DeclareLaunchArgument("port", default_value="12345", description="ROBOT_PORT"),
        DeclareLaunchArgument("mode", default_value="gazebo", description="OPERATION MODE"),
        DeclareLaunchArgument("model", default_value="a0912_scanner", description="ROBOT_MODEL"),
        DeclareLaunchArgument("color", default_value="blue", description="ROBOT_COLOR"),
        DeclareLaunchArgument("gui", default_value="false", description="Start Gazebo GUI"),
        DeclareLaunchArgument("gz", default_value="true", description="USE GAZEBO SIM"),
        DeclareLaunchArgument("x", default_value="0", description="Location x on Gazebo"),
        DeclareLaunchArgument("y", default_value="0", description="Location y on Gazebo"),
        DeclareLaunchArgument("z", default_value="0.12", description="Location z on Gazebo"),
        DeclareLaunchArgument("R", default_value="0", description="Location Roll on Gazebo"),
        DeclareLaunchArgument("P", default_value="0", description="Location Pitch on Gazebo"),
        DeclareLaunchArgument("Y", default_value="0", description="Location Yaw on Gazebo"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.50", description="ROBOT_RT_IP"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument("remap_tf", default_value="true", description="REMAP TF"),
        DeclareLaunchArgument(
            "part",
            default_value="",
            description="Dataset part number. Resolves to <part>.stl for Gazebo object spawning.",
        ),
        DeclareLaunchArgument(
            "object",
            default_value="",
            description="Optional STL filename override from Dataset folder. Example: 16.stl",
        ),
        DeclareLaunchArgument("obj_x", default_value="0.65", description="Dataset object X in Gazebo/base frame"),
        DeclareLaunchArgument("obj_y", default_value="0.0", description="Dataset object Y in Gazebo/base frame"),
        DeclareLaunchArgument("obj_z", default_value="0.0", description="Dataset object Z before floor placement"),
        DeclareLaunchArgument("obj_R", default_value="0.0", description="Dataset object roll"),
        DeclareLaunchArgument("obj_P", default_value="0.0", description="Dataset object pitch"),
        DeclareLaunchArgument("obj_Y", default_value="0.0", description="Dataset object yaw"),
        DeclareLaunchArgument("obj_scale", default_value="0.001", description="Dataset object mesh scale"),
        DeclareLaunchArgument("obj_place_on_floor", default_value="true", description="Place object bottom on floor"),
        DeclareLaunchArgument("obj_floor_clearance_m", default_value="0.02", description="Dataset object floor clearance"),
        DeclareLaunchArgument("floor_z", default_value="0.0", description="Gazebo floor Z"),
    ]

    def _resolve_gazebo_object(context, *args, **kwargs):
        del args, kwargs
        object_value = str(LaunchConfiguration("object").perform(context)).strip()
        part_value = str(LaunchConfiguration("part").perform(context)).strip()

        resolved = object_value or part_value
        if resolved and not resolved.lower().endswith(".stl"):
            resolved = f"{resolved}.stl"

        print(f"[INFO] Gazebo dataset object resolved: part={part_value} object={resolved or '<none>'}")
        return [SetLaunchConfiguration("object", resolved)]


    set_use_sim_time = SetLaunchConfiguration(name="use_sim_time", value="true")
    use_sim_time = LaunchConfiguration("use_sim_time")
    update_rate = "30"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("as_edge_robot_doosan_a0912"),
                    "doosan",
                    "dsr_description",
                    "xacro",
                    LaunchConfiguration("model"),
                ]
            ),
            ".urdf.xacro",
            " ",
            "use_gazebo:=true",
            " ",
            "color:=",
            LaunchConfiguration("color"),
            " ",
            "namespace:=",
            LaunchConfiguration("name"),
            " host:=",
            LaunchConfiguration("host"),
            " rt_host:=",
            LaunchConfiguration("rt_host"),
            " port:=",
            LaunchConfiguration("port"),
            " mode:=",
            LaunchConfiguration("mode"),
            " model:=",
            LaunchConfiguration("model"),
            " update_rate:=",
            update_rate,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/depth/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        remappings=[
            ("/camera/depth/depth_image", "/camera/depth/image_rect_raw"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="log",
    )

    pointcloud_frame_relay_node = Node(
        package="as_sim",
        executable="pointcloud_frame_relay.py",
        name="pointcloud_frame_relay",
        parameters=[
            {"input_topic": "/camera/depth/points"},
            {"output_topic": "/camera/depth/color/points"},
            {"target_frame_id": "camera_depth_optical_frame"},
            {"use_sim_time": use_sim_time},
        ],
        output="log",
    )

    ensure_controllers_node = Node(
        package="as_edge_robot_doosan_a0912",
        executable="ensure_controllers.py",
        name="ensure_gazebo_controllers",
        namespace=LaunchConfiguration("name"),
        output="screen",
        parameters=[
            {
                "controllers": ["joint_state_broadcaster", "dsr_moveit_controller"],
                "timeout_sec": 30.0,
                "switch_timeout_sec": 10.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("as_sim"), "launch", "gazebo_spawn.launch.py")
        ),
        launch_arguments={
            "use_gazebo": LaunchConfiguration("gz"),
            "gui": LaunchConfiguration("gui"),
            "name": LaunchConfiguration("name"),
            "color": LaunchConfiguration("color"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "R": LaunchConfiguration("R"),
            "P": LaunchConfiguration("P"),
            "Y": LaunchConfiguration("Y"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "object": LaunchConfiguration("object"),
            "part": LaunchConfiguration("part"),
            "use_object_csv": "false",
            "obj_x": LaunchConfiguration("obj_x"),
            "obj_y": LaunchConfiguration("obj_y"),
            "obj_z": LaunchConfiguration("obj_z"),
            "obj_R": LaunchConfiguration("obj_R"),
            "obj_P": LaunchConfiguration("obj_P"),
            "obj_Y": LaunchConfiguration("obj_Y"),
            "obj_scale": LaunchConfiguration("obj_scale"),
            "obj_place_on_floor": LaunchConfiguration("obj_place_on_floor"),
            "obj_floor_clearance_m": LaunchConfiguration("obj_floor_clearance_m"),
            "floor_z": LaunchConfiguration("floor_z"),
        }.items(),
    )

    nodes = [
        set_use_sim_time,
        OpaqueFunction(function=_resolve_gazebo_object),
        gz_sim_bridge,
        included_launch,
        pointcloud_frame_relay_node,
        TimerAction(period=8.0, actions=[ensure_controllers_node]),
    ]

    return LaunchDescription(arguments + nodes)
