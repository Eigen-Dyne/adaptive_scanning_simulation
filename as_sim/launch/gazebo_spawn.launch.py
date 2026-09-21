#
#  dsr_bringup2
#  Author: Minsoo Song (minsoo.song@doosan.com)
#
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
#

import csv
import os
import struct
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

arguments = [
    DeclareLaunchArgument("name", default_value="dsr01", description="NAME_SPACE"),
    DeclareLaunchArgument(
        "model", default_value="a0912_scanner", description="ROBOT_MODEL"
    ),
    DeclareLaunchArgument("color", default_value="blue", description="ROBOT_COLOR"),
    DeclareLaunchArgument("gui", default_value="false", description="Start Gazebo GUI"),
    DeclareLaunchArgument(
        "use_gazebo", default_value="true", description="Start Gazebo"
    ),
    DeclareLaunchArgument("x", default_value="0", description="Location x on Gazebo "),
    DeclareLaunchArgument("y", default_value="0", description="Location y on Gazebo"),
    DeclareLaunchArgument("z", default_value="0.0", description="Location z on Gazebo"),
    DeclareLaunchArgument(
        "R", default_value="0", description="Location Roll on Gazebo"
    ),
    DeclareLaunchArgument(
        "P", default_value="0", description="Location Pitch on Gazebo"
    ),
    DeclareLaunchArgument("Y", default_value="0", description="Location Yaw on Gazebo"),
    DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    ),
    DeclareLaunchArgument("remap_tf", default_value="true", description="REMAP TF"),
    DeclareLaunchArgument(
        "object", default_value="", description="STL file from Dataset folder"
    ),
    DeclareLaunchArgument(
        "part",
        default_value="",
        description="Dataset part number alias. Resolves to <part>.stl when object is not set.",
    ),
    DeclareLaunchArgument(
        "use_object_csv",
        default_value="false",
        description="Load dataset object pose from config/objects.csv",
    ),
    # -------- Dataset object pose --------
    DeclareLaunchArgument("obj_x", default_value="0.65", description="Object X"),
    DeclareLaunchArgument("obj_y", default_value="0.0", description="Object Y"),
    DeclareLaunchArgument("obj_z", default_value="0.0", description="Object Z"),
    DeclareLaunchArgument("obj_R", default_value="0.0", description="Object Roll"),
    DeclareLaunchArgument("obj_P", default_value="0.0", description="Object Pitch"),
    DeclareLaunchArgument("obj_Y", default_value="0.0", description="Object Yaw"),
    DeclareLaunchArgument(
        "obj_scale",
        default_value="0.001",
        description="Object mesh scale (uniform)",
    ),
    DeclareLaunchArgument(
        "obj_place_on_floor",
        default_value="true",
        description="Automatically place the dataset object mesh bottom on the floor",
    ),
    DeclareLaunchArgument(
        "obj_floor_clearance_m",
        default_value="0.02",
        description="Extra height above the floor for dataset object placement",
    ),
    DeclareLaunchArgument("floor_z", default_value="0.0", description="Gazebo floor Z"),
]


OBJECT_POSE_OVERRIDES = {
    # Per-object pose override for manual Gazebo tuning.
    "16.stl": {
        "x": "0.65",
    },
}


def _read_stl_bounds(path: str):
    try:
        bounds = None

        def update(x, y, z):
            nonlocal bounds
            if bounds is None:
                bounds = [x, y, z, x, y, z]
                return
            bounds[0] = min(bounds[0], x)
            bounds[1] = min(bounds[1], y)
            bounds[2] = min(bounds[2], z)
            bounds[3] = max(bounds[3], x)
            bounds[4] = max(bounds[4], y)
            bounds[5] = max(bounds[5], z)

        with open(path, "rb") as handle:
            data = handle.read()
        if len(data) >= 84:
            triangle_count = struct.unpack_from("<I", data, 80)[0]
            expected_size = 84 + int(triangle_count) * 50
            if expected_size == len(data):
                offset = 84
                for _ in range(int(triangle_count)):
                    for vertex_idx in range(3):
                        x, y, z = struct.unpack_from(
                            "<fff", data, offset + 12 + vertex_idx * 12
                        )
                        update(x, y, z)
                    offset += 50
                return bounds

        with open(path, "r", encoding="utf-8", errors="ignore") as handle:
            for line in handle:
                parts = line.strip().split()
                if len(parts) == 4 and parts[0].lower() == "vertex":
                    update(float(parts[1]), float(parts[2]), float(parts[3]))
        return bounds
    except Exception as exc:
        print(f"[WARN] Failed to inspect STL bounds: {path}: {exc}")
        return None


def _read_stl_min_z(path: str) -> float | None:
    bounds = _read_stl_bounds(path)
    return None if bounds is None else bounds[2]


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def load_object_pose_from_csv(context):
    if not _as_bool(LaunchConfiguration("use_object_csv").perform(context)):
        print("[INFO] CSV object pose loading disabled; using explicit obj_* launch arguments.")
        return []

    obj_name = LaunchConfiguration("object").perform(context)

    csv_path = os.path.join(
        get_package_share_directory("as_sim"),
        "config",
        "objects.csv",
    )

    if not os.path.exists(csv_path):
        override = OBJECT_POSE_OVERRIDES.get(obj_name)
        if override is not None:
            print(f"[POSE OVERRIDE] {obj_name} → {override}")
            actions = []
            if "x" in override:
                actions.append(SetLaunchConfiguration("obj_x", override["x"]))
            if "y" in override:
                actions.append(SetLaunchConfiguration("obj_y", override["y"]))
            if "z" in override:
                actions.append(SetLaunchConfiguration("obj_z", override["z"]))
            if "roll" in override:
                actions.append(SetLaunchConfiguration("obj_R", override["roll"]))
            if "pitch" in override:
                actions.append(SetLaunchConfiguration("obj_P", override["pitch"]))
            if "yaw" in override:
                actions.append(SetLaunchConfiguration("obj_Y", override["yaw"]))
            if "scale" in override:
                actions.append(SetLaunchConfiguration("obj_scale", override["scale"]))
            return actions
        print(f"[ERROR] CSV not found: {csv_path}")
        return []

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for raw_row in reader:
            row = {
                str(key).strip(): ("" if value is None else str(value).strip())
                for key, value in raw_row.items()
                if key is not None
            }
            if row.get("name") == obj_name:
                scale = row.get("scale", "") or "0.001"
                print(
                    f"[CSV] {obj_name} → "
                    f"x={row.get('x', '')} y={row.get('y', '')} z={row.get('z', '')} "
                    f"R={row.get('roll', '')} P={row.get('pitch', '')} Y={row.get('yaw', '')} scale={scale}"
                )

                return [
                    SetLaunchConfiguration("obj_x", row.get("x", "")),
                    SetLaunchConfiguration("obj_y", row.get("y", "")),
                    SetLaunchConfiguration("obj_z", row.get("z", "")),
                    SetLaunchConfiguration("obj_R", row.get("roll", "")),
                    SetLaunchConfiguration("obj_P", row.get("pitch", "")),
                    SetLaunchConfiguration("obj_Y", row.get("yaw", "")),
                    SetLaunchConfiguration("obj_scale", scale),
                ]

    override = OBJECT_POSE_OVERRIDES.get(obj_name)
    if override is not None:
        print(f"[POSE OVERRIDE] {obj_name} → {override}")
        actions = []
        if "x" in override:
            actions.append(SetLaunchConfiguration("obj_x", override["x"]))
        if "y" in override:
            actions.append(SetLaunchConfiguration("obj_y", override["y"]))
        if "z" in override:
            actions.append(SetLaunchConfiguration("obj_z", override["z"]))
        if "roll" in override:
            actions.append(SetLaunchConfiguration("obj_R", override["roll"]))
        if "pitch" in override:
            actions.append(SetLaunchConfiguration("obj_P", override["pitch"]))
        if "yaw" in override:
            actions.append(SetLaunchConfiguration("obj_Y", override["yaw"]))
        if "scale" in override:
            actions.append(SetLaunchConfiguration("obj_scale", override["scale"]))
        return actions

    print(f"[WARN] No CSV entry for {obj_name}")
    return []


def resolve_object_from_part(context):
    obj = LaunchConfiguration("object").perform(context).strip()
    part = LaunchConfiguration("part").perform(context).strip()
    if obj:
        return []
    if not part:
        return []
    resolved = part if part.lower().endswith(".stl") else f"{part}.stl"
    print(f"[INFO] Dataset object resolved in spawn launch: part={part} object={resolved}")
    return [SetLaunchConfiguration("object", resolved)]


def generate_launch_description():
    pkg_share = get_package_share_directory("as_sim")
    pkg_share_parent = os.path.dirname(pkg_share)
    gz_sim_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    ign_gazebo_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gz_resource_path = os.environ.get("GZ_RESOURCE_PATH", "")

    def _append_resource_path(existing, extra):
        return existing + os.pathsep + extra if existing else extra

    set_gz_sim_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=_append_resource_path(gz_sim_resource_path, pkg_share_parent),
    )
    set_ign_gazebo_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=_append_resource_path(ign_gazebo_resource_path, pkg_share_parent),
    )
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_RESOURCE_PATH",
        value=_append_resource_path(gz_resource_path, pkg_share_parent),
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    gazebo_args_prefix = PythonExpression(
        [
            "'-r -v 1 ' if '",
            LaunchConfiguration("gui"),
            "'.lower() in ('true', '1', 'yes', 'on') "
            "else '-s --headless-rendering -r -v 1 '",
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [
                gazebo_args_prefix,
                PathJoinSubstitution([
                    FindPackageShare("as_sim"),
                    "worlds",
                    "scanner_world.world",
                ])
            ]
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace=LaunchConfiguration('name'),
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            LaunchConfiguration("model"),
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-R",
            LaunchConfiguration("R"),
            "-P",
            LaunchConfiguration("P"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

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
            "use_gazebo:=",
            LaunchConfiguration("use_gazebo"),
            " ",
            "color:=",
            LaunchConfiguration("color"),
            " ",
            "namespace:=",
            LaunchConfiguration("name"),
            # dsr_description2's Gazebo ros2_control macro requires this
            # xacro argument.  Without it launch aborts before the delayed
            # dataset-object spawn action runs.
            " update_rate:=30",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration('name'),
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )

    # ---------- DATASET OBJECT PIPELINE ----------

    resolve_object = OpaqueFunction(function=resolve_object_from_part)
    load_object_pose = OpaqueFunction(function=load_object_pose_from_csv)

    remove_dataset_object = Node(
        package="ros_gz_sim",
        executable="remove",
        output="screen",
        parameters=[
            {"world": "scanner_env"},
            {"entity_name": "dataset_object"},
            {"use_sim_time": use_sim_time},
        ],
    )

    def spawn_dataset_object_factory(context):
        obj = LaunchConfiguration("object").perform(context).strip()
        scale = LaunchConfiguration("obj_scale").perform(context).strip() or "0.001"
        dataset_dir = os.path.join(
            get_package_share_directory("as_sim"),
            "worlds",
            "Dataset",
        )
        stl_path = obj if os.path.isabs(obj) else os.path.join(dataset_dir, obj)

        if not obj:
            print("[WARN] No dataset object specified; skipping dataset spawn.")
            return []
        if not os.path.exists(stl_path):
            raise RuntimeError(f"Dataset STL missing: {stl_path}")
        stl_uri = f"file://{stl_path}"
        spawn_z = LaunchConfiguration("obj_z").perform(context)
        if _as_bool(LaunchConfiguration("obj_place_on_floor").perform(context)):
            try:
                scale_value = float(scale)
                floor_z = float(LaunchConfiguration("floor_z").perform(context))
                clearance_m = float(
                    LaunchConfiguration("obj_floor_clearance_m").perform(context)
                )
                min_z = _read_stl_min_z(stl_path)
                if min_z is not None:
                    spawn_z = f"{floor_z + clearance_m - (float(min_z) * scale_value):.6f}"
                    print(
                        "[INFO] Floor-placing dataset object: "
                        f"mesh_min_z={float(min_z):.6f} scale={scale_value:.6f} "
                        f"floor_z={floor_z:.6f} clearance_m={clearance_m:.6f} "
                        f"spawn_z={spawn_z}"
                    )
            except Exception as exc:
                print(
                    "[WARN] Failed to compute floor placement; using configured "
                    f"obj_z={spawn_z}: {exc}"
                )
        bounds = _read_stl_bounds(stl_path)
        if bounds is not None:
            try:
                scale_value = float(scale)
                min_x, min_y, min_z, max_x, max_y, max_z = bounds
                size_x = max((max_x - min_x) * scale_value, 0.0)
                size_y = max((max_y - min_y) * scale_value, 0.0)
                size_z = max((max_z - min_z) * scale_value, 0.0)
                print(
                    "[INFO] Dataset STL bounds: "
                    f"min=({min_x:.3f},{min_y:.3f},{min_z:.3f}) "
                    f"max=({max_x:.3f},{max_y:.3f},{max_z:.3f}) "
                    f"scaled_size=({size_x:.3f},{size_y:.3f},{size_z:.3f})"
                )
            except Exception as exc:
                print(f"[WARN] Failed to log dataset bounds: {exc}")

        dynamic_sdf = f"""<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="dataset_object">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{stl_uri}</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.4 0.4 0.4 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>{stl_uri}</uri>
            <scale>{scale} {scale} {scale}</scale>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".sdf", prefix="dataset_object_", delete=False
        ) as sdf_file:
            sdf_file.write(dynamic_sdf)
            model_sdf = sdf_file.name

        obj_x = LaunchConfiguration("obj_x").perform(context)
        obj_y = LaunchConfiguration("obj_y").perform(context)
        obj_r = LaunchConfiguration("obj_R").perform(context)
        obj_p = LaunchConfiguration("obj_P").perform(context)
        obj_yaw = LaunchConfiguration("obj_Y").perform(context)
        print(
            "[INFO] Spawning dataset object "
            f"world=scanner_env path={stl_path} "
            f"pose=({obj_x},{obj_y},{spawn_z},{obj_r},{obj_p},{obj_yaw}) "
            f"scale={scale}"
        )
        return [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros_gz_sim",
                    "create",
                    "-world",
                    "scanner_env",
                    "-name",
                    "dataset_object",
                    "-allow_renaming",
                    "true",
                    "-file",
                    model_sdf,
                    "-x",
                    obj_x,
                    "-y",
                    obj_y,
                    "-z",
                    spawn_z,
                    "-R",
                    obj_r,
                    "-P",
                    obj_p,
                    "-Y",
                    obj_yaw,
                ],
                output="screen",
            )
        ]

    spawn_dataset_object = OpaqueFunction(function=spawn_dataset_object_factory)

    remove_dataset_object_pipeline = TimerAction(
        period=5.0,
        actions=[
            resolve_object,
            load_object_pose,
            remove_dataset_object,
        ],
    )

    spawn_dataset_object_pipeline = TimerAction(
        period=6.5,
        actions=[spawn_dataset_object],
    )

    delayed_robot_publishers = TimerAction(
        period=2.0,
        actions=[
            node_robot_state_publisher,
            TimerAction(period=1.0, actions=[gz_spawn_entity]),
        ],
    )

    nodes = [
        set_gz_sim_resource_path,
        set_ign_gazebo_resource_path,
        set_gz_resource_path,
        gazebo,
        delayed_robot_publishers,
        remove_dataset_object_pipeline,
        spawn_dataset_object_pipeline,
    ]

    return LaunchDescription(arguments + nodes)
