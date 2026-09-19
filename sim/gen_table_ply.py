#!/usr/bin/env python3
"""Generate a synthetic table.ply for AdaptiveScanning simulation.

scan_and_perceive.launch.py / edge_scan.launch.py refuse to start unless a
table calibration point cloud exists (see launch_requirements.require_table_ply_actions).
On real hardware that file is produced by table_calibration.launch.py from the
depth camera. For simulation we can synthesize an equivalent: a dense flat sheet
of points, expressed in the robot base frame, lying on the table plane.

Consumers (all read base-frame XYZ points via open3d.read_point_cloud):
  * table_plane_from_ply.py   -> fits/【publishes the table collision plane
  * base_pc_generator (C++)   -> RANSAC table model used to drop table points

The plane height defaults to scan_params `table_plane_z_m` (-0.12 m) and the
footprint covers the configured scan workspace with margin.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import struct
from datetime import datetime, timezone
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, help="Output .ply path")
    parser.add_argument("--z", type=float, default=-0.12, help="Table plane Z in base frame (m)")
    parser.add_argument("--x-min", type=float, default=-0.55)
    parser.add_argument("--x-max", type=float, default=0.55)
    parser.add_argument("--y-min", type=float, default=0.20)
    parser.add_argument("--y-max", type=float, default=0.95)
    parser.add_argument("--step", type=float, default=0.01, help="Grid spacing (m)")
    parser.add_argument("--ascii", action="store_true", help="Write ASCII PLY instead of binary")
    args = parser.parse_args()

    xs = _frange(args.x_min, args.x_max, args.step)
    ys = _frange(args.y_min, args.y_max, args.step)
    points = [(x, y, args.z) for y in ys for x in xs]
    n = len(points)

    if args.ascii:
        with open(args.out, "w", encoding="ascii") as fh:
            fh.write("ply\nformat ascii 1.0\n")
            fh.write(f"element vertex {n}\n")
            fh.write("property float x\nproperty float y\nproperty float z\n")
            fh.write("end_header\n")
            for x, y, z in points:
                fh.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    else:
        with open(args.out, "wb") as fh:
            header = (
                "ply\n"
                "format binary_little_endian 1.0\n"
                f"element vertex {n}\n"
                "property float x\nproperty float y\nproperty float z\n"
                "end_header\n"
            )
            fh.write(header.encode("ascii"))
            for x, y, z in points:
                fh.write(struct.pack("<fff", x, y, z))

    output = Path(args.out)
    metadata = {
        "schema_version": 1,
        "source": "simulation",
        "frame_id": "base_link",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "table_ply_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "transforms": [
            {
                "parent": "base_link",
                "child": "table_plane",
                "translation_m": [0.0, 0.0, args.z],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
            ,
            {
                "parent": "scanner_fixture_bottom_link",
                "child": "camera_link",
                "translation_m": [0.0677418, 0.00951453, 0.104885],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        ],
    }
    output.with_name("metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )

    print(f"[gen_table_ply] wrote {n} points and simulation metadata to {output.parent}")


def _frange(lo: float, hi: float, step: float):
    vals = []
    v = lo
    # inclusive of hi within floating tolerance
    while v <= hi + 1e-9:
        vals.append(round(v, 6))
        v += step
    return vals


if __name__ == "__main__":
    main()
