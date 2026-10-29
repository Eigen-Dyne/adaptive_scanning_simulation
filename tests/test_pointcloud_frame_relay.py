"""Gazebo clouds must carry optical coordinates as well as an optical frame ID."""
import importlib.util
from pathlib import Path
import struct

import pytest
from sensor_msgs.msg import PointCloud2, PointField

spec = importlib.util.spec_from_file_location(
    "pointcloud_frame_relay",
    Path(__file__).resolve().parents[1] / "as_sim/scripts/pointcloud_frame_relay.py",
)
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)


@pytest.mark.parametrize("bigendian", [False, True])
@pytest.mark.parametrize("datatype,format", [(PointField.FLOAT32, "f"), (PointField.FLOAT64, "d")])
def test_optical_conversion_preserves_color_padding_and_source(bigendian, datatype, format):
    endian = ">" if bigendian else "<"
    size = struct.calcsize(format)
    step = 3 * size + 4
    row_step = 2 * step + 8
    data = bytearray([0xAB] * (2 * row_step))
    for row in range(2):
        for col in range(2):
            struct.pack_into(endian + 3 * format, data, row * row_step + col * step, 1., 2., 3.)
    msg = PointCloud2(height=2, width=2, is_bigendian=bigendian,
                     point_step=step, row_step=row_step, data=bytes(data),
                     fields=[PointField(name=name, offset=i * size, datatype=datatype, count=1)
                             for i, name in enumerate(("x", "y", "z"))])
    msg.header.frame_id = "gazebo_camera"
    result = relay.gazebo_to_optical(msg)
    expected = bytearray(data)
    for row in range(2):
        for col in range(2):
            offset = row * row_step + col * step
            assert struct.unpack_from(endian + 3 * format, result.data, offset) == (-2., -3., 1.)
            struct.pack_into(endian + 3 * format, expected, offset, -2., -3., 1.)
    assert bytes(result.data) == bytes(expected)
    assert bytes(msg.data) == bytes(data)
    assert result.header == msg.header
    assert result.fields == msg.fields
