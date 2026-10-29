#!/usr/bin/env python3
"""Check that ROS simulation time is published and monotonic."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockMonitor(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("sim_clock_monotonic_check")
        self.samples: list[int] = []
        self.backward_jumps: list[tuple[int, int]] = []
        self._last_ns: int | None = None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Clock, topic, self._clock_cb, qos)

    def _clock_cb(self, msg: Clock) -> None:
        now_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        if self._last_ns is not None and now_ns < self._last_ns:
            self.backward_jumps.append((self._last_ns, now_ns))
        self._last_ns = now_ns
        self.samples.append(now_ns)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/clock")
    parser.add_argument("--duration-sec", type=float, default=8.0)
    parser.add_argument("--min-samples", type=int, default=20)
    args = parser.parse_args()

    rclpy.init()
    node = ClockMonitor(args.topic)
    deadline = time.monotonic() + max(1.0, float(args.duration_sec))
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    unique_samples = len(set(node.samples))
    if len(node.samples) < max(1, int(args.min_samples)):
        print(
            f"{args.topic} published too few samples: "
            f"{len(node.samples)} < {args.min_samples}",
            file=sys.stderr,
        )
        return 1
    if unique_samples < 2:
        print(f"{args.topic} did not advance during the sample window", file=sys.stderr)
        return 1
    if node.backward_jumps:
        previous_ns, current_ns = node.backward_jumps[0]
        print(
            f"{args.topic} moved backward: {previous_ns} ns -> {current_ns} ns",
            file=sys.stderr,
        )
        return 1

    elapsed_ns = node.samples[-1] - node.samples[0]
    print(
        f"{args.topic} monotonic: samples={len(node.samples)} "
        f"unique={unique_samples} elapsed_sim_s={elapsed_ns / 1e9:.3f}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
