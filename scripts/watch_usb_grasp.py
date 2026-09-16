#!/usr/bin/python3
"""Read-only compact display of USB contact state and the two displacement references."""
import argparse
import json
import math
import time

import rclpy
from std_msgs.msg import String


def vector_mm(value):
    return "unavailable" if value is None else "["+", ".join(f"{x*1000:+.2f}" for x in value)+"]"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--period", type=float, default=.5, help="Display period in wall seconds; state changes print immediately")
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.period) or args.period <= 0:
        parser.error("period must be positive and finite")
    rclpy.init(args=ros_args)
    node = rclpy.create_node("usb_grasp_watch")
    previous, last_print = None, -math.inf

    def receive(message):
        nonlocal previous, last_print
        data = json.loads(message.data)
        current = (data["state"], data["reason"])
        now = time.monotonic()
        if current == previous and now-last_print < args.period:
            return
        previous, last_print = current, now
        angle = data.get("relative_rotation_rad")
        angle_text = "unavailable" if angle is None else f"{math.degrees(angle):.2f}deg"
        loads = data.get("finger_normal_loads_N", {})
        force_text = "/".join("unavailable" if loads.get(name) is None else f"{loads[name]:.2f}"
            for name in ("left_fr3_leftfinger", "left_fr3_rightfinger"))
        print(f"t={data['time_s']:.2f}s {data['state']} support={data['external_support']} | "
              f"world_from_creation_mm={vector_mm(data.get('world_displacement_m'))} | "
              f"slip_in_tcp_mm={vector_mm(data.get('relative_displacement_m'))} rot={angle_text} | "
              f"finger_normal_N={force_text} | {data['reason']}", flush=True)

    node.create_subscription(String, "/maniskill/usb/grasp_state", receive, 10)
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
