"""ROS publication and optional JSONL recording of source-resolved loads."""
import copy
import json
from pathlib import Path

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64, String

from .forces import ForceCollector


class ForceOutput:
    def __init__(self, node, stamp):
        self.node, self.stamp = node, stamp
        self.publishers = {}
        self.normal_publishers = {}
        self.record = None
        names = [name.strip() for name in node.config["force_usb_base_names"].split(",") if name.strip()]
        self.collector = ForceCollector(node.sim.env, names)
        node.sim.env.force_collector = self.collector
        self.summary = node.create_publisher(String, "/maniskill/forces", 10)
        path = node.config["force_record_path"]
        if path:
            destination = Path(path).expanduser()
            destination.parent.mkdir(parents=True, exist_ok=True)
            # Never silently replace a previous experimental record.
            self.record = destination.open("x", encoding="utf-8", buffering=1)
        node.get_logger().info("Interaction forces → /maniskill/forces; local-frame WrenchStamped "
                               "topics under /maniskill/forces/<sensor>/wrench. "
                               "PhysX contact reports contain normal impulses only.")

    def publish(self):
        snapshot = self.collector.snapshot
        if snapshot is not None:
            self._publish(snapshot)

    def _publish(self, snapshot):
        snapshot = dict(snapshot)
        snapshot["activity"] = {
            side: dict(arm="trajectory" if side in self.node.arms else "hold",
                       gripper="moving" if side in self.node.grippers else "hold")
            for side in ("left", "right")}
        payload = json.dumps(snapshot, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        self.summary.publish(String(data=payload))
        if self.record is not None:
            try:
                self.record.write(payload+"\n")
            except OSError as exc:
                self.node.get_logger().error(f"Force recording stopped: {exc}")
                self.record.close()
                self.record = None
        for key, value in snapshot["sensors"].items():
            if not value["available"]:
                continue
            topic = "/maniskill/forces/"+key
            if key not in self.publishers:
                self.publishers[key] = self.node.create_publisher(WrenchStamped, topic+"/wrench", 10)
            message = WrenchStamped()
            message.header.frame_id = value["frame_id"]
            message.header.stamp = self.stamp(snapshot["time_s"])
            message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = value["force_N"]
            message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = value["torque_Nm"]
            self.publishers[key].publish(message)
            if value["normal_load_N"] is not None:
                if key not in self.normal_publishers:
                    self.normal_publishers[key] = self.node.create_publisher(Float64, topic+"/normal_load", 10)
                self.normal_publishers[key].publish(Float64(data=value["normal_load_N"]))

    def fail(self, reason):
        snapshot = self.collector.snapshot
        if snapshot is None:
            return
        snapshot = copy.deepcopy(snapshot)
        snapshot["failure"] = reason
        for value in snapshot["sensors"].values():
            value["available"] = False
            value["reasons"].append("simulation_failed")
            for field in ("force_N", "torque_Nm", "normal_load_N", "peak_substep_force_N",
                          "peak_substep_normal_load_N"):
                value[field] = None
        self.collector.active = False
        self.collector.snapshot = snapshot
        self._publish(snapshot)

    def close(self):
        if self.record is not None:
            self.record.close()
            self.record = None
