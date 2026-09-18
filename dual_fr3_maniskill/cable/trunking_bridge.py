"""MTC deferred creation, sharing serialized contact-grasp services with USB demo."""
from dual_fr3_maniskill.ros.bridge import main as bridge_main
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.cable.ros_bridge import UsbCableBridge
from dual_fr3_maniskill.cable.threading import task_usb_mount


class TrunkingCableBridge(UsbCableBridge):
    def create_simulation(self, assets):
        from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
        path = self.declare_parameter("cable_config", resolve_cable_config(scene="trunking_cable")).value
        self.cable_solver = self.declare_parameter("cable_solver", "mpm").value
        self.load_cable = self.declare_parameter("load_cable", True).value
        self.cable_config = load_config(path, solver=self.cable_solver if self.load_cable else None)
        self.cable_config.setdefault("insertion", {})["enabled"] = self.declare_parameter("insertion_enabled", False).value
        if self.cable_config["insertion"]["enabled"] and self.load_cable and self.cable_solver != "rope_actor":
            raise ValueError("Insertion with cable requires rope_actor; MPM deferred")
        self.cable_config["usb"]["orientation_direction"] = self.declare_parameter(
            "leader_orientation_direction", "reverse").value
        task_usb_mount(self.cable_config)
        return TrunkingCableSimulation(assets, cable_config=self.cable_config,
            cable_solver=self.cable_solver, load_cable=self.load_cable,
            control_freq=self.config["control_freq"], sim_freq=self.config["sim_freq"], viewer=self.config["viewer"])


def main():
    bridge_main(TrunkingCableBridge)
