"""Deferred USB contact grasp with optional cable threaded through both holes."""
from functools import partial

import numpy as np
from transforms3d.quaternions import quat2mat
from ..sapien_compat import sapien

from ..simulation import Simulation
from ..cable.threading import RIGHT_TCP, threaded_positions, threaded_centerline, initial_layout_report
from ..cable.guide import SlidingGuide
from ..cable.backends import create_cable, prepare_backend
from .usb_cable import UsbCableEnv, UsbCableSimulation


class TrunkingCableEnv(UsbCableEnv):
    def _create_cable_for_plug(self, plug):
        config = self.cable_config
        if RIGHT_TCP not in self.agent.links:
            raise ValueError("Threaded cable requires right_fr3_hand_tcp")
        guide = SlidingGuide(self.agent.links[RIGHT_TCP], config["guide"], 0.)
        if self.preparation_tcp_poses and "right" not in self.preparation_tcp_poses:
            raise ValueError("Cable pre-positioning requires a right TCP target")
        target_tcp = self.preparation_tcp_poses.get("right", self.agent.links[RIGHT_TCP].pose)
        initial_hole = target_tcp * sapien.Pose(config["guide"]["center_offset"])

        def layout(cfg, local, r, p):
            if self.cable_solver == "rope_actor":
                points, _ = threaded_centerline(cfg, local, r, p,
                    initial_hole.p, quat2mat(initial_hole.q)[:, 0])
                return points
            points, coordinate = threaded_positions(cfg, local, r, p,
                initial_hole.p, quat2mat(initial_hole.q)[:, 0])
            guide.material_coordinate = coordinate
            return points

        cable = create_cable(self, config, solver=self.cable_solver, plug=plug,
                              layout=layout, guide=guide)
        # Register immediately so any later validation failure cleans it up.
        self.cable = cable
        report = initial_layout_report(config, cable.centerline, quat2mat(plug.pose.q), plug.pose.p,
                                       initial_hole.p, quat2mat(initial_hole.q)[:, 0])
        if not report["passed"]:
            raise ValueError("Initial required-guide alignment failed: " + str(report))
        self.initial_layout_diagnostics = report
        intervals = report["support_material_intervals_m"]
        if self.cable_solver == "rope_actor":
            coordinates = (cable.material_s[:-1]+cable.material_s[1:])/2
            for actor, coordinate in zip(cable.links, coordinates):
                if any(lower <= coordinate <= upper for lower, upper in intervals):
                    drive = self._scene.create_drive(None, actor.pose, actor, sapien.Pose())
                    self.temporary_supports.append(drive)
                    drive.lock_motion(True, True, True, True, True, True)
        else:
            cable.add_temporary_supports(intervals)
        return cable


class TrunkingCableSimulation(UsbCableSimulation):
    def __init__(self, assets, *, cable_config, cable_solver="mpm", load_cable=True, **kwargs):
        if load_cable:
            prepare_backend(cable_solver, cable_config, kwargs.get("sim_freq", 500))
        Simulation.__init__(self, assets,
            env_factory=partial(TrunkingCableEnv, cable_config=cable_config,
                                cable_solver=cable_solver, load_cable=load_cable), **kwargs)
