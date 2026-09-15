"""Velocity-consistent triangle contact proxies for SAPIEN 2."""
import numpy as np
from transforms3d.quaternions import qmult

from ..sapien_compat import sapien


class KinematicContactProxy:
    """One kinematic link, avoiding SAPIEN 2 SActor's gyroscopic prestep.

    In SAPIEN 2.2.2 a rotating kinematic SActor enters the dynamic-body torque
    path on the following step, invalidating its PhysX motion target. A root
    SKLink has the same kinematic collision behavior without that SActor hook.
    """

    def __init__(self, scene, builder):
        self.scene = scene
        self.articulation = builder.build_kinematic()
        self.link, = self.articulation.get_links()

    def __getattr__(self, name):
        return getattr(self.link, name)

    def set_pose(self, pose):
        self.articulation.set_root_pose(pose)

    def close(self):
        self.scene.remove_kinematic_articulation(self.articulation)


def load_kinematic_target():
    try:
        from .._rope_physx import set_kinematic_target
    except ImportError as exc:
        raise RuntimeError(
            "Rope-Actor finger contacts require the _rope_physx extension. "
            "Build dual_fr3_maniskill after installing SAPIEN in .venv; "
            "for another installation set DUAL_FR3_SAPIEN_INCLUDE_DIR in CMake."
        ) from exc
    return set_kinematic_target


def advance_proxy(proxy, link, dt, set_target):
    """Predict one rigid substep from the link's measured spatial velocity.

    Correct the proxy to the current link pose, then move it through PhysX's
    kinematic target. Contacts therefore see a velocity, including rotation,
    instead of a sequence of teleports with zero boundary velocity.
    """
    pose = link.pose
    omega = np.asarray(link.angular_velocity, dtype=float)
    # PhysX reports linear velocity at the center of mass, while set_pose
    # addresses the link frame (the finger's inertial origin is offset).
    center_of_mass = (pose*link.cmass_local_pose).p
    origin_velocity = np.asarray(link.velocity)+np.cross(omega, pose.p-center_of_mass)
    angle = float(np.linalg.norm(omega))*dt
    dq = np.r_[np.cos(angle/2), omega*dt*np.sinc(angle/(2*np.pi))/2]
    target = sapien.Pose(pose.p+origin_velocity*dt, qmult(dq, pose.q))
    proxy.set_pose(pose)
    set_target(proxy._ptr, target.p.tolist(), target.q.tolist())
    return target
