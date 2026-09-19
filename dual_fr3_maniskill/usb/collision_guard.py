"""Asynchronous entry/short-path validation against MoveIt's live planning scene.

Requests carry all measured joints and the measured USB attachment. They never
modify the shared scene or its allowed-collision matrix. This is sampled path
validation, not continuous collision detection; cable contacts remain in PhysX.
"""
import copy
import time

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from sensor_msgs.msg import JointState
from transforms3d.quaternions import mat2quat

from dual_fr3_maniskill.cable.model import USB_LINK
from dual_fr3_maniskill.cable.planning_scene import usb_collision_object
from dual_fr3_maniskill.cable.threading import LEFT_TCP, TOUCH_LINKS
from dual_fr3_maniskill.usb.geometry import SOCKET_NAME


class LocalCollisionGuard:
    def __init__(self, bridge, limits):
        self.bridge, self.limits = bridge, limits
        self.client = bridge.create_client(GetStateValidity, '/check_state_validity')
        self.scene_client = bridge.create_client(GetPlanningScene, '/get_planning_scene')
        self.attachment = AttachedCollisionObject(link_name=LEFT_TCP, touch_links=list(TOUCH_LINKS),
            object=usb_collision_object(bridge.get_parameter('cable_config').value))
        self.futures = []
        self.scene_future = None
        self.started = None
        self.allow_socket = False

    def prepare(self):
        self.cancel()
        if not self.client.service_is_ready() or not self.scene_client.service_is_ready():
            raise RuntimeError('alignment_collision_service_unavailable')
        self.scene_future = self.scene_client.call_async(GetPlanningScene.Request(
            components=PlanningSceneComponents(components=(
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))))
        self.started = time.monotonic()

    def ready(self):
        if self.scene_future is None:
            return True
        self._check_timeout()
        if not self.scene_future.done():
            return False
        scene = self.scene_future.result().scene
        if not any(obj.id == SOCKET_NAME and (obj.meshes or obj.primitives) for obj in scene.world.collision_objects):
            raise RuntimeError('alignment_collision_scene_missing_socket')
        if not any(obj.object.id == USB_LINK for obj in scene.robot_state.attached_collision_objects):
            raise RuntimeError('alignment_collision_scene_missing_usb_attachment')
        if any(obj.id == USB_LINK for obj in scene.world.collision_objects):
            raise RuntimeError('alignment_collision_scene_duplicate_usb')
        self.scene_future = None
        return True

    def submit(self, start, target, mount, *, allow_socket=False):
        if self.futures:
            raise RuntimeError('local_collision_query_already_pending')
        if not self.client.service_is_ready():
            raise RuntimeError('local_collision_service_unavailable')
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, mount[:3, 3])
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = map(float, mat2quat(mount[:3, :3]))
        attachment = copy.deepcopy(self.attachment)
        attachment.object.mesh_poses = [pose]
        # An entry check has no motion: query its state once. Moving steps
        # include start, midpoint and end, even for the smallest servo command.
        steps = max(2, int(np.ceil(np.max(np.abs(target-start)) / self.limits.collision_joint_step_rad)))
        self.allow_socket = allow_socket
        self.started = time.monotonic()
        fractions = [0.] if np.array_equal(start, target) else np.linspace(0., 1., steps + 1)
        for fraction in fractions:
            state = RobotState(is_diff=True, joint_state=JointState(
                name=list(self.bridge.sim.names), position=(start + fraction*(target-start)).tolist()),
                attached_collision_objects=[attachment])
            # Empty group includes both arms, all fingers and attached USB.
            self.futures.append(self.client.call_async(GetStateValidity.Request(robot_state=state)))

    def poll(self):
        self._check_timeout()
        if not all(future.done() for future in self.futures):
            return False
        for future in self.futures:
            result = future.result()
            if result.valid:
                continue
            pairs = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
            # Only the axial insertion may contact its socket. Humble's service
            # reports all body pairs (max_contacts = number_of_bodies squared).
            # A false result without contacts is never interpreted as success.
            if (self.allow_socket and pairs and not result.constraint_result and
                    all(set(pair) == {USB_LINK, SOCKET_NAME} for pair in pairs)):
                continue
            raise RuntimeError('local_path_collision: ' + repr(pairs or ['invalid_state']))
        self.futures = []
        return True

    def _check_timeout(self):
        if self.started is not None and time.monotonic() - self.started > self.limits.collision_timeout_s:
            raise RuntimeError('local_collision_check_timeout')

    def cancel(self):
        for future in [self.scene_future, *self.futures]:
            if future is not None and not future.done():
                future.cancel()
        self.scene_future = None
        self.futures = []
        self.started = None
