"""Serialized bridge adapter: exclusive local ownership of existing joint drives."""
import json
import time
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from transforms3d.quaternions import quat2mat
from dual_fr3_maniskill.engine.sapien_compat import sapien
from dual_fr3_maniskill.usb.geometry import matrix, pose_dict
from dual_fr3_maniskill.usb.insertion import (
    CANCELLABLE_PREPARATION_STATES,
    InsertionPolicy,
    SERVICE_OPERATIONS,
    SERVICE_PREFIX,
)


class InsertionBridge:
    def __init__(self, bridge):
        self.bridge = bridge
        self.owner = False
        self.last_heartbeat = time.monotonic()
        self.goal = None
        self.model = None
        self.empty = InsertionPolicy(bridge.cable_config.get("insertion", {}))
        self.publisher = bridge.create_publisher(String, '/maniskill/usb/insertion_state', 10)
        self.services = [
            bridge.create_service(
                Trigger, SERVICE_PREFIX + operation,
                lambda request, response, op=operation: self.command(op, response))
            for operation in SERVICE_OPERATIONS
        ]

    @property
    def scene(self):
        return self.bridge.sim.env.insertion

    def relinquish(self):
        b = self.bridge
        if self.owner:
            indices = b.arm_indices['left']
            b.sim.target[indices] = b.sim.positions[indices]
            b.desired['left'] = (b.sim.target[indices].copy(), np.zeros(7), np.zeros(7))
            b.reserved.discard(('left', 'insertion'))
            self.owner = False

    def require_open(self, side):
        b = self.bridge
        minimum = float(self.scene.config.get('release_min_half_width_m', {}).get(side, .035 if side == 'left' else .014))
        if min(b.sim.positions[b.sim.indices[f'{side}_fr3_finger_joint{i}']] for i in (1, 2)) < minimum:
            raise RuntimeError(side+' gripper must have both fingers open')

    def require_ready(self, side):
        b = self.bridge
        indices = b.arm_indices[side]
        ready = np.array([b.assets.initial_positions[n] for n in b.arm_names[side]])
        if max(abs(b.sim.positions[indices]-ready)) > .01 or max(abs(b.sim.velocities[indices])) > .02:
            raise RuntimeError(side+' arm has not reached ready at rest')

    def require_released_grasp(self):
        if self.bridge.failure:
            raise RuntimeError(self.bridge.failure)
        obs = self.scene.observe()
        if not obs['grasp_valid'] or obs['temporary_support']:
            raise RuntimeError('Left arm must retain the USB by released contact grasp')

    # Keep these reasons stable: clients and recordings use the existing text.
    FAILURE_REPORTS = {
        'fail_right_release': ('right_release_failed', 'right_arm_preinsertion_failed; see_MTC_log'),
        'fail_right_return': ('right_return_failed', 'right_arm_preinsertion_failed; see_MTC_log'),
        'fail_release': ('release_failed', 'gripper_open_or_planning_detach_failed; see_MTC_log'),
        'fail_return': ('return_failed', 'release_detach_or_return_failed; see_MTC_log'),
    }

    def command(self, operation, response):
        """Trigger wire boundary; scene mutations run on the serialized bridge."""
        scene = self.scene
        if operation in ('start', 'heartbeat'):
            self.last_heartbeat = time.monotonic()
        if scene is None:
            response.success = False
            response.message = 'Insertion disabled or reset; spawn first'
            return response
        policy = scene.policy
        try:
            payload = self.dispatch(operation)
            if payload is None:
                payload = policy.snapshot()
            payload['socket_world_pose'] = pose_dict(matrix(scene.base.pose))
            payload['local_controller_owns_left_arm'] = self.owner
            response.success = True
            response.message = json.dumps(payload, allow_nan=False)
        except (ValueError, RuntimeError) as exc:
            if operation == 'start' and policy.state in policy.ACTIVE:
                policy.stop('blocked', 'controller_initialization_failed: ' + str(exc))
                self.relinquish()
            response.success = False
            response.message = str(exc)
        return response

    def dispatch(self, operation):
        # Status, heartbeat and historically unknown operations return a snapshot.
        # A heartbeat renews the lease in command(), never through status reads.
        if operation in self.FAILURE_REPORTS:
            self.scene.policy.stop(*self.FAILURE_REPORTS[operation])
        elif operation in SERVICE_OPERATIONS and operation not in ('status', 'heartbeat'):
            return getattr(self, 'handle_' + operation)()

    def actions_busy(self):
        b = self.bridge
        return bool(b.reserved or b.arms or b.pending_arms or b.grippers)

    def handle_right_release(self):
        policy = self.scene.policy
        if policy.right_return_complete:
            return
        if self.owner or self.actions_busy():
            raise RuntimeError('Wait for original MTC actions to finish')
        if policy.state not in ('not_started', 'right_releasing'):
            raise RuntimeError('Reset required before right-arm release')
        self.require_released_grasp()
        policy.transition('right_releasing')

    def handle_right_released(self):
        policy = self.scene.policy
        if policy.right_return_complete:
            return
        if policy.state not in ('right_releasing', 'right_returning'):
            raise RuntimeError('Right release has not started')
        self.require_open('right')
        self.require_released_grasp()
        policy.right_gripper_released = True
        policy.transition('right_returning')

    def handle_right_returned(self):
        policy = self.scene.policy
        if policy.right_return_complete:
            return
        if policy.state != 'right_returning' or not policy.right_gripper_released:
            raise RuntimeError('Right gripper must be released before return')
        self.require_open('right')
        self.require_ready('right')
        self.require_released_grasp()
        policy.right_return_complete = True
        policy.transition('right_ready')

    def handle_target(self):
        scene = self.scene
        if not scene.policy.right_return_complete:
            raise RuntimeError('Right arm must release and return before insertion approach')
        if self.owner or self.bridge.reserved:
            raise RuntimeError('Motion active; wait for MTC action completion')
        payload = scene.target()
        payload['end_tcp_pose'] = scene.target(scene.policy.limits.target_depth_m)
        return payload

    def handle_start(self):
        b, scene = self.bridge, self.scene
        policy = scene.policy
        if self.owner or policy.insertion_success:
            return  # Idempotent: never recreate the model or replay insertion.
        if self.actions_busy():
            raise RuntimeError('MTC/gripper action still owns a drive')
        if b.failure:
            raise RuntimeError(b.failure)
        if not policy.right_return_complete:
            raise RuntimeError('Right arm must return before insertion')
        self.require_open('right')
        self.require_ready('right')
        if policy.begin(b.sim.time, scene.observe()):
            self.goal = b.sim.env.agent.links['left_fr3_hand_tcp'].pose
            self.model = b.sim.env.agent.robot.create_pinocchio_model()
            self.owner = True
            b.reserved.add(('left', 'insertion'))

    def handle_cancel(self):
        policy = self.scene.policy
        if self.owner or policy.state in CANCELLABLE_PREPARATION_STATES:
            policy.stop('cancelled', 'client_cancel_or_shutdown')
        self.relinquish()

    def handle_retained(self):
        # This acknowledges retention; before_tick owns constraint creation.
        if not self.scene.policy.retention_active:
            raise RuntimeError('USB has not been fixed to socket; keep fingers closed')

    def handle_released(self):
        policy = self.scene.policy
        if not policy.retention_active:
            raise RuntimeError('Release requires socket retention')
        for side in ('left', 'right'):
            self.require_open(side)
        policy.grippers_released = True
        if not policy.return_complete:
            policy.transition('grippers_released')

    def handle_returning(self):
        policy = self.scene.policy
        if not policy.retention_active or not policy.grippers_released:
            raise RuntimeError('Return requires retained USB and released grippers')
        if not policy.return_complete:
            policy.transition('returning')

    def handle_returned(self):
        policy = self.scene.policy
        if not policy.retention_active or not policy.grippers_released:
            raise RuntimeError('Return cannot complete before retained release')
        for side in ('left', 'right'):
            self.require_ready(side)
            self.require_open(side)
        policy.return_complete = True
        policy.transition('complete')

    def before_tick(self):
        if not self.owner:
            return
        b, scene = self.bridge, self.scene
        if scene is None:
            self.relinquish()
            return
        policy = scene.policy
        try:
            if time.monotonic()-self.last_heartbeat > policy.limits.wall_watchdog_s:
                policy.stop('timeout', 'client_wall_clock_heartbeat_lost')
            if b.failure:
                policy.stop('feedback_unavailable', b.failure)
            obs = scene.observe()
            # Measurements from the just completed sim interval affect next command.
            speed = policy.update(b.sim.time, obs)
            if policy.state == 'inserted_unretained':
                self.relinquish()
                if scene.config.get('retain_after_success', True):
                    scene.retain()
                return
            if policy.state not in policy.ACTIVE:
                self.relinquish()
                return
            axis = -quat2mat(scene.base.pose.q)[:, 0]
            current = b.sim.env.agent.links['left_fr3_hand_tcp'].pose
            if np.linalg.norm(self.goal.p-current.p) > policy.limits.tracking_limit_m:
                raise RuntimeError('joint_drive_tracking_error')
            self.goal = sapien.Pose(self.goal.p+axis*speed*b.dt, self.goal.q)
            robot = b.sim.env.agent.robot
            link = b.sim.env.agent.links['left_fr3_hand_tcp']
            mask = np.zeros(len(b.sim.names), dtype=int)
            indices = b.arm_indices['left']
            mask[indices] = 1
            q, success, error = self.model.compute_inverse_kinematics(robot.get_links().index(link),
                self.goal, initial_qpos=b.sim.target, active_qmask=mask, eps=1e-6, max_iterations=100)
            if not success:
                raise RuntimeError('insertion_IK_failed:'+str(error))
            if max(abs(q[indices]-b.sim.target[indices])) > policy.limits.joint_speed_rad_s*b.dt:
                raise RuntimeError('insertion_joint_command_rate_limit')
            if any(not b.assets.limits[n][0] <= q[i] <= b.assets.limits[n][1] for n, i in zip(b.arm_names['left'], indices)):
                raise RuntimeError('insertion_joint_limit')
            b.sim.target[indices] = q[indices]
            b.desired['left'] = (q[indices].copy(), np.zeros(7), np.zeros(7))
        except Exception as exc:
            policy.stop('blocked', str(exc))
            self.relinquish()

    def publish(self):
        snapshot = self.scene.policy.snapshot() if self.scene is not None else self.empty.snapshot()
        snapshot['local_controller_owns_left_arm'] = self.owner
        self.publisher.publish(String(data=json.dumps(snapshot, allow_nan=False)))
