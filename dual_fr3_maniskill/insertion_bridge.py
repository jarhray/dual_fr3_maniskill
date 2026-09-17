"""Serialized bridge adapter: exclusive local ownership of existing joint drives."""
import json
import time
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from transforms3d.quaternions import quat2mat
from .sapien_compat import sapien
from .insertion_geometry import matrix, pose_dict


class InsertionBridge:
    def __init__(self, bridge):
        self.bridge = bridge
        self.owner = False
        self.last_heartbeat = time.monotonic()
        self.goal = None
        self.model = None
        from .insertion import InsertionPolicy
        self.empty = InsertionPolicy(bridge.cable_config.get("insertion", {}))
        self.publisher = bridge.create_publisher(String, '/maniskill/usb/insertion_state', 10)
        self.services = [bridge.create_service(Trigger, '/maniskill/usb/insertion/'+operation,
            lambda request, response, op=operation: self.command(op, response))
            for operation in ('target', 'start', 'status', 'heartbeat', 'cancel', 'retained', 'released', 'returned', 'returning', 'fail_release', 'fail_return', 'right_release', 'right_released', 'right_returned', 'fail_right_release', 'fail_right_return')]

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

    def command(self, operation, response):
        b, scene = self.bridge, self.scene
        if operation in ("start", "heartbeat"):
            self.last_heartbeat = time.monotonic()
        if scene is None:
            response.success, response.message = False, 'Insertion disabled or reset; spawn first'
            return response
        policy = scene.policy
        try:
            if operation == 'right_release':
                if policy.right_return_complete:
                    payload = policy.snapshot()
                else:
                    if self.owner or b.reserved or b.arms or b.pending_arms or b.grippers:
                        raise RuntimeError('Wait for original MTC actions to finish')
                    if policy.state not in ('not_started', 'right_releasing'):
                        raise RuntimeError('Reset required before right-arm release')
                    self.require_released_grasp()
                    policy.transition('right_releasing')
                    payload = policy.snapshot()
            elif operation == 'right_released':
                if not policy.right_return_complete:
                    if policy.state not in ('right_releasing', 'right_returning'):
                        raise RuntimeError('Right release has not started')
                    self.require_open('right')
                    self.require_released_grasp()
                    policy.right_gripper_released = True
                    policy.transition('right_returning')
                payload = policy.snapshot()
            elif operation == 'right_returned':
                if not policy.right_return_complete:
                    if policy.state != 'right_returning' or not policy.right_gripper_released:
                        raise RuntimeError('Right gripper must be released before return')
                    self.require_open('right')
                    self.require_ready('right')
                    self.require_released_grasp()
                    policy.right_return_complete = True
                    policy.transition('right_ready')
                payload = policy.snapshot()
            elif operation in ('fail_right_release', 'fail_right_return'):
                policy.stop('right_release_failed' if operation == 'fail_right_release' else 'right_return_failed',
                            'right_arm_preinsertion_failed; see_MTC_log')
                payload = policy.snapshot()
            elif operation == 'target':
                if not policy.right_return_complete:
                    raise RuntimeError('Right arm must release and return before insertion approach')
                if self.owner or b.reserved:
                    raise RuntimeError('Motion active; wait for MTC action completion')
                payload = scene.target()
                payload["end_tcp_pose"] = scene.target(policy.limits.target_depth_m)
            elif operation == 'start':
                if self.owner or policy.insertion_success:
                    payload = policy.snapshot()  # idempotent; never replay
                else:
                    if b.reserved or b.arms or b.pending_arms or b.grippers:
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
                    payload = policy.snapshot()
            elif operation == 'cancel':
                if self.owner or policy.state in ('approach', 'right_releasing', 'right_returning', 'right_ready'):
                    policy.stop('cancelled', 'client_cancel_or_shutdown')
                self.relinquish()
                payload = policy.snapshot()
            elif operation == 'retained':
                if not policy.retention_active:
                    raise RuntimeError('USB has not been fixed to socket; keep fingers closed')
                payload = policy.snapshot()
            elif operation == 'released':
                if not policy.retention_active:
                    raise RuntimeError('Release requires socket retention')
                for side in ('left', 'right'):
                    self.require_open(side)
                policy.grippers_released = True
                if not policy.return_complete:
                    policy.transition('grippers_released')
                payload = policy.snapshot()
            elif operation == 'returning':
                if not policy.retention_active or not policy.grippers_released:
                    raise RuntimeError('Return requires retained USB and released grippers')
                if not policy.return_complete:
                    policy.transition('returning')
                payload = policy.snapshot()
            elif operation == 'fail_release':
                policy.stop('release_failed', 'gripper_open_or_planning_detach_failed; see_MTC_log')
                payload = policy.snapshot()
            elif operation == 'returned':
                if not policy.retention_active or not policy.grippers_released:
                    raise RuntimeError('Return cannot complete before retained release')
                for side in ('left', 'right'):
                    self.require_ready(side)
                    self.require_open(side)
                policy.return_complete = True
                policy.transition('complete')
                payload = policy.snapshot()
            elif operation == 'fail_return':
                policy.stop('return_failed', 'release_detach_or_return_failed; see_MTC_log')
                payload = policy.snapshot()
            else:
                payload = policy.snapshot()
            payload['socket_world_pose'] = pose_dict(matrix(scene.base.pose))
            payload['local_controller_owns_left_arm'] = self.owner
            response.success, response.message = True, json.dumps(payload, allow_nan=False)
        except (ValueError, RuntimeError) as exc:
            if operation == 'start' and policy.state in policy.ACTIVE:
                policy.stop('blocked', 'controller_initialization_failed: '+str(exc))
                self.relinquish()
            response.success, response.message = False, str(exc)
        return response

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
