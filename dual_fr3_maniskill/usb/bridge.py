"""Serialized bridge adapter: exclusive local ownership of existing joint drives."""
import json
import time
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from transforms3d.quaternions import mat2quat, quat2mat
from dual_fr3_maniskill.engine.sapien_compat import sapien
from dual_fr3_maniskill.usb.geometry import matrix, pose_dict
from dual_fr3_maniskill.usb.alignment import correction_goal, rotation_distance
from dual_fr3_maniskill.usb.insertion import (
    ALIGNMENT_STATES,
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
        self.collision_guard = None
        self.pending_step = None
        self.alignment_checked = False
        self.entry_waiting = False
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
        guard = getattr(self, 'collision_guard', None)
        if guard is not None:
            guard.cancel()
        self.pending_step = None
        self.entry_waiting = False
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
        if operation in ('align', 'start', 'heartbeat'):
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
            if ((operation == 'start' and policy.state in policy.ACTIVE) or
                    (operation == 'align' and policy.state in ALIGNMENT_STATES)):
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
        if policy.state in ALIGNMENT_STATES:
            raise RuntimeError('Wait for measured local alignment to finish')
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
        check_entry = (policy.local_collision_check == 'entry' and
                       not (policy.state == 'aligned' and getattr(self, 'alignment_checked', False)))
        if policy.begin(b.sim.time, scene.observe()):
            if check_entry:
                self.prepare_collision_check()
                self.entry_waiting = True
            self.goal = b.sim.env.agent.links['left_fr3_hand_tcp'].pose
            self.model = b.sim.env.agent.robot.create_pinocchio_model()
            self.owner = True
            b.reserved.add(('left', 'insertion'))

    def handle_align(self):
        b, scene = self.bridge, self.scene
        policy = scene.policy
        if policy.state in ALIGNMENT_STATES or policy.state == 'aligned' or policy.insertion_success:
            return
        if self.actions_busy() or self.owner:
            raise RuntimeError('MTC/gripper action still owns a drive')
        if b.failure:
            raise RuntimeError(b.failure)
        if not policy.right_return_complete:
            raise RuntimeError('Right arm must return before alignment')
        self.require_open('right')
        self.require_ready('right')
        if policy.begin_alignment(b.sim.time, scene.observe()):
            self.prepare_collision_check()
            self.goal = b.sim.env.agent.links['left_fr3_hand_tcp'].pose
            self.model = b.sim.env.agent.robot.create_pinocchio_model()
            self.owner = True
            b.reserved.add(('left', 'insertion'))

    def prepare_collision_check(self):
        from dual_fr3_maniskill.usb.collision_guard import LocalCollisionGuard
        if getattr(self, 'collision_guard', None) is None:
            self.collision_guard = LocalCollisionGuard(self.bridge, self.scene.policy.alignment_limits)
        self.collision_guard.prepare()
        self.pending_step = None
        self.alignment_checked = False

    def check_local_entry(self):
        """Hold the arm until MoveIt has validated the measured entry state."""
        if not self.collision_guard.ready():
            return False
        if not self.alignment_checked:
            if self.pending_step is None:
                self.submit_checked_step(self.bridge.sim.positions.copy(),
                    self.bridge.sim.env.agent.links['left_fr3_hand_tcp'].pose, alignment=True)
            else:
                self.finish_checked_step(alignment=True)
        return self.alignment_checked

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
            if policy.state in ALIGNMENT_STATES:
                self.alignment_tick(obs)
                return
            if getattr(self, 'entry_waiting', False) and policy.state in policy.ACTIVE:
                # Direct start also needs an entry check. No insertion command
                # has been issued yet, so its motion timers start on approval.
                failure = policy.observation_failure(obs)
                if failure:
                    policy.stop(*failure)
                    self.relinquish()
                elif b.sim.time - policy.started > policy.alignment_limits.timeout_s:
                    policy.stop('timeout', 'entry_simulation_time_limit')
                    self.relinquish()
                elif self.check_local_entry():
                    policy.started = policy.previous = policy.progress_time = b.sim.time
                    policy.progress_depth = obs['depth_m']
                    self.entry_waiting = False
                return
            # Measurements from the just completed sim interval affect next command.
            guarded = (policy.local_collision_check == 'per_step' and
                       getattr(self, 'collision_guard', None) is not None)
            speed = policy.update(b.sim.time, obs, command_dt=0. if guarded else None)
            if policy.state == 'inserted_unretained':
                self.relinquish()
                if scene.config.get('retain_after_success', True):
                    scene.retain()
                return
            if policy.state not in policy.ACTIVE:
                self.relinquish()
                return
            if speed == 0.:
                if guarded:
                    self.collision_guard.cancel()
                    self.pending_step = None
                return
            if getattr(self, 'pending_step', None) is not None:
                self.finish_checked_step(alignment=False)
                if self.pending_step is not None:
                    return
                # Prepare the next checked step in this same tick. Waiting an
                # extra tick here would halve the original insertion speed.
            axis = -quat2mat(scene.base.pose.q)[:, 0]
            current = b.sim.env.agent.links['left_fr3_hand_tcp'].pose
            if np.linalg.norm(self.goal.p-current.p) > policy.limits.tracking_limit_m:
                raise RuntimeError('joint_drive_tracking_error')
            goal = sapien.Pose(self.goal.p+axis*speed*b.dt, self.goal.q)
            robot = b.sim.env.agent.robot
            link = b.sim.env.agent.links['left_fr3_hand_tcp']
            mask = np.zeros(len(b.sim.names), dtype=int)
            indices = b.arm_indices['left']
            mask[indices] = 1
            q, success, error = self.model.compute_inverse_kinematics(robot.get_links().index(link),
                goal, initial_qpos=b.sim.target, active_qmask=mask, eps=1e-6, max_iterations=100)
            if not success:
                raise RuntimeError('insertion_IK_failed:'+str(error))
            if max(abs(q[indices]-b.sim.target[indices])) > policy.limits.joint_speed_rad_s*b.dt:
                raise RuntimeError('insertion_joint_command_rate_limit')
            if any(not b.assets.limits[n][0] <= q[i] <= b.assets.limits[n][1] for n, i in zip(b.arm_names['left'], indices)):
                raise RuntimeError('insertion_joint_limit')
            if not guarded:
                self.apply_step(q, goal)
            else:
                self.submit_checked_step(q, goal, alignment=False)
        except Exception as exc:
            policy.stop('blocked', str(exc))
            self.relinquish()

    def apply_step(self, q, goal):
        b = self.bridge
        indices = b.arm_indices['left']
        self.goal = goal
        b.sim.target[indices] = q[indices]
        b.desired['left'] = (q[indices].copy(), np.zeros(7), np.zeros(7))

    def measured_mount(self):
        env = self.bridge.sim.env
        return matrix(env.agent.links['left_fr3_hand_tcp'].pose.inv() * env.plug.pose)

    def submit_checked_step(self, q, goal, *, alignment):
        start = self.bridge.sim.positions.copy()
        target = start.copy()
        indices = self.bridge.arm_indices['left']
        target[indices] = q[indices]
        mount = self.measured_mount()
        self.collision_guard.submit(start, target, mount, allow_socket=not alignment)
        self.pending_step = (target, goal, start, mount)

    def finish_checked_step(self, *, alignment):
        if not self.collision_guard.poll():
            return
        q, goal, start, mount = self.pending_step
        self.pending_step = None
        b, policy = self.bridge, self.scene.policy
        current_mount = self.measured_mount()
        # A result for a stale grasp or start cannot authorize a new movement.
        if (np.max(np.abs(b.sim.positions-start)) > policy.alignment_limits.collision_joint_step_rad/2 or
                np.linalg.norm(current_mount[:3, 3]-mount[:3, 3]) > .00005 or
                rotation_distance(current_mount[:3, :3], mount[:3, :3]) > .002):
            return
        if alignment:
            policy.record_alignment_step(float(np.linalg.norm(goal.p-self.goal.p)),
                rotation_distance(quat2mat(self.goal.q), quat2mat(goal.q)))
            self.alignment_checked = True
        else:
            distance = float(np.linalg.norm(goal.p-self.goal.p))
            if policy.travel + distance > policy.limits.max_advance_m:
                raise RuntimeError('maximum_advance')
            policy.travel += distance
        self.apply_step(q, goal)

    def alignment_tick(self, observation):
        b, scene = self.bridge, self.scene
        policy = scene.policy
        failure = policy.observation_failure(observation)
        if failure:
            policy.stop(*failure)
            self.relinquish()
            return
        if b.sim.time - policy.alignment_started > policy.alignment_limits.timeout_s:
            policy.stop('timeout', 'alignment_simulation_time_limit')
            self.relinquish()
            return
        if not self.check_local_entry():
            return
        correction_needed = policy.update_alignment(b.sim.time, observation)
        if policy.state not in ALIGNMENT_STATES:
            self.relinquish()
            return
        if not correction_needed:
            # Never execute an in-flight correction once measured convergence
            # starts its dwell; a fresh measurement will decide the next step.
            self.collision_guard.cancel()
            self.pending_step = None
            return
        if self.pending_step is not None:
            self.finish_checked_step(alignment=True)
            return
        env = b.sim.env
        current = env.agent.links['left_fr3_hand_tcp'].pose
        if (np.linalg.norm(self.goal.p-current.p) > policy.limits.tracking_limit_m or
                rotation_distance(quat2mat(self.goal.q), quat2mat(current.q)) > policy.alignment_limits.capture_angle_rad):
            raise RuntimeError('alignment_joint_drive_tracking_error')
        indices = b.arm_indices['left']
        robot = env.agent.robot
        link_index = robot.get_links().index(env.agent.links['left_fr3_hand_tcp'])
        seed = b.sim.positions.copy()
        mask = np.zeros(len(b.sim.names), dtype=int)
        mask[indices] = 1
        # Stay on the current IK branch. Reduce the local step if its required
        # joint motion is too large; never invoke a random/global IK restart.
        for scale in (1., .5, .25, .125):
            target = correction_goal(matrix(scene.base.pose), matrix(current), matrix(env.plug.pose),
                scene.hole, policy.limits.preinsert_m, policy.alignment_limits, b.dt*scale)
            # Integrate the measured correction on the held drive target so a
            # small steady PD/load offset cannot swallow every micro-step.
            # The tracking and cumulative-motion guards bound this integration.
            target = target @ np.linalg.inv(matrix(current)) @ matrix(self.goal)
            goal = sapien.Pose(target[:3, 3], mat2quat(target[:3, :3]))
            q, success, error = self.model.compute_inverse_kinematics(link_index,
                robot.pose.inv()*goal, initial_qpos=seed, active_qmask=mask,
                eps=1e-7, max_iterations=100, damp=1e-6)
            if not success or not np.isfinite(q).all():
                continue
            if max(abs(q[indices]-b.sim.target[indices])) > policy.limits.joint_speed_rad_s*b.dt:
                continue
            if any(not b.assets.limits[n][0] <= q[i] <= b.assets.limits[n][1]
                   for n, i in zip(b.arm_names['left'], indices)):
                continue
            if policy.local_collision_check == 'entry':
                policy.record_alignment_step(float(np.linalg.norm(goal.p-self.goal.p)),
                    rotation_distance(quat2mat(self.goal.q), quat2mat(goal.q)))
                self.apply_step(q, goal)
            else:
                self.submit_checked_step(q, goal, alignment=True)
            return
        raise RuntimeError('alignment_local_IK_or_joint_limit: ' + str(error))

    def publish(self):
        snapshot = self.scene.policy.snapshot() if self.scene is not None else self.empty.snapshot()
        snapshot['local_controller_owns_left_arm'] = self.owner
        self.publisher.publish(String(data=json.dumps(snapshot, allow_nan=False)))
