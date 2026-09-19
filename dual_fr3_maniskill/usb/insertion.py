"""Simulation-time insertion policy, independent of motion/physics adapters."""
from dataclasses import dataclass, asdict, fields
import numpy as np
from dual_fr3_maniskill.usb.geometry import DEPTH
from dual_fr3_maniskill.usb.alignment import AlignmentLimits, error_norm


# Wire names shared by the ROS bridge and MTC client; no ROS imports here.
SERVICE_PREFIX = '/maniskill/usb/insertion/'
SERVICE_OPERATIONS = (
    'target', 'align', 'start', 'status', 'heartbeat', 'cancel', 'retained', 'released',
    'returned', 'returning', 'fail_release', 'fail_return', 'right_release',
    'right_released', 'right_returned', 'fail_right_release', 'fail_right_return',
)
ACTIVE_STATES = ('feedback_advance', 'success_verification')
ALIGNMENT_STATES = ('aligning', 'alignment_verification')
CANCELLABLE_PREPARATION_STATES = ('approach', 'right_releasing', 'right_returning', 'right_ready')


def read_local_collision_check(config):
    mode = config.get('local_collision_check', 'per_step')
    if mode not in ('entry', 'per_step'):
        raise ValueError('insertion.local_collision_check must be entry or per_step')
    return mode


@dataclass(frozen=True)
class InsertionLimits:
    """SI policy defaults; selected cable YAML overrides individual fields.

    Depth/forces use socket axes and the hole centre. Timing is simulation
    time except wall_watchdog_s, which guards the client's monotonic lease.
    """
    preinsert_m: float = .008
    end_clearance_m: float = .001
    target_depth_m: float = .010
    max_advance_m: float = .022
    speed_m_s: float = .001
    acceleration_m_s2: float = .002
    resistance_gain_m_N_s: float = .0004
    filter_tau_s: float = .04
    axial_limit_N: float = 5.
    lateral_limit_N: float = 2.
    torque_limit_Nm: float = .04
    lateral_tolerance_m: float = .00025
    angle_tolerance_rad: float = .025
    depth_tolerance_m: float = .00015
    hold_s: float = .3
    retention_speed_m_s: float = .001
    retention_angular_rad_s: float = .03
    blocked_s: float = 1.5
    progress_epsilon_m: float = .0001
    timeout_s: float = 45.
    wall_watchdog_s: float = 10.
    joint_speed_rad_s: float = .10
    tracking_limit_m: float = .001

    @classmethod
    def read(cls, config):
        result = cls(**{f.name: config[f.name] for f in fields(cls) if f.name in config})
        if any(not np.isfinite(x) or x <= 0 for x in asdict(result).values()):
            raise ValueError('Insertion limits must be positive finite SI values')
        if abs(result.target_depth_m-(DEPTH-result.end_clearance_m)) > 1e-8:
            raise ValueError('target_depth_m must equal CAD cavity depth 0.011 minus end_clearance_m')
        if result.max_advance_m < result.preinsert_m+result.target_depth_m:
            raise ValueError('Maximum advance shorter than preinsert plus target depth')
        return result


class InsertionPolicy:
    ACTIVE = ACTIVE_STATES

    def __init__(self, config):
        self.limits = InsertionLimits.read(config)
        self.alignment_limits = AlignmentLimits.read(config.get('alignment', {}), self.limits)
        self.local_collision_check = read_local_collision_check(config)
        self.reset()

    def reset(self):
        self.state, self.reason = 'not_started', ''
        self.insertion_success = self.retention_active = False
        self.grippers_released = self.return_complete = False
        self.right_gripper_released = self.right_return_complete = False
        self.speed = self.travel = self.filtered = 0.
        self.started = self.previous = self.progress_time = self.hold_since = None
        self.progress_depth = None
        self.measurement = {}
        self.criteria = {}
        self.history = []
        self.retention_record = None
        self.alignment_started = self.alignment_hold_since = None
        self.alignment_progress_time = self.alignment_best_error = None
        self.alignment_travel = self.alignment_rotation = 0.
        self.alignment_initial = {}

    def observation_failure(self, observation):
        values = [observation.get(k) for k in ('depth_m', 'lateral_error_m', 'orientation_error_rad',
            'resistance_N', 'lateral_N', 'torque_Nm', 'relative_speed_m_s', 'relative_angular_rad_s')]
        if not observation.get('feedback_available') or not all(v is not None and np.isfinite(v) for v in values):
            return 'feedback_unavailable', 'invalid_or_missing_sensor'
        if not observation['grasp_valid'] or observation['temporary_support']:
            return 'slip_or_drop', 'grasp_invalid_or_external_support'
        p = self.limits
        if (observation['resistance_N'] > p.axial_limit_N or observation['lateral_N'] > p.lateral_limit_N
                or observation['torque_Nm'] > p.torque_limit_Nm):
            return 'overload', 'raw_contact_limit'
        return None

    def alignment_error_message(self, observation):
        return (f"depth_error_mm={1000*(observation['depth_m']+self.limits.preinsert_m):.4f}, "
                f"lateral_error_mm={1000*observation['lateral_error_m']:.4f}, "
                f"angle_error_deg={np.rad2deg(observation['orientation_error_rad']):.4f}")

    def begin_alignment(self, now, observation):
        if self.state in (*ALIGNMENT_STATES, 'aligned') or self.insertion_success:
            return False
        if self.state not in ('not_started', 'approach'):
            raise RuntimeError('Reset is required after an alignment failure')
        self.measurement = dict(observation)
        failure = self.observation_failure(observation)
        if failure:
            raise RuntimeError('Alignment requires valid released grasp and feedback: ' + failure[1])
        a = self.alignment_limits
        if (error_norm(observation, self.limits.preinsert_m) > a.capture_translation_m or
                observation['orientation_error_rad'] > a.capture_angle_rad):
            raise RuntimeError('USB outside local alignment capture range: ' + self.alignment_error_message(observation))
        self.alignment_initial = dict(observation)
        self.alignment_started = self.alignment_progress_time = float(now)
        self.alignment_hold_since = self.alignment_best_error = None
        self.alignment_travel = self.alignment_rotation = 0.
        self.previous = float(now)
        self.transition('aligning')
        return True

    def update_alignment(self, now, observation):
        """Return whether another correction is needed; completion requires dwell."""
        self.measurement = dict(observation)
        if self.state not in ALIGNMENT_STATES:
            return False
        self.previous = float(now)
        failure = self.observation_failure(observation)
        if failure:
            self.stop(*failure)
            return False
        a, p = self.alignment_limits, self.limits
        position = error_norm(observation, p.preinsert_m)
        angle = observation['orientation_error_rad']
        if position > a.capture_translation_m or angle > a.capture_angle_rad:
            self.stop('blocked', 'alignment_capture_range: ' + self.alignment_error_message(observation))
            return False
        if now - self.alignment_started > a.timeout_s:
            self.stop('timeout', 'alignment_simulation_time_limit')
            return False
        within = position <= a.position_tolerance_m and angle <= a.angle_tolerance_rad
        settled = (observation['relative_speed_m_s'] <= p.retention_speed_m_s and
                   observation['relative_angular_rad_s'] <= p.retention_angular_rad_s)
        self.criteria = dict(alignment_position=position <= a.position_tolerance_m,
                             alignment_orientation=angle <= a.angle_tolerance_rad,
                             low_speed=settled)
        if within and settled:
            if self.alignment_hold_since is None:
                self.alignment_hold_since = now
            self.transition('alignment_verification')
            if now - self.alignment_hold_since >= a.hold_s:
                self.transition('aligned', 'measured_tip_alignment_held')
            return False
        self.alignment_hold_since = None
        self.transition('aligning')
        score = max(position / a.position_tolerance_m, angle / a.angle_tolerance_rad)
        if self.alignment_best_error is None or score < self.alignment_best_error - .05:
            self.alignment_best_error, self.alignment_progress_time = score, now
        elif now - self.alignment_progress_time > a.blocked_s and not within:
            self.stop('blocked', 'alignment_no_measured_progress')
            return False
        return not within

    def record_alignment_step(self, distance, angle):
        a = self.alignment_limits
        if self.alignment_travel + distance > a.max_travel_m or self.alignment_rotation + angle > a.max_rotation_rad:
            raise RuntimeError('alignment_cumulative_motion_limit')
        self.alignment_travel += distance
        self.alignment_rotation += angle

    def transition(self, state, reason=''):
        if self.state != state or self.reason != reason:
            self.history.append(dict(state=state, reason=reason, time_s=self.previous))
        self.state, self.reason = state, reason

    def stop(self, state, reason):
        self.speed = 0.
        self.transition(state, reason)
        return 0.

    def begin(self, now, observation):
        if self.state in self.ACTIVE or self.insertion_success:
            return False
        if self.state not in ('not_started', 'approach', 'aligned'):
            raise RuntimeError('Reset is required after an insertion failure')
        p = self.limits
        self.measurement = dict(observation)
        if not observation['grasp_valid'] or observation['temporary_support']:
            raise RuntimeError('Insertion requires released bilateral contact grasp')
        failure = self.observation_failure(observation)
        if failure:
            raise RuntimeError('Insertion observation rejected: ' + failure[1])
        if (abs(observation['depth_m']+p.preinsert_m) > p.tracking_limit_m or
                observation['lateral_error_m'] > p.lateral_tolerance_m or
                observation['orientation_error_rad'] > p.angle_tolerance_rad):
            raise RuntimeError('Actual USB tip is not aligned at the preinsert pose: ' + self.alignment_error_message(observation))
        if not observation['feedback_available']:
            raise RuntimeError('Insertion feedback is unavailable')
        self.started = self.previous = self.progress_time = float(now)
        self.progress_depth = observation['depth_m']
        self.transition('aligned')
        self.transition('feedback_advance')
        return True

    def update(self, now, observation, *, command_dt=None):
        self.measurement = dict(observation)
        if self.state not in self.ACTIVE:
            return 0.
        dt = float(now)-self.previous
        if dt <= 0:
            return self.speed
        self.previous = float(now)
        p = self.limits
        values = [observation.get(k) for k in ('depth_m', 'lateral_error_m', 'orientation_error_rad',
                    'resistance_N', 'lateral_N', 'torque_Nm', 'relative_speed_m_s', 'relative_angular_rad_s')]
        if not observation.get('feedback_available') or not all(v is not None and np.isfinite(v) for v in values):
            return self.stop('feedback_unavailable', 'invalid_or_missing_sensor')
        if not observation['grasp_valid'] or observation['temporary_support']:
            return self.stop('slip_or_drop', 'grasp_invalid_or_external_support')
        if (observation['resistance_N'] > p.axial_limit_N or observation['lateral_N'] > p.lateral_limit_N
                or observation['torque_Nm'] > p.torque_limit_Nm):
            return self.stop('overload', 'raw_contact_limit')
        if now-self.started > p.timeout_s:
            return self.stop('timeout', 'simulation_time_limit')
        if (observation['lateral_error_m'] > p.lateral_tolerance_m or
                observation['orientation_error_rad'] > p.angle_tolerance_rad):
            return self.stop('blocked', 'lateral_or_orientation_error')
        depth = observation['depth_m']
        if depth > p.target_depth_m+p.depth_tolerance_m:
            return self.stop('blocked', 'depth_overshoot')
        self.filtered += (1-np.exp(-dt/p.filter_tau_s))*(max(0., observation['resistance_N'])-self.filtered)
        self.criteria = dict(depth=depth >= p.target_depth_m-p.depth_tolerance_m,
            lateral=observation['lateral_error_m'] <= p.lateral_tolerance_m,
            orientation=observation['orientation_error_rad'] <= p.angle_tolerance_rad,
            low_speed=observation['relative_speed_m_s'] <= p.retention_speed_m_s and
                observation['relative_angular_rad_s'] <= p.retention_angular_rad_s,
            grasp=observation['grasp_valid'], feedback=observation['feedback_available'], no_overload=True)
        if all(self.criteria.values()):
            if self.hold_since is None:
                self.hold_since = now
            self.transition('success_verification')
            self.speed = 0.
            if now-self.hold_since >= p.hold_s:
                self.criteria['held'] = True
                self.insertion_success = True
                self.transition('inserted_unretained', 'geometry_and_load_criteria_held_without_retention')
            return 0.
        self.hold_since = None
        self.criteria['held'] = False
        self.transition('feedback_advance')
        if depth > self.progress_depth+p.progress_epsilon_m:
            self.progress_depth, self.progress_time = depth, now
        elif now-self.progress_time > p.blocked_s and not self.criteria['depth']:
            return self.stop('blocked', 'no_measured_tip_progress')
        desired = max(0., p.speed_m_s-p.resistance_gain_m_N_s*self.filtered)
        desired = min(desired, max(0., p.target_depth_m-depth)/dt)
        self.speed += float(np.clip(desired-self.speed, -p.acceleration_m_s2*dt, p.acceleration_m_s2*dt))
        if self.criteria['depth']:
            self.speed = 0.
        self.travel += self.speed*(dt if command_dt is None else command_dt)
        if self.travel > p.max_advance_m:
            return self.stop('blocked', 'maximum_advance')
        return self.speed

    def snapshot(self):
        return dict(state=self.state, reason=self.reason, insertion_success=self.insertion_success,
            local_collision_check=self.local_collision_check,
            retention_active=self.retention_active, grippers_released=self.grippers_released,
            right_gripper_released=self.right_gripper_released, right_return_complete=self.right_return_complete,
            return_complete=self.return_complete, speed_m_s=self.speed, travel_m=self.travel,
            filtered_resistance_N=self.filtered, observation=self.measurement,
            criteria=dict(self.criteria), transitions=list(self.history),
            alignment=dict(limits=asdict(self.alignment_limits), initial_observation=self.alignment_initial,
                           travel_m=self.alignment_travel, rotation_rad=self.alignment_rotation),
            retention_record=self.retention_record, limits=asdict(self.limits))
