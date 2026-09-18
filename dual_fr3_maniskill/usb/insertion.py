"""Simulation-time insertion policy, independent of motion/physics adapters."""
from dataclasses import dataclass, asdict, fields
import numpy as np
from dual_fr3_maniskill.usb.geometry import DEPTH


# Wire names shared by the ROS bridge and MTC client; no ROS imports here.
SERVICE_PREFIX = '/maniskill/usb/insertion/'
SERVICE_OPERATIONS = (
    'target', 'start', 'status', 'heartbeat', 'cancel', 'retained', 'released',
    'returned', 'returning', 'fail_release', 'fail_return', 'right_release',
    'right_released', 'right_returned', 'fail_right_release', 'fail_right_return',
)
ACTIVE_STATES = ('feedback_advance', 'success_verification')
CANCELLABLE_PREPARATION_STATES = ('approach', 'right_releasing', 'right_returning', 'right_ready')


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
        if not observation['grasp_valid'] or observation['temporary_support']:
            raise RuntimeError('Insertion requires released bilateral contact grasp')
        if (abs(observation['depth_m']+p.preinsert_m) > p.tracking_limit_m or
                observation['lateral_error_m'] > p.lateral_tolerance_m or
                observation['orientation_error_rad'] > p.angle_tolerance_rad):
            raise RuntimeError('Actual USB tip is not aligned at the preinsert pose')
        if not observation['feedback_available']:
            raise RuntimeError('Insertion feedback is unavailable')
        self.started = self.previous = self.progress_time = float(now)
        self.progress_depth = observation['depth_m']
        self.transition('aligned')
        self.transition('feedback_advance')
        return True

    def update(self, now, observation):
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
        self.travel += self.speed*dt
        if self.travel > p.max_advance_m:
            return self.stop('blocked', 'maximum_advance')
        return self.speed

    def snapshot(self):
        return dict(state=self.state, reason=self.reason, insertion_success=self.insertion_success,
            retention_active=self.retention_active, grippers_released=self.grippers_released,
            right_gripper_released=self.right_gripper_released, right_return_complete=self.right_return_complete,
            return_complete=self.return_complete, speed_m_s=self.speed, travel_m=self.travel,
            filtered_resistance_N=self.filtered, observation=self.measurement,
            criteria=dict(self.criteria), transitions=list(self.history),
            retention_record=self.retention_record, limits=asdict(self.limits))
