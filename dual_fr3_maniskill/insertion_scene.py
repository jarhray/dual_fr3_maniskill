"""Socket physics and insertion observations; mutate only between scene steps."""
from pathlib import Path
import tempfile
import numpy as np
from transforms3d.quaternions import mat2quat, quat2mat
from .sapien_compat import sapien
from .insertion import InsertionPolicy
from .insertion_geometry import SOCKET_NAME, SOCKET_INSTALLATION_FIXTURES, HOLE, matrix, measure, tcp_goal, pose_dict, socket_parts


class SocketInsertion:
    def __init__(self, env):
        self.env = env
        self.config = env.cable_config.get('insertion', {})
        self.hole = np.asarray(self.config.get("hole_center_m", HOLE), dtype=float)
        self.policy = InsertionPolicy(self.config)
        self.base = self.base_support = self.retention = None
        self.installation_filters = []
        self.cache = None
        self.wrench = dict(available=False, reason='not_sampled')
        self.normal_wrench = np.zeros(6)
        self._sum = np.zeros(6)
        self._duration = 0.
        self._reasons = set()
        self._peak = np.zeros(3)
        try:
            self.create()
        except Exception:
            self.close()
            raise

    def create(self):
        from ament_index_python.packages import get_package_share_directory
        from .assets import convert_stl_to_glb
        root = Path(get_package_share_directory('dual_fr3_maniskill'))
        c, env = self.config, self.env
        self.cache = tempfile.TemporaryDirectory(prefix='usb_socket_')
        builder = env._scene.create_actor_builder()
        material = env._scene.create_physical_material(.5, .5, 0.)
        parts = socket_parts(root/c.get('collision_mesh', 'meshes/usb_base_collision.stl'),
                             c.get('clearance_yz_m', [.0004, .0004]), self.hole)
        for index, mesh in enumerate(parts):
            path = Path(self.cache.name)/f'part_{index}.stl'
            mesh.export(path)
            builder.add_collision_from_file(str(path), material=material)
        if env._renderer is not None:
            visual = convert_stl_to_glb(root/c.get('visual_mesh', 'meshes/usb_base_visual.stl'), Path(self.cache.name))
            builder.add_visual_from_file(str(visual))
        builder.set_mass_and_inertia(.1, sapien.Pose(), [.0001]*3)
        self.base = builder.build(SOCKET_NAME)
        # A visible, instrumented base-to-world support. No USB attachment here.
        self.base.set_pose(self.resolve_pose())
        from ._rope_physx import disable_gravity
        disable_gravity(self.base._ptr)
        for shape in self.base.get_collision_shapes():
            shape.contact_offset = c.get('contact_offset_m', .0001)
            shape.rest_offset = c.get('rest_offset_m', 0.)
        if len(self.base.get_collision_shapes()) != len(parts):
            raise RuntimeError('Socket convex-prism import lost collision shapes')
        self.base_support = env._scene.create_drive(None, self.base.pose, self.base, sapien.Pose())
        self.base_support.lock_motion(True, True, True, True, True, True)
        env.fixtures[SOCKET_NAME] = self.base
        self.exclude_installation_contacts()
        # Import fails explicitly at startup rather than substituting finger loads.
        from ._rope_physx import read_world_constraint
        self.read_support = read_world_constraint

    def resolve_pose(self):
        c, env = self.config, self.env
        frame = env.agent.links[c.get('xy_frame', 'left_fr3_link0')].pose
        xy = c.get('xy_m', [.37, -.07])
        p = frame.p+quat2mat(frame.q)@np.array([*xy, 0.])
        trunking = env.fixtures[c.get('height_frame', 'trunking')].pose
        p[2] = (trunking.p+quat2mat(trunking.q)@np.array([0., 0., c.get('height_m', .010)]))[2]
        return sapien.Pose(p, c.get('quaternion_wxyz', [1., 0., 0., 0.]))

    def exclude_installation_contacts(self):
        # One unused ignore bit per fixed installation pair. Never suppress
        # USB, robot, cable or fixture-to-fixture collision affinities.
        actors = self.env._scene.get_all_actors() + self.env._scene.get_all_articulations()
        used = 0
        for actor in actors:
            for body in actor.get_links() if hasattr(actor, 'get_links') else [actor]:
                for shape in body.get_collision_shapes():
                    used |= shape.get_collision_groups()[2]
        for name in SOCKET_INSTALLATION_FIXTURES:
            fixture = self.env.fixtures.get(name)
            if fixture is None:
                continue
            bit = next((1 << i for i in range(32) if not used & (1 << i)), None)
            if bit is None:
                raise RuntimeError('No collision filter bit available for socket installation')
            used |= bit
            socket_shapes = self.base.get_collision_shapes()
            fixture_shapes = fixture.get_collision_shapes()
            if any((s.get_collision_groups()[3] & 0xffff) !=
                   (socket_shapes[0].get_collision_groups()[3] & 0xffff) for s in fixture_shapes):
                raise RuntimeError('Socket installation filter requires matching actor filter domains')
            for shape in [*socket_shapes, *fixture_shapes]:
                groups = list(shape.get_collision_groups())
                groups[2] |= bit
                shape.set_collision_groups(*groups)
                self.installation_filters.append((shape, bit))

    def begin_window(self):
        self._sum[:] = 0.
        self.normal_wrench[:] = 0.
        self._duration = 0.
        self._reasons.clear()
        self._peak[:] = 0.

    def sample(self, dt):
        """Isolated fixed-base reaction includes friction; never add normal twice.

        Base gravity is explicitly disabled. During insertion only USB may load
        this base; any other contacting actor makes the reaction unidentifiable.
        Sensor output is ON USB, in socket axes, moment about the hole centre.
        """
        env, base = self.env, self.base
        plug = env.plug
        rotation = quat2mat(base.pose.q)
        hole = base.pose.p+rotation@self.hole
        normal = np.zeros(6)
        contacts = False
        for contact in env._scene.get_contacts():
            a, b = contact.actor0, contact.actor1
            if base.id not in (a.id, b.id):
                continue
            other = b if a.id == base.id else a
            loaded = any(np.linalg.norm(pt.impulse) > 1e-12 for pt in contact.points)
            if not loaded:
                continue
            contacts = True
            if plug is None or other.id != plug.id:
                self._reasons.add('base_contact_from_other_actor:'+other.name)
            for pt in contact.points:
                force = np.asarray(pt.impulse)*(1 if other.id == a.id else -1)/dt
                normal += np.r_[rotation.T@force, rotation.T@np.cross(pt.position-hole, force)]
        value = np.zeros(6)
        try:
            if self.retention is not None:
                self._reasons.add('retention_active_support_reaction_not_insertion_contact')
            elif contacts:
                load = self.read_support(base._ptr)
                if not load['awake']:
                    raise RuntimeError('support_reaction_stale')
                f, torque = np.asarray(load['force']), np.asarray(load['torque'])
                # World support ON base balances USB ON base. Therefore equals
                # base ON USB at static equilibrium (checked by native probe).
                value = np.r_[rotation.T@f, rotation.T@(torque+np.cross(np.asarray(load['origin'])-hole, f))]
                if np.linalg.norm(base.velocity) > .0001 or np.linalg.norm(base.angular_velocity) > .001:
                    raise RuntimeError('base_not_quasistatic')
            if not np.isfinite(value).all():
                raise RuntimeError('nonfinite_support_reaction')
        except (ValueError, RuntimeError) as exc:
            self._reasons.add(str(exc))
        self._sum += value*dt
        self.normal_wrench += normal*dt
        self._duration += dt
        self._peak = np.maximum(self._peak, [max(0., value[0]), np.linalg.norm(value[1:3]), np.linalg.norm(value[3:])])

    def finish_window(self):
        ok = not self._reasons and self._duration > 0
        value = self._sum/max(self._duration, 1e-15)
        self.wrench = dict(available=ok, reasons=sorted(self._reasons), frame_id=SOCKET_NAME,
            reference_point='hole_center', force_unit='N', torque_unit='N*m',
            convention='force_on_USB; +socket_X_resists_insertion_along_-X',
            source='isolated_world_support_reaction; base_gravity_disabled',
            scope='full_contact_reaction_quasistatic_base; excludes_retention',
            force_N=value[:3].tolist() if ok else None, torque_Nm=value[3:].tolist() if ok else None,
            peak_resistance_N=float(self._peak[0]), peak_lateral_N=float(self._peak[1]), peak_torque_Nm=float(self._peak[2]),
            normal_only_wrench=(self.normal_wrench/max(self._duration, 1e-15)).tolist())
        self.policy.measurement = self.observe()

    def observe(self):
        env, p = self.env, self.policy.limits
        if env.plug is None:
            return dict(feedback_available=False, reason='usb_missing')
        obs = measure(matrix(self.base.pose), matrix(env.plug.pose), self.hole)
        monitor = env.grasp_monitor.snapshot()
        force = self.wrench.get('force_N')
        obs.update(time_s=env._grasp_time, feedback_available=self.wrench['available'],
            resistance_N=None if force is None else max(0., force[0]),
            lateral_N=None if force is None else float(np.linalg.norm(force[1:])),
            torque_Nm=None if force is None else float(np.linalg.norm(self.wrench['torque_Nm'])),
            relative_speed_m_s=float(np.linalg.norm(env.plug.velocity-self.base.velocity)),
            relative_angular_rad_s=float(np.linalg.norm(env.plug.angular_velocity-self.base.angular_velocity)),
            grasp_valid=monitor['stable'], grasp_state=monitor['state'],
            temporary_support=bool(env.support_drive or env.temporary_supports),
            grasp_relative_translation_m=monitor['relative_translation_m'],
            grasp_relative_rotation_rad=monitor['relative_rotation_rad'], wrench=self.wrench,
            physical_constraints=dict(usb_to_tcp=0 if env.mount_drive is None else 1,
                usb_to_world=int(env.support_drive is not None), usb_to_socket=int(self.retention is not None),
                base_to_world=int(self.base_support is not None),
                usb_to_cable=int(getattr(env.cable, 'anchor', None) is not None)),
            pose_source='simulation_ground_truth', usb_world_pose=pose_dict(matrix(env.plug.pose)))
        # Protect using substep peaks as well as the control-period mean.
        obs['resistance_N'] = max(obs['resistance_N'], self._peak[0]) if force is not None else None
        obs['lateral_N'] = max(obs['lateral_N'], self._peak[1]) if force is not None else None
        obs['torque_Nm'] = max(obs['torque_Nm'], self._peak[2]) if force is not None else None
        return obs

    def target(self, depth=None):
        env = self.env
        if env.plug is None or not env.grasp_monitor.snapshot()['stable'] or env.support_drive is not None:
            raise RuntimeError('Approach requires stable released USB contact grasp')
        if self.policy.state not in ('not_started', 'right_ready', 'approach', 'aligned'):
            raise RuntimeError('Reset required before another insertion')
        self.policy.transition('approach')
        result = tcp_goal(matrix(self.base.pose), matrix(env.agent.links['left_fr3_hand_tcp'].pose),
            matrix(env.plug.pose), -self.policy.limits.preinsert_m if depth is None else depth, self.hole)
        return pose_dict(result)

    def retain(self):
        policy, env = self.policy, self.env
        if self.retention is not None:
            return True
        if not policy.insertion_success or policy.state != 'inserted_unretained':
            raise RuntimeError('Retention requires recorded success without a USB fixture')
        obs = self.observe()
        p = policy.limits
        if (not obs['grasp_valid'] or not obs['feedback_available'] or
                abs(obs['depth_m']-p.target_depth_m) > p.depth_tolerance_m or
                obs['lateral_error_m'] > p.lateral_tolerance_m or obs['orientation_error_rad'] > p.angle_tolerance_rad or
                obs['relative_speed_m_s'] > p.retention_speed_m_s or obs['relative_angular_rad_s'] > p.retention_angular_rad_s):
            policy.stop('retention_failed', 'success_or_low_speed_no_longer_valid')
            return False
        def sample():
            return dict(pose=pose_dict(matrix(env.plug.pose)), velocity_m_s=env.plug.velocity.tolist(),
                        angular_velocity_rad_s=env.plug.angular_velocity.tolist())
        before = sample()
        try:
            relative = self.base.pose.inv()*env.plug.pose
            self.retention = env._scene.create_drive(self.base, relative, env.plug, sapien.Pose())
            self.retention.lock_motion(True, True, True, True, True, True)
            policy.retention_record = dict(before=before, after=sample(), success_observation=obs,
                actual_relative_pose=pose_dict(matrix(relative), frame=SOCKET_NAME), mechanism='explicit_simulation_socket_hold')
            policy.retention_active = True
            policy.transition('retained')
            env.grasp_monitor.socket_supported(env._grasp_time)
            return True
        except Exception as exc:
            if self.retention is not None:
                env._scene.remove_drive(self.retention)
                self.retention = None
            policy.stop('retention_failed', str(exc))
            return False

    def close(self):
        for shape, bit in self.installation_filters:
            groups = list(shape.get_collision_groups())
            groups[2] &= ~bit
            shape.set_collision_groups(*groups)
        self.installation_filters.clear()
        for name in ('retention', 'base_support'):
            drive = getattr(self, name)
            if drive is not None:
                self.env._scene.remove_drive(drive)
                setattr(self, name, None)
        if self.base is not None:
            self.env.fixtures.pop(SOCKET_NAME, None)
            self.env._scene.remove_actor(self.base)
            self.base = None
        if self.cache is not None:
            self.cache.cleanup()
        self.policy.reset()
        self.wrench = dict(available=False, reason='reset')
