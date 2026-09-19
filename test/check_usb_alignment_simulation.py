#!/usr/bin/env python3
"""Real contact-grasp servo with native MoveIt checks (no ROS transport/MTC).

Only the initial robot pose is set directly, before the USB exists. Collision
requests use the production message path and real FCL; configurable delayed
replies exercise entry/per-step gating. This is USB-only validation.
"""
import argparse
from concurrent.futures import Future
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as NS

import numpy as np
from transforms3d.axangles import axangle2mat
from transforms3d.quaternions import mat2quat

from dual_fr3_maniskill.cable.model import load_config, USB_LINK
from dual_fr3_maniskill.cable.planning_scene import attached_usb_scene
from dual_fr3_maniskill.cable.threading import task_usb_mount
from dual_fr3_maniskill.engine.sapien_compat import sapien
from dual_fr3_maniskill.robot.assets import prepare_assets
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
from dual_fr3_maniskill.usb.bridge import InsertionBridge
from dual_fr3_maniskill.usb.geometry import HOLE, TIP, USB_IN_SOCKET, matrix, pose_dict
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description
from dual_fr3_trunking_mtc.insertion_task.planning_scene import socket_collision_object


class NativeValidation:
    def __init__(self, directory, description, semantic, socket, config_path, delay_ticks=1):
        from moveit.core.robot_model import RobotModel
        from moveit.core.planning_scene import PlanningScene
        urdf, srdf = directory/'collision.urdf', directory/'collision.srdf'
        urdf.write_text(description); srdf.write_text(semantic)
        self.model = RobotModel(str(urdf), str(srdf))
        self.scene = PlanningScene(self.model)
        self.scene.apply_collision_object(socket_collision_object(socket.config,
            dict(socket_world_pose=pose_dict(matrix(socket.base.pose)))))
        for fixture in ('plate', 'trunking'):
            self.scene.allowed_collision_matrix.set_entry('usb_socket', fixture, True)
        self.scene.process_attached_collision_object(attached_usb_scene(config_path).robot_state.attached_collision_objects[0])
        self.pending, self.checked = [], 0
        self.tick, self.delay_ticks = 0, delay_ticks

    def create_client(self, kind, name):
        def submit(request):
            future = Future()
            self.pending.append((self.tick + self.delay_ticks, name, request, future))
            return future
        return NS(service_is_ready=lambda: True, call_async=submit)

    def resolve(self):
        from moveit.core.collision_detection import CollisionRequest, CollisionResult
        self.tick += 1
        waiting = []
        for ready_tick, name, request, future in self.pending:
            if future.cancelled():
                continue
            if self.tick < ready_tick:
                waiting.append((ready_tick, name, request, future))
                continue
            if name == '/get_planning_scene':
                result = NS(scene=self.scene.planning_scene_message)
            else:
                self.scene.set_current_state(request.robot_state)
                req, res = CollisionRequest(), CollisionResult()
                req.contacts, req.max_contacts = True, 10000
                self.scene.check_collision(req, res, self.scene.current_state)
                result = NS(valid=not res.collision, constraint_result=[], contacts=[
                    NS(contact_body_1=a, contact_body_2=b) for a, b in res.contacts])
                self.checked += 1
            future.set_result(result)
        self.pending = waiting


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--lateral-error', type=float, default=.0008)
    parser.add_argument('--angle-error', type=float, default=.02)
    parser.add_argument('--local-collision-check', choices=('entry', 'per_step'))
    parser.add_argument('--validation-delay-ticks', type=int, default=1)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.validation_delay_ticks < 1:
        parser.error('--validation-delay-ticks must be positive')
    path = resolve_cable_config(scene='trunking_cable')
    config = load_config(path, solver=None)
    config['insertion']['enabled'] = True
    if args.local_collision_check:
        config['insertion']['local_collision_check'] = args.local_collision_check
    description, semantic = build_maniskill_description(scene='trunking_cable', cable_config=path)
    summary = dict(scope=__doc__, passed=False, validation_delay_ticks=args.validation_delay_ticks)
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        assets = prepare_assets(description, semantic, directory)
        sim = TrunkingCableSimulation(assets, cable_config=config, cable_solver='rope_actor',
                                     load_cable=False, control_freq=50, sim_freq=500)
        try:
            env, robot = sim.env, sim.env.agent.robot
            socket = env.insertion
            tcp = env.agent.links['left_fr3_hand_tcp']
            indices = [sim.indices[f'left_fr3_joint{i}'] for i in range(1, 8)]
            mask = np.zeros(len(sim.names), dtype=int); mask[indices] = 1
            mount = np.eye(4); mount[:3, 3], mount[:3, :3] = task_usb_mount(config)
            plug = np.eye(4)
            plug[:3, :3] = axangle2mat([0, 0, 1], args.angle_error) @ USB_IN_SOCKET
            plug[:3, 3] = HOLE + [.008, args.lateral_error, 0.] - plug[:3, :3] @ TIP
            initial = matrix(socket.base.pose) @ plug @ np.linalg.inv(mount)
            model = robot.create_pinocchio_model()
            q, ok, error = model.compute_inverse_kinematics(robot.get_links().index(tcp),
                sapien.Pose(initial[:3, 3], mat2quat(initial[:3, :3])), initial_qpos=sim.target,
                active_qmask=mask, eps=1e-7, max_iterations=1000)
            if not ok:
                raise RuntimeError('Initial IK: ' + str(error))
            robot.set_qpos(q); sim.target[:] = q  # test initial condition, USB absent
            for _ in range(20): sim.step()
            env.spawn_cable()
            env.grasp_monitor.begin_closing(env._grasp_time)
            sim.set_gripper_force('left', 10.)
            finger = sim.indices['left_fr3_finger_joint1']
            for _ in range(100):
                sim.target[finger] = max(0., sim.target[finger]-.0008)
                sim.step()
            env.release_support()
            for _ in range(30): sim.step()
            assert env.grasp_monitor.snapshot()['stable'], env.grasp_monitor.snapshot()
            checker = NativeValidation(directory, description, semantic, socket, path, args.validation_delay_ticks)
            bridge = NS(sim=sim, dt=.02, assets=assets, cable_config=config,
                arm_names={side: [f'{side}_fr3_joint{i}' for i in range(1, 8)] for side in ('left', 'right')},
                arm_indices={side: [sim.indices[f'{side}_fr3_joint{i}'] for i in range(1, 8)] for side in ('left', 'right')},
                desired={}, reserved=set(), arms={}, pending_arms={}, grippers={}, failure=None,
                create_client=checker.create_client, get_parameter=lambda name: NS(value=path),
                create_publisher=lambda *a: None, create_service=lambda *a: None)
            control = InsertionBridge(bridge)
            policy = socket.policy
            policy.right_return_complete = True  # right starts open/ready in this local test
            policy.transition('approach')
            summary['initial'] = socket.observe()

            def run(operation):
                response = control.command(operation, NS())
                if not response.success:
                    raise RuntimeError(response.message)
                while control.owner:
                    control.last_heartbeat = time.monotonic()
                    checker.resolve()
                    control.before_tick()
                    sim.step()

            run('align')
            summary['alignment'] = policy.snapshot()
            summary['collision_states_at_alignment'] = checker.checked
            if policy.state != 'aligned':
                raise RuntimeError(policy.state + ': ' + policy.reason)
            run('start')
            summary['result'] = policy.snapshot()
            summary['collision_states_checked'] = checker.checked
            summary['no_tcp_or_world_support'] = env.support_drive is None and env.mount_drive is None
            summary['passed'] = policy.insertion_success and policy.retention_active
            if policy.local_collision_check == 'entry':
                assert checker.checked == summary['collision_states_at_alignment']
        except Exception as exc:
            summary['passed'] = False
            summary['error'] = str(exc)
            summary['result'] = sim.env.insertion.policy.snapshot()
        finally:
            sim.close()
            args.output.write_text(json.dumps(summary, indent=2) + '\n')
            print(json.dumps({k: v for k, v in summary.items() if k not in ('initial', 'alignment', 'result', 'scope')}))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
