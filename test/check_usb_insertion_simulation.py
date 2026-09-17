#!/usr/bin/env python3
"""Real robot contact-grasp/local-insertion test, initialized near the socket.

This isolates the terminal physics; it is NOT evidence of full MTC/return.
Robot qpos is set only to define the test's initial condition before USB exists.
"""
import argparse,json,tempfile,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from transforms3d.quaternions import mat2quat
from dual_fr3_maniskill.assets import prepare_assets
from dual_fr3_maniskill.cable.model import load_config
from dual_fr3_maniskill.cable.threading import task_usb_mount
from dual_fr3_maniskill.scenes import resolve_cable_config
from dual_fr3_maniskill.insertion_geometry import matrix, HOLE, TIP, USB_IN_SOCKET
from dual_fr3_moveit_config.maniskill_resources import build_maniskill_description


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--grip-setback',type=float,default=0.)
    parser.add_argument('--lateral-error',type=float,default=0.)
    parser.add_argument('--obstacle-depth',type=float,default=0.)
    args=parser.parse_args()
    path=resolve_cable_config(scene='trunking_cable')
    config=load_config(path,solver=None);config['insertion']['enabled']=True
    config['usb']['grip_center']=[0.,-args.grip_setback,0.]
    description,semantic=build_maniskill_description(scene='trunking_cable',cable_config=path)
    from dual_fr3_maniskill.scenes.trunking_cable import TrunkingCableSimulation
    from dual_fr3_maniskill.sapien_compat import sapien
    from dual_fr3_maniskill.forces import ForceCollector
    from dual_fr3_maniskill.insertion_bridge import InsertionBridge
    if args.obstacle_depth:
        # Add a REAL box to the instrumented socket, shortening its free depth.
        # This is a negative physics test, not substituted sensor observations.
        import trimesh
        import dual_fr3_maniskill.insertion_scene as adapter
        original_parts = adapter.socket_parts
        def obstructed(*values, **kwargs):
            parts = original_parts(*values, **kwargs)
            obstacle = trimesh.creation.box(extents=[.001,.01,.016])
            obstacle.apply_translation(HOLE+[-args.obstacle_depth-.0005,0,0])
            return parts+[obstacle]
        adapter.socket_parts = obstructed
    with tempfile.TemporaryDirectory() as directory:
        assets=prepare_assets(description,semantic,Path(directory))
        sim=TrunkingCableSimulation(assets,cable_config=config,cable_solver='rope_actor',load_cable=False,
                                   control_freq=50,sim_freq=500)
        env=sim.env;socket=env.insertion;robot=env.agent.robot
        model=robot.create_pinocchio_model();link=env.agent.links['left_fr3_hand_tcp']
        indices=[sim.indices[f'left_fr3_joint{i}'] for i in range(1,8)]
        mask=np.zeros(len(sim.names),dtype=int);mask[indices]=1
        mount=np.eye(4);mount[:3,3],mount[:3,:3]=task_usb_mount(config)
        plug_target=np.eye(4);plug_target[:3,:3]=USB_IN_SOCKET
        plug_target[:3,3]=HOLE+[.008,0,0]-USB_IN_SOCKET@TIP
        target=matrix(socket.base.pose)@plug_target@np.linalg.inv(mount)
        q,ok,error=model.compute_inverse_kinematics(robot.get_links().index(link),sapien.Pose(target[:3,3],mat2quat(target[:3,:3])),
            initial_qpos=sim.target,active_qmask=mask,eps=1e-6,max_iterations=1000)
        if not ok:raise RuntimeError('Initial IK failed: '+str(error))
        robot.set_qpos(q);sim.target[:]=q  # INITIAL CONDITION ONLY, USB absent
        collector=ForceCollector(env,['usb_socket']);env.force_collector=collector
        output=args.output.open('x',buffering=1)
        summary=dict(scope='full robot local insertion from prepared initial pose; no MTC return',
                     grip_setback_m=args.grip_setback,lateral_error_m=args.lateral_error,obstacle_depth_m=args.obstacle_depth,passed=False)
        phase='settle_initial'
        def step():
            sim.step()
            output.write(json.dumps(dict(collector.snapshot,validation_phase=phase),allow_nan=False)+'\n')
        try:
            for _ in range(20):step()
            env.spawn_cable();env.grasp_monitor.begin_closing(env._grasp_time);sim.set_gripper_force('left',10.)
            phase='closing'
            finger=sim.indices['left_fr3_finger_joint1']
            for _ in range(100):
                sim.target[finger]=max(0.,sim.target[finger]-.04/sim.control_freq);step()
            assert env.grasp_monitor.ready_to_release,env.grasp_monitor.snapshot()
            env.release_support();phase='verify_grasp'
            for _ in range(30):step()
            assert env.grasp_monitor.snapshot()['stable'],env.grasp_monitor.snapshot()
            phase='align_measured_grasp'
            for _ in range(2):
                raw=socket.target();pose=sapien.Pose(raw['position_m'],raw['quaternion_wxyz'])
                start=sim.target.copy()
                q,ok,error=model.compute_inverse_kinematics(robot.get_links().index(link),pose,
                    initial_qpos=start,active_qmask=mask,eps=1e-7,max_iterations=200)
                assert ok,error
                for i in range(100):sim.target[indices]=start[indices]+(q[indices]-start[indices])*(i+1)/100;step()
                for _ in range(25):step()
            summary['aligned_observation']=socket.observe()
            policy=socket.policy
            policy.begin(sim.time,socket.observe())
            bridge=SimpleNamespace(sim=sim,dt=1/sim.control_freq,arm_indices={'left':indices},
                arm_names={'left':[f'left_fr3_joint{i}' for i in range(1,8)]},assets=assets,reserved={('left','insertion')},
                desired={},failure=None)
            control=InsertionBridge.__new__(InsertionBridge);control.bridge=bridge;control.owner=True
            control.goal=link.pose;control.model=model;control.last_heartbeat=time.monotonic()
            if args.lateral_error:
                # Move the arm, not the USB, to introduce an actual alignment error.
                control.goal=sapien.Pose(control.goal.p+[0,args.lateral_error,0],control.goal.q)
            phase='feedback_insertion'
            while control.owner and sim.time-policy.started<50:
                control.last_heartbeat=time.monotonic();control.before_tick();step()
            summary['result']=policy.snapshot()
            if args.obstacle_depth:
                summary['passed'] = policy.state in ('blocked','overload','slip_or_drop') and not policy.insertion_success and not policy.retention_active
            summary['no_temporary_or_TCP_mount']=env.support_drive is None and env.mount_drive is None
            if policy.retention_active:
                held=env.plug.pose
                phase='open_after_retention'
                for _ in range(80):
                    for side in ('left','right'):
                        index=sim.indices[side+'_fr3_finger_joint1'];sim.target[index]=min(.04,sim.target[index]+.04/sim.control_freq)
                    step()
                summary['after_open_displacement_m']=float(np.linalg.norm(env.plug.pose.p-held.p))
                summary['after_open_grasp_state']=env.grasp_monitor.state
                summary['passed']=summary['after_open_displacement_m']<.0001 and env.grasp_monitor.state=='gripper_released'
        except Exception as exc:
            summary['error']=str(exc)
            summary['result']=socket.policy.snapshot()
        finally:
            output.close();sim.close()
            args.output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            print(json.dumps(summary,indent=2))
    return 0 if summary['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
