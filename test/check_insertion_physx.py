#!/usr/bin/env python3
"""Native PhysX geometry/sensor probe. No robot: NOT full MTC acceptance."""
import argparse,json,tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from transforms3d.quaternions import mat2quat
from dual_fr3_maniskill.sapien_compat import sapien
from dual_fr3_maniskill.insertion_scene import SocketInsertion
from dual_fr3_maniskill.insertion_geometry import HOLE,USB_IN_SOCKET,TIP,usb_parts
from ament_index_python.packages import get_package_share_directory


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    engine=sapien.Engine(); scene=engine.create_scene(); scene.set_timestep(.001)
    builder=scene.create_actor_builder(); frame=builder.build_static('frame')
    env=SimpleNamespace(_scene=scene,_renderer=None,fixtures={'trunking':frame},plug=None,
        agent=SimpleNamespace(links={'left_fr3_link0':frame}),
        cable_config={'insertion':{'xy_m':[0.,0.],'height_m':0.}})
    socket=SocketInsertion(env)
    root=Path(get_package_share_directory('dual_fr3_maniskill'))/'meshes'
    result=dict(scope='native PhysX component probe, no robot, no MTC',checks={},samples=[])
    with tempfile.TemporaryDirectory() as tmp:
        builder=scene.create_actor_builder()
        for i,part in enumerate(usb_parts(root/'USB1.stl')):
            path=Path(tmp)/f'usb{i}.stl'; part.export(path)
            builder.add_collision_from_file(str(path),material=scene.create_physical_material(.5,.5,0.))
        builder.set_mass_and_inertia(.015,sapien.Pose(),[1e-6]*3)
        plug=env.plug=builder.build('probe_usb')
        from dual_fr3_maniskill._rope_physx import disable_gravity
        disable_gravity(plug._ptr)
        for shape in plug.get_collision_shapes(): shape.contact_offset=.0001; shape.rest_offset=0.
        def sample(depth,offset=(0,0,0),push=None):
            # Explicit reset between independent component probes only.
            plug.set_pose(sapien.Pose(HOLE+[-depth,0,0]-USB_IN_SOCKET@TIP+offset,mat2quat(USB_IN_SOCKET)))
            plug.set_velocity([0,0,0]); plug.set_angular_velocity([0,0,0])
            socket.begin_window()
            for _ in range(150):
                if push is not None: plug.add_force_at_point(push,plug.pose.p)
                scene.step(); socket.sample(.001)
            value=socket._sum/socket._duration
            record=dict(depth=depth,offset=offset,push=push,force=value[:3].tolist(),
                torque=value[3:].tolist(),normal=(socket.normal_wrench/socket._duration).tolist(),
                reasons=list(socket._reasons),final_pose=plug.pose.p.tolist(),
                shape_types=[type(s.geometry).__name__ for s in socket.base.get_collision_shapes()])
            result['samples'].append(record); return record
        aligned=sample(.010)
        assert np.linalg.norm(aligned['force'])<1e-6,aligned
        result['checks']['actual_imported_cavity_open_at_target']=True
        bottom=sample(.0109,push=[-1.,0,0])
        assert bottom['force'][0]>.5 and not bottom['reasons'],bottom
        result['checks']['back_wall_resistance_sign']=True
        lateral=sample(.005,offset=[0,.0003,0],push=[-.2,1.,0.])
        assert lateral['force'][1]<-.5 and lateral['force'][0]>.05 and not lateral['reasons'],lateral
        result['checks']['full_reaction_includes_axial_friction']=True
        # Moment about the hole centre should balance force applied near USB origin.
        expected = np.cross(np.asarray(lateral['final_pose'])-HOLE, -np.asarray(lateral['push']))
        assert np.linalg.norm(np.asarray(lateral['torque'])-expected) < .001, (lateral, expected)
        result['checks']['hole_reference_torque']=True
        other_builder=scene.create_actor_builder()
        other_builder.add_box_collision(half_size=[.005]*3)
        other=other_builder.build('unexpected_base_load')
        other.set_pose(sapien.Pose([-.04,.015,.0937]))
        socket.begin_window()
        for _ in range(100): scene.step(); socket.sample(.001)
        assert any('base_contact_from_other_actor' in reason for reason in socket._reasons)
        result['checks']['other_actor_load_invalidates_sensor']=True
        result['checks']['37_convex_shapes_no_global_hull']=len(socket.base.get_collision_shapes())==37
        socket.close()
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['checks']))

if __name__=='__main__': main()
