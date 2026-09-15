"""Cable-only copies of static fixtures with cable-scale contact shells."""


def create_fixture_proxy(scene, source, contact_offset):
    """Reuse the fixture's collision assets; leave robot contact shapes intact.

    The shared scene uses a 1 mm shell. In the captured 3 mm cable/slot-edge
    case that shell generated distant patches but missed the overlapping rim.
    A private collider lets the cable use its own shell without changing the
    geometry, materials, or contact settings seen by the robot.
    """
    if source.type != "static":
        raise ValueError("Cable fixture proxies require static actors")
    builder = scene.create_actor_builder()
    for record in source.get_builder().get_collisions():
        common = dict(pose=record.pose, material=record.material)
        if record.type == "Nonconvex":
            builder.add_nonconvex_collision_from_file(record.filename, scale=record.scale, **common)
        elif record.type == "Box":
            builder.add_box_collision(half_size=record.scale, **common)
        elif record.type == "Capsule":
            builder.add_capsule_collision(radius=record.radius, half_length=record.length, **common)
        elif record.type == "Sphere":
            builder.add_sphere_collision(radius=record.radius, **common)
        else:
            raise ValueError(f"Unsupported cable fixture collision: {record.type}")
    # Only rope segments (affinities 2 and 4) can collide with this actor.
    builder.set_collision_groups(6, 0, 0, 0)
    proxy = builder.build_static("rope_fixture_" + source.name)
    try:
        proxy.set_pose(source.pose)
        original, copied = source.get_collision_shapes(), proxy.get_collision_shapes()
        if len(original) != len(copied):
            raise RuntimeError("Cable fixture proxy did not preserve collision shapes")
        for shape, original_shape in zip(copied, original):
            shape.set_local_pose(original_shape.get_local_pose())
            shape.set_physical_material(original_shape.get_physical_material())
            shape.rest_offset = original_shape.rest_offset
            shape.contact_offset = contact_offset
        return proxy
    except Exception:
        scene.remove_actor(proxy)
        raise
