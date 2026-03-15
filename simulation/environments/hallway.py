def _add_box(p, half_extents, position, rgba):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=position)


def load(p, pybullet_data):
    body_ids = []

    hallway_len = 9.0
    wall_thickness = 0.08
    wall_height = 0.7
    hallway_half_width = 1.1

    # Left and right walls
    body_ids.append(
        _add_box(
            p,
            [hallway_len / 2.0, wall_thickness, wall_height],
            [0.0, hallway_half_width + wall_thickness, wall_height],
            [0.82, 0.82, 0.82, 1.0],
        )
    )
    body_ids.append(
        _add_box(
            p,
            [hallway_len / 2.0, wall_thickness, wall_height],
            [0.0, -hallway_half_width - wall_thickness, wall_height],
            [0.82, 0.82, 0.82, 1.0],
        )
    )

    # Back wall
    body_ids.append(
        _add_box(
            p,
            [wall_thickness, hallway_half_width + wall_thickness, wall_height],
            [-hallway_len / 2.0 - wall_thickness, 0.0, wall_height],
            [0.78, 0.78, 0.78, 1.0],
        )
    )

    # Simple door frame near the far end
    x_frame = hallway_len / 2.0 - 1.0
    body_ids.append(_add_box(p, [0.08, 0.25, 0.65], [x_frame, 0.85, 0.65], [0.6, 0.6, 0.62, 1.0]))
    body_ids.append(_add_box(p, [0.08, 0.25, 0.65], [x_frame, -0.85, 0.65], [0.6, 0.6, 0.62, 1.0]))
    body_ids.append(_add_box(p, [0.08, 0.9, 0.08], [x_frame, 0.0, 1.22], [0.6, 0.6, 0.62, 1.0]))

    return body_ids
