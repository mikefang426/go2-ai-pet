def _add_box(p, half_extents, position, rgba):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=position)


def load(p, pybullet_data):
    body_ids = []

    # Build a staircase in front of the spawn.
    step_height = 0.045
    step_depth = 0.36
    step_width = 1.3
    step_count = 8
    x0 = 0.7

    for i in range(step_count):
        z = (i + 1) * step_height / 2.0
        x = x0 + i * step_depth
        body_ids.append(
            _add_box(
                p,
                [step_depth / 2.0, step_width / 2.0, z],
                [x, 0.0, z],
                [0.72, 0.72, 0.72, 1.0],
            )
        )

    # Landing platform
    top_height = step_count * step_height
    landing_depth = 1.6
    landing_x = x0 + step_count * step_depth + landing_depth / 2.0
    body_ids.append(
        _add_box(
            p,
            [landing_depth / 2.0, step_width / 2.0, top_height / 2.0],
            [landing_x, 0.0, top_height / 2.0],
            [0.65, 0.65, 0.65, 1.0],
        )
    )

    # Low side curbs reduce lateral falls without blocking climbing.
    curb_half_height = 0.05
    curb_half_width = 0.03
    path_half_width = step_width / 2.0
    curb_half_len = (step_count * step_depth + landing_depth) / 2.0
    curb_center_x = x0 + (step_count * step_depth) / 2.0 + landing_depth / 2.0
    curb_z = curb_half_height
    body_ids.append(
        _add_box(
            p,
            [curb_half_len, curb_half_width, curb_half_height],
            [curb_center_x, path_half_width + curb_half_width, curb_z],
            [0.58, 0.58, 0.6, 1.0],
        )
    )
    body_ids.append(
        _add_box(
            p,
            [curb_half_len, curb_half_width, curb_half_height],
            [curb_center_x, -path_half_width - curb_half_width, curb_z],
            [0.58, 0.58, 0.6, 1.0],
        )
    )

    return body_ids
