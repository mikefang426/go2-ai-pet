def _add_box(p, half_extents, position, rgba):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=position)


def _add_cylinder(p, radius, height, position, rgba):
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=rgba)
    return p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=position)


def load(p, pybullet_data):
    body_ids = []

    # Low bars / blocks
    body_ids.append(_add_box(p, [0.35, 0.35, 0.08], [1.1, 0.0, 0.08], [0.85, 0.35, 0.35, 1.0]))
    body_ids.append(_add_box(p, [0.3, 0.3, 0.12], [1.9, -0.55, 0.12], [0.35, 0.55, 0.85, 1.0]))
    body_ids.append(_add_box(p, [0.25, 0.25, 0.16], [2.5, 0.55, 0.16], [0.35, 0.75, 0.45, 1.0]))

    # Slalom poles
    body_ids.append(_add_cylinder(p, 0.08, 0.7, [3.0, -0.7, 0.35], [0.95, 0.8, 0.2, 1.0]))
    body_ids.append(_add_cylinder(p, 0.08, 0.7, [3.4, 0.0, 0.35], [0.95, 0.8, 0.2, 1.0]))
    body_ids.append(_add_cylinder(p, 0.08, 0.7, [3.8, 0.7, 0.35], [0.95, 0.8, 0.2, 1.0]))

    # End gate
    body_ids.append(_add_box(p, [0.08, 0.75, 0.45], [4.5, 0.0, 0.45], [0.7, 0.7, 0.75, 1.0]))

    return body_ids
