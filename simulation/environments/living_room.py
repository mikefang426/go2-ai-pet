def _add_box(p, half_extents, position, rgba, mass=0.0):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=position)


def load(p, pybullet_data):
    body_ids = []

    # Coffee table
    body_ids.append(p.loadURDF("table/table.urdf", [1.2, 0.0, 0.0], useFixedBase=True))

    # Couch block
    body_ids.append(_add_box(p, [0.9, 0.35, 0.35], [-1.2, 0.0, 0.35], [0.42, 0.36, 0.31, 1.0]))

    # TV stand block
    body_ids.append(_add_box(p, [0.6, 0.2, 0.25], [0.0, -1.6, 0.25], [0.18, 0.18, 0.2, 1.0]))

    # Side table
    body_ids.append(_add_box(p, [0.25, 0.25, 0.25], [-0.8, 1.0, 0.25], [0.30, 0.24, 0.18, 1.0]))

    return body_ids
