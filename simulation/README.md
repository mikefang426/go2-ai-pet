# Simulation Module

The `simulation/` folder contains the PyBullet-based robot simulator and the
adapter used by the shared robot interface layer.

## Layout

```text
simulation/
├── __init__.py
├── README.md
├── quad_sim.py
├── sim_controller.py
├── environments/
│   ├── __init__.py
│   ├── empty_room.py
│   ├── hallway.py
│   ├── living_room.py
│   ├── obstacle_course.py
│   └── stairs.py
└── assets/
    └── README.md
```

## File Roles

- `quad_sim.py`
  Runs the interactive PyBullet simulation with keyboard control.
- `sim_controller.py`
  Provides the simulation-side `RobotInterface` implementation and environment
  loading utilities.
- `environments/*.py`
  Defines scene builders for supported simulation maps.
- `assets/`
  Reserved for simulation-specific meshes, textures, URDFs, and other custom
  resources as the project grows.

## Notes

- `hallway.py` is kept as an extra supported environment even though your core
  target layout focuses on `empty_room`, `stairs`, `living_room`, and
  `obstacle_course`.
- If custom assets are added later, prefer referencing them from `assets/`
  instead of mixing them into the root of `simulation/`.
