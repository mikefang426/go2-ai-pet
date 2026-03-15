from __future__ import annotations

import importlib
from typing import Any

SUPPORTED_ENVIRONMENTS = (
    "empty_room",
    "living_room",
    "hallway",
    "stairs",
    "obstacle_course",
)


def _module_path(environment: str) -> str:
    if __package__:
        return f"{__package__}.environments.{environment}"
    return f"environments.{environment}"


def load_environment(environment: str, p: Any, pybullet_data: Any) -> list[int]:
    name = (environment or "empty_room").strip().lower()
    if not name:
        name = "empty_room"

    if name not in SUPPORTED_ENVIRONMENTS:
        valid = ", ".join(SUPPORTED_ENVIRONMENTS)
        raise ValueError(f"Unknown environment '{name}'. Valid options: {valid}")

    module = importlib.import_module(_module_path(name))
    loader = getattr(module, "load", None)
    if not callable(loader):
        raise TypeError(f"Environment module '{name}' does not define a callable load(p, pybullet_data)")

    body_ids = loader(p, pybullet_data)
    return body_ids or []
