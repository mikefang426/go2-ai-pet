from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Iterable

from interfaces.behavior import BehaviorContext
from interfaces.robot_interface import RobotInterface


@dataclass(frozen=True)
class PatrolStep:
    vx: float
    vy: float
    yaw: float
    duration: float


@dataclass(frozen=True)
class PatrolConfig:
    command_interval: float = 0.1
    loops: int | None = None
    stop_on_complete: bool = True


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


def _env_loops(name: str, default: int | None) -> int | None:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        loops = int(raw)
    except ValueError:
        return default
    return None if loops <= 0 else loops


def _default_route_from_env() -> tuple[PatrolStep, ...]:
    # A plain perimeter patrol: walk one edge, stop, turn in place, then repeat.
    # Keeping scan/turn motions out of the default route makes the intent obvious in simulation.
    vx = _env_float("PATROL_VX", 0.24)
    turn_yaw = _env_float("PATROL_YAW", 0.72)
    straight_sec = _env_float("PATROL_STRAIGHT_SEC", 3.2)
    turn_sec = _env_float("PATROL_TURN_SEC", 1.9)
    pause_sec = _env_float("PATROL_PAUSE_SEC", 0.35)

    return (
        PatrolStep(vx=vx, vy=0.0, yaw=0.0, duration=straight_sec),
        PatrolStep(vx=0.0, vy=0.0, yaw=0.0, duration=pause_sec),
        PatrolStep(vx=0.0, vy=0.0, yaw=turn_yaw, duration=turn_sec),
        PatrolStep(vx=0.0, vy=0.0, yaw=0.0, duration=pause_sec),
    )


class Patrol:
    def __init__(
        self,
        route: Iterable[PatrolStep] | None = None,
        config: PatrolConfig | None = None,
    ) -> None:
        self.route = tuple(route) if route is not None else _default_route_from_env()
        if not self.route:
            raise ValueError("Patrol requires at least one route step")

        self.config = config or PatrolConfig(
            command_interval=max(0.02, _env_float("PATROL_STEP_SEC", 0.1)),
            loops=_env_loops("PATROL_LOOPS", None),
        )

    def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
        context = context or BehaviorContext()
        try:
            controller.stand()
            loops_completed = 0
            while not context.interrupted:
                for step in self.route:
                    if context.interrupted:
                        break
                    self._run_step(controller, step, context)

                loops_completed += 1
                if self.config.loops is not None and loops_completed >= self.config.loops:
                    break
        finally:
            if self.config.stop_on_complete:
                controller.stop()

    def _run_step(self, controller: RobotInterface, step: PatrolStep, context: BehaviorContext) -> None:
        duration = max(0.0, step.duration)
        deadline = time.monotonic() + duration

        while not context.interrupted:
            controller.walk(step.vx, step.vy, step.yaw)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            wait_for = min(max(0.02, self.config.command_interval), remaining)
            if context.wait(wait_for):
                break
