import os

from interfaces.behavior import BehaviorContext
from interfaces.robot_interface import RobotInterface


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


class FollowUser:
    def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
        if context and context.interrupted:
            controller.stop()
            return
        max_v = max(0.0, _env_float("FOLLOW_MAX_V", 0.35))
        vx = max(0.0, min(max_v, _env_float("FOLLOW_VX", 0.26)))
        controller.walk(vx, 0.0, 0.0)
