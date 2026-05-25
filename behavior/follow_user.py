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
        context = context or BehaviorContext()

        max_v = max(0.0, _env_float("FOLLOW_MAX_V", 0.35))
        vx = max(0.0, min(max_v, _env_float("FOLLOW_VX", 0.26)))
        command_interval = max(0.02, _env_float("FOLLOW_COMMAND_INTERVAL", 0.1))

        try:
            controller.stand()
            while not context.interrupted:
                controller.walk(vx, 0.0, 0.0)
                if context.wait(command_interval):
                    break
        finally:
            controller.stop()
