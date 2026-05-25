from __future__ import annotations

import threading
import time
import unittest
from unittest.mock import patch

from behavior.follow_user import FollowUser
from interfaces.robot_interface import RobotInterface
from planner.behavior_runner import BehaviorRunner


class FakeController(RobotInterface):
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.commands: list[tuple[str, float, float, float]] = []

    def sit(self) -> None:
        with self._lock:
            self.commands.append(("sit", 0.0, 0.0, 0.0))

    def stand(self) -> None:
        with self._lock:
            self.commands.append(("stand", 0.0, 0.0, 0.0))

    def walk(self, vx: float, vy: float, yaw: float) -> None:
        with self._lock:
            self.commands.append(("walk", vx, vy, yaw))

    def snapshot(self) -> list[tuple[str, float, float, float]]:
        with self._lock:
            return list(self.commands)


def wait_until(predicate, timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


class FollowUserTests(unittest.TestCase):
    def test_follow_runs_until_interrupted_and_stops(self) -> None:
        controller = FakeController()
        runner = BehaviorRunner(controller, stop_timeout=0.5)

        with patch.dict(
            "os.environ",
            {
                "FOLLOW_VX": "2.0",
                "FOLLOW_MAX_V": "0.30",
                "FOLLOW_COMMAND_INTERVAL": "0.01",
            },
        ):
            self.assertTrue(runner.start("follow_user", FollowUser()))
            self.assertTrue(
                wait_until(
                    lambda: ("walk", 0.30, 0.0, 0.0) in controller.snapshot()
                )
            )
            self.assertTrue(runner.stop())

        commands = controller.snapshot()
        self.assertEqual(commands[0], ("stand", 0.0, 0.0, 0.0))
        self.assertIn(("walk", 0.30, 0.0, 0.0), commands)
        self.assertEqual(commands[-1], ("walk", 0.0, 0.0, 0.0))
        self.assertFalse(runner.is_running)


if __name__ == "__main__":
    unittest.main()
