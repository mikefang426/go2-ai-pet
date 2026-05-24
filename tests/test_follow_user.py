from __future__ import annotations

import threading
import unittest
from unittest.mock import patch

from behavior.follow_user import FollowUser
from interfaces.robot_interface import RobotInterface


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


class FollowUserTests(unittest.TestCase):
    def test_clamps_configured_forward_speed_to_safe_limit(self) -> None:
        controller = FakeController()

        with patch.dict(
            "os.environ",
            {
                "FOLLOW_VX": "2.0",
                "FOLLOW_MAX_V": "0.30",
                "FOLLOW_COMMAND_INTERVAL": "0.01",
                "FOLLOW_LOOPS": "1",
            },
        ):
            FollowUser().run(controller)

        self.assertEqual(controller.snapshot(), [("walk", 0.30, 0.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
