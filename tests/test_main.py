from __future__ import annotations

import sys
import unittest
from io import StringIO
from unittest.mock import patch

import main as app_main
from interfaces.robot_interface import RobotInterface


class FakeController(RobotInterface):
    def __init__(self) -> None:
        self.commands: list[tuple[str, float, float, float]] = []

    def sit(self) -> None:
        self.commands.append(("sit", 0.0, 0.0, 0.0))

    def stand(self) -> None:
        self.commands.append(("stand", 0.0, 0.0, 0.0))

    def walk(self, vx: float, vy: float, yaw: float) -> None:
        self.commands.append(("walk", vx, vy, yaw))


class MainTests(unittest.TestCase):
    def test_ctrl_c_exits_without_raising(self) -> None:
        controller = FakeController()

        with (
            patch.object(sys, "argv", ["main.py"]),
            patch.object(sys, "stdout", StringIO()),
            patch.object(app_main, "build_robot", return_value=controller),
            patch("builtins.input", side_effect=KeyboardInterrupt),
        ):
            app_main.main()

        self.assertEqual(controller.commands[-1], ("walk", 0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
