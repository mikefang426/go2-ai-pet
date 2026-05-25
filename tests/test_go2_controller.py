from __future__ import annotations

import unittest
from unittest.mock import patch

import robot.go2_controller as go2_controller
from robot.go2_controller import Go2Controller


class FakeSportClient:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def SetTimeout(self, timeout: float) -> None:
        self.calls.append(f"SetTimeout:{timeout}")

    def Init(self) -> None:
        self.calls.append("Init")

    def StopMove(self) -> None:
        self.calls.append("StopMove")

    def Sit(self) -> None:
        self.calls.append("Sit")

    def Hello(self) -> None:
        self.calls.append("Hello")

    def StandUp(self) -> None:
        self.calls.append("StandUp")

    def Move(self, vx: float, vy: float, yaw: float) -> None:
        self.calls.append(f"Move:{vx}:{vy}:{yaw}")


class Go2ControllerTests(unittest.TestCase):
    def test_sit_prefers_official_sit_over_stand_down(self) -> None:
        with patch.object(go2_controller, "SportClient", FakeSportClient):
            controller = Go2Controller()

        controller.sit()

        self.assertIn("Sit", controller.client.calls)

    def test_greet_uses_high_level_hello_preset(self) -> None:
        with (
            patch.object(go2_controller, "SportClient", FakeSportClient),
            patch("robot.go2_controller.time.sleep", return_value=None),
        ):
            controller = Go2Controller()
            controller.greet()

        self.assertEqual(
            controller.client.calls,
            [
                "SetTimeout:10.0",
                "Init",
                "StopMove",
                "Hello",
                "StopMove",
            ],
        )


if __name__ == "__main__":
    unittest.main()
