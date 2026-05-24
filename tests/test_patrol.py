from __future__ import annotations

import threading
import time
import unittest

from behavior.patrol import Patrol, PatrolConfig, PatrolStep
from interfaces.behavior import BehaviorContext
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


class PatrolTests(unittest.TestCase):
    def test_default_patrol_route_walks_stops_and_turns_in_place(self) -> None:
        patrol = Patrol()

        self.assertEqual(len(patrol.route), 4)
        self.assertGreater(patrol.route[0].vx, 0.0)
        self.assertEqual(patrol.route[0].yaw, 0.0)
        self.assertEqual(patrol.route[1], PatrolStep(vx=0.0, vy=0.0, yaw=0.0, duration=0.35))
        self.assertGreater(patrol.route[2].yaw, 0.0)
        self.assertEqual(patrol.route[2].vx, 0.0)
        self.assertEqual(patrol.route[3], PatrolStep(vx=0.0, vy=0.0, yaw=0.0, duration=0.35))

    def test_runner_starts_patrol_asynchronously_and_stops_on_interrupt(self) -> None:
        controller = FakeController()
        patrol = Patrol(
            route=(PatrolStep(vx=0.12, vy=0.0, yaw=0.0, duration=0.5),),
            config=PatrolConfig(command_interval=0.01),
        )
        runner = BehaviorRunner(controller, stop_timeout=0.5)

        start = time.monotonic()
        started = runner.start("patrol", patrol)
        elapsed = time.monotonic() - start

        self.assertTrue(started)
        self.assertLess(elapsed, 0.1)
        self.assertTrue(
            wait_until(
                lambda: any(cmd == ("walk", 0.12, 0.0, 0.0) for cmd in controller.snapshot())
            )
        )

        stopped = runner.stop()

        self.assertTrue(stopped)
        self.assertFalse(runner.is_running)
        self.assertEqual(controller.snapshot()[-1], ("walk", 0.0, 0.0, 0.0))

    def test_finite_patrol_stops_when_route_completes(self) -> None:
        controller = FakeController()
        patrol = Patrol(
            route=(
                PatrolStep(vx=0.10, vy=0.0, yaw=0.0, duration=0.02),
                PatrolStep(vx=0.0, vy=0.0, yaw=0.35, duration=0.02),
            ),
            config=PatrolConfig(command_interval=0.005, loops=1),
        )
        runner = BehaviorRunner(controller)

        self.assertTrue(runner.start("patrol", patrol))

        self.assertTrue(wait_until(lambda: not runner.is_running))
        commands = controller.snapshot()
        self.assertEqual(commands[0], ("stand", 0.0, 0.0, 0.0))
        self.assertIn(("walk", 0.0, 0.0, 0.35), commands)
        self.assertEqual(commands[-1], ("walk", 0.0, 0.0, 0.0))

    def test_runner_refuses_new_behavior_until_active_behavior_stops(self) -> None:
        class BlockingBehavior:
            def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
                time.sleep(0.2)

        controller = FakeController()
        runner = BehaviorRunner(controller, stop_timeout=0.01)

        self.assertTrue(runner.start("blocking", BlockingBehavior()))
        self.assertFalse(runner.start("patrol", Patrol(config=PatrolConfig(loops=1))))
        self.assertEqual(runner.active_name, "blocking")

        self.assertTrue(wait_until(lambda: not runner.is_running, timeout=0.5))


if __name__ == "__main__":
    unittest.main()
