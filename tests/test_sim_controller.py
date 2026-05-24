from __future__ import annotations

import json
import tempfile
import threading
import unittest
from pathlib import Path

from simulation.sim_controller import SimulationController, SimulationControllerConfig


class SimulationControllerTests(unittest.TestCase):
    def test_command_file_writes_are_thread_safe(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            command_file = Path(tmp_dir) / "command.json"
            controller = SimulationController(
                SimulationControllerConfig(
                    launch_simulator=False,
                    command_file=str(command_file),
                )
            )
            errors: list[Exception] = []

            def write_commands(index: int) -> None:
                try:
                    for _ in range(25):
                        controller.walk(0.1 * (index % 2), 0.0, 0.2)
                        controller.stop()
                except Exception as exc:
                    errors.append(exc)

            threads = [threading.Thread(target=write_commands, args=(i,)) for i in range(4)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()

            self.assertEqual(errors, [])
            with command_file.open("r", encoding="utf-8") as fh:
                payload = json.load(fh)
            self.assertEqual(payload["vx"], 0.0)
            self.assertEqual(payload["vy"], 0.0)
            self.assertEqual(payload["wz"], 0.0)

    def test_greet_writes_dedicated_sim_action(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            command_file = Path(tmp_dir) / "command.json"
            controller = SimulationController(
                SimulationControllerConfig(
                    launch_simulator=False,
                    command_file=str(command_file),
                )
            )

            controller.greet()

            with command_file.open("r", encoding="utf-8") as fh:
                payload = json.load(fh)
            self.assertEqual(payload["action"], "greet")
            self.assertEqual(payload["posture"], "stand")
            self.assertGreater(payload["action_id"], 0)

    def test_beg_writes_dedicated_sim_action(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            command_file = Path(tmp_dir) / "command.json"
            controller = SimulationController(
                SimulationControllerConfig(
                    launch_simulator=False,
                    command_file=str(command_file),
                )
            )

            controller.beg()

            with command_file.open("r", encoding="utf-8") as fh:
                payload = json.load(fh)
            self.assertEqual(payload["action"], "beg")
            self.assertEqual(payload["posture"], "stand")
            self.assertGreater(payload["action_id"], 0)


if __name__ == "__main__":
    unittest.main()
