from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


class QuadSimActionTests(unittest.TestCase):
    def test_actions_do_not_crash_quad_sim(self) -> None:
        try:
            import pybullet  # noqa: F401
        except ModuleNotFoundError:
            self.skipTest("pybullet is not installed")

        for action in ("greet",):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as tmp_dir:
                command_file = Path(tmp_dir) / "command.json"
                command_file.write_text(
                    json.dumps(
                        {
                            "vx": 0.0,
                            "vy": 0.0,
                            "wz": 0.0,
                            "posture": "stand",
                            "action": action,
                            "action_id": 1,
                        }
                    ),
                    encoding="utf-8",
                )
                env = os.environ.copy()
                env["QUAD_SIM_FORCE_DIRECT"] = "1"
                env["QUAD_SIM_CMD_FILE"] = str(command_file)
                proc = subprocess.Popen(
                    [sys.executable, "-m", "simulation.quad_sim"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    env=env,
                )
                try:
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline:
                        exit_code = proc.poll()
                        if exit_code is not None:
                            stdout, stderr = proc.communicate(timeout=1.0)
                            self.fail(
                                f"quad_sim exited while executing {action} "
                                f"(code={exit_code})\nSTDOUT:\n{stdout}\nSTDERR:\n{stderr}"
                            )
                        time.sleep(0.1)
                finally:
                    if proc.poll() is None:
                        proc.terminate()
                        try:
                            proc.wait(timeout=5.0)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                            proc.wait(timeout=5.0)
                    proc.communicate(timeout=1.0)


if __name__ == "__main__":
    unittest.main()
