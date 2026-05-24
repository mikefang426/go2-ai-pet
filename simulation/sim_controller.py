from __future__ import annotations

import atexit
import importlib
import json
import os
import subprocess
import tempfile
import threading
from dataclasses import dataclass, field
from pathlib import Path
import sys
from typing import Any

try:
    from interfaces.robot_interface import RobotInterface
    from robot.movement import MotionCommand, clamp_velocity
except ModuleNotFoundError:
    project_root = Path(__file__).resolve().parents[1]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from interfaces.robot_interface import RobotInterface
    from robot.movement import MotionCommand, clamp_velocity

SUPPORTED_ENVIRONMENTS = (
    "empty_room",
    "living_room",
    "hallway",
    "stairs",
    "obstacle_course",
)


def _module_path(environment: str) -> str:
    if __package__:
        return f"{__package__}.environments.{environment}"
    return f"environments.{environment}"


def load_environment(environment: str, p: Any, pybullet_data: Any) -> list[int]:
    name = (environment or "empty_room").strip().lower()
    if not name:
        name = "empty_room"

    if name not in SUPPORTED_ENVIRONMENTS:
        valid = ", ".join(SUPPORTED_ENVIRONMENTS)
        raise ValueError(f"Unknown environment '{name}'. Valid options: {valid}")

    module = importlib.import_module(_module_path(name))
    loader = getattr(module, "load", None)
    if not callable(loader):
        raise TypeError(f"Environment module '{name}' does not define a callable load(p, pybullet_data)")

    body_ids = loader(p, pybullet_data)
    return body_ids or []


@dataclass
class SimulationControllerConfig:
    max_v: float = 0.5
    max_wz: float = 1.0
    auto_stand_on_walk: bool = True
    launch_simulator: bool = True
    show_simulator_logs: bool = False
    command_file: str | None = None


@dataclass
class SimulationController(RobotInterface):
    """Simulation-side controller that can drive quad_sim through a command file."""

    config: SimulationControllerConfig = field(default_factory=SimulationControllerConfig)
    is_standing: bool = True
    latest_command: MotionCommand = field(default_factory=MotionCommand)
    command_file: Path = field(init=False)
    _action_seq: int = field(default=0, init=False)
    _command_lock: threading.RLock = field(default_factory=threading.RLock, init=False, repr=False)
    _sim_process: subprocess.Popen[str] | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.config.command_file:
            self.command_file = Path(self.config.command_file).expanduser().resolve()
        else:
            self.command_file = Path(tempfile.gettempdir()) / "go2_ai_pet_sim_command.json"
        self._write_command(MotionCommand())
        if self.config.launch_simulator:
            self._ensure_simulator_running()
        atexit.register(self._cleanup)

    def _ensure_simulator_running(self) -> None:
        if self._sim_process is not None and self._sim_process.poll() is None:
            return
        project_root = Path(__file__).resolve().parents[1]
        env = os.environ.copy()
        env["QUAD_SIM_CMD_FILE"] = str(self.command_file)
        stdout_target = None if self.config.show_simulator_logs else subprocess.DEVNULL
        stderr_target = None if self.config.show_simulator_logs else subprocess.DEVNULL
        self._sim_process = subprocess.Popen(
            [sys.executable, "-m", "simulation.quad_sim"],
            cwd=project_root,
            env=env,
            stdout=stdout_target,
            stderr=stderr_target,
            text=True,
        )
        print(f"[sim_controller] launched simulator with command file: {self.command_file}")

    def _write_command(
        self,
        cmd: MotionCommand,
        posture: str | None = None,
        action: str | None = None,
        action_id: int | None = None,
    ) -> None:
        with self._command_lock:
            pose = posture if posture is not None else ("stand" if self.is_standing else "sit")
            payload = {"vx": cmd.vx, "vy": cmd.vy, "wz": cmd.wz, "posture": pose}
            if action is not None:
                payload["action"] = action
                payload["action_id"] = int(action_id) if action_id is not None else 0
            tmp_path = self.command_file.with_suffix(".tmp")
            self.command_file.parent.mkdir(parents=True, exist_ok=True)
            with tmp_path.open("w", encoding="utf-8") as fh:
                json.dump(payload, fh)
            tmp_path.replace(self.command_file)

    def _cleanup(self) -> None:
        try:
            self._write_command(MotionCommand())
        except Exception:
            pass

    def sit(self) -> None:
        with self._command_lock:
            self.is_standing = False
            self.latest_command = MotionCommand()
            self._write_command(self.latest_command, posture="sit")

    def stand(self) -> None:
        with self._command_lock:
            self.is_standing = True
            self.latest_command = MotionCommand()
            self._write_command(self.latest_command, posture="stand")

    def walk(self, vx: float, vy: float, yaw: float) -> None:
        if self.config.launch_simulator:
            self._ensure_simulator_running()
        with self._command_lock:
            if not self.is_standing and self.config.auto_stand_on_walk:
                self.stand()
            self.latest_command = clamp_velocity(
                MotionCommand(vx=vx, vy=vy, wz=yaw),
                max_v=self.config.max_v,
                max_wz=self.config.max_wz,
            )
            self._write_command(self.latest_command, posture="stand")

    def stop(self) -> None:
        with self._command_lock:
            self.latest_command = MotionCommand()
            self._write_command(self.latest_command)

    def move(self, cmd: MotionCommand) -> None:
        self.walk(cmd.vx, cmd.vy, cmd.wz)

    def greet(self) -> None:
        if self.config.launch_simulator:
            self._ensure_simulator_running()
        with self._command_lock:
            self.is_standing = True
            self.latest_command = MotionCommand()
            self._action_seq += 1
            self._write_command(self.latest_command, posture="stand", action="greet", action_id=self._action_seq)

    def beg(self) -> None:
        if self.config.launch_simulator:
            self._ensure_simulator_running()
        with self._command_lock:
            self.is_standing = True
            self.latest_command = MotionCommand()
            self._action_seq += 1
            self._write_command(self.latest_command, posture="stand", action="beg", action_id=self._action_seq)

    def shake_hand(self) -> None:
        if self.config.launch_simulator:
            self._ensure_simulator_running()
        with self._command_lock:
            self.is_standing = True
            self.latest_command = MotionCommand()
            self._action_seq += 1
            self._write_command(self.latest_command, posture="stand", action="shake_hand", action_id=self._action_seq)


SimRobot = SimulationController
