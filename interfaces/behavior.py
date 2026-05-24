from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Protocol

from interfaces.robot_interface import RobotInterface


@dataclass
class BehaviorContext:
    """Cancellation signal shared by the orchestrator and a running behavior."""

    stop_event: threading.Event = field(default_factory=threading.Event)

    @property
    def interrupted(self) -> bool:
        return self.stop_event.is_set()

    def wait(self, timeout: float) -> bool:
        return self.stop_event.wait(max(0.0, timeout))


class Behavior(Protocol):
    def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
        """Execute behavior commands through RobotInterface."""
