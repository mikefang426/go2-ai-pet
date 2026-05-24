from __future__ import annotations

import threading

from interfaces.behavior import Behavior, BehaviorContext
from interfaces.robot_interface import RobotInterface


class BehaviorRunner:
    """Centralized owner for behavior execution and interruption."""

    def __init__(self, controller: RobotInterface, stop_timeout: float = 1.0) -> None:
        self.controller = controller
        self.stop_timeout = stop_timeout
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._context: BehaviorContext | None = None
        self._active_name: str | None = None

    @property
    def active_name(self) -> str | None:
        with self._lock:
            thread = self._thread
            if thread is None:
                return None
            if not thread.is_alive():
                self._active_name = None
                self._context = None
                self._thread = None
                return None
            return self._active_name

    @property
    def is_running(self) -> bool:
        return self.active_name is not None

    def start(self, name: str, behavior: Behavior) -> bool:
        if not self.stop():
            return False

        context = BehaviorContext()
        thread = threading.Thread(
            target=self._run_behavior,
            args=(name, behavior, context),
            name=f"BehaviorRunner:{name}",
            daemon=True,
        )
        with self._lock:
            self._active_name = name
            self._context = context
            self._thread = thread
        thread.start()
        return True

    def stop(self) -> bool:
        with self._lock:
            thread = self._thread
            context = self._context

        if thread is None:
            return True

        if context is not None:
            context.stop_event.set()

        if thread is not threading.current_thread():
            thread.join(timeout=self.stop_timeout)

        stopped = not thread.is_alive()
        self.controller.stop()
        if stopped:
            self._clear_if_current(thread)
        return stopped

    def _run_behavior(self, name: str, behavior: Behavior, context: BehaviorContext) -> None:
        try:
            behavior.run(self.controller, context)
        finally:
            with self._lock:
                if self._active_name == name and self._context is context:
                    self._active_name = None
                    self._context = None
                    self._thread = None

    def _clear_if_current(self, thread: threading.Thread) -> None:
        with self._lock:
            if self._thread is thread:
                self._active_name = None
                self._context = None
                self._thread = None
