from __future__ import annotations

from abc import ABC, abstractmethod


class RobotInterface(ABC):
    """Unified control surface for real and simulated robots."""

    @abstractmethod
    def sit(self) -> None:
        """Transition the robot into a sitting pose."""

    @abstractmethod
    def stand(self) -> None:
        """Transition the robot into a standing pose."""

    @abstractmethod
    def walk(self, vx: float, vy: float, yaw: float) -> None:
        """Command planar velocity in the robot frame."""

    def stop(self) -> None:
        self.walk(0.0, 0.0, 0.0)
