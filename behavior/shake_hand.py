import os
import time

from interfaces.behavior import BehaviorContext
from interfaces.robot_interface import RobotInterface


class ShakeHand:
    @staticmethod
    def _hold_motion(
        controller: RobotInterface,
        vx: float,
        vy: float,
        yaw: float,
        duration: float,
        step: float,
        context: BehaviorContext | None = None,
    ) -> None:
        step = max(0.02, step)
        t = 0.0
        while t < max(0.0, duration) and not (context and context.interrupted):
            controller.walk(vx, vy, yaw)
            if context and context.wait(step):
                break
            if context is None:
                time.sleep(step)
            t += step

    def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
        shake_fn = getattr(controller, "shake_hand", None)
        if callable(shake_fn):
            shake_fn()
            return

        controller.stand()
        stabilize_sec = float(os.getenv("SHAKE_HAND_STABILIZE_SEC", "0.18"))
        if context and context.wait(stabilize_sec):
            controller.stop()
            return
        if context is None:
            time.sleep(stabilize_sec)

        step = float(os.getenv("SHAKE_HAND_STEP_SEC", "0.05"))
        cycles = max(1, int(os.getenv("SHAKE_HAND_CYCLES", "3")))
        prepare = [
            (0.03, 0.08, 0.10, 0.28),
            (0.00, 0.00, 0.00, 0.10),
        ]
        shake_pair = [
            (0.11, 0.04, 0.08, 0.16),
            (-0.09, 0.04, -0.04, 0.16),
        ]
        finish = [
            (0.00, -0.04, -0.08, 0.18),
            (0.00, 0.00, 0.00, 0.20),
        ]

        for vx, vy, yaw, duration in prepare:
            self._hold_motion(controller, vx, vy, yaw, duration, step, context)

        for _ in range(cycles):
            for vx, vy, yaw, duration in shake_pair:
                self._hold_motion(controller, vx, vy, yaw, duration, step, context)

        for vx, vy, yaw, duration in finish:
            self._hold_motion(controller, vx, vy, yaw, duration, step, context)

        controller.stop()
