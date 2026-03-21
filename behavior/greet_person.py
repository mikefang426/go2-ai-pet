import os
import time

from interfaces.robot_interface import RobotInterface


class GreetPerson:
    @staticmethod
    def _hold_motion(
        controller: RobotInterface,
        vx: float,
        vy: float,
        yaw: float,
        duration: float,
        step: float,
    ) -> None:
        step = max(0.02, step)
        t = 0.0
        while t < max(0.0, duration):
            controller.walk(vx, vy, yaw)
            time.sleep(step)
            t += step

    @staticmethod
    def _play_steps(
        controller: RobotInterface,
        steps: list[tuple[float, float, float, float]],
        style: float,
        step: float,
        max_v: float,
        max_wz: float,
    ) -> None:
        for vx, vy, yaw, duration in steps:
            svx = max(-max_v, min(max_v, vx * style))
            svy = max(-max_v, min(max_v, vy * style))
            syaw = max(-max_wz, min(max_wz, yaw * style))
            GreetPerson._hold_motion(controller, svx, svy, syaw, duration, step)

    def run(self, controller: RobotInterface) -> None:
        controller.stand()
        time.sleep(float(os.getenv("GREET_STABILIZE_SEC", "0.18")))
        preset = os.getenv("GREET_PRESET", "super").strip().lower()
        if preset in {"super", "max", "intense"}:
            style_default = "2.15"
            dt_default = "0.05"
            max_v_default = "0.50"
            max_wz_default = "1.00"
            sway_cycles_default = "5"
            wag_cycles_default = "4"
            look_steps = [
                (0.08, 0.24, 1.00, 0.90),
                (0.03, 0.00, 0.00, 0.26),
                (0.08, -0.24, -1.00, 0.90),
                (0.03, 0.00, 0.00, 0.26),
            ]
            nod_steps = [
                (0.30, 0.00, 0.00, 0.82),
                (-0.22, 0.00, 0.00, 0.66),
                (0.25, 0.00, 0.00, 0.66),
                (-0.12, 0.00, 0.00, 0.52),
                (0.00, 0.00, 0.00, 0.30),
            ]
            sway_pair = [
                (0.13, 0.27, 1.00, 0.70),
                (0.04, 0.00, 0.00, 0.16),
                (0.13, -0.27, -1.00, 0.70),
                (0.04, 0.00, 0.00, 0.16),
            ]
            wag_pair = [
                (0.18, 0.18, 1.00, 0.72),
                (0.18, -0.18, -1.00, 0.72),
            ]
            print("[greet] preset=super sequence: look-left/right -> big nod -> body sway -> happy wag")
        else:
            style_default = "1.55"
            dt_default = "0.06"
            max_v_default = "0.42"
            max_wz_default = "1.00"
            sway_cycles_default = "3"
            wag_cycles_default = "2"
            look_steps = [
                (0.06, 0.18, 0.95, 0.70),
                (0.02, 0.00, 0.00, 0.28),
                (0.06, -0.18, -0.95, 0.70),
                (0.02, 0.00, 0.00, 0.28),
            ]
            nod_steps = [
                (0.26, 0.00, 0.00, 0.68),
                (-0.18, 0.00, 0.00, 0.58),
                (0.20, 0.00, 0.00, 0.58),
                (0.00, 0.00, 0.00, 0.28),
            ]
            sway_pair = [
                (0.11, 0.24, 0.98, 0.62),
                (0.03, 0.00, 0.00, 0.18),
                (0.11, -0.24, -0.98, 0.62),
                (0.03, 0.00, 0.00, 0.18),
            ]
            wag_pair = [
                (0.15, 0.16, 1.00, 0.60),
                (0.15, -0.16, -1.00, 0.60),
            ]
            print("[greet] preset=expressive sequence: look-left/right -> nod -> sway -> wag")

        style = float(os.getenv("GREET_STYLE_SCALE", style_default))
        dt = float(os.getenv("GREET_STEP_SEC", dt_default))
        max_v = float(os.getenv("GREET_MAX_V", max_v_default))
        max_wz = float(os.getenv("GREET_MAX_WZ", max_wz_default))
        sway_cycles = int(os.getenv("GREET_SWAY_CYCLES", sway_cycles_default))
        wag_cycles = int(os.getenv("GREET_WAG_CYCLES", wag_cycles_default))

        self._play_steps(controller, look_steps, style, dt, max_v, max_wz)
        self._play_steps(controller, nod_steps, style, dt, max_v, max_wz)

        sway_steps = []
        for _ in range(max(1, sway_cycles)):
            sway_steps.extend(sway_pair)
        self._play_steps(controller, sway_steps, style, dt, max_v, max_wz)

        wag_steps = []
        for _ in range(max(1, wag_cycles)):
            wag_steps.extend(wag_pair)
        wag_steps.append((0.00, 0.00, 0.00, 0.44))
        self._play_steps(controller, wag_steps, style, dt, max_v, max_wz)

        controller.stop()
