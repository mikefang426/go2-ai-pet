import math
import os
import time

from interfaces.robot_interface import RobotInterface


class GreetPerson:
    def run(self, controller: RobotInterface) -> None:
        controller.stand()
        # Time-based yaw wag works better with command smoothing than short pulses.
        duration = float(os.getenv("GREET_DURATION", "2.2"))
        amplitude = float(os.getenv("GREET_YAW_AMPLITUDE", "0.95"))
        frequency_hz = float(os.getenv("GREET_YAW_FREQ_HZ", "0.9"))
        dt = 0.08
        t = 0.0
        while t < duration:
            ramp = min(1.0, t / 0.35, max(0.0, duration - t) / 0.35)
            yaw = amplitude * ramp * math.sin(2.0 * math.pi * frequency_hz * t)
            controller.walk(0.0, 0.0, yaw)
            time.sleep(dt)
            t += dt
        controller.stop()
