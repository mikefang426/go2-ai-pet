import os

from interfaces.robot_interface import RobotInterface


class Patrol:
    def run(self, controller: RobotInterface) -> None:
        vx = float(os.getenv("PATROL_VX", "0.22"))
        yaw = float(os.getenv("PATROL_YAW", "0.32"))
        controller.walk(vx, 0.0, yaw)
