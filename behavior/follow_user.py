import os

from interfaces.robot_interface import RobotInterface


class FollowUser:
    def run(self, controller: RobotInterface) -> None:
        vx = float(os.getenv("FOLLOW_VX", "0.26"))
        controller.walk(vx, 0.0, 0.0)
