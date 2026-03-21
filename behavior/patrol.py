import os

from interfaces.robot_interface import RobotInterface


class Patrol:
    def run(self, controller: RobotInterface) -> None:
        # Make patrol turning more obvious by biasing yaw higher than forward speed.
        vx = float(os.getenv("PATROL_VX", "0.15"))
        yaw = float(os.getenv("PATROL_YAW", "0.65"))
        controller.walk(vx, 0.0, yaw)
