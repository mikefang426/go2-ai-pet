from robot.go2_controller import Go2Controller
from robot.movement import MotionCommand


class FollowUser:
    def run(self, controller: Go2Controller) -> None:
        controller.move(MotionCommand(vx=0.2, vy=0.0, wz=0.0))
