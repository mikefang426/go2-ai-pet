from interfaces.robot_interface import RobotInterface


class Sit:
    def run(self, controller: RobotInterface) -> None:
        controller.sit()
