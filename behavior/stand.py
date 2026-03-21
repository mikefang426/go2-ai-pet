from interfaces.robot_interface import RobotInterface


class Stand:
    def run(self, controller: RobotInterface) -> None:
        controller.stand()
