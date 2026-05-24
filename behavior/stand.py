from interfaces.behavior import BehaviorContext
from interfaces.robot_interface import RobotInterface


class Stand:
    def run(self, controller: RobotInterface, context: BehaviorContext | None = None) -> None:
        if context and context.interrupted:
            controller.stop()
            return
        controller.stand()
